//! 会话内 RGB-D 关键帧回环：外观候选、PnP、深度一致性与连续三帧确认。
//! VINS-Fusion 提供四自由度约束思路；ORB-SLAM3 提供近邻排除与连续验证思路。
use firefly_base::{FrameId, RigidTransform};
use firefly_pubsub::{
    odom::OdomMessage,
    vision::{DESC_DIM, FeatureMessage, LoopConstraint, MAX_FEATURES},
};
use firefly_vision_map::VisionMapPoint;
use firefly_vision_match::{
    CameraIntrinsics,
    calibration::{body_from_left_camera, pinhole},
    depth::RegisteredDepth,
    solve_visual_pose,
};
use nalgebra::{Matrix4, Vector3};
use ort::session::Session;

const OLD_BODY: FrameId = FrameId(1024);
/// 长度米、角度弧度、时间传感器秒；阈值在运行前固定。
#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    pub enabled: bool,
    pub max_keyframes: usize,
    pub candidates: usize,
    pub min_age: f64,
    pub min_travel: f64,
    pub keyframe_period: f64,
    pub depth_error: f64,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            enabled: true,
            max_keyframes: 512,
            candidates: 3,
            min_age: 15.,
            min_travel: 2.,
            keyframe_period: 1.,
            depth_error: 0.1,
        }
    }
}
impl Options {
    pub fn validate(&self) -> Result<(), String> {
        if !(2..=4096).contains(&self.max_keyframes)
            || !(1..=8).contains(&self.candidates)
            || !self.min_age.is_finite()
            || self.min_age < 15.
            || !self.min_travel.is_finite()
            || self.min_travel < 2.
            || !self.keyframe_period.is_finite()
            || self.keyframe_period < 0.5
            || !self.depth_error.is_finite()
            || !(0.01..=0.2).contains(&self.depth_error)
        {
            return Err("invalid online loop options".into());
        }
        Ok(())
    }
}
struct Keyframe {
    odom: OdomMessage,
    points: Vec<VisionMapPoint>,
    embedding: [f64; DESC_DIM],
    travel: f64,
}
struct Verified {
    transform: RigidTransform,
    inliers: u32,
    error: f64,
}
/// 连续确认比较的是 odom 侧修正，不能比较随飞机运动变化的相对位姿。
#[derive(Default)]
struct Confirmation {
    last: Option<(f64, Matrix4<f64>, u32)>,
}
impl Confirmation {
    fn observe(&mut self, t: f64, correction: Matrix4<f64>) -> u32 {
        let count = self
            .last
            .as_ref()
            .filter(|(time, old, _)| t > *time && t - time <= 3. && consistent(old, &correction))
            .map_or(1, |(_, _, count)| count + 1);
        self.last = Some((t, correction, count));
        count
    }
    fn miss(&mut self) {
        self.last = None;
    }
}
fn consistent(a: &Matrix4<f64>, b: &Matrix4<f64>) -> bool {
    let ta = a.fixed_view::<3, 1>(0, 3);
    let tb = b.fixed_view::<3, 1>(0, 3);
    let ra = nalgebra::UnitQuaternion::from_matrix(&a.fixed_view::<3, 3>(0, 0).into_owned());
    let rb = nalgebra::UnitQuaternion::from_matrix(&b.fixed_view::<3, 3>(0, 0).into_owned());
    (ta - tb).norm() < 0.3 && ra.angle_to(&rb) < 5_f64.to_radians()
}
fn embedding<'a>(descriptors: impl Iterator<Item = &'a [f32; DESC_DIM]>) -> [f64; DESC_DIM] {
    let mut out = [0.; DESC_DIM];
    for descriptor in descriptors {
        for (i, &v) in descriptor.iter().enumerate() {
            out[i] += f64::from(v);
        }
    }
    let norm = out.iter().map(|v| v * v).sum::<f64>().sqrt();
    if norm > 1e-12 {
        for v in &mut out {
            *v /= norm;
        }
    }
    out
}
fn point_in_body(depth: &RegisteredDepth, uv: [f32; 2]) -> Option<Vector3<f64>> {
    let p = depth.point_at(uv.map(f64::from))?;
    Some(body_from_left_camera().point(p.into()).coords)
}
/// 2D-3D 重投影与独立当前深度同时验证，避免单平面二义性和局部误匹配。
fn verify(
    feat: &FeatureMessage,
    depth: &RegisteredDepth,
    pairs: &[(usize, [f64; 3], f32)],
    prior: Matrix4<f64>,
    depth_error: f64,
) -> Result<Verified, String> {
    if pairs.len() < 30 {
        return Err("insufficient geometric support".into());
    }
    let uv: Vec<_> = pairs.iter().map(|(i, _, _)| feat.keypoints[*i]).collect();
    let xyz: Vec<_> = pairs.iter().map(|(_, p, _)| *p).collect();
    let k = pinhole();
    let intrinsics = CameraIntrinsics {
        focal: k.focal,
        cx: k.cx,
        cy: k.cy,
    };
    let paired: Vec<_> = pairs
        .iter()
        .filter_map(|&(i, p, _)| Some((point_in_body(depth, feat.keypoints[i])?, Vector3::from(p))))
        .collect();
    let source: Vec<_> = paired.iter().map(|(p, _)| *p).collect();
    let target: Vec<_> = paired.iter().map(|(_, p)| *p).collect();
    let seed = firefly_vision_match::rgbd::rigid_consensus(
        &source,
        &target,
        OLD_BODY,
        FrameId::BODY,
        depth_error,
    )
    .ok_or("RGB-D has no metric consensus")?;
    let raw =
        RigidTransform::from_matrix(OLD_BODY, FrameId::BODY, &prior).map_err(|e| e.to_string())?;
    if (seed.isometry().translation.vector - raw.isometry().translation.vector).norm() > 3.
        || seed.isometry().rotation.angle_to(&raw.isometry().rotation) > 15_f64.to_radians()
    {
        return Err("loop outside odometry innovation gate".into());
    }
    let camera_prior = seed
        .compose(&body_from_left_camera())
        .map_err(|e| e.to_string())?
        .matrix();
    let pose = solve_visual_pose(&uv, &xyz, intrinsics, Some(camera_prior))
        .map_err(|e| e.to_string())?
        .ok_or("PnP has no valid solution")?;
    let camera = RigidTransform::from_matrix(OLD_BODY, FrameId::LEFT_CAMERA, &pose.t_global)
        .map_err(|e| e.to_string())?;
    let transform = camera
        .compose(&body_from_left_camera().inverse())
        .map_err(|e| e.to_string())?;
    let mut count = 0u32;
    let mut error = 0.;
    let mut cells = [false; 12];
    for &(i, point, _) in pairs {
        let Some(body) = point_in_body(depth, feat.keypoints[i]) else {
            continue;
        };
        if (transform.point(body.into()).coords - Vector3::from(point)).norm() > depth_error {
            continue;
        }
        let p = camera.inverse().point(point.into()).coords;
        let Some(pixel) = k.pixel(p) else {
            continue;
        };
        let observed = feat.keypoints[i];
        let distance = ((pixel[0] - f64::from(observed[0])).powi(2)
            + (pixel[1] - f64::from(observed[1])).powi(2))
        .sqrt();
        if distance > 3. {
            continue;
        }
        let x = (observed[0] / 80.).floor() as usize;
        let y = (observed[1] / 80.).floor() as usize;
        if x >= 4 || y >= 3 {
            continue;
        }
        cells[y * 4 + x] = true;
        count += 1;
        error += distance;
    }
    if count < 30
        || f64::from(count) < 0.4 * pairs.len() as f64
        || cells.iter().filter(|&&v| v).count() < 4
    {
        return Err(format!(
            "geometric support: {count}/{} pairs, {} cells",
            pairs.len(),
            cells.iter().filter(|&&v| v).count()
        ));
    }
    Ok(Verified {
        transform,
        inliers: count,
        error: error / f64::from(count),
    })
}
/// 内存有界且不复用原始 VIO 坐标系会话；容量耗尽显式拒绝增加关键帧。
pub struct Online {
    options: Options,
    frames: Vec<Keyframe>,
    confirmation: Confirmation,
    previous: Option<OdomMessage>,
    travel: f64,
    last_query: f64,
    last_emitted: f64,
    pub diagnostics: [f64; 4],
    last_rejection: Option<String>,
    /// 推理缓冲与 io-binding（首次 `process` 时按会话创建后复用）。
    bindings: Option<super::MatchBindings>,
}
impl Online {
    pub fn new(options: Options) -> Self {
        Self {
            options,
            frames: Vec::new(),
            confirmation: Confirmation::default(),
            previous: None,
            travel: 0.,
            last_query: f64::NEG_INFINITY,
            last_emitted: f64::NEG_INFINITY,
            diagnostics: [0.; 4],
            last_rejection: None,
            bindings: None,
        }
    }
    #[fastrace::trace]
    pub fn process(
        &mut self,
        session: &mut Session,
        feat: &FeatureMessage,
        odom: OdomMessage,
        depth: &RegisteredDepth,
        cancelled: impl Fn() -> bool,
    ) -> Result<Option<LoopConstraint>, Box<dyn std::error::Error>> {
        if !self.options.enabled
            || !feat.timestamp.is_finite()
            || feat.timestamp - self.last_query < self.options.keyframe_period
            || (odom.timestamp - feat.timestamp).abs() > 1e-6
            || !odom.is_initialized
        {
            return Ok(None);
        }
        self.last_query = feat.timestamp;
        let raw = odom.body_pose(FrameId::ODOM)?;
        if let Some(previous) = self.previous {
            self.travel += (raw.isometry().translation.vector
                - previous
                    .body_pose(FrameId::ODOM)?
                    .isometry()
                    .translation
                    .vector)
                .norm();
        }
        self.previous = Some(odom);
        let n = (feat.count as usize).min(MAX_FEATURES);
        let summary = embedding(feat.descriptors[..n].iter());
        let mut candidates: Vec<_> = self
            .frames
            .iter()
            .enumerate()
            .filter(|(_, f)| {
                odom.timestamp - f.odom.timestamp >= self.options.min_age
                    && self.travel - f.travel >= self.options.min_travel
            })
            .map(|(i, f)| {
                (
                    i,
                    summary
                        .iter()
                        .zip(f.embedding)
                        .map(|(a, b)| a * b)
                        .sum::<f64>(),
                )
            })
            .collect();
        candidates.sort_by(|a, b| b.1.total_cmp(&a.1));
        candidates.truncate(self.options.candidates);
        if self.bindings.is_none() {
            self.bindings = Some(super::MatchBindings::new(session)?);
        }
        let bindings = self.bindings.as_mut().ok_or("推理绑定未建立")?;
        super::bind_query(bindings, feat)?;
        let mut accepted: Option<(usize, Verified, Matrix4<f64>)> = None;
        let mut ambiguous = false;
        for &(index, _) in &candidates {
            if cancelled() {
                return Ok(None);
            }
            let old = &self.frames[index];
            let old_raw = old.odom.body_pose(FrameId::ODOM)?;
            // 不同时刻机体系具有不同标识；PnP 内部矩阵表达 old_body←current_body。
            let old_pose = RigidTransform::from_matrix(FrameId::ODOM, OLD_BODY, &old_raw.matrix())?;
            let prior = old_pose.inverse().compose(&raw)?.matrix();
            let pairs = super::match_points(session, bindings, &old.points, feat)?;
            let found = match verify(feat, depth, &pairs, prior, self.options.depth_error) {
                Ok(found) => found,
                Err(reason) => {
                    log::debug!("回环候选 {index} 拒收: {reason}");
                    self.last_rejection = Some(reason);
                    continue;
                }
            };
            {
                // C 将原始当前 odom 位姿映到由历史关键帧独立确定的当前位姿。
                let corrected = old_pose.compose(&found.transform)?;
                let correction = corrected.matrix() * raw.isometry().inverse().to_homogeneous();
                if let Some((_, previous, previous_correction)) = &accepted {
                    if !consistent(previous_correction, &correction) {
                        ambiguous = true;
                        break;
                    }
                    if previous.inliers >= found.inliers {
                        continue;
                    }
                }
                accepted = Some((index, found, correction));
            }
        }
        let mut output = None;
        let mut confirmations = 0;
        let mut inliers = 0;
        if !ambiguous && let Some((index, found, correction)) = accepted {
            confirmations = self.confirmation.observe(odom.timestamp, correction);
            inliers = found.inliers;
            if confirmations >= 3 && odom.timestamp - self.last_emitted >= 5. {
                let q = found.transform.isometry().rotation.quaternion();
                output = Some(LoopConstraint {
                    from: self.frames[index].odom,
                    to: odom,
                    translation: found.transform.isometry().translation.vector.into(),
                    quaternion: [q.i, q.j, q.k, q.w],
                    inliers: found.inliers,
                    confirmations,
                    reprojection_error: found.error,
                });
                self.last_emitted = odom.timestamp;
                self.confirmation.miss();
            }
        } else {
            self.confirmation.miss();
        }
        if self.frames.len() < self.options.max_keyframes {
            let points: Vec<_> = (0..n)
                .filter_map(|i| {
                    Some(VisionMapPoint {
                        position: point_in_body(depth, feat.keypoints[i])?.into(),
                        uv: feat.keypoints[i],
                        descriptor: feat.descriptors[i],
                        score: feat.scores[i],
                    })
                })
                .collect();
            if points.len() >= 30 {
                let summary = embedding(points.iter().map(|p| &p.descriptor));
                self.frames.push(Keyframe {
                    odom,
                    points,
                    embedding: summary,
                    travel: self.travel,
                });
            }
        } else {
            log::warn!("在线关键帧容量耗尽：{}", self.options.max_keyframes);
        }
        self.diagnostics = [
            self.frames.len() as f64,
            candidates.len() as f64,
            f64::from(inliers),
            f64::from(confirmations),
        ];
        Ok(output)
    }
    pub fn miss(&mut self) {
        self.confirmation.miss();
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn confirmations_are_consecutive_and_motion_compensated() {
        let mut c = Confirmation::default();
        let m = Matrix4::identity();
        assert_eq!(c.observe(1., m), 1);
        assert_eq!(c.observe(2., m), 2);
        assert_eq!(c.observe(3., m), 3);
        c.miss();
        assert_eq!(c.observe(4., m), 1);
        let mut wrong = m;
        wrong[(0, 3)] = 1.;
        assert_eq!(c.observe(5., wrong), 1);
        assert_eq!(c.observe(9., wrong), 1);
        assert_eq!(c.observe(9., wrong), 1);
        assert!(
            Options {
                min_age: 0.,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
    }
    #[test]
    #[allow(clippy::large_stack_arrays)]
    fn geometry_requires_spread_depth_and_correct_correspondence() {
        use firefly_vision_match::{calibration::left_from_depth, depth::register_depth};
        let k = pinhole();
        let source: Vec<_> = (0..320 * 240)
            .map(|i| 2. + (i % 320) as f32 * 0.002 + (i / 320) as f32 * 0.001)
            .collect();
        let depth = register_depth(&source, k, k, &left_from_depth()).unwrap();
        let mut feat = FeatureMessage {
            timestamp: 1.,
            count: 48,
            keypoints: [[0.; 2]; MAX_FEATURES],
            descriptors: [[0.; DESC_DIM]; MAX_FEATURES],
            scores: [1.; MAX_FEATURES],
        };
        let mut pairs = Vec::new();
        for y in 0..6 {
            for x in 0..8 {
                let i = y * 8 + x;
                let uv = [30. + x as f32 * 35., 25. + y as f32 * 35.];
                feat.keypoints[i] = uv;
                pairs.push((i, point_in_body(&depth, uv).unwrap().into(), 1.));
            }
        }
        let mut prior = Matrix4::identity();
        prior[(0, 3)] = 0.1;
        let result = verify(&feat, &depth, &pairs, prior, 0.1).unwrap();
        assert_eq!(result.inliers, 48);
        assert!(result.transform.isometry().translation.vector.norm() < 1e-4);
        let wrong_depth = register_depth(&vec![5.; 320 * 240], k, k, &left_from_depth()).unwrap();
        assert!(verify(&feat, &wrong_depth, &pairs, prior, 0.1).is_err());
        for item in pairs.iter_mut().step_by(2) {
            item.1[1] += 2.;
        }
        assert!(verify(&feat, &depth, &pairs, prior, 0.1).is_err());
    }
    /// 重访同一真实采集图验证模型、在线存储、三次确认、回环 IPC 载荷与图校正。
    /// 图像重复仅验证集成契约；跨视角飞行性能由任务验收独立判定。
    #[test]
    #[ignore = "requires verified RMUC captures, visual map and ONNX weights"]
    #[allow(clippy::large_stack_arrays)]
    #[allow(clippy::too_many_lines)]
    fn real_model_revisit_contract() {
        use firefly_localization::graph::{GraphOptions, PoseGraph};
        use firefly_vision_match::{calibration::left_from_depth, depth::register_depth};
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let map =
            firefly_vision_map::load_map(&root.join("apps/planner/maps/rmuc2026.ffvmap")).unwrap();
        let frame = map
            .frames
            .iter()
            .find(|f| {
                let mut cells = [false; 12];
                for p in &f.points {
                    let x = (p.uv[0] / 80.) as usize;
                    let y = (p.uv[1] / 80.) as usize;
                    if x < 4 && y < 3 {
                        cells[y * 4 + x] = true;
                    }
                }
                f.points.len() > 150 && cells.iter().filter(|&&v| v).count() >= 6
            })
            .unwrap();
        let captures = std::env::var("FIREFLY_LOOP_CAPTURES")
            .expect("set capture directory from map manifest");
        let data = std::fs::read_dir(captures)
            .unwrap()
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().is_some_and(|v| v == "bin"))
            .map(|e| std::fs::read(e.path()).unwrap())
            .find(|bytes| {
                (f64::from_le_bytes(bytes[16 + 320 * 240 * 5 + 56..][..8].try_into().unwrap())
                    - frame.timestamp)
                    .abs()
                    < 1e-9
            })
            .unwrap();
        let depths: Vec<_> = data[16 + 320 * 240..16 + 320 * 240 * 5]
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect();
        let depth = register_depth(&depths, pinhole(), pinhole(), &left_from_depth()).unwrap();
        let mut feat = FeatureMessage {
            timestamp: 1.,
            count: frame.points.len() as u32,
            keypoints: [[0.; 2]; MAX_FEATURES],
            descriptors: [[0.; DESC_DIM]; MAX_FEATURES],
            scores: [1.; MAX_FEATURES],
        };
        for (i, p) in frame.points.iter().enumerate() {
            feat.keypoints[i] = p.uv;
            feat.descriptors[i] = p.descriptor;
        }
        let mut session =
            super::super::load_session(root.join(super::super::DEFAULT_MODEL).to_str().unwrap())
                .unwrap();
        let mut online = Online::new(Options::default());
        let mut odom = OdomMessage {
            timestamp: 1.,
            is_initialized: true,
            ..Default::default()
        };
        let initial =
            RigidTransform::from_parts(FrameId::MAP, FrameId::ODOM, [0.; 3], [0., 0., 0., 1.])
                .unwrap();
        let mut graph = PoseGraph::new(GraphOptions::default(), initial).unwrap();
        graph
            .insert(1., odom.body_pose(FrameId::ODOM).unwrap())
            .unwrap();
        assert!(
            online
                .process(&mut session, &feat, odom, &depth, || false)
                .unwrap()
                .is_none()
        );
        let count = feat.count;
        feat.count = 0;
        feat.timestamp = 2.;
        odom.timestamp = 2.;
        odom.position_x = 2.;
        assert!(
            online
                .process(&mut session, &feat, odom, &depth, || false)
                .unwrap()
                .is_none()
        );
        graph
            .insert(2., odom.body_pose(FrameId::ODOM).unwrap())
            .unwrap();
        feat.count = count;
        odom.position_x = 0.4;
        for t in [20., 21.] {
            feat.timestamp = t;
            odom.timestamp = t;
            assert!(
                online
                    .process(&mut session, &feat, odom, &depth, || false)
                    .unwrap()
                    .is_none()
            );
        }
        feat.timestamp = 22.;
        odom.timestamp = 22.;
        let edge = online
            .process(&mut session, &feat, odom, &depth, || false)
            .unwrap()
            .unwrap_or_else(|| {
                panic!(
                    "three confirmed revisits must emit a loop: {:?}",
                    (online.diagnostics, &online.last_rejection)
                )
            });
        assert_eq!(edge.confirmations, 3);
        assert!(edge.inliers >= 30);
        graph.observe_loop(&edge).unwrap();
        let corrected = graph
            .alignment()
            .compose(&odom.body_pose(FrameId::ODOM).unwrap())
            .unwrap();
        assert!(corrected.isometry().translation.vector.norm() < 0.2);
        assert!((odom.position_x - 0.4).abs() < 1e-12);
        let before = graph.alignment();
        let mut bad = edge;
        bad.to.timestamp = 23.;
        bad.inliers = 2;
        assert!(graph.observe_loop(&bad).is_err());
        assert!((before.matrix() - graph.alignment().matrix()).norm() < 1e-12);
    }
    #[test]
    #[ignore = "requires verified RMUC captures, visual map and ONNX weights"]
    #[allow(clippy::large_stack_arrays)]
    #[allow(clippy::float_cmp)]
    fn real_model_insufficient_overlap_rejection_contract() {
        use firefly_vision_match::{calibration::left_from_depth, depth::register_depth};
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let map =
            firefly_vision_map::load_map(&root.join("apps/planner/maps/rmuc2026.ffvmap")).unwrap();
        let select = |x| {
            map.frames
                .iter()
                .find(|f| f.position == [x, -1., 1.2] && f.quat_xyzw == [0., 0., 0., 1.])
                .unwrap()
        };
        let old = select(-13.);
        let current = select(-12.);
        let captures = std::env::var("FIREFLY_LOOP_CAPTURES")
            .expect("set capture directory from map manifest");
        let data = std::fs::read_dir(captures)
            .unwrap()
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().is_some_and(|v| v == "bin"))
            .map(|e| std::fs::read(e.path()).unwrap())
            .find(|bytes| {
                (f64::from_le_bytes(bytes[16 + 320 * 240 * 5 + 56..][..8].try_into().unwrap())
                    - current.timestamp)
                    .abs()
                    < 1e-9
            })
            .unwrap();
        let depths: Vec<_> = data[16 + 320 * 240..16 + 320 * 240 * 5]
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect();
        let depth = register_depth(&depths, pinhole(), pinhole(), &left_from_depth()).unwrap();
        let mut feat = FeatureMessage {
            timestamp: 20.,
            count: current.points.len() as u32,
            keypoints: [[0.; 2]; MAX_FEATURES],
            descriptors: [[0.; DESC_DIM]; MAX_FEATURES],
            scores: [1.; MAX_FEATURES],
        };
        for (i, p) in current.points.iter().enumerate() {
            feat.keypoints[i] = p.uv;
            feat.descriptors[i] = p.descriptor;
        }
        let points: Vec<_> = old
            .points
            .iter()
            .map(|p| VisionMapPoint {
                position: [p.position[0] + 13., p.position[1] + 1., p.position[2] - 1.2],
                ..p.clone()
            })
            .collect();
        let mut session =
            super::super::load_session(root.join(super::super::DEFAULT_MODEL).to_str().unwrap())
                .unwrap();
        let mut bindings = super::super::MatchBindings::new(&session).unwrap();
        super::super::bind_query(&mut bindings, &feat).unwrap();
        let pairs =
            super::super::match_points(&mut session, &mut bindings, &points, &feat).unwrap();
        let mut prior = Matrix4::identity();
        prior[(0, 3)] = 1.4;
        assert!(
            pairs.len() < 30,
            "fixture must have insufficient overlap: {}",
            pairs.len()
        );
        assert!(verify(&feat, &depth, &pairs, prior, 0.1).is_err());
    }
}
