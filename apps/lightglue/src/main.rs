//! ALIKED 特征匹配：离线地图定位与在线 RGB-D 关键帧回环。
//! `PoseObservation` 表达地图绝对位姿；`LoopConstraint` 表达历史机体到当前机体。
//! 原始 VIO 与深度由独立线程配对，地图先验仅用于离线库图查询。

mod online;
mod sensors;
#[cfg(test)]
mod validation;
mod worker;

use firefly_pubsub::event::TopicListener;
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::{CORRECTED_ODOM_TOPIC, OdomMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::{CorrectedOdomSubscriber, Subscriber};
use firefly_pubsub::vision::{
    DESC_DIM, FEATURE_TOPIC, FeatureMessage, MAX_FEATURES as NUM_POINTS, OBS_SOURCE_VISUAL,
    POSE_OBS_TOPIC, PoseObservation,
};
use firefly_vision_map::VisionMap;
use firefly_vision_match::calibration::{MUJOCO_FOCAL, body_pose_to_cam, cam_pose_to_body};
use firefly_vision_match::{CameraIntrinsics, pose_covariance, solve_visual_pose};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetAttachmentId;
use nalgebra::{Rotation3, UnitQuaternion};
use ort::session::Session;
use ort::value::Tensor;

/// 缺省权重路径（相对运行目录，通常为仓库根）。
const DEFAULT_MODEL: &str = "models/lightglue-aliked-k512.onnx";
/// 相机分辨率（`image_size` 输入，`[W, H]` int64）。
const WIDTH: i64 = 320;
const HEIGHT: i64 = 240;
/// 库图查询半径（米，VIO 先验周围）。
const QUERY_RADIUS: f64 = 3.0;
/// 单次查询候选帧数：拼多帧对应破平面退化（单帧共面时 `PnP` 有翻转二义性）。
const QUERY_TOPK: usize = 3;
/// 匹配分数阈值（图内建 0.1 过滤，此处再收紧）。
const SCORE_THRESHOLD: f32 = 0.2;
/// 候选帧航向门限（度）：先验机头方向与库图帧机头方向的夹角超过此值的帧剔除。
/// 同一位置可能采了多个朝向的关键帧，只按距离选会挑到背/侧视帧而匹配失败。
const MAX_HEADING_DEG: f64 = 60.0;
/// odom 先验新鲜度（秒），超时跳过本次查询。
const ODOM_FRESH_TIMEOUT: f64 = 1.0;

/// 解析 `--model/--map`（缺省见 [`DEFAULT_MODEL`]，地图必填）。
fn parse_args() -> Result<(String, String, String), String> {
    let mut it = std::env::args().skip(1);
    let mut model = DEFAULT_MODEL.to_owned();
    let mut map: Option<String> = None;
    let mut config = "configs/lightglue.toml".to_owned();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--model" => {
                model = it
                    .next()
                    .ok_or_else(|| "missing --model value".to_owned())?;
            }
            "--config" => {
                config = it.next().ok_or("missing --config value")?;
            }
            "--map" => {
                map = Some(it.next().ok_or_else(|| "missing --map value".to_owned())?);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let map = map.ok_or_else(|| "missing --map <map.ffvmap>".to_owned())?;
    Ok((model, map, config))
}

/// 加载 ONNX 会话；六条算子线程（207ms→180ms/匹配），空闲时休眠。
fn load_session(model: &str) -> Result<Session, Box<dyn std::error::Error>> {
    if !std::path::Path::new(model).is_file() {
        return Err(format!("权重缺失：{model}（见 models/，离线导出，不进 git）").into());
    }
    let session = Session::builder()?
        .with_intra_threads(6)?
        .with_intra_op_spinning(false)?
        .commit_from_file(model)?;
    log::info!("lightglue 会话就绪：{model}");
    Ok(session)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let (model, map_path, config) = parse_args().map_err(|e| {
        eprintln!(
            "{e}\n用法：lightglue --map <map.ffvmap> [--model models/lightglue-aliked-k512.onnx]"
        );
        std::process::exit(2);
    })?;
    let map = firefly_vision_map::load_map(std::path::Path::new(&map_path)).map_err(|e| {
        eprintln!("视觉库图加载失败: {e}");
        std::process::exit(2);
    })?;
    log::info!(
        "视觉库图就绪（{} 帧，{} 点）",
        map.frames.len(),
        map.num_points()
    );
    let mut session = load_session(&model)?;
    smoke_inference(&mut session)?;
    let options: online::Options = toml::from_str(&std::fs::read_to_string(config)?)?;
    options.validate()?;
    let loop_session = Session::builder()?
        .with_intra_threads(1)?
        .commit_from_file(&model)?;
    run_loop(&mut session, &map, options, loop_session)?;
    Ok(())
}

/// 冒烟推理：零特征自匹配，校验输出形状（`[1,512]` 静态）。
fn smoke_inference(session: &mut Session) -> Result<(), Box<dyn std::error::Error>> {
    let zeros = |n: usize| {
        Tensor::from_array((
            [1usize, NUM_POINTS, n],
            vec![0f32; NUM_POINTS * n].into_boxed_slice(),
        ))
    };
    let size = Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?;
    let outputs = session.run(ort::inputs![
        "k0" => zeros(2)?,
        "d0" => zeros(128)?,
        "k1" => zeros(2)?,
        "d1" => zeros(128)?,
        "s0" => size,
        "s1" => Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?,
    ])?;
    for name in ["matches0", "matches1"] {
        let (shape, _) = outputs[name].try_extract_tensor::<i64>()?;
        assert_eq!(&shape[..], &[1, NUM_POINTS as i64], "{name} 形状");
    }
    for name in ["scores0", "scores1"] {
        let (shape, _) = outputs[name].try_extract_tensor::<f32>()?;
        assert_eq!(&shape[..], &[1, NUM_POINTS as i64], "{name} 形状");
    }
    log::info!("lightglue 冒烟推理通过（输出 [1,{NUM_POINTS}] 静态）");
    Ok(())
}

/// 主循环：特征事件唤醒 → 取最新特征 + 先验 odom → 查库匹配 + `PnP` → 发观测。
///
/// 查询先验固定为地图系校正里程计，局部里程计不能直接查询地图位置。
#[allow(clippy::too_many_lines)]
fn run_loop(
    session: &mut Session,
    map: &VisionMap,
    options: online::Options,
    loop_session: Session,
) -> Result<(), firefly_error::Error> {
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "lightglue");
    let sensors = sensors::Sensors::start()?;
    let online = worker::Worker::start(loop_session, options)?;
    let loop_pub = Publisher::<firefly_pubsub::vision::LoopConstraint>::with_topic(
        &node,
        firefly_pubsub::vision::LOOP_TOPIC,
    )?;
    let keyframe_pub =
        Publisher::<OdomMessage>::with_topic(&node, firefly_pubsub::vision::KEYFRAME_TOPIC)?;
    let viz = firefly_pubsub::viz::VizPublisher::new(&node)?;
    let feat_sub = Subscriber::<FeatureMessage>::with_topic(&node, FEATURE_TOPIC)?;
    log::info!("已订阅特征话题 {FEATURE_TOPIC}");
    let corrected_sub = CorrectedOdomSubscriber::new(&node)?;
    log::info!("已订阅地图系里程计 {CORRECTED_ODOM_TOPIC}（查询先验）");
    let obs_pub = Publisher::<PoseObservation>::with_topic(&node, POSE_OBS_TOPIC)?;
    log::info!("已打开位姿观测话题 {POSE_OBS_TOPIC}");

    let feat_events = TopicListener::with_topic(&node, FEATURE_TOPIC)?;
    let waitset = WaitSetBuilder::new()
        .create::<ipc::Service>()
        .map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建 WaitSet 失败: {e:?}"),
            )
        })?;
    let _feat_guard = waitset.attach_notification(&feat_events).map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("挂载特征事件监听失败: {e:?}"),
        )
    })?;
    let tick_guard = waitset
        .attach_interval(std::time::Duration::from_millis(500))
        .map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("挂载心跳定时器失败: {e:?}"),
            )
        })?;

    // 特征发布端带 notify（`aliked` 为 `with_topic_notify`）：特征到即醒；
    // 心跳仅兜底无事件时的 odom 缓存更新与断流自愈。
    let mut latest_corrected: Option<OdomMessage> = None;
    // 地图查询节流 1Hz：查询耗时（~200ms）超过特征周期（5Hz）必然积压，
    // 显式降频保每次查询新鲜，吞吐与新鲜度解耦（回环 worker 不受影响）。
    let mut last_map_query = f64::NEG_INFINITY;
    let on_event = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
        let root = fastrace::Span::root("lightglue", fastrace::prelude::SpanContext::random());
        let trace_guard = root.set_local_parent();
        if sensors.is_stopped() {
            return CallbackProgression::Stop;
        }
        let _ = attachment_id.has_event_from(&tick_guard);
        let _ = feat_events.drain();
        while let Ok(Some(sample)) = corrected_sub.receive() {
            latest_corrected = Some(*sample);
        }
        let mut latest_feat: Option<FeatureMessage> = None;
        while let Ok(Some(sample)) = feat_sub.receive() {
            latest_feat = Some(*sample);
        }
        if sensors.is_stopped() {
            return CallbackProgression::Stop;
        }
        if let Some(feat) = latest_feat {
            if let Some((odom, depth)) = sensors.sample(feat.timestamp) {
                online.submit(feat, odom, depth);
            }
        }
        let prior = latest_corrected.filter(|m| m.is_initialized).and_then(|m| {
            let feat = latest_feat.as_ref()?;
            if feat.timestamp - m.timestamp > ODOM_FRESH_TIMEOUT {
                return None;
            }
            sensors.map_prior(feat.timestamp, m)
        });
        if let (Some(feat), Some(odom)) = (latest_feat, prior)
            && feat.timestamp - odom.timestamp <= ODOM_FRESH_TIMEOUT
            && feat.timestamp - last_map_query >= 1.0
        {
            last_map_query = feat.timestamp;
            // 特征时间戳即 sim 时钟；查询前后各 pump 一次（匹配阻塞下积压排空）。
            // 查询耗时即管线滞后主项（ORT 推理，rrd 侧只能看到特征时间戳，
            // 到达滞后由 localization 融合/超期日志体现）。
            firefly_observability::set_sim_time(feat.timestamp);
            firefly_observability::pump_log_ipc(&log_ipc);
            let t_query = std::time::Instant::now();
            let query_outcome = query_once(session, map, &feat, &odom, || sensors.is_stopped());
            let mut timing = firefly_pubsub::viz::VizMessage::base(
                firefly_pubsub::viz::kind::SCALARS,
                feat.timestamp,
                "vision/debug/map_query_wall_s",
            );
            timing.scalars[0] = t_query.elapsed().as_secs_f64();
            timing.scalar_count = 1;
            let _ = viz.publish(timing);
            log::debug!(
                "视觉查询耗时 {:.2}s（特征 t={:.2}）",
                t_query.elapsed().as_secs_f64(),
                feat.timestamp
            );
            match query_outcome {
                Ok(Some(obs)) => {
                    if let Err(e) = obs_pub.publish(obs) {
                        log::warn!("位姿观测发布失败: {e}");
                    } else {
                        log::info!(
                            "视觉观测 t={:.2} pos=({:.2},{:.2},{:.2}) inliers {}/{} err {:.2}px",
                            obs.timestamp,
                            obs.position_x,
                            obs.position_y,
                            obs.position_z,
                            obs.num_inliers,
                            obs.total_points,
                            obs.error
                        );
                    }
                }
                Ok(None) => log::debug!("视觉查询无有效位姿（拒收）"),
                Err(e) => log::warn!("视觉查询失败: {e}"),
            }
            firefly_observability::pump_log_ipc(&log_ipc);
        }
        for result in online.results() {
            if let Some(edge) = result.edge {
                if let Err(e) = loop_pub.publish(edge) {
                    log::warn!("回环发布失败: {e}");
                }
            }
            if let Some(odom) = result.keyframe {
                if let Err(e) = keyframe_pub.publish(odom) {
                    log::warn!("关键帧发布失败: {e}");
                }
            }
            let mut msg = firefly_pubsub::viz::VizMessage::base(
                firefly_pubsub::viz::kind::SCALARS,
                result.timestamp,
                "loop/debug/frontend",
            );
            msg.scalars[..4].copy_from_slice(&result.diagnostics);
            msg.scalar_count = 4;
            let _ = viz.publish(msg);
        }
        drop(trace_guard);
        drop(root);
        fastrace::flush();
        CallbackProgression::Continue
    };
    match waitset.wait_and_process(on_event) {
        Ok(
            iceoryx2::waitset::WaitSetRunResult::Interrupt
            | iceoryx2::waitset::WaitSetRunResult::TerminationRequest,
        ) => {
            log::info!("收到终止信号，优雅退出");
            firefly_observability::pump_log_ipc(&log_ipc);
        }
        Ok(_) => {}
        Err(e) => {
            return Err(firefly_error::Error::temporary(
                firefly_error::ErrorKind::Internal,
                format!("WaitSet 事件等待失败: {e:?}"),
            ));
        }
    }
    firefly_observability::pump_log_ipc(&log_ipc);
    Ok(())
}

/// 单帧匹配：query 特征 vs 库图一帧 → 2D-3D 对应（丢弃填充/低分匹配）。
#[allow(clippy::type_complexity)]
#[fastrace::trace]
fn match_points(
    session: &mut Session,
    points: &[firefly_vision_map::VisionMapPoint],
    feat: &FeatureMessage,
    k0: &Tensor<f32>,
    d0: &Tensor<f32>,
) -> Result<Vec<(usize, [f64; 3], f32)>, Box<dyn std::error::Error>> {
    let n_map = points.len().min(NUM_POINTS);
    if n_map < 6 {
        return Ok(Vec::new());
    }
    // 库侧 0 填充（后过滤填充匹配）。
    let mut k1 = vec![0f32; NUM_POINTS * 2];
    let mut d1 = vec![0f32; NUM_POINTS * DESC_DIM];
    for (i, p) in points.iter().take(n_map).enumerate() {
        k1[2 * i] = p.uv[0];
        k1[2 * i + 1] = p.uv[1];
        d1[i * DESC_DIM..(i + 1) * DESC_DIM].copy_from_slice(&p.descriptor);
    }
    let outputs = session.run(ort::inputs![
        "k0" => k0,
        "d0" => d0,
        "k1" => Tensor::from_array(([1usize, NUM_POINTS, 2], k1.into_boxed_slice()))?,
        "d1" => Tensor::from_array(([1usize, NUM_POINTS, DESC_DIM], d1.into_boxed_slice()))?,
        "s0" => Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?,
        "s1" => Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?,
    ])?;
    let (_, matches) = outputs["matches0"].try_extract_tensor::<i64>()?;
    let (_, scores) = outputs["scores0"].try_extract_tensor::<f32>()?;
    let mut pairs = Vec::new();
    for qi in 0..(feat.count as usize).min(NUM_POINTS) {
        let mi = matches[qi];
        if mi < 0 || mi as usize >= n_map || scores[qi] < SCORE_THRESHOLD {
            continue;
        }
        pairs.push((qi, points[mi as usize].position, scores[qi]));
    }
    Ok(pairs)
}

/// 候选库图帧：先验邻域内 + 航向接近，按距离升序取 `QUERY_TOPK`。
///
/// 航向 = 机体系 `+x` 在世界系的方向；同位置多朝向的关键帧只按距离会挑到背/侧视帧。
fn candidates_by_heading(
    map: &VisionMap,
    t_body_prior: &nalgebra::Matrix4<f64>,
    prior_pos: [f64; 3],
) -> Vec<usize> {
    let prior_rot = t_body_prior.fixed_view::<3, 3>(0, 0).into_owned();
    let prior_fwd = prior_rot * nalgebra::Vector3::new(1.0, 0.0, 0.0);
    let cos_limit = MAX_HEADING_DEG.to_radians().cos();
    let mut scored: Vec<(usize, f64)> = Vec::new();
    for (i, frame) in map.frames.iter().enumerate() {
        let dx = frame.position[0] - prior_pos[0];
        let dy = frame.position[1] - prior_pos[1];
        let dz = frame.position[2] - prior_pos[2];
        let d2 = dx * dx + dy * dy + dz * dz;
        if d2 > QUERY_RADIUS * QUERY_RADIUS {
            continue;
        }
        let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            frame.quat_xyzw[3],
            frame.quat_xyzw[0],
            frame.quat_xyzw[1],
            frame.quat_xyzw[2],
        ));
        if (q * nalgebra::Vector3::new(1.0, 0.0, 0.0)).dot(&prior_fwd) < cos_limit {
            continue;
        }
        scored.push((i, d2));
    }
    scored.sort_by(|a, b| a.1.total_cmp(&b.1));
    scored.truncate(QUERY_TOPK);
    scored.into_iter().map(|(i, _)| i).collect()
}

/// 单次查询：先验邻域多帧匹配拼对应 → `PnP` → 机体位姿观测。
#[fastrace::trace]
fn query_once(
    session: &mut Session,
    map: &VisionMap,
    feat: &FeatureMessage,
    odom: &OdomMessage,
    cancelled: impl Fn() -> bool,
) -> Result<Option<PoseObservation>, Box<dyn std::error::Error>> {
    let t_body_prior = odom.body_pose(firefly_base::FrameId::MAP)?.matrix();
    let prior_pos = [0, 1, 2].map(|axis| t_body_prior[(axis, 3)]);
    let candidates = candidates_by_heading(map, &t_body_prior, prior_pos);
    log::debug!(
        "视觉查询 t={:.2} prior=({:.1},{:.1},{:.1}) candidates={candidates:?}",
        feat.timestamp,
        prior_pos[0],
        prior_pos[1],
        prior_pos[2]
    );
    if candidates.is_empty() {
        log::info!("视觉查询拒绝 t={:.2}: no_candidates", feat.timestamp);
        return Ok(None);
    }
    // 多帧拼对应：单帧共面时 PnP 有翻转二义性，多视角点集破退化。
    // query 侧输入各帧复用（一次组装、一次装 Tensor，多次推理时借用）。
    let mut k0 = vec![0f32; NUM_POINTS * 2];
    let mut d0 = vec![0f32; NUM_POINTS * DESC_DIM];
    for i in 0..NUM_POINTS {
        k0[2 * i] = feat.keypoints[i][0];
        k0[2 * i + 1] = feat.keypoints[i][1];
        d0[i * DESC_DIM..(i + 1) * DESC_DIM].copy_from_slice(&feat.descriptors[i]);
    }
    let k0t = Tensor::from_array(([1usize, NUM_POINTS, 2], k0.into_boxed_slice()))?;
    let d0t = Tensor::from_array(([1usize, NUM_POINTS, DESC_DIM], d0.into_boxed_slice()))?;
    // 同一像素只保留最高分的地图对应，避免跨库帧重复计算信息量。
    let mut best = vec![None::<([f64; 3], f32)>; NUM_POINTS];
    for (candidate_index, &frame_idx) in candidates.iter().enumerate() {
        if cancelled() {
            return Ok(None);
        }
        for (qi, point, score) in
            match_points(session, &map.frames[frame_idx].points, feat, &k0t, &d0t)?
        {
            if best[qi].is_none_or(|(_, previous)| score > previous) {
                best[qi] = Some((point, score));
            }
        }
        if candidate_index + 1 < candidates.len() && has_spatial_support(&best) {
            if let Some(obs) = observation_from_matches(feat, &best, &t_body_prior)? {
                if obs.num_inliers >= 30
                    && f64::from(obs.num_inliers) >= 0.3 * f64::from(obs.total_points)
                    && obs.error <= 3.
                {
                    return Ok(Some(obs));
                }
            }
        }
    }
    observation_from_matches(feat, &best, &t_body_prior)
}

/// 提前结束候选搜索须有非平面支撑；退化点集继续融合其他视角。
fn has_spatial_support(matches: &[Option<([f64; 3], f32)>]) -> bool {
    let points: Vec<_> = matches
        .iter()
        .flatten()
        .map(|(p, _)| nalgebra::Vector3::from(*p))
        .collect();
    if points.len() < 30 {
        return false;
    }
    let mean = points.iter().sum::<nalgebra::Vector3<f64>>() / points.len() as f64;
    let covariance = points
        .iter()
        .fold(nalgebra::Matrix3::zeros(), |sum, point| {
            let d = point - mean;
            sum + d * d.transpose()
        })
        / points.len() as f64;
    let eigen = covariance.symmetric_eigen().eigenvalues;
    eigen.min() > 0.01 * eigen.max() && eigen.min() > 0.0025
}

#[fastrace::trace]
fn observation_from_matches(
    feat: &FeatureMessage,
    best: &[Option<([f64; 3], f32)>],
    t_body_prior: &nalgebra::Matrix4<f64>,
) -> Result<Option<PoseObservation>, Box<dyn std::error::Error>> {
    let mut pairs_2d = Vec::new();
    let mut pairs_3d = Vec::new();
    for (qi, matched) in best.iter().enumerate() {
        if let Some((point, _)) = matched {
            pairs_2d.push(feat.keypoints[qi]);
            pairs_3d.push(*point);
        }
    }
    if pairs_2d.len() < 6 {
        log::info!(
            "视觉查询拒绝 t={:.2}: correspondences={}",
            feat.timestamp,
            pairs_2d.len()
        );
        return Ok(None);
    }
    let total = pairs_2d.len();
    let intrinsics = CameraIntrinsics {
        focal: MUJOCO_FOCAL,
        cx: 160.0,
        cy: 120.0,
    };
    let t_cam_prior = body_pose_to_cam(t_body_prior);
    let Some(pose) = solve_visual_pose(&pairs_2d, &pairs_3d, intrinsics, Some(t_cam_prior))
        .map_err(|e| format!("PnP: {e}"))?
    else {
        log::info!(
            "视觉查询拒绝 t={:.2}: pnp_failed correspondences={}",
            feat.timestamp,
            total
        );
        return Ok(None);
    };
    let t_body = cam_pose_to_body(&pose.t_global);
    let cov = firefly_vision_match::calibration::body_from_left_camera()
        .tangent_covariance(&pose_covariance(&pose));
    let mut covariance = [0f64; 36];
    for r in 0..6 {
        for c in 0..6 {
            covariance[r * 6 + c] = cov[(r, c)];
        }
    }
    let rot = t_body.fixed_view::<3, 3>(0, 0).into_owned();
    let quat = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix(&rot));
    let q = quat.quaternion();
    Ok(Some(PoseObservation {
        timestamp: feat.timestamp,
        source: OBS_SOURCE_VISUAL,
        position_x: t_body[(0, 3)],
        position_y: t_body[(1, 3)],
        position_z: t_body[(2, 3)],
        quat_x: q.i,
        quat_y: q.j,
        quat_z: q.k,
        quat_w: q.w,
        covariance,
        num_inliers: pose.num_inliers as u32,
        total_points: total as u32,
        error: pose.mean_reproj_px,
        converged: true,
    }))
}

#[cfg(test)]
mod tests {
    use super::DEFAULT_MODEL;

    /// 已知库帧提供独立地图位姿，匹配和 `PnP` 必须恢复它。
    #[test]
    #[ignore = "requires exported models and RMUC visual map"]
    #[allow(clippy::large_stack_arrays)]
    fn real_map_matching_and_pose_contract() {
        use super::*;
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let map =
            firefly_vision_map::load_map(&root.join("apps/planner/maps/rmuc2026.ffvmap")).unwrap();
        let mut session = load_session(&test_model_path().expect("model required")).unwrap();
        let frame = map
            .frames
            .iter()
            .find(|f| f.points.len() >= 100)
            .expect("populated map frame");
        let n = frame.points.len().min(NUM_POINTS);
        let mut features = FeatureMessage {
            timestamp: 1.,
            count: n as u32,
            keypoints: [[0.; 2]; NUM_POINTS],
            descriptors: [[0.; DESC_DIM]; NUM_POINTS],
            scores: [0.; NUM_POINTS],
        };
        for (i, p) in frame.points.iter().take(n).enumerate() {
            features.keypoints[i] = p.uv;
            features.descriptors[i] = p.descriptor;
            features.scores[i] = p.score;
        }
        let q = frame.quat_xyzw;
        let prior = OdomMessage {
            timestamp: 1.,
            is_initialized: true,
            position_x: frame.position[0] + 0.1,
            position_y: frame.position[1],
            position_z: frame.position[2],
            quat_x: q[0],
            quat_y: q[1],
            quat_z: q[2],
            quat_w: q[3],
            ..Default::default()
        };
        let observation = query_once(&mut session, &map, &features, &prior, || false)
            .unwrap()
            .expect("verified pose");
        let error = nalgebra::Vector3::new(
            observation.position_x - frame.position[0],
            observation.position_y - frame.position[1],
            observation.position_z - frame.position[2],
        );
        assert!(error.norm() < 0.05, "map pose error {error}");
        assert!(observation.num_inliers >= 30);
        assert!(observation.error < 1.);
    }

    /// 测试用权重路径：先 CWD相对，再相对 `CARGO_MANIFEST_DIR` 回仓库根；
    /// 都缺失时跳过（`models/` 已 ignore，CI 无权重）。
    fn test_model_path() -> Option<String> {
        let cwd = std::path::PathBuf::from(DEFAULT_MODEL);
        if cwd.is_file() {
            return cwd.to_str().map(str::to_owned);
        }
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join(DEFAULT_MODEL);
        if root.is_file() {
            return root.to_str().map(str::to_owned);
        }
        None
    }

    /// 端到端冒烟：权重缺失时跳过。
    #[test]
    fn onnx_smoke_or_skip() {
        let Some(path) = test_model_path() else {
            println!("skip: 无权重 {DEFAULT_MODEL}");
            return;
        };
        let mut session = super::load_session(&path).unwrap();
        super::smoke_inference(&mut session).unwrap();
    }
}

#[cfg(test)]
mod early_exit_tests {
    use super::*;
    #[test]
    fn early_exit_requires_nonplanar_spatial_support() {
        let grid: Vec<_> = (0..64)
            .map(|i| {
                Some((
                    [((i % 4) as f64), ((i / 4 % 4) as f64), ((i / 16) as f64)],
                    1.,
                ))
            })
            .collect();
        assert!(has_spatial_support(&grid));
        let flat: Vec<_> = grid
            .iter()
            .map(|p| p.map(|(v, s)| ([v[0], v[1], 0.], s)))
            .collect();
        assert!(!has_spatial_support(&flat));
        assert!(!has_spatial_support(&grid[..29]));
        let small: Vec<_> = grid
            .iter()
            .map(|p| p.map(|(v, s)| (v.map(|a| a * 0.001), s)))
            .collect();
        assert!(!has_spatial_support(&small));
    }
}
