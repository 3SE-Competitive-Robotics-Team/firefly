//! 重力对齐的四自由度位姿图：VIO 相对边、地图锚点和在线回环边。
//! 对照 VINS-Fusion optimize4DoF；状态为地图位置与航向，俯仰/滚转取自 VIO。
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Matrix4, UnitQuaternion, Vector3, Vector4};
use std::collections::BTreeMap;

/// 图优化数值参数；长度为米，角度为弧度。
#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GraphOptions {
    pub max_nodes: usize,
    pub iterations: usize,
    pub odom_position_sigma: f64,
    pub odom_yaw_sigma: f64,
    pub map_position_sigma: f64,
    pub map_yaw_sigma: f64,
    pub loop_position_sigma: f64,
    pub loop_yaw_sigma: f64,
    pub huber: f64,
}
impl Default for GraphOptions {
    fn default() -> Self {
        Self {
            max_nodes: 512,
            iterations: 10,
            odom_position_sigma: 0.1,
            odom_yaw_sigma: 0.05,
            map_position_sigma: 0.1,
            map_yaw_sigma: 0.05,
            loop_position_sigma: 0.05,
            loop_yaw_sigma: 0.03,
            huber: 2.5,
        }
    }
}
impl GraphOptions {
    /// # Errors
    /// 节点/迭代上限或噪声参数不合法。
    pub fn validate(&self) -> Result<()> {
        if self.max_nodes < 2
            || self.max_nodes > 4096
            || self.iterations == 0
            || self.iterations > 50
            || [
                self.odom_position_sigma,
                self.odom_yaw_sigma,
                self.map_position_sigma,
                self.map_yaw_sigma,
                self.loop_position_sigma,
                self.loop_yaw_sigma,
                self.huber,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.)
        {
            return Err(invalid("invalid pose graph configuration"));
        }
        Ok(())
    }
}
#[derive(Clone)]
struct Node {
    raw: RigidTransform,
    value: Vector4<f64>,
    tilt: UnitQuaternion<f64>,
    anchor: Option<Vector4<f64>>,
}
#[derive(Clone)]
struct Edge {
    i: usize,
    j: usize,
    measurement: Vector4<f64>,
    sigma: Vector4<f64>,
    robust: bool,
}
/// 参数角归一到 [-π,π]。
fn wrap(x: f64) -> f64 {
    x.sin().atan2(x.cos())
}
fn coordinates(pose: &RigidTransform) -> Vector4<f64> {
    let p = pose.isometry();
    let (_, _, yaw) = p.rotation.euler_angles();
    Vector4::new(p.translation.x, p.translation.y, p.translation.z, yaw)
}
fn rotation(n: &Node) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), n.value.w) * n.tilt
}
fn relative(a: &Node, b: &Node) -> Vector4<f64> {
    let t = rotation(a).inverse() * (b.value.xyz() - a.value.xyz());
    Vector4::new(t.x, t.y, t.z, wrap(b.value.w - a.value.w))
}
/// 残差和解析雅可比；位移表达在前节点机体系。
fn linearize(a: &Node, b: &Node, z: &Vector4<f64>) -> (Vector4<f64>, Matrix4<f64>, Matrix4<f64>) {
    let mut r = relative(a, b) - z;
    r.w = wrap(r.w);
    let rt = rotation(a).inverse().to_rotation_matrix().into_inner();
    let mut ja = Matrix4::zeros();
    let mut jb = Matrix4::zeros();
    ja.fixed_view_mut::<3, 3>(0, 0).copy_from(&-rt);
    jb.fixed_view_mut::<3, 3>(0, 0).copy_from(&rt);
    let dyaw = -rt * Vector3::z().cross(&(b.value.xyz() - a.value.xyz()));
    ja.fixed_view_mut::<3, 1>(0, 3).copy_from(&dyaw);
    ja[(3, 3)] = -1.;
    jb[(3, 3)] = 1.;
    (r, ja, jb)
}
/// 一个进程会话中的有界图；首节点固定，图满时拒绝扩容。
pub struct PoseGraph {
    options: GraphOptions,
    initial: RigidTransform,
    nodes: BTreeMap<u64, Node>,
    loops: BTreeMap<(u64, u64), Vector4<f64>>,
    drift: RigidTransform,
}
impl PoseGraph {
    /// # Errors
    /// 参数非正、无效尺寸或变换不是 map←odom。
    pub fn new(options: GraphOptions, initial: RigidTransform) -> Result<Self> {
        options.validate()?;
        if initial.target() != FrameId::MAP
            || initial.source() != FrameId::ODOM
            || (initial.isometry().rotation * Vector3::z() - Vector3::z()).norm() > 1e-6
        {
            return Err(invalid("invalid pose graph configuration"));
        }
        Ok(Self {
            options,
            initial,
            nodes: BTreeMap::new(),
            loops: BTreeMap::new(),
            drift: initial,
        })
    }
    /// 原始 VIO 节点；同一时间戳只能对应同一测量，不能覆盖历史。
    /// # Errors
    /// 时间非法、图容量耗尽、坐标错误或重复时间戳内容冲突。
    pub fn insert(&mut self, t: f64, raw: RigidTransform) -> Result<()> {
        if !t.is_finite()
            || t.is_sign_negative()
            || raw.target() != FrameId::ODOM
            || raw.source() != FrameId::BODY
        {
            return Err(invalid("invalid graph odometry"));
        }
        let key = t.to_bits();
        if let Some(old) = self.nodes.get(&key) {
            if (old.raw.matrix() - raw.matrix()).norm() > 1e-6 {
                return Err(invalid("conflicting keyframe odometry"));
            }
            return Ok(());
        }
        if self.nodes.len() >= self.options.max_nodes {
            return Err(invalid("pose graph capacity reached"));
        }
        if self
            .nodes
            .first_key_value()
            .is_some_and(|(&first, _)| key < first)
        {
            return Err(invalid("keyframe predates fixed graph origin"));
        }
        let map = self.drift.compose(&raw)?;
        let value = coordinates(&map);
        let tilt =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -value.w) * map.isometry().rotation;
        self.nodes.insert(
            key,
            Node {
                raw,
                value,
                tilt,
                anchor: None,
            },
        );
        Ok(())
    }
    /// 独立地图观测只插入一次；同一观测不经第二滤波器重复融合。
    /// # Errors
    /// 节点缺失、重复锚点、非法地图位姿或几何新息超限。
    pub fn anchor(&mut self, t: f64, map: RigidTransform) -> Result<()> {
        if map.target() != FrameId::MAP || map.source() != FrameId::BODY {
            return Err(invalid("invalid map observation frames"));
        }
        let node = self
            .nodes
            .get_mut(&t.to_bits())
            .ok_or_else(|| invalid("missing map keyframe"))?;
        let value = coordinates(&map);
        if node.anchor.is_some()
            || (node.value.xyz() - value.xyz()).norm() > 3.
            || wrap(node.value.w - value.w).abs() > 30_f64.to_radians()
        {
            return Err(invalid(
                "map observation duplicate or outside geometric gate",
            ));
        }
        node.anchor = Some(value);
        if let Err(e) = self.optimize() {
            if let Some(node) = self.nodes.get_mut(&t.to_bits()) {
                node.anchor = None;
            }
            return Err(e);
        }
        Ok(())
    }
    /// 回环相对位姿 `T_old_body_new_body`；前端须经过几何和连续一致性验证。
    /// # Errors
    /// 时间次序、端点或重复边不满足契约；优化不收敛。
    pub fn close_loop(
        &mut self,
        from: f64,
        to: f64,
        translation: Vector3<f64>,
        yaw: f64,
    ) -> Result<()> {
        if !from.is_finite()
            || !to.is_finite()
            || from >= to
            || !translation.iter().all(|v| v.is_finite())
            || !yaw.is_finite()
        {
            return Err(invalid("invalid loop edge"));
        }
        let key = (from.to_bits(), to.to_bits());
        if !self.nodes.contains_key(&key.0)
            || !self.nodes.contains_key(&key.1)
            || self.loops.contains_key(&key)
        {
            return Err(invalid("missing or duplicate loop endpoint"));
        }
        self.loops.insert(
            key,
            Vector4::new(translation.x, translation.y, translation.z, wrap(yaw)),
        );
        if let Err(e) = self.optimize() {
            self.loops.remove(&key);
            return Err(e);
        }
        Ok(())
    }
    /// 验证绝对视觉观测，并将匹配时刻的原始里程计加入图。
    /// # Errors
    /// 质量、坐标、时间或图优化不满足契约。
    pub fn observe_map(
        &mut self,
        obs: &firefly_pubsub::vision::PoseObservation,
        raw: &Matrix4<f64>,
    ) -> Result<()> {
        if !obs.converged
            || obs.source != firefly_pubsub::vision::OBS_SOURCE_VISUAL
            || obs.num_inliers < 30
            || obs.num_inliers > obs.total_points
            || f64::from(obs.num_inliers) < 0.3 * f64::from(obs.total_points)
            || !obs.error.is_finite()
            || !(0. ..=3.).contains(&obs.error)
        {
            return Err(invalid("map observation quality gate"));
        }
        let map = RigidTransform::from_parts(
            FrameId::MAP,
            FrameId::BODY,
            [obs.position_x, obs.position_y, obs.position_z],
            [obs.quat_x, obs.quat_y, obs.quat_z, obs.quat_w],
        )?;
        self.insert(
            obs.timestamp,
            RigidTransform::from_matrix(FrameId::ODOM, FrameId::BODY, raw)?,
        )?;
        self.anchor(obs.timestamp, map)
    }
    /// 检查回环端点与重力一致性；右端姿态的航向差必须在世界水平面求取。
    /// # Errors
    /// 质量、时间、四元数、重力或几何新息门控失败。
    pub fn observe_loop(&mut self, edge: &firefly_pubsub::vision::LoopConstraint) -> Result<()> {
        if !edge.from.is_initialized
            || !edge.to.is_initialized
            || edge.from.timestamp < 0.
            || edge.to.timestamp - edge.from.timestamp < 15.
            || edge.inliers < 30
            || edge.confirmations < 3
            || !edge.reprojection_error.is_finite()
            || !(0. ..=3.).contains(&edge.reprojection_error)
        {
            return Err(invalid("loop observation quality gate"));
        }
        let a = edge.from.body_pose(FrameId::ODOM)?;
        let b = edge.to.body_pose(FrameId::ODOM)?;
        let relative = RigidTransform::from_parts(
            FrameId(1024),
            FrameId::BODY,
            edge.translation,
            edge.quaternion,
        )?;
        let old = RigidTransform::from_matrix(FrameId::ODOM, FrameId(1024), &a.matrix())?;
        let predicted = old.compose(&relative)?;
        let r = predicted.isometry().rotation;
        let yaw = r.euler_angles().2;
        let predicted_yaw = b.isometry().rotation.euler_angles().2;
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), predicted_yaw - yaw) * r;
        if tilt.angle_to(&b.isometry().rotation) > 5_f64.to_radians()
            || (predicted.isometry().translation.vector - b.isometry().translation.vector).norm()
                > 3.
            || wrap(yaw - predicted_yaw).abs() > 30_f64.to_radians()
        {
            return Err(invalid("loop gravity or innovation gate"));
        }
        self.insert(edge.from.timestamp, a)?;
        self.insert(edge.to.timestamp, b)?;
        self.close_loop(
            edge.from.timestamp,
            edge.to.timestamp,
            Vector3::from(edge.translation),
            wrap(yaw - a.isometry().rotation.euler_angles().2),
        )
    }
    #[must_use]
    pub fn alignment(&self) -> RigidTransform {
        self.drift
    }
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }
    #[must_use]
    pub fn loop_count(&self) -> usize {
        self.loops.len()
    }
    #[must_use]
    pub fn last_time(&self) -> Option<f64> {
        self.nodes.last_key_value().map(|(&k, _)| f64::from_bits(k))
    }
    fn edges(&self, keys: &[u64]) -> Vec<Edge> {
        let raw: Vec<_> = keys
            .iter()
            .map(|k| {
                let n = &self.nodes[k];
                let p = self.initial.compose(&n.raw).expect("validated frames");
                let value = coordinates(&p);
                let tilt = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -value.w)
                    * p.isometry().rotation;
                Node {
                    value,
                    tilt,
                    ..n.clone()
                }
            })
            .collect();
        let sig = |p, y| Vector4::new(p, p, p, y);
        let mut edges = Vec::new();
        for j in 1..keys.len() {
            for i in j.saturating_sub(4)..j {
                edges.push(Edge {
                    i,
                    j,
                    measurement: relative(&raw[i], &raw[j]),
                    sigma: sig(
                        self.options.odom_position_sigma,
                        self.options.odom_yaw_sigma,
                    ),
                    robust: false,
                });
            }
        }
        for (&(a, b), &measurement) in &self.loops {
            if let (Ok(i), Ok(j)) = (keys.binary_search(&a), keys.binary_search(&b)) {
                edges.push(Edge {
                    i,
                    j,
                    measurement,
                    sigma: sig(
                        self.options.loop_position_sigma,
                        self.options.loop_yaw_sigma,
                    ),
                    robust: true,
                });
            }
        }
        edges
    }
    /// 矩阵无关 PCG 解稀疏正规方程；回溯保证鲁棒目标下降，失败不发布部分解。
    #[fastrace::trace]
    fn optimize(&mut self) -> Result<()> {
        let keys: Vec<_> = self.nodes.keys().copied().collect();
        let mut nodes: Vec<_> = self.nodes.values().cloned().collect();
        let edges = self.edges(&keys);
        for _ in 0..self.options.iterations {
            let (blocks, gradient) = normal(&nodes, &edges, &self.options);
            let Some(delta) = pcg(&blocks, &gradient) else {
                return Err(invalid("pose graph linear solve failed"));
            };
            if delta.iter().map(Vector4::norm_squared).sum::<f64>().sqrt() < 1e-7 {
                break;
            }
            let cost = objective(&nodes, &edges, &self.options);
            let mut accepted = false;
            for step in 0..12 {
                let mut candidate = nodes.clone();
                let scale = 0.5_f64.powi(step);
                for i in 1..nodes.len() {
                    candidate[i].value -= scale * delta[i];
                    candidate[i].value.w = wrap(candidate[i].value.w);
                }
                if objective(&candidate, &edges, &self.options) < cost {
                    nodes = candidate;
                    accepted = true;
                    break;
                }
            }
            if !accepted {
                return Err(invalid("pose graph backtracking failed"));
            }
        }
        let last = nodes.last().ok_or_else(|| invalid("empty graph"))?;
        let yaw = last.value.w - coordinates(&last.raw).w;
        let r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
        let t = last.value.xyz() - r * last.raw.isometry().translation.vector;
        let q = r.quaternion();
        let drift = RigidTransform::from_parts(
            FrameId::MAP,
            FrameId::ODOM,
            t.into(),
            [q.i, q.j, q.k, q.w],
        )?;
        for (key, node) in keys.into_iter().zip(nodes) {
            self.nodes.insert(key, node);
        }
        self.drift = drift;
        Ok(())
    }
}
fn invalid(s: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, s)
}
fn robust(r: f64, h: f64) -> (f64, f64) {
    if r <= h {
        (0.5 * r * r, 1.)
    } else {
        (h * (r - 0.5 * h), h / r)
    }
}
fn anchor_residual(n: &Node, options: &GraphOptions) -> Option<(Vector4<f64>, Matrix4<f64>)> {
    n.anchor.map(|a| {
        let mut r = n.value - a;
        r.w = wrap(r.w);
        let scale = Vector4::new(
            1. / options.map_position_sigma,
            1. / options.map_position_sigma,
            1. / options.map_position_sigma,
            1. / options.map_yaw_sigma,
        );
        (r.component_mul(&scale), Matrix4::from_diagonal(&scale))
    })
}
fn objective(nodes: &[Node], edges: &[Edge], o: &GraphOptions) -> f64 {
    edges
        .iter()
        .map(|e| {
            let (mut r, _, _) = linearize(&nodes[e.i], &nodes[e.j], &e.measurement);
            r.component_div_assign(&e.sigma);
            if e.robust {
                robust(r.norm(), o.huber).0
            } else {
                0.5 * r.norm_squared()
            }
        })
        .sum::<f64>()
        + nodes
            .iter()
            .filter_map(|n| anchor_residual(n, o))
            .map(|(r, _)| robust(r.norm(), o.huber).0)
            .sum::<f64>()
}
struct Blocks {
    diagonal: Vec<Matrix4<f64>>,
    cross: Vec<(usize, usize, Matrix4<f64>)>,
}
fn normal(nodes: &[Node], edges: &[Edge], o: &GraphOptions) -> (Blocks, Vec<Vector4<f64>>) {
    let mut blocks = Blocks {
        diagonal: vec![Matrix4::identity() * 1e-6; nodes.len()],
        cross: Vec::new(),
    };
    let mut g = vec![Vector4::zeros(); nodes.len()];
    for e in edges {
        let (r, ja, jb) = linearize(&nodes[e.i], &nodes[e.j], &e.measurement);
        let w = Matrix4::from_diagonal(&e.sigma.map(|s| 1. / s));
        let r = w * r;
        let ja = w * ja;
        let jb = w * jb;
        let weight = if e.robust {
            robust(r.norm(), o.huber).1
        } else {
            1.
        };
        if e.i > 0 {
            blocks.diagonal[e.i] += weight * ja.transpose() * ja;
            g[e.i] += weight * ja.transpose() * r;
        }
        if e.j > 0 {
            blocks.diagonal[e.j] += weight * jb.transpose() * jb;
            g[e.j] += weight * jb.transpose() * r;
        }
        if e.i > 0 && e.j > 0 {
            blocks.cross.push((e.i, e.j, weight * ja.transpose() * jb));
        }
    }
    for (i, n) in nodes.iter().enumerate().skip(1) {
        if let Some((r, j)) = anchor_residual(n, o) {
            let w = robust(r.norm(), o.huber).1;
            blocks.diagonal[i] += w * j.transpose() * j;
            g[i] += w * j.transpose() * r;
        }
    }
    blocks.diagonal[0] = Matrix4::identity();
    (blocks, g)
}
fn apply(a: &Blocks, x: &[Vector4<f64>]) -> Vec<Vector4<f64>> {
    let mut out: Vec<_> = a.diagonal.iter().zip(x).map(|(a, x)| a * x).collect();
    for &(i, j, b) in &a.cross {
        out[i] += b * x[j];
        out[j] += b.transpose() * x[i];
    }
    out
}
fn dot(a: &[Vector4<f64>], b: &[Vector4<f64>]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a.dot(b)).sum()
}
fn pcg(a: &Blocks, b: &[Vector4<f64>]) -> Option<Vec<Vector4<f64>>> {
    let inverse: Vec<_> = a
        .diagonal
        .iter()
        .map(|d| d.try_inverse())
        .collect::<Option<_>>()?;
    let mut x = vec![Vector4::zeros(); b.len()];
    let mut r = b.to_vec();
    let mut z: Vec<_> = inverse.iter().zip(&r).map(|(a, b)| a * b).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let tolerance = 1e-14 * dot(b, b).max(1.);
    for _ in 0..(b.len() * 8).min(1000) {
        if dot(&r, &r) <= tolerance {
            return Some(x);
        }
        let ap = apply(a, &p);
        let den = dot(&p, &ap);
        if !den.is_finite() || den <= 0. {
            return None;
        }
        let alpha = rz / den;
        for i in 0..x.len() {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
            z[i] = inverse[i] * r[i];
        }
        let next = dot(&r, &z);
        let beta = next / rz;
        for i in 0..p.len() {
            p[i] = z[i] + beta * p[i];
        }
        rz = next;
    }
    (dot(&r, &r) <= 1e-8 * dot(b, b).max(1.)).then_some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn pose(x: f64, y: f64, yaw: f64) -> RigidTransform {
        let q = UnitQuaternion::from_euler_angles(0.03, -0.04, yaw);
        let q = q.quaternion();
        RigidTransform::from_parts(
            FrameId::ODOM,
            FrameId::BODY,
            [x, y, 0.],
            [q.i, q.j, q.k, q.w],
        )
        .unwrap()
    }
    fn origin() -> RigidTransform {
        RigidTransform::from_parts(FrameId::MAP, FrameId::ODOM, [0.; 3], [0., 0., 0., 1.]).unwrap()
    }
    #[test]
    fn residual_analytic_value_and_jacobians() {
        let mut g = PoseGraph::new(GraphOptions::default(), origin()).unwrap();
        g.insert(0., pose(0., 0., 0.)).unwrap();
        g.insert(1., pose(2., 1., 0.3)).unwrap();
        let a = g.nodes[&0f64.to_bits()].clone();
        let b = g.nodes[&1f64.to_bits()].clone();
        let z = Vector4::zeros();
        let (r, ja, jb) = linearize(&a, &b, &z);
        let expected = a.raw.isometry().rotation.inverse() * Vector3::new(2., 1., 0.);
        assert!((r.xyz() - expected).norm() < 1e-12);
        assert!((r.w - 0.3).abs() < 1e-12);
        for i in 0..4 {
            for side in 0..2 {
                let (mut ap, mut am, mut bp, mut bm) = (a.clone(), a.clone(), b.clone(), b.clone());
                if side == 0 {
                    ap.value[i] += 1e-6;
                    am.value[i] -= 1e-6;
                } else {
                    bp.value[i] += 1e-6;
                    bm.value[i] -= 1e-6;
                }
                let fd = (linearize(&ap, &bp, &z).0 - linearize(&am, &bm, &z).0) / 2e-6;
                assert!(
                    (fd - if side == 0 {
                        ja.column(i)
                    } else {
                        jb.column(i)
                    })
                    .norm()
                        < 1e-6
                );
            }
        }
    }
    #[test]
    fn revisiting_origin_reduces_drift_without_changing_raw_odometry() {
        let mut g = PoseGraph::new(GraphOptions::default(), origin()).unwrap();
        for i in 0..=20 {
            let t = f64::from(i);
            let angle = t * std::f64::consts::TAU / 20.;
            g.insert(t, pose(angle.sin() + 0.03 * t, 1. - angle.cos(), 0.))
                .unwrap();
        }
        let raw = g.nodes[&20f64.to_bits()].raw;
        g.close_loop(0., 20., Vector3::zeros(), 0.).unwrap();
        let corrected = g.alignment().compose(&raw).unwrap();
        assert!(corrected.isometry().translation.vector.norm() < 0.2);
        assert!((g.nodes[&20f64.to_bits()].raw.matrix() - raw.matrix()).norm() < 1e-15);
        assert!(g.nodes[&0f64.to_bits()].value.norm() < 1e-12);
        assert!(g.close_loop(0., 20., Vector3::zeros(), 0.).is_err());
    }
    #[test]
    fn maps_and_loops_share_one_objective_and_capacity_is_explicit() {
        let opts = GraphOptions {
            max_nodes: 3,
            ..Default::default()
        };
        let mut g = PoseGraph::new(opts, origin()).unwrap();
        for i in 0..3 {
            g.insert(f64::from(i), pose(f64::from(i), 0., 0.)).unwrap();
        }
        assert!(g.insert(3., pose(3., 0., 0.)).is_err());
        let map = origin().compose(&pose(1.8, 0., 0.)).unwrap();
        g.anchor(2., map).unwrap();
        let p = g.alignment().compose(&pose(2., 0., 0.)).unwrap();
        assert!(p.isometry().translation.x < 2. && p.isometry().translation.x > 1.8);
        assert!(g.anchor(2., map).is_err());
        assert!(g.insert(f64::NAN, pose(0., 0., 0.)).is_err());
    }
    #[test]
    fn robust_objective_gradient_matches_independent_differences() {
        let mut graph = PoseGraph::new(GraphOptions::default(), origin()).unwrap();
        for i in 0..5 {
            graph
                .insert(f64::from(i), pose(f64::from(i) * 0.2, 0.1, 0.3))
                .unwrap();
        }
        graph.loops.insert(
            (0_f64.to_bits(), 4_f64.to_bits()),
            Vector4::new(0.6, 0., 0., 0.1),
        );
        let keys: Vec<_> = graph.nodes.keys().copied().collect();
        let edges = graph.edges(&keys);
        let mut nodes: Vec<_> = graph.nodes.values().cloned().collect();
        nodes[2].value.x += 0.03;
        nodes[3].value.w += 0.01;
        nodes[3].anchor = Some(Vector4::new(0.55, 0.12, 0., 0.32));
        let (_, gradient) = normal(&nodes, &edges, &graph.options);
        for i in 1..nodes.len() {
            for (axis, expected) in gradient[i].iter().enumerate() {
                let mut positive = nodes.clone();
                let mut negative = nodes.clone();
                positive[i].value[axis] += 1e-6;
                negative[i].value[axis] -= 1e-6;
                let fd = (objective(&positive, &edges, &graph.options)
                    - objective(&negative, &edges, &graph.options))
                    / 2e-6;
                assert!((fd - expected).abs() < 1e-5);
            }
        }
    }
    #[test]
    fn yaw_loop_preserves_gravity_and_rejects_bad_observations() {
        use firefly_pubsub::{odom::OdomMessage, vision::LoopConstraint};
        let mut graph = PoseGraph::new(GraphOptions::default(), origin()).unwrap();
        for i in 0..=20 {
            let t = f64::from(i);
            graph.insert(t, pose(t * 0.02, 0., t * 0.006)).unwrap();
        }
        graph.close_loop(0., 20., Vector3::zeros(), 0.).unwrap();
        let raw = pose(0.4, 0., 0.12);
        let corrected = graph.alignment().compose(&raw).unwrap();
        assert!(corrected.isometry().rotation.euler_angles().2.abs() < 0.05);
        let (roll, pitch, _) = corrected.isometry().rotation.euler_angles();
        assert!((roll - 0.03).abs() < 1e-12 && (pitch + 0.04).abs() < 1e-12);
        let mut edge = LoopConstraint {
            from: OdomMessage {
                timestamp: 0.,
                is_initialized: true,
                ..Default::default()
            },
            to: OdomMessage {
                timestamp: 30.,
                is_initialized: true,
                ..Default::default()
            },
            translation: [0.; 3],
            quaternion: [0., 0., 0., 1.],
            inliers: 40,
            confirmations: 3,
            reprojection_error: 0.2,
        };
        edge.quaternion = [0.3, 0., 0., (1. - 0.09_f64).sqrt()];
        let before = graph.alignment();
        assert!(graph.observe_loop(&edge).is_err());
        edge.quaternion = [0., 0., 0., 1.];
        edge.confirmations = 1;
        assert!(graph.observe_loop(&edge).is_err());
        edge.confirmations = 3;
        edge.reprojection_error = f64::NAN;
        assert!(graph.observe_loop(&edge).is_err());
        assert!((before.matrix() - graph.alignment().matrix()).norm() < 1e-12);
    }
}
