//! 视觉定位融合：VIO 时间配对、map←odom 修正与地图里程计发布。

use std::path::PathBuf;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use firefly_localization::config::LocalizationConfig;
use firefly_localization::convert::corrected_odom;
use firefly_localization::filter::{FusionFilter, RelocGate};
use firefly_observability::init as init_observability;
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::CORRECTED_ODOM_TOPIC;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::CorrectedOdomPublisher;
use firefly_pubsub::subscriber::{OdomSubscriber, Subscriber};
use firefly_pubsub::vision::{POSE_OBS_TOPIC, PoseObservation};
use firefly_pubsub::viz::{VizMessage, VizPublisher, kind};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::{Isometry3, Matrix4, Quaternion, Translation3, UnitQuaternion, Vector3};

const LOOP_PERIOD: Duration = Duration::from_millis(10);
/// 矫正后位姿可视化节拍（10Hz 主循环每 tick 发一次位姿+轨迹段，与
/// `vio/odom`/`gt/pose` 同频率，rrd 里可逐点对比三条轨迹）。
const VIZ_PERIOD: usize = 1;
/// 矫正后位姿图例颜色（绿，与 vio 橙 / 真值蓝区分）。
const CORRECTED_COLOR: (u8, u8, u8) = (60, 200, 80);
const DEFAULT_CONFIG: &str = "configs/localization.toml";
const ODOM_FRESH_TIMEOUT: f64 = 1.0;
/// 视觉观测待融合队列上限（条）：观测先到、odom 后到时暂存，按 `timestamp`
/// 排序，odom 追上即融合；溢出时丢最旧（对端断流的背压语义，非延时等待）。
const PENDING_VISUAL_CAP: usize = 8;
/// 待融合观测超期（秒）：相对 `t_sim` 过期即丢弃（对端断流时不无限囤积；
/// 取 10s 覆盖 ORT CPU 推理长尾滞后（实测 5.7s），过期丢弃打 info（稀少，
/// 丢一条少一次修正）。过期观测经内插仍自洽（慢漂移假设），滞后误差远小于漂移本身。
const PENDING_VISUAL_TIMEOUT: f64 = 10.0;
/// odom 内插窗口（条）：100Hz 下约 10s，与超期对齐（窗口外无法内插，留了也白留；
/// 62KB，可忽略）。
const ODOM_HIST_CAP: usize = 1000;

/// 命令行参数。
struct Args {
    config: PathBuf,
    odom_topic: String,
}

fn parse_args() -> Result<Args> {
    let mut it = std::env::args().skip(1);
    let mut args = Args {
        config: PathBuf::from(DEFAULT_CONFIG),
        odom_topic: firefly_pubsub::odom::ODOM_TOPIC.to_string(),
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--odom-topic" => {
                args.odom_topic = it.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --odom-topic value")
                })?;
            }
            "--config" => {
                args.config = PathBuf::from(it.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --config value")
                })?);
            }
            other => {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    format!("unknown argument {other}"),
                ));
            }
        }
    }
    Ok(args)
}

/// 历史 odom 在目标时刻插值（位置 lerp + 姿态 slerp；时刻越界返回空，
/// 该 tick 跳过——不以外推污染配准）。
fn interp_odom(
    hist: &std::collections::VecDeque<(f64, OdomMessage)>,
    t: f64,
) -> Option<Matrix4<f64>> {
    if !t.is_finite() || hist.len() < 2 || t < hist.front()?.0 || t > hist.back()?.0 {
        return None;
    }
    let mut prev = &hist[0];
    for cur in hist.iter().skip(1) {
        if cur.0 >= t {
            let span = cur.0 - prev.0;
            if span <= 1e-9 {
                return cur
                    .1
                    .body_pose(FrameId::ODOM)
                    .ok()
                    .map(|pose| pose.matrix());
            }
            let a = ((t - prev.0) / span).clamp(0.0, 1.0);
            let (p0, p1) = (&prev.1, &cur.1);
            let pos = Vector3::new(
                p0.position_x + a * (p1.position_x - p0.position_x),
                p0.position_y + a * (p1.position_y - p0.position_y),
                p0.position_z + a * (p1.position_z - p0.position_z),
            );
            let q0 = UnitQuaternion::from_quaternion(Quaternion::new(
                p0.quat_w, p0.quat_x, p0.quat_y, p0.quat_z,
            ));
            let q1 = UnitQuaternion::from_quaternion(Quaternion::new(
                p1.quat_w, p1.quat_x, p1.quat_y, p1.quat_z,
            ));
            let q = q0.slerp(&q1, a);
            return Some(Isometry3::from_parts(Translation3::from(pos), q).to_homogeneous());
        }
        prev = cur;
    }
    None
}

/// 门控判决映射为诊断四元组 `[metric, limit, applied_trans_m, accepted]`。
///
/// `metric/limit` 为本次实际判决的门（接受与 `chi2` 拒收时为 `chi2`/阈值，
/// 新息拒收时为新息平移量/门限），`applied_trans_m` 为本次注入名义量的平移
/// 修正量（拒收为 0），`accepted` 取 1/0。预检/数值拒收无判决数值，返回空
///（调用方只走 `log`，不发标量）。
fn gate_diag(gate: &RelocGate, max_innovation_trans: f64) -> Option<[f64; 4]> {
    match gate {
        RelocGate::Accepted {
            chi2,
            threshold,
            delta,
        } => Some([*chi2, *threshold, delta.fixed_rows::<3>(3).norm(), 1.0]),
        RelocGate::RejectedChi2 { chi2, threshold } => Some([*chi2, *threshold, 0.0, 0.0]),
        RelocGate::RejectedInnovation { trans, .. } => {
            Some([*trans, max_innovation_trans, 0.0, 0.0])
        }
        RelocGate::RejectedPrecheck { .. } | RelocGate::RejectedNumerical { .. } => None,
    }
}

struct App {
    fusion: FusionFilter,
    origin: firefly_localization::config::InitialAlignment,
    alignment_ready: bool,
    estimator_reset: bool,
    reloc_ticks: usize,
    viewer_odom: Option<OdomSubscriber>,
    corrected_pub: Option<CorrectedOdomPublisher>,
    viz_pub: Option<VizPublisher>,
    corr_prev: Option<[f64; 3]>,
    latest_odom: Option<OdomMessage>,
    /// 视觉位姿观测订阅（`lightglue` 进程发布，同一 `FusionFilter` 融合）。
    visual_obs: Option<Subscriber<PoseObservation>>,
    /// odom 环形历史（时间戳，消息）：按观测时间戳插值位姿，消除
    /// 最新配对 ~0.1s 失配（1.5m/s 下 15cm 系统性错位）；`ODOM_HIST_CAP` 深
    /// 容忍低速 odom 与视觉滞后内插。
    odom_hist: std::collections::VecDeque<(f64, OdomMessage)>,
    /// 待融合视觉观测（按 `timestamp` 排序）：观测先到、odom 后到时暂存，
    /// odom 追上即融合——事件驱动的订阅关系，无延时等待。
    pending_visual: std::collections::VecDeque<PoseObservation>,
    last_odom_recv: std::time::Instant,
    last_published: f64,
    last_visual: f64,
    /// 新息门限快照（`cfg.fusion.visual.max_innovation_trans`，门控
    /// 诊断标量的 `limit` 分量与滤波器用同一值，按路径传入）。
    innov_limit_visual: f64,
    t_sim: f64,
    /// 日志聚合句柄（主循环作用域持有，每 tick 传给 `pump_log_ipc`）。
    log_ipc: firefly_observability::LogIpc,
    _node: firefly_pubsub::node::IpcNode,
}

impl App {
    #[allow(clippy::needless_pass_by_value)]
    fn new(cfg: LocalizationConfig, odom_topic: &str) -> Result<Self> {
        let innov_limit_visual = cfg.fusion.visual.max_innovation_trans;
        let fusion = FusionFilter::new(cfg.fusion);
        let node = create_node()?;
        let log_ipc = firefly_observability::init_ipc(&node, "localization");
        let odom_sub = Some(OdomSubscriber::with_topic(&node, odom_topic)?);
        let visual_obs = Some(Subscriber::with_topic(&node, POSE_OBS_TOPIC)?);
        let corrected_pub = Some(CorrectedOdomPublisher::new(&node)?);
        let viz_pub = match VizPublisher::new(&node) {
            Ok(p) => {
                log::info!(
                    "已打开话题 {}（矫正后位姿可视化）",
                    firefly_pubsub::viz::VIZ_TOPIC
                );
                Some(p)
            }
            Err(e) => {
                log::warn!("可视化发布不可用：{e}");
                None
            }
        };
        Ok(Self {
            fusion,
            origin: cfg.origin.ok_or_else(|| {
                Error::new(
                    ErrorKind::InvalidArgument,
                    "explicit fixed startup [origin] is required",
                )
            })?,
            alignment_ready: false,
            estimator_reset: false,
            reloc_ticks: 0,
            viewer_odom: odom_sub,
            visual_obs,
            corrected_pub,
            viz_pub,
            corr_prev: None,
            latest_odom: None,
            odom_hist: std::collections::VecDeque::with_capacity(ODOM_HIST_CAP),
            pending_visual: std::collections::VecDeque::with_capacity(PENDING_VISUAL_CAP),
            last_odom_recv: std::time::Instant::now(),
            last_published: f64::NEG_INFINITY,
            last_visual: f64::NEG_INFINITY,
            innov_limit_visual,
            t_sim: 0.0,
            log_ipc,
            _node: node,
        })
    }

    fn poll_sensors(&mut self) -> Result<()> {
        if let Some(sub) = &self.viewer_odom {
            let mut odom_arrived = false;
            while let Some(sample) = sub.receive()? {
                let m: OdomMessage = *sample;
                if self.alignment_ready && !m.is_initialized {
                    self.estimator_reset = true;
                    log::error!("VIO 初始化状态丢失，地图输出锁存停止；须在固定启动点重启");
                }
                if self.estimator_reset {
                    continue;
                }
                if !m.is_initialized
                    || !m.timestamp.is_finite()
                    || m.timestamp < 0.0
                    || self
                        .latest_odom
                        .as_ref()
                        .is_some_and(|last| m.timestamp <= last.timestamp)
                {
                    continue;
                }
                let Ok(pose) = m.body_pose(FrameId::ODOM) else {
                    continue;
                };
                if !self.alignment_ready {
                    let Ok(alignment) = self.origin.at_start(&m) else {
                        continue;
                    };
                    self.fusion.set_alignment(alignment)?;
                    self.alignment_ready = true;
                    log::info!("固定启动位姿与静止 VIO 对齐，地图里程计就绪");
                }
                self.t_sim = self.t_sim.max(m.timestamp);
                self.last_odom_recv = std::time::Instant::now();
                let t_vio = pose.matrix();
                self.fusion.predict(&t_vio);
                self.latest_odom = Some(m);
                self.odom_hist.push_back((m.timestamp, m));
                while self.odom_hist.len() > ODOM_HIST_CAP {
                    self.odom_hist.pop_front();
                }
                odom_arrived = true;
            }
            // odom 到达即追一次待融合队列：观测先到、odom 后到的竞态在此闭合，
            // 无需延时等待——唤醒源仍是数据到达本身。
            if odom_arrived {
                self.drain_pending_visual();
            }
        }
        if self.visual_obs.is_some() {
            let mut arrived = Vec::new();
            if let Some(sub) = &self.visual_obs {
                while let Some(sample) = sub.receive()? {
                    let obs: PoseObservation = *sample;
                    arrived.push(obs);
                }
            }
            for obs in arrived {
                self.enqueue_visual(obs);
            }
            self.drain_pending_visual();
        }
        Ok(())
    }

    /// 观测入队：按 `timestamp` 有序插入，溢出丢最旧（值语义：`Copy` 类型，
    /// 384B 过栈拷贝的代价远小于一次 `PnP`/融合，见 `fuse_visual` 同惯例）。
    #[allow(clippy::large_types_passed_by_value)]
    fn enqueue_visual(&mut self, obs: PoseObservation) {
        if !obs.timestamp.is_finite()
            || obs.timestamp < 0.
            || obs.timestamp <= self.last_visual
            || obs.source != firefly_pubsub::vision::OBS_SOURCE_VISUAL
            || self
                .pending_visual
                .iter()
                .any(|o| o.timestamp.to_bits() == obs.timestamp.to_bits())
        {
            return;
        }
        let pos = self
            .pending_visual
            .iter()
            .position(|o| o.timestamp > obs.timestamp)
            .unwrap_or(self.pending_visual.len());
        self.pending_visual.insert(pos, obs);
        while self.pending_visual.len() > PENDING_VISUAL_CAP {
            self.pending_visual.pop_front();
        }
    }

    /// 排空待融合队列：`ts` 落入 `odom_hist` 内插范围即融合；超期即丢弃；
    /// 队首仍超前（odom 未追上）即停——等下次 odom 到达再排，不丢弃。
    fn drain_pending_visual(&mut self) {
        while let Some(ts) = self.pending_visual.front().map(|o| o.timestamp) {
            if ts <= self.last_visual
                || self.odom_hist.front().is_some_and(|o| ts < o.0)
                || self.t_sim - ts > PENDING_VISUAL_TIMEOUT
            {
                self.pending_visual.pop_front();
                log::info!(
                    "视觉观测超期丢弃（ts={ts:.2}，滞后 {:.2}s，少一次修正）",
                    self.t_sim - ts
                );
                continue;
            }
            if interp_odom(&self.odom_hist, ts).is_none() {
                break;
            }
            let obs = self.pending_visual.pop_front().expect("front checked");
            log::debug!(
                "视觉融合（ts={:.2}，管线滞后 {:.2}s）",
                obs.timestamp,
                self.t_sim - obs.timestamp
            );
            self.last_visual = ts;
            self.fuse_visual(&obs);
        }
    }

    /// 视觉观测融合：按观测时刻插值 odom 作预测位姿，同一 `FusionFilter` 更新。
    #[fastrace::trace]
    fn fuse_visual(&mut self, obs: &PoseObservation) {
        let Some(t_vio) = interp_odom(&self.odom_hist, obs.timestamp) else {
            log::debug!("视觉观测无 odom 内插（ts={:.2}），跳过", obs.timestamp);
            return;
        };
        let gate = self.fusion.update_with_observation(&t_vio, obs);
        self.publish_gate_viz(&gate, self.innov_limit_visual);
        match gate {
            RelocGate::Accepted {
                chi2, threshold, ..
            } => {
                log::info!(
                    "视觉矫正接受 chi2 {chi2:.2}/{threshold:.2} inliers {}/{} err {:.3}",
                    obs.num_inliers,
                    obs.total_points,
                    obs.error
                );
            }
            RelocGate::RejectedChi2 { chi2, threshold } => {
                log::debug!("视觉 chi2 拒收 {chi2:.2}>{threshold:.2}");
            }
            RelocGate::RejectedInnovation { trans, rot_deg } => {
                log::debug!("视觉新息拒收 trans {trans:.2}m rot {rot_deg:.2}°");
            }
            RelocGate::RejectedPrecheck { reason } => {
                log::debug!("视觉预检拒收: {reason}");
            }
            RelocGate::RejectedNumerical { reason } => {
                log::warn!("视觉数值异常拒收: {reason}");
            }
        }
    }

    fn publish_corrected(&mut self) -> Result<()> {
        if !self.alignment_ready || self.estimator_reset {
            return Ok(());
        }
        let Some(pub_) = &self.corrected_pub else {
            return Ok(());
        };
        let Some(odom) = &self.latest_odom else {
            return Ok(());
        };
        if self.last_odom_recv.elapsed().as_secs_f64() >= ODOM_FRESH_TIMEOUT
            || odom.timestamp <= self.last_published
        {
            return Ok(());
        }
        let alignment =
            RigidTransform::from_matrix(FrameId::MAP, FrameId::ODOM, self.fusion.drift())?;
        let msg = corrected_odom(odom, &alignment)?;
        pub_.publish(msg).map(|_| ())?;
        self.last_published = msg.timestamp;
        self.log_corrected_viz(&msg);
        Ok(())
    }

    /// 门控诊断可视化（每次融合尝试即发，事件驱动，无尝试不发）。
    ///
    /// `corr/debug/gate` 四元组见 [`gate_diag`]，`corr/debug/drift` 三元组为
    /// 判决时刻累计漂移 `T_drift` 的平移分量 `[dx, dy, dz]`。发布失败只降级，
    /// 不影响融合。
    fn publish_gate_viz(&self, gate: &RelocGate, innov_limit: f64) {
        let Some([metric, limit, applied, accepted]) = gate_diag(gate, innov_limit) else {
            return;
        };
        let Some(viz) = &self.viz_pub else {
            return;
        };
        let mut g = VizMessage::base(kind::SCALARS, self.t_sim, "corr/debug/gate");
        g.scalars[0] = metric;
        g.scalars[1] = limit;
        g.scalars[2] = applied;
        g.scalars[3] = accepted;
        g.scalar_count = 4;
        if let Err(e) = viz.publish(g) {
            log::debug!("viz 发布 corr/debug/gate 失败：{e}");
            return;
        }
        let d = self.fusion.drift();
        let mut drift = VizMessage::base(kind::SCALARS, self.t_sim, "corr/debug/drift");
        drift.scalars[0] = d[(0, 3)];
        drift.scalars[1] = d[(1, 3)];
        drift.scalars[2] = d[(2, 3)];
        drift.scalar_count = 3;
        if let Err(e) = viz.publish(drift) {
            log::debug!("viz 发布 corr/debug/drift 失败：{e}");
        }
    }

    /// 矫正后位姿可视化（`corr/odom` 位姿 + `corr/traj` 增量轨迹段，绿，
    /// 与 vio `log_viz` 同增量段写法；发布失败只降级，不影响融合）。
    fn log_corrected_viz(&mut self, msg: &OdomMessage) {
        if !self.reloc_ticks.is_multiple_of(VIZ_PERIOD) {
            return;
        }
        let Some(viz) = &self.viz_pub else {
            return;
        };
        let pos = [msg.position_x, msg.position_y, msg.position_z];
        let quat = [msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w];
        let mut pose = VizMessage::base(kind::POSE, self.t_sim, "corr/odom");
        pose.color = [CORRECTED_COLOR.0, CORRECTED_COLOR.1, CORRECTED_COLOR.2];
        pose.xyz = pos;
        pose.quat_xyzw = quat;
        if let Err(e) = viz.publish(pose) {
            log::debug!("viz 发布 corr 位姿失败：{e}");
            return;
        }
        if let Some(prev) = self.corr_prev {
            let mut seg = VizMessage::base(kind::LINE_STRIP, self.t_sim, "corr/traj");
            seg.color = [CORRECTED_COLOR.0, CORRECTED_COLOR.1, CORRECTED_COLOR.2];
            seg.points[0] = prev;
            seg.points[1] = pos;
            seg.point_count = 2;
            if let Err(e) = viz.publish(seg) {
                log::debug!("viz 发布 corr/traj 段失败：{e}");
            }
        }
        self.corr_prev = Some(pos);
    }

    fn step(&mut self) -> Result<()> {
        self.poll_sensors()?;
        firefly_observability::set_sim_time(self.t_sim);
        firefly_observability::pump_log_ipc(&self.log_ipc);
        self.reloc_ticks = self.reloc_ticks.wrapping_add(1);
        self.publish_corrected()?;
        Ok(())
    }

    fn run(&mut self) -> Result<()> {
        log::info!(
            "localization 进程启动：订阅 VIO odom + 视觉位姿观测，发布 {CORRECTED_ODOM_TOPIC}"
        );
        let waitset = iceoryx2::waitset::WaitSetBuilder::new()
            .create::<iceoryx2::prelude::ipc::Service>()
            .map_err(|e| Error::new(ErrorKind::Internal, format!("创建 WaitSet 失败: {e:?}")))?;
        let tick_guard = waitset
            .attach_interval(LOOP_PERIOD)
            .map_err(|e| Error::new(ErrorKind::Internal, format!("挂载节拍定时器失败: {e:?}")))?;
        let on_tick = |attachment_id: iceoryx2::waitset::WaitSetAttachmentId<ipc::Service>| {
            if !attachment_id.has_event_from(&tick_guard) {
                return CallbackProgression::Continue;
            }
            let root = Span::root("localization", SpanContext::random().sampled(false));
            let guard = root.set_local_parent();
            let step = self.step();
            drop(guard);
            drop(root);
            if let Err(e) = step {
                log::warn!("tick 失败：{e}");
            }
            CallbackProgression::Continue
        };
        match waitset.wait_and_process(on_tick) {
            Ok(WaitSetRunResult::Interrupt | WaitSetRunResult::TerminationRequest) => {
                log::info!("收到终止信号，优雅退出");
            }
            Ok(_) => {}
            Err(e) => {
                return Err(Error::new(
                    ErrorKind::Internal,
                    format!("WaitSet 事件等待失败: {e:?}"),
                ));
            }
        }
        Ok(())
    }
}

fn main() {
    init_observability();
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!(
                "{e}\n用法：localization [--config configs/localization.toml] [--odom-topic Firefly/Odometry]"
            );
            std::process::exit(2);
        }
    };
    let cfg = match LocalizationConfig::load(&args.config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("加载配置失败 {}：{e}", args.config.display());
            std::process::exit(1);
        }
    };
    log::info!("已加载配置 {}", args.config.display());
    let mut app = match App::new(cfg, &args.odom_topic) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("初始化失败：{e}");
            firefly_observability::flush();
            std::process::exit(1);
        }
    };
    if let Err(e) = app.run() {
        log::error!("localization 失败：{e}");
        firefly_observability::pump_log_ipc(&app.log_ipc);
        firefly_observability::flush();
        std::process::exit(1);
    }
    firefly_observability::pump_log_ipc(&app.log_ipc);
    firefly_observability::flush();
}

#[cfg(test)]
mod tests {
    use super::{RelocGate, gate_diag};
    use nalgebra::Vector6;

    #[test]
    fn interpolation_has_closed_bounds_and_analytic_pose() {
        use super::*;
        let a = OdomMessage {
            timestamp: 1.,
            quat_w: 1.,
            ..Default::default()
        };
        let b = OdomMessage {
            timestamp: 3.,
            position_x: 4.,
            quat_w: 0.,
            quat_z: 1.,
            ..a
        };
        let history = [(1., a), (3., b)].into();
        assert!(interp_odom(&history, 0.999).is_none());
        assert!(interp_odom(&history, 3.001).is_none());
        assert!(interp_odom(&history, f64::NAN).is_none());
        let mid = interp_odom(&history, 2.).unwrap();
        assert!((mid[(0, 3)] - 2.).abs() < 1e-12);
        assert!((mid[(1, 0)] - 1.).abs() < 1e-12);
        assert!(interp_odom(&history, 1.).unwrap()[(0, 3)].abs() < 1e-12);
        assert!((interp_odom(&history, 3.).unwrap()[(0, 3)] - 4.).abs() < 1e-12);
    }

    /// 接受：`chi2`/阈值透传，平移修正量取 `delta` 平移分量模，标志为 1。
    #[test]
    fn accepted_maps_chi2_and_applied() {
        let mut delta = Vector6::zeros();
        delta[3] = 0.06;
        delta[4] = 0.08;
        let gate = RelocGate::Accepted {
            chi2: 1.5,
            threshold: 6.3,
            delta,
        };
        let d = gate_diag(&gate, 0.3).expect("accepted 必须有诊断值");
        assert!((d[0] - 1.5).abs() < 1e-12);
        assert!((d[1] - 6.3).abs() < 1e-12);
        assert!((d[2] - 0.1).abs() < 1e-12);
        assert!((d[3] - 1.0).abs() < 1e-12);
    }

    /// 新息拒收：`metric` 为新息平移量，`limit` 为传入的门限，修正量为 0。
    #[test]
    fn innovation_reject_maps_trans_and_limit() {
        let gate = RelocGate::RejectedInnovation {
            trans: 1.6,
            rot_deg: 2.0,
        };
        let d = gate_diag(&gate, 0.3).expect("innovation 拒收必须有诊断值");
        assert!((d[0] - 1.6).abs() < 1e-12);
        assert!((d[1] - 0.3).abs() < 1e-12);
        assert!(d[2].abs() < 1e-12);
        assert!(d[3].abs() < 1e-12);
    }

    /// 预检拒收无判决数值：返回空，调用方不发标量。
    #[test]
    fn precheck_maps_none() {
        let gate = RelocGate::RejectedPrecheck {
            reason: "not converged",
        };
        assert!(gate_diag(&gate, 0.3).is_none());
    }
}
