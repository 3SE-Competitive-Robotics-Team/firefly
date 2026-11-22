use std::path::PathBuf;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use firefly_localization::CorrectionOptions;
use firefly_localization::config::LocalizationConfig;
use firefly_localization::convert::corrected_odom;
use firefly_localization::filter::{FusionFilter, RelocGate};
use firefly_localization::quality::{Quality, QualityOptions};
use firefly_observability::init as init_observability;
use firefly_pubsub::navigation::{NAVIGATION_TOPIC, NavigationMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::CORRECTED_ODOM_TOPIC;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::vision::{POSE_OBS_TOPIC, PoseObservation};
use firefly_pubsub::viz::{ScalarMessage, VizMessage, VizPublisher, kind};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::{Isometry3, Matrix4, Quaternion, Translation3, UnitQuaternion, Vector3};

const LOOP_PERIOD: Duration = Duration::from_millis(10);
const CORRECTED_COLOR: (u8, u8, u8) = (60, 200, 80);
fn default_config() -> PathBuf {
    firefly_config::paths::config("localization.toml")
}
const ODOM_FRESH_TIMEOUT: f64 = 1.0;
const PENDING_VISUAL_CAP: usize = 8;
const PENDING_VISUAL_TIMEOUT: f64 = 10.0;
const ODOM_HIST_CAP: usize = 1000;

/// 发布校正后的里程计
#[derive(usage_rs::Cli)]
#[usage(bin = "localization")]
struct Arguments {
    /// 应用配置文件
    #[usage(long)]
    config: Option<PathBuf>,
    /// 输入里程计 topic
    #[usage(long)]
    odom_topic: Option<String>,
}

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

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum AlignmentState {
    Waiting,
    Ready,
    Reset,
}

#[derive(Clone, Copy)]
struct CommittedAlignment {
    transform: RigidTransform,
    revision: u64,
}

struct App {
    fusion: FusionFilter,
    quality: Quality,
    quality_options: QualityOptions,
    origin: firefly_localization::config::InitialAlignment,
    alignment: AlignmentState,
    navigation_pub: Publisher<NavigationMessage>,
    /// 最近一次提交坐标系时的对齐；修订号只随超过门限的修正递增。
    committed: Option<CommittedAlignment>,
    correction: CorrectionOptions,
    odom_sub: Subscriber<OdomMessage>,
    corrected_pub: Publisher<OdomMessage>,
    viz_pub: Option<VizPublisher>,
    corr_prev: Option<[f64; 3]>,
    latest_odom: Option<(OdomMessage, Instant)>,
    visual_obs: Subscriber<PoseObservation>,
    odom_hist: std::collections::VecDeque<(f64, OdomMessage)>,
    pending_visual: std::collections::VecDeque<PoseObservation>,
    last_published: Option<f64>,
    last_visual: Option<f64>,
    innov_limit_visual: f64,
    t_sim: f64,
    log_ipc: firefly_observability::LogIpc,
    _node: firefly_pubsub::node::IpcNode,
}

impl App {
    fn new(cfg: LocalizationConfig, odom_topic: &str) -> Result<Self> {
        cfg.validate()?;
        let origin = cfg.origin.ok_or_else(|| {
            Error::new(
                ErrorKind::InvalidArgument,
                "external localization requires an explicit origin",
            )
            .with_operation("create_external_localization")
        })?;
        let innov_limit_visual = cfg.fusion.visual.max_innovation_trans;
        let fusion = FusionFilter::new(cfg.fusion);
        let node = create_node()?;
        let log_ipc = firefly_observability::init_ipc(&node, "localization");
        let odom_sub = Subscriber::with_topic(&node, odom_topic)?;
        let visual_obs = Subscriber::with_topic(&node, POSE_OBS_TOPIC)?;
        let corrected_pub = Publisher::with_topic(&node, CORRECTED_ODOM_TOPIC)?;
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
            quality: Quality::default(),
            quality_options: cfg.quality,
            origin,
            alignment: AlignmentState::Waiting,
            navigation_pub: Publisher::with_topic(&node, NAVIGATION_TOPIC)?,
            committed: None,
            correction: cfg.correction,
            odom_sub,
            visual_obs,
            corrected_pub,
            viz_pub,
            corr_prev: None,
            latest_odom: None,
            odom_hist: std::collections::VecDeque::with_capacity(ODOM_HIST_CAP),
            pending_visual: std::collections::VecDeque::with_capacity(PENDING_VISUAL_CAP),
            last_published: None,
            last_visual: None,
            innov_limit_visual,
            t_sim: 0.0,
            log_ipc,
            _node: node,
        })
    }

    fn poll_sensors(&mut self) -> Result<()> {
        {
            let sub = &self.odom_sub;
            let mut odom_arrived = false;
            while let Some(sample) = sub.receive()? {
                let m: OdomMessage = *sample;
                if self.alignment == AlignmentState::Ready && !m.is_initialized {
                    self.alignment = AlignmentState::Reset;
                    log::error!("VIO 初始化状态丢失，地图输出锁存停止；须重启会话");
                }
                if self.alignment == AlignmentState::Reset {
                    continue;
                }
                if !m.is_initialized
                    || !m.timestamp.is_finite()
                    || m.timestamp < 0.0
                    || self
                        .latest_odom
                        .as_ref()
                        .is_some_and(|(last, _)| m.timestamp <= last.timestamp)
                {
                    continue;
                }
                let Ok(pose) = m.body_pose(FrameId::ODOM) else {
                    continue;
                };
                if self.alignment == AlignmentState::Waiting {
                    let Ok(alignment) = self.origin.at_start(&m) else {
                        continue;
                    };
                    self.fusion.set_alignment(alignment)?;
                    self.alignment = AlignmentState::Ready;
                    log::info!("启动坐标系与静止 VIO 对齐，地图里程计就绪");
                }
                self.t_sim = self.t_sim.max(m.timestamp);
                self.latest_odom = Some((m, Instant::now()));
                let t_vio = pose.matrix();
                self.fusion.predict(&t_vio);
                self.odom_hist.push_back((m.timestamp, m));
                while self.odom_hist.len() > ODOM_HIST_CAP {
                    self.odom_hist.pop_front();
                }
                odom_arrived = true;
            }
            if odom_arrived {
                self.drain_pending_visual();
            }
        }
        let mut arrived = Vec::new();
        while let Some(sample) = self.visual_obs.receive()? {
            arrived.push(*sample);
        }
        for obs in &arrived {
            self.enqueue_visual(obs);
        }
        self.drain_pending_visual();
        Ok(())
    }

    fn enqueue_visual(&mut self, obs: &PoseObservation) {
        if !obs.timestamp.is_finite()
            || obs.timestamp < 0.
            || self.last_visual.is_some_and(|last| obs.timestamp <= last)
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
        self.pending_visual.insert(pos, *obs);
        while self.pending_visual.len() > PENDING_VISUAL_CAP {
            self.pending_visual.pop_front();
        }
    }

    /// 排空待融合队列：`ts` 落入 `odom_hist` 内插范围即融合；超期即丢弃；
    /// 队首仍超前（odom 未追上）即停——等下次 odom 到达再排，不丢弃。
    fn drain_pending_visual(&mut self) {
        while let Some(ts) = self.pending_visual.front().map(|o| o.timestamp) {
            if self.last_visual.is_some_and(|last| ts <= last)
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
            self.last_visual = Some(ts);
            self.fuse_visual(&obs);
        }
    }

    #[fastrace::trace]
    fn fuse_visual(&mut self, obs: &PoseObservation) {
        let Some(t_vio) = interp_odom(&self.odom_hist, obs.timestamp) else {
            log::debug!("视觉观测无 odom 内插（ts={:.2}），跳过", obs.timestamp);
            return;
        };
        let gate = self.fusion.update_with_observation(&t_vio, obs);
        if matches!(gate, RelocGate::Accepted { .. }) {
            let predicted = self.fusion.drift() * t_vio;
            self.quality.observe(
                obs.timestamp,
                (predicted.fixed_view::<3, 1>(0, 3)
                    - Vector3::new(obs.position_x, obs.position_y, obs.position_z))
                .norm(),
            );
        }
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

    fn next_alignment(
        &self,
        transform: RigidTransform,
        odom: &OdomMessage,
    ) -> Result<CommittedAlignment> {
        let Some(previous) = self.committed else {
            return Ok(CommittedAlignment {
                transform,
                revision: 0,
            });
        };
        let body = odom.body_pose(FrameId::ODOM)?;
        if !self.correction.committed(
            &transform.compose(&body)?,
            &previous.transform.compose(&body)?,
        ) {
            return Ok(previous);
        }
        let revision = previous
            .revision
            .checked_add(1)
            .ok_or_else(|| Error::new(ErrorKind::Internal, "alignment revision exhausted"))?;
        Ok(CommittedAlignment {
            transform,
            revision,
        })
    }

    fn publish_corrected(&mut self) -> Result<()> {
        if self.alignment != AlignmentState::Ready {
            return Ok(());
        }
        let Some((odom, received)) = &self.latest_odom else {
            return Ok(());
        };
        if received.elapsed().as_secs_f64() >= ODOM_FRESH_TIMEOUT
            || self
                .last_published
                .is_some_and(|last| odom.timestamp <= last)
        {
            return Ok(());
        }
        let alignment =
            RigidTransform::from_matrix(FrameId::MAP, FrameId::ODOM, self.fusion.drift())?;
        let status = self.quality.status(odom.timestamp, &self.quality_options);
        if let Some(viz) = &self.viz_pub {
            let mut metric = ScalarMessage::base(odom.timestamp, "corr/debug/map_observation");
            metric.scalars[..3].copy_from_slice(&[
                if status.visual_timestamp >= 0. {
                    odom.timestamp - status.visual_timestamp
                } else {
                    -1.
                },
                status.position_disagreement,
                f64::from(status.recent_consistent),
            ]);
            metric.scalar_count = 3;
            let _ = viz.publish(metric);
        }
        // 同一修订必须发布同一变换；融合候选累计超过门限后再提交。
        let committed = self.next_alignment(alignment, odom)?;
        self.committed = Some(committed);
        let msg = corrected_odom(odom, &committed.transform)?;
        let mut navigation = NavigationMessage::new(msg, &committed.transform, committed.revision);
        navigation.external_map = 1;
        self.navigation_pub.publish(navigation)?;
        self.corrected_pub.publish(msg)?;
        self.last_published = Some(msg.timestamp);
        self.log_corrected_viz(&msg);
        Ok(())
    }

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
        let mut g = ScalarMessage::base(self.t_sim, "corr/debug/gate");
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
        let mut drift = ScalarMessage::base(self.t_sim, "corr/debug/drift");
        drift.scalars[0] = d[(0, 3)];
        drift.scalars[1] = d[(1, 3)];
        drift.scalars[2] = d[(2, 3)];
        drift.scalar_count = 3;
        if let Err(e) = viz.publish(drift) {
            log::debug!("viz 发布 corr/debug/drift 失败：{e}");
        }
    }

    fn log_corrected_viz(&mut self, msg: &OdomMessage) {
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
        self.publish_corrected()?;
        Ok(())
    }

    fn run(&mut self) -> Result<()> {
        log::info!("外部位姿融合启动：ESKF，发布 {CORRECTED_ODOM_TOPIC}");
        let waitset = iceoryx2::waitset::WaitSetBuilder::new()
            .create::<iceoryx2::prelude::ipc::Service>()
            .map_err(|e| Error::caused_by(ErrorKind::Internal, "创建 WaitSet 失败", e))?;
        let tick_guard = waitset
            .attach_interval(LOOP_PERIOD)
            .map_err(|e| Error::caused_by(ErrorKind::Internal, "挂载节拍定时器失败", e))?;
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
                return Err(Error::caused_by(
                    ErrorKind::Internal,
                    "WaitSet 事件等待失败",
                    e,
                ));
            }
        }
        Ok(())
    }
}

fn main() -> std::process::ExitCode {
    let result = run();
    firefly_observability::flush();
    result.unwrap_or_else(|e| firefly_error::report("localization", Err(e)))
}

fn run() -> Result<std::process::ExitCode> {
    let args = Arguments::parse();
    let config = args.config.unwrap_or_else(default_config);
    let cfg = firefly_config::startup::<LocalizationConfig>(
        &config,
        init_observability,
        LocalizationConfig::validate,
    )?;
    let mut app = App::new(
        cfg,
        args.odom_topic
            .as_deref()
            .unwrap_or(firefly_pubsub::odom::ODOM_TOPIC),
    )?;
    let code =
        firefly_error::report_with(app.run(), |error| log::error!("localization 失败：{error}"));
    firefly_observability::pump_log_ipc(&app.log_ipc);
    Ok(code)
}

#[cfg(test)]
mod tests {
    use super::{
        AlignmentState, App, CORRECTED_ODOM_TOPIC, LocalizationConfig, OdomMessage, Publisher,
        RelocGate, Subscriber, create_node, gate_diag,
    };
    use nalgebra::Vector6;

    #[test]
    fn corrections_below_the_drift_threshold_keep_the_published_alignment() {
        std::thread::Builder::new()
            .stack_size(8 * 1024 * 1024)
            .spawn(|| {
                use super::*;
                use firefly_pubsub::publish::Publisher;
                let topic = format!("Firefly/Test/MapOdometry/{}", std::process::id());
                let mut app = App::new(toml::from_str("[origin]").unwrap(), &topic).unwrap();
                let node = create_node().unwrap();
                let _raw = Publisher::<OdomMessage>::with_topic(&node, &topic).unwrap();
                let corrected_topic = format!("{topic}/Corrected");
                let navigation_topic = format!("{topic}/Navigation");
                app.corrected_pub = Publisher::with_topic(&node, &corrected_topic).unwrap();
                app.navigation_pub = Publisher::with_topic(&node, &navigation_topic).unwrap();
                let corrected =
                    Subscriber::<OdomMessage>::with_topic(&node, &corrected_topic).unwrap();
                let navigation =
                    Subscriber::<NavigationMessage>::with_topic(&node, &navigation_topic).unwrap();
                let shifted = |x: f64| {
                    RigidTransform::from_parts(
                        FrameId::MAP,
                        FrameId::ODOM,
                        [x, 0., 0.],
                        [0., 0., 0., 1.],
                    )
                    .unwrap()
                };
                let base = shifted(0.);
                app.alignment = AlignmentState::Ready;
                app.fusion.set_alignment(base).unwrap();
                let mut sample = OdomMessage {
                    timestamp: 3.,
                    is_initialized: true,
                    ..Default::default()
                };
                app.latest_odom = Some((sample, Instant::now()));
                app.publish_corrected().unwrap();
                let initial = *navigation.receive().unwrap().unwrap();
                assert_eq!(initial.revision, 0);
                assert_eq!(initial.translation, [0.; 3]);
                assert_eq!(corrected.receive().unwrap().unwrap().position_x, 0.);
                app.fusion.set_alignment(shifted(0.09)).unwrap();
                sample.timestamp = 4.;
                app.latest_odom = Some((sample, Instant::now()));
                app.publish_corrected().unwrap();
                let small = *navigation.receive().unwrap().unwrap();
                assert_eq!(small.revision, initial.revision);
                assert_eq!(small.translation, initial.translation);
                assert_eq!(small.rotation, initial.rotation);
                assert_eq!(small.odom.position_x, 0.);
                assert_eq!(corrected.receive().unwrap().unwrap().position_x, 0.);
                app.fusion.set_alignment(shifted(0.11)).unwrap();
                sample.timestamp = 5.;
                app.latest_odom = Some((sample, Instant::now()));
                app.publish_corrected().unwrap();
                let committed = *navigation.receive().unwrap().unwrap();
                assert_eq!(committed.revision, 1);
                assert_eq!(committed.translation[0], 0.11);
                assert_eq!(committed.odom.position_x, 0.11);
                assert_eq!(corrected.receive().unwrap().unwrap().position_x, 0.11);
            })
            .unwrap()
            .join()
            .unwrap();
    }

    #[test]
    fn optional_external_localization_preserves_odometry_freshness_and_latches_reset() {
        let topic = format!("Firefly/Test/ExternalLocalization/{}", std::process::id());
        let mut app = App::new(toml::from_str("[origin]").unwrap(), &topic).unwrap();
        let node = create_node().unwrap();
        let raw = Publisher::<OdomMessage>::with_topic(&node, &topic).unwrap();
        let corrected = Subscriber::<OdomMessage>::with_topic(&node, CORRECTED_ODOM_TOPIC).unwrap();
        let mut sample = OdomMessage {
            timestamp: 2.0,
            is_initialized: true,
            ..Default::default()
        };
        raw.publish(sample).unwrap();
        app.step().unwrap();
        assert_eq!(app.alignment, AlignmentState::Ready);
        assert!(corrected.receive().unwrap().is_some());
        sample.timestamp = 3.0;
        sample.position_x = 2.0;
        raw.publish(sample).unwrap();
        app.step().unwrap();
        assert!((corrected.receive().unwrap().unwrap().position_x - 2.0).abs() < 1e-12);
        let stale = std::time::Instant::now()
            .checked_sub(std::time::Duration::from_secs(2))
            .unwrap();
        sample.timestamp = 3.1;
        app.latest_odom = Some((sample, stale));
        app.publish_corrected().unwrap();
        assert!(corrected.receive().unwrap().is_none());
        sample.timestamp = 4.0;
        sample.is_initialized = false;
        raw.publish(sample).unwrap();
        app.step().unwrap();
        assert_eq!(app.alignment, AlignmentState::Reset);
        sample.timestamp = 5.0;
        sample.is_initialized = true;
        raw.publish(sample).unwrap();
        app.step().unwrap();
        assert!(corrected.receive().unwrap().is_none());
    }

    #[test]
    fn optional_localization_requires_explicit_external_origin_before_opening_ipc() {
        assert!(
            App::new(
                LocalizationConfig::default(),
                "Firefly/Test/NoExternalOrigin"
            )
            .is_err()
        );
    }

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

    #[test]
    fn precheck_maps_none() {
        let gate = RelocGate::RejectedPrecheck {
            reason: "not converged",
        };
        assert!(gate_diag(&gate, 0.3).is_none());
    }
}
