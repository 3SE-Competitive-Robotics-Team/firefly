//! GICP 全局重定位进程：低频矫正 VIO 漂移。
//!
//! 订阅 `Firefly/Odometry`（VIO）+ `Firefly/Depth`（深度），以静态先验
//! `MapFile` 为靶图做 `GICP`，经 `FusionFilter`（`R=h⁻¹` + `chi2`）融合后
//! 发布 `Firefly/CorrectedOdometry` 供 `planner` 订阅（回退到原始 odom）。
//!
//! 运行：`cargo run -p gicp`（配合 `uv run firefly-sim` + `cargo run -p vio` + `cargo run -p planner`）
//! 或 `cargo run -p gicp -- --map apps/planner/maps/gate.ffmap`。

use std::path::PathBuf;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use firefly_gicp::points::point_cloud::PointCloud;
use firefly_gicp::points::traits::{PointCloudMut, PointCloudTrait};
use firefly_localization::config::LocalizationConfig;
use firefly_localization::convert::{matrix_to_odom, odom_to_matrix};
use firefly_localization::filter::FusionFilter;
use firefly_localization::reloc::GlobalRelocalizer;
use firefly_map::{DepthCamera, MapFile};
use firefly_observability::init as init_observability;
use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::CORRECTED_ODOM_TOPIC;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::CorrectedOdomPublisher;
use firefly_pubsub::subscriber::{OdomSubscriber, Subscriber};
use firefly_pubsub::vision::{POSE_OBS_TOPIC, PoseObservation};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::{Isometry3, Matrix4, Quaternion, Translation3, UnitQuaternion, Vector3, Vector4};

const LOOP_PERIOD: Duration = Duration::from_millis(100);
const RELOC_PERIOD: usize = 10;
const DEFAULT_CONFIG: &str = "configs/gicp.toml";
const DEFAULT_MAP_HINT: &str = "未指定 --map，加载 MuJoCo 默认场景静态地图";
const ODOM_FRESH_TIMEOUT: f64 = 1.0;
/// 视觉观测待融合队列上限（条）：观测先到、odom 后到时暂存，按 `timestamp`
/// 排序，odom 追上即融合；溢出时丢最旧（对端断流的背压语义，非延时等待）。
const PENDING_VISUAL_CAP: usize = 8;
/// 待融合观测超期（秒）：相对 `t_sim` 过期即丢弃（对端断流时不无限囤积）。
const PENDING_VISUAL_TIMEOUT: f64 = 2.0;

/// 命令行参数。
struct Args {
    map: Option<PathBuf>,
    config: PathBuf,
    odom_topic: String,
}

fn parse_args() -> Result<Args> {
    let mut it = std::env::args().skip(1);
    let mut args = Args {
        map: None,
        config: PathBuf::from(DEFAULT_CONFIG),
        odom_topic: firefly_pubsub::odom::ODOM_TOPIC.to_string(),
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--map" => {
                args.map = Some(PathBuf::from(it.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --map value")
                })?));
            }
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

fn open_sub<T: std::fmt::Debug + ZeroCopySend + 'static>(
    node: &firefly_pubsub::node::IpcNode,
    topic: &str,
    ok_msg: &str,
    err_msg: &str,
) -> Option<Subscriber<T>> {
    match Subscriber::<T>::with_topic(node, topic) {
        Ok(s) => {
            log::info!("{ok_msg}（topic {topic}）");
            Some(s)
        }
        Err(e) => {
            log::warn!("{err_msg}: {e}");
            None
        }
    }
}

/// 历史 odom 在目标时刻插值（位置 lerp + 姿态 slerp；时刻越界返回空，
/// 该 tick 跳过——不以外推污染配准）。
fn interp_odom(
    hist: &std::collections::VecDeque<(f64, OdomMessage)>,
    t: f64,
) -> Option<Matrix4<f64>> {
    if hist.len() < 2 {
        return None;
    }
    let mut prev = &hist[0];
    for cur in hist.iter().skip(1) {
        if cur.0 >= t {
            let span = cur.0 - prev.0;
            if span <= 1e-9 {
                return Some(odom_to_matrix(&cur.1));
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

fn depth_to_body_cloud(depth: &[f32], cam: &DepthCamera) -> PointCloud {
    let mut pts = Vec::new();
    let mut v = 0usize;
    while v < cam.height {
        let mut u = 0usize;
        while u < cam.width {
            let z = f64::from(depth[v * cam.width + u]);
            if z > 0.05 && z <= cam.max_range && z.is_finite() {
                let dx = (u as f64 - cam.cx) / cam.focal;
                let dy = -(v as f64 - cam.cy) / cam.focal;
                let hit_cam = Vector3::new(dx * z, dy * z, -z);
                let hit_body = cam.pos_in_body + cam.rot_cam_to_body * hit_cam;
                pts.push(hit_body);
            }
            u += cam.pixel_step;
        }
        v += cam.pixel_step;
    }
    let mut cloud = PointCloud::new();
    cloud.resize(pts.len());
    for (i, p) in pts.into_iter().enumerate() {
        cloud.set_point(i, Vector4::new(p.x, p.y, p.z, 1.0));
    }
    cloud
}

fn mujoco_map_file() -> MapFile {
    // 复用 planner 的默认地图逻辑：与 MuJoCo scene.py 同构
    // 简化：空地图时 reloc 会 warn 并禁用；真实部署需 --map 指定 ffmap
    // 此处直接返回空，触发 from_map_file 的空检查
    let occupied = Vec::new();
    MapFile {
        resolution: 0.4,
        origin: [0.0, -5.0, 0.0],
        dims: [80, 35, 13],
        occupied,
        decor: Vec::new(),
        motions: Vec::new(),
    }
}

struct App {
    fusion: FusionFilter,
    reloc: Option<GlobalRelocalizer>,
    reloc_ticks: usize,
    viewer_odom: Option<OdomSubscriber>,
    depth: Option<Subscriber<DepthImageMessage>>,
    corrected_pub: Option<CorrectedOdomPublisher>,
    latest_odom: Option<OdomMessage>,
    latest_depth: Option<DepthImageMessage>,
    /// 视觉位姿观测订阅（`lightglue` 进程发布，同一 `FusionFilter` 融合）。
    visual_obs: Option<Subscriber<PoseObservation>>,
    /// odom 环形历史（时间戳，消息）：按观测时间戳插值位姿，消除
    /// 最新配对 ~0.1s 失配（1.5m/s 下 15cm 系统性错位）；256 深容忍低速 odom。
    odom_hist: std::collections::VecDeque<(f64, OdomMessage)>,
    /// 待融合视觉观测（按 `timestamp` 排序）：观测先到、odom 后到时暂存，
    /// odom 追上即融合——事件驱动的订阅关系，无延时等待。
    pending_visual: std::collections::VecDeque<PoseObservation>,
    last_odom_recv: f64,
    depth_cam: DepthCamera,
    t_sim: f64,
    _node: firefly_pubsub::node::IpcNode,
}

impl App {
    #[allow(clippy::needless_pass_by_value)]
    fn new(map_file: MapFile, cfg: LocalizationConfig, odom_topic: &str) -> Result<Self> {
        let fusion = FusionFilter::new(cfg.fusion);
        let reloc = match GlobalRelocalizer::from_map_file(&map_file, cfg.reloc) {
            Ok(r) => {
                log::info!("全局重定位靶图就绪（{} 点）", r.target().num_points());
                Some(r)
            }
            Err(e) => {
                log::warn!("全局重定位靶图不可用（空地图）：{e}");
                None
            }
        };
        let node = create_node()?;
        let odom_sub = match OdomSubscriber::with_topic(&node, odom_topic) {
            Ok(s) => {
                log::info!("已订阅 odom 话题（{odom_topic}，VIO/VOID 状态源）");
                Some(s)
            }
            Err(e) => {
                log::warn!("odom 订阅不可用：{e}");
                None
            }
        };
        let depth = open_sub::<DepthImageMessage>(
            &node,
            DEPTH_TOPIC,
            "已订阅深度话题（感知输入）",
            "深度订阅不可用，GICP 停用",
        );
        let visual_obs = open_sub::<PoseObservation>(
            &node,
            POSE_OBS_TOPIC,
            "已订阅视觉位姿观测（lightglue 输入）",
            "视觉观测订阅不可用，仅 GICP 融合",
        );
        let corrected_pub = match CorrectedOdomPublisher::new(&node) {
            Ok(p) => Some(p),
            Err(e) => {
                log::warn!("校正后里程计发布不可用：{e}");
                None
            }
        };
        Ok(Self {
            fusion,
            reloc,
            reloc_ticks: 0,
            viewer_odom: odom_sub,
            depth,
            visual_obs,
            corrected_pub,
            latest_odom: None,
            latest_depth: None,
            odom_hist: std::collections::VecDeque::with_capacity(256),
            pending_visual: std::collections::VecDeque::with_capacity(PENDING_VISUAL_CAP),
            last_odom_recv: f64::NEG_INFINITY,
            depth_cam: DepthCamera::mujoco_default(),
            t_sim: 0.0,
            _node: node,
        })
    }

    fn poll_sensors(&mut self) -> Result<()> {
        if let Some(sub) = &self.viewer_odom {
            let mut odom_arrived = false;
            while let Some(sample) = sub.receive()? {
                let m: OdomMessage = *sample;
                self.t_sim = self.t_sim.max(m.timestamp);
                self.last_odom_recv = m.timestamp;
                let t_vio = odom_to_matrix(&m);
                self.fusion.predict(&t_vio);
                self.latest_odom = Some(m);
                self.odom_hist.push_back((m.timestamp, m));
                while self.odom_hist.len() > 256 {
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
        if let Some(sub) = &self.depth {
            while let Some(sample) = sub.receive()? {
                let m: DepthImageMessage = *sample;
                self.t_sim = self.t_sim.max(m.timestamp);
                self.latest_depth = Some(m);
            }
        }
        if self.visual_obs.is_some() {
            let mut arrived = Vec::new();
            if let Some(sub) = &self.visual_obs {
                while let Some(sample) = sub.receive()? {
                    let obs: PoseObservation = *sample;
                    self.t_sim = self.t_sim.max(obs.timestamp);
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
            if self.t_sim - ts > PENDING_VISUAL_TIMEOUT {
                self.pending_visual.pop_front();
                continue;
            }
            if interp_odom(&self.odom_hist, ts).is_none() {
                break;
            }
            let obs = self.pending_visual.pop_front().expect("front checked");
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
        match self.fusion.update_with_observation(&t_vio, obs) {
            firefly_localization::filter::RelocGate::Accepted {
                chi2, threshold, ..
            } => {
                log::info!(
                    "视觉矫正接受 chi2 {chi2:.2}/{threshold:.2} inliers {}/{} err {:.3}",
                    obs.num_inliers,
                    obs.total_points,
                    obs.error
                );
            }
            firefly_localization::filter::RelocGate::RejectedChi2 { chi2, threshold } => {
                log::debug!("视觉 chi2 拒收 {chi2:.2}>{threshold:.2}");
            }
            firefly_localization::filter::RelocGate::RejectedInnovation { trans, rot_deg } => {
                log::debug!("视觉新息拒收 trans {trans:.2}m rot {rot_deg:.2}°");
            }
            firefly_localization::filter::RelocGate::RejectedPrecheck { reason } => {
                log::debug!("视觉预检拒收: {reason}");
            }
            firefly_localization::filter::RelocGate::RejectedNumerical { reason } => {
                log::warn!("视觉数值异常拒收: {reason}");
            }
        }
    }

    #[fastrace::trace]
    fn try_relocalize(&mut self) {
        let Some(reloc) = &self.reloc else { return };
        let Some(depth) = &self.latest_depth else {
            return;
        };
        let Some(_odom) = &self.latest_odom else {
            return;
        };
        if !self.reloc_ticks.is_multiple_of(RELOC_PERIOD) {
            return;
        }
        // 初值与量测同源时刻：按深度时间戳插值 odom（最新配对失配可达 0.1s）
        let Some(t_vio) = interp_odom(&self.odom_hist, depth.timestamp) else {
            return;
        };
        let body_cloud = depth_to_body_cloud(&depth.data, &self.depth_cam);
        if body_cloud.num_points() < 30 {
            return;
        }
        let init = self.fusion.corrected_pose(&t_vio);
        let res = reloc.relocalize(&body_cloud, &init);
        let total = res.total_points;
        let r = &res.result;
        let gate = self.fusion.update(
            &t_vio,
            &r.t_target_source,
            &r.h,
            r.num_inliers,
            total,
            r.error,
            r.converged,
        );
        match gate {
            firefly_localization::filter::RelocGate::Accepted {
                chi2, threshold, ..
            } => {
                log::info!(
                    "GICP矫正接受 chi2 {chi2:.2}/{threshold:.2} inliers {}/{} err {:.3}",
                    r.num_inliers,
                    total,
                    r.error
                );
            }
            firefly_localization::filter::RelocGate::RejectedChi2 { chi2, threshold } => {
                log::debug!("GICP chi2拒收 {chi2:.2}>{threshold:.2}");
            }
            firefly_localization::filter::RelocGate::RejectedInnovation { trans, rot_deg } => {
                log::debug!("GICP新息拒收 trans {trans:.2}m rot {rot_deg:.2}°（疑似别名误锁）");
            }
            firefly_localization::filter::RelocGate::RejectedPrecheck { reason } => {
                log::debug!("GICP预检拒收: {reason}");
            }
            firefly_localization::filter::RelocGate::RejectedNumerical { reason } => {
                log::warn!("GICP数值异常拒收: {reason}");
            }
        }
    }

    fn publish_corrected(&self) -> Result<()> {
        let Some(pub_) = &self.corrected_pub else {
            return Ok(());
        };
        let Some(odom) = &self.latest_odom else {
            return Ok(());
        };
        if self.t_sim - self.last_odom_recv >= ODOM_FRESH_TIMEOUT {
            return Ok(());
        }
        let t_vio = odom_to_matrix(odom);
        let t_corr = self.fusion.corrected_pose(&t_vio);
        let msg = matrix_to_odom(&t_corr, odom, self.fusion.drift());
        pub_.publish(msg).map(|_| ())
    }

    fn step(&mut self) -> Result<()> {
        self.poll_sensors()?;
        self.reloc_ticks = self.reloc_ticks.wrapping_add(1);
        self.try_relocalize();
        self.publish_corrected()?;
        Ok(())
    }

    fn run(&mut self) -> Result<()> {
        log::info!(
            "gicp 进程启动：订阅 VIO odom + 深度，1Hz GICP 融合，发布 {CORRECTED_ODOM_TOPIC}"
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
            let root = Span::root("gicp", SpanContext::random().sampled(false));
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
                "{e}\n用法：gicp [--map <map.ffmap>] [--config configs/gicp.toml] [--odom-topic Firefly/Odometry]"
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
    let map_file = if let Some(p) = &args.map {
        match MapFile::from_file(p) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("加载地图失败：{e}");
                std::process::exit(1);
            }
        }
    } else {
        log::info!("{DEFAULT_MAP_HINT}");
        // 尝试加载 MuJoCo 默认场景的静态地图（与 planner 同构）
        // 若文件不存在则用空地图占位（GICP 将自动禁用）
        let default_path = PathBuf::from("apps/planner/maps/gate.ffmap");
        if default_path.exists() {
            MapFile::from_file(&default_path).unwrap_or_else(|_| mujoco_map_file())
        } else {
            mujoco_map_file()
        }
    };
    let mut app = match App::new(map_file, cfg, &args.odom_topic) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("初始化失败：{e}");
            firefly_observability::flush();
            std::process::exit(1);
        }
    };
    if let Err(e) = app.run() {
        log::error!("gicp 失败：{e}");
        firefly_observability::flush();
        std::process::exit(1);
    }
    firefly_observability::flush();
}
