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
use firefly_localization::filter::FusionFilter;
use firefly_localization::reloc::GlobalRelocalizer;
use firefly_map::{DepthCamera, MapFile};
use firefly_observability::init as init_observability;
use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::{CORRECTED_ODOM_TOPIC, CorrectedOdomPublisher};
use firefly_pubsub::subscriber::{OdomSubscriber, Subscriber};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::{Isometry3, Matrix4, Quaternion, Translation3, UnitQuaternion, Vector3, Vector4};

const LOOP_PERIOD: Duration = Duration::from_millis(100);
const RELOC_PERIOD: usize = 10;
const DEFAULT_CONFIG: &str = "configs/gicp.toml";
const DEFAULT_MAP_HINT: &str = "未指定 --map，加载 MuJoCo 默认场景静态地图";
const ODOM_FRESH_TIMEOUT: f64 = 1.0;

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
        odom_topic: firefly_pubsub::publish::ODOM_TOPIC.to_string(),
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

fn odom_to_matrix(msg: &OdomMessage) -> Matrix4<f64> {
    let t = Vector3::new(msg.position_x, msg.position_y, msg.position_z);
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        msg.quat_w, msg.quat_x, msg.quat_y, msg.quat_z,
    ));
    Isometry3::from_parts(Translation3::new(t.x, t.y, t.z), q).to_homogeneous()
}

fn matrix_to_odom(t_corr: &Matrix4<f64>, src: &OdomMessage, drift: &Matrix4<f64>) -> OdomMessage {
    let p = t_corr.fixed_view::<3, 1>(0, 3).into_owned();
    let r = t_corr.fixed_view::<3, 3>(0, 0).into_owned();
    let quat = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&r));
    let q = quat.quaternion();
    let drift_rot = drift.fixed_view::<3, 3>(0, 0).into_owned();
    let v = Vector3::new(src.velocity_x, src.velocity_y, src.velocity_z);
    let v_corr = drift_rot * v;
    OdomMessage {
        timestamp: src.timestamp,
        position_x: p.x,
        position_y: p.y,
        position_z: p.z,
        velocity_x: v_corr.x,
        velocity_y: v_corr.y,
        velocity_z: v_corr.z,
        quat_x: q.i,
        quat_y: q.j,
        quat_z: q.k,
        quat_w: q.w,
        is_initialized: src.is_initialized,
    }
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
            corrected_pub,
            latest_odom: None,
            latest_depth: None,
            last_odom_recv: f64::NEG_INFINITY,
            depth_cam: DepthCamera::mujoco_default(),
            t_sim: 0.0,
            _node: node,
        })
    }

    fn poll_sensors(&mut self) -> Result<()> {
        if let Some(sub) = &self.viewer_odom {
            while let Some(sample) = sub.receive()? {
                let m: OdomMessage = *sample;
                self.t_sim = self.t_sim.max(m.timestamp);
                self.last_odom_recv = m.timestamp;
                let t_vio = odom_to_matrix(&m);
                self.fusion.predict(&t_vio);
                self.latest_odom = Some(m);
            }
        }
        if let Some(sub) = &self.depth {
            while let Some(sample) = sub.receive()? {
                let m: DepthImageMessage = *sample;
                self.t_sim = self.t_sim.max(m.timestamp);
                self.latest_depth = Some(m);
            }
        }
        Ok(())
    }

    #[fastrace::trace]
    fn try_relocalize(&mut self) {
        let Some(reloc) = &self.reloc else { return };
        let Some(depth) = &self.latest_depth else {
            return;
        };
        let Some(odom) = &self.latest_odom else {
            return;
        };
        if !self.reloc_ticks.is_multiple_of(RELOC_PERIOD) {
            return;
        }
        let t_vio = odom_to_matrix(odom);
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
