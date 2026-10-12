//! IPC 接线：真值订阅 → 固定 10Hz 回读 → 工作线程逐像素处理 → 图像发布。
//!
//! 时序（对照 `firefly-sim/main.py` 的 10Hz 相机节拍）：
//! - `poll_pose` 排空真值更新位姿，按**固定 10Hz 节拍**（`CAPTURE_PERIOD`，
//!   以 sim 时间为轴）发起回读，并激活传感器相机；回读被渲染世界取走后收起
//!   相机，限制传感器渲染只在有采集请求时执行。
//! - 三份同 `seq` 回复到齐后装配成 [`CaptureJob`] 交给工作线程；工作线程算完
//!   立即发布传感器，并回传 [`ProcessedCapture`] 供主线程更新调试显示。
//! - 发布走 [`Publisher::publish`]，自动注入续接后的 trace 上下文。

use std::collections::HashMap;
use std::sync::{
    Arc,
    atomic::{AtomicU32, Ordering},
};
use std::thread::JoinHandle;
use std::time::Instant;

use bevy::ecs::system::NonSend;
use bevy::prelude::*;
use fastrace::prelude::*;
use firefly_pubsub::camera::{
    CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage,
    IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH,
};
use firefly_pubsub::event::{CAMERA_PAIR_TOPIC, TopicNotifier};
use firefly_pubsub::navigation::{NAVIGATION_TOPIC, NavigationMessage};
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::odom::{GROUND_TRUTH_TOPIC, OdomMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::trace::TraceContext;
use firefly_pubsub::viz::{VizMessage, VizPublisher, kind};

use crate::capture::{CaptureHub, CaptureRequest, CapturedFrame, SensorKind};
use crate::process::{CaptureJob, Latest, ProcessedCapture, run_worker};
use crate::rig::{Eye, PoseState, apply_pose_to_eyes};
use crate::ui::DebugViews;

/// 出图节拍（秒，sim 时间轴）：固定 10Hz，与真值到达边沿解耦。
pub const CAPTURE_PERIOD: f64 = 0.1;

/// 节拍比较容差（秒）：sim 时间戳有浮点抖动，`next_stamp` 精确相等比较会
/// 偶发漏拍（漏一拍即掉到 5Hz）。取半个周期的 10%。
const CAPTURE_TOLERANCE: f64 = CAPTURE_PERIOD * 0.1;

/// 主线程订阅端口束（节点最后声明最后释放，对照 `firefly-pubsub/node` 的 Drop 纪律）。
///
/// iceoryx2 端口是单线程的（`Rc` 内核，非 `Send/Sync`），只能以
/// [`NonSend`](bevy::ecs::system::NonSend) 资源持有——触碰端口的系统
/// 自动约束到主线程，构造上不可能跨线程触碰。
pub struct IpcPorts {
    /// 位姿订阅（`Firefly/GroundTruth`，只保留最新）。
    pub pose_sub: Subscriber<OdomMessage>,
    /// 可视化订阅（`Firefly/Viz` 的地图、路径与 VIO 图像点集）。
    pub viz_sub: Option<Subscriber<VizMessage>>,
    /// 地图融合定位，只用于在线调试影子与轨迹。
    pub corrected_sub: Option<Subscriber<NavigationMessage>>,
    /// 进程共享节点，必须在全部端口之后释放。
    pub node: IpcNode,
}

/// 打开主线程的订阅端口。
///
/// # Errors
/// 节点/端口创建失败（IPC 资源不可用，见 `firefly-pubsub` 各构造器）。
pub fn open_ports(offline: bool) -> Result<IpcPorts, firefly_error::Error> {
    let node = create_node()?;
    let topic = |name: &str| {
        if offline {
            name.replacen("Firefly/", "Firefly/Offline/", 1)
        } else {
            name.to_owned()
        }
    };
    let pose_sub = Subscriber::with_topic(&node, &topic(GROUND_TRUTH_TOPIC))?;
    let viz_sub = crate::voxels::open_subscription(&node);
    let corrected_sub = if offline {
        None
    } else {
        Some(Subscriber::with_topic(&node, NAVIGATION_TOPIC)?)
    };
    log::info!("IPC 就绪：订阅位姿，offline={offline}");
    Ok(IpcPorts {
        pose_sub,
        viz_sub,
        corrected_sub,
        node,
    })
}

/// 传感器发布端口只在像素工作线程构造和使用，节点最后释放。
struct SensorPublishers {
    left_pub: Publisher<GrayImageMessage>,
    right_pub: Publisher<GrayImageMessage>,
    depth_pub: Publisher<DepthImageMessage>,
    pair_notify: TopicNotifier,
    viz_pub: VizPublisher,
    _node: IpcNode,
}

impl SensorPublishers {
    fn open(offline: bool) -> Result<Self, firefly_error::Error> {
        let node = create_node()?;
        let topic = |name: &str| {
            if offline {
                name.replacen("Firefly/", "Firefly/Offline/", 1)
            } else {
                name.to_owned()
            }
        };
        Ok(Self {
            left_pub: Publisher::with_topic(&node, &topic(CAMERA_LEFT_TOPIC))?,
            right_pub: Publisher::with_topic(&node, &topic(CAMERA_RIGHT_TOPIC))?,
            depth_pub: Publisher::with_topic(&node, &topic(DEPTH_TOPIC))?,
            pair_notify: TopicNotifier::with_topic(&node, &topic(CAMERA_PAIR_TOPIC))?,
            viz_pub: VizPublisher::new(&node)?,
            _node: node,
        })
    }
}

/// 待发布帧（三份同 `seq` 回复在此集合）。
#[derive(Resource)]
pub struct PendingFrames {
    /// `seq` → 已到回复。
    pub frames: HashMap<u64, Vec<CapturedFrame>>,
    /// 下一个回读序号。
    pub next_seq: u64,
}

impl Default for PendingFrames {
    fn default() -> Self {
        // 序号从 1 起：`HubInner::taken_seq` 初值 0 不能被误判为「本拍已被取走」。
        Self {
            frames: HashMap::new(),
            next_seq: 1,
        }
    }
}

/// 出图统计（诊断用：区分应用帧率与真实供图速率）。
#[derive(Resource, Default)]
pub struct CaptureStats {
    /// 本统计周期内已发布的拍数。
    pub published: u32,
    /// 最近发布拍的时间戳（秒，sim 时间）。
    pub last_stamp: f64,
}

/// 传感器出图节拍与相机激活状态（主线程独占）。
#[derive(Resource, Default)]
pub struct SensorCapture {
    /// 下一个允许触发的 sim 时刻（秒）。
    next_stamp: f64,
    /// 传感器相机是否处于出图窗口（激活中）。
    active: bool,
    /// 当前出图窗口对应的请求序号（渲染世界取走它即收起相机）。
    requested_seq: u64,
}

/// 逐像素处理与发布不依赖 UI 消费；任务与显示缓存各只保留最新一拍。
#[derive(Resource)]
pub struct CapturePipeline {
    jobs: Arc<Latest<CaptureJob>>,
    results: Arc<Latest<ProcessedCapture>>,
    published: Arc<AtomicU32>,
    worker: Option<JoinHandle<()>>,
}

impl CapturePipeline {
    pub fn spawn(
        options: crate::depth_noise::DepthNoiseOptions,
        offline: bool,
    ) -> Result<Self, firefly_error::Error> {
        let jobs = Arc::new(Latest::<CaptureJob>::default());
        let results = Arc::new(Latest::default());
        let published = Arc::new(AtomicU32::new(0));
        let (input, output, count) = (jobs.clone(), results.clone(), published.clone());
        let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel(1);
        let worker = std::thread::Builder::new()
            .name("sensor-process".to_owned())
            .spawn(move || {
                let ports = match SensorPublishers::open(offline) {
                    Ok(ports) => {
                        let _ = ready_tx.send(Ok(()));
                        ports
                    }
                    Err(error) => {
                        let _ = ready_tx.send(Err(error));
                        return;
                    }
                };
                run_worker(&input, &output, &options, offline, |result| {
                    publish(&ports, result);
                    count.fetch_add(1, Ordering::Relaxed);
                });
            })
            .map_err(|error| {
                firefly_error::Error::new(firefly_error::ErrorKind::Internal, error.to_string())
                    .with_context("operation", "spawn sensor processor")
            })?;
        let pipeline = Self {
            jobs,
            results,
            published,
            worker: Some(worker),
        };
        ready_rx.recv().map_err(|error| {
            firefly_error::Error::new(firefly_error::ErrorKind::Internal, error.to_string())
                .with_context("operation", "open sensor publishers")
        })??;
        Ok(pipeline)
    }
}

impl Drop for CapturePipeline {
    fn drop(&mut self) {
        self.jobs.close();
        if let Some(worker) = self.worker.take()
            && worker.join().is_err()
        {
            log::error!("sensor-process 线程 panic");
        }
    }
}

/// 位姿轮询 + 出图节拍：排空真值保留最新 → 按固定节拍发起回读并按需激活相机。
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
pub fn poll_pose(
    ports: NonSend<IpcPorts>,
    mut pose: ResMut<PoseState>,
    mut viewer: ResMut<crate::viewer_pose::ViewerPose>,
    mut overlay: ResMut<crate::flight_overlay::FlightOverlay>,
    time: Res<Time>,
    wall: Res<Time<Real>>,
    hub: Res<CaptureHub>,
    targets: Option<Res<crate::rig::SensorTargets>>,
    mut capture: ResMut<SensorCapture>,
    mut pending: ResMut<PendingFrames>,
    mut eyes: Query<(&Eye, &mut Transform)>,
    mut cameras: Query<&mut Camera, With<Eye>>,
) {
    let mut newest: Option<(OdomMessage, TraceContext)> = None;
    loop {
        match ports.pose_sub.receive() {
            Ok(Some(sample)) => {
                if ports.corrected_sub.is_some() {
                    overlay.observe_actual(&sample, wall.elapsed_secs_f64());
                }
                newest = Some((*sample, *sample.user_header()));
            }
            Ok(None) => break,
            Err(e) => {
                log::warn!("真值接收失败：{e:?}");
                break;
            }
        }
    }
    if let Some((msg, _)) = newest
        && msg.is_initialized
    {
        viewer.observe(
            Transform {
                translation: Vec3::new(
                    msg.position_x as f32,
                    msg.position_y as f32,
                    msg.position_z as f32,
                ),
                rotation: Quat::from_xyzw(
                    msg.quat_x as f32,
                    msg.quat_y as f32,
                    msg.quat_z as f32,
                    msg.quat_w as f32,
                ),
                ..default()
            },
            msg.timestamp,
            time.elapsed_secs_f64(),
        );
    }
    if let Some((msg, header)) = newest
        && msg.is_initialized
        && !hub.has_pending()
    {
        pose.pos = Vec3::new(
            msg.position_x as f32,
            msg.position_y as f32,
            msg.position_z as f32,
        );
        pose.quat = Quat::from_xyzw(
            msg.quat_x as f32,
            msg.quat_y as f32,
            msg.quat_z as f32,
            msg.quat_w as f32,
        );
        pose.stamp = msg.timestamp;
        pose.trace = header;
        pose.has_pose = true;
    }

    // 渲染世界已取走本窗口的请求（拷贝已编码）→ 收起传感器相机，停止空转。
    if capture.active && hub.taken_seq() >= capture.requested_seq {
        capture.active = false;
    }

    // 按仿真时间以 10Hz 触发采集；回读双缓冲，
    // 只要还有空闲组就可发起（上一拍在途不阻塞下一拍）。
    if !capture.active
        && hub.can_issue()
        && !hub.has_pending()
        && pose.has_pose
        && pose.stamp >= capture.next_stamp
        && let Some(targets) = targets
    {
        let seq = pending.next_seq;
        pending.next_seq += 1;
        hub.request(CaptureRequest {
            seq,
            timing: crate::capture::CaptureTiming::default(),
            stamp: pose.stamp,
            trace: pose.trace,
            left: targets.left.id(),
            right: targets.right.id(),
        });
        capture.active = true;
        capture.requested_seq = seq;
        capture.next_stamp = pose.stamp + CAPTURE_PERIOD - CAPTURE_TOLERANCE;
        log::debug!("pose t={:.3} 回读 seq={seq}", pose.stamp);
    }

    // rig 摆位与激活同系统内完成：本帧渲染即用新变换、且只在出图窗口渲染。
    apply_pose_to_eyes(&pose, &mut eyes);
    let active = capture.active;
    for mut camera in &mut cameras {
        if camera.is_active != active {
            camera.is_active = active;
        }
    }
}

/// 回读装配：三份同 `seq` 到齐 → 交给工作线程（不阻塞主线程逐像素计算）。
#[allow(clippy::needless_pass_by_value)]
pub fn drain_captures(
    hub: Res<CaptureHub>,
    mut pending: ResMut<PendingFrames>,
    pipeline: Res<CapturePipeline>,
) {
    for frame in hub.drain() {
        pending.frames.entry(frame.seq).or_default().push(frame);
    }
    let ready: Vec<u64> = pending
        .frames
        .iter()
        .filter(|(_, v)| v.len() >= 3)
        .map(|(k, _)| *k)
        .collect();
    if ready.is_empty() {
        return;
    }
    // 只处理最新完整节拍，旧节拍丢弃（渲染追最新，积压无意义）。
    let seq = ready.into_iter().max().expect("non-empty");
    let frames = pending.frames.remove(&seq).unwrap_or_default();
    pending.frames.clear();
    let mut left_rgb = None;
    let mut right_rgb = None;
    let mut depth_raw = None;
    let mut stamp = 0.0;
    let mut trace = TraceContext::empty();
    let mut timing = frames[0].timing;
    for frame in frames {
        timing.readback = timing.readback.max(frame.timing.readback);
        stamp = frame.stamp;
        trace = frame.trace;
        match frame.kind {
            SensorKind::Left => left_rgb = Some(frame.bytes),
            SensorKind::Right => right_rgb = Some(frame.bytes),
            SensorKind::Depth => depth_raw = Some(frame.bytes),
        }
    }
    let (Some(left_rgb), Some(right_rgb), Some(depth_raw)) = (left_rgb, right_rgb, depth_raw)
    else {
        log::warn!("seq={seq} 回复不全（同类重复），丢弃本节拍");
        return;
    };

    timing.assembled = Instant::now();
    let job = CaptureJob {
        seq,
        timing,
        stamp,
        trace,
        left_rgb,
        right_rgb,
        depth_raw,
    };
    pipeline.jobs.replace(job);
}

/// UI 只取最新显示副本；发布已由工作线程完成。
#[allow(clippy::needless_pass_by_value)]
pub fn update_processed_views(
    pipeline: Res<CapturePipeline>,
    mut stats: ResMut<CaptureStats>,
    mut views: ResMut<DebugViews>,
    mut images: ResMut<Assets<Image>>,
    mut keypoints: ResMut<crate::keypoints::KeypointOverlay>,
    mut exits: MessageWriter<AppExit>,
) {
    if pipeline
        .worker
        .as_ref()
        .is_some_and(JoinHandle::is_finished)
    {
        log::error!("sensor-process 工作线程意外退出");
        exits.write(AppExit::error());
    }
    stats.published += pipeline.published.swap(0, Ordering::Relaxed);
    if let Some(result) = pipeline.results.take() {
        stats.last_stamp = result.stamp;
        update_views(&mut views, &mut images, &mut keypoints, result);
    }
}

/// 发布一拍：续接 trace → 左/右灰度 + 深度 + 相机对事件。
fn publish(ports: &SensorPublishers, result: &ProcessedCapture) {
    // 帧 trace：续接 sim 相机周期的 span，无上游时用未采样 root（对照 vio）。
    let root = result
        .trace
        .continue_span("render-sensors")
        .unwrap_or_else(|| Span::root("render-sensors", SpanContext::random().sampled(false)));
    let _guard = root.set_local_parent();

    publish_gray(&ports.left_pub, &result.left_gray, result.stamp, 0);
    publish_gray(&ports.right_pub, &result.right_gray, result.stamp, 1);
    publish_depth(&ports.depth_pub, &result.depth, result.stamp);
    ports.pair_notify.notify_sent_sample();
    let t = result.timing;
    for (entity, values) in [
        (
            "render/capture/stages_s",
            [
                t.submitted.duration_since(t.issued).as_secs_f64(),
                t.readback.duration_since(t.submitted).as_secs_f64(),
                t.assembled.duration_since(t.readback).as_secs_f64(),
                t.started.duration_since(t.assembled).as_secs_f64(),
            ],
        ),
        (
            "render/capture/publish_s",
            [
                t.processed.duration_since(t.started).as_secs_f64(),
                t.processed.elapsed().as_secs_f64(),
                t.issued.elapsed().as_secs_f64(),
                result.seq as f64,
            ],
        ),
    ] {
        let mut message = VizMessage::base(kind::SCALARS, result.stamp, entity);
        message.scalars = values;
        message.scalar_count = 4;
        let _ = ports.viz_pub.publish(message);
    }
    log::debug!(
        "发布 seq={} t={:.3}（左/右/深度 + pair 事件）",
        result.seq,
        result.stamp
    );
}

/// 灰度发布（行主序，`sensor_id` 左右目区分）。
#[allow(clippy::large_stack_arrays)]
fn publish_gray(publisher: &Publisher<GrayImageMessage>, gray: &[u8], stamp: f64, sensor_id: i32) {
    let mut msg = GrayImageMessage {
        timestamp: stamp,
        sensor_id,
        width: IMAGE_WIDTH as u32,
        height: IMAGE_HEIGHT as u32,
        data: [0u8; IMAGE_SIZE],
    };
    msg.data.copy_from_slice(gray);
    if let Err(e) = publisher.publish(msg) {
        log::warn!("灰度发布失败（id={sensor_id}）：{e:?}");
    }
}

/// 深度发布（米制 f32，行主序；噪声已在工作线程加入）。
#[allow(clippy::large_stack_arrays)]
fn publish_depth(publisher: &Publisher<DepthImageMessage>, depth: &[f32], stamp: f64) {
    let mut msg = DepthImageMessage {
        timestamp: stamp,
        sensor_id: 0,
        width: IMAGE_WIDTH as u32,
        height: IMAGE_HEIGHT as u32,
        data: [0.0; IMAGE_SIZE],
    };
    msg.data.copy_from_slice(depth);
    if let Err(e) = publisher.publish(msg) {
        log::warn!("深度发布失败：{e:?}");
    }
}

/// RGB/深度显示图直接搬入 `Assets<Image>`；灰度原图交给关键点显示缓存。
fn update_views(
    views: &mut DebugViews,
    images: &mut Assets<Image>,
    keypoints: &mut crate::keypoints::KeypointOverlay,
    result: ProcessedCapture,
) {
    keypoints.push_frame(result.stamp, [result.left_gray, result.right_gray]);
    if let Some(mut image) = images.get_mut(&views.left_rgb) {
        image.data = Some(result.left_rgb);
    }
    if let Some(mut image) = images.get_mut(&views.right_rgb) {
        image.data = Some(result.right_rgb);
    }
    if let Some(mut image) = images.get_mut(&views.depth) {
        image.data = Some(result.depth_rgba);
    }
}
