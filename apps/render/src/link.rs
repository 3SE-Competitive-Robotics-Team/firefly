//! IPC 接线：真值订阅 → 固定 10Hz 回读 → 工作线程逐像素处理 → 图像发布。
//!
//! 时序（对照 `firefly-sim/main.py` 的 10Hz 相机节拍）：
//! - `poll_pose` 排空真值更新位姿，按**固定 10Hz 节拍**（`CAPTURE_PERIOD`，
//!   以 sim 时间为轴）发起回读，并激活传感器相机；回读被渲染世界取走后收起
//!   相机（按需渲染，不再每帧空转）。
//! - 三份同 `seq` 回复到齐后装配成 [`CaptureJob`] 交给工作线程；工作线程算完
//!   回 [`ProcessedCapture`]，主线程只做发布与调试显示写入（端口主线程独占）。
//! - 发布走 [`Publisher::publish`]，自动注入续接后的 trace 上下文。

use std::collections::HashMap;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, SyncSender};

use bevy::ecs::system::NonSend;
use bevy::prelude::*;
use fastrace::prelude::*;
use firefly_pubsub::camera::{
    CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage,
    IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH,
};
use firefly_pubsub::event::{CAMERA_PAIR_TOPIC, TopicNotifier};
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::odom::{GROUND_TRUTH_TOPIC, OdomMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::trace::TraceContext;

use crate::capture::{CaptureHub, CaptureRequest, CapturedFrame, SensorKind};
use crate::process::{CaptureJob, ProcessedCapture, run_worker};
use crate::rig::{Eye, PoseState, apply_pose_to_eyes};
use crate::ui::DebugViews;

/// 出图节拍（秒，sim 时间轴）：固定 10Hz，与真值到达边沿解耦。
pub const CAPTURE_PERIOD: f64 = 0.1;

/// 节拍比较容差（秒）：sim 时间戳有浮点抖动，`next_stamp` 精确相等比较会
/// 偶发漏拍（漏一拍即掉到 5Hz）。取半个周期的 10%。
const CAPTURE_TOLERANCE: f64 = CAPTURE_PERIOD * 0.1;

/// IPC 端口束（节点最后声明最后释放，对照 `firefly-pubsub/node` 的 Drop 纪律）。
///
/// iceoryx2 端口是单线程的（`Rc` 内核，非 `Send/Sync`），只能以
/// [`NonSend`](bevy::ecs::system::NonSend) 资源持有——触碰端口的系统
/// 自动约束到主线程，构造上不可能跨线程触碰。
pub struct IpcPorts {
    /// 位姿订阅（`Firefly/GroundTruth`，只保留最新）。
    pub pose_sub: Subscriber<OdomMessage>,
    /// 左目灰度发布。
    pub left_pub: Publisher<GrayImageMessage>,
    /// 右目灰度发布。
    pub right_pub: Publisher<GrayImageMessage>,
    /// 深度发布。
    pub depth_pub: Publisher<DepthImageMessage>,
    /// 相机对事件通知（左右目成对发布后单次唤醒）。
    pub pair_notify: TopicNotifier,
    /// 进程共享节点（最后释放：仅持有以延续生命周期，不直接调用）。
    #[allow(dead_code)]
    pub node: IpcNode,
}

/// 打开全部 IPC 端口。
///
/// # Errors
/// 节点/端口创建失败（IPC 资源不可用，见 `firefly-pubsub` 各构造器）。
pub fn open_ports() -> Result<IpcPorts, firefly_error::Error> {
    let node = create_node()?;
    let pose_sub = Subscriber::with_topic(&node, GROUND_TRUTH_TOPIC)?;
    let left_pub = Publisher::with_topic(&node, CAMERA_LEFT_TOPIC)?;
    let right_pub = Publisher::with_topic(&node, CAMERA_RIGHT_TOPIC)?;
    let depth_pub = Publisher::with_topic(&node, DEPTH_TOPIC)?;
    let pair_notify = TopicNotifier::with_topic(&node, CAMERA_PAIR_TOPIC)?;
    log::info!("IPC 就绪：订阅真值，发布双目/深度（`--no-camera` 的 sim 配合）");
    Ok(IpcPorts {
        pose_sub,
        left_pub,
        right_pub,
        depth_pub,
        pair_notify,
        node,
    })
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

/// 逐像素处理流水（工作线程 + 单槽背压：算不过来时只留最新一拍）。
#[derive(Resource)]
pub struct CapturePipeline {
    /// 任务通道（工作线程消费）。
    tx: Mutex<SyncSender<CaptureJob>>,
    /// 结果通道（主线程消费）。
    rx: Mutex<Receiver<ProcessedCapture>>,
    /// 是否有任务在算（主线程状态）。
    busy: bool,
    /// 在算期间到达的最新一拍（旧的直接丢）。
    pending: Option<CaptureJob>,
}

impl CapturePipeline {
    /// 起工作线程（进程生命周期内常驻，退出时通道关闭自动结束）。
    #[must_use]
    pub fn spawn() -> Self {
        let (tx, job_rx) = std::sync::mpsc::sync_channel::<CaptureJob>(1);
        let (res_tx, rx) = std::sync::mpsc::sync_channel::<ProcessedCapture>(1);
        std::thread::Builder::new()
            .name("sensor-process".to_owned())
            .spawn(move || run_worker(&job_rx, &res_tx))
            .expect("sensor-process 线程创建失败");
        Self {
            tx: Mutex::new(tx),
            rx: Mutex::new(rx),
            busy: false,
            pending: None,
        }
    }

    /// 投递一拍（非阻塞；通道满即丢，背压由 `busy`/`pending` 单槽承担）。
    fn submit(&self, job: CaptureJob) {
        let _ = self.tx.lock().expect("pipeline tx").try_send(job);
    }

    /// 取回已完成的一拍（非阻塞）。
    fn try_recv(&self) -> Option<ProcessedCapture> {
        self.rx.lock().expect("pipeline rx").try_recv().ok()
    }
}

/// 位姿轮询 + 出图节拍：排空真值保留最新 → 按固定节拍发起回读并按需激活相机。
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
pub fn poll_pose(
    ports: NonSend<IpcPorts>,
    mut pose: ResMut<PoseState>,
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
                newest = Some((*sample, *sample.user_header()));
            }
            Ok(None) => break,
            Err(e) => {
                log::warn!("真值接收失败：{e:?}");
                break;
            }
        }
    }
    if let Some((msg, header)) = newest
        && msg.is_initialized
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

    // 固定 10Hz 节拍触发：以 sim 时间为轴，真值边沿抖动不再丢帧；回读双缓冲，
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
    mut pipeline: ResMut<CapturePipeline>,
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
    for frame in frames {
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

    let job = CaptureJob {
        seq,
        stamp,
        trace,
        left_rgb,
        right_rgb,
        depth_raw,
    };
    if pipeline.busy {
        pipeline.pending = Some(job);
    } else {
        pipeline.submit(job);
        pipeline.busy = true;
    }
}

/// 工作线程结果回收：发布 + 调试显示写入（主线程只做端口触碰与内存搬运）。
#[allow(clippy::needless_pass_by_value)]
pub fn publish_processed(
    ports: NonSend<IpcPorts>,
    mut pipeline: ResMut<CapturePipeline>,
    mut stats: ResMut<CaptureStats>,
    mut views: ResMut<DebugViews>,
    mut images: ResMut<Assets<Image>>,
) {
    while let Some(result) = pipeline.try_recv() {
        pipeline.busy = false;
        stats.published += 1;
        stats.last_stamp = result.stamp;
        publish(&ports, &result);
        update_views(&mut views, &mut images, result);
        if let Some(next) = pipeline.pending.take() {
            pipeline.submit(next);
            pipeline.busy = true;
        }
    }
}

/// 发布一拍：续接 trace → 左/右灰度 + 深度 + 相机对事件。
fn publish(ports: &IpcPorts, result: &ProcessedCapture) {
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

/// 调试图更新（工作线程算好的 RGBA 直接搬入 `Assets<Image>`，无二次转换）。
fn update_views(views: &mut DebugViews, images: &mut Assets<Image>, result: ProcessedCapture) {
    if let Some(mut image) = images.get_mut(&views.left_rgb) {
        image.data = Some(result.left_rgb);
    }
    if let Some(mut image) = images.get_mut(&views.right_rgb) {
        image.data = Some(result.right_rgb);
    }
    if let Some(mut image) = images.get_mut(&views.left_gray) {
        image.data = Some(result.left_gray_rgba);
    }
    if let Some(mut image) = images.get_mut(&views.right_gray) {
        image.data = Some(result.right_gray_rgba);
    }
    if let Some(mut image) = images.get_mut(&views.depth) {
        image.data = Some(result.depth_rgba);
    }
}
