//! IPC 接线：真值订阅 → 固定 10Hz 回读 → 工作线程逐像素处理 → 图像发布。
//!
//! 时序（对照 `firefly-sim/main.py` 的 10Hz 相机节拍）：
//! - `poll_pose` 排空真值更新位姿，按**固定 10Hz 节拍**（`CAPTURE_PERIOD`，
//!   以 sim 时间为轴）发起回读，并激活传感器相机；回读被渲染世界取走后收起
//!   相机，限制传感器渲染只在有采集请求时执行。
//! - 三份同 `seq` 回复到齐后装配成 [`CaptureJob`] 交给工作线程；工作线程算完
//!   立即发布传感器，不向主线程回传像素缓冲。
//! - 图像走 [`Publisher::publish_with`]，就地填充并注入续接后的 trace 上下文。

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
    IMAGE_HEIGHT, IMAGE_WIDTH,
};
use firefly_pubsub::event::{CAMERA_PAIR_TOPIC, TopicNotifier};
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::odom::{GROUND_TRUTH_TOPIC, OdomMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::trace::TraceContext;
use firefly_pubsub::viz::{ScalarMessage, VizPublisher};

use crate::capture::{CaptureHub, CaptureReply, CaptureRequest, CapturedFrame, SensorKind};
use crate::process::{CaptureJob, Latest, ProcessedCapture, run_worker};
use crate::rig::{Eye, PoseState, apply_pose_to_eyes};

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
    log::info!("IPC 就绪：订阅位姿，offline={offline}");
    Ok(IpcPorts { pose_sub, node })
}

/// 传感器发布端口只在像素工作线程构造和使用，节点最后释放。
struct SensorPublishers {
    left_pub: Publisher<GrayImageMessage>,
    right_pub: Publisher<GrayImageMessage>,
    depth_pub: Publisher<DepthImageMessage>,
    pair_notify: TopicNotifier,
    viz_pub: VizPublisher,
    calibration_stamp: std::cell::Cell<f64>,
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
            calibration_stamp: std::cell::Cell::new(f64::NEG_INFINITY),
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
    /// 已完成或失败的最新序号；迟到片段必须丢弃并归还缓冲。
    retired_seq: u64,
}

impl Default for PendingFrames {
    fn default() -> Self {
        // 序号从 1 起：`HubInner::taken_seq` 初值 0 不能被误判为「本拍已被取走」。
        Self {
            frames: HashMap::new(),
            next_seq: 1,
            retired_seq: 0,
        }
    }
}

/// 出图统计（诊断用：区分应用帧率与真实供图速率）。
#[derive(Resource, Default)]
pub struct CaptureStats {
    /// 本统计周期内已发布的拍数。
    pub published: u32,
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

/// 逐像素处理与发布；待处理任务只保留最新一拍。
#[derive(Resource)]
pub struct CapturePipeline {
    jobs: Arc<Latest<CaptureJob>>,
    published: Arc<AtomicU32>,
    worker: Option<JoinHandle<()>>,
}

impl CapturePipeline {
    pub fn spawn(
        options: crate::depth_noise::DepthNoiseOptions,
        offline: bool,
    ) -> Result<Self, firefly_error::Error> {
        let jobs = Arc::new(Latest::<CaptureJob>::default());
        let published = Arc::new(AtomicU32::new(0));
        let (input, count) = (jobs.clone(), published.clone());
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
                run_worker(&input, &options, offline, |result| {
                    publish(&ports, result);
                    count.fetch_add(1, Ordering::Relaxed);
                });
            })
            .map_err(|error| {
                firefly_error::Error::new(firefly_error::ErrorKind::Internal, error.to_string())
                    .with_operation("spawn sensor processor")
            })?;
        let pipeline = Self {
            jobs,
            published,
            worker: Some(worker),
        };
        ready_rx.recv().map_err(|error| {
            firefly_error::Error::new(firefly_error::ErrorKind::Internal, error.to_string())
                .with_operation("open sensor publishers")
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
    if let Some(job) = pending.assemble(hub.drain()) {
        pipeline.jobs.replace(job);
    }
}

impl PendingFrames {
    fn retire(&mut self, seq: u64) {
        self.retired_seq = self.retired_seq.max(seq);
        self.frames.retain(|id, _| *id > self.retired_seq);
    }

    /// 保留较新的未完整拍；淘汰序号的迟到片段不得再次占用缓冲。
    fn assemble(&mut self, replies: impl IntoIterator<Item = CaptureReply>) -> Option<CaptureJob> {
        for reply in replies {
            match reply {
                CaptureReply::Frame(frame) if frame.seq > self.retired_seq => {
                    self.frames.entry(frame.seq).or_default().push(frame);
                }
                CaptureReply::Failed(seq) => self.retire(seq),
                CaptureReply::Frame(_) => {}
            }
        }
        let seq = self
            .frames
            .iter()
            .filter(|(_, frames)| frames.len() >= 3)
            .map(|(seq, _)| *seq)
            .max()?;
        let frames = self.frames.remove(&seq).expect("complete frame");
        self.retire(seq);
        let mut left_gray = None;
        let mut right_gray = None;
        let mut depth_raw = None;
        let mut stamp = 0.0;
        let mut trace = TraceContext::default();
        let mut timing = frames[0].timing;
        for frame in frames {
            timing.readback = timing.readback.max(frame.timing.readback);
            stamp = frame.stamp;
            trace = frame.trace;
            match frame.kind {
                SensorKind::Left => left_gray = Some(frame.bytes),
                SensorKind::Right => right_gray = Some(frame.bytes),
                SensorKind::Depth => depth_raw = Some(frame.bytes),
            }
        }
        let (Some(left_gray), Some(right_gray), Some(depth_raw)) =
            (left_gray, right_gray, depth_raw)
        else {
            log::warn!("seq={seq} 回复不全（同类重复），丢弃本节拍");
            return None;
        };

        timing.assembled = Instant::now();
        Some(CaptureJob {
            seq,
            timing,
            stamp,
            trace,
            left_gray,
            right_gray,
            depth_raw,
        })
    }
}

/// 汇总发布计数，工作线程提前退出视为传感器故障。
#[allow(clippy::needless_pass_by_value)]
pub fn poll_processor(
    pipeline: Res<CapturePipeline>,
    mut stats: ResMut<CaptureStats>,
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
}

/// 发布一拍：续接 trace → 左/右灰度 + 深度 + 相机对事件。
fn publish(ports: &SensorPublishers, result: &ProcessedCapture) {
    // 帧 trace：续接 sim 相机周期的 span，无上游时用未采样 root（对照 vio）。
    let root = result
        .trace
        .continue_span("render-sensors")
        .unwrap_or_else(|| Span::root("render-sensors", SpanContext::random().sampled(false)));
    let _guard = root.set_local_parent();

    if result.stamp - ports.calibration_stamp.get() >= 1.0 {
        let _ = ports
            .viz_pub
            .publish(crate::rig::left_camera_calibration(result.stamp));
        ports.calibration_stamp.set(result.stamp);
    }
    publish_gray(&ports.left_pub, &result.left_gray, result.stamp, 0);
    publish_gray(&ports.right_pub, &result.right_gray, result.stamp, 1);
    publish_depth(&ports.depth_pub, result.depth, result.stamp);
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
            ]
            .as_slice(),
        ),
        (
            "render/capture/publish_s",
            [
                t.processed.duration_since(t.started).as_secs_f64(),
                t.processed.elapsed().as_secs_f64(),
                t.issued.elapsed().as_secs_f64(),
            ]
            .as_slice(),
        ),
    ] {
        let mut message = ScalarMessage::base(result.stamp, entity);
        message.scalars[..values.len()].copy_from_slice(values);
        message.scalar_count = values.len() as u32;
        let _ = ports.viz_pub.publish(message);
    }
    log::debug!(
        "发布 seq={} t={:.3}（左/右/深度 + pair 事件）",
        result.seq,
        result.stamp
    );
}

/// 灰度发布（行主序，`sensor_id` 左右目区分）。
fn publish_gray(publisher: &Publisher<GrayImageMessage>, gray: &[u8], stamp: f64, sensor_id: i32) {
    if let Err(e) = publisher.publish_with(|msg| {
        msg.timestamp = stamp;
        msg.sensor_id = sensor_id;
        msg.width = IMAGE_WIDTH as u32;
        msg.height = IMAGE_HEIGHT as u32;
        msg.data.copy_from_slice(gray);
    }) {
        log::warn!("灰度发布失败（id={sensor_id}）：{e:?}");
    }
}

/// 深度发布（米制 f32，行主序；噪声已在工作线程加入）。
fn publish_depth(publisher: &Publisher<DepthImageMessage>, depth: &[f32], stamp: f64) {
    if let Err(e) = publisher.publish_with(|msg| {
        msg.timestamp = stamp;
        msg.sensor_id = 0;
        msg.width = IMAGE_WIDTH as u32;
        msg.height = IMAGE_HEIGHT as u32;
        msg.data.copy_from_slice(depth);
    }) {
        log::warn!("深度发布失败：{e:?}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::buffer::BufferPool;
    use crate::capture::{CaptureTiming, FRAME_BYTES, GRAY_BYTES};

    struct Replies {
        left: BufferPool,
        right: BufferPool,
        depth: BufferPool,
    }

    impl Replies {
        fn new() -> Self {
            Self {
                left: BufferPool::new(2, GRAY_BYTES),
                right: BufferPool::new(2, GRAY_BYTES),
                depth: BufferPool::new(2, FRAME_BYTES),
            }
        }

        fn frame(&self, seq: u64, kind: SensorKind) -> CaptureReply {
            let pool = match kind {
                SensorKind::Left => &self.left,
                SensorKind::Right => &self.right,
                SensorKind::Depth => &self.depth,
            };
            CaptureReply::Frame(CapturedFrame {
                seq,
                timing: CaptureTiming::default(),
                kind,
                stamp: seq as f64,
                trace: TraceContext::default(),
                bytes: pool.take().unwrap(),
            })
        }

        fn complete(&self, seq: u64) -> [CaptureReply; 3] {
            [
                self.frame(seq, SensorKind::Left),
                self.frame(seq, SensorKind::Right),
                self.frame(seq, SensorKind::Depth),
            ]
        }
    }

    #[test]
    fn newer_partial_capture_survives_older_complete_capture() {
        let replies = Replies::new();
        let mut pending = PendingFrames::default();
        assert!(
            pending
                .assemble([replies.frame(2, SensorKind::Left)])
                .is_none()
        );
        let first = pending.assemble(replies.complete(1)).unwrap();
        assert_eq!(first.seq, 1);
        drop(first);
        let second = pending
            .assemble([
                replies.frame(2, SensorKind::Depth),
                replies.frame(2, SensorKind::Right),
            ])
            .unwrap();
        assert_eq!(second.seq, 2);
        assert!((second.stamp - 2.).abs() < f64::EPSILON);
        assert!(pending.frames.is_empty());
    }

    #[test]
    fn late_and_failed_fragments_release_buffers_for_future_captures() {
        let replies = Replies::new();
        let mut pending = PendingFrames::default();
        assert!(
            pending
                .assemble([replies.frame(1, SensorKind::Left)])
                .is_none()
        );
        let newer = pending.assemble(replies.complete(2)).unwrap();
        drop(newer);
        assert!(
            pending
                .assemble([replies.frame(1, SensorKind::Depth)])
                .is_none()
        );
        assert!(pending.frames.is_empty());
        assert!(
            pending
                .assemble([
                    replies.frame(3, SensorKind::Left),
                    replies.frame(3, SensorKind::Right),
                    CaptureReply::Failed(3)
                ])
                .is_none()
        );
        assert!(
            pending
                .assemble([replies.frame(3, SensorKind::Depth)])
                .is_none()
        );
        assert!(pending.frames.is_empty());
        for seq in 4..=6 {
            let job = pending.assemble(replies.complete(seq)).unwrap();
            assert_eq!(job.seq, seq);
            drop(job);
        }
    }
}
