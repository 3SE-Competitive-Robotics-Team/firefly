//! IPC 接线：真值订阅 → 回读请求 → 图像发布 + 相机对事件。
//!
//! 时序（对照 `firefly-sim/main.py` 的 10Hz 相机节拍）：新真值到达即摆 rig
//! 相机并发起回读；三份同 `seq` 回复到齐后，以位姿时间戳发布左/右灰度 +
//! 深度，随后单次通知 `Firefly/CameraPair`（VIO 事件驱动即到即醒）。
//! 发布走 [`Publisher::publish`]，自动注入续接后的 trace 上下文。

use std::collections::HashMap;

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
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::capture::{CaptureHub, CaptureRequest, CapturedFrame, SensorKind};
use crate::rig::{PoseState, SENSOR_NEAR, apply_pose_to_eyes};
use crate::sensors;
use crate::ui::DebugViews;

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
#[derive(Resource, Default)]
pub struct PendingFrames {
    /// `seq` → 已到回复。
    pub frames: HashMap<u64, Vec<CapturedFrame>>,
    /// 下一个回读序号。
    pub next_seq: u64,
    /// 深度噪声随机源。
    pub rng: Option<StdRng>,
}

/// 位姿轮询：排空真值保留最新 → 更新位姿/rig → 发起回读。
#[allow(clippy::needless_pass_by_value)]
pub fn poll_pose(
    ports: NonSend<IpcPorts>,
    mut pose: ResMut<PoseState>,
    hub: Res<CaptureHub>,
    targets: Option<Res<crate::rig::SensorTargets>>,
    mut pending: ResMut<PendingFrames>,
    mut eyes: Query<(&crate::rig::Eye, &mut Transform)>,
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
    let Some((msg, header)) = newest else { return };
    if !msg.is_initialized {
        return;
    }
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
    pose.has_pose = true;

    // rig 摆位与请求同系统内完成：本帧渲染即用新变换出图。
    apply_pose_to_eyes(&pose, &mut eyes);
    let Some(targets) = targets else { return };
    let seq = pending.next_seq;
    pending.next_seq += 1;
    hub.request(CaptureRequest {
        seq,
        stamp: msg.timestamp,
        trace: header,
        left: targets.left.id(),
        right: targets.right.id(),
    });
    log::debug!("pose t={:.3} 回读 seq={seq}", msg.timestamp);
}

/// 回读装配：三份同 `seq` 到齐 → 灰度/噪声/发布/事件/调试图一次完成。
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
pub fn drain_captures(
    ports: NonSend<IpcPorts>,
    hub: Res<CaptureHub>,
    mut pending: ResMut<PendingFrames>,
    mut views: ResMut<DebugViews>,
    mut images: ResMut<Assets<Image>>,
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
    // 只发布最新完整节拍，旧节拍丢弃（渲染追最新，积压无意义）。
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

    // 帧 trace：续接 sim 相机周期的 span，无上游时用未采样 root（对照 vio）。
    let root = trace
        .continue_span("render-sensors")
        .unwrap_or_else(|| Span::root("render-sensors", SpanContext::random().sampled(false)));
    let _guard = root.set_local_parent();

    let rng = pending
        .rng
        .get_or_insert_with(|| StdRng::from_rng(&mut rand::rng()));
    let mut left_gray = vec![0u8; IMAGE_SIZE];
    let mut right_gray = vec![0u8; IMAGE_SIZE];
    sensors::rgb_to_gray(&left_rgb, &mut left_gray);
    sensors::rgb_to_gray(&right_rgb, &mut right_gray);
    let mut depth = vec![0.0f32; IMAGE_SIZE];
    sensors::linearize_depth(&depth_raw, &mut depth, SENSOR_NEAR);
    sensors::add_depth_noise(
        &mut depth,
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        crate::rig::FOV_Y_DEG.to_radians(),
        rng,
    );

    publish_gray(&ports.left_pub, &left_gray, stamp, 0);
    publish_gray(&ports.right_pub, &right_gray, stamp, 1);
    publish_depth(&ports.depth_pub, &depth, stamp);
    ports.pair_notify.notify_sent_sample();
    log::debug!("发布 seq={seq} t={stamp:.3}（左/右/深度 + pair 事件）");

    // 调试图更新（显示与发布同源，CPU 侧二次转换）。
    update_views(
        &mut views,
        &mut images,
        &left_rgb,
        &right_rgb,
        &left_gray,
        &right_gray,
        &depth,
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

/// 深度发布（米制 f32，行主序；噪声已在调用方加入）。
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

/// 调试图更新（左/右 RGB 直显，灰度与深度经显示映射）。
#[allow(clippy::too_many_arguments)]
fn update_views(
    views: &mut DebugViews,
    images: &mut Assets<Image>,
    left_rgb: &[u8],
    right_rgb: &[u8],
    left_gray: &[u8],
    right_gray: &[u8],
    depth: &[f32],
) {
    if let Some(mut image) = images.get_mut(&views.left_rgb) {
        image.data = Some(left_rgb.to_vec());
    }
    if let Some(mut image) = images.get_mut(&views.right_rgb) {
        image.data = Some(right_rgb.to_vec());
    }
    for (handle, gray) in [
        (&views.left_gray, left_gray),
        (&views.right_gray, right_gray),
    ] {
        if let Some(mut image) = images.get_mut(handle) {
            let mut rgba = vec![0u8; 4 * IMAGE_SIZE];
            sensors::gray_to_display(gray, &mut rgba);
            image.data = Some(rgba);
        }
    }
    if let Some(mut image) = images.get_mut(&views.depth) {
        let mut rgba = vec![0u8; 4 * IMAGE_SIZE];
        sensors::depth_to_display(depth, &mut rgba, sensors::DISPLAY_DEPTH_RANGE);
        image.data = Some(rgba);
    }
}
