//! 传感器输入源：iceoryx2 物理环境（`MuJoCo`）。
//!
//! 订阅 IMU / 左目灰度 / 深度三路话题，按时间戳把深度帧与相机帧配对成
//! [`crate::FrameInput`]（相机时刻为准，10Hz 对齐，容忍 20ms 偏差）。
//! trace 上下文随消息 User Header 携带（见 [`firefly_pubsub::trace`]）。

use std::collections::VecDeque;

use firefly_pubsub::camera::{CAMERA_LEFT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage};
use firefly_pubsub::imu::ImuSubscriber;
use firefly_pubsub::node::IpcNode;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::trace::TraceContext;
use firefly_void_types::sensor::ImuSample;
use nalgebra::Vector3;

/// 深度帧与相机帧的时间戳配对容差（秒；sim 同周期发布，10Hz 帧间隔 0.1s）。
const PAIR_TOLERANCE: f64 = 0.02;

/// 已配对的左目灰度帧（自有数据，供主循环构造借用）。
#[derive(Debug)]
pub struct CameraData {
    /// 时间戳（仿真秒）。
    pub t: f64,
    /// 左目灰度（行主序）。
    pub left_gray: Vec<u8>,
    pub width: usize,
    pub height: usize,
}

/// 已配对的深度帧（自有数据，f32 → f64）。
#[derive(Debug)]
pub struct DepthData {
    /// 时间戳（仿真秒）。
    pub t: f64,
    /// 深度值（米，行主序，0 为空洞）。
    pub depth: Vec<f64>,
    pub width: usize,
    pub height: usize,
}

/// iceoryx2 物理环境输入源。
pub struct IceoryxInput {
    /// IMU 订阅（`Firefly/Imu`）。
    imu_sub: ImuSubscriber,
    /// 左目灰度订阅（`Firefly/CameraLeft`）。
    left_sub: Subscriber<GrayImageMessage>,
    /// 深度订阅（`Firefly/Depth`）。
    depth_sub: Subscriber<DepthImageMessage>,
    /// 待消费 IMU 队列。
    imu_queue: VecDeque<ImuSample>,
    /// 最近左目帧（等待与深度配对）。
    last_left: Option<CameraData>,
    /// 最近深度帧（等待与左目配对）。
    last_depth: Option<DepthData>,
    /// 最新收到数据的时刻（秒）。
    now_t: f64,
    /// 最近收到消息携带的 trace 上下文 `(trace_id, span_id, sampled)`。
    last_trace: Option<(u128, u64, bool)>,
}

impl IceoryxInput {
    /// 打开订阅（话题不存在时自动创建空服务，等待发布端）。
    ///
    /// # Errors
    /// iceoryx2 订阅创建失败。
    pub fn new(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Ok(Self {
            imu_sub: ImuSubscriber::new(node)?,
            left_sub: Subscriber::with_topic(node, CAMERA_LEFT_TOPIC)?,
            depth_sub: Subscriber::with_topic(node, DEPTH_TOPIC)?,
            imu_queue: VecDeque::new(),
            last_left: None,
            last_depth: None,
            now_t: 0.0,
            last_trace: None,
        })
    }

    /// 记录一条消息携带的 trace 上下文（若有）。
    fn capture_trace(&mut self, ctx: &TraceContext) {
        if ctx.is_traced() {
            self.last_trace = Some((ctx.trace_id(), ctx.span_id, ctx.sampled()));
        }
    }

    /// 排空三路订阅：IMU 全量入队，相机/深度各保留最新一帧。
    pub fn advance(&mut self) {
        // 排空 IMU
        let mut imu_count = 0u32;
        while let Ok(Some(sample)) = self.imu_sub.receive() {
            self.capture_trace(sample.user_header());
            let m = *sample;
            self.imu_queue.push_back(ImuSample {
                t: m.timestamp,
                omega: Vector3::new(
                    m.angular_velocity_x,
                    m.angular_velocity_y,
                    m.angular_velocity_z,
                ),
                acc: Vector3::new(
                    m.linear_acceleration_x,
                    m.linear_acceleration_y,
                    m.linear_acceleration_z,
                ),
            });
            self.now_t = self.now_t.max(m.timestamp);
            imu_count += 1;
        }
        // 拉取左目：排空并保留最新（与 vio 相同的差帧教训）
        let mut left_count = 0u32;
        while let Ok(Some(s)) = self.left_sub.receive() {
            self.capture_trace(s.user_header());
            self.last_left = Some(CameraData {
                t: s.timestamp,
                left_gray: s.data.to_vec(),
                width: s.width as usize,
                height: s.height as usize,
            });
            self.now_t = self.now_t.max(s.timestamp);
            left_count += 1;
        }
        // 拉取深度：排空并保留最新
        let mut depth_count = 0u32;
        while let Ok(Some(s)) = self.depth_sub.receive() {
            self.capture_trace(s.user_header());
            self.last_depth = Some(DepthData {
                t: s.timestamp,
                depth: s.data.iter().map(|&v| f64::from(v)).collect(),
                width: s.width as usize,
                height: s.height as usize,
            });
            self.now_t = self.now_t.max(s.timestamp);
            depth_count += 1;
        }
        if left_count > 0 || depth_count > 0 {
            log::debug!(
                "advance: imu={imu_count} left={left_count} depth={depth_count} now={:.3}",
                self.now_t
            );
        }
    }

    /// 当前时刻（秒）。
    #[must_use]
    pub fn now(&self) -> f64 {
        self.now_t
    }

    /// 最近消息的 trace 上下文。
    #[must_use]
    pub fn last_trace(&self) -> Option<(u128, u64, bool)> {
        self.last_trace
    }

    /// 下一条 IMU。
    #[must_use]
    pub fn next_imu(&mut self) -> Option<ImuSample> {
        self.imu_queue.pop_front()
    }

    /// 取一对配对的相机+深度帧（相机时刻为准，时间戳差 ≤ [`PAIR_TOLERANCE`]）。
    ///
    /// 配对成功后清空两侧缓存（下一帧重新累积）。
    #[must_use]
    pub fn next_frame(&mut self) -> Option<(CameraData, DepthData)> {
        let (Some(cam), Some(dep)) = (&self.last_left, &self.last_depth) else {
            return None;
        };
        if (cam.t - dep.t).abs() > PAIR_TOLERANCE {
            return None;
        }
        let cam = self.last_left.take().expect("checked above");
        let dep = self.last_depth.take().expect("checked above");
        Some((cam, dep))
    }
}
