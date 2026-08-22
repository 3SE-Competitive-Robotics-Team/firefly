//! 传感器输入源：iceoryx2 物理环境（`MuJoCo`）。
//!
//! 实现 `firefly_vio_core::input::SensorInput` 端口：订阅 `MuJoCo` 发布的
//! IMU 与双目灰度话题，按时间戳配对成 `CameraData`。

use std::collections::VecDeque;

use firefly_pubsub::camera::{CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, GrayImageMessage};
use firefly_pubsub::imu::ImuSubscriber;
use firefly_pubsub::node::IpcNode;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::trace::TraceContext;
use firefly_vio_core::input::SensorInput;
use firefly_vio_core::sensor::{CameraData, GrayImage, ImuData};
use nalgebra::Vector3;

/// iceoryx2 物理环境输入源：订阅 `MuJoCo` 发布的 IMU 与双目灰度。
pub struct IceoryxInput {
    /// IMU 订阅（`Firefly/Imu`）。
    imu_sub: ImuSubscriber,
    /// 左目灰度订阅（`Firefly/CameraLeft`）。
    left_sub: Subscriber<GrayImageMessage>,
    /// 右目灰度订阅（`Firefly/CameraRight`）。
    right_sub: Subscriber<GrayImageMessage>,
    /// 待消费 IMU 队列。
    imu_queue: VecDeque<ImuData>,
    /// 最近左目帧（等待与右目配对）。
    last_left: Option<GrayImageMessage>,
    /// 最近右目帧（等待与左目配对）。
    last_right: Option<GrayImageMessage>,
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
            right_sub: Subscriber::with_topic(node, CAMERA_RIGHT_TOPIC)?,
            imu_queue: VecDeque::new(),
            last_left: None,
            last_right: None,
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
}

impl SensorInput for IceoryxInput {
    fn advance(&mut self) {
        // 排空 IMU
        let mut imu_count = 0u32;
        while let Ok(Some(sample)) = self.imu_sub.receive() {
            self.capture_trace(sample.user_header());
            let m = *sample;
            self.imu_queue.push_back(ImuData {
                timestamp: m.timestamp,
                wm: Vector3::new(
                    m.angular_velocity_x,
                    m.angular_velocity_y,
                    m.angular_velocity_z,
                ),
                am: Vector3::new(
                    m.linear_acceleration_x,
                    m.linear_acceleration_y,
                    m.linear_acceleration_z,
                ),
            });
            self.now_t = self.now_t.max(m.timestamp);
            imu_count += 1;
        }
        // 拉取双目帧：**排空各侧队列并保留最新**。sim 同帧先发 left 再发 right，
        // 若每轮只读一个样本，左右目会差一帧（0.1s 滞后）而让 `next_camera` 的
        // 0.01s 配对容差永远失败 → 相机帧被丢 ~90% → 滤波器喂不进视觉。
        let mut left_count = 0u32;
        let mut right_count = 0u32;
        while let Ok(Some(s)) = self.left_sub.receive() {
            self.capture_trace(s.user_header());
            self.last_left = Some(*s);
            self.now_t = self.now_t.max(s.timestamp);
            left_count += 1;
        }
        while let Ok(Some(s)) = self.right_sub.receive() {
            self.capture_trace(s.user_header());
            self.last_right = Some(*s);
            self.now_t = self.now_t.max(s.timestamp);
            right_count += 1;
        }
        // 诊断：有新相机帧时打印
        if left_count > 0 || right_count > 0 {
            log::debug!(
                "advance: imu={} left={} right={} now={:.3}",
                imu_count,
                left_count,
                right_count,
                self.now_t
            );
        }
    }

    fn now(&self) -> f64 {
        self.now_t
    }

    fn last_trace(&self) -> Option<(u128, u64, bool)> {
        self.last_trace
    }

    fn next_imu(&mut self) -> Option<ImuData> {
        self.imu_queue.pop_front()
    }

    fn next_camera(&mut self) -> Option<CameraData> {
        // 左右目按时间戳配对（同帧发布，容忍 10ms 偏差）
        const TOLERANCE: f64 = 0.01;
        let (Some(l), Some(r)) = (&self.last_left, &self.last_right) else {
            return None;
        };
        if (l.timestamp - r.timestamp).abs() > TOLERANCE {
            return None;
        }
        let left = GrayImage {
            width: l.width as usize,
            height: l.height as usize,
            data: l.data.to_vec(),
        };
        let right = GrayImage {
            width: r.width as usize,
            height: r.height as usize,
            data: r.data.to_vec(),
        };
        let timestamp = l.timestamp.max(r.timestamp);
        let sensor_ids = vec![l.sensor_id, r.sensor_id];
        // 全零 mask（对照 OpenVINS `VioManager.cpp` 的 `cv::Mat::zeros` 默认掩码：
        // 0 = 全像素有效，tracker 的 `feed_new_camera` 要求 `images.len()==masks.len()`，
        // 空 mask 会被校验拒绝 → 跟踪器从未运行 → MSCKF 无特征）。
        let masks = vec![
            GrayImage {
                width: left.width,
                height: left.height,
                data: vec![0; left.width * left.height],
            },
            GrayImage {
                width: right.width,
                height: right.height,
                data: vec![0; right.width * right.height],
            },
        ];
        self.last_left = None;
        self.last_right = None;
        Some(CameraData {
            timestamp,
            sensor_ids,
            images: vec![left, right],
            masks,
        })
    }
}
