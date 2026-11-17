//! 版本化消息协议：固定 48 字节帧头 + 按类型编码的消息体。
//!
//! 两个方向各自独占端点，消息类型与端点绑定：
//!
//! | 端点 | 方向 | 消息 |
//! |---|---|---|
//! | 传感器 | 主机 → 板端 | `Imu`、`GrayLeft`、`GrayRight`、`Depth`、`Airframe`、`Hello`、`Command`、`Goal`、`Activity`、`ImageChunk`、`Ping`、`Bye` |
//! | 执行器 | 板端 → 主机 | `HelloAck`、`Control`、`Odometry`、`CorrectedOdometry`、`VioAttitude`、`Reference`、`PlannerStatus`、`LandingTarget`、`Navigation`、`Viz`、`Voxels`、`Scalar`、`Log`、`Stats`、`Pong` |
//!
//! 所有多字节字段为小端。大消息在 [`crate::frame`] 层按
//! [`MAX_FRAGMENT`] 分片，接收端按序号重组；任一分片缺失或次序异常即丢弃
//! 整条消息并计数，不把截断图像当有效测量。
//!
//! `capture_time` 与消息体里的时间戳都取自发布端的同一时钟域。板端算法消费的
//! 传感器与状态时间沿用仿真时间轴；板端判断新鲜度只用本机单调时钟的到达时刻。

use std::io;

use firefly_pubsub::activity::ActivityMessage;
use firefly_pubsub::attitude::AttitudeMessage;
use firefly_pubsub::camera::{
    DOWN_SIZE, DepthImageMessage, GrayImageMessage, IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH,
};
use firefly_pubsub::command::CommandMessage;
use firefly_pubsub::control::ControlMessage;
use firefly_pubsub::goal::GoalMessage;
use firefly_pubsub::imu::ImuMessage;
use firefly_pubsub::landing::LandingTargetMessage;
use firefly_pubsub::log::{LogMessage, TAG_MAX, TEXT_MAX};
use firefly_pubsub::navigation::NavigationMessage;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::plant::AirframeMessage;
use firefly_pubsub::reference::ReferenceMessage;
use firefly_pubsub::trajectory::PlannerStatusMessage;
use firefly_pubsub::viz::{
    ARROWS_MAX, BINS_MAX, ENTITY_MAX, POINTS_MAX, SCALARS_MAX, ScalarMessage, VOXELS_MAX,
    VizMessage, VoxelMessage, kind as viz_kind,
};

/// 帧头魔数（ASCII `FHL1`）。
pub const MAGIC: [u8; 4] = *b"FHL1";
/// 当前协议版本；接收端拒绝版本不一致的帧，主机与板端同步部署。
pub const VERSION: u16 = 8;
/// 固定帧头长度（字节）。
pub const HEADER_LEN: usize = 48;
/// 单帧负载上限（字节）；大消息按此分片，限制单次 USB 传输的排队时延。
pub const MAX_FRAGMENT: usize = 16 * 1024;
/// 单条重组消息上限（字节）；深度图 320×240×f32 约 300KiB。
pub const MAX_MESSAGE: usize = 512 * 1024;
/// 单个 IMU 批次的最大样本数（主机按到达批次聚合）。
pub const MAX_IMU_BATCH: usize = 64;
/// 图像分块头长度：时间戳 8 + 传感器 1 + 图像号 4 + 块下标 2 + 块总数 2。
pub const IMAGE_CHUNK_HEADER_LEN: usize = 17;
/// 单块像素上限：块头 + 像素能放进一个 16KiB 帧，不再二次分片。
pub const IMAGE_CHUNK_DATA_MAX: usize = MAX_FRAGMENT - 64;

/// 消息所属的 USB 端点/方向。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Link {
    /// 传感器与指令下行：主机 → 板端。
    Sensor,
    /// 控制与遥测上行：板端 → 主机。
    Actuator,
}

/// 消息类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum Kind {
    /// 主机握手：协议版本、会话号与角色。
    Hello = 1,
    /// 板端握手应答：板端会话号、运行时长与端点状态。
    HelloAck = 2,
    /// 原始 IMU 批次。
    Imu = 3,
    /// 左目灰度图。
    GrayLeft = 4,
    /// 右目灰度图。
    GrayRight = 5,
    /// 深度图。
    Depth = 6,
    /// 机体与执行器描述。
    Airframe = 7,
    /// 飞控控制输出。
    Control = 8,
    /// 原始里程计。
    Odometry = 9,
    /// 校正后里程计（外部定位或回环后）。
    CorrectedOdometry = 10,
    /// VIO 高频预测姿态。
    VioAttitude = 11,
    /// 地面站指令。
    Command = 12,
    /// 规划目标。
    Goal = 13,
    /// 遥测标量。
    Scalar = 14,
    /// 结构化日志。
    Log = 15,
    /// 传输统计。
    Stats = 16,
    /// 往返时延探测。
    Ping = 17,
    /// 往返时延应答。
    Pong = 18,
    /// 会话结束。
    Bye = 19,
    /// 图像分块（大图按块传输，块间可插高优先消息）。
    ImageChunk = 20,
    /// 板端规划参考，供主机独立评分。
    Reference = 21,
    /// 板端规划目标身份和正常完成状态。
    PlannerStatus = 22,
    /// 原子导航与显示坐标关系。
    Navigation = 23,
    /// 有界空间诊断。
    Viz = 24,
    /// 有序体素更新分块。
    Voxels = 25,
    /// 视觉降落目标。
    LandingTarget = 26,
    /// 有时限的资源请求；SIL/HIL 共用同一消息。
    Activity = 27,
}

impl Kind {
    /// 线上编码。
    #[must_use]
    pub const fn id(self) -> u16 {
        self as u16
    }

    /// 从线上编码解析；未知编码返回 `None`。
    #[must_use]
    pub const fn from_id(id: u16) -> Option<Self> {
        Some(match id {
            1 => Self::Hello,
            2 => Self::HelloAck,
            3 => Self::Imu,
            4 => Self::GrayLeft,
            5 => Self::GrayRight,
            6 => Self::Depth,
            7 => Self::Airframe,
            8 => Self::Control,
            9 => Self::Odometry,
            10 => Self::CorrectedOdometry,
            11 => Self::VioAttitude,
            12 => Self::Command,
            13 => Self::Goal,
            14 => Self::Scalar,
            15 => Self::Log,
            16 => Self::Stats,
            17 => Self::Ping,
            18 => Self::Pong,
            19 => Self::Bye,
            20 => Self::ImageChunk,
            21 => Self::Reference,
            22 => Self::PlannerStatus,
            23 => Self::Navigation,
            24 => Self::Viz,
            25 => Self::Voxels,
            26 => Self::LandingTarget,
            27 => Self::Activity,
            _ => return None,
        })
    }

    /// 该类型允许出现的端点。
    #[must_use]
    pub const fn link(self) -> Link {
        match self {
            Self::Imu
            | Self::GrayLeft
            | Self::GrayRight
            | Self::Depth
            | Self::ImageChunk
            | Self::Airframe
            | Self::Hello
            | Self::Command
            | Self::Goal
            | Self::Activity
            | Self::Ping
            | Self::Bye => Link::Sensor,
            Self::HelloAck
            | Self::Navigation
            | Self::Viz
            | Self::Voxels
            | Self::LandingTarget
            | Self::Reference
            | Self::PlannerStatus
            | Self::Control
            | Self::Odometry
            | Self::CorrectedOdometry
            | Self::VioAttitude
            | Self::Scalar
            | Self::Log
            | Self::Stats
            | Self::Pong => Link::Actuator,
        }
    }

    /// 诊断用名称。
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Hello => "hello",
            Self::HelloAck => "hello_ack",
            Self::Reference => "reference",
            Self::PlannerStatus => "planner_status",
            Self::Navigation => "navigation",
            Self::Viz => "viz",
            Self::Voxels => "voxels",
            Self::Imu => "imu",
            Self::GrayLeft => "gray_left",
            Self::GrayRight => "gray_right",
            Self::Depth => "depth",
            Self::ImageChunk => "image_chunk",
            Self::Airframe => "airframe",
            Self::Control => "control",
            Self::Odometry => "odometry",
            Self::CorrectedOdometry => "corrected_odometry",
            Self::VioAttitude => "vio_attitude",
            Self::Command => "command",
            Self::Goal => "goal",
            Self::LandingTarget => "landing_target",
            Self::Activity => "activity",
            Self::Scalar => "scalar",
            Self::Log => "log",
            Self::Stats => "stats",
            Self::Ping => "ping",
            Self::Pong => "pong",
            Self::Bye => "bye",
        }
    }
}

/// 帧头。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Header {
    /// 消息类型。
    pub kind: Kind,
    /// 发布端会话号；跨会话残留帧必须丢弃。
    pub session: u64,
    /// 消息序号（每条消息递增，分片共享同一序号）。
    pub seq: u64,
    /// 消息主时间戳（秒，发布端时钟域），仅用于诊断与统计。
    pub capture_time: f64,
    /// 当前分片下标（从 0 开始）。
    pub fragment_index: u16,
    /// 本条消息的分片总数。
    pub fragment_count: u16,
    /// 当前分片负载长度（字节）。
    pub payload_len: u32,
}

/// 板端运行统计（板端 → 主机，1Hz）。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stats {
    /// 板端传感器端点已收消息数。
    pub sensor_messages: u64,
    /// 板端指令端点已收消息数。
    pub command_messages: u64,
    /// 板端上行已发控制消息数。
    pub control_messages: u64,
    /// 板端上行已发遥测消息数。
    pub telemetry_messages: u64,
    /// 板端丢弃的传感器图像数（旧帧被新帧替换）。
    pub dropped_images: u64,
    /// 板端检测到的 IMU 时间戳间隙（秒累计）。
    pub imu_gap_seconds: f64,
    /// 板端上行队列当前积压（消息数）。
    pub uplink_backlog: u32,
    /// 板端最近一次收到主机数据距统计时刻的单调秒数。
    pub since_host_data_s: f64,
    /// 板端上行写超时或错误计数。
    pub uplink_errors: u64,
}

/// 图像分块头：同一 `image_id` 的块在板端拼回一张图。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImageChunkHeader {
    /// 图像采集时间（秒）。
    pub timestamp: f64,
    /// 传感器：0 左目、1 右目、2 深度。
    pub sensor_id: u8,
    /// 图像号；新图号到达即丢弃未完成的旧图。
    pub image_id: u32,
    /// 本块下标。
    pub chunk_index: u16,
    /// 本图总块数。
    pub chunk_count: u16,
}

impl ImageChunkHeader {
    /// 线上长度。
    pub const LEN: usize = IMAGE_CHUNK_HEADER_LEN;

    /// 编码为定长头。
    #[must_use]
    pub fn encode(&self) -> [u8; Self::LEN] {
        let mut out = [0_u8; Self::LEN];
        out[..8].copy_from_slice(&self.timestamp.to_le_bytes());
        out[8] = self.sensor_id;
        out[9..13].copy_from_slice(&self.image_id.to_le_bytes());
        out[13..15].copy_from_slice(&self.chunk_index.to_le_bytes());
        out[15..17].copy_from_slice(&self.chunk_count.to_le_bytes());
        out
    }

    /// 从完整块体解析；限制每块长度和每张图的块数，使快速重组路径有界。
    ///
    /// # Errors
    /// 长度不足、块数越界、块号越界、传感器号非法、块体过大或深度像素不完整。
    pub fn decode(body: &[u8]) -> io::Result<Self> {
        let raw: [u8; Self::LEN] = body
            .get(..Self::LEN)
            .and_then(|slice| slice.try_into().ok())
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "image chunk header"))?;
        let header = Self {
            timestamp: f64::from_le_bytes(raw[..8].try_into().unwrap_or_default()),
            sensor_id: raw[8],
            image_id: u32::from_le_bytes(raw[9..13].try_into().unwrap_or_default()),
            chunk_index: u16::from_le_bytes(raw[13..15].try_into().unwrap_or_default()),
            chunk_count: u16::from_le_bytes(raw[15..17].try_into().unwrap_or_default()),
        };
        if header.chunk_count == 0
            || header.chunk_index >= header.chunk_count
            || header.sensor_id > 3
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "image chunk header",
            ));
        }
        let image_bytes = if header.sensor_id == 3 {
            DOWN_SIZE
        } else {
            IMAGE_SIZE * if header.sensor_id == 2 { 4 } else { 1 }
        };
        if usize::from(header.chunk_count) > image_bytes.div_ceil(IMAGE_CHUNK_DATA_MAX)
            || body.len() - Self::LEN > IMAGE_CHUNK_DATA_MAX
            || (header.sensor_id == 2 && !(body.len() - Self::LEN).is_multiple_of(4))
        {
            return Err(invalid("image chunk exceeds reassembly budget"));
        }
        Ok(header)
    }
}

/// 一张图的一个分块。
#[derive(Clone, Debug, PartialEq)]
pub struct ImageChunk {
    /// 分块头。
    pub header: ImageChunkHeader,
    /// 像素/深度字节；按块号顺序拼接。
    pub data: Vec<u8>,
}

/// 协议消息。
///
/// 变体大小差异来自定长遥测结构；消息按值传递时只搬移几百字节，
/// 图像与深度已装箱。
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub enum Message {
    /// 主机握手。
    Hello(Hello),
    /// 板端握手应答。
    HelloAck(HelloAck),
    /// IMU 批次（至少一条）。
    Imu(Vec<ImuMessage>),
    /// 灰度图（装箱，避免 300KiB 消息按值搬移撑爆线程栈）。
    Gray(Box<GrayImageMessage>),
    /// 深度图（装箱）。
    Depth(Box<DepthImageMessage>),
    /// 图像分块。
    ImageChunk(ImageChunk),
    /// 机体描述。
    Airframe(AirframeMessage),
    /// 控制输出。
    Control(ControlMessage),
    /// 原始里程计。
    Odometry(OdomMessage),
    /// 校正后里程计。
    CorrectedOdometry(OdomMessage),
    /// 预测姿态。
    VioAttitude(AttitudeMessage),
    /// 地面站指令。
    Command(CommandMessage),
    /// 规划目标。
    Goal(GoalMessage),
    /// 板端规划参考。
    Reference(ReferenceMessage),
    /// 板端规划执行快照。
    PlannerStatus(PlannerStatusMessage),
    /// 原子导航快照。
    Navigation(NavigationMessage),
    /// 空间诊断，仅编码有效数组元素。
    Viz(Box<VizMessage>),
    /// 体素分块，仅编码有效索引。
    Voxels(Box<VoxelMessage>),
    /// 遥测标量。
    Scalar(ScalarMessage),
    /// 视觉降落目标，保持采集时间和 VIO 会话。
    LandingTarget(LandingTargetMessage),
    /// 任务资源租约。
    Activity(ActivityMessage),
    /// 结构化日志。
    Log(LogMessage),
    /// 传输统计。
    Stats(Stats),
    /// 时延探测。
    Ping(u64),
    /// 时延应答。
    Pong(u64),
    /// 会话结束（`0` 正常，`1` 板端故障，`2` 主机取消）。
    Bye(u8),
}

/// 主机握手内容。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hello {
    /// 协议版本。
    pub version: u16,
    /// 主机角色标识（固定 [`ROLE_HOST`]）。
    pub role: u8,
    /// 主机本次会话号。
    pub session: u64,
}

/// 板端握手应答。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HelloAck {
    /// 协议版本。
    pub version: u16,
    /// 0 = 接受；非 0 为拒绝原因（1 版本不兼容，2 端点不可用，3 会话繁忙）。
    pub status: u8,
    /// 板端本次上电会话号（用于识别板端重启）。
    pub board_session: u64,
    /// 板端系统运行时长（秒）。
    pub uptime_s: f64,
}

/// 主机角色编码。
pub const ROLE_HOST: u8 = 1;
/// 板端角色编码。
pub const ROLE_BOARD: u8 = 2;

impl PartialEq for Message {
    /// 线上表示相同即相等；用于往返测试与重放比对。
    fn eq(&self, other: &Self) -> bool {
        self.kind() == other.kind() && self.encode() == other.encode()
    }
}

impl Message {
    /// 消息类型。
    #[must_use]
    pub const fn kind(&self) -> Kind {
        match self {
            Self::Hello(_) => Kind::Hello,
            Self::HelloAck(_) => Kind::HelloAck,
            Self::Imu(_) => Kind::Imu,
            Self::Gray(message) => {
                if message.sensor_id == 0 {
                    Kind::GrayLeft
                } else {
                    Kind::GrayRight
                }
            }
            Self::Depth(_) => Kind::Depth,
            Self::ImageChunk(_) => Kind::ImageChunk,
            Self::Airframe(_) => Kind::Airframe,
            Self::Control(_) => Kind::Control,
            Self::Odometry(_) => Kind::Odometry,
            Self::CorrectedOdometry(_) => Kind::CorrectedOdometry,
            Self::VioAttitude(_) => Kind::VioAttitude,
            Self::Command(_) => Kind::Command,
            Self::Goal(_) => Kind::Goal,
            Self::Reference(_) => Kind::Reference,
            Self::PlannerStatus(_) => Kind::PlannerStatus,
            Self::Navigation(_) => Kind::Navigation,
            Self::Viz(_) => Kind::Viz,
            Self::Voxels(_) => Kind::Voxels,
            Self::LandingTarget(_) => Kind::LandingTarget,
            Self::Activity(_) => Kind::Activity,
            Self::Scalar(_) => Kind::Scalar,
            Self::Log(_) => Kind::Log,
            Self::Stats(_) => Kind::Stats,
            Self::Ping(_) => Kind::Ping,
            Self::Pong(_) => Kind::Pong,
            Self::Bye(_) => Kind::Bye,
        }
    }

    /// 消息主时间戳（秒）；不携带时钟的消息返回 `-1`。
    #[must_use]
    pub fn capture_time(&self) -> f64 {
        match self {
            Self::Imu(samples) => samples.first().map_or(-1.0, |m| m.timestamp),
            Self::Gray(m) => m.timestamp,
            Self::Depth(m) => m.timestamp,
            Self::ImageChunk(m) => m.header.timestamp,
            Self::Airframe(m) => m.timestamp,
            Self::Control(m) => m.state_time,
            Self::Odometry(m) | Self::CorrectedOdometry(m) => m.timestamp,
            Self::VioAttitude(m) => m.timestamp,
            Self::Command(m) => m.timestamp,
            Self::Goal(m) => m.timestamp,
            Self::Reference(m) => m.timestamp,
            Self::PlannerStatus(m) => m.timestamp,
            Self::Navigation(m) => m.odom.timestamp,
            Self::Viz(m) => m.timestamp,
            Self::Voxels(m) => m.timestamp,
            Self::LandingTarget(m) => m.timestamp,
            Self::Activity(m) => m.timestamp,
            Self::Scalar(m) => m.timestamp,
            Self::Log(m) => m.sim_time,
            Self::Hello(_)
            | Self::HelloAck(_)
            | Self::Stats(_)
            | Self::Ping(_)
            | Self::Pong(_)
            | Self::Bye(_) => -1.0,
        }
    }

    /// 编码消息体。
    #[must_use]
    #[allow(clippy::too_many_lines)] // 线上格式按类型平铺，拆函数只会增加跳转
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        match self {
            Self::Hello(hello) => {
                put_u16(&mut out, hello.version);
                out.push(hello.role);
                out.push(0);
                put_u64(&mut out, hello.session);
            }
            Self::HelloAck(ack) => {
                put_u16(&mut out, ack.version);
                out.push(ack.status);
                out.push(0);
                put_u64(&mut out, ack.board_session);
                put_f64(&mut out, ack.uptime_s);
            }
            Self::Imu(samples) => {
                put_u16(&mut out, u16::try_from(samples.len()).unwrap_or(0));
                put_u16(&mut out, 0);
                for sample in samples {
                    put_f64(&mut out, sample.timestamp);
                    for value in [
                        sample.angular_velocity_x,
                        sample.angular_velocity_y,
                        sample.angular_velocity_z,
                        sample.linear_acceleration_x,
                        sample.linear_acceleration_y,
                        sample.linear_acceleration_z,
                    ] {
                        put_f64(&mut out, value);
                    }
                }
            }
            Self::Gray(message) => {
                put_u16(&mut out, u16::try_from(message.width).unwrap_or(0));
                put_u16(&mut out, u16::try_from(message.height).unwrap_or(0));
                out.push(u8::try_from(message.sensor_id).unwrap_or(0));
                out.push(0);
                put_f64(&mut out, message.timestamp);
                out.extend_from_slice(&message.data);
            }
            Self::Depth(message) => {
                put_u16(&mut out, u16::try_from(message.width).unwrap_or(0));
                put_u16(&mut out, u16::try_from(message.height).unwrap_or(0));
                put_i32(&mut out, message.sensor_id);
                put_f64(&mut out, message.timestamp);
                for &value in &message.data {
                    out.extend_from_slice(&value.to_le_bytes());
                }
            }
            Self::ImageChunk(chunk) => {
                out.extend_from_slice(&chunk.header.encode());
                out.extend_from_slice(&chunk.data);
            }
            Self::Airframe(message) => {
                put_f64(&mut out, message.timestamp);
                put_f64(&mut out, message.mass);
                for value in [message.inertia_x, message.inertia_y, message.inertia_z] {
                    put_f64(&mut out, value);
                }
                put_f64(&mut out, message.linear_drag);
                put_f64(&mut out, message.angular_drag);
                for rotor in message.rotor_positions {
                    for value in rotor {
                        put_f64(&mut out, value);
                    }
                }
                for value in message.rotor_spins {
                    put_f64(&mut out, value);
                }
                put_f64(&mut out, message.max_thrust_per_motor);
                put_f64(&mut out, message.torque_coefficient);
            }
            Self::Control(message) => {
                put_f64(&mut out, message.state_time);
                put_u64(&mut out, message.tick);
                for value in message.thrust {
                    put_f64(&mut out, value);
                }
            }
            Self::Odometry(message) | Self::CorrectedOdometry(message) => {
                put_f64(&mut out, message.timestamp);
                for value in [
                    message.position_x,
                    message.position_y,
                    message.position_z,
                    message.velocity_x,
                    message.velocity_y,
                    message.velocity_z,
                    message.quat_x,
                    message.quat_y,
                    message.quat_z,
                    message.quat_w,
                ] {
                    put_f64(&mut out, value);
                }
                out.push(u8::from(message.is_initialized));
            }
            Self::VioAttitude(message) => {
                put_f64(&mut out, message.timestamp);
                put_f64(&mut out, message.posterior_time);
                put_u64(&mut out, message.session);
                for value in message.quat_xyzw {
                    put_f64(&mut out, value);
                }
                for value in message.angular_velocity {
                    put_f64(&mut out, value);
                }
                for value in message.gyro_bias {
                    put_f64(&mut out, value);
                }
            }
            Self::Command(message) => {
                put_f64(&mut out, message.timestamp);
                out.push(message.kind);
                put_f64(&mut out, message.altitude);
                put_u64(&mut out, message.sequence);
            }
            Self::Goal(message) => {
                put_f64(&mut out, message.timestamp);
                put_f64(&mut out, message.position_x);
                put_f64(&mut out, message.position_y);
                put_f64(&mut out, message.position_z);
                put_f64(&mut out, message.yaw);
                put_u64(&mut out, message.has_yaw);
            }
            Self::Reference(message) => {
                for value in [
                    message.timestamp,
                    message.position_x,
                    message.position_y,
                    message.position_z,
                    message.velocity_x,
                    message.velocity_y,
                    message.velocity_z,
                    message.acceleration_x,
                    message.acceleration_y,
                    message.acceleration_z,
                    message.yaw,
                    message.yaw_dot,
                ] {
                    put_f64(&mut out, value);
                }
                for value in message
                    .alignment_translation
                    .into_iter()
                    .chain(message.alignment_rotation)
                {
                    put_f64(&mut out, value);
                }
                put_u64(&mut out, message.has_alignment);
                put_f64(&mut out, message.goal_timestamp);
            }
            Self::PlannerStatus(message) => {
                put_f64(&mut out, message.timestamp);
                put_f64(&mut out, message.goal_timestamp);
                put_u64(&mut out, message.finished);
            }
            Self::Navigation(message) => {
                out.extend(Self::Odometry(message.odom).encode());
                put_f64s(&mut out, &message.translation);
                put_f64s(&mut out, &message.rotation);
                put_u64(&mut out, message.revision);
                put_u64(&mut out, message.external_map);
            }
            Self::Viz(message) => {
                put_u32(&mut out, message.kind);
                put_u32(&mut out, message.entity_len);
                out.extend_from_slice(&message.entity);
                put_f64(&mut out, message.timestamp);
                put_u64(&mut out, message.sequence);
                out.extend_from_slice(&message.color);
                put_f64s(&mut out, &message.xyz);
                put_f64s(&mut out, &message.quat_xyzw);
                put_u32(&mut out, message.point_count);
                for point in &message.points[..(message.point_count as usize).min(POINTS_MAX)] {
                    put_f64s(&mut out, point);
                }
                put_u32(&mut out, message.arrow_count);
                for (origin, vector) in message.arrow_origins
                    [..(message.arrow_count as usize).min(ARROWS_MAX)]
                    .iter()
                    .zip(&message.arrow_vectors[..(message.arrow_count as usize).min(ARROWS_MAX)])
                {
                    put_f64s(&mut out, origin);
                    put_f64s(&mut out, vector);
                }
                put_u32(&mut out, message.scalar_count);
                put_f64s(
                    &mut out,
                    &message.scalars[..(message.scalar_count as usize).min(SCALARS_MAX)],
                );
                put_u32(&mut out, message.bin_count);
                for value in &message.bins[..(message.bin_count as usize).min(BINS_MAX)] {
                    put_u64(&mut out, *value);
                }
                put_i64(&mut out, message.bin_start);
                put_i64(&mut out, message.bin_width);
            }
            Self::Voxels(message) => {
                put_u32(&mut out, message.kind);
                put_u32(&mut out, message.entity_len);
                out.extend_from_slice(&message.entity);
                put_f64(&mut out, message.timestamp);
                put_u64(&mut out, message.sequence);
                put_u32(&mut out, message.chunk_index);
                put_u32(&mut out, message.chunk_count);
                for value in message.voxel_size.into_iter().chain(message.voxel_origin) {
                    out.extend_from_slice(&value.to_le_bytes());
                }
                put_u32(&mut out, message.voxel_count);
                for index in &message.voxels[..(message.voxel_count as usize).min(VOXELS_MAX)] {
                    for value in index {
                        put_i32(&mut out, *value);
                    }
                }
            }
            Self::Activity(message) => {
                put_f64(&mut out, message.timestamp);
                put_u64(&mut out, message.owner);
                put_u64(&mut out, message.sequence);
                put_u32(&mut out, message.mask);
                put_u32(&mut out, message.lease_ms);
            }
            Self::LandingTarget(message) => {
                put_f64(&mut out, message.timestamp);
                put_u64(&mut out, message.session);
                for value in message.position {
                    put_f64(&mut out, value);
                }
                put_f64(&mut out, message.reprojection_px);
                put_f64(&mut out, message.confidence);
                put_u32(&mut out, message.team);
                put_u32(&mut out, message.valid);
            }
            Self::Scalar(message) => {
                put_u32(&mut out, message.kind);
                put_u32(&mut out, message.entity_len);
                put_f64(&mut out, message.timestamp);
                out.extend_from_slice(&message.entity);
                for value in message.scalars {
                    put_f64(&mut out, value);
                }
                put_u32(&mut out, message.scalar_count);
                put_u64(&mut out, message.sequence);
            }
            Self::Log(message) => {
                out.push(message.log_level);
                out.push(message.tag_len);
                put_u16(&mut out, message.reserved[0].into());
                put_u32(&mut out, message.text_len);
                put_f64(&mut out, message.sim_time);
                out.extend_from_slice(&message.tag);
                out.extend_from_slice(&message.text);
                put_i64(&mut out, message.wall_secs);
                put_u32(&mut out, message.wall_nanos);
                put_u64(&mut out, message.trace_id_hi);
                put_u64(&mut out, message.trace_id_lo);
                put_u64(&mut out, message.sequence);
                put_u64(&mut out, message.dropped_records);
            }
            Self::Stats(stats) => {
                put_u64(&mut out, stats.sensor_messages);
                put_u64(&mut out, stats.command_messages);
                put_u64(&mut out, stats.control_messages);
                put_u64(&mut out, stats.telemetry_messages);
                put_u64(&mut out, stats.dropped_images);
                put_f64(&mut out, stats.imu_gap_seconds);
                put_u32(&mut out, stats.uplink_backlog);
                put_f64(&mut out, stats.since_host_data_s);
                put_u64(&mut out, stats.uplink_errors);
            }
            Self::Ping(token) | Self::Pong(token) => put_u64(&mut out, *token),
            Self::Bye(reason) => out.push(*reason),
        }
        out
    }

    /// 按类型解码消息体。
    ///
    /// # Errors
    /// 长度、取值或消息类型不合法时返回 [`io::ErrorKind::InvalidData`]。
    #[allow(clippy::too_many_lines)] // 与 encode 对称的平铺结构
    pub fn decode(kind: Kind, body: &[u8]) -> io::Result<Self> {
        let mut input = Input::new(body);
        let message = match kind {
            Kind::Hello => {
                let version = input.u16()?;
                let role = input.u8()?;
                let _reserved = input.u8()?;
                Self::Hello(Hello {
                    version,
                    role,
                    session: input.u64()?,
                })
            }
            Kind::HelloAck => {
                let version = input.u16()?;
                let status = input.u8()?;
                let _reserved = input.u8()?;
                Self::HelloAck(HelloAck {
                    version,
                    status,
                    board_session: input.u64()?,
                    uptime_s: input.f64()?,
                })
            }
            Kind::Imu => {
                let count = usize::from(input.u16()?);
                let _reserved = input.u16()?;
                if count == 0 || count > MAX_IMU_BATCH {
                    return Err(invalid("imu batch size"));
                }
                let mut samples = Vec::with_capacity(count);
                for _ in 0..count {
                    let timestamp = input.f64()?;
                    samples.push(ImuMessage {
                        timestamp,
                        angular_velocity_x: input.f64()?,
                        angular_velocity_y: input.f64()?,
                        angular_velocity_z: input.f64()?,
                        linear_acceleration_x: input.f64()?,
                        linear_acceleration_y: input.f64()?,
                        linear_acceleration_z: input.f64()?,
                    });
                }
                Self::Imu(samples)
            }
            Kind::GrayLeft | Kind::GrayRight => {
                let width = input.u16()?;
                let height = input.u16()?;
                let sensor_id = i32::from(input.u8()?);
                let _reserved = input.u8()?;
                if usize::from(width) != IMAGE_WIDTH || usize::from(height) != IMAGE_HEIGHT {
                    return Err(invalid("gray image size"));
                }
                if (sensor_id == 0) != (kind == Kind::GrayLeft) {
                    return Err(invalid("gray image sensor id"));
                }
                let timestamp = input.f64()?;
                let data: [u8; IMAGE_SIZE] = input
                    .bytes(IMAGE_SIZE)?
                    .try_into()
                    .map_err(|_| invalid("gray image payload"))?;
                Self::Gray(Box::new(GrayImageMessage {
                    timestamp,
                    sensor_id,
                    width: u32::from(width),
                    height: u32::from(height),
                    data,
                }))
            }
            Kind::Depth => {
                let width = input.u16()?;
                let height = input.u16()?;
                let sensor_id = input.i32()?;
                if usize::from(width) != IMAGE_WIDTH || usize::from(height) != IMAGE_HEIGHT {
                    return Err(invalid("depth image size"));
                }
                let timestamp = input.f64()?;
                let raw = input.bytes(IMAGE_SIZE * 4)?;
                let mut message = Box::new(DepthImageMessage {
                    timestamp,
                    sensor_id,
                    width: u32::from(width),
                    height: u32::from(height),
                    ..DepthImageMessage::default()
                });
                for (index, slot) in message.data.iter_mut().enumerate() {
                    let offset = index * 4;
                    *slot = f32::from_le_bytes([
                        raw[offset],
                        raw[offset + 1],
                        raw[offset + 2],
                        raw[offset + 3],
                    ]);
                }
                Self::Depth(message)
            }
            Kind::ImageChunk => {
                let body = input.bytes(body.len())?;
                let header = ImageChunkHeader::decode(body)?;
                let data = body[IMAGE_CHUNK_HEADER_LEN..].to_vec();
                Self::ImageChunk(ImageChunk { header, data })
            }
            Kind::Airframe => {
                let timestamp = input.f64()?;
                let mass = input.f64()?;
                let inertia_x = input.f64()?;
                let inertia_y = input.f64()?;
                let inertia_z = input.f64()?;
                let linear_drag = input.f64()?;
                let angular_drag = input.f64()?;
                let mut rotor_positions = [[0.0; 3]; 4];
                for rotor in &mut rotor_positions {
                    for value in rotor {
                        *value = input.f64()?;
                    }
                }
                let mut rotor_spins = [0.0; 4];
                for value in &mut rotor_spins {
                    *value = input.f64()?;
                }
                Self::Airframe(AirframeMessage {
                    timestamp,
                    mass,
                    inertia_x,
                    inertia_y,
                    inertia_z,
                    linear_drag,
                    angular_drag,
                    rotor_positions,
                    rotor_spins,
                    max_thrust_per_motor: input.f64()?,
                    torque_coefficient: input.f64()?,
                })
            }
            Kind::Control => {
                let state_time = input.f64()?;
                let tick = input.u64()?;
                let mut thrust = [0.0; 4];
                for value in &mut thrust {
                    *value = input.f64()?;
                }
                Self::Control(ControlMessage {
                    state_time,
                    thrust,
                    tick,
                })
            }
            Kind::Odometry | Kind::CorrectedOdometry => {
                let message = input.odom()?;
                if kind == Kind::Odometry {
                    Self::Odometry(message)
                } else {
                    Self::CorrectedOdometry(message)
                }
            }
            Kind::VioAttitude => {
                let timestamp = input.f64()?;
                let posterior_time = input.f64()?;
                let session = input.u64()?;
                let mut quat_xyzw = [0.0; 4];
                for value in &mut quat_xyzw {
                    *value = input.f64()?;
                }
                let mut angular_velocity = [0.0; 3];
                for value in &mut angular_velocity {
                    *value = input.f64()?;
                }
                let mut gyro_bias = [0.0; 3];
                for value in &mut gyro_bias {
                    *value = input.f64()?;
                }
                Self::VioAttitude(AttitudeMessage {
                    timestamp,
                    posterior_time,
                    session,
                    quat_xyzw,
                    angular_velocity,
                    gyro_bias,
                })
            }
            Kind::Command => {
                let timestamp = input.f64()?;
                let kind = input.u8()?;
                let altitude = input.f64()?;
                let sequence = input.u64()?;
                Self::Command(CommandMessage {
                    timestamp,
                    kind,
                    altitude,
                    sequence,
                })
            }
            Kind::Goal => Self::Goal(GoalMessage {
                timestamp: input.f64()?,
                position_x: input.f64()?,
                position_y: input.f64()?,
                position_z: input.f64()?,
                yaw: input.f64()?,
                has_yaw: input.u64()?,
            }),
            Kind::Reference => Self::Reference(ReferenceMessage {
                timestamp: input.f64()?,
                position_x: input.f64()?,
                position_y: input.f64()?,
                position_z: input.f64()?,
                velocity_x: input.f64()?,
                velocity_y: input.f64()?,
                velocity_z: input.f64()?,
                acceleration_x: input.f64()?,
                acceleration_y: input.f64()?,
                acceleration_z: input.f64()?,
                yaw: input.f64()?,
                yaw_dot: input.f64()?,
                alignment_translation: [input.f64()?, input.f64()?, input.f64()?],
                alignment_rotation: [input.f64()?, input.f64()?, input.f64()?, input.f64()?],
                has_alignment: input.u64()?,
                goal_timestamp: input.f64()?,
            }),
            Kind::PlannerStatus => Self::PlannerStatus(PlannerStatusMessage {
                timestamp: input.f64()?,
                goal_timestamp: input.f64()?,
                finished: input.u64()?,
            }),
            Kind::Navigation => Self::Navigation(NavigationMessage {
                odom: input.odom()?,
                translation: input.f64s()?,
                rotation: input.f64s()?,
                revision: input.u64()?,
                external_map: input.u64()?,
            }),
            Kind::Viz => {
                let mut message = Box::<VizMessage>::default();
                message.kind = input.u32()?;
                message.entity_len = input.count(ENTITY_MAX)?;
                message.entity.copy_from_slice(input.bytes(ENTITY_MAX)?);
                message.timestamp = input.f64()?;
                message.sequence = input.u64()?;
                message.color.copy_from_slice(input.bytes(3)?);
                message.xyz = input.f64s()?;
                message.quat_xyzw = input.f64s()?;
                message.point_count = input.count(POINTS_MAX)?;
                for point in &mut message.points[..message.point_count as usize] {
                    *point = input.f64s()?;
                }
                message.arrow_count = input.count(ARROWS_MAX)?;
                for (origin, vector) in message.arrow_origins[..message.arrow_count as usize]
                    .iter_mut()
                    .zip(&mut message.arrow_vectors[..message.arrow_count as usize])
                {
                    *origin = input.f64s()?;
                    *vector = input.f64s()?;
                }
                message.scalar_count = input.count(SCALARS_MAX)?;
                for value in &mut message.scalars[..message.scalar_count as usize] {
                    *value = input.f64()?;
                }
                message.bin_count = input.count(BINS_MAX)?;
                for value in &mut message.bins[..message.bin_count as usize] {
                    *value = input.u64()?;
                }
                message.bin_start = input.i64()?;
                message.bin_width = input.i64()?;
                Self::Viz(message)
            }
            Kind::Voxels => {
                let kind = input.u32()?;
                if !matches!(
                    kind,
                    viz_kind::VOXELS | viz_kind::VOXELS_ADD | viz_kind::VOXELS_REMOVE
                ) {
                    return Err(invalid("voxel kind"));
                }
                let entity_len = input.count(ENTITY_MAX)?;
                let mut message = Box::new(VoxelMessage::base(kind, -1.0, ""));
                message.entity_len = entity_len;
                message.entity.copy_from_slice(input.bytes(ENTITY_MAX)?);
                message.timestamp = input.f64()?;
                message.sequence = input.u64()?;
                message.chunk_index = input.u32()?;
                message.chunk_count = input.u32()?;
                if message.chunk_count == 0 || message.chunk_index >= message.chunk_count {
                    return Err(invalid("voxel chunk index"));
                }
                for value in message
                    .voxel_size
                    .iter_mut()
                    .chain(&mut message.voxel_origin)
                {
                    *value =
                        f32::from_le_bytes(input.bytes(4)?.try_into().map_err(|_| invalid("f32"))?);
                }
                message.voxel_count = input.count(VOXELS_MAX)?;
                for index in &mut message.voxels[..message.voxel_count as usize] {
                    for value in index {
                        *value = input.i32()?;
                    }
                }
                Self::Voxels(message)
            }
            Kind::Activity => {
                let message = ActivityMessage {
                    timestamp: input.f64()?,
                    owner: input.u64()?,
                    sequence: input.u64()?,
                    mask: input.u32()?,
                    lease_ms: input.u32()?,
                };
                if !message.valid() {
                    return Err(invalid("activity lease"));
                }
                Self::Activity(message)
            }
            Kind::LandingTarget => {
                let timestamp = input.f64()?;
                let session = input.u64()?;
                let position = [input.f64()?, input.f64()?, input.f64()?];
                let reprojection_px = input.f64()?;
                let confidence = input.f64()?;
                let team = input.u32()?;
                let valid = input.u32()?;
                if team > 1 || valid > 1 {
                    return Err(invalid("landing target flags"));
                }
                Self::LandingTarget(LandingTargetMessage {
                    timestamp,
                    session,
                    position,
                    reprojection_px,
                    confidence,
                    team,
                    valid,
                })
            }
            Kind::Scalar => {
                let message_kind = input.u32()?;
                let entity_len = input.u32()?;
                if message_kind != viz_kind::SCALARS {
                    return Err(invalid("scalar kind"));
                }
                let timestamp = input.f64()?;
                let entity_slice = input.bytes(ENTITY_MAX)?;
                let mut entity = [0_u8; ENTITY_MAX];
                entity.copy_from_slice(entity_slice);
                let mut scalars = [0.0; SCALARS_MAX];
                for value in &mut scalars {
                    *value = input.f64()?;
                }
                let scalar_count = input.u32()?;
                let sequence = input.u64()?;
                if entity_len as usize > ENTITY_MAX {
                    return Err(invalid("scalar entity length"));
                }
                if scalar_count as usize > SCALARS_MAX {
                    return Err(invalid("scalar count"));
                }
                Self::Scalar(ScalarMessage {
                    kind: message_kind,
                    entity_len,
                    timestamp,
                    entity,
                    scalars,
                    scalar_count,
                    sequence,
                })
            }
            Kind::Log => {
                let log_level = input.u8()?;
                let tag_len = input.u8()?;
                let _reserved = input.u16()?;
                let text_len = input.u32()?;
                if text_len as usize > TEXT_MAX || tag_len as usize > TAG_MAX {
                    return Err(invalid("log text length"));
                }
                let sim_time = input.f64()?;
                let mut tag = [0_u8; TAG_MAX];
                tag.copy_from_slice(input.bytes(TAG_MAX)?);
                let mut text = [0_u8; TEXT_MAX];
                text.copy_from_slice(input.bytes(TEXT_MAX)?);
                Self::Log(LogMessage {
                    log_level,
                    tag_len,
                    reserved: [0; 2],
                    text_len,
                    tag,
                    text,
                    sim_time,
                    wall_secs: input.i64()?,
                    wall_nanos: input.u32()?,
                    trace_id_hi: input.u64()?,
                    trace_id_lo: input.u64()?,
                    sequence: input.u64()?,
                    dropped_records: input.u64()?,
                })
            }
            Kind::Stats => Self::Stats(Stats {
                sensor_messages: input.u64()?,
                command_messages: input.u64()?,
                control_messages: input.u64()?,
                telemetry_messages: input.u64()?,
                dropped_images: input.u64()?,
                imu_gap_seconds: input.f64()?,
                uplink_backlog: input.u32()?,
                since_host_data_s: input.f64()?,
                uplink_errors: input.u64()?,
            }),
            Kind::Ping => Self::Ping(input.u64()?),
            Kind::Pong => Self::Pong(input.u64()?),
            Kind::Bye => Self::Bye(input.u8()?),
        };
        if input.remaining() != 0 {
            return Err(invalid("trailing bytes"));
        }
        Ok(message)
    }
}

/// 编码帧头。
#[must_use]
pub fn encode_header(header: &Header) -> [u8; HEADER_LEN] {
    let mut out = [0_u8; HEADER_LEN];
    out[0..4].copy_from_slice(&MAGIC);
    out[4..6].copy_from_slice(&VERSION.to_le_bytes());
    out[6..8].copy_from_slice(&header.kind.id().to_le_bytes());
    out[8..10].copy_from_slice(&0_u16.to_le_bytes());
    out[10..12].copy_from_slice(&u16::try_from(HEADER_LEN).unwrap_or(0).to_le_bytes());
    out[12..20].copy_from_slice(&header.session.to_le_bytes());
    out[20..28].copy_from_slice(&header.seq.to_le_bytes());
    out[28..36].copy_from_slice(&header.capture_time.to_le_bytes());
    out[36..38].copy_from_slice(&header.fragment_index.to_le_bytes());
    out[38..40].copy_from_slice(&header.fragment_count.to_le_bytes());
    out[40..44].copy_from_slice(&0_u32.to_le_bytes());
    out[44..48].copy_from_slice(&header.payload_len.to_le_bytes());
    out
}

/// 解码帧头。
///
/// # Errors
/// 魔数、版本、长度或消息类型非法时返回 [`io::ErrorKind::InvalidData`]。
pub fn decode_header(raw: &[u8; HEADER_LEN]) -> io::Result<Header> {
    if raw[0..4] != MAGIC {
        return Err(invalid("frame magic"));
    }
    let version = u16::from_le_bytes([raw[4], raw[5]]);
    if version != VERSION {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("frame version {version}; expected {VERSION}; rebuild and deploy both peers"),
        ));
    }
    let kind =
        Kind::from_id(u16::from_le_bytes([raw[6], raw[7]])).ok_or_else(|| invalid("frame kind"))?;
    let header_len = usize::from(u16::from_le_bytes([raw[10], raw[11]]));
    if header_len != HEADER_LEN {
        return Err(invalid("frame header length"));
    }
    let payload_len = u32::from_le_bytes([raw[44], raw[45], raw[46], raw[47]]);
    if payload_len as usize > MAX_FRAGMENT {
        return Err(invalid("frame payload length"));
    }
    let header = Header {
        kind,
        session: u64::from_le_bytes(raw[12..20].try_into().map_err(|_| invalid("session"))?),
        seq: u64::from_le_bytes(raw[20..28].try_into().map_err(|_| invalid("seq"))?),
        capture_time: f64::from_le_bytes(raw[28..36].try_into().map_err(|_| invalid("time"))?),
        fragment_index: u16::from_le_bytes([raw[36], raw[37]]),
        fragment_count: u16::from_le_bytes([raw[38], raw[39]]),
        payload_len,
    };
    if header.fragment_count == 0 || header.fragment_index >= header.fragment_count {
        return Err(invalid("frame fragment index"));
    }
    Ok(header)
}

fn invalid(what: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, what)
}

struct Input<'a> {
    data: &'a [u8],
    offset: usize,
}

impl<'a> Input<'a> {
    const fn new(data: &'a [u8]) -> Self {
        Self { data, offset: 0 }
    }

    fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.offset)
    }

    fn bytes(&mut self, len: usize) -> io::Result<&'a [u8]> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or_else(|| invalid("length overflow"))?;
        let slice = self
            .data
            .get(self.offset..end)
            .ok_or_else(|| invalid("truncated body"))?;
        self.offset = end;
        Ok(slice)
    }

    fn u8(&mut self) -> io::Result<u8> {
        Ok(self.bytes(1)?[0])
    }

    fn u16(&mut self) -> io::Result<u16> {
        Ok(u16::from_le_bytes(
            self.bytes(2)?.try_into().map_err(|_| invalid("u16"))?,
        ))
    }

    fn u32(&mut self) -> io::Result<u32> {
        Ok(u32::from_le_bytes(
            self.bytes(4)?.try_into().map_err(|_| invalid("u32"))?,
        ))
    }

    fn i32(&mut self) -> io::Result<i32> {
        Ok(i32::from_le_bytes(
            self.bytes(4)?.try_into().map_err(|_| invalid("i32"))?,
        ))
    }

    fn u64(&mut self) -> io::Result<u64> {
        Ok(u64::from_le_bytes(
            self.bytes(8)?.try_into().map_err(|_| invalid("u64"))?,
        ))
    }

    fn i64(&mut self) -> io::Result<i64> {
        Ok(i64::from_le_bytes(
            self.bytes(8)?.try_into().map_err(|_| invalid("i64"))?,
        ))
    }

    fn f64(&mut self) -> io::Result<f64> {
        Ok(f64::from_le_bytes(
            self.bytes(8)?.try_into().map_err(|_| invalid("f64"))?,
        ))
    }

    fn f64s<const N: usize>(&mut self) -> io::Result<[f64; N]> {
        let mut values = [0.; N];
        for value in &mut values {
            *value = self.f64()?;
        }
        Ok(values)
    }

    fn count(&mut self, maximum: usize) -> io::Result<u32> {
        let count = self.u32()?;
        if count as usize > maximum {
            return Err(invalid("diagnostic count exceeds budget"));
        }
        Ok(count)
    }

    fn odom(&mut self) -> io::Result<OdomMessage> {
        Ok(OdomMessage {
            timestamp: self.f64()?,
            position_x: self.f64()?,
            position_y: self.f64()?,
            position_z: self.f64()?,
            velocity_x: self.f64()?,
            velocity_y: self.f64()?,
            velocity_z: self.f64()?,
            quat_x: self.f64()?,
            quat_y: self.f64()?,
            quat_z: self.f64()?,
            quat_w: self.f64()?,
            is_initialized: self.u8()? != 0,
        })
    }
}

fn put_f64s(out: &mut Vec<u8>, values: &[f64]) {
    for value in values {
        put_f64(out, *value);
    }
}

fn put_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_i32(out: &mut Vec<u8>, value: i32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_i64(out: &mut Vec<u8>, value: i64) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_f64(out: &mut Vec<u8>, value: f64) {
    out.extend_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod visualization_tests {
    use super::*;
    use crate::frame::{FrameReader, FrameWriter};

    fn roundtrip(message: &Message) {
        assert_eq!(message.kind().link(), Link::Actuator);
        let mut writer = FrameWriter::new(Vec::new());
        writer.write_message(message).unwrap();
        let bytes = writer.into_inner();
        let frame = FrameReader::new(bytes.as_slice()).read_frame().unwrap();
        assert_eq!(&frame.message, message);
        assert_eq!(frame.header.capture_time, message.capture_time());
        let body = message.encode();
        assert!(Message::decode(message.kind(), &body[..body.len() - 1]).is_err());
    }

    #[test]
    fn navigation_keeps_atomic_alignment_and_session_metadata() {
        roundtrip(&Message::Navigation(NavigationMessage {
            odom: OdomMessage {
                timestamp: 7.25,
                position_x: 12.,
                quat_w: 1.,
                is_initialized: true,
                ..OdomMessage::default()
            },
            translation: [2., 3., 4.],
            rotation: [0., 0., 0.6, 0.8],
            revision: 42,
            external_map: 1,
        }));
    }

    #[test]
    fn landing_target_roundtrip_keeps_session_quality_and_position() {
        roundtrip(&Message::LandingTarget(LandingTargetMessage {
            timestamp: 8.,
            session: 42,
            position: [0.1, -0.2, 0.3],
            reprojection_px: 0.7,
            confidence: 0.91,
            team: 1,
            valid: 1,
        }));
    }

    #[test]
    fn spatial_diagnostics_keep_every_active_field_at_capacity() {
        for kind in [
            viz_kind::POSE,
            viz_kind::LINE_STRIP,
            viz_kind::ARROWS,
            viz_kind::KEYPOINTS_2D,
            viz_kind::CAMERA,
            viz_kind::BAR_CHART,
            viz_kind::CLEAR,
        ] {
            let mut message =
                Box::new(VizMessage::base(kind, 3.5, "vio/prediction/pose_body_odom"));
            message.sequence = 923;
            message.color = [19, 83, 254];
            message.xyz = [1., 2., 3.];
            message.quat_xyzw = [0., 0., 0.6, 0.8];
            message.point_count = POINTS_MAX as u32;
            message.points.fill([17., -23., 42.]);
            message.arrow_count = ARROWS_MAX as u32;
            message.arrow_origins.fill([1., 2., 3.]);
            message.arrow_vectors.fill([-4., 5., -6.]);
            message.scalar_count = SCALARS_MAX as u32;
            message.scalars = [4., 5., 6., 7.];
            message.bin_count = BINS_MAX as u32;
            message.bins.fill(u64::MAX);
            message.bin_start = -23;
            message.bin_width = 17;
            roundtrip(&Message::Viz(message));
        }
        assert!(Message::Viz(Box::<VizMessage>::default()).encode().len() < 256);
    }

    #[test]
    fn voxel_chunks_keep_indices_geometry_and_order_at_capacity() {
        for kind in [
            viz_kind::VOXELS,
            viz_kind::VOXELS_ADD,
            viz_kind::VOXELS_REMOVE,
        ] {
            let mut message = Box::new(VoxelMessage::base(kind, 23.7, "plan/collision"));
            message.sequence = 123;
            message.chunk_index = 2;
            message.chunk_count = 3;
            message.voxel_size = [0.1, 0.2, 0.3];
            message.voxel_origin = [-13., 7., 9.];
            message.voxel_count = VOXELS_MAX as u32;
            message.voxels.fill([i32::MIN, 123, i32::MAX]);
            assert_eq!(Message::Voxels(message.clone()).encode().len(), 196_732);
            roundtrip(&Message::Voxels(message));
        }
    }

    #[test]
    fn diagnostic_decoders_reject_counts_beyond_ipc_budgets() {
        let mut geometry = Box::<VizMessage>::default();
        geometry.point_count = POINTS_MAX as u32 + 1;
        assert!(Message::decode(Kind::Viz, &Message::Viz(geometry).encode()).is_err());
        let mut voxels = Box::new(VoxelMessage::base(viz_kind::VOXELS, 1., "plan/collision"));
        voxels.voxel_count = VOXELS_MAX as u32 + 1;
        assert!(Message::decode(Kind::Voxels, &Message::Voxels(voxels).encode()).is_err());
    }
}

#[cfg(test)]
mod image_chunk_tests {
    use super::*;

    fn message(sensor_id: u8, chunk_index: u16, chunk_count: u16, len: usize) -> Message {
        Message::ImageChunk(ImageChunk {
            header: ImageChunkHeader {
                timestamp: 1.0,
                sensor_id,
                image_id: 42,
                chunk_index,
                chunk_count,
            },
            data: vec![7; len],
        })
    }

    #[test]
    fn typed_image_chunks_roundtrip_at_gray_and_depth_boundaries() {
        for sensor in 0..=3 {
            let size = if sensor == 3 {
                DOWN_SIZE
            } else {
                IMAGE_SIZE * if sensor == 2 { 4 } else { 1 }
            };
            let chunks = size.div_ceil(IMAGE_CHUNK_DATA_MAX) as u16;
            for index in 0..chunks {
                let len =
                    (size - usize::from(index) * IMAGE_CHUNK_DATA_MAX).min(IMAGE_CHUNK_DATA_MAX);
                let original = message(sensor, index, chunks, len);
                assert_eq!(
                    Message::decode(Kind::ImageChunk, &original.encode()).unwrap(),
                    original
                );
            }
        }
    }

    #[test]
    fn raw_image_chunk_decoder_enforces_reassembly_memory_bounds() {
        for sensor in 0..=3 {
            let size = if sensor == 3 {
                DOWN_SIZE
            } else {
                IMAGE_SIZE * if sensor == 2 { 4 } else { 1 }
            };
            let chunks = size.div_ceil(IMAGE_CHUNK_DATA_MAX) as u16;
            for (count, len) in [
                (u16::MAX, 1),
                (chunks + 1, 1),
                (chunks, IMAGE_CHUNK_DATA_MAX + 1),
            ] {
                let body = message(sensor, 0, count, len).encode();
                assert!(
                    ImageChunkHeader::decode(&body).is_err(),
                    "sensor={sensor} count={count} len={len}"
                );
                assert!(Message::decode(Kind::ImageChunk, &body).is_err());
            }
            let valid = message(sensor, 0, chunks, IMAGE_CHUNK_DATA_MAX).encode();
            assert!(ImageChunkHeader::decode(&valid).is_ok());
        }
    }

    #[test]
    fn depth_chunks_preserve_whole_float_pixels() {
        let chunks = (IMAGE_SIZE * 4).div_ceil(IMAGE_CHUNK_DATA_MAX) as u16;
        let malformed = message(2, 0, chunks, IMAGE_CHUNK_DATA_MAX - 1).encode();
        assert!(ImageChunkHeader::decode(&malformed).is_err());
        assert!(Message::decode(Kind::ImageChunk, &malformed).is_err());
        let valid = message(2, 0, chunks, IMAGE_CHUNK_DATA_MAX).encode();
        assert!(ImageChunkHeader::decode(&valid).is_ok());
    }
}
