//! 版本化消息协议：固定 48 字节帧头 + 按类型编码的消息体。
//!
//! 三个方向独占端点，消息类型与端点绑定：
//!
//! | 端点 | 方向 | 消息 |
//! |---|---|---|
//! | 传感器 | 主机 → 板端 | `Imu`、`GrayLeft`、`GrayRight`、`Depth`、`Airframe`、`Hello`、`Command`、`Goal`、`Ping`、`Bye` |
//! | 执行器 | 板端 → 主机 | `HelloAck`、`Control`、`Odometry`、`CorrectedOdometry`、`VioAttitude`、`Scalar`、`Log`、`Stats`、`Pong` |
//!
//! 所有多字节字段为小端。大消息在 [`crate::frame`] 层按
//! [`MAX_FRAGMENT`] 分片，接收端按序号重组；任一分片缺失或次序异常即丢弃
//! 整条消息并计数，不把截断图像当有效测量。
//!
//! `capture_time` 与消息体里的时间戳都取自发布端的同一时钟域。板端算法消费的
//! 传感器与状态时间沿用仿真时间轴；板端判断新鲜度只用本机单调时钟的到达时刻。

use std::io;

use firefly_pubsub::attitude::AttitudeMessage;
use firefly_pubsub::camera::{
    DepthImageMessage, GrayImageMessage, IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH,
};
use firefly_pubsub::command::CommandMessage;
use firefly_pubsub::control::ControlMessage;
use firefly_pubsub::goal::GoalMessage;
use firefly_pubsub::imu::ImuMessage;
use firefly_pubsub::log::{LogMessage, TAG_MAX, TEXT_MAX};
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::plant::AirframeMessage;
use firefly_pubsub::viz::{ENTITY_MAX, SCALARS_MAX, ScalarMessage, kind as viz_kind};

/// 帧头魔数（ASCII `FHL1`）。
pub const MAGIC: [u8; 4] = *b"FHL1";
/// 当前协议版本；版本不一致时只允许 `Hello`/`HelloAck` 用于协商失败原因。
pub const VERSION: u16 = 2;
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
            | Self::Ping
            | Self::Bye => Link::Sensor,
            Self::HelloAck
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

    /// 从消息体前缀解析；校验块号、块数与传感器号。
    ///
    /// # Errors
    /// 长度不足、块数为零、块号越界或传感器号非法。
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
            || header.sensor_id > 2
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "image chunk header",
            ));
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
    /// 遥测标量。
    Scalar(ScalarMessage),
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
                for value in message.data {
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
                let header = ImageChunkHeader::decode(body)?;
                let data = body[IMAGE_CHUNK_HEADER_LEN..].to_vec();
                if data.len() > IMAGE_CHUNK_DATA_MAX {
                    return Err(invalid("image chunk payload"));
                }
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
                let timestamp = input.f64()?;
                let mut values = [0.0; 10];
                for value in &mut values {
                    *value = input.f64()?;
                }
                let is_initialized = input.u8()? != 0;
                let message = OdomMessage {
                    timestamp,
                    position_x: values[0],
                    position_y: values[1],
                    position_z: values[2],
                    velocity_x: values[3],
                    velocity_y: values[4],
                    velocity_z: values[5],
                    quat_x: values[6],
                    quat_y: values[7],
                    quat_z: values[8],
                    quat_w: values[9],
                    is_initialized,
                };
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
            }),
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
                    trace_id_hi: 0,
                    trace_id_lo: 0,
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
        return Err(invalid("frame version"));
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
