//! Rerun 数据出口：小型标量、空间诊断和体素分别使用有界 IPC 话题。
//! 计算进程只发布测量时间与数值，录制策略及显示由 firefly-viz 持有。

use iceoryx2::prelude::*;
use std::cell::Cell;

/// 统一可视化话题（Rust 发布端 → Python `firefly-viz` 订阅端）。
pub const VIZ_TOPIC: &str = "Firefly/Viz";

/// 消息类型（`VizMessage::kind` 取值，与 `firefly_viz/messages.py` 一致）。
pub mod kind {
    /// 位姿（`vio/odom`、`gt/pose` 等刚体变换实体）。
    pub const POSE: u32 = 1;
    /// 折线（轨迹增量段、全局路径）。
    pub const LINE_STRIP: u32 = 2;
    /// 占据体素网格**全量快照**（静态先验地图；消费端以此**替换**自己的集合）。
    pub const VOXELS: u32 = 3;
    /// 占据体素**增量新增**（感知地图、动态障碍；消费端并入集合）。
    pub const VOXELS_ADD: u32 = 8;
    /// 占据体素**增量删除**（滑出观察窗、衰减回未知；消费端从集合移除）。
    pub const VOXELS_REMOVE: u32 = 9;
    /// 标量曲线（`db_size`、`track_avg_len` 等单值序列）。
    pub const SCALARS: u32 = 4;
    /// 直方图（`track_length` 桶计数）。
    pub const BAR_CHART: u32 = 5;
    /// 3D 箭头（障碍平面法线、轨迹速度向量）。
    pub const ARROWS: u32 = 6;
    /// 递归清除指定的非空实体路径。
    pub const CLEAR: u32 = 7;
    /// 图像像素坐标点集：`points = [u, v, track_id]`；`scalars = [宽, 高, 总点数]`。
    /// `point_count` 为实际携带点数，允许小于总数；空点集清除该帧标记。
    pub const KEYPOINTS_2D: u32 = 10;
    /// 数值矩阵：`scalars=[行数,列数]`，`points` 展平后为行主序元素，
    /// `point_count` 为元素数（上限 `3 * POINTS_MAX`）。
    pub const MATRIX: u32 = 11;
    /// 固定相机标定：scalars=[fx,fy,cx,cy]，points[0]=[width,height,0]，pose 为 body←CV camera。
    pub const CAMERA: u32 = 12;
}

/// VIO 左/右相机当前活动点；时间戳为原始相机时钟，不是滤波器时间。
pub const VIO_KEYPOINT_ENTITIES: [&str; 2] =
    ["vio/debug/keypoints/left", "vio/debug/keypoints/right"];

/// 实体路径字节上限（64 字节，UTF-8；超出截断，见 [`VizMessage::set_entity`]）。
pub const ENTITY_MAX: usize = 64;
/// 服务 `subscriber_max_buffer_size`（订阅端环形缓冲历史上限；多实体 10Hz
/// 突发下防溢出丢帧，订阅端 `buffer_size` 不得超过该值）。
pub const VIZ_BUFFER_SIZE: usize = 256;
/// 折线点数上限（512；发布端超限硬断言，调用方必须分段或按用途限采样）。
pub const POINTS_MAX: usize = 512;
/// 箭头数上限（256，起点 + 向量各一数组；轨迹速度箭头 100 采样点、障碍
/// 平面法线均远小于此）。
pub const ARROWS_MAX: usize = 256;
/// 单个体素分块的索引数上限；完整更新可包含多个连续分块。
pub const VOXELS_MAX: usize = 16_384;
/// 直方图桶数上限（64；vio `track_length` 直方图 21 桶）。
pub const BINS_MAX: usize = 64;
/// 标量值个数上限（4）。
pub const SCALARS_MAX: usize = 4;

/// 可视化消息（扁平 tagged-union：`kind` 区分类型，其余字段按类型取用）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyVizMessageV2")]
pub struct VizMessage {
    /// 消息类型（[`kind`] 常量）。
    pub kind: u32,
    /// 实体路径有效字节数（[`Self::set_entity`] 写入）。
    pub entity_len: u32,
    /// 颜色 RGB（`[r, g, b]`，按 kind 取用）。
    pub color: [u8; 3],
    /// 仿真时钟时间戳（秒，`sim_time` 时间轴）。
    pub timestamp: f64,
    /// 实体路径（UTF-8，`[u8; 64]` 定长 + `len`）。
    pub entity: [u8; ENTITY_MAX],
    /// 位置（pose）。
    pub xyz: [f64; 3],
    /// Hamilton xyzw、机体→父坐标系；与 `OdomMessage` 的分量解释边界一致。
    pub quat_xyzw: [f64; 4],
    /// 折线或图像点集；`matrix` 复用其连续 f64 存储（见 [`kind::MATRIX`]）。
    pub points: [[f64; 3]; POINTS_MAX],
    /// `points` 有效点数；`matrix` 时为展平后的有效元素数。
    pub point_count: u32,
    /// 箭头数（arrows）。
    pub arrow_count: u32,
    /// 箭头起点（arrows；`arrow_origins[..arrow_count]`）。
    pub arrow_origins: [[f64; 3]; ARROWS_MAX],
    /// 箭头向量（arrows；`arrow_vectors[..arrow_count]`）。
    pub arrow_vectors: [[f64; 3]; ARROWS_MAX],
    /// 标量值或点集/矩阵元数据（`scalars[..scalar_count]`）。
    pub scalars: [f64; SCALARS_MAX],
    /// `scalars` 有效数。
    pub scalar_count: u32,
    /// 直方图桶计数（`bar_chart`；`bins[..bin_count]`）。
    pub bins: [u64; BINS_MAX],
    /// `bins` 有效数。
    pub bin_count: u32,
    /// 直方图首桶下界（`bar_chart`）。
    pub bin_start: i64,
    /// 桶宽（`bar_chart`）。
    pub bin_width: i64,
    /// 发布器内连续序号，用于检测订阅缓冲覆盖。
    pub sequence: u64,
}

impl Default for VizMessage {
    #[allow(clippy::large_stack_arrays)]
    fn default() -> Self {
        Self {
            kind: 0,
            entity_len: 0,
            color: [0; 3],
            timestamp: -1.0,
            entity: [0u8; ENTITY_MAX],
            xyz: [0.0; 3],
            quat_xyzw: [0.0; 4],
            points: [[0.0; 3]; POINTS_MAX],
            point_count: 0,
            arrow_count: 0,
            arrow_origins: [[0.0; 3]; ARROWS_MAX],
            arrow_vectors: [[0.0; 3]; ARROWS_MAX],
            scalars: [0.0; SCALARS_MAX],
            scalar_count: 0,
            bins: [0; BINS_MAX],
            bin_count: 0,
            bin_start: 0,
            bin_width: 0,
            sequence: 0,
        }
    }
}

impl VizMessage {
    /// 构造已设 `kind`/`timestamp`/`entity` 的默认实例（其余字段零值，
    /// 按 kind 填充 payload 字段）。
    #[must_use]
    pub fn base(kind: u32, timestamp: f64, entity: &str) -> Self {
        let mut m = Self {
            kind,
            timestamp,
            ..Self::default()
        };
        m.set_entity(entity);
        m
    }

    /// 构造行主序数值矩阵，保持扁平 IPC 布局。
    ///
    /// # Panics
    /// 行列数为零、元素数量与尺寸不符或超出定长载荷。
    #[must_use]
    pub fn matrix(timestamp: f64, entity: &str, rows: usize, cols: usize, values: &[f64]) -> Self {
        assert!(rows > 0 && cols > 0 && rows.checked_mul(cols) == Some(values.len()));
        assert!(values.len() <= 3 * POINTS_MAX);
        let mut msg = Self::base(kind::MATRIX, timestamp, entity);
        msg.scalars[..2].copy_from_slice(&[rows as f64, cols as f64]);
        msg.scalar_count = 2;
        msg.points.as_flattened_mut()[..values.len()].copy_from_slice(values);
        msg.point_count = values.len() as u32;
        msg
    }

    /// 设置实体路径（UTF-8，超长截断；内部以字符边界保护不切碎多字节序列）。
    pub fn set_entity(&mut self, entity: &str) {
        (self.entity, self.entity_len) = entity_bytes(entity);
    }

    /// 实体路径（按 `entity_len` 截断，非法 UTF-8 以替换符呈现）。
    #[must_use]
    pub fn entity(&self) -> &str {
        let n = (self.entity_len as usize).min(ENTITY_MAX);
        std::str::from_utf8(&self.entity[..n]).unwrap_or("\u{FFFD}")
    }
}

/// 标量话题；每条消息固定 128 字节。
pub const SCALAR_TOPIC: &str = "Firefly/VizScalars";
/// 1kHz IMU 双标量与相机更新突发的独立接收容量；每条仅 128 字节。
pub const SCALAR_BUFFER_SIZE: usize = 1024;
/// 体素快照与增量话题。
pub const VOXEL_TOPIC: &str = "Firefly/VizVoxels";
/// 单一 planner 发布体素；独立缓冲限制大载荷的共享内存预算。
pub const VOXEL_BUFFER_SIZE: usize = 32;
/// 所有可视化话题允许的发布者数。
pub const VIZ_MAX_PUBLISHERS: usize = 8;

/// 不携带图像、点集或体素的标量载荷。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyScalarMessage")]
pub struct ScalarMessage {
    pub kind: u32,
    pub entity_len: u32,
    pub timestamp: f64,
    pub entity: [u8; ENTITY_MAX],
    pub scalars: [f64; SCALARS_MAX],
    pub scalar_count: u32,
    pub sequence: u64,
}

impl ScalarMessage {
    /// 构造指定实体和测量时刻的空标量消息。
    #[must_use]
    pub fn base(timestamp: f64, entity: &str) -> Self {
        let (entity, entity_len) = entity_bytes(entity);
        Self {
            kind: kind::SCALARS,
            entity_len,
            timestamp,
            entity,
            scalars: [0.; SCALARS_MAX],
            scalar_count: 0,
            sequence: 0,
        }
    }
}

/// 体素有独立容量与通道，不放大标量和特征点消息。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyVoxelMessage")]
pub struct VoxelMessage {
    pub kind: u32,
    pub entity_len: u32,
    pub timestamp: f64,
    pub entity: [u8; ENTITY_MAX],
    pub voxels: [[i32; 3]; VOXELS_MAX],
    pub voxel_count: u32,
    pub voxel_size: [f32; 3],
    pub voxel_origin: [f32; 3],
    /// 发布器内单调序号；接收端检测缺块、重启和乱序。
    pub sequence: u64,
    /// 本次同实体、同类型、同测量时刻更新的分块序号（从零开始）。
    pub chunk_index: u32,
    /// 本次更新的总分块数；空快照也发送一个空块。
    pub chunk_count: u32,
}

impl VoxelMessage {
    /// 构造一个空的单块更新；序号由发布器填写。
    #[must_use]
    #[allow(clippy::large_stack_arrays)]
    pub fn base(kind: u32, timestamp: f64, entity: &str) -> Self {
        let (entity, entity_len) = entity_bytes(entity);
        Self {
            kind,
            entity_len,
            timestamp,
            entity,
            voxels: [[0; 3]; VOXELS_MAX],
            voxel_count: 0,
            voxel_size: [0.; 3],
            voxel_origin: [0.; 3],
            sequence: 0,
            chunk_index: 0,
            chunk_count: 1,
        }
    }
}

fn entity_bytes(value: &str) -> ([u8; ENTITY_MAX], u32) {
    let mut n = value.len().min(ENTITY_MAX);
    while !value.is_char_boundary(n) {
        n -= 1;
    }
    let mut bytes = [0; ENTITY_MAX];
    bytes[..n].copy_from_slice(&value.as_bytes()[..n]);
    (bytes, n as u32)
}

/// 按载荷类型路由至对应 IPC 话题，避免将小消息装入最大载荷的 enum。
pub trait VizPayload {
    /// # Errors
    /// IPC 借出或发送失败。
    fn publish_to(
        self,
        publisher: &VizPublisher,
    ) -> Result<crate::trace::TraceContext, firefly_error::Error>;
}

/// 小型诊断发布器；只有显式启用体素的 planner 分配大载荷发布缓冲。
pub struct VizPublisher {
    geometry: crate::publish::Publisher<VizMessage>,
    scalars: crate::publish::Publisher<ScalarMessage>,
    voxels: Option<crate::publish::Publisher<VoxelMessage>>,
    voxel_sequence: Cell<u64>,
    geometry_sequence: Cell<u64>,
    scalar_sequence: Cell<u64>,
}

impl VizPublisher {
    /// # Errors
    /// 任一服务或发布端创建失败。
    pub fn new(node: &crate::node::IpcNode) -> Result<Self, firefly_error::Error> {
        use crate::publish::Publisher;
        Ok(Self {
            geometry: Publisher::with_topic_buffer_publishers(
                node,
                VIZ_TOPIC,
                Some(VIZ_BUFFER_SIZE),
                Some(VIZ_MAX_PUBLISHERS),
            )?,
            scalars: Publisher::with_topic_buffer_publishers(
                node,
                SCALAR_TOPIC,
                Some(SCALAR_BUFFER_SIZE),
                Some(VIZ_MAX_PUBLISHERS),
            )?,
            voxels: None,
            voxel_sequence: Cell::new(0),
            geometry_sequence: Cell::new(0),
            scalar_sequence: Cell::new(0),
        })
    }

    /// 打开诊断与单发布者体素通道。
    ///
    /// # Errors
    /// 任一服务或发布端创建失败。
    pub fn with_voxels(node: &crate::node::IpcNode) -> Result<Self, firefly_error::Error> {
        let mut publisher = Self::new(node)?;
        publisher.voxels = Some(crate::publish::Publisher::with_topic_buffer_publishers(
            node,
            VOXEL_TOPIC,
            Some(VOXEL_BUFFER_SIZE),
            Some(1),
        )?);
        Ok(publisher)
    }

    /// 发布失败记入结构化日志；诊断链路不得阻塞算法。
    /// # Errors
    /// IPC 借出或发送失败。
    pub fn publish(
        &self,
        msg: impl VizPayload,
    ) -> Result<crate::trace::TraceContext, firefly_error::Error> {
        let result = msg.publish_to(self);
        if let Err(error) = &result {
            log::warn!("可视化 IPC 发布失败: {error}");
        }
        result
    }
}

impl VizPayload for VizMessage {
    fn publish_to(
        mut self,
        publisher: &VizPublisher,
    ) -> Result<crate::trace::TraceContext, firefly_error::Error> {
        self.sequence = publisher.geometry_sequence.get();
        publisher
            .geometry_sequence
            .set(self.sequence.wrapping_add(1));
        publisher.geometry.publish(self)
    }
}
impl VizPayload for ScalarMessage {
    fn publish_to(
        mut self,
        publisher: &VizPublisher,
    ) -> Result<crate::trace::TraceContext, firefly_error::Error> {
        self.sequence = publisher.scalar_sequence.get();
        publisher.scalar_sequence.set(self.sequence.wrapping_add(1));
        publisher.scalars.publish(self)
    }
}
impl VizPayload for VoxelMessage {
    fn publish_to(
        mut self,
        publisher: &VizPublisher,
    ) -> Result<crate::trace::TraceContext, firefly_error::Error> {
        let voxels = publisher.voxels.as_ref().ok_or_else(|| {
            firefly_error::Error::new(firefly_error::ErrorKind::InvalidArgument, "体素通道未启用")
        })?;
        self.sequence = publisher.voxel_sequence.get();
        publisher.voxel_sequence.set(self.sequence.wrapping_add(1));
        voxels.publish(self)
    }
}

/// 订阅完整可视化突发；与发布端使用同一服务容量，支持任意启动顺序。
///
/// # Errors
/// 服务或订阅器创建失败时返回 IPC 错误。
pub fn subscribe(
    node: &crate::node::IpcNode,
) -> Result<crate::subscriber::Subscriber<VizMessage>, firefly_error::Error> {
    subscribe_with_topic(node, VIZ_TOPIC).map_err(|error| error.with_context("topic", VIZ_TOPIC))
}

fn subscribe_with_topic(
    node: &crate::node::IpcNode,
    topic: &str,
) -> Result<crate::subscriber::Subscriber<VizMessage>, firefly_error::Error> {
    use firefly_error::{Error, ErrorKind};
    let name = topic
        .try_into()
        .map_err(|e| Error::new(ErrorKind::InvalidArgument, format!("非法可视化话题：{e:?}")))?;
    let service = node
        .service_builder(&name)
        .publish_subscribe::<VizMessage>()
        .max_publishers(VIZ_MAX_PUBLISHERS)
        .user_header::<crate::trace::TraceContext>()
        .subscriber_max_buffer_size(VIZ_BUFFER_SIZE)
        .open_or_create()
        .map_err(|e| Error::new(ErrorKind::Internal, format!("打开可视化话题失败：{e:?}")))?;
    let subscriber = service
        .subscriber_builder()
        .buffer_size(VIZ_BUFFER_SIZE)
        .create()
        .map_err(|e| Error::new(ErrorKind::Internal, format!("订阅可视化话题失败：{e:?}")))?;
    Ok(crate::subscriber::Subscriber::from_inner(subscriber))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 编译期断言：满足 iceoryx2 零拷贝约束。
    fn assert_zero_copy_send<T: ZeroCopySend>() {}

    #[test]
    fn matrix_preserves_row_major_shape_and_values() {
        let msg = VizMessage::matrix(1.5, "vio/covariance", 2, 3, &[1., 2., 3., 4., 5., 6.]);
        assert_eq!(msg.kind, kind::MATRIX);
        assert_eq!(msg.scalar_count, 2);
        assert_eq!(msg.scalars[..2], [2., 3.]);
        assert_eq!(msg.point_count, 6);
        assert_eq!(msg.points[..2], [[1., 2., 3.], [4., 5., 6.]]);
    }

    #[test]
    fn subscriber_first_retains_geometry_burst() {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let topic = format!("Firefly/Test/Viz/{unique}");
        let node = crate::node::create_node().unwrap();
        let subscriber = subscribe_with_topic(&node, &topic).unwrap();
        let publisher =
            crate::publish::Publisher::with_topic_and_buffer(&node, &topic, Some(VIZ_BUFFER_SIZE))
                .unwrap();
        publisher
            .publish(VizMessage::base(
                kind::KEYPOINTS_2D,
                1.0,
                VIO_KEYPOINT_ENTITIES[0],
            ))
            .unwrap();
        for _ in 0..20 {
            publisher
                .publish(VizMessage::base(kind::POSE, 1.0, "vio/odom"))
                .unwrap();
        }
        let first = subscriber.receive().unwrap().expect("关键点快照");
        assert_eq!(first.kind, kind::KEYPOINTS_2D);
        drop(first);
        let mut count = 0;
        while subscriber.receive().unwrap().is_some() {
            count += 1;
        }
        assert_eq!(count, 20);
    }

    #[test]
    fn scalar_queue_retains_one_khz_imu_burst_in_order() {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let topic = format!("Firefly/Test/Scalars/{unique}");
        let node = crate::node::create_node().unwrap();
        let subscriber = crate::subscriber::Subscriber::<ScalarMessage>::with_topic_and_buffer(
            &node,
            &topic,
            SCALAR_BUFFER_SIZE,
        )
        .unwrap();
        let publisher = crate::publish::Publisher::with_topic_and_buffer(
            &node,
            &topic,
            Some(SCALAR_BUFFER_SIZE),
        )
        .unwrap();
        // 250ms IMU 双通道突发，覆盖录制接收线程短暂停顿。
        for sequence in 0_u32..500 {
            let mut message = ScalarMessage::base(f64::from(sequence / 2) * 0.001, "vio/input/imu");
            message.sequence = u64::from(sequence);
            publisher.publish(message).unwrap();
        }
        for sequence in 0..500 {
            let sample = subscriber.receive().unwrap().expect("IMU scalar");
            assert_eq!(sample.sequence, sequence);
        }
        assert!(subscriber.receive().unwrap().is_none());
    }

    #[test]
    fn viz_message_is_plain_old_data() {
        assert_zero_copy_send::<VizMessage>();
        assert_eq!(std::mem::size_of::<VizMessage>(), 25_312);
        assert_eq!(std::mem::size_of::<ScalarMessage>(), 128);
        assert_eq!(std::mem::size_of::<VoxelMessage>(), 196_736);
        assert_zero_copy_send::<ScalarMessage>();
        assert_zero_copy_send::<VoxelMessage>();
        let m = VizMessage::default();
        assert_eq!(m.kind, 0);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
        assert_eq!(m.entity_len, 0);
    }

    #[test]
    fn entity_round_trips() {
        let mut m = VizMessage::default();
        m.set_entity("vio/odom");
        assert_eq!(m.entity(), "vio/odom");
        assert_eq!(m.entity_len, 8);
    }

    #[test]
    fn entity_truncates_at_char_boundary() {
        let mut m = VizMessage::default();
        // 63 个 ASCII + 1 个 3 字节 UTF-8 字符 → 66 字节，须截断到字符边界
        let s = format!("{}{}", "a".repeat(ENTITY_MAX - 1), "中");
        m.set_entity(&s);
        assert!(m.entity_len as usize <= ENTITY_MAX);
        assert!(m.entity().is_ascii());
        assert_eq!(m.entity(), "a".repeat(ENTITY_MAX - 1));
    }

    #[test]
    fn entity_truncates_long_ascii() {
        let mut m = VizMessage::default();
        let s = "b".repeat(ENTITY_MAX + 10);
        m.set_entity(&s);
        assert_eq!(m.entity_len as usize, ENTITY_MAX);
        assert_eq!(m.entity(), "b".repeat(ENTITY_MAX));
    }

    #[test]
    #[allow(clippy::float_cmp)] // 零填充断言：精确比较即意图
    fn default_is_zero_filled() {
        let m = VizMessage::default();
        assert!(m.points.iter().all(|p| p == &[0.0; 3]));
        assert!(m.arrow_origins.iter().all(|p| p == &[0.0; 3]));
        assert!(m.arrow_vectors.iter().all(|p| p == &[0.0; 3]));
        assert!(m.bins.iter().all(|&b| b == 0));
        assert_eq!(m.point_count, 0);
        assert_eq!(m.arrow_count, 0);
        assert_eq!(m.scalar_count, 0);
        assert_eq!(m.bin_count, 0);
    }
}
