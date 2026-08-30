//! 可视化消息（iceoryx2 zero-copy，Rust → `firefly-viz` Python 进程）。
//!
//! 单一话题 [`VIZ_TOPIC`] 承载全部可视化数据：计算线程（vio/planner）只做
//! 零拷贝发布，rerun 写入统一由 Python 进程 `firefly-viz` 消费（计算线程
//! 零 IO）。消息为**单一扁平结构体**——`kind` 区分消息类型，各类型字段平铺，
//! 无效字段必须保持零值（iceoryx2 `ZeroCopySend` derive 拒绝 enum/union，
//! 见 `iceoryx2-bb-derive-macros` 的 `zero_copy_send_compile_tests.rs`；
//! Python 端以 `ctypes.Structure` 平铺镜像，见 `firefly_viz/messages.py`）。
//!
//! 布局约束：`#[repr(C)]` 定长、无堆指针、`'static`、无 `Drop`。总大小由
//! 最大的体素数组决定（16384 体素 × 12B ≈ 192KB，与 `DepthImageMessage`
//! 307KB 同量级，iceoryx2 共享内存无压力）。

use iceoryx2::prelude::*;

/// 统一可视化话题（Rust 发布端 → Python `firefly-viz` 订阅端）。
pub const VIZ_TOPIC: &str = "Firefly/Viz";

/// 消息类型（`VizMessage::kind` 取值，与 `firefly_viz/messages.py` 一致）。
pub mod kind {
    /// 位姿（`vio/odom`、`gt/pose` 等刚体变换实体）。
    pub const POSE: u32 = 1;
    /// 折线（轨迹增量段、全局路径）。
    pub const LINE_STRIP: u32 = 2;
    /// 占据体素网格（静态先验地图、感知地图、动态障碍）。
    pub const VOXELS: u32 = 3;
    /// 标量曲线（`db_size`、`track_avg_len` 等单值序列）。
    pub const SCALARS: u32 = 4;
    /// 直方图（`track_length` 桶计数）。
    pub const BAR_CHART: u32 = 5;
    /// 3D 箭头（障碍平面法线、轨迹速度向量）。
    pub const ARROWS: u32 = 6;
    /// 清除实体（空 entity 表示全清；当前无发布端，预留扩展点）。
    pub const CLEAR: u32 = 7;
}

/// 实体路径字节上限（64 字节，UTF-8；超出截断，见 [`VizMessage::set_entity`]）。
pub const ENTITY_MAX: usize = 64;
/// 服务 `subscriber_max_buffer_size`（订阅端环形缓冲历史上限；多实体 10Hz
/// 突发下防溢出丢帧，订阅端 `buffer_size` 不得超过该值）。
pub const VIZ_BUFFER_SIZE: usize = 256;
/// 折线点数上限（512；vio 增量两点段与 A* 简化路径均远小于此，发布端超限硬断言）。
pub const POINTS_MAX: usize = 512;
/// 箭头数上限（256，起点 + 向量各一数组；轨迹速度箭头 100 采样点、障碍
/// 平面法线均远小于此）。
pub const ARROWS_MAX: usize = 256;
/// 体素索引数上限（16384 × 12B = 192KB；感知地图 80×35×13=36400 体素全
/// 占据是理论上限，实际数千——发布端超限截断并 `log::warn` 一次，锁存防刷屏）。
pub const VOXELS_MAX: usize = 16_384;
/// 直方图桶数上限（64；vio `track_length` 直方图 21 桶）。
pub const BINS_MAX: usize = 64;
/// 标量值个数上限（4）。
pub const SCALARS_MAX: usize = 4;

/// 可视化消息（扁平 tagged-union：`kind` 区分类型，其余字段按类型取用）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyVizMessage")]
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
    /// 姿态四元数 xyzw（pose；JPL 分量顺序，与 `OdomMessage` 一致）。
    pub quat_xyzw: [f64; 4],
    /// 折线点集（`line_strip`；`points[..point_count]`）。
    pub points: [[f64; 3]; POINTS_MAX],
    /// `points` 有效数。
    pub point_count: u32,
    /// 箭头数（arrows）。
    pub arrow_count: u32,
    /// 箭头起点（arrows；`arrow_origins[..arrow_count]`）。
    pub arrow_origins: [[f64; 3]; ARROWS_MAX],
    /// 箭头向量（arrows；`arrow_vectors[..arrow_count]`）。
    pub arrow_vectors: [[f64; 3]; ARROWS_MAX],
    /// 体素索引（`[x, y, z]`，voxels；`voxels[..voxel_count]`）。
    pub voxels: [[i32; 3]; VOXELS_MAX],
    /// `voxels` 有效数。
    pub voxel_count: u32,
    /// 体素尺寸（米）。
    pub voxel_size: [f32; 3],
    /// 体素网格原点（世界坐标，米）。
    pub voxel_origin: [f32; 3],
    /// 标量值（scalars；`scalars[..scalar_count]`）。
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
}

impl Default for VizMessage {
    // 定长共享内存消息（约 217KB），大数组是设计使然
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
            voxels: [[0; 3]; VOXELS_MAX],
            voxel_count: 0,
            voxel_size: [0.0; 3],
            voxel_origin: [0.0; 3],
            scalars: [0.0; SCALARS_MAX],
            scalar_count: 0,
            bins: [0; BINS_MAX],
            bin_count: 0,
            bin_start: 0,
            bin_width: 0,
        }
    }
}

impl VizMessage {
    /// 构造已设 `kind`/`timestamp`/`entity` 的默认实例（其余字段零值，
    /// 按 kind 填充 payload 字段）。颜色默认白色，需配色时再覆写。
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

    /// 设置实体路径（UTF-8，超长截断；内部以字符边界保护不切碎多字节序列）。
    pub fn set_entity(&mut self, entity: &str) {
        let bytes = entity.as_bytes();
        let mut n = bytes.len().min(ENTITY_MAX);
        // 回退到合法字符边界：先退过续字节（0b10xxxxxx），若停在多字节序列
        // 首字节（0b11xxxxxx）再退一位（该序列整体超出上限）
        while n > 0 && (bytes[n - 1] & 0xC0) == 0x80 {
            n -= 1;
        }
        if n > 0 && (bytes[n - 1] & 0xC0) == 0xC0 {
            n -= 1;
        }
        self.entity[..n].copy_from_slice(&bytes[..n]);
        self.entity_len = n as u32;
    }

    /// 实体路径（按 `entity_len` 截断，非法 UTF-8 以替换符呈现）。
    #[must_use]
    pub fn entity(&self) -> &str {
        let n = (self.entity_len as usize).min(ENTITY_MAX);
        std::str::from_utf8(&self.entity[..n]).unwrap_or("\u{FFFD}")
    }
}

/// 可视化发布器（话题 [`VIZ_TOPIC`]，泛型核心的命名封装）。
pub struct VizPublisher(pub(crate) crate::publish::Publisher<VizMessage>);

impl VizPublisher {
    /// 打开统一可视化话题的发布器（服务 `subscriber_max_buffer_size` = 256，
    /// 防多实体 10Hz 突发溢出；订阅端 `buffer_size` 不得超过该值）。
    ///
    /// # Errors
    /// 见 [`crate::publish::Publisher::with_topic_and_buffer`]。
    pub fn new(node: &crate::node::IpcNode) -> Result<Self, firefly_error::Error> {
        Ok(Self(crate::publish::Publisher::with_topic_and_buffer(
            node,
            VIZ_TOPIC,
            Some(VIZ_BUFFER_SIZE),
        )?))
    }

    /// 发布一条可视化消息（trace 上下文自动注入，见
    /// [`crate::publish::Publisher::publish`]）。
    ///
    /// # Errors
    /// 见 [`crate::publish::Publisher::publish`]。
    pub fn publish(
        &self,
        msg: VizMessage,
    ) -> Result<crate::trace::TraceContext, firefly_error::Error> {
        self.0.publish(msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 编译期断言：满足 iceoryx2 零拷贝约束。
    fn assert_zero_copy_send<T: ZeroCopySend>() {}

    #[test]
    fn viz_message_is_plain_old_data() {
        assert_zero_copy_send::<VizMessage>();
        // 定长布局（8 字节对齐）：4(kind)+4(len)+3(color)+1(pad)+8(ts)+
        // 64(entity)+24(xyz)+32(quat)+12288(points)+4+4+6144(origins)+
        // 6144(vectors)+196608(voxels)+4+12+12+32(scalars)+4+4(pad)+
        // 512(bins)+4+4(pad)+8+8 = 221944
        assert_eq!(std::mem::size_of::<VizMessage>(), 221_944);
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
        assert!(m.voxels.iter().all(|v| v == &[0; 3]));
        assert!(m.bins.iter().all(|&b| b == 0));
        assert_eq!(m.point_count, 0);
        assert_eq!(m.arrow_count, 0);
        assert_eq!(m.voxel_count, 0);
        assert_eq!(m.scalar_count, 0);
        assert_eq!(m.bin_count, 0);
    }
}
