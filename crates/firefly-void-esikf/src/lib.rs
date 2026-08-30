//! ESIKF 核心：前向/后向传播、顺序更新与协方差递推。
//!
//! 技术蓝本为 FAST-LIVO2 论文第 IV 节 Algorithm 1
//! （`~/Projects/fast_livo2/FAST-LIVO2-paper.pdf`）与官方实现
//! `src/IMU_Processing.cpp` / `src/voxel_map.cpp`：
//! - [`propagator`]：IMU 前向传播与扫描内后向传播；
//! - [`update`]：顺序更新框架（Algorithm 1 主体）与 [`update::MeasurementModel`] trait。
//!
//! 测量模型 trait 由 `firefly-void-measure` 在后续阶段实现，本 crate 不依赖测量 crate。

pub mod propagator;
pub mod update;
