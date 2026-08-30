//! 低频全局重定位与 VIO 的松耦合融合。
//!
//! - `filter`：误差态 EKF，状态为 `VIO→全局` 的漂移 `SE(3)`，预测由 VIO 增量驱动，
//!   观测为 GICP 全局位姿，`R = h⁻¹`，`chi2` 门控与 Joseph 更新。
//! - `reloc`：`depth→PointCloud → preprocess → GICP` 的重定位封装。

#![allow(clippy::pedantic)]

pub mod filter;
pub mod reloc;

pub use filter::{FusionFilter, FusionOptions, RelocGate};
pub use reloc::{GlobalRelocalizer, RelocOptions, RelocResult};
