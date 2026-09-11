//! 低频全局重定位与 VIO 的松耦合融合。
//!
//! - `filter`：误差态 EKF，状态为 `VIO→全局` 的漂移 `SE(3)`，预测由 VIO 增量驱动，
//!   观测为几何/视觉全局位姿（`R = h⁻¹`），`chi2` 门控与 Joseph 更新。
//! - `reloc`：`depth→PointCloud → preprocess → GICP` 的几何重定位封装。
//! - `convert`：位姿表示转换（各融合消费端共用）。
//!
//! 数值滤波代码：单字符矩阵名（`h/z/r/k`）与有限比较为领域惯例。
#![allow(clippy::many_single_char_names, clippy::float_cmp)]

pub mod config;
pub mod convert;
pub mod filter;
pub mod reloc;

pub use config::LocalizationConfig;
pub use convert::{matrix_to_odom, odom_to_matrix};
pub use filter::{FusionFilter, FusionOptions, GateProfile, Observation, RelocGate};
pub use reloc::{GlobalRelocalizer, RelocOptions, RelocResult};
