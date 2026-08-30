//! 通用工具（对照 official `util/`）。
//!
//! 含体素降采样、法向/协方差估计、SE(3)/SO(3) 李代数运算、快速取整、空间哈希。

pub mod downsampling;
pub mod fast_floor;
pub mod lie;
pub mod normal_estimation;
pub mod ply;
pub mod vector3i_hash;
