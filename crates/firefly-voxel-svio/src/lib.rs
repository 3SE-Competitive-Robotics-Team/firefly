//! 体素选点（对照 Voxel-SVIO，Yuan et al. RA-L 2025）。
//!
//! 每帧只把相机正看着的体素里的地图点喂给 SLAM 更新，每体素限量取点：
//! 算力自动集中到新帧约束，空间分布均匀，老帧点自然落选。
//!
//! 本 crate 只做分布索引与访问节拍（纯管理，不碰滤波数学）：调用方
//! （`firefly-vio`）持有路标位置，本结构存 `featid → 体素` 索引。

pub mod map;
pub mod options;

pub use map::{VoxelKey, VoxelMap};
pub use options::VoxelOptions;
