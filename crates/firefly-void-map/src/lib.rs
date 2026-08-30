//! DIVO 自适应体素地图（P2，论文 V 节 Local Mapping 完整实现）。
//!
//! 技术蓝本为 FAST-LIVO2（`~/Projects/fast_livo2/`）：
//! - 地图结构（V-A）：[`voxel::VoxelMap`] 哈希根体素 + [`octree::OctoNode`] 八叉细分；
//! - 几何构建与更新（V-B）：[`plane::fit_plane`] SVD 平面判据与 `Σ_nq`；
//! - 视觉地图点与补丁（V-C/D）：[`visual_point`] + [`image_patch`] 补丁金字塔与 NCC；
//! - 法向精化（V-E）：[`normal_refine`] 仿射扭曲 + 光度最小化；
//! - 可见性查询与光线投射（VII-A）：[`raycast`]。
//!
//! 接口契约见 [`voxel::VoxelMap`]（P3 测量模型与 P4 接线使用）。

pub mod image_patch;
pub mod normal_refine;
pub mod octree;
pub mod options;
pub mod plane;
pub mod raycast;
pub mod visual_point;
pub mod voxel;

pub use options::{PlaneOptions, VoxelMapOptions};
pub use plane::{VoxelPlane, fit_plane};
pub use visual_point::{PatchObservation, VisualPoint, VisualPointView};
pub use voxel::{VoxelKey, VoxelMap};
