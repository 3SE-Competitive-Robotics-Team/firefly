//! 坐标基础：`T_target_source` 把 source 坐标转换到 target，长度单位为米。
//! 跨坐标系组合必须使用带方向标识的刚体变换；算法内部可使用矩阵。
mod frames;
pub mod se3;
pub use frames::{FrameId, FrameTree, RigidTransform};
