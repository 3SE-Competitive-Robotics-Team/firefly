//! 点云抽象（对照 official `points/`）。
//!
//! 核心算法（KdTree、法向估计、降采样）面向 [`traits::PointCloud`] trait 编程，
//! concrete [`point_cloud::PointCloud`] 是默认实现，亦可为 `Vec<Vector3/Vector4>` 等
//! 具体点类型实现该 trait。

pub mod point_cloud;
pub mod traits;

pub use point_cloud::PointCloud;
