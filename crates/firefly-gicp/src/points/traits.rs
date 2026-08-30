//! 点云 trait 抽象（对照 official `points/traits.hpp`）。
//!
//! 官方以 `traits::Traits<T>` 特化 + 自由函数 `traits::size/point/...` 实现能力分派；
//! Rust 侧用 trait 方法直接表达同一能力面。`point/normal/cov` 一律返回四元齐次量：
//! 点 `(x, y, z, 1)`、法向 `(nx, ny, nz, 0)`、协方差仅填左上 3×3，底行右列补零。

use nalgebra::{Matrix4, Vector4};

/// 点云只读能力面（对照 `traits::Traits<T>` 的读取类方法）。
///
/// 具体点类型只需实现 [`num_points`] 与 [`point`]，其余方法提供默认实现；
/// 不携带法向/协方差的类型令 [`has_normals`]/[`has_covs`] 返回 `false`，
/// 此时 [`normal`]/[`cov`] 返回零量（调用方应先检查能力位）。
///
/// 命名 `PointCloudTrait` 以避免与 concrete [`crate::points::point_cloud::PointCloud`]
/// 同名冲突；泛型边界写作 `P: PointCloudTrait`。
pub trait PointCloudTrait {
    /// 点数。
    fn num_points(&self) -> usize;

    /// 是否含点坐标（默认：点数 > 0）。
    fn has_points(&self) -> bool {
        self.num_points() > 0
    }

    /// 是否含法向。
    fn has_normals(&self) -> bool;

    /// 是否含协方差。
    fn has_covs(&self) -> bool;

    /// 取第 `i` 个点，齐次形 `(x, y, z, 1)`。
    fn point(&self, i: usize) -> Vector4<f64>;

    /// 取第 `i` 个法向，齐次形 `(nx, ny, nz, 0)`。
    fn normal(&self, i: usize) -> Vector4<f64>;

    /// 取第 `i` 个协方差（仅左上 3×3 有效）。
    fn cov(&self, i: usize) -> Matrix4<f64>;
}

/// 点云可写能力面（对照 `traits::Traits<T>` 的 `resize/set_*` 方法）。
///
/// 降采样与法向估计会向输出/原地点云写入属性，故要求该 trait。
pub trait PointCloudMut {
    /// 重设点云容量（点/法向/协方差一并缩放）。
    fn resize(&mut self, n: usize);

    /// 写第 `i` 个点，`pt = (x, y, z, 1)`。
    fn set_point(&mut self, i: usize, pt: Vector4<f64>);

    /// 写第 `i` 个法向，`n = (nx, ny, nz, 0)`。
    fn set_normal(&mut self, i: usize, n: Vector4<f64>);

    /// 写第 `i` 个协方差（仅左上 3×3 有效）。
    fn set_cov(&mut self, i: usize, cov: Matrix4<f64>);
}
