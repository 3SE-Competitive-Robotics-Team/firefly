//! concrete 点云（对照 official `points/point_cloud.hpp`）。
//!
//! 同时持有点坐标、法向、协方差三套缓冲；`resize` 同步三者容量。

use nalgebra::{Matrix3, Matrix4, Vector3, Vector4};

use super::traits::{PointCloudMut, PointCloudTrait};

/// 点云：点坐标 + 法向 + 协方差。
///
/// - `points`：点坐标 `(x, y, z, 1)`；
/// - `normals`：法向 `(nx, ny, nz, 0)`；
/// - `covs`：协方差（左上 3×3 有效，其余补零）。
#[derive(Clone, Debug, Default)]
pub struct PointCloud {
    /// 点坐标。
    pub points: Vec<Vector4<f64>>,
    /// 法向。
    pub normals: Vec<Vector4<f64>>,
    /// 协方差。
    pub covs: Vec<Matrix4<f64>>,
}

impl PointCloud {
    /// 构造空点云。
    pub fn new() -> Self {
        Self::default()
    }

    /// 由三维点构造（齐次 `w = 1`）。
    pub fn from_points3(points: &[Vector3<f64>]) -> Self {
        let mut cloud = PointCloud::new();
        cloud.resize(points.len());
        for (i, p) in points.iter().enumerate() {
            cloud.set_point(i, Vector4::new(p.x, p.y, p.z, 1.0));
        }
        cloud
    }

    /// 由四维齐次点构造（保留 `w`）。
    pub fn from_points4(points: &[Vector4<f64>]) -> Self {
        let mut cloud = PointCloud::new();
        cloud.resize(points.len());
        for (i, p) in points.iter().enumerate() {
            cloud.set_point(i, *p);
        }
        cloud
    }

    /// 追加一个点（齐次 `w = 1`）。
    pub fn push_point(&mut self, p: &Vector3<f64>) {
        let i = self.points.len();
        self.resize(i + 1);
        self.set_point(i, Vector4::new(p.x, p.y, p.z, 1.0));
    }

    /// 点数。
    pub fn size(&self) -> usize {
        self.points.len()
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// 取第 `i` 个点的可变引用（不变量：坐标 `w = 1`）。
    pub fn point_mut(&mut self, i: usize) -> &mut Vector4<f64> {
        &mut self.points[i]
    }

    /// 由 3×3 协方差构造四维齐次协方差（左上 3×3 填充，右下 `w` 置零）。
    pub fn cov_from_3x3(cov3: &Matrix3<f64>) -> Matrix4<f64> {
        let mut cov = Matrix4::zeros();
        cov.fixed_view_mut::<3, 3>(0, 0).copy_from(cov3);
        cov
    }
}

impl PointCloudTrait for PointCloud {
    fn num_points(&self) -> usize {
        self.points.len()
    }

    fn has_points(&self) -> bool {
        !self.points.is_empty()
    }

    fn has_normals(&self) -> bool {
        !self.normals.is_empty()
    }

    fn has_covs(&self) -> bool {
        !self.covs.is_empty()
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        self.points[i]
    }

    fn normal(&self, i: usize) -> Vector4<f64> {
        self.normals[i]
    }

    fn cov(&self, i: usize) -> Matrix4<f64> {
        self.covs[i]
    }
}

impl PointCloudMut for PointCloud {
    fn resize(&mut self, n: usize) {
        self.points.resize(n, Vector4::zeros());
        self.normals.resize(n, Vector4::zeros());
        self.covs.resize(n, Matrix4::zeros());
    }

    fn set_point(&mut self, i: usize, pt: Vector4<f64>) {
        self.points[i] = pt;
    }

    fn set_normal(&mut self, i: usize, n: Vector4<f64>) {
        self.normals[i] = n;
    }

    fn set_cov(&mut self, i: usize, cov: Matrix4<f64>) {
        self.covs[i] = cov;
    }
}

/// 对 `Vec<Vector3<f64>>` 实现只读点云（对照 `eigen.hpp` 的 `std::vector<Eigen::Matrix<Scalar, 3, 1>>`）。
impl PointCloudTrait for Vec<Vector3<f64>> {
    fn num_points(&self) -> usize {
        self.len()
    }

    fn has_normals(&self) -> bool {
        false
    }

    fn has_covs(&self) -> bool {
        false
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        let p = self[i];
        Vector4::new(p.x, p.y, p.z, 1.0)
    }

    fn normal(&self, _i: usize) -> Vector4<f64> {
        Vector4::zeros()
    }

    fn cov(&self, _i: usize) -> Matrix4<f64> {
        Matrix4::zeros()
    }
}

/// 对 `Vec<Vector4<f64>>` 实现只读点云（对照 `eigen.hpp` 的 `std::vector<Eigen::Matrix<Scalar, 4, 1>>`）。
impl PointCloudTrait for Vec<Vector4<f64>> {
    fn num_points(&self) -> usize {
        self.len()
    }

    fn has_normals(&self) -> bool {
        false
    }

    fn has_covs(&self) -> bool {
        false
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        self[i]
    }

    fn normal(&self, _i: usize) -> Vector4<f64> {
        Vector4::zeros()
    }

    fn cov(&self, _i: usize) -> Matrix4<f64> {
        Matrix4::zeros()
    }
}

/// 对 `Vec<Vector3<f32>>` 实现只读点云（窄精度输入）。
impl PointCloudTrait for Vec<Vector3<f32>> {
    fn num_points(&self) -> usize {
        self.len()
    }

    fn has_normals(&self) -> bool {
        false
    }

    fn has_covs(&self) -> bool {
        false
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        let p = self[i];
        Vector4::new(f64::from(p.x), f64::from(p.y), f64::from(p.z), 1.0)
    }

    fn normal(&self, _i: usize) -> Vector4<f64> {
        Vector4::zeros()
    }

    fn cov(&self, _i: usize) -> Matrix4<f64> {
        Matrix4::zeros()
    }
}

/// 对 `Vec<Vector4<f32>>` 实现只读点云（窄精度输入）。
impl PointCloudTrait for Vec<Vector4<f32>> {
    fn num_points(&self) -> usize {
        self.len()
    }

    fn has_normals(&self) -> bool {
        false
    }

    fn has_covs(&self) -> bool {
        false
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        let p = self[i];
        Vector4::new(
            f64::from(p.x),
            f64::from(p.y),
            f64::from(p.z),
            f64::from(p.w),
        )
    }

    fn normal(&self, _i: usize) -> Vector4<f64> {
        Vector4::zeros()
    }

    fn cov(&self, _i: usize) -> Matrix4<f64> {
        Matrix4::zeros()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::traits::{PointCloudMut, PointCloudTrait};

    #[test]
    fn point_cloud_roundtrip() {
        let mut cloud = PointCloud::new();
        cloud.resize(4);
        for i in 0..4 {
            cloud.set_point(
                i,
                Vector4::new(i as f64, (i * 2) as f64, (i * 3) as f64, 1.0),
            );
            cloud.set_normal(i, Vector4::new(0.0, 0.0, 1.0, 0.0));
            let mut c = Matrix4::zeros();
            c[(0, 0)] = 1.0;
            cloud.set_cov(i, c);
        }
        assert_eq!(cloud.num_points(), 4);
        assert!(cloud.has_points());
        assert!(cloud.has_normals());
        assert!(cloud.has_covs());
        for i in 0..4 {
            assert_eq!(
                cloud.point(i),
                Vector4::new(i as f64, (i * 2) as f64, (i * 3) as f64, 1.0)
            );
            assert_eq!(cloud.normal(i), Vector4::new(0.0, 0.0, 1.0, 0.0));
            assert_eq!(cloud.cov(i)[(0, 0)], 1.0);
        }
        cloud.resize(2);
        assert_eq!(cloud.num_points(), 2);
        assert_eq!(cloud.point(0), Vector4::new(0.0, 0.0, 0.0, 1.0));
        assert_eq!(cloud.normal(1), Vector4::new(0.0, 0.0, 1.0, 0.0));
    }

    #[test]
    fn vec_trait_readonly() {
        let pts: Vec<Vector3<f64>> = vec![Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0)];
        assert_eq!(pts.num_points(), 2);
        assert!(!pts.has_normals());
        assert_eq!(pts.point(0), Vector4::new(1.0, 2.0, 3.0, 1.0));
        assert_eq!(pts.point(1), Vector4::new(4.0, 5.0, 6.0, 1.0));
    }
}
