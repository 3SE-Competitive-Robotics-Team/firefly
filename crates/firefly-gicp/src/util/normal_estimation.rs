//! 法向 / 协方差估计（对照 official `util/normal_estimation.hpp`）。
//!
//! 对每个点做 k 近邻，由邻域协方差的最小特征向量定法向，最大方差方向定协方差。
//! 邻点数不足（< 5）时写入无效值。并行性用 rayon 实现：先并行计算各点特征向量，
//! 再顺序回填，避免树与点云的借用冲突。

use nalgebra::{Matrix3, Matrix4, SymmetricEigen, Vector3, Vector4};
use rayon::prelude::*;

use crate::ann::knn_result::INVALID_INDEX;
use crate::points::traits::{PointCloudMut, PointCloudTrait};

/// 局部特征设定器（对照 `NormalSetter` / `CovarianceSetter` / `NormalCovarianceSetter`）。
///
/// 把协方差特征向量转成法向/协方差，或对邻点不足的点写无效值。
pub trait LocalFeatureSetter: Sync {
    /// 由升序特征向量矩阵（列 = 特征向量）计算法向，并按点定向翻转到局部坐标原点侧。
    fn compute_normal(&self, query_point: &Vector4<f64>, eigvecs: &Matrix3<f64>) -> Vector4<f64>;

    /// 由升序特征向量矩阵计算协方差（各向异性特征值）。
    fn compute_cov(&self, eigvecs: &Matrix3<f64>) -> Matrix4<f64>;

    /// 把计算出的 `(normal, cov)` 写回点云。
    fn set<P: PointCloudTrait + PointCloudMut>(
        &self,
        cloud: &mut P,
        i: usize,
        normal: &Vector4<f64>,
        cov: &Matrix4<f64>,
    );

    /// 邻点不足时写无效值（法向零，协方差单位阵）。
    fn set_invalid<P: PointCloudTrait + PointCloudMut>(&self, cloud: &mut P, i: usize);
}

/// 法向设定器：最小特征向量定向为法向（对照 `NormalSetter`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct NormalSetter;

impl LocalFeatureSetter for NormalSetter {
    fn compute_normal(&self, query_point: &Vector4<f64>, eigvecs: &Matrix3<f64>) -> Vector4<f64> {
        let mut normal = Vector4::zeros();
        normal.fixed_rows_mut::<3>(0).copy_from(&eigvecs.column(0));
        if query_point.dot(&normal) > 0.0 {
            normal = -normal;
        }
        normal
    }

    fn compute_cov(&self, _eigvecs: &Matrix3<f64>) -> Matrix4<f64> {
        Matrix4::zeros()
    }

    fn set<P: PointCloudTrait + PointCloudMut>(
        &self,
        cloud: &mut P,
        i: usize,
        normal: &Vector4<f64>,
        _cov: &Matrix4<f64>,
    ) {
        cloud.set_normal(i, *normal);
    }

    fn set_invalid<P: PointCloudTrait + PointCloudMut>(&self, cloud: &mut P, i: usize) {
        cloud.set_normal(i, Vector4::zeros());
    }
}

/// 协方差设定器：各向异性特征值（最小方向 1e-3，其余 1.0）（对照 `CovarianceSetter`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct CovarianceSetter;

impl LocalFeatureSetter for CovarianceSetter {
    fn compute_normal(&self, _query_point: &Vector4<f64>, _eigvecs: &Matrix3<f64>) -> Vector4<f64> {
        Vector4::zeros()
    }

    fn compute_cov(&self, eigvecs: &Matrix3<f64>) -> Matrix4<f64> {
        let values = Vector3::new(1e-3, 1.0, 1.0);
        let cov3 = eigvecs * Matrix3::from_diagonal(&values) * eigvecs.transpose();
        let mut cov = Matrix4::zeros();
        cov.fixed_view_mut::<3, 3>(0, 0).copy_from(&cov3);
        cov
    }

    fn set<P: PointCloudTrait + PointCloudMut>(
        &self,
        cloud: &mut P,
        i: usize,
        _normal: &Vector4<f64>,
        cov: &Matrix4<f64>,
    ) {
        cloud.set_cov(i, *cov);
    }

    fn set_invalid<P: PointCloudTrait + PointCloudMut>(&self, cloud: &mut P, i: usize) {
        let mut cov = Matrix4::identity();
        cov[(3, 3)] = 0.0;
        cloud.set_cov(i, cov);
    }
}

/// 法向与协方差一并设定（对照 `NormalCovarianceSetter`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct NormalCovarianceSetter;

impl LocalFeatureSetter for NormalCovarianceSetter {
    fn compute_normal(&self, query_point: &Vector4<f64>, eigvecs: &Matrix3<f64>) -> Vector4<f64> {
        NormalSetter.compute_normal(query_point, eigvecs)
    }

    fn compute_cov(&self, eigvecs: &Matrix3<f64>) -> Matrix4<f64> {
        CovarianceSetter.compute_cov(eigvecs)
    }

    fn set<P: PointCloudTrait + PointCloudMut>(
        &self,
        cloud: &mut P,
        i: usize,
        normal: &Vector4<f64>,
        cov: &Matrix4<f64>,
    ) {
        cloud.set_normal(i, *normal);
        cloud.set_cov(i, *cov);
    }

    fn set_invalid<P: PointCloudTrait + PointCloudMut>(&self, cloud: &mut P, i: usize) {
        NormalSetter.set_invalid(cloud, i);
        CovarianceSetter.set_invalid(cloud, i);
    }
}

/// 并行逐点计算局部特征，返回每点的 `(normal, cov)`；邻点不足则 `None`。
///
/// - `tree`：最近邻搜索结构（读共享，不持有点云）；
/// - 除数 `n` 为邻域点数（含查询点自身）。
#[fastrace::trace]
fn compute_local_features<P, N, S>(
    cloud: &P,
    tree: &N,
    num_neighbors: usize,
    setter: S,
) -> Vec<Option<(Vector4<f64>, Matrix4<f64>)>>
where
    P: PointCloudTrait + Sync,
    N: crate::ann::NearestNeighborSearch + Sync,
    S: LocalFeatureSetter + Send,
{
    let size = cloud.num_points();
    (0..size)
        .into_par_iter()
        .map(|i| {
            let query = cloud.point(i);
            let mut k_indices = vec![INVALID_INDEX; num_neighbors];
            let mut k_sq_dists = vec![f64::MAX; num_neighbors];
            let n = tree.knn_search(&query, num_neighbors, &mut k_indices, &mut k_sq_dists);

            if n < 5 {
                return None;
            }

            let nf = n as f64;
            let mut sum_points = Vector4::zeros();
            let mut sum_cross = Matrix4::zeros();
            for &idx in &k_indices[..n] {
                let pt = cloud.point(idx);
                sum_points += pt;
                sum_cross += pt * pt.transpose();
            }

            let mean = sum_points / nf;
            // 对照官方 normal_estimation.hpp：`cov = (sum_cross - mean * sum_points^T) / n`，
            // 用 sum_points（= n·mean）保留交叉项数值精度
            let cov = (sum_cross - mean * sum_points.transpose()) / nf;
            let eigvecs = sorted_eigen(&cov.fixed_view::<3, 3>(0, 0).into_owned());

            let normal = setter.compute_normal(&query, &eigvecs);
            let cov_mat = setter.compute_cov(&eigvecs);
            Some((normal, cov_mat))
        })
        .collect()
}

/// 顺序回填计算出的特征（释放树对点云的借用后执行）。
fn apply_local_features<P, S>(
    cloud: &mut P,
    results: &[Option<(Vector4<f64>, Matrix4<f64>)>],
    setter: S,
) where
    P: PointCloudTrait + PointCloudMut,
    S: LocalFeatureSetter,
{
    for i in 0..results.len() {
        match &results[i] {
            Some((normal, cov)) => setter.set(cloud, i, normal, cov),
            None => setter.set_invalid(cloud, i),
        }
    }
}

/// 对称矩阵特征分解，返回特征向量按特征值升序排列的矩阵。
///
/// 官方 Eigen `SelfAdjointEigenSolver` 返回升序；nalgebra `SymmetricEigen` 返回无序，
/// 故手动按特征值排序以对齐列序（第 0 列为最小特征向量）。
fn sorted_eigen(cov3: &Matrix3<f64>) -> Matrix3<f64> {
    let eig = SymmetricEigen::new(*cov3);
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| eig.eigenvalues[a].partial_cmp(&eig.eigenvalues[b]).unwrap());
    let cols = [
        eig.eigenvectors.column(order[0]).into_owned(),
        eig.eigenvectors.column(order[1]).into_owned(),
        eig.eigenvectors.column(order[2]).into_owned(),
    ];
    Matrix3::from_columns(&cols)
}

/// 估计点法向（自建 KdTree）。
pub fn estimate_normals<P: PointCloudTrait + PointCloudMut + Sync>(
    cloud: &mut P,
    num_neighbors: usize,
) {
    let results = {
        let tree =
            crate::ann::kdtree::UnsafeKdTree::<P, crate::ann::AxisAlignedProjection>::new(cloud);
        compute_local_features(cloud, &tree, num_neighbors, NormalSetter)
    };
    apply_local_features(cloud, &results, NormalSetter);
}

/// 估计点协方差（自建 KdTree）。
pub fn estimate_covariances<P: PointCloudTrait + PointCloudMut + Sync>(
    cloud: &mut P,
    num_neighbors: usize,
) {
    let results = {
        let tree =
            crate::ann::kdtree::UnsafeKdTree::<P, crate::ann::AxisAlignedProjection>::new(cloud);
        compute_local_features(cloud, &tree, num_neighbors, CovarianceSetter)
    };
    apply_local_features(cloud, &results, CovarianceSetter);
}

/// 一并估计点法向与协方差（自建 KdTree）。
pub fn estimate_normals_covariances<P: PointCloudTrait + PointCloudMut + Sync>(
    cloud: &mut P,
    num_neighbors: usize,
) {
    let results = {
        let tree =
            crate::ann::kdtree::UnsafeKdTree::<P, crate::ann::AxisAlignedProjection>::new(cloud);
        compute_local_features(cloud, &tree, num_neighbors, NormalCovarianceSetter)
    };
    apply_local_features(cloud, &results, NormalCovarianceSetter);
}

/// 估计点法向（复用已建 KdTree）。
pub fn estimate_normals_with_tree<P, N>(cloud: &mut P, tree: &N, num_neighbors: usize)
where
    P: PointCloudTrait + PointCloudMut + Sync,
    N: crate::ann::NearestNeighborSearch + Sync,
{
    let results = compute_local_features(cloud, tree, num_neighbors, NormalSetter);
    apply_local_features(cloud, &results, NormalSetter);
}

/// 估计点协方差（复用已建 KdTree）。
pub fn estimate_covariances_with_tree<P, N>(cloud: &mut P, tree: &N, num_neighbors: usize)
where
    P: PointCloudTrait + PointCloudMut + Sync,
    N: crate::ann::NearestNeighborSearch + Sync,
{
    let results = compute_local_features(cloud, tree, num_neighbors, CovarianceSetter);
    apply_local_features(cloud, &results, CovarianceSetter);
}

/// 一并估计点法向与协方差（复用已建 KdTree）。
pub fn estimate_normals_covariances_with_tree<P, N>(cloud: &mut P, tree: &N, num_neighbors: usize)
where
    P: PointCloudTrait + PointCloudMut + Sync,
    N: crate::ann::NearestNeighborSearch + Sync,
{
    let results = compute_local_features(cloud, tree, num_neighbors, NormalCovarianceSetter);
    apply_local_features(cloud, &results, NormalCovarianceSetter);
}
