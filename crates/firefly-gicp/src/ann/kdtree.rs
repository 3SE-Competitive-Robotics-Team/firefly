//! KdTree 最近邻搜索（对照 official `ann/kdtree.hpp`）。
//!
//! 自顶向下建树：非叶节点按最大方差轴（或法向方向）中位切分，叶节点顺序存储
//! 全局点索引；查询沿「近侧优先、远侧按需」回溯，算法结构与官方逐行对齐。
//! 数值行为（并列距离取舍、空点云处理）以官方测试为准。
//!
//! 并行性通过 rayon 在调用方（法向估计）实现；本模块只提供可复用的单树查询。

use nalgebra::{Matrix4, Vector4};

use crate::ann::NearestNeighborSearch;
use crate::ann::knn_result::{INVALID_INDEX, KnnResult, KnnSetting};
use crate::points::traits::PointCloudTrait;

/// 无效节点哨兵（对照 `INVALID_NODE = std::numeric_limits<uint32_t>::max()`）。
pub const INVALID_NODE: u32 = u32::MAX;

/// 投影轴搜索参数（对照 `ProjectionSetting`）。
#[derive(Clone, Copy, Debug)]
pub struct ProjectionSetting {
    /// 用于估计方差的最大采样点数。
    pub max_scan_count: usize,
}

impl Default for ProjectionSetting {
    fn default() -> Self {
        Self {
            max_scan_count: 128,
        }
    }
}

/// 投影轴抽象（对照 `AxisAlignedProjection` / `NormalProjection`）。
///
/// `project` 把点投到一维；`find_axis` 在给定点区间内挑选方差最大的投影方向。
pub trait Projection: Clone + Send + Sync {
    /// 把点投影到一维标量。
    fn project(&self, pt: &Vector4<f64>) -> f64;

    /// 在 `[first, last)` 索引区间内挑选方差最大的投影轴。
    fn find_axis<P: PointCloudTrait>(
        points: &P,
        indices: &[usize],
        first: usize,
        last: usize,
        setting: &ProjectionSetting,
    ) -> Self;
}

/// 轴对齐投影：选取 XYZ 中方差最大者（对照 `AxisAlignedProjection`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct AxisAlignedProjection {
    /// 轴索引（0:X, 1:Y, 2:Z）。
    pub axis: usize,
}

impl Projection for AxisAlignedProjection {
    fn project(&self, pt: &Vector4<f64>) -> f64 {
        pt[self.axis]
    }

    fn find_axis<P: PointCloudTrait>(
        points: &P,
        indices: &[usize],
        first: usize,
        last: usize,
        setting: &ProjectionSetting,
    ) -> Self {
        let n = last - first;
        let step = if n < setting.max_scan_count {
            1
        } else {
            (n / setting.max_scan_count).max(1)
        };
        let num_steps = n / step;

        let mut sum_pt = Vector4::zeros();
        let mut sum_sq = Vector4::zeros();
        for s in 0..num_steps {
            let pt = points.point(indices[first + step * s]);
            sum_pt += pt;
            sum_sq += pt.component_mul(&pt);
        }

        // mean.w = 采样点数；var 缩放常数不影响 argmax
        let mean = sum_pt / sum_pt.w;
        let var = sum_sq - mean.component_mul(&sum_pt);
        let axis = if var[0] > var[1] {
            if var[0] > var[2] { 0 } else { 2 }
        } else if var[1] > var[2] {
            1
        } else {
            2
        };
        AxisAlignedProjection { axis }
    }
}

/// 法向投影：选取 3D 方差最大方向（协方差最大特征向量，对照 `NormalProjection`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct NormalProjection {
    /// 投影方向（单位向量）。
    pub normal: [f64; 3],
}

impl Projection for NormalProjection {
    fn project(&self, pt: &Vector4<f64>) -> f64 {
        self.normal[0] * pt[0] + self.normal[1] * pt[1] + self.normal[2] * pt[2]
    }

    fn find_axis<P: PointCloudTrait>(
        points: &P,
        indices: &[usize],
        first: usize,
        last: usize,
        setting: &ProjectionSetting,
    ) -> Self {
        use nalgebra::SymmetricEigen;

        let n = last - first;
        let step = if n < setting.max_scan_count {
            1
        } else {
            (n / setting.max_scan_count).max(1)
        };
        let num_steps = n / step;

        let mut sum_pt = Vector4::zeros();
        let mut sum_sq = Matrix4::zeros();
        for s in 0..num_steps {
            let pt = points.point(indices[first + step * s]);
            sum_pt += pt;
            sum_sq += pt * pt.transpose();
        }

        let mean = sum_pt / sum_pt.w;
        let cov = (sum_sq - mean * sum_pt.transpose()) / sum_pt.w;
        let eig = SymmetricEigen::new(cov.fixed_view::<3, 3>(0, 0).into_owned());

        // 特征向量按特征值升序排列；最大特征值方向取第 2 列（对照 eigenvectors().col(2)）
        let mut order = [0usize, 1, 2];
        order.sort_by(|&a, &b| eig.eigenvalues[a].partial_cmp(&eig.eigenvalues[b]).unwrap());
        let v = eig.eigenvectors.column(order[2]);

        NormalProjection {
            normal: [v[0], v[1], v[2]],
        }
    }
}

#[derive(Clone)]
enum NodeInner<P: Projection> {
    Leaf { first: u32, last: u32 },
    NonLeaf { proj: P, thresh: f64 },
}

/// KdTree 节点（对照 `KdTreeNode`）。
///
/// 叶节点存全局索引区间 `[first, last)`；非叶节点存投影轴与切分阈值。
/// 以 `left == INVALID_NODE` 标识叶节点（同官方 union 约定）。
#[derive(Clone)]
pub struct KdTreeNode<P: Projection> {
    inner: NodeInner<P>,
    left: u32,
    right: u32,
}

impl<P: Projection> KdTreeNode<P> {
    fn dummy() -> Self {
        Self {
            inner: NodeInner::Leaf { first: 0, last: 0 },
            left: INVALID_NODE,
            right: INVALID_NODE,
        }
    }

    fn leaf(first: u32, last: u32) -> Self {
        Self {
            inner: NodeInner::Leaf { first, last },
            left: INVALID_NODE,
            right: INVALID_NODE,
        }
    }

    fn non_leaf(proj: P, thresh: f64) -> Self {
        Self {
            inner: NodeInner::NonLeaf { proj, thresh },
            left: INVALID_NODE,
            right: INVALID_NODE,
        }
    }
}

/// KdTree 构建器（对照 `KdTreeBuilder`）。
#[derive(Clone, Copy, Debug)]
pub struct KdTreeBuilder {
    /// 叶节点最大点数。
    pub max_leaf_size: usize,
    /// 投影轴搜索参数。
    pub projection_setting: ProjectionSetting,
}

impl Default for KdTreeBuilder {
    fn default() -> Self {
        Self {
            max_leaf_size: 20,
            projection_setting: ProjectionSetting::default(),
        }
    }
}

impl KdTreeBuilder {
    /// 构建 KdTree，返回 `(root, nodes, indices)`。
    pub fn build<P, Proj>(&self, points: &P) -> (u32, Vec<KdTreeNode<Proj>>, Vec<usize>)
    where
        P: PointCloudTrait,
        Proj: Projection,
    {
        let n = points.num_points();
        if n == 0 {
            return (INVALID_NODE, Vec::new(), Vec::new());
        }

        let mut indices: Vec<usize> = (0..n).collect();
        // 满二叉树节点数上界 2n-1，预分配 2n 避免越界
        let mut nodes: Vec<KdTreeNode<Proj>> = vec![KdTreeNode::dummy(); 2 * n];
        let mut node_count = 0usize;
        let root = self.create_node(points, &mut indices, 0, n, &mut nodes, &mut node_count);
        nodes.truncate(node_count);
        (root, nodes, indices)
    }

    fn create_node<P, Proj>(
        &self,
        points: &P,
        indices: &mut [usize],
        first: usize,
        last: usize,
        nodes: &mut Vec<KdTreeNode<Proj>>,
        node_count: &mut usize,
    ) -> u32
    where
        P: PointCloudTrait,
        Proj: Projection,
    {
        let n = last - first;
        let node_index = *node_count;
        *node_count += 1;

        if n <= self.max_leaf_size {
            nodes[node_index] = KdTreeNode::leaf(first as u32, last as u32);
            return node_index as u32;
        }

        let proj = Proj::find_axis(points, indices, first, last, &self.projection_setting);
        let mid = first + n / 2;
        indices[first..last].select_nth_unstable_by(mid - first, |&a, &b| {
            let da = proj.project(&points.point(a));
            let db = proj.project(&points.point(b));
            da.partial_cmp(&db).expect("点坐标有限，投影值非 NaN")
        });
        let thresh = proj.project(&points.point(indices[mid]));

        nodes[node_index] = KdTreeNode::non_leaf(proj, thresh);
        let left = self.create_node(points, indices, first, mid, nodes, node_count);
        nodes[node_index].left = left;
        let right = self.create_node(points, indices, mid, last, nodes, node_count);
        nodes[node_index].right = right;
        node_index as u32
    }
}

/// 不持有点云所有权的 KdTree（对照 `UnsafeKdTree`）。
///
/// 点云须在其生命周期内保持有效；法向估计等场景下在局部作用域内构建。
pub struct UnsafeKdTree<'a, P: PointCloudTrait, Proj: Projection = AxisAlignedProjection> {
    points: &'a P,
    indices: Vec<usize>,
    root: u32,
    nodes: Vec<KdTreeNode<Proj>>,
}

impl<'a, P: PointCloudTrait, Proj: Projection + Default> UnsafeKdTree<'a, P, Proj> {
    /// 以默认构建器建树。
    pub fn new(points: &'a P) -> Self {
        Self::with_builder(points, &KdTreeBuilder::default())
    }
}

impl<'a, P: PointCloudTrait, Proj: Projection> UnsafeKdTree<'a, P, Proj> {
    /// 以指定构建器建树。
    pub fn with_builder(points: &'a P, builder: &KdTreeBuilder) -> Self {
        let (root, nodes, indices) = builder.build(points);
        Self {
            points,
            indices,
            root,
            nodes,
        }
    }

    /// k 近邻搜索；`k_indices`/`k_sq_dists` 长度须 ≥ `k`，结果按距离升序。
    pub fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        if self.root == INVALID_NODE {
            return 0;
        }
        let mut result = KnnResult::new(k_indices, k_sq_dists, k);
        let setting = KnnSetting::default();
        knn_search_recursive(
            self.points,
            &self.nodes,
            &self.indices,
            self.root,
            query,
            &mut result,
            &setting,
        );
        result.num_found()
    }

    /// 最近邻搜索；命中返回 1 并写入结果，否则返回 0。
    pub fn nearest_neighbor_search(
        &self,
        query: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        let mut idx = [INVALID_INDEX];
        let mut dist = [0.0f64];
        let n = self.knn_search(query, 1, &mut idx, &mut dist);
        *k_index = idx[0];
        *k_sq_dist = dist[0];
        n
    }

    /// 批量并行 k 近邻（`rayon`，对照 `kdtree_omp.hpp` 的并行查询）。
    ///
    /// 对 `queries` 中每个点并行执行 `knn_search`，返回与输入等长的 `(indices, sq_dists)` 列表。
    /// 每个内层 `Vec` 长度为实际找到的近邻数（≤ `k`）。
    pub fn par_knn_search_batch(
        &self,
        queries: &[Vector4<f64>],
        k: usize,
    ) -> Vec<(Vec<usize>, Vec<f64>)>
    where
        P: Sync,
        Proj: Sync,
    {
        use rayon::prelude::*;
        queries
            .par_iter()
            .map(|q| {
                let mut idx = vec![INVALID_INDEX; k];
                let mut dist = vec![f64::MAX; k];
                let n = self.knn_search(q, k, &mut idx, &mut dist);
                idx.truncate(n);
                dist.truncate(n);
                (idx, dist)
            })
            .collect()
    }

    /// 批量并行最近邻（`rayon`）。
    pub fn par_nearest_search_batch(&self, queries: &[Vector4<f64>]) -> Vec<Option<(usize, f64)>>
    where
        P: Sync,
        Proj: Sync,
    {
        use rayon::prelude::*;
        queries
            .par_iter()
            .map(|q| {
                let mut idx = 0usize;
                let mut dist = 0.0f64;
                let n = self.nearest_neighbor_search(q, &mut idx, &mut dist);
                if n == 1 { Some((idx, dist)) } else { None }
            })
            .collect()
    }
}

/// 持有点云所有权的 KdTree（对照 `KdTree`）。
///
/// 点云被内部持有，可跨作用域复用；搜索接口与 `UnsafeKdTree` 一致。
pub struct KdTree<P: PointCloudTrait, Proj: Projection = AxisAlignedProjection> {
    points: P,
    indices: Vec<usize>,
    root: u32,
    nodes: Vec<KdTreeNode<Proj>>,
}

impl<P: PointCloudTrait, Proj: Projection + Default> KdTree<P, Proj> {
    /// 以默认构建器建树（消费点云）。
    pub fn new(points: P) -> Self {
        Self::with_builder(points, &KdTreeBuilder::default())
    }
}

impl<P: PointCloudTrait, Proj: Projection> KdTree<P, Proj> {
    /// 以指定构建器建树（消费点云）。
    pub fn with_builder(points: P, builder: &KdTreeBuilder) -> Self {
        let (root, nodes, indices) = builder.build(&points);
        Self {
            points,
            indices,
            root,
            nodes,
        }
    }

    /// 取内部点云引用。
    pub fn points(&self) -> &P {
        &self.points
    }

    /// k 近邻搜索。
    pub fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        if self.root == INVALID_NODE {
            return 0;
        }
        let mut result = KnnResult::new(k_indices, k_sq_dists, k);
        let setting = KnnSetting::default();
        knn_search_recursive(
            &self.points,
            &self.nodes,
            &self.indices,
            self.root,
            query,
            &mut result,
            &setting,
        );
        result.num_found()
    }

    /// 最近邻搜索。
    pub fn nearest_neighbor_search(
        &self,
        query: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        let mut idx = [INVALID_INDEX];
        let mut dist = [0.0f64];
        let n = self.knn_search(query, 1, &mut idx, &mut dist);
        *k_index = idx[0];
        *k_sq_dist = dist[0];
        n
    }

    /// 批量并行 k 近邻（`rayon`，对照 `kdtree_omp.hpp` 的并行查询）。
    pub fn par_knn_search_batch(
        &self,
        queries: &[Vector4<f64>],
        k: usize,
    ) -> Vec<(Vec<usize>, Vec<f64>)>
    where
        P: Sync,
        Proj: Sync,
    {
        use rayon::prelude::*;
        queries
            .par_iter()
            .map(|q| {
                let mut idx = vec![INVALID_INDEX; k];
                let mut dist = vec![f64::MAX; k];
                let n = self.knn_search(q, k, &mut idx, &mut dist);
                idx.truncate(n);
                dist.truncate(n);
                (idx, dist)
            })
            .collect()
    }

    /// 批量并行最近邻（`rayon`）。
    pub fn par_nearest_search_batch(&self, queries: &[Vector4<f64>]) -> Vec<Option<(usize, f64)>>
    where
        P: Sync,
        Proj: Sync,
    {
        use rayon::prelude::*;
        queries
            .par_iter()
            .map(|q| {
                let mut idx = 0usize;
                let mut dist = 0.0f64;
                let n = self.nearest_neighbor_search(q, &mut idx, &mut dist);
                if n == 1 { Some((idx, dist)) } else { None }
            })
            .collect()
    }
}

impl<'a, P: PointCloudTrait, Proj: Projection> NearestNeighborSearch for UnsafeKdTree<'a, P, Proj> {
    fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        self.knn_search(query, k, k_indices, k_sq_dists)
    }
}

impl<P: PointCloudTrait, Proj: Projection> NearestNeighborSearch for KdTree<P, Proj> {
    fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        self.knn_search(query, k, k_indices, k_sq_dists)
    }
}

fn knn_search_recursive<P, Proj>(
    points: &P,
    nodes: &[KdTreeNode<Proj>],
    indices: &[usize],
    node_index: u32,
    query: &Vector4<f64>,
    result: &mut KnnResult,
    setting: &KnnSetting,
) -> bool
where
    P: PointCloudTrait,
    Proj: Projection,
{
    if node_index == INVALID_NODE {
        return true;
    }
    let node = &nodes[node_index as usize];
    match &node.inner {
        NodeInner::Leaf { first, last } => {
            for i in *first as usize..*last as usize {
                let pt = points.point(indices[i]);
                let d = (pt - query).norm_squared();
                result.push(indices[i], d);
            }
            !setting.fulfilled(result)
        }
        NodeInner::NonLeaf { proj, thresh } => {
            let val = proj.project(query);
            let diff = val - *thresh;
            let cut_sq = diff * diff;
            let (best, other) = if diff < 0.0 {
                (node.left, node.right)
            } else {
                (node.right, node.left)
            };
            if !knn_search_recursive(points, nodes, indices, best, query, result, setting) {
                return false;
            }
            if result.worst_distance() > cut_sq {
                return knn_search_recursive(points, nodes, indices, other, query, result, setting);
            }
            true
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::point_cloud::PointCloud;
    use crate::points::traits::{PointCloudMut, PointCloudTrait};
    use nalgebra::Vector3;

    /// 暴力 KNN：全量排序取前 k（对照官方 bruteforce；无并列时与 kdtree 唯一一致）。
    fn bruteforce_knn(target: &PointCloud, query: &Vector4<f64>, k: usize) -> Vec<(usize, f64)> {
        let mut v: Vec<(usize, f64)> = (0..target.num_points())
            .map(|j| {
                let d = (target.point(j) - query).norm_squared();
                (j, d)
            })
            .collect();
        v.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        v.truncate(k);
        v
    }

    fn rand_cloud(rng: &mut u64, n: usize, scale: f64) -> PointCloud {
        let mut c = PointCloud::new();
        c.resize(n);
        for i in 0..n {
            let x = (splitmix64(rng) as f64 / u64::MAX as f64 - 0.5) * scale;
            let y = (splitmix64(rng) as f64 / u64::MAX as f64 - 0.5) * scale;
            let z = (splitmix64(rng) as f64 / u64::MAX as f64 - 0.5) * scale;
            c.set_point(i, Vector4::new(x, y, z, 1.0));
        }
        c
    }

    fn grid_cloud() -> PointCloud {
        let coords: Vec<Vector3<f64>> = (0..7i64)
            .flat_map(|x| {
                (0..7i64).flat_map(move |y| {
                    (0..7i64).map(move |z| Vector3::new(x as f64, y as f64, z as f64))
                })
            })
            .collect();
        PointCloud::from_points3(&coords)
    }

    #[test]
    fn empty_tree_no_search() {
        let empty = PointCloud::new();
        let tree = UnsafeKdTree::<PointCloud>::new(&empty);
        let mut idx = [0usize; 5];
        let mut dist = [0.0f64; 5];
        assert_eq!(
            tree.knn_search(&Vector4::new(0.0, 0.0, 0.0, 1.0), 5, &mut idx, &mut dist),
            0
        );
    }

    #[test]
    fn knn_matches_bruteforce_random() {
        let mut rng = 0x1357_9BDF;
        let target = rand_cloud(&mut rng, 300, 100.0);
        let tree = KdTree::<PointCloud>::new(target.clone());

        let queries: Vec<Vector4<f64>> = (0..50)
            .map(|_| {
                let base = target.point((splitmix64(&mut rng) as usize) % target.num_points());
                Vector4::new(
                    base.x + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 2.0,
                    base.y + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 2.0,
                    base.z + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 2.0,
                    1.0,
                )
            })
            .collect();

        for q in &queries {
            for &k in &[1usize, 2, 3, 5, 10, 20] {
                let bf = bruteforce_knn(&target, q, k);
                let mut idx = vec![0usize; k];
                let mut dist = vec![0.0f64; k];
                let n = tree.knn_search(q, k, &mut idx, &mut dist);
                assert_eq!(n, k, "k={k}");
                // 距离升序
                for j in 1..k {
                    assert!(dist[j] >= dist[j - 1], "距离须升序 k={k}");
                }
                // 索引与距离与暴力一致（随机点无并列 → 唯一答案）
                for j in 0..k {
                    assert_eq!(idx[j], bf[j].0, "k={k} 第{j}近邻索引须一致");
                    assert!(
                        (dist[j] - bf[j].1).abs() < 1e-9,
                        "k={k} 第{j}近邻距离须一致"
                    );
                }
                // 最近邻搜索
                let mut n_idx = 0usize;
                let mut n_dist = 0.0f64;
                let found = tree.nearest_neighbor_search(q, &mut n_idx, &mut n_dist);
                assert_eq!(found, 1);
                assert_eq!(n_idx, bf[0].0);
                assert!((n_dist - bf[0].1).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn knn_matches_bruteforce_grid() {
        // 网格点存在大量等距并列，索引不唯一；仅校验距离多重集一致且重算距离吻合
        let target = grid_cloud();
        let tree = KdTree::<PointCloud>::new(target.clone());
        let queries = vec![
            Vector4::new(3.0, 3.0, 3.0, 1.0),
            Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector4::new(6.0, 2.0, 4.0, 1.0),
        ];
        for q in &queries {
            for &k in &[1usize, 5, 20] {
                let bf = bruteforce_knn(&target, q, k);
                let bf_dists: Vec<f64> = bf.iter().map(|(_, d)| *d).collect();
                let mut idx = vec![0usize; k];
                let mut dist = vec![0.0f64; k];
                let n = tree.knn_search(q, k, &mut idx, &mut dist);
                assert_eq!(n, k);
                // 距离须升序且与暴力距离多重集一致
                for j in 1..k {
                    assert!(dist[j] + 1e-12 >= dist[j - 1], "距离须升序");
                }
                let mut bf_sorted = bf_dists.clone();
                let mut kd_sorted = dist.clone();
                bf_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
                kd_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
                for j in 0..k {
                    assert!(
                        (bf_sorted[j] - kd_sorted[j]).abs() < 1e-9,
                        "距离多重集须一致 k={k} j={j} bf={} kd={}",
                        bf_sorted[j],
                        kd_sorted[j]
                    );
                }
                for j in 0..k {
                    let recomputed = (target.point(idx[j]) - q).norm_squared();
                    assert!((recomputed - dist[j]).abs() < 1e-9, "返回距离须可重算");
                }
            }
        }
    }

    #[test]
    fn grid_distinct_points_count() {
        // 343 个互异网格点；k=20 的最近邻必含自查（距离 0）
        let target = grid_cloud();
        let tree = KdTree::<PointCloud>::new(target.clone());
        let q = Vector4::new(3.0, 3.0, 3.0, 1.0);
        let mut idx = [0usize; 20];
        let mut dist = [0.0f64; 20];
        let n = tree.knn_search(&q, 20, &mut idx, &mut dist);
        assert_eq!(n, 20);
        let self_pos = target
            .points
            .iter()
            .position(|p| {
                (p.x - 3.0).abs() < 1e-9 && (p.y - 3.0).abs() < 1e-9 && (p.z - 3.0).abs() < 1e-9
            })
            .unwrap();
        assert_eq!(idx[0], self_pos);
        assert!(dist[0] < 1e-12);
    }

    #[test]
    fn normal_projection_builds() {
        let mut rng = 0x2468_ACEF;
        let target = rand_cloud(&mut rng, 200, 50.0);
        let tree = KdTree::<PointCloud, NormalProjection>::new(target.clone());
        let q = target.point(0);
        let mut idx = [0usize; 10];
        let mut dist = [0.0f64; 10];
        let n = tree.knn_search(&q, 10, &mut idx, &mut dist);
        assert_eq!(n, 10);
        for j in 1..10 {
            assert!(dist[j] >= dist[j - 1]);
        }
    }

    #[test]
    fn par_batch_matches_serial() {
        let mut rng = 0xDEAD_BEEF;
        let target = rand_cloud(&mut rng, 500, 80.0);
        let tree = KdTree::<PointCloud>::new(target.clone());
        let queries: Vec<Vector4<f64>> = (0..100)
            .map(|_| {
                let base = target.point((splitmix64(&mut rng) as usize) % target.num_points());
                Vector4::new(
                    base.x + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 5.0,
                    base.y + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 5.0,
                    base.z + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 5.0,
                    1.0,
                )
            })
            .collect();

        // 串行逐个查询
        let mut serial = Vec::new();
        for q in &queries {
            let mut idx = vec![0usize; 5];
            let mut dist = vec![0.0; 5];
            let n = tree.knn_search(q, 5, &mut idx, &mut dist);
            idx.truncate(n);
            dist.truncate(n);
            serial.push((idx, dist));
        }

        // 并行批量查询
        let parallel = tree.par_knn_search_batch(&queries, 5);
        assert_eq!(serial.len(), parallel.len());
        for (s, p) in serial.iter().zip(parallel.iter()) {
            assert_eq!(s.0, p.0);
            for (a, b) in s.1.iter().zip(p.1.iter()) {
                assert!((a - b).abs() < 1e-12);
            }
        }

        // 最近邻批量
        let mut serial_nearest = Vec::new();
        for q in &queries {
            let mut idx = 0usize;
            let mut dist = 0.0;
            let n = tree.nearest_neighbor_search(q, &mut idx, &mut dist);
            serial_nearest.push(if n == 1 { Some((idx, dist)) } else { None });
        }
        let parallel_nearest = tree.par_nearest_search_batch(&queries);
        assert_eq!(serial_nearest, parallel_nearest);
    }

    fn splitmix64(state: &mut u64) -> u64 {
        *state = state.wrapping_add(0x9E37_9B97_F4A7_C15B);
        let mut z = *state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}
