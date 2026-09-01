//! 根体素内八叉树（对照 `voxel_map.h:129` `VoxelOctoTree`）。
//!
//! 每个节点持有：
//! - 临时点集 `temp_points`（未初始化/未成熟时累积）；
//! - 局部平面 [`crate::plane::VoxelPlane`]（`is_plane` 后固定）；
//! - 8 个子节点（`is_plane` 为假且未达最大层时细分）。
//!
//! 状态机（对照 `voxel_map.cpp:219` `UpdateOctoTree`）：
//! 未初始化（点数不足）→ 累积；达到阈值 → `fit_plane`；
//! 判平面 → 固定；不判平面 → 细分（递归），直至最大层丢弃。

use nalgebra::{Matrix3, Vector3};

use crate::options::{PlaneOptions, VoxelMapOptions};
use crate::plane::{VoxelPlane, fit_plane};

/// P10.11：平面法向对齐到「指向相机」——消除 SVD 特征向量符号歧义。
///
/// SVD 拟合的法向 ±n 等价，同一物理面被多个根体素覆盖时相邻体素可能
/// 拟合出相反法向；残差/H 阵带符号，同方向位姿误差在相反法向平面给出
/// 相反残差，聚合修正互相抵消甚至反向（实测同一 y 平面 3 个 +y、2 个
/// -y，深度修正方向错乱）。对齐到观测方向（`n·(center−cam) > 0`）使
/// 同一物理面跨体素法向一致。
pub fn align_normal_to_camera(plane: &mut VoxelPlane, camera_pos: &Vector3<f64>) {
    let to_cam = camera_pos - plane.center;
    if to_cam.norm() > 1e-12 && plane.normal.dot(&to_cam) < 0.0 {
        plane.normal = -plane.normal;
        plane.d = -plane.d;
        // plane_var 的块序 [n, q]：法向块行/列翻转（n 符号翻转的雅可比传播）
        // 简化处理：法向翻转后 Σ_nq 的 [0:3,0:3] 块不变（对称），
        // [0:3,3:6] 与 [3:6,0:3] 交叉块翻转（n 与 q 的耦合符号）
        for i in 0..3 {
            for j in 3..6 {
                let v = plane.plane_var[(i, j)];
                plane.plane_var[(i, j)] = -v;
                plane.plane_var[(j, i)] = -v;
            }
        }
    }
}

/// 八叉树节点。
#[derive(Debug, Clone)]
pub struct OctoNode {
    /// 节点层（根为 0）。
    pub layer: usize,
    /// 最大细分层（全局上限，构造时固化）。
    max_layer: usize,
    /// 节点中心（世界系）。
    pub center: Vector3<f64>,
    /// 半边长（根节点 = `root_size/2`，每层减半）。
    half_len: f64,
    /// 临时点集（世界系点 + 协方差）。
    temp_points: Vec<Vector3<f64>>,
    temp_covs: Vec<Matrix3<f64>>,
    /// 局部平面（`is_plane == true` 后有效）。
    pub plane: Option<VoxelPlane>,
    /// 子节点（`[leaf_index]`，索引 = 4·dx + 2·dy + dz）。
    leaves: [Option<Box<OctoNode>>; 8],
    /// 已初始化（达到拟合阈值后置真）。
    init: bool,
    /// 可更新（成熟后关闭，新点丢弃）。
    update_enable: bool,
    /// 自上次拟合后的新增点数。
    new_points: usize,
}

impl OctoNode {
    /// 构造节点（根节点由 [`OctoNode::new_root`] 创建）。
    #[must_use]
    pub fn new(
        layer: usize,
        max_layer: usize,
        center: Vector3<f64>,
        half_len: f64,
        opts: &VoxelMapOptions,
    ) -> Self {
        Self {
            layer,
            max_layer,
            center,
            half_len,
            temp_points: Vec::with_capacity(opts.layer_init_num[layer.min(4)]),
            temp_covs: Vec::with_capacity(opts.layer_init_num[layer.min(4)]),
            plane: None,
            leaves: std::array::from_fn(|_| None),
            init: false,
            update_enable: true,
            new_points: 0,
        }
    }

    /// 创建根节点（中心 = 根体素中心）。
    #[must_use]
    pub fn new_root(center: Vector3<f64>, opts: &VoxelMapOptions) -> Self {
        Self::new(0, opts.max_layer, center, opts.root_size / 2.0, opts)
    }

    /// 递归查找点所在的叶子节点（对照 `find_correspond`，`voxel_map.cpp:292`）。
    ///
    /// 返回规则：未初始化 / 已判平面 / 已达最大层的节点即返回自身。
    #[must_use]
    pub fn find_correspond(&self, p: Vector3<f64>) -> &Self {
        if !self.init || self.plane.is_some() || self.layer >= self.max_layer {
            return self;
        }
        let leaf = self.leaf_index(p);
        match &self.leaves[leaf] {
            Some(child) => child.find_correspond(p),
            None => self,
        }
    }

    /// 插入一个点（几何更新，对照 `voxel_map.cpp:219` `UpdateOctoTree`）。
    ///
    /// `camera_pos`：注册帧相机世界系位置（P10.11 法向对齐，见
    /// [`crate::voxel::VoxelMap::register_points`]）。
    pub fn insert(
        &mut self,
        p: Vector3<f64>,
        cov: Matrix3<f64>,
        opts: &VoxelMapOptions,
        camera_pos: &Vector3<f64>,
    ) {
        if !self.init {
            self.new_points += 1;
            self.temp_points.push(p);
            self.temp_covs.push(cov);
            let threshold = opts.layer_init_num[self.layer.min(4)];
            if self.temp_points.len() > threshold {
                self.init_octo_tree(opts, camera_pos);
            }
            return;
        }

        if self.plane.is_some() {
            // 成熟平面：丢弃新点；未成熟：累积并按阈值重拟合（对照
            // `voxel_map.cpp:229-246` 官方语义。P10.11 曾试滑动窗口
            // refit——悬停回归：成熟平面持续跟随估计漂移，深度把位置推
            // 向被拖平面，悬停 y 漂 2m、姿态漂 11°；已回退）
            if self.update_enable {
                self.new_points += 1;
                self.temp_points.push(p);
                self.temp_covs.push(cov);
                if self.new_points > opts.update_size_threshold {
                    self.refit_plane(opts, camera_pos);
                    self.new_points = 0;
                }
                if self.temp_points.len() >= opts.max_points_per_plane {
                    self.update_enable = false;
                    self.temp_points.clear();
                    self.temp_covs.clear();
                    self.new_points = 0;
                }
            }
            return;
        }

        // 非平面：细分递归
        if self.layer < self.max_layer {
            let leaf = self.leaf_index(p);
            if self.leaves[leaf].is_none() {
                let child = OctoNode::new(
                    self.layer + 1,
                    self.max_layer,
                    self.center + Self::leaf_offset(leaf, self.half_len / 2.0),
                    self.half_len / 2.0,
                    opts,
                );
                self.leaves[leaf] = Some(Box::new(child));
            }
            if let Some(child) = &mut self.leaves[leaf] {
                child.insert(p, cov, opts, camera_pos);
            }
        } else {
            // 最大层仍不判平面：按 update_enable 丢弃或累积
            self.insert_at_max_layer(p, cov, opts, camera_pos);
        }
    }

    /// 最大层节点的累积/丢弃逻辑（与成熟平面一致）。
    fn insert_at_max_layer(
        &mut self,
        p: Vector3<f64>,
        cov: Matrix3<f64>,
        opts: &VoxelMapOptions,
        camera_pos: &Vector3<f64>,
    ) {
        if self.update_enable {
            self.new_points += 1;
            self.temp_points.push(p);
            self.temp_covs.push(cov);
            if self.new_points > opts.update_size_threshold {
                self.refit_plane(opts, camera_pos);
                self.new_points = 0;
            }
            if self.temp_points.len() >= opts.max_points_per_plane {
                self.update_enable = false;
                self.temp_points.clear();
                self.temp_covs.clear();
                self.new_points = 0;
            }
        }
    }

    /// 首次达到阈值时的初始化（拟合平面或细分）。
    fn init_octo_tree(&mut self, opts: &VoxelMapOptions, camera_pos: &Vector3<f64>) {
        let threshold = opts.layer_init_num[self.layer.min(4)];
        if self.temp_points.len() > threshold {
            let plane_opts = PlaneOptions::from(opts);
            if let Some(mut plane) = fit_plane(&self.temp_points, &self.temp_covs, &plane_opts) {
                // P10.11：法向对齐到指向相机（消除 SVD 符号歧义）
                align_normal_to_camera(&mut plane, camera_pos);
                self.plane = Some(plane);
                if self.plane.as_ref().is_some_and(|pl| pl.is_mature) {
                    self.update_enable = false;
                    self.temp_points.clear();
                    self.temp_covs.clear();
                }
            } else {
                self.cut_octo_tree(opts, camera_pos);
            }
            self.init = true;
            self.new_points = 0;
        }
    }

    /// 细分：把临时点分入子节点并递归初始化。
    fn cut_octo_tree(&mut self, opts: &VoxelMapOptions, camera_pos: &Vector3<f64>) {
        if self.layer >= self.max_layer {
            return;
        }
        let points = std::mem::take(&mut self.temp_points);
        let covs = std::mem::take(&mut self.temp_covs);
        for (p, cov) in points.into_iter().zip(covs) {
            let leaf = self.leaf_index(p);
            if self.leaves[leaf].is_none() {
                let child = OctoNode::new(
                    self.layer + 1,
                    self.max_layer,
                    self.center + Self::leaf_offset(leaf, self.half_len / 2.0),
                    self.half_len / 2.0,
                    opts,
                );
                self.leaves[leaf] = Some(Box::new(child));
            }
            if let Some(child) = &mut self.leaves[leaf] {
                child.insert(p, cov, opts, camera_pos);
            }
        }
    }

    /// 重拟合平面（增量更新，对照 `voxel_map.cpp:237`）。
    fn refit_plane(&mut self, opts: &VoxelMapOptions, camera_pos: &Vector3<f64>) {
        let plane_opts = PlaneOptions::from(opts);
        if let Some(mut plane) = fit_plane(&self.temp_points, &self.temp_covs, &plane_opts) {
            // P10.11：法向对齐到指向相机（消除 SVD 符号歧义）
            align_normal_to_camera(&mut plane, camera_pos);
            self.plane = Some(plane);
            if self.plane.as_ref().is_some_and(|pl| pl.is_mature) {
                self.update_enable = false;
                self.temp_points.clear();
                self.temp_covs.clear();
            }
        }
    }

    /// 点所在子节点索引（`4·dx + 2·dy + dz`，对照 `voxel_map.cpp:176`）。
    fn leaf_index(&self, p: Vector3<f64>) -> usize {
        let dx = usize::from(p[0] > self.center[0]);
        let dy = usize::from(p[1] > self.center[1]);
        let dz = usize::from(p[2] > self.center[2]);
        4 * dx + 2 * dy + dz
    }

    /// 子节点中心偏移（`(2·dx−1)·half_len` 各轴）。
    fn leaf_offset(leaf: usize, half: f64) -> Vector3<f64> {
        let dx = if leaf >= 4 { 1.0 } else { -1.0 };
        let dy = if leaf % 4 >= 2 { 1.0 } else { -1.0 };
        let dz = if leaf % 2 == 1 { 1.0 } else { -1.0 };
        Vector3::new(dx * half, dy * half, dz * half)
    }

    /// 遍历本节点子树中所有有效平面（深度优先）。
    pub fn collect_planes<'a>(&'a self, out: &mut Vec<&'a VoxelPlane>) {
        if let Some(plane) = self.plane.as_ref().filter(|p| p.is_plane) {
            out.push(plane);
            return; // 平面节点不再细分
        }
        for child in self.leaves.iter().flatten() {
            child.collect_planes(out);
        }
    }

    /// 本节点子树中的平面数量（调试/viz）。
    #[must_use]
    pub fn plane_count(&self) -> usize {
        if self.plane.as_ref().is_some_and(|p| p.is_plane) {
            1
        } else {
            self.leaves.iter().flatten().map(|c| c.plane_count()).sum()
        }
    }

    /// 本节点子树的最大深度。
    #[must_use]
    pub fn depth(&self) -> usize {
        self.leaves
            .iter()
            .flatten()
            .fold(self.layer, |acc, child| acc.max(child.depth()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::VoxelMapOptions;

    fn point_cloud_on_plane(n_points: usize, noise: f64) -> Vec<(Vector3<f64>, Matrix3<f64>)> {
        let cov = Matrix3::identity() * (noise * noise);
        (0..n_points)
            .map(|i| {
                let x = (i % 100) as f64 * 0.004;
                let y = (i / 100) as f64 * 0.004;
                let z = 0.5 + noise * ((i % 7) as f64 - 3.0) / 3.0;
                (Vector3::new(x, y, z), cov)
            })
            .collect()
    }

    #[test]
    fn fit_single_plane_params_accurate() {
        let opts = VoxelMapOptions::default();
        let pts = point_cloud_on_plane(10_000, 0.01);
        let points: Vec<_> = pts.iter().map(|(p, _)| *p).collect();
        let covs: Vec<_> = pts.iter().map(|(_, c)| *c).collect();
        let plane = fit_plane(&points, &covs, &PlaneOptions::from(&opts)).unwrap();
        assert!(plane.is_plane);
        // 法向应沿 z 轴（允许符号翻转）
        assert!(
            (plane.normal - Vector3::z_axis().into_inner()).norm() < 0.01
                || (plane.normal + Vector3::z_axis().into_inner()).norm() < 0.01
        );
        // 中心 = 平面 z 位置（0.5）
        assert!((plane.center[2] - 0.5).abs() < 0.05);
        // Σ_nq 有限
        assert!(plane.plane_var.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn octree_subdivides_nonplanar_and_discards() {
        // 球面点云（全局非平面）：根体素不判平面并触发细分，深度受限。
        // 点序经 Fisher-Yates 洗牌，保证首次拟合（200 点）即覆盖整个球面。
        let opts = VoxelMapOptions {
            layer_init_num: [200, 5, 5, 5, 5],
            ..VoxelMapOptions::default()
        };
        let mut node = OctoNode::new_root(Vector3::new(1.0, 1.0, 1.0), &opts);
        let cov = Matrix3::identity() * 1e-4;
        // 球面均匀采样（Fib 球），半径 0.4
        let n = 500;
        let mut pts: Vec<Vector3<f64>> = (0..n)
            .map(|i| {
                let y = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
                let r = (1.0 - y * y).sqrt();
                let phi = i as f64 * 2.399_963_229_728_653;
                Vector3::new(
                    1.0 + 0.4 * r * phi.cos(),
                    1.0 + 0.4 * r * phi.sin(),
                    1.0 + 0.4 * y,
                )
            })
            .collect();
        // 确定性洗牌（LCG）
        let mut seed = 0x9E37_79B9_7F4A_7C15u64;
        for i in (1..n).rev() {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let j = ((seed >> 33) % (i as u64 + 1)) as usize;
            pts.swap(i, j);
        }
        for p in pts {
            node.insert(p, cov, &opts, &Vector3::zeros());
        }
        // 深度不超过 max_layer
        assert!(node.depth() <= opts.max_layer);
        // 根节点不判平面（球面在 0.5m 体素尺度非平面）
        assert!(node.plane.is_none(), "球面点云根体素不应判为平面");
        // 细分确实发生（根节点有子节点）
        assert!(
            node.leaves.iter().flatten().next().is_some(),
            "非平面应触发细分"
        );
    }
}
