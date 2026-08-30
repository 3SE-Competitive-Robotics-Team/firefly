//! 自适应体素地图主体（论文 V 节，对照 `voxel_map.h:187` `VoxelMapManager`）。
//!
//! 结构：哈希表 `root_size=0.5m` 根体素 → 八叉树细分（`octree.rs`）。
//! 职责：
//! - 几何构建与更新（`register_points`，论文 V-B）；
//! - 视觉地图点生成与补丁增补（`update_visual`，论文 V-C/D）；
//! - 可见性查询与光线投射（`raycast.rs`，论文 VII-A）；
//! - 局部地图滑窗（`on_update_end`，论文 V-A 末段，对照 `mapSliding`）。

use nalgebra::{Isometry3, Matrix3, Point3, Vector2, Vector3};
use std::collections::HashMap;

use crate::normal_refine;
use crate::octree::OctoNode;
use crate::options::VoxelMapOptions;
use crate::plane::VoxelPlane;
use crate::visual_point::{PatchObservation, VisualPoint, VisualPointView};

/// 刚体位姿变换点（nalgebra 的 `Isometry * Vector3` 不应用平移，
/// 点变换必须走 `transform_point`）。
#[must_use]
pub fn transform_point(pose: &Isometry3<f64>, p: &Vector3<f64>) -> Vector3<f64> {
    pose.transform_point(&Point3::from(*p)).coords
}

/// 根体素位置（整数索引，对照 `voxel_map.h:96` `VOXEL_LOCATION`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VoxelKey {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

impl VoxelKey {
    /// 由世界系点求根体素索引（负数向下取整，对照 `voxel_map.cpp:562-567`）。
    #[must_use]
    pub fn from_point(p: &Vector3<f64>, root_size: f64) -> Self {
        let floor = |v: f64| -> i64 {
            let q = v / root_size;
            if q < 0.0 {
                (q - 1.0).floor() as i64
            } else {
                q.floor() as i64
            }
        };
        Self {
            x: floor(p[0]),
            y: floor(p[1]),
            z: floor(p[2]),
        }
    }

    /// 根体素中心（世界系）。
    #[must_use]
    pub fn center(&self, root_size: f64) -> Vector3<f64> {
        Vector3::new(
            (self.x as f64 + 0.5) * root_size,
            (self.y as f64 + 0.5) * root_size,
            (self.z as f64 + 0.5) * root_size,
        )
    }
}

/// 根体素：八叉树 + 视觉点列表。
#[derive(Debug)]
pub struct RootVoxel {
    /// 八叉树根节点。
    pub octo: OctoNode,
    /// 挂在本根体素的视觉地图点（池索引）。
    pub visual_points: Vec<usize>,
}

/// 地图主体。
#[derive(Debug)]
pub struct VoxelMap {
    opts: VoxelMapOptions,
    /// 根体素哈希表。
    roots: HashMap<VoxelKey, RootVoxel>,
    /// 全局视觉点池。
    visual_pool: Vec<VisualPoint>,
    /// 上次滑窗位置。
    last_slide_pos: Vector3<f64>,
    /// 当前帧号。
    frame_id: u32,
}

impl VoxelMap {
    /// 构造地图。
    #[must_use]
    pub fn new(opts: VoxelMapOptions) -> Self {
        let center = Vector3::zeros();
        Self {
            opts,
            roots: HashMap::new(),
            visual_pool: Vec::new(),
            last_slide_pos: center,
            frame_id: 0,
        }
    }

    /// 几何更新：注册全局系点云（论文 V-B）。
    ///
    /// 逐点入根体素 → 八叉树插入（新体素 SVD 判平面，非平面细分至最大层丢弃，
    /// 成熟平面固定并丢弃新点）。`covs` 为各点世界系协方差。
    ///
    /// # Panics
    /// `points_g.len() != covs.len()` 时 panic。
    #[fastrace::trace]
    pub fn register_points(&mut self, points_g: &[Vector3<f64>], covs: &[Matrix3<f64>]) {
        assert_eq!(points_g.len(), covs.len(), "点与协方差数量必须一致");
        for (p, cov) in points_g.iter().zip(covs) {
            let key = VoxelKey::from_point(p, self.opts.root_size);
            let root = self.roots.entry(key).or_insert_with(|| RootVoxel {
                octo: OctoNode::new_root(key.center(self.opts.root_size), &self.opts),
                visual_points: Vec::new(),
            });
            root.octo.insert(*p, *cov, &self.opts);
        }
    }

    /// 视觉地图点生成与更新（论文 V-C/V-D）。
    ///
    /// 步骤：
    /// 1. 投影候选点（平面中心）到图像，每网格保留最小深度候选；
    /// 2. 30×30 网格：无视觉点的格子用梯度最高候选新建（挂补丁金字塔+
    ///    位姿+曝光+法向）；有视觉点的格子按帧间隔/像素偏移增补补丁；
    /// 3. 更新参考补丁（评分 (12) 式）。
    #[fastrace::trace]
    pub fn update_visual(
        &mut self,
        cam_pose: &Isometry3<f64>,
        gray: &firefly_void_types::visual::GrayImage,
        intrinsics: &firefly_void_types::visual::Intrinsics,
        state: &firefly_void_types::visual::VisualState,
    ) {
        self.frame_id = state.frame_id;
        let (cols, rows) = self.opts.grid_dims(gray.width(), gray.height());
        let grid_n = cols * rows;
        // 网格 → 候选（每格最小深度）
        let mut grid_best_depth: Vec<Option<f64>> = vec![None; grid_n];
        let mut grid_best_vp: Vec<Option<VisualPoint>> = vec![None; grid_n];

        // 1. 收集候选：遍历所有平面
        let mut planes: Vec<(&VoxelKey, Vector3<f64>, Vector3<f64>, bool)> = Vec::new();
        for (key, root) in &self.roots {
            let mut pl = Vec::new();
            root.octo.collect_planes(&mut pl);
            for plane in pl {
                planes.push((key, plane.center, plane.normal, plane.is_mature));
            }
        }
        for (_, center, normal, mature) in planes {
            if let Some(px) =
                Self::project_in_fov(&center, cam_pose, intrinsics, gray.width(), gray.height())
            {
                let (gx, gy) = (px[0] as usize, px[1] as usize);
                let col = (gx / self.opts.grid_size).min(cols - 1);
                let row = (gy / self.opts.grid_size).min(rows - 1);
                let idx = row * cols + col;
                let p_cam = transform_point(cam_pose, &center);
                if p_cam[2] <= 0.0 {
                    continue;
                }
                let keep = grid_best_depth[idx].is_none_or(|d| p_cam[2] < d);
                if keep {
                    grid_best_depth[idx] = Some(p_cam[2]);
                    let mut vp = VisualPoint::new(center, Matrix3::identity() * 1e-4, normal);
                    vp.from_mature_plane = mature;
                    grid_best_vp[idx] = Some(vp);
                }
            }
        }

        // 2. 网格遍历：新建或增补
        for (idx, best_vp) in grid_best_vp.iter_mut().enumerate() {
            let col = idx % cols;
            let row = idx / cols;
            let existing_id = self.grid_visual_point_id(col, row, cols, cam_pose, intrinsics);
            match existing_id {
                None => {
                    // 无视觉点：用梯度最高候选新建
                    if let Some(mut vp) = best_vp.take() {
                        let center_x = (col as f64 + 0.5) * self.opts.grid_size as f64;
                        let center_y = (row as f64 + 0.5) * self.opts.grid_size as f64;
                        let px = Vector2::new(center_x, center_y);
                        let obs = PatchObservation::new(
                            state.frame_id,
                            *cam_pose,
                            state.inv_expo_time,
                            px,
                            gray,
                            &self.opts,
                        );
                        vp.add_observation(obs);
                        // 观测不足评分阈值时首个补丁即参考（论文 V-D 前的初始参考）
                        if vp.obs.len() < self.opts.min_obs_for_score {
                            vp.ref_patch = Some(0);
                        } else {
                            vp.update_reference_patch(&self.opts);
                        }
                        let vp_id = self.visual_pool.len();
                        self.visual_pool.push(vp);
                        let key =
                            VoxelKey::from_point(&self.visual_pool[vp_id].pos, self.opts.root_size);
                        if let Some(root) = self.roots.get_mut(&key) {
                            root.visual_points.push(vp_id);
                        }
                    }
                }
                Some(vp_id) => {
                    // 已有视觉点：按条件增补补丁
                    self.append_patch_if_needed(vp_id, cam_pose, gray, intrinsics, state);
                }
            }
        }
    }

    /// 增补补丁（论文 V-C 条件：>20 帧或像素偏移 >40px），并做参考补丁更新
    /// 与法向精化（论文 V-D/V-E）。
    fn append_patch_if_needed(
        &mut self,
        vp_id: usize,
        cam_pose: &Isometry3<f64>,
        gray: &firefly_void_types::visual::GrayImage,
        intrinsics: &firefly_void_types::visual::Intrinsics,
        state: &firefly_void_types::visual::VisualState,
    ) {
        let p_world = self.visual_pool[vp_id].pos;
        let Some(px) = intrinsics.project(&transform_point(cam_pose, &p_world)) else {
            return;
        };
        if px[0] < 0.0
            || px[0] >= gray.width() as f64
            || px[1] < 0.0
            || px[1] >= gray.height() as f64
        {
            return;
        }

        let add = {
            let vp = &self.visual_pool[vp_id];
            match vp.obs.last() {
                Some(last) => {
                    let frame_gap = state.frame_id.saturating_sub(last.frame_id);
                    let pixel_dist = (px - last.px).norm();
                    frame_gap > self.opts.patch_add_frame_gap
                        || pixel_dist > self.opts.patch_add_pixel_dist
                }
                None => true,
            }
        };
        if !add {
            return;
        }

        let vp = &mut self.visual_pool[vp_id];
        if vp.obs.len() >= self.opts.max_obs_per_point {
            vp.drop_lowest_score();
        }
        let obs = PatchObservation::new(
            state.frame_id,
            *cam_pose,
            state.inv_expo_time,
            px,
            gray,
            &self.opts,
        );
        vp.add_observation(obs);
        vp.update_reference_patch(&self.opts);

        // 法向精化（独立线程由调用方经 mpsc 调度；此处同步执行）
        if let Some(n) = normal_refine::refine_normal(vp, intrinsics, 10) {
            let update = (n - vp.previous_normal).norm();
            vp.previous_normal = vp.normal;
            vp.normal = n;
            if update < self.opts.normal_converge_thresh
                && vp.obs_count() >= self.opts.min_obs_for_converge
            {
                vp.converged = true;
                vp.finalize_converged();
            }
        }
    }

    /// 网格内投影视觉点的池索引（保留最小深度者）。
    fn grid_visual_point_id(
        &self,
        col: usize,
        row: usize,
        cols: usize,
        cam_pose: &Isometry3<f64>,
        intrinsics: &firefly_void_types::visual::Intrinsics,
    ) -> Option<usize> {
        let grid = self.opts.grid_size as f64;
        let x_min = col as f64 * grid;
        let y_min = row as f64 * grid;
        let x_max = (col + 1) as f64 * grid;
        let y_max = (row + 1) as f64 * grid;
        let mut best_id = None;
        let mut best_depth = f64::INFINITY;
        for (id, vp) in self.visual_pool.iter().enumerate() {
            let Some(px) = intrinsics.project(&transform_point(cam_pose, &vp.pos)) else {
                continue;
            };
            if px[0] >= x_min && px[0] < x_max && px[1] >= y_min && px[1] < y_max {
                let p_cam = transform_point(cam_pose, &vp.pos);
                if p_cam[2] < best_depth {
                    best_depth = p_cam[2];
                    best_id = Some(id);
                }
            }
        }
        let _ = cols;
        best_id
    }

    /// 点在图像内且投影合法（FoV 初筛）。
    fn project_in_fov(
        p: &Vector3<f64>,
        cam_pose: &Isometry3<f64>,
        intrinsics: &firefly_void_types::visual::Intrinsics,
        width: usize,
        height: usize,
    ) -> Option<Vector2<f64>> {
        let px = intrinsics.project(&transform_point(cam_pose, p))?;
        if px[0] >= 0.0 && px[0] < width as f64 && px[1] >= 0.0 && px[1] < height as f64 {
            Some(px)
        } else {
            None
        }
    }

    /// 收集体素内视觉点视图（`visible_map_points`/`raycast` 用）。
    ///
    /// `key` 为根体素键；把该根体素内视觉点转成 [`VisualPointView`]。
    pub fn collect_visual_points(
        &self,
        key: &VoxelKey,
        cam_pose: &Isometry3<f64>,
        intrinsics: &firefly_void_types::visual::Intrinsics,
        out: &mut Vec<VisualPointView>,
    ) {
        let Some(root) = self.roots.get(key) else {
            return;
        };
        for &id in &root.visual_points {
            let vp = &self.visual_pool[id];
            let Some(ref_idx) = vp.ref_patch else {
                continue;
            };
            let Some(px) = intrinsics.project(&transform_point(cam_pose, &vp.pos)) else {
                continue;
            };
            let obs = &vp.obs[ref_idx];
            out.push(VisualPointView {
                pos: vp.pos,
                normal: vp.normal,
                ref_patch: obs.patch.clone(),
                ref_pose: obs.pose,
                ref_inv_expo: obs.inv_expo_time,
                px,
            });
        }
    }

    /// 地图滑窗（论文 V-A 末段，对照 `voxel_map.cpp:924` `mapSliding`）。
    ///
    /// 当前位置与上次滑窗位置距离超过阈值时，以当前位置为中心保留
    /// `[−half, +half]` 根体素，移出部分删除（环形缓冲语义）。
    pub fn on_update_end(&mut self, pos: &Vector3<f64>) {
        if (pos - self.last_slide_pos).norm() < self.opts.sliding_thresh {
            return;
        }
        self.last_slide_pos = *pos;
        let half = self.opts.half_map_size;
        let key_now = VoxelKey::from_point(pos, self.opts.root_size);
        let (x0, x1) = (key_now.x - half, key_now.x + half);
        let (y0, y1) = (key_now.y - half, key_now.y + half);
        let (z0, z1) = (key_now.z - half, key_now.z + half);
        self.roots.retain(|k, _| {
            k.x >= x0 && k.x <= x1 && k.y >= y0 && k.y <= y1 && k.z >= z0 && k.z <= z1
        });
    }

    /// 根体素查询（借出八叉树）。
    #[must_use]
    pub fn root_at(&self, p: &Vector3<f64>) -> Option<&OctoNode> {
        let key = VoxelKey::from_point(p, self.opts.root_size);
        self.roots.get(&key).map(|r| &r.octo)
    }

    /// 根体素键查询（借出八叉树）。
    #[must_use]
    pub fn root_at_key(&self, key: &VoxelKey) -> Option<&OctoNode> {
        self.roots.get(key).map(|r| &r.octo)
    }

    /// 遍历所有平面（viz/调试）。
    pub fn planes(&self) -> impl Iterator<Item = &VoxelPlane> {
        self.roots.values().flat_map(|root| {
            let mut planes = Vec::new();
            root.octo.collect_planes(&mut planes);
            planes.into_iter()
        })
    }

    /// 根体素数（滑窗容量验证用）。
    #[must_use]
    pub fn root_count(&self) -> usize {
        self.roots.len()
    }

    /// 视觉点总数。
    #[must_use]
    pub fn visual_point_count(&self) -> usize {
        self.visual_pool.len()
    }

    /// 视觉点池（测试/viz 用）。
    #[must_use]
    pub fn visual_points(&self) -> &[VisualPoint] {
        &self.visual_pool
    }

    /// 地图参数（测试用）。
    #[must_use]
    pub const fn options(&self) -> &VoxelMapOptions {
        &self.opts
    }
}

/// 图像灰度梯度幅值（中心差分，论文 V-C 梯度显著判据）。
#[must_use]
pub fn gray_gradient(gray: &firefly_void_types::visual::GrayImage, x: usize, y: usize) -> f64 {
    let w = gray.width();
    let h = gray.height();
    let get = |x: usize, y: usize| -> f64 {
        f64::from(gray.get(x.min(w - 1), y.min(h - 1)).unwrap_or(0))
    };
    let gx = if x > 0 && x + 1 < w {
        get(x + 1, y) - get(x - 1, y)
    } else {
        0.0
    };
    let gy = if y > 0 && y + 1 < h {
        get(x, y + 1) - get(x, y - 1)
    } else {
        0.0
    };
    (gx * gx + gy * gy).sqrt()
}
