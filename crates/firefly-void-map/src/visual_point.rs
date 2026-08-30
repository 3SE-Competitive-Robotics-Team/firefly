//! 视觉地图点与补丁金字塔（论文 V-C/V-D，对照 `visual_point.h` 与
//! `vio.cpp` 的 `VisualPoint`/`Feature`）。
//!
//! 视觉地图点 = 成熟/未成熟平面上的候选点 + 若干观测补丁：
//! - 补丁金字塔 3 层（论文 V-C：同尺寸 11×11，逐层半采样，`vio.cpp:203`
//!   `getImagePatch` 的 level 缩放）；
//! - 参考补丁评分（论文 (12) 式，`vio.cpp:1036-1096` `updateReferencePatch`）。

use nalgebra::{Isometry3, Matrix3, Vector2, Vector3};

use crate::image_patch::{PatchPyramid, extract_patch_pyramid};
use crate::options::VoxelMapOptions;

/// 一次观测（一个补丁金字塔 + 位姿/曝光）。
#[derive(Debug, Clone)]
pub struct PatchObservation {
    /// 观测帧号（用于"距上次增补 >20 帧"判据）。
    pub frame_id: u32,
    /// 观测位姿（世界系 → 相机系）。
    pub pose: Isometry3<f64>,
    /// 逆曝光时间。
    pub inv_expo_time: f64,
    /// 补丁金字塔（3 层）。
    pub patch: PatchPyramid,
    /// 补丁中心像素（当前帧）。
    pub px: Vector2<f64>,
    /// 评分（参考补丁更新时计算）。
    pub score: f64,
    /// 均值（NCC 缓存的补丁灰度均值，`vio.cpp:1054`）。
    pub mean: f64,
}

impl PatchObservation {
    /// 新建观测（提取补丁金字塔）。
    #[must_use]
    pub fn new(
        frame_id: u32,
        pose: Isometry3<f64>,
        inv_expo_time: f64,
        px: Vector2<f64>,
        image: &firefly_void_types::visual::GrayImage,
        opts: &VoxelMapOptions,
    ) -> Self {
        let patch = extract_patch_pyramid(image, px, opts);
        let mean = if patch.level0_mean().is_finite() {
            patch.level0_mean()
        } else {
            0.0
        };
        Self {
            frame_id,
            pose,
            inv_expo_time,
            patch,
            px,
            score: 0.0,
            mean,
        }
    }
}

/// 视觉地图点。
#[derive(Debug, Clone)]
pub struct VisualPoint {
    /// 世界系位置（单位 m）。
    pub pos: Vector3<f64>,
    /// 平面法向（世界系）。
    pub normal: Vector3<f64>,
    /// 法向信息矩阵（法向协方差之逆）。
    pub normal_information: Matrix3<f64>,
    /// 上次更新前的法向（用于收敛判据）。
    pub previous_normal: Vector3<f64>,
    /// 点协方差（世界系）。
    pub covariance: Matrix3<f64>,
    /// 观测集（补丁）。
    pub obs: Vec<PatchObservation>,
    /// 参考补丁索引（`obs[idx]`）。
    pub ref_patch: Option<usize>,
    /// 法向已初始化。
    pub normal_initialized: bool,
    /// 已收敛（法向固定，删除其余补丁）。
    pub converged: bool,
    /// 所属平面是否成熟（成熟平面只保留最近 50 点作候选）。
    pub from_mature_plane: bool,
}

impl VisualPoint {
    /// 构造（位置 + 首个观测）。
    #[must_use]
    pub fn new(pos: Vector3<f64>, covariance: Matrix3<f64>, normal: Vector3<f64>) -> Self {
        let normal = if normal.norm() > 1e-12 {
            normal.normalize()
        } else {
            Vector3::z_axis().into_inner()
        };
        Self {
            pos,
            normal,
            normal_information: Matrix3::identity(),
            previous_normal: normal,
            covariance,
            obs: Vec::new(),
            ref_patch: None,
            normal_initialized: true,
            converged: false,
            from_mature_plane: false,
        }
    }

    /// 添加观测（补丁增补，论文 V-C 条件由调用方检查）。
    pub fn add_observation(&mut self, obs: PatchObservation) {
        self.obs.push(obs);
    }

    /// 删除最低分观测（保持 obs 数量上限，对照 `vio.cpp:947-953`）。
    pub fn drop_lowest_score(&mut self) {
        if self.obs.is_empty() {
            return;
        }
        let mut min_idx = 0;
        for (i, o) in self.obs.iter().enumerate().skip(1) {
            if o.score < self.obs[min_idx].score {
                min_idx = i;
            }
        }
        self.obs.remove(min_idx);
    }

    /// 更新参考补丁（论文 (12) 式评分，对照 `vio.cpp:1036-1096`）。
    ///
    /// 对每个补丁 `f`：`S = (1−ω₁)·ΣNCC(f,gᵢ)/n + ω₁·c`，取最高分者为参考。
    /// `ω₁ = tr(Σ_n)/(1+e^{tr(Σ_n)})`，`c = n̂·p̂`（法向与视角余弦）。
    pub fn update_reference_patch(&mut self, opts: &VoxelMapOptions) {
        if self.obs.len() < opts.min_obs_for_score {
            return;
        }
        let tr_sigma = self
            .normal_information
            .try_inverse()
            .map_or(0.0, |cov| cov.trace());
        let omega1 = tr_sigma / (1.0 + tr_sigma.exp());
        let mut best_score = f64::NEG_INFINITY;
        let mut best_idx = 0;
        let n_obs = self.obs.len();
        for idx in 0..n_obs {
            // 视角余弦 c = n̂ · p̂（p 为观测位姿到点的方向，在观测系）
            let pf = crate::voxel::transform_point(&self.obs[idx].pose, &self.pos);
            let dir = if pf.norm() > 1e-12 {
                pf.normalize()
            } else {
                Vector3::zeros()
            };
            let norm_vec = self.obs[idx].pose.rotation * self.normal;
            let c = dir.dot(&norm_vec);

            // 平均 NCC（与其余补丁，第 0 层金字塔、均值中心化）
            let mut ncc_sum = 0.0;
            let mut count = 0;
            for j in 0..n_obs {
                if j == idx {
                    continue;
                }
                ncc_sum += self.obs[idx].patch.ncc_level0(&self.obs[j].patch);
                count += 1;
            }
            let avg_ncc = if count > 0 {
                ncc_sum / f64::from(count)
            } else {
                0.0
            };
            let score = (1.0 - omega1) * avg_ncc + omega1 * c;
            self.obs[idx].score = score;
            if score > best_score {
                best_score = score;
                best_idx = idx;
            }
        }
        self.ref_patch = Some(best_idx);
    }

    /// 收敛后固定参考补丁与法向、删除其余补丁（论文 V-E 末段）。
    pub fn finalize_converged(&mut self) {
        if let Some(idx) = self.ref_patch {
            let keep = self.obs.swap_remove(idx);
            self.obs.clear();
            self.obs.push(keep);
            self.ref_patch = Some(0);
            self.converged = true;
        }
    }

    /// 法向更新收敛判据（对照 `vio.cpp:1022-1030`）。
    #[must_use]
    pub fn normal_converged(&self, thresh: f64) -> bool {
        (self.normal - self.previous_normal).norm() < thresh
    }

    /// 当前观测数。
    #[must_use]
    pub fn obs_count(&self) -> usize {
        self.obs.len()
    }
}

/// 可见视图（`visible_map_points` 输出，P3 视觉测量模型输入）。
#[derive(Debug, Clone)]
pub struct VisualPointView {
    /// 世界系位置。
    pub pos: Vector3<f64>,
    /// 世界系法向。
    pub normal: Vector3<f64>,
    /// 参考补丁（第 0 层数据 + 尺寸）。
    pub ref_patch: PatchPyramid,
    /// 参考观测位姿。
    pub ref_pose: Isometry3<f64>,
    /// 参考观测逆曝光。
    pub ref_inv_expo: f64,
    /// 当前帧投影像素。
    pub px: Vector2<f64>,
}
