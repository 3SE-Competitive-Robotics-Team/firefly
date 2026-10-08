//! 活跃轨迹跨帧射线交会（对照 Voxel-SVIO `voxelStereoVio::triangulateActiveTracks`
//! 的正规方程累积段，`stereoVio.cpp:627-775`）。
//!
//! 逐观测把「三维点落在该相机视线射线上」写成 3×3 正规方程 `A p = b`
//! （`A = B_perpᵀ B_perp`、`b = A p_CinG`，`B_perp = skew(b̂)`），跨帧累积；
//! 累积观测数 `≥ 4`（官方 `count > 3`）后求解，再用 SVD 条件数与相机系深度
//! 过滤，产出世界坐标。与 [`crate::VoxelMap`] 同级：只做几何累积，不碰滤波数学。
//!
//! 多相机同帧的累积行为逐行对照官方：官方 else 分支对「上帧已存在」的特征用
//! `A_new = Ai + A_prev` 赋值（同帧后一个相机覆盖前一个），而 if 分支用
//! `std::map::insert`（首相机写入后不再覆盖）——即每个特征每帧只累加一条射线。

use std::collections::HashMap;

use firefly_vio_types::quat_ops::skew_x;
use nalgebra::{Matrix3, Vector2, Vector3};

/// 官方最小累积观测数（`active_feat_linsys_count_new > 3`）。
const MIN_OBSERVATIONS: usize = 4;

/// 交会过滤阈值（对照 `FeatureInitializerOptions` 的 `max_cond_number`/
/// `min_dist`/`max_dist`）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntersectOptions {
    /// 正规方程 SVD 条件数上限。
    pub max_cond_number: f64,
    /// 相机系最小深度（米）。
    pub min_dist: f64,
    /// 相机系最大深度（米）。
    pub max_dist: f64,
}

impl Default for IntersectOptions {
    fn default() -> Self {
        Self {
            max_cond_number: 100_000.0,
            min_dist: 0.10,
            max_dist: 60.0,
        }
    }
}

/// 单条观测（对照官方一次 `last_ids[cam]` / `last_obs[cam]` 迭代）。
#[derive(Debug, Clone, Copy)]
pub struct RayObservation {
    /// 特征 id。
    pub featid: usize,
    /// 去畸变归一化像素 `(x/z, y/z)`。
    pub uv_norm: Vector2<f64>,
    /// 相机系到世界系旋转 `R_GtoCi`。
    pub rot_g_to_c: Matrix3<f64>,
    /// 相机原点在世界系位置 `p_CinG`。
    pub pos_c_in_g: Vector3<f64>,
}

/// 单特征累积的正规方程与观测计数。
#[derive(Debug, Clone, Copy)]
struct LinSys {
    a: Matrix3<f64>,
    b: Vector3<f64>,
    count: usize,
}

/// 活跃轨迹跨帧交会累积器。
#[derive(Debug, Clone)]
pub struct ActiveTrackTriangulator {
    options: IntersectOptions,
    accum: HashMap<usize, LinSys>,
}

impl ActiveTrackTriangulator {
    /// 构造。
    #[must_use]
    pub fn new(options: IntersectOptions) -> Self {
        Self {
            options,
            accum: HashMap::new(),
        }
    }

    /// 当前在跟踪的特征数（诊断用）。
    #[must_use]
    pub fn num_tracked(&self) -> usize {
        self.accum.len()
    }

    /// 处理一帧的全部观测（多相机合并），返回本帧解出并通过过滤的
    /// `(featid, p_G)`；同一特征以处理顺序后者覆盖前者。
    ///
    /// 累积器整体替换为「本帧观测 + 上帧累积」（对照官方
    /// `active_feat_linsys_* = *_new`）：本帧未被观测的特征自动淘汰。
    pub fn update(&mut self, observations: &[RayObservation]) -> Vec<(usize, Vector3<f64>)> {
        let mut new_accum: HashMap<usize, LinSys> = HashMap::with_capacity(self.accum.len());
        let mut solved: HashMap<usize, Vector3<f64>> = HashMap::new();
        for obs in observations {
            let bearing =
                obs.rot_g_to_c.transpose() * Vector3::new(obs.uv_norm.x, obs.uv_norm.y, 1.0);
            let Some(unit) = bearing.try_normalize(1e-12) else {
                continue;
            };
            let b_perp = skew_x(&unit);
            let ai = b_perp.transpose() * b_perp;
            let bi = ai * obs.pos_c_in_g;
            // 官方 else 分支（上帧已有）逐相机覆盖；if 分支（上帧没有）首相机
            // 写入后不再覆盖。
            let entry = if let Some(prev) = self.accum.get(&obs.featid) {
                let e = LinSys {
                    a: ai + prev.a,
                    b: bi + prev.b,
                    count: prev.count + 1,
                };
                new_accum.insert(obs.featid, e);
                e
            } else {
                *new_accum.entry(obs.featid).or_insert(LinSys {
                    a: ai,
                    b: bi,
                    count: 1,
                })
            };
            if entry.count >= MIN_OBSERVATIONS
                && let Some(p_g) = self.solve_and_check(&entry, obs)
            {
                solved.insert(obs.featid, p_g);
            }
        }
        self.accum = new_accum;
        solved.into_iter().collect()
    }

    /// 解正规方程并做条件数/深度过滤（对照官方 `count > 3` 后的 `colPivHouseholderQR`
    /// 求解与 `cond/z/isnan` 检查）。
    fn solve_and_check(&self, lin: &LinSys, obs: &RayObservation) -> Option<Vector3<f64>> {
        let p_g = lin.a.col_piv_qr().solve(&lin.b)?;
        let p_c = obs.rot_g_to_c * (p_g - obs.pos_c_in_g);
        let sv = lin.a.svd(false, false).singular_values;
        let cond = sv[0] / sv[sv.len() - 1];
        if !(cond.is_finite() && cond.abs() <= self.options.max_cond_number)
            || p_c.z < self.options.min_dist
            || p_c.z > self.options.max_dist
            || !p_c.norm().is_finite()
        {
            return None;
        }
        Some(p_g)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const POINT: Vector3<f64> = Vector3::new(0.0, 0.0, 5.0);

    /// 相机在世界系 `pos`、姿态为单位阵时观测世界点 `p_g` 的一条射线。
    fn ray(featid: usize, pos: Vector3<f64>, p_g: Vector3<f64>) -> RayObservation {
        let d = p_g - pos;
        RayObservation {
            featid,
            uv_norm: Vector2::new(d.x / d.z, d.y / d.z),
            rot_g_to_c: Matrix3::identity(),
            pos_c_in_g: pos,
        }
    }

    /// 四个不同位置的相机，射线方向互不退化。
    fn four_views() -> [Vector3<f64>; 4] {
        [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 0.5, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
        ]
    }

    #[test]
    fn emits_only_after_four_frames() {
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions::default());
        for (i, pos) in four_views().iter().enumerate() {
            let out = tri.update(&[ray(1, *pos, POINT)]);
            if i < 3 {
                assert!(out.is_empty(), "第 {} 帧不应产出", i + 1);
            } else {
                assert_eq!(out.len(), 1);
                assert!((out[0].1 - POINT).norm() < 1e-6, "got {}", out[0].1);
            }
        }
        assert_eq!(tri.num_tracked(), 1);
    }

    #[test]
    fn one_ray_per_feature_per_frame() {
        // 官方同帧多相机只保留最后一条射线：两帧 × 两相机后计数仍为 2，
        // 因此不足 4 帧不产出。
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions::default());
        let views = four_views();
        for frame in 0..2 {
            let obs = [
                ray(1, views[frame * 2], POINT),
                ray(1, views[frame * 2 + 1], POINT),
            ];
            assert!(tri.update(&obs).is_empty(), "第 {} 帧不应产出", frame + 1);
        }
    }

    #[test]
    fn drops_features_not_observed_this_frame() {
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions::default());
        let _ = tri.update(&[ray(1, Vector3::zeros(), POINT)]);
        assert_eq!(tri.num_tracked(), 1);
        let _ = tri.update(&[]);
        assert_eq!(tri.num_tracked(), 0, "本帧未观测的特征应被淘汰");
    }

    #[test]
    fn empty_observations_yield_nothing() {
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions::default());
        assert_eq!(tri.update(&[]), [] as [(usize, Vector3<f64>); 0]);
    }

    #[test]
    fn rejects_point_closer_than_min_dist() {
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions::default());
        let near = Vector3::new(0.0, 0.0, 0.05); // < min_dist = 0.10
        let views = four_views();
        for pos in &views {
            let _ = tri.update(&[ray(1, *pos, near)]);
        }
        // 深度过滤：最后一次调用仍不产出
        let out = tri.update(&[ray(1, views[0], near)]);
        assert!(out.is_empty(), "过近的点不应产出");
    }

    #[test]
    fn rejects_beyond_max_dist() {
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions {
            max_dist: 1.0,
            ..IntersectOptions::default()
        });
        let far = Vector3::new(0.0, 0.0, 5.0); // > max_dist = 1.0
        for pos in &four_views() {
            let _ = tri.update(&[ray(1, *pos, far)]);
        }
        assert_eq!(tri.num_tracked(), 1);
        let out = tri.update(&[ray(1, four_views()[0], far)]);
        assert!(out.is_empty(), "过远的点不应产出");
    }

    #[test]
    fn tight_condition_limit_rejects_degenerate_rays() {
        // 条件数上限极小 → 任何解都被拒。
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions {
            max_cond_number: 1e-12,
            ..IntersectOptions::default()
        });
        for pos in &four_views() {
            let _ = tri.update(&[ray(1, *pos, POINT)]);
        }
        let out = tri.update(&[ray(1, four_views()[0], POINT)]);
        assert!(out.is_empty(), "条件数超限应被拒");
    }

    #[test]
    fn separate_features_do_not_interfere() {
        let mut tri = ActiveTrackTriangulator::new(IntersectOptions::default());
        let point_b = Vector3::new(1.0, -1.0, 6.0);
        for pos in &four_views() {
            let _ = tri.update(&[ray(1, *pos, POINT), ray(2, *pos, point_b)]);
        }
        assert_eq!(tri.num_tracked(), 2);
        let out = tri.update(&[
            ray(1, four_views()[0], POINT),
            ray(2, four_views()[0], point_b),
        ]);
        assert_eq!(out.len(), 2);
        for (id, p) in out {
            let truth = if id == 1 { POINT } else { point_b };
            assert!((p - truth).norm() < 1e-6, "id {id} got {p}");
        }
    }
}
