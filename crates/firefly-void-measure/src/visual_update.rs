//! 稀疏直接视觉测量模型（论文 VII 节）。
//!
//! 残差对照论文 (22) 式：`0 = τ_k·I_k(u_i) − τ_r·I_r(u'_i)`，
//! `u_i = π(^C T_I·^G p_i)`（论文 (23) 式），参考补丁经仿射扭曲
//! （论文 (13) 式，`normal_refine::affine_warp` 同构）映射到当前帧。
//!
//! 雅可比对照 `vio.cpp:1611-1628`（状态旋转为右乘扰动
//! `R_wi ← R_wi·Exp(δθ)`，与 `State::boxplus` 一致，`common_lib.h:170`）：
//! - `p_cam = R_cw·p_w + p_cw`，由扰动推导（`R_ci = I`、`p_ci = 0`）：
//!   `∂p_cam/∂δθ = ⌊p_cam×⌋`，`∂p_cam/∂δp = −R_cw`；
//! - `J_img = τ_k·∇I_k/scale`（图像梯度，金字塔层内），
//!   `J_dpi = ∂u/∂p_cam`（`computeProjectionJacobian`，`vio.cpp:189-201`）；
//! - 旋转块 `JdR = J_img·J_dpi·⌊p_cam×⌋`，平移块 `Jdt = −J_img·J_dpi·R_cw`，
//!   曝光列 `Jdτ = I_k(u)`（`vio.cpp:1628`）。
//!
//! 外点剔除（论文 VII-A 末段）：
//! - 深度不连续：地图点深度与邻域深度差 > 阈值（对照 `vio.cpp:632`）；
//! - 视角过大：参考/当前视角余弦 < `min_view_cos`；
//! - 像素误差门控 `outlier_threshold·patch_size_total`（`vio.cpp:763`）；
//! - Huber 核（可配，`δ = ∞` 时关闭）。

use firefly_void_esikf::update::{EskfUpdater, InfoMatrix, MeasurementModel, visual_convergence};
use firefly_void_map::visual_point::VisualPointView;
use firefly_void_map::voxel::transform_point;
use firefly_void_types::state::{DIM_STATE, State};
use firefly_void_types::visual::{GrayImage, Intrinsics};
use nalgebra::{
    DMatrix, DVector, Isometry3, Matrix2, Matrix3, Rotation3, UnitQuaternion, Vector2, Vector3,
};

use crate::options::VisualOptions;

/// 相机内参矩阵 `K`。
#[must_use]
pub fn camera_matrix(intrinsics: &Intrinsics) -> Matrix3<f64> {
    Matrix3::new(
        intrinsics.fx,
        0.0,
        intrinsics.cx,
        0.0,
        intrinsics.fy,
        intrinsics.cy,
        0.0,
        0.0,
        1.0,
    )
}

/// 仿射扭曲矩阵（论文 (13) 式，参考系 → 当前系）。
///
/// `A = P(I_k R_{Ir} + I_k t_{Ir} · nᵀ/(nᵀ·p)) P⁻¹`，与
/// `firefly-void-map` 的 `normal_refine::affine_warp` 同构。
#[must_use]
pub fn affine_warp(
    pose_cur: &Isometry3<f64>,
    pose_ref: &Isometry3<f64>,
    normal_ref: &Vector3<f64>,
    p_ref: &Vector3<f64>,
    intrinsics: &Intrinsics,
) -> Matrix2<f64> {
    let k = camera_matrix(intrinsics);
    let k_inv = k.try_inverse().unwrap_or_else(Matrix3::identity);
    let t_rel = pose_cur * pose_ref.inverse();
    let r_rel = t_rel.rotation.to_rotation_matrix().into_inner();
    let t_vec = t_rel.translation.vector;
    let denom = normal_ref.dot(p_ref);
    if denom.abs() < 1e-12 {
        return Matrix2::identity();
    }
    let inner = r_rel + t_vec * (normal_ref.transpose() / denom);
    let a = k * inner * k_inv;
    Matrix2::new(a[(0, 0)], a[(0, 1)], a[(1, 0)], a[(1, 1)])
}

/// 图像双线性采样（越界返回 `None`）。
#[must_use]
pub fn bilinear_sample(image: &GrayImage, u: f64, v: f64) -> Option<f64> {
    if u < 0.0 || v < 0.0 || u > image.width() as f64 - 1.0 || v > image.height() as f64 - 1.0 {
        return None;
    }
    let x0 = u.floor() as usize;
    let y0 = v.floor() as usize;
    let x1 = (x0 + 1).min(image.width() - 1);
    let y1 = (y0 + 1).min(image.height() - 1);
    let fx = u - x0 as f64;
    let fy = v - y0 as f64;
    let v00 = f64::from(image.get(x0, y0)?);
    let v10 = f64::from(image.get(x1, y0)?);
    let v01 = f64::from(image.get(x0, y1)?);
    let v11 = f64::from(image.get(x1, y1)?);
    Some(
        (1.0 - fx) * (1.0 - fy) * v00
            + fx * (1.0 - fy) * v10
            + (1.0 - fx) * fy * v01
            + fx * fy * v11,
    )
}

/// 图像梯度（双线性采样的精确导数）。
///
/// `I(u,v)` 为像素双线性插值，其偏导由相邻像素差分给出
/// （与 `bilinear_sample` 的插值一致，避免中央差分与采样函数的
/// 离散化错配）。
#[must_use]
pub fn image_gradient(image: &GrayImage, u: f64, v: f64) -> Vector2<f64> {
    let u_i = u.floor() as i64;
    let v_i = v.floor() as i64;
    let width = image.width() as i64;
    let height = image.height() as i64;
    let get = |x: i64, y: i64| -> f64 {
        let x = x.clamp(0, width - 1) as usize;
        let y = y.clamp(0, height - 1) as usize;
        f64::from(image.get(x, y).unwrap_or(0))
    };
    // ∂I/∂u = I(u0+1, v) − I(u0, v)（双线性插值的精确偏导）
    let fu = v - v_i as f64;
    let du_bot = get(u_i + 1, v_i) - get(u_i, v_i);
    let du_top = get(u_i + 1, v_i + 1) - get(u_i, v_i + 1);
    let du = (1.0 - fu) * du_bot + fu * du_top;
    let fv = u - u_i as f64;
    let dv_left = get(u_i, v_i + 1) - get(u_i, v_i);
    let dv_right = get(u_i + 1, v_i + 1) - get(u_i + 1, v_i);
    let dv = (1.0 - fv) * dv_left + fv * dv_right;
    Vector2::new(du, dv)
}

/// 像素对相机系点的投影雅可比 `∂u/∂p`（对照 `vio.cpp:189-201`）。
#[must_use]
pub fn projection_jacobian(p_cam: &Vector3<f64>, intrinsics: &Intrinsics) -> Matrix2x3 {
    let z_inv = 1.0 / p_cam[2];
    let z_inv_2 = z_inv * z_inv;
    Matrix2x3::new(
        intrinsics.fx * z_inv,
        0.0,
        -intrinsics.fx * p_cam[0] * z_inv_2,
        0.0,
        intrinsics.fy * z_inv,
        -intrinsics.fy * p_cam[1] * z_inv_2,
    )
}

/// 2×3 矩阵别名。
pub type Matrix2x3 = nalgebra::Matrix2x3<f64>;

/// 补丁内双线性采样（越界截断到边缘，对照 `vio.cpp:311-315`
/// `interpolateMat_8u` 的边缘处理）。
#[must_use]
pub fn sample_patch_bilinear(data: &[f64], patch_size: usize, x: f64, y: f64) -> f64 {
    let x0 = x.floor().clamp(0.0, (patch_size - 1) as f64) as usize;
    let y0 = y.floor().clamp(0.0, (patch_size - 1) as f64) as usize;
    let x1 = (x0 + 1).min(patch_size - 1);
    let y1 = (y0 + 1).min(patch_size - 1);
    let fx = x - x0 as f64;
    let fy = y - y0 as f64;
    let v00 = data[y0 * patch_size + x0];
    let v10 = data[y0 * patch_size + x1];
    let v01 = data[y1 * patch_size + x0];
    let v11 = data[y1 * patch_size + x1];
    (1.0 - fx) * (1.0 - fy) * v00 + fx * (1.0 - fy) * v10 + (1.0 - fx) * fy * v01 + fx * fy * v11
}

/// 稀疏直接视觉测量模型（单个金字塔层）。
///
/// 由调用方按金字塔层从粗到细构造/复用（`level` 固化在模型中，
/// 对应官方 `computeJacobianAndUpdateEKF` 的逐层循环，`vio.cpp:790`）。
pub struct VisualMeasurement<'a> {
    /// 当前帧灰度图。
    image: &'a GrayImage,
    /// 可见视觉地图点（含参考补丁、参考位姿/曝光）。
    points: Vec<VisualPointView>,
    /// 当前帧深度图（`Some((data, width, height))`，用于深度不连续剔除）。
    depth: Option<(&'a [f64], usize, usize)>,
    intrinsics: Intrinsics,
    opts: VisualOptions,
    /// 金字塔层。
    level: usize,
    /// 扭曲后的参考补丁（`warp_patch`，`vio.cpp:739-742`），
    /// 逐点逐金字塔层，长度 `points.len() × patch_n`。
    ///
    /// 由入口状态冻结（对照官方 `vio.cpp:710-712`：`retrieveFromVisualSparseMap`
    /// 计算一次 A 并把参考补丁扭曲成 `warp_patch`，`updateState` 迭代内
    /// 保持不变；当前补丁在迭代中只随投影中心移动，`vio.cpp:1580-1621`）。
    warp_patches: Vec<Vec<f64>>,
}

impl<'a> VisualMeasurement<'a> {
    /// 构造（由调用方给定入口状态与预计算参考补丁扭曲）。
    #[must_use]
    pub fn new(
        image: &'a GrayImage,
        points: Vec<VisualPointView>,
        warp_patches: Vec<Vec<f64>>,
        intrinsics: Intrinsics,
        opts: VisualOptions,
        level: usize,
    ) -> Self {
        Self {
            image,
            points,
            intrinsics,
            opts,
            depth: None,
            level,
            warp_patches,
        }
    }

    /// 预计算仿射扭曲（参考系 → 当前系）。
    ///
    /// 由入口状态的相机位姿计算并冻结（对照官方 `vio.cpp:710-712`）。
    /// 法向须在参考相机系（`normal_refine::affine_warp` 的约定，
    /// 论文 (13) 式 `nᵀ·Ir p`）。
    #[must_use]
    pub fn compute_warps(
        points: &[VisualPointView],
        state: &State,
        intrinsics: &Intrinsics,
    ) -> Vec<Matrix2<f64>> {
        let cam_pose = Self::cam_pose_from_state(state);
        points
            .iter()
            .map(|point| {
                let p_ref_cam = transform_point(&point.ref_pose, &point.pos);
                let n_ref = point.ref_pose.rotation.inverse() * point.normal;
                affine_warp(&cam_pose, &point.ref_pose, &n_ref, &p_ref_cam, intrinsics)
            })
            .collect()
    }

    /// 扭曲参考补丁（`warp_patch`，对照 `vio.cpp:292-315` `warpAffine`）。
    ///
    /// 官方对参考原图在 `px_ref + A_ref_cur·(px_patch·(1<<search)·(1<<level))`
    /// 采样（`vio.cpp:308-311`），`updateState` 当前补丁在 `scale=(1<<(level+search))`
    /// 网格上采样（`vio.cpp:1566`）——warp 网格含 `(1<<search)·(1<<level)` 两级
    /// scale，与当前补丁重采样 scale 相消，等价于**在金字塔 level 层内做
    /// `A_ref_cur·off` 亚像素偏移**。本实现以参考补丁金字塔 level 层为源
    /// （层内索引 `(x,y)` 对应参考图 `start − half·scale + (x,y)·scale`，像素为
    /// 亚像素权重 `subpix=(px−start)/scale` 的四角插值，`extract_patch_pyramid`，
    /// `image_patch.rs:86-104`）：
    ///
    /// `local = subpix + half + A_ref_cur·off`
    ///
    /// 与官方逐像素一致（两级 scale 相消；`subpix` 为金字塔采样权重，对应官方
    /// `getImagePatch` 的 `w_ref_*`，`vio.cpp:210-211`）。`search_level` 在本实现
    /// 无意义：官方搜索层选择「参考图金字塔中哪一层匹配」，本实现参考补丁
    /// 金字塔即该匹配层，当前层网格由 `scale` 直接对应（`pyramid_update` 逐层
    /// 从粗到细，与官方 `updateState` 的 `level` 循环等价）。
    ///
    /// 早期实现用 `A_ref_cur·off/scale + half`（off 少乘 scale 又除 scale），
    /// `scale>1` 时参考补丁错位 `(scale−1)·off` 像素（实测 level1/2 错位
    /// 34/65 灰度），视觉残差在错误位置采样、一迭代即回退。
    #[must_use]
    pub fn compute_warp_patches(
        points: &[VisualPointView],
        warps: &[Matrix2<f64>],
        patch_size: usize,
        level: usize,
    ) -> Vec<Vec<f64>> {
        let half = (patch_size as i64) / 2;
        let scale = 1usize << level;
        points
            .iter()
            .zip(warps)
            .map(|(point, a)| {
                // 官方 A_ref_cur = A_cur_ref⁻¹（vio.cpp:296）
                let a_inv = a.try_inverse().unwrap_or_else(Matrix2::identity);
                let px = point.px;
                // 金字塔采样亚像素权重（官方 getImagePatch，vio.cpp:210-211）
                let start_u = (px[0] / scale as f64).floor() * scale as f64;
                let start_v = (px[1] / scale as f64).floor() * scale as f64;
                let subpix_u = (px[0] - start_u) / scale as f64;
                let subpix_v = (px[1] - start_v) / scale as f64;
                let ref_data = &point.ref_patch.levels[level.min(point.ref_patch.levels.len() - 1)];
                let mut out = Vec::with_capacity(patch_size * patch_size);
                for py in 0..patch_size {
                    for px_i in 0..patch_size {
                        let du = px_i as f64 - half as f64;
                        let dv = py as f64 - half as f64;
                        // 层内索引 = subpix + half + A_ref_cur·off（两级 scale 相消）
                        let u_ref =
                            half as f64 + subpix_u + a_inv[(0, 0)] * du + a_inv[(0, 1)] * dv;
                        let v_ref =
                            half as f64 + subpix_v + a_inv[(1, 0)] * du + a_inv[(1, 1)] * dv;
                        let value = sample_patch_bilinear(ref_data, patch_size, u_ref, v_ref);
                        out.push(value);
                    }
                }
                out
            })
            .collect()
    }

    /// 由状态构造当前相机位姿（世界系 → 相机系）。
    ///
    /// 深度相机与左目共面、外参近似单位阵（DESIGN.md §3），
    /// `^C T_G = ^C T_I · (^G T_I)⁻¹`；对照官方
    /// `Rcw = Rci·Rwiᵀ`、`Pcw = −Rci·Rwiᵀ·Pwi`（`vio.cpp:1542-1543`），
    /// 每轮迭代由当前状态重算。
    #[must_use]
    fn cam_pose_from_state(x: &State) -> Isometry3<f64> {
        let r_wi = x.rot.matrix();
        let p_wi = x.pos;
        let r_cw = r_wi.transpose();
        let p_cw = -r_cw * p_wi;
        Isometry3::from_parts(
            nalgebra::Translation3::from(p_cw),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r_cw)),
        )
    }

    /// 附带当前帧深度图（深度不连续剔除用）。
    #[must_use]
    pub fn with_depth(mut self, depth: &'a [f64], width: usize, height: usize) -> Self {
        self.depth = Some((depth, width, height));
        self
    }

    /// 补丁尺寸（来自参考补丁）。
    fn patch_size(&self) -> usize {
        self.points
            .first()
            .map_or(self.opts.patch_size, |p| p.ref_patch.patch_size)
    }

    /// 深度不连续剔除（对照 `vio.cpp:619-640`）：点深度与邻域深度差
    /// 超过阈值判外点。
    fn depth_discontinuous(&self, p_cam: &Vector3<f64>, px: &Vector2<f64>) -> bool {
        let Some((depth, width, height)) = self.depth else {
            return false;
        };
        let half = 1i64;
        let (u, v) = (px[0].floor() as i64, px[1].floor() as i64);
        for du in -half..=half {
            for dv in -half..=half {
                let uu = u + du;
                let vv = v + dv;
                if uu < 0 || vv < 0 || uu >= width as i64 || vv >= height as i64 {
                    continue;
                }
                let d = depth[vv as usize * width + uu as usize];
                if d > 0.0 && (p_cam[2] - d).abs() > self.opts.depth_discontinuity_thresh {
                    return true;
                }
            }
        }
        false
    }

    /// 单层有效测量数（R < 1e6 的逐像素行数；有效点门控用）。
    #[must_use]
    pub fn effective_count(&self, x: &State) -> usize {
        let (z, _, r) = self.level_residual(x);
        z.iter()
            .zip(r.diagonal().iter())
            .filter(|&(_, sig)| *sig < 1e6)
            .count()
    }

    /// 单层残差：对每个可见点，把预扭曲参考补丁（入口状态冻结）与
    /// 当前帧以投影中心为中心的固定网格补丁比较（论文 (21)(22) 式）。
    ///
    /// 逐像素建模（对照官方 `vio.cpp:1623`：`z(i·patch_total + …)` 逐像素
    /// 残差行），`R = img_point_cov` 标量协方差（官方 `config/avia.yaml:32`）。
    /// 官方用 `HᵀH/img_point_cov` 聚合（`vio.cpp:1660-1661`），与逐像素
    /// `R = img_point_cov·I` 的 ESIKF 信息矩阵等价。
    ///
    /// 早期实现把逐像素残差按点聚合为均值（`z = mean r_i`、`H = mean J_i`），
    /// 雅可比正负抵消导致信息矩阵近零、GN 步长≈0——视觉更新形同虚设
    /// （esikf 一迭代残差即升、0 步回退）。
    ///
    /// 固定维度契约：返回行数恒为 `points.len() × patch_n`（与 [`dim`] 一致，
    /// 对照官方 `H_DIM`，`vio.cpp:1530`）。无效像素（越界/深度不连续/视角/
    /// 误差门控）填零信息（`z=0, H=0, R=1e12`）。
    #[allow(clippy::many_single_char_names)] // 单字符 `z`/`h`/`r`/`k` 为论文 (21)(22) 式记号
    fn level_residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        let patch_size = self.patch_size();
        let half = (patch_size as i64) / 2;
        let patch_n = patch_size * patch_size;
        let n_points = self.points.len();
        let m = n_points * patch_n;
        let cam_pose = Self::cam_pose_from_state(x);
        let r_cw = cam_pose.rotation.to_rotation_matrix().into_inner();
        let p_cw = cam_pose.translation.vector;
        let zero_info = 1e12;

        // 固定维度 m = 点数 × patch_n（对照官方 `H_DIM = total_points *
        // patch_size_total`，vio.cpp:1530；无效像素 z/H 置零，官方
        // `z.setZero()` + 越界像素不写，`vio.cpp:1532`）。
        let mut zs = vec![0.0; m];
        let mut rs = vec![zero_info; m];
        let mut h = DMatrix::zeros(m, DIM_STATE);

        for (i, point) in self.points.iter().enumerate() {
            // 预扭曲参考补丁（入口状态冻结的 warp_patch）
            let warp_ref = &self.warp_patches[i];
            if warp_ref.len() != patch_n {
                continue;
            }
            let p_cam = r_cw * point.pos + p_cw;
            if p_cam[2] <= 0.0 {
                continue;
            }
            // 视角判据（论文 VII-A）：当前视角与平面法向夹角过大丢弃。
            // 法向朝相机时 `(−p_cam)·(R_cw·n) > 0`，近正对 ≈ 1。
            let view_cur = (-p_cam).normalize();
            let normal_cur = r_cw * point.normal;
            if view_cur.dot(&normal_cur) < self.opts.min_view_cos {
                continue;
            }
            let Some(px) = self.intrinsics.project(&p_cam) else {
                continue;
            };
            if self.depth_discontinuous(&p_cam, &px) {
                continue;
            }

            let j_dpi = projection_jacobian(&p_cam, &self.intrinsics);
            let p_hat = firefly_void_types::so3::skew(&p_cam);

            // 当前补丁以投影中心为锚点、scale 步长采样（官方 getImagePatch
            // 网格，vio.cpp:203-222；锚点含亚像素 = px，步长 scale = 1<<level）。
            // 与参考 warp 网格 `px_ref + A_ref_cur·off·scale` 对齐（残差逐像素
            // 比较同一网格坐标）。
            let scale = 1usize << self.level;
            let row0 = i * patch_n;

            let mut patch_error = 0.0;
            let mut n_pix = 0usize;
            for py in 0..patch_size {
                for px_i in 0..patch_size {
                    let du = px_i as f64 - half as f64;
                    let dv = py as f64 - half as f64;
                    // 官方 getImagePatch 网格（vio.cpp:203-222）：锚点 = 投影
                    // 中心（含亚像素），步长 scale
                    let u_l = px[0] + du * scale as f64;
                    let v_l = px[1] + dv * scale as f64;
                    let Some(cur_value) = bilinear_sample(self.image, u_l, v_l) else {
                        continue;
                    };
                    let ref_value = warp_ref[py * patch_size + px_i];
                    // 残差：τ_k·I_k − τ_r·I_r（论文 (22) 式，逐像素行）
                    let res = x.inv_expo_time * cur_value - point.ref_inv_expo * ref_value;
                    let row = row0 + py * patch_size + px_i;
                    zs[row] = res;
                    rs[row] = self.opts.img_point_cov;
                    patch_error += res * res;
                    n_pix += 1;

                    // 图像梯度（当前层）：官方 Jimg 含 `inv_scale`（vio.cpp:1613），
                    // 残差对状态导数用 `∇I/scale·scale = ∇I`（全分辨率步长采样时
                    // scale 步长采样使梯度为 `∇I·scale`，除以 scale 抵消）
                    let grad = image_gradient(self.image, u_l, v_l) * x.inv_expo_time;
                    let j_img = nalgebra::RowVector2::new(grad[0], grad[1]);
                    // 旋转块 JdR = J_img·J_dpi·⌊p_cam×⌋（1×3），
                    // 平移块 Jdt = −J_img·J_dpi·R_cw（1×3）
                    let j_drot = j_img * j_dpi * p_hat;
                    let j_dp = -j_img * j_dpi * r_cw;
                    // 曝光列（对照官方 `exposure_estimate_en` 分支，vio.cpp:1628）
                    let j_dtau = if self.opts.estimate_exposure {
                        cur_value // ∂h/∂τ_k = I_k(u)
                    } else {
                        0.0 // 固定曝光：τ 无自由度，避免迭代内推 τ 破坏残差
                    };

                    // 旋转块 3 列、平移块 3 列（j_drot/j_dp 为 1×3 行向量）
                    h[(row, 0)] = j_drot[0];
                    h[(row, 1)] = j_drot[1];
                    h[(row, 2)] = j_drot[2];
                    h[(row, 3)] = j_dp[0];
                    h[(row, 4)] = j_dp[1];
                    h[(row, 5)] = j_dp[2];
                    h[(row, 6)] = j_dtau;
                }
            }
            // 误差门控（对照 vio.cpp:763）：整点平均误差超阈值时整点置零信息
            if n_pix > 0 && patch_error > self.opts.outlier_threshold * patch_n as f64 {
                for row in row0..row0 + patch_n {
                    zs[row] = 0.0;
                    rs[row] = zero_info;
                    h.row_mut(row).fill(0.0);
                }
            }
        }

        let z_vec = DVector::from_column_slice(&zs);
        let r_mat = DMatrix::from_diagonal(&DVector::from_column_slice(&rs));
        (z_vec, h, r_mat)
    }

    /// 单层聚合信息量 `(S, b, e) = (HᵀR⁻¹H, HᵀR⁻¹z, zᵀR⁻¹z)`。
    ///
    /// 与 [`level_residual`] 同一测量模型，但直接在循环里把逐像素雅可比
    /// 累加进 19×19 信息矩阵，不构造 `m×19` 残差矩阵（视觉逐像素 67k 行
    /// 下避免单帧 ~2s 的分配/乘法开销；对照官方 `H_T_H = H_sub_T·H_sub`、
    /// `HTz = H_sub_T·z`，`vio.cpp:1660-1662`）。
    #[allow(clippy::many_single_char_names)] // 单字符 `s`/`b`/`e` 为信息量记号
    fn level_information(&self, x: &State) -> (InfoMatrix, DVector<f64>, f64) {
        let patch_size = self.patch_size();
        let half = (patch_size as i64) / 2;
        let patch_n = patch_size * patch_size;
        let cam_pose = Self::cam_pose_from_state(x);
        let r_cw = cam_pose.rotation.to_rotation_matrix().into_inner();
        let p_cw = cam_pose.translation.vector;
        let r_inv = 1.0 / self.opts.img_point_cov;

        let mut s = InfoMatrix::zeros();
        let mut b = DVector::zeros(DIM_STATE);
        let mut e = 0.0;

        for (i, point) in self.points.iter().enumerate() {
            let warp_ref = &self.warp_patches[i];
            if warp_ref.len() != patch_n {
                continue;
            }
            let p_cam = r_cw * point.pos + p_cw;
            if p_cam[2] <= 0.0 {
                continue;
            }
            let view_cur = (-p_cam).normalize();
            let normal_cur = r_cw * point.normal;
            if view_cur.dot(&normal_cur) < self.opts.min_view_cos {
                continue;
            }
            let Some(px) = self.intrinsics.project(&p_cam) else {
                continue;
            };
            if self.depth_discontinuous(&p_cam, &px) {
                continue;
            }

            let j_dpi = projection_jacobian(&p_cam, &self.intrinsics);
            let p_hat = firefly_void_types::so3::skew(&p_cam);

            let scale = 1usize << self.level;

            let mut patch_error = 0.0;
            let mut n_pix = 0usize;
            // 先算整点 patch_error 判门控，门控通过再累加信息（对照
            // `vio.cpp:763`：先算 patch_error 再决定是否用该点）
            let mut pixel_res: Vec<f64> = Vec::with_capacity(patch_n);
            let mut pixel_j: Vec<[f64; 7]> = Vec::with_capacity(patch_n);
            for py in 0..patch_size {
                for px_i in 0..patch_size {
                    let du = px_i as f64 - half as f64;
                    let dv = py as f64 - half as f64;
                    let u_l = px[0] + du * scale as f64;
                    let v_l = px[1] + dv * scale as f64;
                    let Some(cur_value) = bilinear_sample(self.image, u_l, v_l) else {
                        continue;
                    };
                    let ref_value = warp_ref[py * patch_size + px_i];
                    let res = x.inv_expo_time * cur_value - point.ref_inv_expo * ref_value;
                    patch_error += res * res;
                    n_pix += 1;

                    let grad = image_gradient(self.image, u_l, v_l) * x.inv_expo_time;
                    let j_img = nalgebra::RowVector2::new(grad[0], grad[1]);
                    let j_drot = j_img * j_dpi * p_hat;
                    let j_dp = -j_img * j_dpi * r_cw;
                    let j_dtau = if self.opts.estimate_exposure {
                        cur_value
                    } else {
                        0.0
                    };
                    pixel_res.push(res);
                    pixel_j.push([
                        j_drot[0], j_drot[1], j_drot[2], j_dp[0], j_dp[1], j_dp[2], j_dtau,
                    ]);
                }
            }
            if n_pix > 0 && patch_error > self.opts.outlier_threshold * patch_n as f64 {
                continue;
            }
            for (k, res) in pixel_res.iter().enumerate() {
                let hj = &pixel_j[k];
                // S += hᵀ·r_inv·h（7 维 → 19×19 累加）
                for a in 0..7 {
                    for bb in 0..7 {
                        s[(a, bb)] += hj[a] * hj[bb] * r_inv;
                    }
                }
                for a in 0..7 {
                    b[a] += hj[a] * res * r_inv;
                }
                e += res * res * r_inv;
            }
        }

        (s, b, e)
    }

    /// 以当前状态为初值，运行整条金字塔（粗 → 细）的顺序更新。
    ///
    /// 每层构造对应层的 [`VisualMeasurement`]（仿射扭曲与扭曲参考补丁
    /// 由当前状态冻结）并经 [`EskfUpdater`] 迭代收敛；层内重匹配
    /// （re-warp + 再更新，对照官方 Rematch Judgement，`voxel_map.cpp:
    /// 480-483`）后进入下一层。结果作为下一层初值（对照官方
    /// `computeJacobianAndUpdateEKF`，`vio.cpp:790-801`）。
    ///
    /// # Errors
    /// 转发 [`EskfUpdater::update`] 的收敛/维度错误。
    pub fn pyramid_update(
        image: &'a GrayImage,
        points: &[VisualPointView],
        depth: Option<(&'a [f64], usize, usize)>,
        intrinsics: Intrinsics,
        opts: VisualOptions,
        state: &mut State,
    ) -> Result<usize, firefly_error::Error> {
        let patch_size = points
            .first()
            .map_or(opts.patch_size, |p| p.ref_patch.patch_size);
        let mut total_iterations = 0;
        for level in (0..opts.pyramid_level).rev() {
            // 层内重匹配：warp 冻结 → esikf 更新 → 用新状态重算 warp。
            // 最细层多做两轮（对照官方 Rematch Judgement，
            // `voxel_map.cpp:480-483`）。
            let rematches = if level == 0 { 3 } else { 1 };
            for _rematch in 0..rematches {
                let warps = Self::compute_warps(points, state, &intrinsics);
                let warp_patches = Self::compute_warp_patches(points, &warps, patch_size, level);
                let model = Self::new(
                    image,
                    points.to_owned(),
                    warp_patches,
                    intrinsics,
                    opts,
                    level,
                )
                .with_depth_opt(depth);
                // 有效点门控：无有效测量才跳过本层（对照官方
                // `computeJacobianAndUpdateEKF` 的 `total_points == 0` 早退，
                // `vio.cpp:786`）。不再预跑 effective_count（逐像素模型下
                // 每层构建 67k 行矩阵的开销是重复的——esikf update 内部
                // 的零信息行自动忽略）。
                if model.points.is_empty() {
                    continue;
                }
                let mut updater =
                    EskfUpdater::new(model, opts.max_iterations, visual_convergence())
                        .with_error_descent_acceptance();
                let (iters, _) = updater.update(state)?;
                total_iterations += iters;
            }
        }
        Ok(total_iterations)
    }

    /// 附带深度图的构造（`pyramid_update` 内部用）。
    fn with_depth_opt(mut self, depth: Option<(&'a [f64], usize, usize)>) -> Self {
        self.depth = depth;
        self
    }
}

impl MeasurementModel for VisualMeasurement<'_> {
    fn residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        self.level_residual(x)
    }

    fn information(&self, x: &State) -> (InfoMatrix, DVector<f64>, f64) {
        self.level_information(x)
    }

    fn dim(&self) -> usize {
        self.points.len() * self.patch_size() * self.patch_size()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::INIT_INV_EXPO;
    use firefly_void_map::image_patch::PatchPyramid;
    use firefly_void_map::visual_point::VisualPointView;
    use nalgebra::{Rotation3, Translation3, UnitQuaternion};

    fn intrinsics() -> Intrinsics {
        Intrinsics::new(300.0, 300.0, 160.0, 120.0)
    }

    fn opts() -> VisualOptions {
        VisualOptions {
            pyramid_level: 1,
            max_iterations: 5,
            convergence_eps: 1e-6,
            img_point_cov: 20_000.0,
            ..VisualOptions::default()
        }
    }

    /// 合成平滑灰度图（连续梯度，直接对齐所需）。
    ///
    /// `sin/cos` 双频叠加：处处非零梯度，避免棋盘格的 piecewise-constant
    /// 梯度退化（直接法在该图像上梯度 0 处无信息、边缘处不连续）。
    fn smooth_image(width: usize, height: usize) -> GrayImage {
        let mut data = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                let u = x as f64 * 0.06;
                let v = y as f64 * 0.08;
                let value =
                    120.0 + 60.0 * (u.sin() * v.cos() + 0.5 * (2.0 * u).sin() * (3.0 * v).sin());
                data.push(value.round().clamp(0.0, 255.0) as u8);
            }
        }
        GrayImage::new(width, height, data)
    }

    /// 把世界系平面点投影到图像并提取参考补丁金字塔（3 层，11×11）。
    fn make_point(
        img: &GrayImage,
        pose: &Isometry3<f64>,
        intrinsics: &Intrinsics,
        pos: Vector3<f64>,
    ) -> VisualPointView {
        let p_cam = transform_point(pose, &pos);
        let px = intrinsics.project(&p_cam).unwrap();
        let ps = 11usize;
        let half = (ps as i64) / 2;
        let mut levels = Vec::with_capacity(3);
        for level in 0..3usize {
            let scale = 1usize << level;
            let mut data = Vec::with_capacity(ps * ps);
            for y in 0..ps {
                for x in 0..ps {
                    // 参考补丁金字塔：以投影中心为锚点、scale 步长采样
                    let u = px[0] + (x as i64 - half) as f64 * scale as f64;
                    let v = px[1] + (y as i64 - half) as f64 * scale as f64;
                    data.push(bilinear_sample(img, u, v).unwrap_or(0.0));
                }
            }
            levels.push(data);
        }
        let patch = PatchPyramid {
            levels,
            scale: vec![1, 2, 4],
            patch_size: ps,
        };
        VisualPointView {
            pos,
            normal: -Vector3::z_axis().into_inner(), // 面向相机（+z 光轴）
            ref_patch: patch,
            ref_pose: *pose,
            ref_inv_expo: INIT_INV_EXPO,
            px,
        }
    }

    /// 由参考图像按真值位姿重渲染当前帧：逐像素反投影到世界系平面
    /// `z=2`（法向 −z），经参考位姿投回参考图采样，再乘曝光系数。
    fn render_current_frame(
        ref_img: &GrayImage,
        intrinsics: &Intrinsics,
        truth_pose: &Isometry3<f64>,
        exposure: f64,
    ) -> GrayImage {
        let width = ref_img.width();
        let height = ref_img.height();
        // 世界系平面 z=2：p_w[2] = 2。p_w = Rᵀ·(t·d) − Rᵀ·t_cur
        let r = truth_pose.rotation.to_rotation_matrix().into_inner();
        let t_cur = truth_pose.translation.vector;
        let r_cam_world = r.transpose();
        let t_cam_world = -r_cam_world * t_cur;
        let mut data = Vec::with_capacity(width * height);
        for v in 0..height {
            for u in 0..width {
                // 当前帧像素 → 相机系单位方向
                let d = Vector3::new(
                    (u as f64 - intrinsics.cx) / intrinsics.fx,
                    (v as f64 - intrinsics.cy) / intrinsics.fy,
                    1.0,
                );
                // p_w = Rᵀ·(t·d) + t_cam_world，令 p_w[2] = 2 解 t
                let denom = r_cam_world * d;
                if denom[2].abs() < 1e-12 {
                    data.push(60);
                    continue;
                }
                let t = (2.0 - t_cam_world[2]) / denom[2];
                let p_w = t * denom + t_cam_world;
                // 参考相机系（参考位姿 = 世界系）
                let value = if p_w[2] > 0.0 {
                    intrinsics
                        .project(&p_w)
                        .and_then(|p| bilinear_sample(ref_img, p[0], p[1]))
                        .unwrap_or(60.0)
                } else {
                    60.0
                };
                data.push((value * exposure).round().clamp(0.0, 255.0) as u8);
            }
        }
        GrayImage::new(width, height, data)
    }

    #[test]
    fn visual_alignment_converges_to_truth() {
        // 合成平滑图像 + 已知相对位姿（带平移视差）+ 曝光差：金字塔
        // 对齐收敛后位姿误差 < 0.5°/1cm，τ 误差 < 5%。
        // 初始估计取真值附近（真实里程计帧间假设：上一帧估计在
        // 当前真值 1cm/0.5° 邻域内）。
        let ref_img = smooth_image(320, 240);
        let intrinsics = intrinsics();
        // 平移视差使深度可观测（纯正对平面存在尺度模糊）
        let truth_pose = Isometry3::from_parts(
            Translation3::new(0.04, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.02),
        );
        let ref_pose =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity());
        // 世界系平面 z=2 上的 25 个点（法向 −z 面向相机；≥ 有效点门控阈值）
        let mut points = Vec::new();
        for i in 0..25 {
            let x = -0.1 + f64::from(i % 5) * 0.05;
            let y = -0.1 + f64::from(i / 5) * 0.05;
            points.push(make_point(
                &ref_img,
                &ref_pose,
                &intrinsics,
                Vector3::new(x, y, 2.0),
            ));
        }
        let cur_img = render_current_frame(&ref_img, &intrinsics, &truth_pose, 1.2);
        // 状态持有 body-in-world 位姿 `(^G R_I, ^G p_I)`；
        // 相机位姿 `^C T_G = (R_wi, p_wi)⁻¹`，故真值状态 = 相机位姿求逆。
        let truth_rot = truth_pose.rotation.inverse().to_rotation_matrix();
        let truth_pos = -(truth_rot * truth_pose.translation.vector);
        let truth_state = State {
            rot: truth_rot,
            pos: truth_pos,
            inv_expo_time: 1.0 / 1.2,
            ..State::default()
        };
        // 在真值位姿下残差应 ≈ 0（渲染与模型一致性的检查，第 0 层）
        {
            let warps = VisualMeasurement::compute_warps(&points, &truth_state, &intrinsics);
            let warp_patches = VisualMeasurement::compute_warp_patches(&points, &warps, 11, 0);
            let m0 = VisualMeasurement::new(
                &cur_img,
                points.clone(),
                warp_patches,
                intrinsics,
                opts(),
                0,
            );
            let (z0, _, r0) = m0.residual(&truth_state);
            // 真值位姿下残差应≈0（允许补丁金字塔亚像素插值边界二阶差，
            // 最大 ~4 灰度，远低于 outlier 门控 31.6 灰度 RMS）
            assert!(
                z0.iter()
                    .zip(r0.diagonal().iter())
                    .filter(|&(_, sig)| *sig < 1e6)
                    .all(|(z, _)| z.abs() < 5.0),
                "真值位姿下残差应≈0"
            );
        }
        let mut x = State {
            // 初始估计：真值附近的传播先验（1cm / 0.5° 邻域，真实里程计
            // 帧间假设）；τ 初值取相对首帧的传播值
            rot: truth_rot,
            pos: truth_pos + Vector3::new(0.006, -0.004, 0.002),
            inv_expo_time: 1.0 / 1.2,
            ..State::default()
        };
        // τ 先验放宽（传播后协方差），让测量主导
        x.cov[(6, 6)] = 0.01;
        let iters =
            VisualMeasurement::pyramid_update(&cur_img, &points, None, intrinsics, opts(), &mut x)
                .unwrap();
        assert!(iters > 0);
        // 估计状态为 body-in-world：与真值状态比较
        let rot_err = UnitQuaternion::from_rotation_matrix(&x.rot)
            .angle_to(&UnitQuaternion::from_rotation_matrix(&truth_rot));
        let pos_err = (x.pos - truth_pos).norm();
        // 曝光 1.2 倍 → 真值 τ = 1/1.2
        let tau_truth = 1.0 / 1.2;
        assert!(
            (x.inv_expo_time - tau_truth).abs() / tau_truth < 0.05,
            "τ 误差 {:.3} 应 < 5%",
            (x.inv_expo_time - tau_truth).abs() / tau_truth
        );
        assert!(
            rot_err.to_degrees() < 0.5,
            "旋转误差 {:.4}° 应 < 0.5°（pos_err={pos_err}）",
            rot_err.to_degrees()
        );
        // 单平面正对视角存在深度（z）尺度模糊（平面单应零空间），
        // 切向（x/y）平移误差应 < 1cm；深度由深度测量模型约束
        // （见 tests/sequential_update.rs 的两模型顺序更新）。
        let tang_err = (x.pos - truth_pos).xy().norm();
        assert!(tang_err < 0.01, "切向位置误差 {tang_err} m 应 < 1cm");
    }

    #[test]
    #[allow(clippy::many_single_char_names)] // 数值检查中的单字符为误差状态/矩阵元素记号
    #[allow(clippy::too_many_lines)] // 数值雅可比对比逐元素检查，行数由检查密度决定
    fn visual_jacobian_matches_finite_difference() {
        // 解析 H 与数值雅可比一致（视觉残差对误差状态的导数）
        let ref_img = smooth_image(320, 240);
        let intrinsics = intrinsics();
        let ref_pose =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity());
        let mut points = Vec::new();
        for i in 0..8 {
            let x = -0.08 + f64::from(i % 4) * 0.05;
            let y = -0.08 + f64::from(i / 4) * 0.05;
            points.push(make_point(
                &ref_img,
                &ref_pose,
                &intrinsics,
                Vector3::new(x, y, 2.0),
            ));
        }
        // 当前帧：参考重渲染（无位姿变化）
        let cur_img = render_current_frame(&ref_img, &intrinsics, &Isometry3::identity(), 1.0);
        let x = State {
            pos: Vector3::new(0.005, -0.003, 0.0),
            rot: Rotation3::from_axis_angle(&Vector3::y_axis(), 0.005),
            ..State::default()
        };
        // 仿射扭曲由入口状态冻结（对照官方 vio.cpp:710-712），解析与
        // 数值都用同一组冻结 A/warp_patch。
        let warps = VisualMeasurement::compute_warps(&points, &x, &intrinsics);
        let warp_patches = VisualMeasurement::compute_warp_patches(&points, &warps, 11, 0);
        let m = VisualMeasurement::new(
            &cur_img,
            points.clone(),
            warp_patches.clone(),
            intrinsics,
            opts(),
            0,
        );
        let (z0, h_mat, r0) = m.residual(&x);
        eprintln!(
            "z0[0]={} r0[0][0]={} h0row=[{} {} {} | {} {} {} | {}]",
            z0[0],
            r0[(0, 0)],
            h_mat[(0, 0)],
            h_mat[(0, 1)],
            h_mat[(0, 2)],
            h_mat[(0, 3)],
            h_mat[(0, 4)],
            h_mat[(0, 5)],
            h_mat[(0, 6)]
        );

        let residual_fn = |s: &State| -> DVector<f64> {
            let mm = VisualMeasurement::new(
                &cur_img,
                points.clone(),
                warp_patches.clone(),
                intrinsics,
                opts(),
                0,
            );
            let (z, _, _) = mm.residual(s);
            z
        };
        let mut h_num = DMatrix::zeros(h_mat.nrows(), DIM_STATE);
        for j in 0..DIM_STATE {
            let mut dp = firefly_void_types::state::ErrorState::zeros();
            dp[j] = 1e-6;
            let zp = residual_fn(&x.boxplus(&dp));
            dp[j] = -1e-6;
            let zm = residual_fn(&x.boxplus(&dp));
            h_num.set_column(j, &((&zp - &zm) / 2e-6));
            if j == 1 {
                eprintln!(
                    "col1: zp[0]={} zm[0]={} ana[0][1]={}",
                    zp[0],
                    zm[0],
                    h_mat[(0, 1)]
                );
            }
        }
        // 只比较有效行（R < 1e6）
        let (_, _, r_mat) = m.residual(&x);
        let mut max_err = 0.0f64;
        let mut worst = (0usize, 0usize, 0.0, 0.0);
        let mut bad_rows = 0;
        for i in 0..h_mat.nrows() {
            if r_mat[(i, i)] >= 1e6 {
                continue;
            }
            let mut row_err: f64 = 0.0;
            for j in 0..DIM_STATE {
                let err_cell = (h_mat[(i, j)] - h_num[(i, j)]).abs();
                row_err = row_err.max(err_cell);
                if err_cell > max_err {
                    max_err = err_cell;
                    worst = (i, j, h_mat[(i, j)], h_num[(i, j)]);
                }
            }
            if row_err > 1e-3 {
                bad_rows += 1;
            }
        }
        eprintln!(
            "worst: row={} col={} ana={} num={} bad_rows={bad_rows}/{}",
            worst.0,
            worst.1,
            worst.2,
            worst.3,
            h_mat.nrows()
        );
        assert!(
            max_err < 1e-3,
            "视觉 H 与数值 H 最大误差 {max_err} 应 < 1e-3"
        );
    }

    #[test]
    fn depth_discontinuity_rejects_occluded() {
        // 遮挡外点剔除：注入深度不连续点，残差集不含外点（有效行数减少）
        let ref_img = smooth_image(320, 240);
        let intrinsics = intrinsics();
        let ref_pose =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity());
        let mut points = Vec::new();
        for i in 0..16 {
            let x = -0.1 + f64::from(i % 4) * 0.06;
            let y = -0.1 + f64::from(i / 4) * 0.06;
            points.push(make_point(
                &ref_img,
                &ref_pose,
                &intrinsics,
                Vector3::new(x, y, 2.0),
            ));
        }
        // 当前帧 = 参考（无位姿变化），深度图大部分与点深度一致，
        // 但 20% 点被遮挡（邻域深度差 > 阈值）
        let cur_img = render_current_frame(&ref_img, &intrinsics, &Isometry3::identity(), 1.0);
        let mut depth = vec![2.0; 320 * 240];
        for (i, p) in points.iter().enumerate() {
            if i % 5 == 0 {
                // 遮挡：邻域深度 = 1.0（前景遮挡物）
                let px = p.px;
                for dy in -2i64..=2 {
                    for dx in -2i64..=2 {
                        let u = (px[0] as i64 + dx).clamp(0, 319) as usize;
                        let v = (px[1] as i64 + dy).clamp(0, 239) as usize;
                        depth[v * 320 + u] = 1.0;
                    }
                }
            }
        }
        let warps = VisualMeasurement::compute_warps(&points, &State::default(), &intrinsics);
        let warp_patches = VisualMeasurement::compute_warp_patches(&points, &warps, 11, 0);
        let m = VisualMeasurement::new(
            &cur_img,
            points.clone(),
            warp_patches,
            intrinsics,
            opts(),
            0,
        )
        .with_depth(&depth, 320, 240);
        let (z, _, r) = m.residual(&State::default());
        // 固定维度 = 点数 × patch_n；遮挡 4 点（i%5==0）应置零信息
        assert_eq!(z.len(), 16 * 11 * 11, "固定维度 = 点数 × patch_n");
        let valid = z
            .iter()
            .zip(r.diagonal().iter())
            .filter(|&(_, sig)| *sig < 1e6)
            .count();
        assert!(valid > 0, "应有有效测量");
        // 12 个未遮挡点 × 121 像素，全部有效
        assert!(
            valid <= 12 * 11 * 11,
            "遮挡外点应被剔除，实际有效 {valid} 像素"
        );
    }
}
