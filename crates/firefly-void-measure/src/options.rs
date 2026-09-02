//! 测量模型参数配置。
//!
//! 对照 FAST-LIVO2 官方配置（`config/*.yaml` 的 `lio`/`vio` 段）与
//! 仿真深度噪声模型（`packages/firefly-mujoco/src/firefly_mujoco/env.py`）：
//! - 深度点不确定度：仿真视差域高斯 `σ_disp = 4·depth_noise` px、
//!   `σ_z ≈ z²·σ_disp/(f·B)`（`env.py:160-171`），官方 `dept_err`/`beam_err`
//!   （`config/avia.yaml:51-52`）在深度相机下退化为像素角不确定度；
//! - 视觉：`outlier_threshold`/`img_point_cov`（`config/avia.yaml:31-32`）。

/// 逆曝光时间初值 `τ₀ = 1`（论文 VII-B：`τ₀ = 1` 消除 (22) 式全零退化）。
pub const INIT_INV_EXPO: f64 = 1.0;

/// 仿真深度噪声参数（`env.py` 实际值）。
pub mod sim_depth_noise {
    /// 视差噪声标量（`depth_noise = 0.02`，`env.py:48`）。
    pub const DEPTH_NOISE: f64 = 0.02;
    /// 视差标准差系数（`σ_disp = 4·depth_noise` px，`env.py:166`）。
    pub const SIGMA_DISP_COEFF: f64 = 4.0;
    /// 双目基线（`B = 0.05` m，`env.py:162`）。
    pub const BASELINE: f64 = 0.05;
    /// 深度相机焦距（`f ≈ 168.6` px，由 fovy 70.88°/H=240 推出，`env.py:161`）。
    pub const DEPTH_FOCAL: f64 = 168.6;
    /// 空洞率范围（5~15%，`env.py:215`）。
    pub const HOLE_RATE_MIN: f64 = 0.05;
    pub const HOLE_RATE_MAX: f64 = 0.15;
}

/// 深度测量参数。
#[derive(Debug, Clone, Copy)]
pub struct DepthOptions {
    /// 距离不确定度系数 `σ_disp/(f·B)`（`env.py:166,161-162`）。
    /// 与仿真一致：`σ_z = z²·σ_disp/(f·B)`。
    pub depth_sigma_coeff: f64,
    /// 深度量化（像素角域，等效官方 `dept_err`，`config/avia.yaml:51`）。
    pub dept_err: f64,
    /// 方向不确定度角（rad，等效官方 `beam_err`，`config/avia.yaml:52`；
    /// 深度相机无束发散角，代之以 1/2 像素量化角）。
    pub beam_err: f64,
    /// 外点门控倍数 `sigma_num`（对照 `voxel_map.cpp:737`）。
    pub sigma_num: f64,
    /// 平面-点径向判据倍数 `radius_k`（对照 `voxel_map.cpp:719`）。
    pub radius_k: f64,
}

impl Default for DepthOptions {
    fn default() -> Self {
        use sim_depth_noise::{BASELINE, DEPTH_FOCAL, DEPTH_NOISE, SIGMA_DISP_COEFF};
        Self {
            depth_sigma_coeff: SIGMA_DISP_COEFF * DEPTH_NOISE / (DEPTH_FOCAL * BASELINE),
            dept_err: 0.02,
            beam_err: 0.01,
            sigma_num: 3.0,
            radius_k: 3.0,
        }
    }
}

/// 先验平面测量参数（P11.2：静态先验面批次）。
///
/// 与 [`DepthOptions`] 同构的门控参数 + 先验平面噪声放大：
/// 先验面来自离线建图/解析几何，其 `Σ_nq` 不可信时用 [`var_scale`]
/// 放大（诚实给大 σ，避免把在线估计拉偏——对照参考实现
/// `plan_weight_tan` 切向弱约束语义，`map_location.cpp:1750-1752`）。
#[derive(Debug, Clone, Copy)]
pub struct PriorOptions {
    /// 外点门控倍数 `sigma_num`（对照 `voxel_map.cpp:737`）。
    pub sigma_num: f64,
    /// 平面-点径向判据倍数 `radius_k`（对照 `voxel_map.cpp:719`）。
    pub radius_k: f64,
    /// 先验平面 `Σ_nq` 放大系数（各向同性乘子，默认 1.0 = 装载值原样）。
    pub var_scale: f64,
}

impl Default for PriorOptions {
    fn default() -> Self {
        Self {
            sigma_num: 3.0,
            radius_k: 3.0,
            var_scale: 1.0,
        }
    }
}

/// 视觉测量参数。
#[derive(Debug, Clone, Copy)]
pub struct VisualOptions {
    /// 逐像素测量噪声方差。
    ///
    /// 官方 `img_point_cov`（`config/avia.yaml:32`）为逐像素标量协方差
    /// （`HᵀH/img_point_cov`，`vio.cpp:1660-1661`）；本实现逐像素建模
    /// （`level_residual` 每像素一行、`R = img_point_cov`），与官方一致。
    pub img_point_cov: f64,
    /// 外点像素误差阈值（`outlier_threshold`，`config/avia.yaml:31`）。
    pub outlier_threshold: f64,
    /// 补丁边长（像素，论文 V-C 11×11；与地图 `patch_size` 一致）。
    pub patch_size: usize,
    /// 金字塔层数（`patch_pyrimid_level`，`config/avia.yaml:34`）。
    pub pyramid_level: usize,
    /// 每层迭代上限（`max_iterations`，`config/NTU_VIRAL.yaml:29`）。
    pub max_iterations: usize,
    /// 收敛判据：层内状态增量范数阈值（rad/m）。
    pub convergence_eps: f64,
    /// 外点核：Huber 阈值（像素灰度，可配，缺省关闭）。
    pub huber_delta: f64,
    /// 遮挡/深度不连续判据：当前深度与地图点深度差阈值（m）。
    pub depth_discontinuity_thresh: f64,
    /// 参考/当前视角余弦下限（论文 VII-A：视角 > 80° 丢弃）。
    pub min_view_cos: f64,
    /// 是否估计曝光（对照官方 `exposure_estimate_en`，`config/avia.yaml:37`）。
    ///
    /// 关闭时残差雅可比的曝光列（第 7 列，`vio.cpp:1628` 只在 en 时写）
    /// 置零——固定曝光（仿真）下 τ 无自由度，迭代内推 τ 会破坏已对齐
    /// 的光度残差（实测 GN 一步残差即升、状态零步回退）。
    pub estimate_exposure: bool,
}

impl Default for VisualOptions {
    fn default() -> Self {
        Self {
            // 官方 avia.yaml:32 逐像素噪声方差
            img_point_cov: 100.0,
            outlier_threshold: 1000.0,
            patch_size: 11,
            pyramid_level: 3,
            max_iterations: 5,
            convergence_eps: 1e-4,
            huber_delta: f64::INFINITY,
            depth_discontinuity_thresh: 0.5,
            min_view_cos: 0.17,      // 80°（论文 VII-A 末段）
            estimate_exposure: true, // 官方默认开启（config/avia.yaml:37）
        }
    }
}

/// 重定位初值参数。
#[derive(Debug, Clone, Copy)]
pub struct RelocalizeOptions {
    /// 候选数 `N`（协方差椭球内撒点）。
    pub n_candidates: usize,
    /// 点-平面配准迭代上限。
    pub max_iterations: usize,
}

impl Default for RelocalizeOptions {
    fn default() -> Self {
        Self {
            n_candidates: 32,
            max_iterations: 10,
        }
    }
}
