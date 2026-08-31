//! `configs/void.toml` 加载与 DIVO 管线参数。
//!
//! 缺键回落算法 crate 的 `*Options::default()`（配置最小化原则），
//! 文件缺失/解析失败报错（对照 `apps/vio/src/config.rs` 范式）。
//!
//! 坐标约定（MuJoCo 场景实测，`scene.py` 与 `firefly-map::DepthCamera`）：
//! - 深度相机为 OpenGL 系（前向 `-z_cam`、图 y 向上），反投影点
//!   `p_cam = (dx·z, dy·z, -z)`；
//! - 左目与深度相机同刚体（`xyaxes="0 -1 0  0 0 1"`：前向 `+x_body`、
//!   上 `+z_body`、右 `-y_body`），P3 视觉模型为针孔系（前向 `+z`、图 y 向下）；
//! - 估计器运行在**虚拟针孔相机系**（见 [`crate::VoidOdometry`]）：
//!   状态旋转 = 真实机体经 `R_body_to_cam` 旋转后的姿态，视觉外参单位阵；
//!   [`depth_ext_rot`] 为深度相机系 → 虚拟 IMU 系的旋转（默认已含两步
//!   `R_cam_to_body · R_body_to_cam`，见 [`default_depth_ext`]）。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use firefly_void_esikf::propagator::PropagationNoise;
use firefly_void_map::options::VoxelMapOptions;
use firefly_void_measure::options::{DepthOptions, VisualOptions};
use nalgebra::{Matrix3, Rotation3};
use serde::Deserialize;

/// 深度相机 → 虚拟 IMU 系旋转（行主序 3×3，默认值见 [`default_depth_ext`]）。
#[must_use]
pub const fn default_depth_ext() -> [[f64; 3]; 3] {
    // R_cam_to_body（OpenGL 系 → 真实机体，firefly-map::DepthCamera）
    // · R_body_to_cam（真实机体 → 虚拟针孔系，Mujoco 左目朝向标定）
    // = [[1,0,0],[0,-1,0],[0,0,-1]]：深度像素 (u,v) 与左目针孔像素逐点重合
    // （共面相机），仅 y/z 轴翻转。
    [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
}

/// 真实机体 → 虚拟针孔系旋转 `R_bv`（行主序 3×3，默认值见
/// [`default_body_ext`]）。IMU 的角速度/比力经它转到虚拟系后再进入
/// 传播（`ω_v = R_bv·ω_b`、`a_v = R_bv·a_b`）；初始姿态
/// `R_wv(0) = R_wb(0)·R_bvᵀ`。
#[must_use]
pub const fn default_body_ext() -> [[f64; 3]; 3] {
    // R_gl_to_pinhole · R_cam_to_bodyᵀ（firefly-map::DepthCamera 的
    // rot_cam_to_body 转置 = body → OpenGL 相机系，再经图 y 翻转转到针孔系）：
    // body 前向 +x → 针孔 +z、body 左 +y → 针孔 −x、body 上 +z → 针孔 −y。
    [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
}

/// IMU 传播噪声配置（字段 = [`PropagationNoise`]，默认值同算法 crate）。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct PropagationNoiseConfig {
    pub gyr: f64,
    pub acc: f64,
    pub bias_gyr: f64,
    pub bias_acc: f64,
    pub inv_expo: f64,
}

impl Default for PropagationNoiseConfig {
    fn default() -> Self {
        let n = PropagationNoise::default();
        // bias 随机游走收紧（sim 无真实偏置；默认值偏大，估计器会把
        // 测量偏差吸收进 ba，位置随后被拉走）
        Self {
            gyr: n.gyr,
            acc: n.acc,
            bias_gyr: n.bias_gyr / 10.0,
            bias_acc: n.bias_acc / 10.0,
            inv_expo: n.inv_expo,
        }
    }
}

impl From<&PropagationNoiseConfig> for PropagationNoise {
    fn from(c: &PropagationNoiseConfig) -> Self {
        Self {
            gyr: c.gyr,
            acc: c.acc,
            bias_gyr: c.bias_gyr,
            bias_acc: c.bias_acc,
            inv_expo: c.inv_expo,
        }
    }
}

/// 深度测量配置（字段 = [`DepthOptions`] + 点云下采样）。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct DepthConfig {
    /// 深度点云下采样体素边长（m，0.1m 网格内保留深度不确定度最大点；
    /// FAST-LIO2 常规取值；控制单帧测量数 ~2000）。
    pub downsample_voxel: f64,
    /// 深度上限（m）：超出丢弃（远距视差噪声 σ∝z² 爆炸，无信息）。
    pub max_range: f64,
    pub depth_sigma_coeff: f64,
    pub dept_err: f64,
    pub beam_err: f64,
    pub sigma_num: f64,
    pub radius_k: f64,
}

impl Default for DepthConfig {
    fn default() -> Self {
        let o = DepthOptions::default();
        Self {
            downsample_voxel: 0.1,
            // 深度上限 6m：覆盖场景立柱（侧向平面约束 roll/pitch，
            // 悬停姿态才可观）
            max_range: 6.0,
            depth_sigma_coeff: o.depth_sigma_coeff,
            dept_err: o.dept_err,
            beam_err: o.beam_err,
            sigma_num: o.sigma_num,
            radius_k: o.radius_k,
        }
    }
}

impl From<&DepthConfig> for DepthOptions {
    fn from(c: &DepthConfig) -> Self {
        Self {
            depth_sigma_coeff: c.depth_sigma_coeff,
            dept_err: c.dept_err,
            beam_err: c.beam_err,
            sigma_num: c.sigma_num,
            radius_k: c.radius_k,
        }
    }
}

/// 视觉测量配置（字段 = [`VisualOptions`]）。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct VisualConfig {
    pub img_point_cov: f64,
    pub outlier_threshold: f64,
    pub patch_size: usize,
    pub pyramid_level: usize,
    pub max_iterations: usize,
    pub convergence_eps: f64,
    pub huber_delta: f64,
    pub depth_discontinuity_thresh: f64,
    pub min_view_cos: f64,
}

impl Default for VisualConfig {
    fn default() -> Self {
        let o = VisualOptions::default();
        Self {
            img_point_cov: o.img_point_cov,
            outlier_threshold: o.outlier_threshold,
            patch_size: o.patch_size,
            pyramid_level: o.pyramid_level,
            max_iterations: o.max_iterations,
            convergence_eps: o.convergence_eps,
            huber_delta: o.huber_delta,
            depth_discontinuity_thresh: o.depth_discontinuity_thresh,
            min_view_cos: o.min_view_cos,
        }
    }
}

impl From<&VisualConfig> for VisualOptions {
    fn from(c: &VisualConfig) -> Self {
        Self {
            img_point_cov: c.img_point_cov,
            outlier_threshold: c.outlier_threshold,
            patch_size: c.patch_size,
            pyramid_level: c.pyramid_level,
            max_iterations: c.max_iterations,
            convergence_eps: c.convergence_eps,
            huber_delta: c.huber_delta,
            depth_discontinuity_thresh: c.depth_discontinuity_thresh,
            min_view_cos: c.min_view_cos,
        }
    }
}

/// 体素地图配置（字段 = [`VoxelMapOptions`]，`fov` 以度计便于配置）。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct MapConfig {
    pub root_size: f64,
    pub max_layer: usize,
    pub layer_init_num: [usize; 5],
    pub max_points_per_plane: usize,
    pub update_size_threshold: usize,
    pub planer_threshold: f64,
    pub half_map_size: i64,
    pub sliding_thresh: f64,
    pub grid_size: usize,
    pub patch_size: usize,
    pub patch_pyramid_level: usize,
    pub patch_add_frame_gap: u32,
    pub patch_add_pixel_dist: f64,
    pub normal_converge_thresh: f64,
    pub max_obs_per_point: usize,
    pub min_obs_for_score: usize,
    pub min_obs_for_converge: usize,
    pub fov_deg: f64,
    pub ray_depth_min: f64,
    pub ray_depth_max: f64,
}

impl Default for MapConfig {
    fn default() -> Self {
        let o = VoxelMapOptions::default();
        Self {
            root_size: o.root_size,
            max_layer: o.max_layer,
            layer_init_num: o.layer_init_num,
            max_points_per_plane: o.max_points_per_plane,
            update_size_threshold: o.update_size_threshold,
            planer_threshold: o.planer_threshold,
            half_map_size: o.half_map_size,
            sliding_thresh: o.sliding_thresh,
            grid_size: o.grid_size,
            patch_size: o.patch_size,
            patch_pyramid_level: o.patch_pyramid_level,
            patch_add_frame_gap: o.patch_add_frame_gap,
            patch_add_pixel_dist: o.patch_add_pixel_dist,
            normal_converge_thresh: o.normal_converge_thresh,
            max_obs_per_point: o.max_obs_per_point,
            min_obs_for_score: o.min_obs_for_score,
            min_obs_for_converge: o.min_obs_for_converge,
            fov_deg: o.fov.to_degrees(),
            ray_depth_min: o.ray_depth_min,
            ray_depth_max: o.ray_depth_max,
        }
    }
}

impl From<&MapConfig> for VoxelMapOptions {
    fn from(c: &MapConfig) -> Self {
        Self {
            root_size: c.root_size,
            max_layer: c.max_layer,
            layer_init_num: c.layer_init_num,
            max_points_per_plane: c.max_points_per_plane,
            update_size_threshold: c.update_size_threshold,
            planer_threshold: c.planer_threshold,
            half_map_size: c.half_map_size,
            sliding_thresh: c.sliding_thresh,
            grid_size: c.grid_size,
            patch_size: c.patch_size,
            patch_pyramid_level: c.patch_pyramid_level,
            patch_add_frame_gap: c.patch_add_frame_gap,
            patch_add_pixel_dist: c.patch_add_pixel_dist,
            normal_converge_thresh: c.normal_converge_thresh,
            max_obs_per_point: c.max_obs_per_point,
            min_obs_for_score: c.min_obs_for_score,
            min_obs_for_converge: c.min_obs_for_converge,
            fov: c.fov_deg.to_radians(),
            ray_depth_min: c.ray_depth_min,
            ray_depth_max: c.ray_depth_max,
        }
    }
}

/// `configs/void.toml` 顶层。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct VoidOptions {
    /// 初始位置（全局系，m；缺省与 `MuJoCo` 起点 `SIM_START=[1.0,4.0,1.0]` 一致）。
    pub t0: [f64; 3],
    /// 曝光时间估计开关（仿真固定曝光：关闭则 τ 恒 1，视觉残差
    /// `I_k − I_r` 无曝光自由度——实测 τ 随机游走会把位置拉偏）。
    pub estimate_exposure: bool,
    pub imu: PropagationNoiseConfig,
    pub depth: DepthConfig,
    pub visual: VisualConfig,
    pub map: MapConfig,
    /// 深度相机 → 虚拟 IMU 系旋转（行主序 3×3，缺省 [`default_depth_ext`]）。
    pub depth_ext_rot: [[f64; 3]; 3],
    /// 真实机体 → 虚拟 IMU 系旋转 `R_bv`（行主序 3×3，缺省 [`default_body_ext`]）。
    pub body_ext_rot: [[f64; 3]; 3],
}

impl Default for VoidOptions {
    fn default() -> Self {
        Self {
            t0: [1.0, 4.0, 1.0],
            estimate_exposure: false,
            imu: PropagationNoiseConfig::default(),
            depth: DepthConfig::default(),
            visual: VisualConfig::default(),
            map: MapConfig::default(),
            depth_ext_rot: default_depth_ext(),
            body_ext_rot: default_body_ext(),
        }
    }
}

impl VoidOptions {
    /// 深度相机 → 虚拟 IMU 系旋转（`Isometry3` 形式，纯旋转无平移）。
    #[must_use]
    pub fn depth_ext_isometry(&self) -> nalgebra::Isometry3<f64> {
        Self::rot_isometry(&self.depth_ext_rot)
    }

    /// 真实机体 → 虚拟 IMU 系旋转 `R_bv`（`Isometry3` 形式，纯旋转无平移）。
    #[must_use]
    pub fn body_ext_isometry(&self) -> nalgebra::Isometry3<f64> {
        Self::rot_isometry(&self.body_ext_rot)
    }

    /// 行主序 3×3 数组 → 纯旋转 `Isometry3`。
    fn rot_isometry(rot: &[[f64; 3]; 3]) -> nalgebra::Isometry3<f64> {
        let flat: Vec<f64> = rot.iter().flatten().copied().collect();
        let m = Matrix3::from_row_slice(&flat);
        nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, 0.0, 0.0),
            nalgebra::UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(m)),
        )
    }

    /// 从 TOML 文件加载。
    ///
    /// # Errors
    /// 文件不可读（`NotFound`）或 TOML 解析失败。
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let raw = std::fs::read_to_string(path)
            .map_err(|e| Error::new(ErrorKind::NotFound, "config file not found").with_source(e))?;
        toml::from_str(&raw)
            .map_err(|e| Error::new(ErrorKind::InvalidArgument, "invalid config").with_source(e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 随仓库发布的配置文件必须可解析且为部署值。
    #[test]
    fn shipped_config_parses() {
        let cfg = VoidOptions::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/void.toml"
        ))
        .expect("shipped configs/void.toml must parse");
        assert!((cfg.t0[0] - 1.0).abs() < 1e-12);
        assert!((cfg.t0[1] - 4.0).abs() < 1e-12);
        assert!((cfg.t0[2] - 1.0).abs() < 1e-12);
        assert!((cfg.depth.downsample_voxel - 0.1).abs() < 1e-12);
        assert!((cfg.depth.max_range - 6.0).abs() < 1e-12);
        assert_eq!(cfg.map.max_layer, 3);
    }

    /// 缺键回落默认值（最小化配置合法）。
    #[test]
    fn partial_file_falls_back_to_defaults() {
        let cfg: VoidOptions = toml::from_str("[depth]\ndownsample_voxel = 0.4").unwrap();
        assert!((cfg.depth.downsample_voxel - 0.4).abs() < 1e-12);
        assert!((cfg.depth.max_range - 6.0).abs() < 1e-12);
        assert!((cfg.map.root_size - 0.5).abs() < 1e-12);
        assert!((cfg.depth.depth_sigma_coeff - 0.08 / 8.43).abs() < 1e-6);
        assert!((cfg.t0[0] - 1.0).abs() < 1e-12);
    }

    /// 默认深度外参：深度像素与左目针孔像素逐点重合（共面相机）。
    #[test]
    fn default_depth_ext_maps_pixels_to_pinhole() {
        let cfg = VoidOptions::default();
        let ext = cfg.depth_ext_isometry();
        // OpenGL 系正前方点 (0, 0, -5) → 虚拟针孔系 (0, 0, 5)（前向 +z）
        let p =
            firefly_void_map::voxel::transform_point(&ext, &nalgebra::Vector3::new(0.0, 0.0, -5.0));
        assert!((p - nalgebra::Vector3::new(0.0, 0.0, 5.0)).norm() < 1e-12);
        // 上方像素点 (0, +0.1z, -z)（图 y 向上）→ 针孔 y 向下
        let p2 =
            firefly_void_map::voxel::transform_point(&ext, &nalgebra::Vector3::new(0.0, 0.1, -1.0));
        assert!((p2 - nalgebra::Vector3::new(0.0, -0.1, 1.0)).norm() < 1e-12);
    }
}
