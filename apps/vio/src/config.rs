//! `configs/vio.toml` 加载：缺键回落默认值，文件缺失/解析失败报错。
//!
//! 默认值 = 随仓库发布的 `MuJoCo` 双目部署（与 `configs/vio.toml` 一致）。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;

/// 双目相机标定（同内参，IMU 居中）。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct Camera {
    pub width: usize,
    pub height: usize,
    /// 焦距（像素）。
    pub focal: f64,
    /// 基线（米）。
    pub baseline: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            width: 320,
            height: 240,
            // MuJoCo fovy=70.88° 推导：(H/2)/tan(fovy/2)
            focal: 168.607,
            baseline: 0.05,
        }
    }
}

/// IMU 连续时间噪声（须与传感器实际注入/数据手册匹配）。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct Imu {
    pub gyro_noise: f64,
    pub gyro_walk: f64,
    pub accel_noise: f64,
    pub accel_walk: f64,
}

impl Default for Imu {
    fn default() -> Self {
        // MuJoCo 注入：σ_gyro=0.002 rad/s、σ_accel=0.02 m/s² 每采样高斯
        // @200Hz → 连续谱密度 σ_cont = σ_disc·√fs
        Self {
            gyro_noise: 2.83e-2,
            gyro_walk: 1.9e-5,
            accel_noise: 2.83e-1,
            accel_walk: 3.0e-3,
        }
    }
}

/// 视觉前端。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct Frontend {
    /// 每相机 KLT 目标特征数。
    pub num_pts: usize,
}

impl Default for Frontend {
    fn default() -> Self {
        Self { num_pts: 300 }
    }
}

/// 估计器调参。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct Estimator {
    /// GT 初始化 bg/ba 先验 σ（sim 无偏置用 1e-6，真机 0.02）。
    pub init_bias_sigma: f64,
    /// 三角化深度/基线比上限。
    pub max_baseline: f64,
}

impl Default for Estimator {
    fn default() -> Self {
        Self {
            init_bias_sigma: 1e-6,
            max_baseline: firefly_vio_core::triangulation::TriangulationOptions::default()
                .max_baseline,
        }
    }
}

/// SLAM 路标与体素选点（默认关闭，保持现有纯 MSCKF 行为）。
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct Slam {
    /// 最大 SLAM 路标数（0 = 关闭 SLAM 分支）。
    pub max_slam_features: usize,
    /// 体素选点总开关（对照 Voxel-SVIO；开启要求 `max_slam_features > 0`）。
    pub voxel_selection: bool,
    /// 体素边长（米）。
    pub voxel_size: f64,
    /// 每体素上限点数。
    pub max_points_per_voxel: usize,
    /// 体素内最小点间距（米）。
    pub min_point_distance: f64,
}

impl Default for Slam {
    fn default() -> Self {
        let voxel = firefly_voxel_svio::VoxelOptions::default();
        Self {
            // OpenVINS 默认 25；10 轨迹 bench 均值 -8%（logs/bench suite_*_10x1x34s）
            max_slam_features: 25,
            voxel_selection: false,
            voxel_size: voxel.voxel_size,
            max_points_per_voxel: voxel.max_points_per_voxel,
            min_point_distance: voxel.min_point_distance,
        }
    }
}

/// `configs/vio.toml` 顶层。
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct VioConfig {
    pub camera: Camera,
    pub imu: Imu,
    pub frontend: Frontend,
    pub estimator: Estimator,
    pub slam: Slam,
}

impl VioConfig {
    /// 从 TOML 文件加载。
    ///
    /// # Errors
    ///
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
    use super::VioConfig;

    /// 随仓库发布的配置文件必须可解析且为完整部署值。
    #[test]
    fn shipped_config_parses() {
        let cfg = VioConfig::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/vio.toml"
        ))
        .expect("shipped configs/vio.toml must parse");
        assert_eq!(cfg.camera.width, 320);
        assert_eq!(cfg.camera.height, 240);
        assert!((cfg.camera.focal - 168.607).abs() < 1e-2);
        assert!((cfg.camera.baseline - 0.05).abs() < 1e-9);
        assert_eq!(cfg.frontend.num_pts, 300);
        assert!((cfg.estimator.max_baseline - 120.0).abs() < 1e-9);
    }

    /// 缺键回落默认值（最小化配置合法）。
    #[test]
    fn partial_file_falls_back_to_defaults() {
        let cfg: VioConfig = toml::from_str("[estimator]\nmax_baseline = 60.0").unwrap();
        assert_eq!(cfg.camera.width, 320);
        assert!((cfg.estimator.max_baseline - 60.0).abs() < 1e-9);
        assert!((cfg.estimator.init_bias_sigma - 1e-6).abs() < 1e-15);
        assert_eq!(cfg.slam.max_slam_features, 25);
        assert!(!cfg.slam.voxel_selection);
    }

    /// `[slam]` 开启 SLAM + 体素选点（A/B 实验配置）。
    #[test]
    fn slam_section_enables_voxel_selection() {
        let cfg: VioConfig = toml::from_str(
            "[slam]\nmax_slam_features = 25\nvoxel_selection = true\nvoxel_size = 0.2",
        )
        .unwrap();
        assert_eq!(cfg.slam.max_slam_features, 25);
        assert!(cfg.slam.voxel_selection);
        assert!((cfg.slam.voxel_size - 0.2).abs() < 1e-12);
        assert_eq!(cfg.slam.max_points_per_voxel, 5);
    }
}
