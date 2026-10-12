//! `configs/vio.toml` 加载：缺键回落默认值，缺文件使用默认值，读取/解析失败报错。
//!
//! 相机与 IMU 默认值对应 RMUC render 双目部署；体素选点由配置启用。

use std::path::Path;

use firefly_error::Result;
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
    /// 陀螺白噪声密度，rad/s/√Hz；采样标准差为 density/√Δt。
    pub gyro_noise: f64,
    pub gyro_walk: f64,
    /// 加速度白噪声密度，m/s²/√Hz；采样标准差为 density/√Δt。
    pub accel_noise: f64,
    pub accel_walk: f64,
}

impl Default for Imu {
    fn default() -> Self {
        // 连续密度与 MuJoCo 采样噪声同源；离散方差由实际测量周期决定。
        Self {
            gyro_noise: 2.0e-4,
            gyro_walk: 1.9e-5,
            accel_noise: 2.0e-3,
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
    /// 三角化深度/基线比上限。
    pub max_baseline: f64,
}

impl Default for Estimator {
    fn default() -> Self {
        Self {
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
    /// 查询邻域半径（体素格；0 = 单体素，1 = 27 邻域）。
    pub neighbor_radius: i32,
    /// 每体素喂入全部候选点（false = 每体素只取首点）。
    pub use_all_points: bool,
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
            neighbor_radius: voxel.neighbor_radius,
            use_all_points: voxel.use_all_points,
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
    /// 文件存在但不可读，或 TOML 解析失败。
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        firefly_config::load(path)
    }
}

#[cfg(test)]
mod tests {
    use super::{Slam, VioConfig};

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
        assert!(
            cfg.slam.voxel_selection,
            "部署配置启用体素地图（Voxel-SVIO）"
        );
        assert_eq!(cfg.slam.max_slam_features, 100);
        // 官方 EuRoC/TUM-VI 调参：0.1m/±1 的代码默认邻域命中过少。
        assert!((cfg.slam.voxel_size - 0.3).abs() < 1e-12);
        assert_eq!(cfg.slam.neighbor_radius, 2);
        assert!((cfg.slam.min_point_distance - 0.0).abs() < 1e-12);
    }

    #[test]
    fn imu_density_preserves_calibrated_integral_variance_across_rates() {
        let config = VioConfig::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/vio.toml"
        ))
        .unwrap();
        for imu in [config.imu, super::Imu::default()] {
            for rate in [100., 1000.] {
                // 同一秒积分噪声方差 = N·(σ_density²·fs)·Δt² = σ_density²。
                let gyro_integral_variance = rate * (imu.gyro_noise.powi(2) * rate) / rate.powi(2);
                let accel_integral_variance =
                    rate * (imu.accel_noise.powi(2) * rate) / rate.powi(2);
                assert!((gyro_integral_variance - 4e-8).abs() < 1e-18);
                assert!((accel_integral_variance - 4e-6).abs() < 1e-18);
            }
        }
    }

    /// 缺键回落默认值（最小化配置合法）。
    #[test]
    fn partial_file_falls_back_to_defaults() {
        let cfg: VioConfig = toml::from_str("[estimator]\nmax_baseline = 60.0").unwrap();
        assert_eq!(cfg.camera.width, 320);
        assert!((cfg.estimator.max_baseline - 60.0).abs() < 1e-9);
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

    /// 体素选点的邻域半径与全点开关可配置，缺省回落代码默认（对照
    /// `VoxelOptions`：1 / false）。
    #[test]
    fn slam_section_carries_voxel_query_options() {
        let cfg: VioConfig = toml::from_str(
            "[slam]\nvoxel_selection = true\nneighbor_radius = 0\nuse_all_points = true",
        )
        .unwrap();
        assert_eq!(cfg.slam.neighbor_radius, 0);
        assert!(cfg.slam.use_all_points);

        let default: Slam = toml::from_str("voxel_selection = true").unwrap();
        let voxel = firefly_voxel_svio::VoxelOptions::default();
        assert_eq!(default.neighbor_radius, voxel.neighbor_radius);
        assert_eq!(default.use_all_points, voxel.use_all_points);
    }
}
