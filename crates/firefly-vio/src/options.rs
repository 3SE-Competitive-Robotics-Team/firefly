//! MSCKF 滤波器选项（对照 `OpenVINS` `ov_msckf` 的
//! `StateOptions.h`/`VioManagerOptions.h`/`UpdaterOptions.h`）。
//!
//! 只含 struct + 默认值；配置文件解析属 apps 层职责，这里不引入配置依赖。

use firefly_vio_core::noise::ImuNoise;

/// IMU 数值积分方式（对照 `StateOptions::IntegrationMethod`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegrationMethod {
    /// 零阶离散（`predict_mean_discrete`）。
    Discrete,
    /// 四阶 Runge–Kutta（默认）。
    Rk4,
    /// 解析闭式（ACII²，`predict_mean_analytic`）。
    Analytical,
}

/// IMU 固有误差模型（对照 `StateOptions::ImuModel`；与
/// `firefly-vio-core::imu_model::ImuModel` 同义，这里保留编排层独立枚举以
/// 镜像 C++ 结构）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuModel {
    /// kalibr 模型：尺度/错切按行主序填充。
    Kalibr,
    /// rpng 模型：尺度/错切按列主序填充。
    Rpng,
}

/// 特征表示（对照 `ov_type::LandmarkRepresentation`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatRepresentation {
    /// 全局 3D 坐标（默认）。
    Global3D,
    /// 全局全逆深度（`θ,φ,ρ`）。
    GlobalFullInverseDepth,
    /// 锚定 3D。
    Anchored3D,
    /// 锚定全逆深度（`θ,φ,ρ`）。
    AnchoredFullInverseDepth,
    /// 锚定 MSCKF 逆深度（MSCKF 用；`α,β,ρ = px/pz, py/pz, 1/pz`）。
    AnchoredMsckfInverseDepth,
    /// 锚定单逆深度（只估深度 `ρ`，方位锁定在首次观测）。
    AnchoredInverseDepthSingle,
}

impl FeatRepresentation {
    /// 是否为锚定（相对）表示（对照
    /// `LandmarkRepresentation::is_relative_representation`）。
    #[must_use]
    pub const fn is_relative(self) -> bool {
        matches!(
            self,
            Self::Anchored3D
                | Self::AnchoredFullInverseDepth
                | Self::AnchoredMsckfInverseDepth
                | Self::AnchoredInverseDepthSingle
        )
    }
}

/// 滤波器核心选项（对照 `StateOptions`，默认值与其成员初始化一致）。
// 与 C++ 结构 1:1 移植（6 个布尔开关），拆分反而破坏对照可审计性。
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone)]
pub struct StateOptions {
    /// 是否使用首估计雅可比（FEJ）。
    pub do_fej: bool,
    /// 数值积分方式。
    pub integration_method: IntegrationMethod,
    /// 是否在线标定相机-IMU 外参。
    pub do_calib_camera_pose: bool,
    /// 是否在线标定相机内参。
    pub do_calib_camera_intrinsics: bool,
    /// 是否在线标定相机-IMU 时间偏移。
    pub do_calib_camera_timeoffset: bool,
    /// 是否在线标定 IMU 内参（尺度/错切）。
    pub do_calib_imu_intrinsics: bool,
    /// 是否在线标定重力敏感度 Tg。
    pub do_calib_imu_g_sensitivity: bool,
    /// IMU 固有误差模型。
    pub imu_model: ImuModel,
    /// 滑动窗口克隆数上限。
    pub max_clone_size: usize,
    /// 最大估计 SLAM 特征数。
    pub max_slam_features: usize,
    /// 单次更新最多使用的 SLAM 特征数（对照 `max_slam_in_update`）。
    pub max_slam_in_update: usize,
    /// 单次更新最多使用的 MSCKF 特征数。
    pub max_msckf_in_update: usize,
    /// ARUCO 特征数上限。无 aruco 跟踪器时为 0（对照 C++ `VioManager` 未
    /// 配置 `TrackAruco` 时的语义：所有特征走 SLAM 分支）。
    pub max_aruco_features: usize,
    /// 相机数量。
    pub num_cameras: usize,
    /// MSCKF 特征表示。
    pub feat_rep_msckf: FeatRepresentation,
    /// SLAM 特征表示。
    pub feat_rep_slam: FeatRepresentation,
}

impl Default for StateOptions {
    fn default() -> Self {
        Self {
            do_fej: true,
            integration_method: IntegrationMethod::Rk4,
            do_calib_camera_pose: false,
            do_calib_camera_intrinsics: false,
            do_calib_camera_timeoffset: false,
            do_calib_imu_intrinsics: false,
            do_calib_imu_g_sensitivity: false,
            imu_model: ImuModel::Kalibr,
            max_clone_size: 11,
            max_slam_features: 25,
            max_slam_in_update: 1000,
            max_msckf_in_update: 1000,
            // 无 aruco 跟踪器 → 0（对照 C++ 未配置 TrackAruco）
            max_aruco_features: 0,
            num_cameras: 1,
            feat_rep_msckf: FeatRepresentation::Global3D,
            feat_rep_slam: FeatRepresentation::Global3D,
        }
    }
}

impl StateOptions {
    /// IMU 固有标定的误差态维度（对照 `State::imu_intrinsic_size`）：
    /// 内参（6+6，若开）→ +9（若开重力敏感）→ +3（若开旋转标定）——
    /// 注意与 `firefly-vio-core::propagation::PropagationOptions::imu_intrinsic_size`
    /// 保持一致（传播的 F 维度按同式构造）。
    #[must_use]
    pub fn imu_intrinsic_size(&self) -> usize {
        let mut sz = 0;
        if self.do_calib_imu_intrinsics {
            sz += 15;
            if self.do_calib_imu_g_sensitivity {
                sz += 9;
            }
        }
        sz
    }

    /// 传播子选项（firefly-vio-core 的 `PropagationOptions` 对照）。
    #[must_use]
    pub fn to_propagation_options(&self) -> firefly_vio_core::propagation::PropagationOptions {
        use firefly_vio_core::propagation::IntegrationMethod as PIm;
        firefly_vio_core::propagation::PropagationOptions {
            integration_method: match self.integration_method {
                IntegrationMethod::Discrete => PIm::Discrete,
                IntegrationMethod::Rk4 => PIm::Rk4,
                IntegrationMethod::Analytical => PIm::Analytical,
            },
            do_calib_imu_intrinsics: self.do_calib_imu_intrinsics,
            do_calib_imu_g_sensitivity: self.do_calib_imu_g_sensitivity,
            do_fej: self.do_fej,
            imu_model: match self.imu_model {
                ImuModel::Kalibr => firefly_vio_core::imu_model::ImuModel::Kalibr,
                ImuModel::Rpng => firefly_vio_core::imu_model::ImuModel::Rpng,
            },
        }
    }
}

/// 更新器选项（对照 `UpdaterOptions`：像素噪声 sigma 与 chi2 乘子）。
#[derive(Debug, Clone)]
pub struct UpdaterOptions {
    /// chi2 检验乘子（对照 `UpdaterOptions.h:36` 默认 5）。
    pub chi2_multipler: f64,
    /// 像素测量噪声 sigma（px），默认 1.0（对照 `UpdaterOptions.h:38`）。
    pub sigma_pix: f64,
    /// 像素测量噪声方差（`sigma_pix` 平方）。
    pub sigma_pix_sq: f64,
}

impl Default for UpdaterOptions {
    fn default() -> Self {
        Self {
            chi2_multipler: 5.0,
            sigma_pix: 1.0,
            sigma_pix_sq: 1.0,
        }
    }
}

/// VIO 管理器选项（对照 `VioManagerOptions`）。
///
/// 零速更新参数（对照 `VioManagerOptions` 的 zupt 系列字段）。
#[derive(Debug, Clone)]
pub struct ZeroVelocityOptions {
    /// 是否尝试零速更新（官方参数 `try_zupt`）。
    pub try_zero_velocity: bool,
    /// 超过该速度不做零速更新（官方参数 `zupt_max_velocity`）。
    pub max_velocity: f64,
    /// 测量噪声乘子（官方参数 `zupt_noise_multiplier`）。
    pub noise_multiplier: f64,
    /// 超过该平均视差不做零速更新（官方参数 `zupt_max_disparity`）。
    pub max_disparity: f64,
    /// 仅在初始化阶段使用（官方参数 `zupt_only_at_beginning`）。
    pub only_at_beginning: bool,
    /// 是否使用积分加速度约束（把"零速期间位移应为零"作为位置级测量；
    /// 对照 C++ 的 `integrated_accel_constraint`，默认 false——官方标注
    /// untested，由调用方决定是否开启）。
    pub integrated_accel_constraint: bool,
}

impl Default for ZeroVelocityOptions {
    fn default() -> Self {
        Self {
            try_zero_velocity: false,
            max_velocity: 1.0,
            noise_multiplier: 1.0,
            max_disparity: 1.0,
            only_at_beginning: false,
            integrated_accel_constraint: false,
        }
    }
}

/// 裁剪：ARUCO 相关选项（对应 updater 后续移植）不在此列出。
#[derive(Debug, Clone)]
pub struct VioManagerOptions {
    /// 核心状态选项。
    pub state_options: StateOptions,
    /// IMU 连续时间噪声。
    pub imu_noises: ImuNoise,
    /// 初始化器选项（对照 `VioManagerOptions::init_options`）。
    pub init_options: firefly_vio_init::options::InitOptions,
    /// MSCKF 更新选项。
    pub msckf_options: UpdaterOptions,
    /// SLAM 更新选项。
    pub slam_options: UpdaterOptions,
    /// 三角化参数（对照 C++ `feature_initializer_options`，经管理器下发两个更新器）。
    pub triangulation_options: firefly_vio_core::triangulation::TriangulationOptions,
    /// 初始化完成后 SLAM 特征可用前的延迟（对照 `dt_slam_delay`）。
    pub dt_slam_delay: f64,
    /// 零速更新参数。
    pub zero_velocity_options: ZeroVelocityOptions,
    /// GT 初始化时 bg/ba 的初始先验 σ（rad/s 与 m/s²）。默认 0.02（对照
    /// C++ `initialize_with_gt` 的诚实先验：允许视觉学习偏置）。MuJoCo 仿真
    /// 无真实偏置，视觉会把 KLT 亚像素偏置误学成 bg/ba（σ=0.02 下 bg 学到
    /// -0.03 rad/s → roll 以 1.9°/s 漂 → 位置二次发散；实测 34s 2704m vs
    /// 冻结 271m）。应用层可设小值（如 1e-6）声明"无偏置"场景。
    pub init_bias_sigma: f64,
}

impl Default for VioManagerOptions {
    fn default() -> Self {
        Self {
            state_options: StateOptions::default(),
            imu_noises: ImuNoise::default(),
            init_options: firefly_vio_init::options::InitOptions::default(),
            msckf_options: UpdaterOptions::default(),
            slam_options: UpdaterOptions::default(),
            triangulation_options: firefly_vio_core::triangulation::TriangulationOptions::default(),
            // 对照 C++ `VioManagerOptions::dt_slam_delay = 2.0`
            dt_slam_delay: 2.0,
            zero_velocity_options: ZeroVelocityOptions::default(),
            init_bias_sigma: 0.02,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_cpp() {
        let s = StateOptions::default();
        assert!(s.do_fej);
        assert_eq!(s.integration_method, IntegrationMethod::Rk4);
        assert_eq!(s.max_clone_size, 11);
        assert_eq!(s.max_msckf_in_update, 1000);
        assert_eq!(s.num_cameras, 1);
        assert!(!s.do_calib_camera_pose);
        assert!(!s.do_calib_imu_intrinsics);
        assert_eq!(s.imu_model, ImuModel::Kalibr);

        let v = VioManagerOptions::default();
        assert!((v.dt_slam_delay - 2.0).abs() < 1e-12);

        let u = UpdaterOptions::default();
        assert!((u.chi2_multipler - 5.0).abs() < 1e-12);
        // σ_pix=1.0（对照 UpdaterOptions.h:38 官方默认）
        assert!((u.sigma_pix - 1.0).abs() < 1e-12);
        assert!((u.sigma_pix_sq - 1.0).abs() < 1e-12);
    }

    #[test]
    fn intrinsic_size_follows_cpp() {
        let s = StateOptions::default();
        assert_eq!(s.imu_intrinsic_size(), 0);
        let s2 = StateOptions {
            do_calib_imu_intrinsics: true,
            ..StateOptions::default()
        };
        assert_eq!(s2.imu_intrinsic_size(), 15);
        let s3 = StateOptions {
            do_calib_imu_intrinsics: true,
            do_calib_imu_g_sensitivity: true,
            ..StateOptions::default()
        };
        assert_eq!(s3.imu_intrinsic_size(), 24);
    }

    #[test]
    fn propagation_options_roundtrip() {
        let s = StateOptions::default();
        let p = s.to_propagation_options();
        assert_eq!(
            p.integration_method,
            firefly_vio_core::propagation::IntegrationMethod::Rk4
        );
        assert!(!p.do_calib_imu_intrinsics);
    }
}
