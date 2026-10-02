//! `configs/localization.toml` 加载（纯数据 `Options`，缺键回落 `Default`）。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;

use crate::filter::FusionOptions;

/// 地图启动先验与视觉融合配置。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct LocalizationConfig {
    /// 固定启动机体位姿；属于部署配置，不来自在线真值。
    pub origin: Option<InitialAlignment>,
    /// 融合参数。
    pub fusion: FusionOptions,
    /// 默认统一位姿图；ESKF 用于无回环的对照运行，两者不叠加校正。
    pub backend: Backend,
    pub graph: crate::graph::GraphOptions,
    pub quality: crate::quality::QualityOptions,
}

/// 静止启动时机体在地图中的位置与航向；odom 航向规范由测量确定。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct InitialAlignment {
    /// 米，地图系。
    pub position: [f64; 3],
    /// 弧度，启动机体 +X 在地图水平面的航向。
    pub yaw: f64,
}
impl InitialAlignment {
    /// 用静止局部机体位姿计算 map←odom，保留 IMU 确定的重力方向。
    /// 对照 VINS-Fusion `pose_graph.cpp`：yaw 差与 `p_map − R_map_odom p_odom`。
    /// # Errors
    /// 未初始化、非有限、移动中或已离开局部启动原点。
    pub fn at_start(
        &self,
        odom: &firefly_pubsub::odom::OdomMessage,
    ) -> Result<firefly_base::RigidTransform> {
        use nalgebra::{UnitQuaternion, Vector3};
        let pose = odom.body_pose(firefly_base::FrameId::ODOM)?;
        let position = pose.isometry().translation.vector;
        let speed = Vector3::new(odom.velocity_x, odom.velocity_y, odom.velocity_z).norm();
        if !odom.is_initialized
            || !odom.timestamp.is_finite()
            || odom.timestamp < 0.
            || !speed.is_finite()
            || speed > 0.1
            || position.norm() > 0.1
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "fixed prior requires stationary initialized odometry at startup origin",
            ));
        }
        let forward = pose.vector(Vector3::x());
        if forward.xy().norm() < 1e-6 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "startup heading is undefined",
            ));
        }
        let yaw = self.yaw - forward.y.atan2(forward.x);
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
        let translation = Vector3::from(self.position) - rotation * position;
        let half = yaw * 0.5;
        firefly_base::RigidTransform::from_parts(
            firefly_base::FrameId::MAP,
            firefly_base::FrameId::ODOM,
            translation.into(),
            [0., 0., half.sin(), half.cos()],
        )
    }
}

impl LocalizationConfig {
    /// 从 TOML 文件加载。
    ///
    /// # Errors
    ///
    /// 文件不可读或解析失败。
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let raw = std::fs::read_to_string(path.as_ref())
            .map_err(|e| Error::new(ErrorKind::NotFound, "config file not found").with_source(e))?;
        toml::from_str(&raw)
            .map_err(|e| Error::new(ErrorKind::InvalidArgument, "invalid config").with_source(e))
    }
}

/// 地图定位后端，进程启动时固定。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Backend {
    #[default]
    PoseGraph,
    Eskf,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_origin_and_reset_preserve_known_alignment() {
        let cfg: LocalizationConfig =
            toml::from_str("[origin]\nposition = [-13.0, 0.0, 0.405]\nyaw = 1.5707963267948966")
                .unwrap();
        let alignment = cfg
            .origin
            .unwrap()
            .at_start(&firefly_pubsub::odom::OdomMessage {
                timestamp: 1.,
                is_initialized: true,
                ..Default::default()
            })
            .unwrap();
        let mut fusion = crate::FusionFilter::with_default();
        fusion.set_alignment(alignment).unwrap();
        fusion.reset();
        let p = fusion.corrected_pose(&nalgebra::Matrix4::identity());
        assert!((p[(0, 3)] + 13.0).abs() < 1e-12);
        assert!((p[(2, 3)] - 0.405).abs() < 1e-12);
        assert!((p[(1, 0)] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn misplaced_fusion_options_are_rejected() {
        assert!(
            toml::from_str::<LocalizationConfig>("[fusion.visual]\nr_floor_pos = 0.1").is_err()
        );
    }

    fn local_sample(yaw: f64) -> firefly_pubsub::odom::OdomMessage {
        let q = nalgebra::UnitQuaternion::from_euler_angles(0.03, -0.04, yaw);
        let q = q.quaternion();
        firefly_pubsub::odom::OdomMessage {
            timestamp: 2.,
            is_initialized: true,
            position_x: 0.03,
            position_y: -0.02,
            position_z: 0.01,
            quat_x: q.i,
            quat_y: q.j,
            quat_z: q.k,
            quat_w: q.w,
            ..Default::default()
        }
    }

    #[test]
    fn startup_alignment_is_invariant_to_local_yaw_gauge() {
        use nalgebra::{UnitQuaternion, Vector3};
        let prior = InitialAlignment {
            position: [-13., 0., 0.405],
            yaw: 0.7,
        };
        for yaw in [0., 0.7, std::f64::consts::PI, -std::f64::consts::FRAC_PI_2] {
            let sample = local_sample(yaw);
            let alignment = prior.at_start(&sample).unwrap();
            let mapped = crate::corrected_odom(&sample, &alignment).unwrap();
            let p = mapped.body_pose(firefly_base::FrameId::MAP).unwrap();
            assert!(
                (p.isometry().translation.vector - Vector3::from(prior.position)).norm() < 1e-12
            );
            let expected = UnitQuaternion::from_euler_angles(0.03, -0.04, 0.7);
            assert!((p.isometry().rotation.inverse() * expected).angle() < 1e-12);
            let epsilon = 1e-6;
            let numeric = (prior
                .at_start(&local_sample(yaw + epsilon))
                .unwrap()
                .matrix()
                - prior
                    .at_start(&local_sample(yaw - epsilon))
                    .unwrap()
                    .matrix())
                / (2. * epsilon);
            let r = alignment
                .isometry()
                .rotation
                .to_rotation_matrix()
                .into_inner();
            let j = -r * firefly_base::se3::skew(&Vector3::z());
            assert!((numeric.fixed_view::<3, 3>(0, 0) - j).norm() < 1e-8);
            assert!(
                (numeric.fixed_view::<3, 1>(0, 3) + j * Vector3::new(0.03, -0.02, 0.01)).norm()
                    < 1e-8
            );
        }
    }

    #[test]
    fn fixed_prior_cannot_align_airborne_moving_or_invalid_samples() {
        let prior = InitialAlignment::default();
        let sample = local_sample(0.);
        for bad in [
            firefly_pubsub::odom::OdomMessage {
                is_initialized: false,
                ..sample
            },
            firefly_pubsub::odom::OdomMessage {
                position_z: 1.,
                ..sample
            },
            firefly_pubsub::odom::OdomMessage {
                velocity_x: 0.2,
                ..sample
            },
            firefly_pubsub::odom::OdomMessage {
                velocity_z: f64::NAN,
                ..sample
            },
        ] {
            assert!(prior.at_start(&bad).is_err());
        }
        assert!(LocalizationConfig::default().origin.is_none());
    }

    #[test]
    fn defaults_roundtrip() {
        let cfg = LocalizationConfig::default();
        assert!((cfg.fusion.min_inlier_ratio - 0.3).abs() < 1e-12);
    }

    #[test]
    fn partial_toml_falls_back() {
        let cfg: LocalizationConfig = toml::from_str("fusion.min_inlier_ratio = 0.4").unwrap();
        assert!((cfg.fusion.min_inlier_ratio - 0.4).abs() < 1e-12);
    }

    /// 部分子表缺键回落紧画像（保守：缺配置不断言零值，不静默放行）。
    #[test]
    fn partial_subtable_falls_back_to_tight() {
        let cfg: LocalizationConfig =
            toml::from_str("[fusion.visual]\nmax_innovation_trans = 20.0").unwrap();
        assert!((cfg.fusion.visual.max_innovation_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_innovation_rot - 5.0_f64.to_radians()).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_correction_trans - 0.5).abs() < 1e-12);
    }

    #[test]
    fn shipped_config_parses() {
        let cfg = LocalizationConfig::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/localization.toml"
        ))
        .expect("configs/localization.toml must parse");
        assert!((cfg.fusion.min_inlier_ratio - 0.3).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_innovation_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.process_noise_pos_per_m - 0.04).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_correction_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.p_min_pos - 0.25).abs() < 1e-12);
    }
}
