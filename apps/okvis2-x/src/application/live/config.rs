// SPDX-License-Identifier: BSD-3-Clause
//! Online sensor calibration and transport limits; all times are sensor-clock seconds.
use crate::{
    cameras::{
        camera_base::CameraBase,
        eucm_camera::EucmCamera,
        n_camera_system::{CameraType, DepthType, DistortionType, NCameraSystem},
    },
    config::parameters::{
        CameraParameters, DisplayOptions, EstimatorParameters, FrontendParameters, ImuParameters,
        OnlineCalibrationParameters, OutputParameters, ViParameters,
    },
    kinematics::transformation::Transformation,
    mapping::config_mapping::SubMapConfig,
};
use firefly_base::{FrameId, rig};
use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};
use nalgebra::{IsometryMatrix3, UnitQuaternion, Vector3};
use serde::Deserialize;
use std::{collections::BTreeSet, sync::Arc};

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Extrinsic {
    pub translation: [f64; 3],
    /// Hamilton xyzw, body ← sensor.
    pub rotation: [f64; 4],
}
impl Default for Extrinsic {
    fn default() -> Self {
        Self {
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}
impl Extrinsic {
    /// # Errors
    /// Rejects nonfinite translation or a non-unit Hamilton quaternion.
    pub fn pose(&self) -> Result<Transformation> {
        let rigid = firefly_base::RigidTransform::from_parts(
            FrameId::BODY,
            FrameId::LEFT_CAMERA,
            self.translation,
            self.rotation,
        )?;
        let pose = rigid.isometry();
        Ok(Transformation::new(
            pose.translation.vector,
            pose.rotation.into_inner(),
        ))
    }
}
#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Camera {
    pub intrinsics: [f64; 4],
    pub extrinsic: Extrinsic,
}
impl Default for Camera {
    fn default() -> Self {
        Self::rig(FrameId::LEFT_CAMERA)
    }
}
impl Camera {
    fn rig(frame: FrameId) -> Self {
        let rotation =
            UnitQuaternion::from_matrix(&(rig::cam_axes_in_body() * rig::CV_FROM_RENDER));
        let q = rotation.quaternion();
        let f = IMAGE_HEIGHT as f64 * 0.5 / (rig::FOV_Y_DEG.to_radians() * 0.5).tan();
        let center = rig::principal_point(IMAGE_WIDTH, IMAGE_HEIGHT);
        Self {
            intrinsics: [f, f, center[0], center[1]],
            extrinsic: Extrinsic {
                translation: rig::position_in_body(frame).into(),
                rotation: [q.i, q.j, q.k, q.w],
            },
        }
    }
}
#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Imu {
    pub extrinsic: Extrinsic,
    pub accelerometer_scale: [f64; 3],
    pub gyro_noise: f64,
    pub gyro_walk: f64,
    pub accel_noise: f64,
    pub accel_walk: f64,
    pub gravity: f64,
}
impl Default for Imu {
    fn default() -> Self {
        Self {
            extrinsic: Extrinsic::default(),
            accelerometer_scale: [1.0; 3],
            gyro_noise: 2e-4,
            gyro_walk: 1.9e-5,
            accel_noise: 2e-3,
            accel_walk: 3e-3,
            gravity: 9.81,
        }
    }
}
#[derive(Clone, Copy, Debug, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Navigation {
    #[default]
    Okvis,
    ExternalLocalization,
}
#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub left: Camera,
    pub right: Camera,
    pub depth: Camera,
    pub imu: Imu,
    pub image_delay: f64,
    pub stereo_tolerance: f64,
    pub imu_history_seconds: f64,
    pub loop_closures: bool,
    pub num_keypoints: i32,
    pub realtime_iterations: i32,
    pub realtime_time_limit: f64,
    pub mapping: bool,
    /// An explicitly started external localization service owns navigation output.
    pub navigation: Navigation,
    pub diagnostics: bool,
    pub map_resolution: f32,
    pub map_dimension: f32,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            left: Camera::rig(FrameId::LEFT_CAMERA),
            right: Camera::rig(FrameId::RIGHT_CAMERA),
            depth: Camera::rig(FrameId::DEPTH_CAMERA),
            imu: Imu::default(),
            image_delay: 0.0,
            stereo_tolerance: 0.001,
            imu_history_seconds: 5.0,
            loop_closures: true,
            num_keypoints: 2500,
            realtime_iterations: 5,
            realtime_time_limit: 0.03,
            mapping: true,
            navigation: Navigation::Okvis,
            diagnostics: true,
            map_resolution: 0.25,
            map_dimension: 16.0,
        }
    }
}
impl Config {
    /// # Errors
    /// Rejects nonfinite calibration, invalid clocks/noise or unrepresentable mapping geometry.
    pub fn validate(&self) -> Result<()> {
        let positive = [
            self.imu_history_seconds,
            self.realtime_time_limit,
            self.imu.gravity,
            self.imu.gyro_noise,
            self.imu.gyro_walk,
            self.imu.accel_noise,
            self.imu.accel_walk,
        ];
        if positive.iter().any(|x| !x.is_finite() || *x <= 0.0)
            || !self.image_delay.is_finite()
            || !self.stereo_tolerance.is_finite()
            || self.stereo_tolerance < 0.0
            || self.stereo_tolerance > 0.01
            || self.num_keypoints <= 0
            || self.realtime_iterations <= 0
            || !self.map_dimension.is_finite()
            || !self.map_resolution.is_finite()
            || self.map_dimension <= 0.0
            || self.map_resolution <= 0.0
            || self.map_resolution > self.map_dimension
            || !(self.map_dimension / self.map_resolution).is_finite()
            || self.map_dimension / self.map_resolution > (1u32 << 19) as f32
            || self
                .imu
                .accelerometer_scale
                .iter()
                .any(|x| !x.is_finite() || *x <= 0.0)
        {
            return Err(invalid(
                "invalid online calibration, history, noise or geometry",
            ));
        }
        self.imu.extrinsic.pose()?;
        for camera in [&self.left, &self.right, &self.depth] {
            camera.extrinsic.pose()?;
            if camera.intrinsics.iter().any(|x| !x.is_finite())
                || camera.intrinsics[0] <= 0.0
                || camera.intrinsics[1] <= 0.0
            {
                return Err(invalid("invalid camera intrinsics"));
            }
        }
        Ok(())
    }
    pub(crate) fn parameters(&self) -> Result<ViParameters> {
        let mut cameras = NCameraSystem::new();
        let body_from_imu = self.imu.extrinsic.pose()?;
        for (index, camera) in [&self.left, &self.right, &self.depth]
            .into_iter()
            .enumerate()
        {
            let t_sc = Arc::new(body_from_imu.inverse().compose(&camera.extrinsic.pose()?));
            let [fx, fy, cx, cy] = camera.intrinsics;
            // `NoDistortion` 在上游对应 EUCM 几何（FrameRelativeAdapter 按相机类取焦距），
            // alpha=beta=0 时 EUCM 退化为针孔，与 live 标定的内参模型一致。
            // awareness maps 不能省：缺了 BRISK 工厂会以 "camera awareness maps not
            // initialised" 失败。
            let mut eucm = EucmCamera::new(
                (IMAGE_WIDTH as i32, IMAGE_HEIGHT as i32),
                (fx, fy, cx, cy, 0.0, 0.0),
                index as u64,
            );
            eucm.initialise_camera_awareness_maps();
            cameras.add_camera(
                &t_sc,
                Arc::new(eucm),
                DistortionType::NoDistortion,
                true,
                CameraType {
                    is_colour: false,
                    is_used: index < 2,
                    is_used_mapping: index == 2,
                    depth_type: DepthType {
                        is_depth_camera: index == 2,
                        ..DepthType::default()
                    },
                },
            );
        }
        Ok(ViParameters {
            n_camera_system: cameras,
            camera: CameraParameters {
                timestamp_tolerance: self.stereo_tolerance,
                sync_cameras: BTreeSet::from([0, 1]),
                stereo_indices: vec![0, 1],
                image_delay: self.image_delay,
                online_calibration: OnlineCalibrationParameters {
                    do_extrinsics: false,
                    do_extrinsics_final_ba: false,
                    sigma_r: 0.01,
                    sigma_alpha: 0.01,
                    sigma_r_final_ba: 0.01,
                    sigma_alpha_final_ba: 0.01,
                },
            },
            imu: self.imu_parameters()?,

            frontend: FrontendParameters {
                detection_threshold: 30.0,
                absolute_threshold: 5.0,
                matching_threshold: 60.0,
                octaves: 0,
                max_num_keypoints: self.num_keypoints,
                keyframe_overlap: 0.55,
                use_cnn: false,
                parallelise_detection: false,
                num_matching_threads: 1,
            },
            estimator: EstimatorParameters {
                num_keyframes: 5,
                num_loop_closure_frames: 5,
                num_imu_frames: 3,
                do_loop_closures: self.loop_closures,
                do_final_ba: false,
                enforce_realtime: true,
                realtime_min_iterations: 1,
                realtime_max_iterations: self.realtime_iterations,
                realtime_time_limit: self.realtime_time_limit,
                realtime_num_threads: 1,
                full_graph_iterations: 10,
                full_graph_num_threads: 1,
                p_dbow: 0.1,
                drift_percentage_heuristic: 10.0,
            },
            output: OutputParameters {
                display: DisplayOptions::default(),
                enable_submapping: self.mapping,
            },
            gps: None,
            lidar: None,
            rgb: None,
        })
    }
    pub(crate) fn imu_parameters(&self) -> Result<ImuParameters> {
        Ok(ImuParameters {
            r#use: true,
            t_bs: self.imu.extrinsic.pose()?,
            a_max: 200.0,
            g_max: 30.0,
            sigma_g_c: self.imu.gyro_noise,
            sigma_bg: 0.01,
            sigma_a_c: self.imu.accel_noise,
            sigma_ba: 0.1,
            sigma_gw_c: self.imu.gyro_walk,
            sigma_aw_c: self.imu.accel_walk,
            g0: Vector3::zeros(),
            a0: Vector3::zeros(),
            g: self.imu.gravity,
            s_a: Vector3::from(self.imu.accelerometer_scale),
        })
    }
    pub(crate) fn map(&self) -> firefly_supereight2::map::MapConfig {
        firefly_supereight2::map::MapConfig {
            dim: Vector3::repeat(self.map_dimension),
            res: self.map_resolution,
            t_mw: IsometryMatrix3::translation(
                self.map_dimension / 2.0,
                self.map_dimension / 2.0,
                self.map_dimension / 2.0,
            ),
        }
    }
    pub(crate) fn submaps() -> SubMapConfig {
        SubMapConfig {
            results_directory: String::new(),
            ..SubMapConfig::default()
        }
    }
}
pub(super) fn invalid(message: impl std::fmt::Display) -> Error {
    Error::new(ErrorKind::InvalidArgument, message.to_string()).with_operation("okvis2_x::ipc")
}
