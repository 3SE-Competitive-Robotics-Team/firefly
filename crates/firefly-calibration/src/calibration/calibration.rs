use super::calib_bias::{CalibAccelBias, CalibGyroBias};
use crate::{
    camera::generic_camera::GenericCamera, scalar::BasaltScalar, spline::rd_spline::RdSpline,
};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector2, Vector3};
use std::mem::size_of;

/// Camera-to-IMU transforms, camera models, photometric and inertial calibration.
#[derive(Clone, Debug)]
pub struct Calibration<S: BasaltScalar = f64> {
    pub t_i_c: Vec<Isometry3<S>>,
    pub intrinsics: Vec<GenericCamera<S>>,
    pub resolution: Vec<Vector2<i32>>,
    pub vignette: Vec<RdSpline<S, 1, 4>>,
    /// Add this offset to a raw camera timestamp to obtain the IMU timestamp.
    pub cam_time_offset_ns: i64,
    pub calib_accel_bias: CalibAccelBias<S>,
    pub calib_gyro_bias: CalibGyroBias<S>,
    pub imu_update_rate: S,
    pub gyro_noise_std: Vector3<S>,
    pub accel_noise_std: Vector3<S>,
    pub gyro_bias_std: Vector3<S>,
    pub accel_bias_std: Vector3<S>,
}
impl<S: BasaltScalar> Default for Calibration<S> {
    fn default() -> Self {
        Self {
            t_i_c: Vec::new(),
            intrinsics: Vec::new(),
            resolution: Vec::new(),
            vignette: Vec::new(),
            cam_time_offset_ns: 0,
            calib_accel_bias: CalibAccelBias::default(),
            calib_gyro_bias: CalibGyroBias::default(),
            imu_update_rate: S::from_f64_lossy(200.0),
            gyro_noise_std: Vector3::repeat(S::from_f64_lossy(0.000_282)),
            accel_noise_std: Vector3::repeat(S::from_f64_lossy(0.016)),
            accel_bias_std: Vector3::repeat(S::from_f64_lossy(0.001)),
            gyro_bias_std: Vector3::repeat(S::from_f64_lossy(0.0001)),
        }
    }
}
impl<S: BasaltScalar> Calibration<S> {
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> Calibration<T> {
        Calibration {
            t_i_c: self.t_i_c.iter().map(cast_pose).collect(),
            intrinsics: self.intrinsics.iter().map(GenericCamera::cast).collect(),
            resolution: self.resolution.clone(),
            vignette: self.vignette.iter().map(RdSpline::cast).collect(),
            cam_time_offset_ns: self.cam_time_offset_ns,
            calib_accel_bias: self.calib_accel_bias.cast(),
            calib_gyro_bias: self.calib_gyro_bias.cast(),
            imu_update_rate: T::from_f64_lossy(self.imu_update_rate.as_f64()),
            gyro_noise_std: self.gyro_noise_std.map(|v| T::from_f64_lossy(v.as_f64())),
            accel_noise_std: self.accel_noise_std.map(|v| T::from_f64_lossy(v.as_f64())),
            gyro_bias_std: self.gyro_bias_std.map(|v| T::from_f64_lossy(v.as_f64())),
            accel_bias_std: self.accel_bias_std.map(|v| T::from_f64_lossy(v.as_f64())),
        }
    }
    /// Continuous-time noise converted using sqrt(IMU update rate).
    /// The spelling retains the original upstream public method name.
    #[must_use]
    pub fn dicrete_time_gyro_noise_std(&self) -> Vector3<S> {
        self.gyro_noise_std * self.imu_update_rate.sqrt()
    }
    #[must_use]
    pub fn dicrete_time_accel_noise_std(&self) -> Vector3<S> {
        self.accel_noise_std * self.imu_update_rate.sqrt()
    }
}

/// Motion-capture origin/marker transforms and both timestamp offsets.
#[derive(Clone, Debug)]
pub struct MocapCalibration<S: BasaltScalar = f64> {
    pub t_moc_w: Isometry3<S>,
    pub t_i_mark: Isometry3<S>,
    pub mocap_time_offset_ns: i64,
    pub mocap_to_imu_offset_ns: i64,
}
impl<S: BasaltScalar> Default for MocapCalibration<S> {
    fn default() -> Self {
        Self {
            t_moc_w: Isometry3::identity(),
            t_i_mark: Isometry3::identity(),
            mocap_time_offset_ns: 0,
            mocap_to_imu_offset_ns: 0,
        }
    }
}
fn cast_pose<S: BasaltScalar, T: BasaltScalar>(pose: &Isometry3<S>) -> Isometry3<T> {
    Isometry3::from_parts(
        Translation3::from(
            pose.translation
                .vector
                .map(|v| T::from_f64_lossy(v.as_f64())),
        ),
        cast_rotation(&pose.rotation),
    )
}

fn cast_rotation<S: BasaltScalar, T: BasaltScalar>(
    rotation: &UnitQuaternion<S>,
) -> UnitQuaternion<T> {
    let quaternion = Quaternion::from_vector(
        rotation
            .quaternion()
            .coords
            .map(|value| T::from_f64_lossy(value.as_f64())),
    );
    if size_of::<S>() == size_of::<T>() {
        UnitQuaternion::new_unchecked(quaternion)
    } else {
        UnitQuaternion::new_normalize(quaternion)
    }
}
