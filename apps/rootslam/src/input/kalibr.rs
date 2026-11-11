//! UZH's official Kalibr sensor files. Application configuration remains TOML.
use super::{SensorModel, error};
use firefly_calibration::{
    calibration::Calibration,
    camera::{generic_camera::GenericCamera, kannala_brandt_camera4::KannalaBrandtCamera4},
};
use firefly_error::Result;
use nalgebra::{Isometry3, Matrix3, SVector, Translation3, UnitQuaternion, Vector2, Vector3};
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    fs::File,
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
struct Camera {
    #[serde(rename = "camera_model")]
    model: String,
    distortion_model: String,
    intrinsics: [f64; 4],
    distortion_coeffs: [f64; 4],
    resolution: [i32; 2],
    #[serde(rename = "T_cam_imu")]
    transform: [[f64; 4]; 4],
    timeshift_cam_imu: f64,
}
#[derive(Clone, Copy, Deserialize)]
struct Imu {
    accelerometer_noise_density: f64,
    accelerometer_random_walk: f64,
    gyroscope_noise_density: f64,
    gyroscope_random_walk: f64,
    update_rate: f64,
}

fn read<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let file = File::open(path).map_err(|e| {
        error("Cannot open Kalibr file")
            .with_source(e)
            .with_context("path", path.display())
    })?;
    serde_yaml_ng::from_reader(file).map_err(|e| {
        error("Invalid Kalibr sensor file")
            .with_source(e)
            .with_context("path", path.display())
    })
}

pub(super) fn load(directory: &Path) -> Result<SensorModel> {
    let paths: Vec<PathBuf> = std::fs::read_dir(directory)
        .map_err(|e| error("Cannot read calibration directory").with_source(e))?
        .map(|entry| entry.map(|e| e.path()))
        .collect::<std::io::Result<Vec<_>>>()
        .map_err(|e| error("Cannot read calibration directory entry").with_source(e))?
        .into_iter()
        .filter(|p| {
            p.file_name()
                .is_some_and(|n| n.to_string_lossy().starts_with("camchain-imucam-"))
                && p.extension().is_some_and(|e| e == "yaml")
        })
        .collect();
    if paths.len() != 1 {
        return Err(error(
            "UZH calibration directory requires exactly one camchain-imucam-*.yaml",
        ));
    }
    model(read(&paths[0])?, read(&directory.join("imu.yaml"))?)
}

#[expect(
    clippy::cast_possible_truncation,
    reason = "Finite bounded seconds rounded to integer nanoseconds"
)]
fn model(mut cameras: BTreeMap<String, Camera>, imu: Imu) -> Result<SensorModel> {
    if cameras.len() != 2 {
        return Err(error("UZH Snapdragon adapter requires cam0 and cam1"));
    }
    let mut calibration = Calibration {
        imu_update_rate: imu.update_rate,
        accel_noise_std: Vector3::repeat(imu.accelerometer_noise_density),
        gyro_noise_std: Vector3::repeat(imu.gyroscope_noise_density),
        accel_bias_std: Vector3::repeat(imu.accelerometer_random_walk),
        gyro_bias_std: Vector3::repeat(imu.gyroscope_random_walk),
        ..Calibration::default()
    };
    let mut left_shift: Option<f64> = None;
    for name in ["cam0", "cam1"] {
        let camera = cameras
            .remove(name)
            .ok_or_else(|| error("Missing Kalibr camera"))?;
        if camera.model != "pinhole"
            || camera.distortion_model != "equidistant"
            || camera
                .intrinsics
                .iter()
                .chain(&camera.distortion_coeffs)
                .any(|v| !v.is_finite())
            || camera.intrinsics[0] <= 0.
            || camera.intrinsics[1] <= 0.
            || !camera.timeshift_cam_imu.is_finite()
            || camera.timeshift_cam_imu.abs() > 1.
        {
            return Err(error(
                "UZH requires finite pinhole/equidistant calibration and a bounded time offset",
            ));
        }
        if let Some(shift) = left_shift {
            if (camera.timeshift_cam_imu - shift).abs() > 0.001 {
                return Err(error(
                    "Camera time offsets differ by more than the 1 ms stereo tolerance",
                ));
            }
        } else {
            left_shift = Some(camera.timeshift_cam_imu);
            calibration.cam_time_offset_ns = (camera.timeshift_cam_imu * 1e9).round() as i64;
        }
        let m = camera.transform;
        let rotation = Matrix3::from_fn(|r, c| m[r][c]);
        if m.iter().flatten().any(|v| !v.is_finite())
            || m[3] != [0., 0., 0., 1.]
            || (rotation.transpose() * rotation - Matrix3::identity()).norm() > 1e-6
            || (rotation.determinant() - 1.).abs() > 1e-6
        {
            return Err(error("T_cam_imu must be a rigid transform"));
        }
        let camera_from_imu = Isometry3::from_parts(
            Translation3::new(m[0][3], m[1][3], m[2][3]),
            UnitQuaternion::from_matrix(&rotation),
        );
        calibration.t_i_c.push(camera_from_imu.inverse());
        calibration
            .resolution
            .push(Vector2::from(camera.resolution));
        calibration
            .intrinsics
            .push(GenericCamera::KannalaBrandt4(KannalaBrandtCamera4::new(
                SVector::from_iterator(
                    camera
                        .intrinsics
                        .into_iter()
                        .chain(camera.distortion_coeffs),
                ),
            )));
    }
    let model = SensorModel {
        calibration,
        gravity: Vector3::new(0., 0., -9.81),
    };
    model.validate()?;
    Ok(model)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn camera() -> Camera {
        Camera {
            model: "pinhole".into(),
            distortion_model: "equidistant".into(),
            intrinsics: [280., 279., 320., 240.],
            distortion_coeffs: [0.01, -0.02, 0.003, -0.004],
            resolution: [640, 480],
            timeshift_cam_imu: -0.016,
            transform: [
                [0., -1., 0., 1.],
                [1., 0., 0., 2.],
                [0., 0., 1., 3.],
                [0., 0., 0., 1.],
            ],
        }
    }
    fn imu() -> Imu {
        Imu {
            accelerometer_noise_density: 0.1,
            accelerometer_random_walk: 0.002,
            gyroscope_noise_density: 0.05,
            gyroscope_random_walk: 0.00004,
            update_rate: 500.,
        }
    }
    #[test]
    fn kalibr_inverts_extrinsics_and_preserves_equidistant_projection() {
        let m = model(
            BTreeMap::from([("cam0".into(), camera()), ("cam1".into(), camera())]),
            imu(),
        )
        .unwrap();
        assert!(
            (m.calibration.t_i_c[0].translation.vector - Vector3::new(-2., 1., -3.)).norm() < 1e-12
        );
        assert!((m.calibration.t_i_c[0].rotation * Vector3::x() + Vector3::y()).norm() < 1e-12);
        assert_eq!(m.calibration.cam_time_offset_ns, -16_000_000);
        let theta = 0.7_f64.atan();
        let d = theta
            * (1. + 0.01 * theta.powi(2) - 0.02 * theta.powi(4) + 0.003 * theta.powi(6)
                - 0.004 * theta.powi(8));
        let mut pixel = Vector2::zeros();
        assert!(m.calibration.intrinsics[0].project(&Vector3::new(0.7, 0., 1.), &mut pixel, None));
        assert!((pixel - Vector2::new(280. * d + 320., 240.)).norm() < 1e-10);
    }
    #[test]
    fn rejects_reflections_and_asynchronous_stereo() {
        let mut right = camera();
        right.timeshift_cam_imu = 0.;
        assert!(
            model(
                BTreeMap::from([("cam0".into(), camera()), ("cam1".into(), right)]),
                imu()
            )
            .is_err()
        );
        let mut left = camera();
        left.transform[2][2] = -1.;
        assert!(
            model(
                BTreeMap::from([("cam0".into(), left), ("cam1".into(), camera())]),
                imu()
            )
            .is_err()
        );
    }
}
