//! Sensor-source adapters. All sources use calibrated measurement timestamps and feed
//! the same Basalt image and IMU types; ground truth is evaluation-only.
mod kalibr;
use firefly_basalt::io::dataset_io::{DatasetIoFactory, DatasetIoInterface, VioDatasetPtr};
use firefly_calibration::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use std::{
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

#[derive(Default, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Source {
    #[default]
    Ipc,
    Dataset {
        format: Format,
        path: PathBuf,
        calibration: PathBuf,
    },
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Format {
    Euroc,
    Uzh,
    Kitti,
    Lamaria,
}

pub struct SensorModel {
    pub calibration: Calibration<f64>,
    pub gravity: Vector3<f64>,
}

fn error(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam.input")
}

/// Kalibr convention: `t_imu = t_camera + timeshift_cam_imu`.
/// Dataset file lookup retains the original camera timestamp; estimation and
/// trajectory export use the IMU clock. Ground truth retains its supplied
/// timestamps and is evaluated separately.
/// # Errors
/// Rejects overflow or a negative normalized timestamp.
pub(crate) fn camera_time_in_imu_clock(time: i64, offset: i64) -> Result<i64> {
    time.checked_add(offset)
        .filter(|t| *t >= 0)
        .ok_or_else(|| error("Invalid calibrated camera timestamp"))
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let file = File::open(path).map_err(|e| {
        error("Cannot open sensor calibration")
            .with_context("path", path.display())
            .with_source(e)
    })?;
    serde_json::from_reader(BufReader::new(file)).map_err(|e| {
        error("Invalid sensor calibration")
            .with_context("path", path.display())
            .with_source(e)
    })
}

impl Source {
    /// # Errors
    /// Rejects missing, malformed or incompatible calibration.
    pub fn model(&self) -> Result<SensorModel> {
        let model = match self {
            Self::Ipc => SensorModel {
                calibration: simulator_calibration(),
                gravity: Vector3::new(0., 0., -9.81),
            },
            Self::Dataset {
                format: Format::Lamaria,
                calibration,
                ..
            } => read_json::<AriaCalibration>(calibration)?.model()?,
            Self::Dataset {
                format: Format::Uzh,
                calibration,
                ..
            } if calibration.is_dir() => kalibr::load(calibration)?,
            Self::Dataset { calibration, .. } => {
                #[derive(Deserialize)]
                struct Archive {
                    value0: Calibration<f64>,
                }
                SensorModel {
                    calibration: read_json::<Archive>(calibration)?.value0,
                    gravity: Vector3::new(0., 0., -9.81),
                }
            }
        };
        model.validate()?;
        Ok(model)
    }

    /// # Errors
    /// Rejects live input, invalid dataset metadata or absent images.
    pub fn dataset(&self) -> Result<VioDatasetPtr> {
        let Self::Dataset { format, path, .. } = self else {
            return Err(error("IPC input is not a dataset"));
        };
        let mut reader: Box<dyn DatasetIoInterface> = match format {
            Format::Lamaria => {
                let mut reader = firefly_basalt::io::dataset_io_euroc::EurocIo::new(false);
                reader.read_asl(&path.join("aria"))?;
                return reader
                    .data()
                    .ok_or_else(|| error("Dataset reader returned no data"));
            }
            Format::Euroc => DatasetIoFactory::get_dataset_io("euroc", false)?,
            Format::Uzh => DatasetIoFactory::get_dataset_io("uzh", false)?,
            Format::Kitti => DatasetIoFactory::get_dataset_io("kitti", false)?,
        };
        reader.read(path)?;
        reader
            .data()
            .ok_or_else(|| error("Dataset reader returned no data"))
    }
}

impl SensorModel {
    /// # Errors
    /// Rejects inconsistent sensor dimensions, transforms or noise densities.
    pub fn validate(&self) -> Result<()> {
        let c = &self.calibration;
        if c.intrinsics.is_empty()
            || c.t_i_c.len() != c.intrinsics.len()
            || c.resolution.len() != c.intrinsics.len()
            || c.resolution.iter().any(|r| r.x <= 0 || r.y <= 0)
            || !c.imu_update_rate.is_finite()
            || c.imu_update_rate <= 0.
            || !self.gravity.iter().all(|v| v.is_finite())
            || self.gravity.norm() <= 0.
            || c.t_i_c
                .iter()
                .any(|p| !p.to_homogeneous().iter().all(|v| v.is_finite()))
            || [
                c.gyro_noise_std,
                c.accel_noise_std,
                c.gyro_bias_std,
                c.accel_bias_std,
            ]
            .iter()
            .any(|v| v.iter().any(|x| !x.is_finite() || *x <= 0.))
        {
            return Err(error("Invalid stereo/IMU sensor model"));
        }
        Ok(())
    }
}

/// Preserves native image sizes and the source measurement clock at ingress.
/// # Errors
/// Rejects absent cameras or images inconsistent with their calibration.
pub fn image_frame(
    calibration: &Calibration<f64>,
    t_ns: i64,
    img_data: Vec<firefly_basalt::io::dataset_io::ImageData>,
) -> Result<firefly_basalt::optical_flow::OpticalFlowInput> {
    if t_ns < 0
        || img_data.len() != calibration.resolution.len()
        || img_data
            .iter()
            .zip(&calibration.resolution)
            .any(|(image, resolution)| {
                image.img.as_ref().is_none_or(|image| {
                    usize::try_from(resolution.x) != Ok(image.w)
                        || usize::try_from(resolution.y) != Ok(image.h)
                })
            })
    {
        return Err(error("Camera frame does not match sensor calibration")
            .with_context("timestamp_ns", t_ns));
    }
    Ok(firefly_basalt::optical_flow::OpticalFlowInput { t_ns, img_data })
}

#[derive(Deserialize)]
struct Transform {
    qvec: [f64; 4],
    tvec: [f64; 3],
}
impl Transform {
    fn pose(&self) -> Result<Isometry3<f64>> {
        let [x, y, z, w] = self.qvec;
        let quaternion = Quaternion::new(w, x, y, z);
        if !quaternion.norm().is_finite()
            || (quaternion.norm() - 1.).abs() > 1e-5
            || !self.tvec.iter().all(|x| x.is_finite())
        {
            return Err(error("Invalid T_body_sensor transform"));
        }
        Ok(Isometry3::from_parts(
            Translation3::from(self.tvec),
            UnitQuaternion::new_normalize(quaternion),
        ))
    }
}
#[derive(Deserialize)]
struct Resolution {
    width: i32,
    height: i32,
}
#[derive(Deserialize)]
struct Camera {
    model: String,
    params: [f64; 4],
    resolution: Resolution,
    #[serde(rename = "T_b_s")]
    transform: Transform,
}
#[derive(Deserialize)]
struct Imu {
    #[serde(rename = "T_b_s")]
    transform: Transform,
    gravity_magnitude: f64,
    #[serde(rename = "imu_rate")]
    rate: f64,
    acc_noise_density: f64,
    gyro_noise_density: f64,
    acc_bias_random_walk_sigma: f64,
    gyro_bias_random_walk_sigma: f64,
}
#[derive(Deserialize)]
struct AriaCalibration {
    cam0: Camera,
    cam1: Camera,
    imu0: Imu,
}
impl AriaCalibration {
    fn model(self) -> Result<SensorModel> {
        if !self.imu0.gravity_magnitude.is_finite() || self.imu0.gravity_magnitude <= 0. {
            return Err(error(
                "LaMAria gravity magnitude must be positive and finite",
            ));
        }
        let imu_from_body = self.imu0.transform.pose()?.inverse();
        let mut c = Calibration {
            imu_update_rate: self.imu0.rate,
            accel_noise_std: Vector3::repeat(self.imu0.acc_noise_density),
            gyro_noise_std: Vector3::repeat(self.imu0.gyro_noise_density),
            accel_bias_std: Vector3::repeat(self.imu0.acc_bias_random_walk_sigma),
            gyro_bias_std: Vector3::repeat(self.imu0.gyro_bias_random_walk_sigma),
            ..Calibration::default()
        };
        for camera in [self.cam0, self.cam1] {
            if camera.model != "PINHOLE"
                || !camera.params.iter().all(|v| v.is_finite())
                || camera.params[0] <= 0.
                || camera.params[1] <= 0.
            {
                return Err(error("LaMAria ASL requires its PINHOLE calibration"));
            }
            let mut model =
                firefly_calibration::camera::generic_camera::GenericCamera::from_name("pinhole");
            model.set_from_init(Vector4::from(camera.params));
            c.intrinsics.push(model);
            c.resolution.push(Vector2::new(
                camera.resolution.width,
                camera.resolution.height,
            ));
            c.t_i_c.push(imu_from_body * camera.transform.pose()?);
        }
        Ok(SensorModel {
            calibration: c,
            gravity: Vector3::new(0., 0., -self.imu0.gravity_magnitude),
        })
    }
}

pub(crate) fn simulator_calibration() -> Calibration<f64> {
    use firefly_base::{FrameId, rig};
    use firefly_calibration::camera::generic_camera::GenericCamera;
    use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector3, Vector4};
    let focal = IMAGE_HEIGHT as f64 * 0.5 / (rig::FOV_Y_DEG.to_radians() * 0.5).tan();
    let center = rig::principal_point(IMAGE_WIDTH, IMAGE_HEIGHT);
    let mut camera = GenericCamera::from_name("pinhole");
    camera.set_from_init(Vector4::new(focal, focal, center[0], center[1]));
    let pose = |frame| {
        let rotation =
            UnitQuaternion::from_matrix(&(rig::cam_axes_in_body() * rig::CV_FROM_RENDER));
        Isometry3::from_parts(Translation3::from(rig::position_in_body(frame)), rotation)
    };
    Calibration {
        t_i_c: vec![pose(FrameId::LEFT_CAMERA), pose(FrameId::RIGHT_CAMERA)],
        intrinsics: vec![camera.clone(), camera],
        resolution: vec![Vector2::new(IMAGE_WIDTH as i32, IMAGE_HEIGHT as i32); 2],
        imu_update_rate: 1_000.0,
        gyro_noise_std: Vector3::repeat(0.0002),
        accel_noise_std: Vector3::repeat(0.002),
        gyro_bias_std: Vector3::repeat(0.000_019),
        accel_bias_std: Vector3::repeat(0.003),
        ..Calibration::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_basalt::io::dataset_io::ImageData;
    use firefly_calibration::image::image::ManagedImage;
    use std::sync::Arc;

    #[test]
    fn kalibr_clock_direction_and_overflow() {
        assert_eq!(
            camera_time_in_imu_clock(1_000_000_000, -16_000_000).unwrap(),
            984_000_000
        );
        assert_eq!(
            camera_time_in_imu_clock(1_000_000_000, 0).unwrap(),
            1_000_000_000
        );
        assert!(camera_time_in_imu_clock(i64::MAX, 1).is_err());
        assert!(camera_time_in_imu_clock(0, -1).is_err());
    }

    fn aria() -> AriaCalibration {
        let camera = |width, height, translation| Camera {
            model: "PINHOLE".into(),
            params: [240., 240., 380., 285.],
            resolution: Resolution { width, height },
            transform: Transform {
                qvec: [0., 0., 0., 1.],
                tvec: translation,
            },
        };
        AriaCalibration {
            cam0: camera(758, 572, [2., 0., 0.]),
            cam1: camera(757, 569, [2., 0.1, 0.]),
            imu0: Imu {
                transform: Transform {
                    qvec: [
                        0.,
                        0.,
                        std::f64::consts::FRAC_1_SQRT_2,
                        std::f64::consts::FRAC_1_SQRT_2,
                    ],
                    tvec: [1., 0., 0.],
                },
                gravity_magnitude: 9.806,
                rate: 1000.,
                acc_noise_density: 0.001,
                gyro_noise_density: 0.0002,
                acc_bias_random_walk_sigma: 0.003,
                gyro_bias_random_walk_sigma: 0.0001,
            },
        }
    }

    #[test]
    fn aria_uses_xyzw_and_sensor_to_body_transforms() {
        let model = aria().model().unwrap();
        model.validate().unwrap();
        let first = &model.calibration.t_i_c[0];
        assert!((first.translation.vector - Vector3::new(0., -1., 0.)).norm() < 1e-12);
        assert!((first.rotation * Vector3::x() + Vector3::y()).norm() < 1e-12);
        assert!((model.gravity.z + 9.806).abs() < 1e-12);
        assert_eq!(
            model.calibration.resolution,
            [Vector2::new(758, 572), Vector2::new(757, 569)]
        );
    }

    #[test]
    fn ingress_keeps_native_dimensions_and_rejects_missing_or_resized_images() {
        let model = aria().model().unwrap();
        let images = || {
            vec![
                ImageData {
                    img: Some(Arc::new(ManagedImage::new(758, 572))),
                    exposure: 0.,
                },
                ImageData {
                    img: Some(Arc::new(ManagedImage::new(757, 569))),
                    exposure: 0.,
                },
            ]
        };
        let frame = image_frame(&model.calibration, 1_389_350_666_375, images()).unwrap();
        assert_eq!(frame.t_ns, 1_389_350_666_375);
        assert_eq!(frame.img_data[1].img.as_ref().unwrap().w, 757);
        let mut missing = images();
        missing[0].img = None;
        assert!(image_frame(&model.calibration, frame.t_ns, missing).is_err());
        let mut resized = images();
        resized[0].img = Some(Arc::new(ManagedImage::new(320, 240)));
        assert!(image_frame(&model.calibration, frame.t_ns, resized).is_err());
    }

    #[test]
    fn raw_fisheye_calibration_cannot_be_used_for_asl_images() {
        let mut calibration = aria();
        calibration.cam0.model = "FISHEYE624".into();
        assert!(calibration.model().is_err());
    }

    #[test]
    fn detection_validation_uses_the_source_dimensions() {
        let options = crate::config::Options {
            optical_flow_detection_grid_size: Some(300),
            ..Default::default()
        };
        assert!(options.validate_model(&aria().model().unwrap()).is_ok());
        assert!(
            options
                .validate_model(&Source::Ipc.model().unwrap())
                .is_err()
        );
    }
}
