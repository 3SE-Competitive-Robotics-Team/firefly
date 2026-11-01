use firefly_basalt::{
    io::dataset_io::{AccelData, GyroData, ImageData, VioDataset},
    utils::vio_config::VioConfig,
    vi_estimator::vio_estimator::VioEstimatorFactory,
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::image::image::ManagedImage;
use nalgebra::{Isometry3, Vector3, Vector4};
use std::sync::{Arc, Mutex, mpsc};

struct StationaryDataset {
    times: Vec<i64>,
    gyro: Vec<GyroData>,
    accel: Vec<AccelData>,
    images: Vec<ImageData>,
    panic_images: bool,
}
impl VioDataset for StationaryDataset {
    fn num_cams(&self) -> usize {
        2
    }
    fn image_timestamps(&mut self) -> &mut Vec<i64> {
        &mut self.times
    }
    fn accel_data(&self) -> &[AccelData] {
        &self.accel
    }
    fn gyro_data(&self) -> &[GyroData] {
        &self.gyro
    }
    fn gt_timestamps(&self) -> &[i64] {
        &[]
    }
    fn gt_pose_data(&self) -> &[Isometry3<f64>] {
        &[]
    }
    fn mocap_to_imu_offset_ns(&self) -> i64 {
        0
    }
    fn image_data(&mut self, _time: i64) -> firefly_error::Result<Vec<ImageData>> {
        assert!(!self.panic_images, "injected image reader panic");
        Ok(self.images.clone())
    }
}

#[test]
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "Bounded positive synthetic grayscale intensity"
)]
fn full_image_imu_pipeline_outputs_states_and_shuts_down() {
    let mut calibration = Calibration::default();
    let mut images = Vec::new();
    for camera in 0..2 {
        let mut model = GenericCamera::from_name("pinhole");
        model.set_from_init(Vector4::new(200.0, 200.0, 128.0, 96.0));
        calibration.intrinsics.push(model);
        calibration
            .t_i_c
            .push(Isometry3::translation(f64::from(camera) * 0.1, 0.0, 0.0));
        let mut image = ManagedImage::new(256, 192);
        for y in 0_u32..192 {
            for x in 0_u32..256 {
                let px = f64::from(x) + f64::from(camera) * 5.0;
                let intensity = 30000.0
                    + 10000.0 * (0.13 * px).sin()
                    + 8000.0 * (0.17 * f64::from(y)).cos()
                    + 6000.0 * (0.11 * px + 0.07 * f64::from(y)).sin();
                image.data_mut()[y as usize * 256 + x as usize] = intensity as u16;
            }
        }
        images.push(ImageData {
            img: Some(Arc::new(image)),
            exposure: 0.0,
        });
    }
    let times: Vec<_> = (0_i64..8).map(|i| 1_000_000_000 + i * 50_000_000).collect();
    let gyro = (0_i64..100)
        .map(|i| GyroData {
            timestamp_ns: 995_000_000 + i * 5_000_000,
            data: Vector3::zeros(),
        })
        .collect();
    let accel = (0_i64..100)
        .map(|i| AccelData {
            timestamp_ns: 995_000_000 + i * 5_000_000,
            data: Vector3::new(0.0, 0.0, 9.81),
        })
        .collect();
    let dataset = Arc::new(Mutex::new(StationaryDataset {
        times: times.clone(),
        gyro,
        accel,
        images,
        panic_images: false,
    }));
    let config = VioConfig {
        optical_flow_skip_frames: 1,
        ..VioConfig::default()
    };
    let mut estimator = VioEstimatorFactory::get_vio_estimator(
        &config,
        &calibration,
        &Vector3::new(0.0, 0.0, -9.81),
        true,
        true,
    )
    .unwrap();
    let (maps, snapshots) = mpsc::channel();
    estimator.outputs.local_map = Some(maps);
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        sender
            .send(firefly_rootslam::dataset::run(
                dataset,
                &calibration,
                &config,
                estimator,
                0,
            ))
            .unwrap();
    });
    let states = receiver
        .recv_timeout(std::time::Duration::from_secs(20))
        .expect("pipeline must drain")
        .unwrap();
    assert_eq!(states.len(), times.len());
    assert!(
        snapshots
            .try_iter()
            .flatten()
            .any(|snapshot| !snapshot.landmarks.is_empty()),
        "Stereo images must produce optimized landmarks"
    );
    for (state, time) in states.iter().zip(times) {
        assert_eq!(state.state.pose.t_ns, time);
        assert!(state.state.pose.t_w_i.translation.vector.norm() < 1e-4);
        assert!(state.state.vel_w_i.norm() < 1e-4);
    }
}

#[test]
fn image_reader_panic_terminates_downstream_workers() {
    let mut calibration = Calibration::default();
    for camera in 0..2 {
        let mut model = GenericCamera::from_name("pinhole");
        model.set_from_init(Vector4::new(200.0, 200.0, 128.0, 96.0));
        calibration.intrinsics.push(model);
        calibration
            .t_i_c
            .push(Isometry3::translation(f64::from(camera) * 0.1, 0.0, 0.0));
    }
    let dataset = Arc::new(Mutex::new(StationaryDataset {
        times: vec![1_000_000_000],
        gyro: vec![GyroData {
            timestamp_ns: 1_000_000_000,
            data: Vector3::zeros(),
        }],
        accel: vec![AccelData {
            timestamp_ns: 1_000_000_000,
            data: Vector3::new(0.0, 0.0, 9.81),
        }],
        images: Vec::new(),
        panic_images: true,
    }));
    let config = VioConfig::default();
    let estimator = VioEstimatorFactory::get_vio_estimator(
        &config,
        &calibration,
        &Vector3::new(0.0, 0.0, -9.81),
        true,
        true,
    )
    .unwrap();
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        sender
            .send(firefly_rootslam::dataset::run(
                dataset,
                &calibration,
                &config,
                estimator,
                0,
            ))
            .unwrap();
    });
    assert!(
        receiver
            .recv_timeout(std::time::Duration::from_secs(5))
            .expect("panic must not leave estimator waiting for an end marker")
            .is_err()
    );
}
