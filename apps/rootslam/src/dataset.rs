// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Dataset feeds and state collection corresponding to original `src/vio.cpp`.
use firefly_basalt::imu::imu_types::ImuData;
use firefly_basalt::imu::imu_types::PoseVelBiasState;
use firefly_basalt::serialization::eigen_io::EigenMatrix;
use firefly_basalt::{
    io::{dataset_io::VioDatasetPtr, marg_data_io::MargDataSaver},
    optical_flow::OpticalFlowFactory,
    utils::vio_config::VioConfig,
    vi_estimator::vio_estimator::VioEstimatorBase,
};
use firefly_calibration::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;
use std::{
    io::{BufWriter, Write},
    path::Path,
    sync::{Arc, mpsc},
    time::{Duration, Instant},
};

#[derive(Default)]
pub struct RunOptions<'a> {
    pub max_frames: usize,
    pub stop: Option<&'a std::sync::atomic::AtomicBool>,
    pub marg_data: Option<&'a Path>,
    pub debug_directory: Option<&'a Path>,
}
pub struct RunReport {
    pub states: Vec<Arc<PoseVelBiasState<f64>>>,
    pub duration: Duration,
}

fn error(message: &str) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("rootslam.dataset.run")
}

/// Sends the terminal marker on completion and unwind while the optical-flow
/// handle retains its own sender.
pub(crate) struct EndMarker<T>(pub Option<mpsc::SyncSender<Option<Arc<T>>>>);
impl<T> EndMarker<T> {
    pub(crate) fn finish(&mut self) -> Result<()> {
        self.0
            .take()
            .expect("producer finished once")
            .send(None)
            .map_err(|_| error("Input consumer disconnected"))
    }
}
impl<T> Drop for EndMarker<T> {
    fn drop(&mut self) {
        if let Some(sender) = self.0.take() {
            let _ = sender.send(None);
        }
    }
}

/// Runs independent image/IMU producers and consumes the estimator's state stream.
/// `max_frames == 0` consumes every image; raw IMU measurements are calibrated by
/// the estimator, matching the original dataset executable.
/// # Errors
/// Propagates dataset, processing, queue-disconnection and worker-panic errors.
pub fn run(
    dataset: VioDatasetPtr,
    calibration: &Calibration<f64>,
    config: &VioConfig,
    estimator: VioEstimatorBase,
    max_frames: usize,
) -> Result<Vec<Arc<PoseVelBiasState<f64>>>> {
    run_with_options(
        dataset,
        calibration,
        config,
        estimator,
        &RunOptions {
            max_frames,
            ..RunOptions::default()
        },
    )
    .map(|report| report.states)
}

/// Runs with original optional native marginalization archives and statistics.
/// Marginalization archives and statistics use new output directories.
/// # Errors
/// Propagates setup, processing, writer, worker-panic and finalization errors.
pub fn run_with_options(
    dataset: VioDatasetPtr,
    calibration: &Calibration<f64>,
    config: &VioConfig,
    mut estimator: VioEstimatorBase,
    options: &RunOptions<'_>,
) -> Result<RunReport> {
    let saver = if let Some(path) = options.marg_data {
        if estimator.outputs.marg.is_some() || estimator.outputs.marg_bounded.is_some() {
            return Err(error("Marginalization output already has a consumer"));
        }
        let saver = MargDataSaver::new(path)?;
        let data = dataset
            .lock()
            .map_err(|_| error("Dataset mutex poisoned"))?;
        let mut output =
            BufWriter::new(std::fs::File::create(path.join("gt.cereal")).map_err(archive_error)?);
        write_ground_truth(&mut output, data.gt_timestamps(), data.gt_pose_data())?;
        output.flush().map_err(archive_error)?;
        drop(data);
        estimator.outputs.marg_bounded = Some(saver.in_marg_queue.clone());
        Some(saver)
    } else {
        None
    };
    let result = run_inner(dataset, calibration, config, estimator, options);
    let completion = saver.map_or(Ok(()), MargDataSaver::finish);
    result.and_then(|report| completion.map(|()| report))
}

/// Original `vio.cpp`: native Cereal vector counts, int64 timestamps, then
/// fixed Eigen Vector3 translation coefficients. All dataset GT is saved even
/// when `max_frames` restricts the estimation input.
/// # Errors
/// Reports inconsistent GT arrays or output I/O failures.
pub fn write_ground_truth(
    output: &mut impl Write,
    times: &[i64],
    poses: &[nalgebra::Isometry3<f64>],
) -> Result<()> {
    if times.len() != poses.len() {
        return Err(error("Ground truth timestamp and pose counts differ"));
    }
    output
        .write_all(&(times.len() as u64).to_ne_bytes())
        .map_err(archive_error)?;
    for time in times {
        output
            .write_all(&time.to_ne_bytes())
            .map_err(archive_error)?;
    }
    output
        .write_all(&(poses.len() as u64).to_ne_bytes())
        .map_err(archive_error)?;
    for pose in poses {
        EigenMatrix(pose.translation.vector)
            .write_binary(output)
            .map_err(archive_error)?;
    }
    Ok(())
}
fn archive_error(source: std::io::Error) -> Error {
    Error::from(source).with_operation("basalt.dataset.archive")
}

fn run_inner(
    dataset: VioDatasetPtr,
    calibration: &Calibration<f64>,
    config: &VioConfig,
    mut estimator: VioEstimatorBase,
    options: &RunOptions<'_>,
) -> Result<RunReport> {
    let max_frames = options.max_frames;
    let inputs = DatasetInputs::read(&dataset, max_frames)?;
    for &time in &inputs.timestamps {
        crate::input::camera_time_in_imu_clock(time, calibration.cam_time_offset_ns)?;
    }
    let requested_frames = inputs.timestamps.len();
    let DatasetInputs {
        timestamps,
        gyro,
        accel,
    } = inputs;
    let (states_tx, states_rx) = mpsc::channel();
    estimator.outputs.state = Some(states_tx);
    let flow = OpticalFlowFactory::get_optical_flow_with_output(
        config,
        calibration,
        estimator.vision_data_queue().clone(),
    )?;
    estimator.initialize(Vector3::zeros(), Vector3::zeros())?;
    // Only the worker retains this producer, so a panic also closes the stream.
    estimator.outputs.state.take();
    estimator.outputs.marg_bounded.take();
    let images = flow.input_queue().clone();
    let imu = estimator.imu_data_queue().clone();
    let (image_result, imu_result, states, started) = std::thread::scope(|scope| {
        let image_thread = scope.spawn(move || -> Result<()> {
            let mut ending = EndMarker(Some(images.clone()));
            let result = (|| {
                for t_ns in timestamps {
                    if options
                        .stop
                        .is_some_and(|stop| stop.load(std::sync::atomic::Ordering::Acquire))
                    {
                        break;
                    }
                    let img_data = dataset
                        .lock()
                        .map_err(|_| error("Dataset mutex poisoned"))?
                        .image_data(t_ns)?;
                    images
                        .send(Some(Arc::new(crate::input::image_frame(
                            calibration,
                            crate::input::camera_time_in_imu_clock(
                                t_ns,
                                calibration.cam_time_offset_ns,
                            )?,
                            img_data,
                        )?)))
                        .map_err(|_| error("Optical flow input disconnected"))?;
                }
                Ok(())
            })();
            result.and(ending.finish())
        });
        let imu_thread = scope.spawn(move || -> Result<()> {
            for (gyro, accel) in gyro.into_iter().zip(accel) {
                if imu
                    .send(Some(Arc::new(ImuData {
                        t_ns: gyro.timestamp_ns,
                        gyro: gyro.data,
                        accel: accel.data,
                    })))
                    .is_err()
                {
                    // The estimator may finish before the remaining IMU tail.
                    return Ok(());
                }
            }
            let _ = imu.send(None);
            Ok(())
        });
        let started = Instant::now();
        let states = collect_states(&states_rx, requested_frames);
        (
            image_thread
                .join()
                .map_err(|_| error("Image producer panicked"))
                .and_then(|x| x),
            imu_thread
                .join()
                .map_err(|_| error("IMU producer panicked"))
                .and_then(|x| x),
            states,
            started,
        )
    });
    let flow_result = flow.shutdown();
    let estimator_result = estimator.maybe_join();
    let duration = started.elapsed();
    estimator_result?;
    flow_result?;
    image_result?;
    imu_result?;
    if let Some(directory) = options.debug_directory {
        save_debug_statistics(&estimator, directory)?;
    }
    Ok(RunReport { states, duration })
}

fn save_debug_statistics(estimator: &VioEstimatorBase, directory: &Path) -> Result<()> {
    if let Some(parent) = directory
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent).map_err(archive_error)?;
    }
    std::fs::create_dir(directory).map_err(archive_error)?;
    estimator.debug_finalize(directory)
}

fn collect_states(
    receiver: &mpsc::Receiver<Option<Arc<PoseVelBiasState<f64>>>>,
    requested_frames: usize,
) -> Vec<Arc<PoseVelBiasState<f64>>> {
    let mut states = Vec::new();
    while let Ok(Some(state)) = receiver.recv() {
        #[expect(
            clippy::cast_precision_loss,
            reason = "Timestamp converted to the existing observability seconds clock"
        )]
        firefly_observability::set_sim_time(state.state.pose.t_ns as f64 * 1e-9);
        states.push(state);
        if states.len().is_multiple_of(100) {
            log::info!(
                "Local estimation: {} / {requested_frames} states",
                states.len()
            );
        }
    }
    states
}

struct DatasetInputs {
    timestamps: Vec<i64>,
    gyro: Vec<firefly_basalt::io::dataset_io::GyroData>,
    accel: Vec<firefly_basalt::io::dataset_io::AccelData>,
}
impl DatasetInputs {
    fn read(dataset: &VioDatasetPtr, max_frames: usize) -> Result<Self> {
        let (mut timestamps, gyro, accel) = {
            let mut data = dataset
                .lock()
                .map_err(|_| error("Dataset mutex poisoned"))?;
            (
                data.image_timestamps().clone(),
                data.gyro_data().to_vec(),
                data.accel_data().to_vec(),
            )
        };
        if max_frames > 0 {
            timestamps.truncate(max_frames);
        }
        let requested_frames = timestamps.len();
        log::info!(
            "Dataset input: {requested_frames} camera frames, {} IMU samples",
            gyro.len()
        );
        if gyro.len() != accel.len() {
            return Err(error("Gyroscope and accelerometer counts differ"));
        }
        if timestamps.is_empty()
            || timestamps.windows(2).any(|t| t[0] >= t[1])
            || gyro
                .windows(2)
                .any(|g| g[0].timestamp_ns >= g[1].timestamp_ns)
            || gyro.iter().zip(&accel).any(|(g, a)| {
                g.timestamp_ns != a.timestamp_ns
                    || !g.data.iter().chain(a.data.iter()).all(|v| v.is_finite())
            })
        {
            return Err(error(
                "Dataset measurements must be finite, paired and strictly ordered",
            ));
        }
        Ok(Self {
            timestamps,
            gyro,
            accel,
        })
    }
}

#[cfg(test)]
mod input_tests {
    use super::*;
    use firefly_basalt::io::dataset_io::{AccelData, GyroData, ImageData, VioDataset};
    use nalgebra::Isometry3;
    use std::sync::Mutex;

    struct Measurements {
        times: Vec<i64>,
        gyro: Vec<GyroData>,
        accel: Vec<AccelData>,
    }
    impl VioDataset for Measurements {
        fn num_cams(&self) -> usize {
            2
        }
        fn image_timestamps(&mut self) -> &mut Vec<i64> {
            &mut self.times
        }
        fn gyro_data(&self) -> &[GyroData] {
            &self.gyro
        }
        fn accel_data(&self) -> &[AccelData] {
            &self.accel
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
        fn image_data(&mut self, _: i64) -> Result<Vec<ImageData>> {
            panic!("metadata validation must precede image loading")
        }
    }
    fn data(times: Vec<i64>, accel_time: i64) -> VioDatasetPtr {
        Arc::new(Mutex::new(Measurements {
            times,
            gyro: vec![GyroData {
                timestamp_ns: 10,
                data: Vector3::zeros(),
            }],
            accel: vec![AccelData {
                timestamp_ns: accel_time,
                data: Vector3::zeros(),
            }],
        }))
    }
    #[test]
    fn metadata_requires_matching_imu_times_and_ordered_images() {
        assert!(DatasetInputs::read(&data(vec![10, 20], 11), 0).is_err());
        assert!(DatasetInputs::read(&data(vec![20, 10], 10), 0).is_err());
        assert!(DatasetInputs::read(&data(vec![10, 10], 10), 0).is_err());
        assert_eq!(
            DatasetInputs::read(&data(vec![10, 20], 10), 1)
                .unwrap()
                .timestamps,
            vec![10]
        );
    }
}
