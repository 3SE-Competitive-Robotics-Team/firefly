//! Original Basalt headless calibration, alignment and evaluation entry points.
use firefly_basalt::{
    calibration::{cam_calib::CamCalib, cam_imu_calib::CamImuCalib},
    io::dataset_io::DatasetIoFactory,
    kitti_eval, time_alignment,
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::calibration::MocapCalibration;
use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufReader, BufWriter, Write},
    path::PathBuf,
};

#[derive(Default, Deserialize)]
#[serde(tag = "task", rename_all = "kebab-case", deny_unknown_fields)]
enum Options {
    #[default]
    #[serde(skip)]
    Unspecified,
    Mapper {
        calibration: PathBuf,
        cache: PathBuf,
        vio_config: Option<PathBuf>,
        trajectory: Option<PathBuf>,
        result_path: Option<PathBuf>,
        #[serde(default = "enabled")]
        tum: bool,
    },
    OptFlow {
        calibration: PathBuf,
        dataset_path: PathBuf,
        dataset_type: String,
        vio_config: Option<PathBuf>,
    },
    GlobalBa {
        problem: PathBuf,
        output: PathBuf,
        precision: firefly_rootslam_mapping::global_optimization::Precision,
        krylov_precision: firefly_rootslam_mapping::global_optimization::KrylovPrecision,
        max_iterations: Option<usize>,
        linear_system_archive: Option<PathBuf>,
    },
    MapperSim {
        calibration: PathBuf,
        cache: PathBuf,
        output: Option<PathBuf>,
    },
    MapperSimNaive {
        calibration: PathBuf,
        cache: PathBuf,
        output: Option<PathBuf>,
    },
    VioSim {
        #[serde(flatten)]
        input: SimulationInput,
    },
    KittiEval {
        ground_truth: PathBuf,
        estimate: PathBuf,
        output: PathBuf,
    },
    TimeAlign {
        dataset_path: PathBuf,
        dataset_type: String,
        calibration: Option<PathBuf>,
        mocap_calibration: Option<PathBuf>,
        output: PathBuf,
        output_ground_truth: Option<PathBuf>,
        #[serde(default = "max_offset")]
        max_offset_s: f64,
    },
    Calibrate {
        #[serde(flatten)]
        input: CalibrationInput,
        camera_types: Vec<String>,
    },
    CalibrateImu {
        #[serde(flatten)]
        input: CalibrationInput,
        #[serde(default = "imu_noise")]
        imu_noise: [f64; 4],
    },
}
#[derive(Deserialize)]
struct SimulationInput {
    calibration: PathBuf,
    vio_config: Option<PathBuf>,
    output: PathBuf,
    archive: Option<PathBuf>,
    #[serde(default = "simulation_points")]
    num_points: usize,
    #[serde(default = "simulation_distance")]
    point_distance: f64,
    #[serde(default = "enabled")]
    use_imu: bool,
    #[serde(default = "enabled")]
    use_double: bool,
    marg_data: Option<PathBuf>,
    result_path: Option<PathBuf>,
}
#[derive(Deserialize)]
struct CalibrationInput {
    dataset_path: PathBuf,
    dataset_type: String,
    aprilgrid: PathBuf,
    result_path: PathBuf,
    cache_name: Option<String>,
    #[serde(default = "skip_images")]
    skip_images: usize,
}
fn max_offset() -> f64 {
    10.
}
fn simulation_points() -> usize {
    1000
}
fn simulation_distance() -> f64 {
    10.
}
fn enabled() -> bool {
    true
}
fn skip_images() -> usize {
    1
}
fn imu_noise() -> [f64; 4] {
    [0.016, 0.000_282, 0.001, 0.0001]
}
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("basalt.tools")
}
fn io_error(error: std::io::Error) -> Error {
    Error::from(error).with_operation("basalt.tools.io")
}
fn validate_alignment(result: &time_alignment::Alignment) -> Result<()> {
    if result.best_error_index < 0 || !result.best_error.is_finite() {
        return Err(invalid("No valid angular velocity associations"));
    }
    let index = usize::try_from(result.best_error_index)
        .map_err(|_| invalid("Invalid time alignment result index"))?;
    let offset = result
        .offsets_ms
        .get(index)
        .ok_or_else(|| invalid("Missing time alignment search sample"))?
        * 1e6;
    let representable = |value: f64| {
        value.is_finite() && value >= f64::from(i32::MIN) && value <= f64::from(i32::MAX)
    };
    if !representable(offset) || offset.round() != f64::from(result.best_offset_ns) {
        return Err(invalid(
            "Alignment offset exceeds the original 32-bit nanosecond result range",
        ));
    }
    if result.coefficients.x > 1e-9 {
        let refined = f64::from(result.best_offset_ns)
            - 100_000. * result.coefficients.y / (2. * result.coefficients.x);
        if !representable(refined) {
            return Err(invalid(
                "Refined alignment offset exceeds the original 32-bit nanosecond result range",
            ));
        }
    }
    Ok(())
}
#[derive(Deserialize)]
struct Archive<T> {
    value0: T,
}
fn load<T: serde::de::DeserializeOwned>(path: &PathBuf) -> Result<T> {
    serde_json::from_reader::<_, Archive<T>>(BufReader::new(File::open(path).map_err(io_error)?))
        .map(|value| value.value0)
        .map_err(|error| invalid(&error.to_string()))
}
impl Options {
    fn validate(&self) -> Result<()> {
        if let Self::CalibrateImu { imu_noise, .. } = self
            && imu_noise
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.)
        {
            return Err(invalid(
                "IMU noise and random-walk standard deviations must be finite and positive",
            ));
        }
        match self {
            Self::GlobalBa {
                max_iterations: Some(0),
                ..
            } => return Err(invalid("Iteration budget must be positive")),
            Self::VioSim { input } => {
                if input.num_points == 0
                    || input.num_points > i32::MAX as usize
                    || !input.point_distance.is_finite()
                    || input.point_distance <= 0.
                {
                    return Err(invalid(
                        "Simulation requires positive point count and finite positive distance",
                    ));
                }
            }
            Self::Unspecified => return Err(invalid("A task configuration is required")),
            Self::TimeAlign {
                calibration,
                mocap_calibration,
                max_offset_s,
                ..
            } => {
                if calibration.is_some() != mocap_calibration.is_some() {
                    return Err(invalid("Both calibration files must be supplied together"));
                }
                if !max_offset_s.is_finite()
                    || *max_offset_s <= 0.
                    || *max_offset_s >= i64::MAX as f64 * 1e-9
                {
                    return Err(invalid(
                        "Time search bound must be positive and representable in nanoseconds",
                    ));
                }
            }
            Self::Calibrate { input, .. } | Self::CalibrateImu { input, .. } => {
                if input.skip_images == 0 || i32::try_from(input.skip_images).is_err() {
                    return Err(invalid("skip_images must be positive and fit i32"));
                }
            }
            Self::GlobalBa { .. }
            | Self::Mapper { .. }
            | Self::OptFlow { .. }
            | Self::KittiEval { .. }
            | Self::MapperSim { .. }
            | Self::MapperSimNaive { .. } => {}
        }
        Ok(())
    }
    fn run(&self) -> Result<()> {
        match self {
            Self::Mapper {
                calibration,
                cache,
                vio_config,
                trajectory,
                result_path,
                tum,
            } => run_mapper(
                calibration,
                cache,
                vio_config.as_deref(),
                trajectory.as_deref(),
                result_path.as_deref(),
                *tum,
            ),
            Self::OptFlow {
                calibration,
                dataset_path,
                dataset_type,
                vio_config,
            } => run_opt_flow(
                calibration,
                dataset_path,
                dataset_type,
                vio_config.as_deref(),
            ),
            Self::GlobalBa {
                problem,
                output,
                precision,
                krylov_precision,
                max_iterations,
                linear_system_archive,
            } => run_global_replay(
                problem,
                output,
                *precision,
                *krylov_precision,
                *max_iterations,
                linear_system_archive.as_deref(),
            ),
            Self::MapperSim {
                calibration,
                cache,
                output,
            }
            | Self::MapperSimNaive {
                calibration,
                cache,
                output,
            } => run_mapper_simulation(
                calibration,
                cache,
                output.as_ref(),
                matches!(self, Self::MapperSimNaive { .. }),
            ),
            Self::VioSim { input } => run_simulation(input),
            Self::Unspecified => Err(invalid("A task configuration is required")),
            Self::KittiEval {
                ground_truth,
                estimate,
                output,
            } => run_kitti(ground_truth, estimate, output),
            Self::TimeAlign {
                dataset_path,
                dataset_type,
                calibration,
                mocap_calibration,
                output,
                output_ground_truth,
                max_offset_s,
            } => {
                let data = read_dataset(dataset_path, dataset_type)?;
                let data = data.lock().map_err(|_| invalid("Dataset lock poisoned"))?;
                let camera: Option<Calibration<f64>> =
                    calibration.as_ref().map(load).transpose()?;
                let mocap: Option<MocapCalibration<f64>> =
                    mocap_calibration.as_ref().map(load).transpose()?;
                let result = time_alignment::align(
                    &*data,
                    camera.as_ref().zip(mocap.as_ref()),
                    *max_offset_s,
                );
                validate_alignment(&result)?;
                write_alignment_outputs(
                    &result,
                    &*data,
                    output,
                    output_ground_truth.as_deref(),
                    mocap.as_ref(),
                )
            }
            Self::Calibrate {
                input,
                camera_types,
            } => input.calibrate(camera_types),
            Self::CalibrateImu { input, imu_noise } => input.calibrate_imu(*imu_noise),
        }
    }
}
fn read_dataset(
    path: &std::path::Path,
    kind: &str,
) -> Result<firefly_basalt::io::dataset_io::VioDatasetPtr> {
    let mut reader = DatasetIoFactory::get_dataset_io(kind, false)?;
    reader.read(path)?;
    reader
        .data()
        .ok_or_else(|| invalid("Dataset reader returned no data"))
}
fn write_alignment_outputs(
    result: &time_alignment::Alignment,
    data: &dyn firefly_basalt::io::dataset_io::VioDataset,
    output: &std::path::Path,
    output_ground_truth: Option<&std::path::Path>,
    mocap: Option<&MocapCalibration<f64>>,
) -> Result<()> {
    let mut writer = BufWriter::new(File::create(output).map_err(io_error)?);
    result
        .write_offsets(&mut writer, data.mocap_to_imu_offset_ns())
        .map_err(io_error)?;
    writer.flush().map_err(io_error)?;
    if let Some(path) = output_ground_truth {
        time_alignment::write_aligned_dataset(
            path,
            data,
            i64::from(result.best_offset_refined_ns),
            mocap,
        )
        .map_err(io_error)?;
    }
    Ok(())
}
fn run_kitti(
    ground_truth: &std::path::Path,
    estimate: &std::path::Path,
    output: &std::path::Path,
) -> Result<()> {
    let gt = kitti_eval::load_poses(ground_truth).map_err(io_error)?;
    let estimate = kitti_eval::load_poses(estimate).map_err(io_error)?;
    if gt.is_empty() || gt.len() != estimate.len() {
        return Err(invalid(
            "KITTI trajectories must be nonempty and have equal lengths",
        ));
    }
    let results = kitti_eval::eval_kitti(&kitti_eval::DEFAULT_LENGTHS, &gt, &estimate);
    let mut writer = BufWriter::new(File::create(output).map_err(io_error)?);
    kitti_eval::write_results(&mut writer, &results).map_err(io_error)?;
    writer.flush().map_err(io_error)
}
fn read_vio_config(
    path: Option<&std::path::Path>,
) -> Result<firefly_basalt::utils::vio_config::VioConfig> {
    let mut config = firefly_basalt::utils::vio_config::VioConfig::default();
    if let Some(path) = path {
        config.load(path)?;
    }
    Ok(config)
}
fn run_mapper(
    calibration: &std::path::Path,
    cache: &std::path::Path,
    config: Option<&std::path::Path>,
    trajectory: Option<&std::path::Path>,
    result_path: Option<&std::path::Path>,
    tum: bool,
) -> Result<()> {
    let mut mapper =
        firefly_basalt::mapper::Mapper::load_data(calibration, cache, &read_vio_config(config)?)?;
    let started = std::time::Instant::now();
    mapper.run()?;
    let duration = started.elapsed();
    if let Some(path) = trajectory {
        let mut writer = BufWriter::new(File::create(path).map_err(io_error)?);
        mapper.save_trajectory(&mut writer, tum).map_err(io_error)?;
        writer.flush().map_err(io_error)?;
    }
    if let Some(path) = result_path {
        let result = firefly_rootslam::dataset_output::OriginalResult {
            rms_ate: mapper.align(),
            num_frames: mapper.mapper.base.frame_poses.len(),
            exec_time_ns: duration.as_nanos(),
        };
        let mut writer = BufWriter::new(File::create(path).map_err(io_error)?);
        serde_json::to_writer_pretty(&mut writer, &result)
            .map_err(|source| invalid("Cannot serialize mapper result").with_source(source))?;
        writer.write_all(b"\n").map_err(io_error)?;
        writer.flush().map_err(io_error)?;
    }
    log::info!(
        "Offline mapper finished: {} poses, {} points",
        mapper.mapper.base.frame_poses.len(),
        mapper.points.len()
    );
    Ok(())
}
fn run_opt_flow(
    calibration: &PathBuf,
    dataset_path: &std::path::Path,
    dataset_type: &str,
    config: Option<&std::path::Path>,
) -> Result<()> {
    let calibration: Calibration<f64> = load(calibration)?;
    let dataset = read_dataset(dataset_path, dataset_type)?;
    let mut data = dataset
        .lock()
        .map_err(|_| invalid("Dataset lock poisoned"))?;
    let statistics = collect_opt_flow(&mut *data, &calibration, &read_vio_config(config)?)?;
    log::info!(
        "Optical flow finished: {} frames, {} tracks, mean track length {}",
        statistics.observations.len(),
        statistics.keypoint_stats.len(),
        statistics.mean_track_length()
    );
    Ok(())
}
fn collect_opt_flow(
    dataset: &mut dyn firefly_basalt::io::dataset_io::VioDataset,
    calibration: &Calibration<f64>,
    config: &firefly_basalt::utils::vio_config::VioConfig,
) -> Result<firefly_basalt::opt_flow::FlowStatistics> {
    use firefly_basalt::{
        opt_flow::{FlowStatistics, feed_images},
        optical_flow::{OpticalFlowFactory, OpticalFlowInput},
    };
    use std::sync::{Arc, mpsc::SyncSender};
    struct End(SyncSender<Option<Arc<OpticalFlowInput>>>);
    impl Drop for End {
        fn drop(&mut self) {
            let _ = self.0.send(None);
        }
    }
    let flow = OpticalFlowFactory::get_optical_flow(config, calibration)?;
    let sender = flow.input_queue().clone();
    let (producer, statistics) = std::thread::scope(|scope| {
        let producer = scope.spawn(move || -> Result<()> {
            let ending = End(sender);
            for image in feed_images(dataset) {
                ending
                    .0
                    .send(Some(image?))
                    .map_err(|_| invalid("Optical flow input disconnected"))?;
            }
            Ok(())
        });
        let mut statistics = FlowStatistics::default();
        while let Ok(Some(result)) = flow.output_queue().recv() {
            statistics.read_result((*result).clone());
        }
        (
            producer
                .join()
                .map_err(|_| invalid("Image producer panicked"))
                .and_then(|result| result),
            statistics,
        )
    });
    let processing = flow.shutdown();
    producer.and(processing)?;
    Ok(statistics)
}
fn run_global_replay(
    problem: &std::path::Path,
    output: &std::path::Path,
    precision: firefly_rootslam_mapping::global_optimization::Precision,
    krylov_precision: firefly_rootslam_mapping::global_optimization::KrylovPrecision,
    max_iterations: Option<usize>,
    linear_system_archive: Option<&std::path::Path>,
) -> Result<()> {
    let problem = firefly_rootslam_mapping::global_problem::GlobalProblem::load(problem)?;
    let (snapshot, calibration, mut options) = problem.restore()?;
    options.precision = precision;
    options.krylov_precision = krylov_precision;
    if let Some(budget) = max_iterations {
        options.max_iterations = budget;
    }
    options.problem_archive = None;
    options.linear_system_archive = linear_system_archive.map(std::path::Path::to_path_buf);
    let result = firefly_rootslam_mapping::global_optimization::optimize_global(
        &snapshot,
        &calibration,
        &options,
    )?;
    log::info!(
        "Global replay: cost {} -> {}, {} accepted / {} iterations, converged={}, {} linear iterations",
        result.initial_cost,
        result.final_cost,
        result.accepted_steps,
        result.iterations,
        result.converged,
        result.linear_iterations
    );
    firefly_rootslam_mapping::global_problem::ReplayResult::from_result(&result, options)
        .write_new(output)
}
fn run_mapper_simulation(
    calibration: &std::path::Path,
    cache: &std::path::Path,
    output: Option<&PathBuf>,
    naive: bool,
) -> Result<()> {
    let error = if naive {
        let mut simulation =
            firefly_basalt::mapper_sim_naive::NaiveMapperSimulation::load(calibration, cache)?;
        let result = simulation.run()?;
        output.map(|_| simulation.align(&result)).transpose()?
    } else {
        let mut simulation =
            firefly_basalt::mapper_sim::MapperSimulation::load(calibration, cache)?;
        simulation.run()?;
        output.map(|_| simulation.align())
    };
    if let (Some(path), Some(error)) = (output, error) {
        let mut writer = BufWriter::new(File::create(path).map_err(io_error)?);
        writeln!(writer, "{error}").map_err(io_error)?;
        writer.flush().map_err(io_error)?;
    }
    Ok(())
}
fn run_simulation(input: &SimulationInput) -> Result<()> {
    let simulation = firefly_basalt::vio_sim::Simulation::generate(
        load(&input.calibration)?,
        input.num_points,
        input.point_distance,
    );
    if let Some(path) = &input.archive {
        let mut writer = BufWriter::new(File::create(path).map_err(io_error)?);
        simulation.write_archive(&mut writer).map_err(io_error)?;
        writer.flush().map_err(io_error)?;
    }
    let mut configuration = firefly_basalt::utils::vio_config::VioConfig::default();
    if let Some(path) = &input.vio_config {
        configuration.load(path)?;
    }
    let states = if let Some(path) = &input.marg_data {
        firefly_rootslam::simulation::run_with_cache(
            &simulation,
            &configuration,
            input.use_imu,
            input.use_double,
            path,
        )?
    } else {
        firefly_rootslam::simulation::run(
            &simulation,
            &configuration,
            input.use_imu,
            input.use_double,
        )?
    };
    if let Some(path) = &input.result_path {
        let times: Vec<_> = states.iter().map(|state| state.state.pose.t_ns).collect();
        if times != simulation.frame_timestamps {
            return Err(invalid(
                "Simulation estimator did not produce every input frame",
            ));
        }
        let positions: Vec<_> = states
            .iter()
            .map(|state| state.state.pose.t_w_i.translation.vector)
            .collect();
        let mut truth: Vec<_> = simulation
            .frame_poses
            .iter()
            .map(|pose| pose.translation.vector)
            .collect();
        let error = firefly_basalt::vi_estimator::vio_estimator::align_svd(
            &times,
            &positions,
            &simulation.frame_timestamps,
            &mut truth,
        );
        let mut writer = BufWriter::new(File::create(path).map_err(io_error)?);
        writeln!(writer, "{error}").map_err(io_error)?;
        writer.flush().map_err(io_error)?;
    }
    let mut writer = BufWriter::new(File::create(&input.output).map_err(io_error)?);
    for state in states {
        let pose = &state.state.pose;
        let t = pose.t_w_i.translation.vector;
        let q = pose.t_w_i.rotation.quaternion();
        writeln!(
            writer,
            "{}.{:09} {:.17} {:.17} {:.17} {:.17} {:.17} {:.17} {:.17}",
            pose.t_ns.div_euclid(1_000_000_000),
            pose.t_ns.rem_euclid(1_000_000_000),
            t.x,
            t.y,
            t.z,
            q.i,
            q.j,
            q.k,
            q.w
        )
        .map_err(io_error)?;
    }
    writer.flush().map_err(io_error)
}
impl CalibrationInput {
    fn calibrate(&self, camera_types: &[String]) -> Result<()> {
        let mut controller = CamCalib::new(
            &self.dataset_path,
            &self.dataset_type,
            &self.aprilgrid,
            &self.result_path,
            self.cache_name.as_deref().unwrap_or("calib-cam"),
            i32::try_from(self.skip_images).map_err(|_| invalid("skip_images exceeds i32"))?,
            camera_types.to_vec(),
        )?;
        controller.load_dataset()?;
        controller.detect_corners()?;
        controller.init_cam_intrinsics()?;
        controller.init_cam_poses()?;
        controller.init_cam_extrinsics()?;
        controller.init_optimization()?;
        while !controller.optimize_with_param(true, None)? {}
        controller.compute_vign()?;
        controller.save_calib()
    }
    fn calibrate_imu(&self, imu_noise: [f64; 4]) -> Result<()> {
        let cache_name = self.cache_name.as_deref().unwrap_or("calib-cam-imu");
        let mut controller = CamImuCalib::load_dataset(
            &self.dataset_path,
            &self.dataset_type,
            &self.aprilgrid,
            &self.result_path,
            cache_name,
            self.skip_images,
            imu_noise,
        )?;
        controller.detect_corners()?;
        controller.init_cam_poses()?;
        controller.save_preprocessing(&self.result_path, cache_name)?;
        controller.init_cam_imu_transform()?;
        controller.init_optimization()?;
        let mut stats = BTreeMap::new();
        while !controller.optimize_with_param(true, &mut stats) {}
        controller.options.opt_cam_time_offset = true;
        controller.options.opt_imu_scale = true;
        while !controller.optimize_with_param(true, &mut stats) {}
        controller.init_mocap()?;
        controller.options.use_mocap = true;
        while !controller.optimize_with_param(true, &mut stats) {}
        controller.save_calib(&self.result_path)?;
        controller.save_mocap_calib(&self.result_path)
    }
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let first = args.next();
    if first.as_deref() == Some("--help") || first.as_deref() == Some("-h") {
        writeln!(std::io::stdout(),"firefly-basalt-tools --config <path.toml>\nTasks: kitti-eval, time-align, calibrate, calibrate-imu, vio-sim, mapper, mapper-sim, mapper-sim-naive, opt-flow, global-ba").map_err(io_error)?;
        return Ok(());
    }
    if first.as_deref() != Some("--config") {
        return Err(invalid("Usage: firefly-basalt-tools --config <path.toml>"));
    }
    let path = args
        .next()
        .ok_or_else(|| invalid("Missing configuration path"))?;
    if args.next().is_some() {
        return Err(invalid("Unexpected argument"));
    }
    let options = firefly_config::startup(path, firefly_observability::init, Options::validate)?;
    let node = firefly_pubsub::node::create_node()?;
    let logs = firefly_observability::init_ipc(&node, "basalt-tools");
    std::thread::scope(|scope| {
        let worker = scope.spawn(|| options.run());
        while !worker.is_finished() {
            firefly_observability::pump_log_ipc(&logs);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let result = worker.join().map_err(|_| {
            Error::new(ErrorKind::Internal, "Basalt utility worker panicked")
                .with_operation("basalt.tools.run")
        })?;
        if let Err(error) = &result {
            log::error!("{error}");
        }
        firefly_observability::pump_log_ipc(&logs);
        result
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FlowDataset {
        timestamps: Vec<i64>,
        image: firefly_basalt::io::dataset_io::ImageData,
        failure: u8,
    }
    impl firefly_basalt::io::dataset_io::VioDataset for FlowDataset {
        fn num_cams(&self) -> usize {
            1
        }
        fn image_timestamps(&mut self) -> &mut Vec<i64> {
            &mut self.timestamps
        }
        fn accel_data(&self) -> &[firefly_basalt::io::dataset_io::AccelData] {
            &[]
        }
        fn gyro_data(&self) -> &[firefly_basalt::io::dataset_io::GyroData] {
            &[]
        }
        fn gt_timestamps(&self) -> &[i64] {
            &[]
        }
        fn gt_pose_data(&self) -> &[nalgebra::Isometry3<f64>] {
            &[]
        }
        fn mocap_to_imu_offset_ns(&self) -> i64 {
            0
        }
        fn image_data(
            &mut self,
            time: i64,
        ) -> Result<Vec<firefly_basalt::io::dataset_io::ImageData>> {
            if time == 2 && self.failure == 1 {
                return Err(invalid("Injected image failure"));
            }
            assert!(!(time == 2 && self.failure == 2), "Injected image panic");
            Ok(vec![self.image.clone()])
        }
    }

    #[test]
    fn optical_flow_task_collects_real_tracks_and_joins_after_read_error_or_panic() {
        use firefly_calibration::camera::generic_camera::GenericCamera;
        use firefly_calibration::image::image::ManagedImage;
        let mut model = GenericCamera::from_name("pinhole");
        model.set_from_init(nalgebra::Vector4::new(200., 200., 128., 96.));
        let calibration = Calibration {
            intrinsics: vec![model],
            t_i_c: vec![nalgebra::Isometry3::identity()],
            ..Calibration::default()
        };
        let mut image = ManagedImage::<u16>::new(256, 192);
        for (index, value) in image.data_mut().iter_mut().enumerate() {
            *value = u16::try_from((index * 137 + index * index * 17) % 65536).unwrap();
        }
        let image = firefly_basalt::io::dataset_io::ImageData {
            img: Some(std::sync::Arc::new(image)),
            exposure: 0.,
        };
        let config = firefly_basalt::utils::vio_config::VioConfig {
            optical_flow_skip_frames: 1,
            ..Default::default()
        };
        for failure in 0..=2 {
            let mut data = FlowDataset {
                timestamps: vec![1, 2, 3],
                image: image.clone(),
                failure,
            };
            let result = collect_opt_flow(&mut data, &calibration, &config);
            if failure == 0 {
                let result = result.unwrap();
                assert_eq!(result.observations.len(), 3);
                assert!(!result.keypoint_stats.is_empty());
                assert!(result.mean_track_length() >= 1.);
                assert!(
                    result
                        .observations
                        .values()
                        .all(|frame| frame.input_images.is_none())
                );
            } else {
                assert!(result.is_err());
            }
        }
    }

    #[test]
    fn offline_task_configuration_routes_to_real_input_validation() {
        for (task, extra) in [
            ("mapper", serde_json::json!({"cache":"missing-cache"})),
            (
                "opt-flow",
                serde_json::json!({"dataset_path":"missing-dataset","dataset_type":"euroc"}),
            ),
        ] {
            let mut value = extra;
            value["task"] = task.into();
            value["calibration"] = "missing-calibration.json".into();
            let options: Options = serde_json::from_value(value).unwrap();
            options.validate().unwrap();
            assert!(options.run().is_err());
        }
    }

    #[test]
    fn unassociated_time_alignment_cannot_write_a_successful_zero_offset() {
        let result = time_alignment::search(
            time_alignment::RotationSamples::default(),
            time_alignment::RotationSamples::default(),
            0.001,
        );
        assert!(validate_alignment(&result).is_err());
    }

    #[test]
    fn large_measured_offset_cannot_wrap_into_a_successful_result() {
        let values = vec![
            nalgebra::Vector3::zeros(),
            nalgebra::Vector3::x(),
            nalgebra::Vector3::x() * 2.,
        ];
        let result = time_alignment::search(
            time_alignment::RotationSamples {
                timestamps: vec![3_000_000_000, 3_010_000_000, 3_020_000_000],
                values: values.clone(),
            },
            time_alignment::RotationSamples {
                timestamps: vec![0, 10_000_000, 20_000_000],
                values,
            },
            3.1,
        );
        assert!(result.best_error_index >= 0);
        assert!(validate_alignment(&result).is_err());
    }

    #[test]
    fn zero_imu_noise_is_rejected_before_optimizer_construction() {
        let options: Options = serde_json::from_value(serde_json::json!({
            "task":"calibrate-imu", "dataset_path":"dataset", "dataset_type":"bag",
            "aprilgrid":"grid.json", "result_path":"calibration",
            "imu_noise":[0.,0.000_282,0.001,0.0001]
        }))
        .unwrap();
        assert!(options.validate().is_err());
    }

    #[test]
    fn calibration_configuration_reaches_original_defaults() {
        let options: Options = serde_json::from_value(serde_json::json!({
            "task":"calibrate-imu", "dataset_path":"dataset", "dataset_type":"bag",
            "aprilgrid":"grid.json", "result_path":"calibration"
        }))
        .unwrap();
        options.validate().unwrap();
        let Options::CalibrateImu { input, imu_noise } = options else {
            panic!("wrong task")
        };
        assert_eq!(input.skip_images, 1);
        assert_eq!(imu_noise, [0.016, 0.000_282, 0.001, 0.0001]);
        assert!(input.cache_name.is_none());
    }

    #[test]
    fn time_alignment_requires_paired_calibrations_and_accepts_default_range() {
        let mut value = serde_json::json!({"task":"time-align", "dataset_path":"dataset", "dataset_type":"bag", "output":"offset.json"});
        serde_json::from_value::<Options>(value.clone())
            .unwrap()
            .validate()
            .unwrap();
        value["calibration"] = "calibration.json".into();
        assert!(
            serde_json::from_value::<Options>(value)
                .unwrap()
                .validate()
                .is_err()
        );
    }
}
