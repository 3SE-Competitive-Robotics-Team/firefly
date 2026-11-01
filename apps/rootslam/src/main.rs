use firefly_basalt::{
    io::dataset_io::DatasetIoFactory, utils::vio_config::VioConfig,
    vi_estimator::vio_estimator::VioEstimatorFactory,
};
use firefly_basalt_headers::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam::dataset_output::{
    OriginalResult, TrajectoryFormat, align_ground_truth, write_trajectory,
};
use firefly_rootslam_mapping::{
    global_optimization::OptimizationOptions, loop_closure::LoopOptions, map::MapPolicy,
};
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufReader, BufWriter, Write},
    path::PathBuf,
};

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "Independent original estimator and mapping configuration switches"
)]
struct Options {
    dataset_path: PathBuf,
    dataset_type: String,
    camera_calibration: PathBuf,
    vio_config: Option<PathBuf>,
    trajectory: Option<PathBuf>,
    trajectory_format: TrajectoryFormat,
    groundtruth: Option<PathBuf>,
    marg_data: Option<PathBuf>,
    debug_directory: Option<PathBuf>,
    result_path: Option<PathBuf>,
    metrics_path: Option<PathBuf>,
    global_result_path: Option<PathBuf>,
    /// Basalt's alignment association limit; not specified by `RootSLAM`.
    max_ground_truth_gap_ns: i64,
    use_imu: bool,
    use_double: bool,
    mapping: bool,
    global_optimization: bool,
    optimization: OptimizationOptions,
    #[serde(rename = "loop_options")]
    loops: LoopOptions,
    map_policy: Option<MapPolicy>,
    max_frames: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            dataset_path: PathBuf::new(),
            dataset_type: "euroc".into(),
            camera_calibration: PathBuf::new(),
            vio_config: None,
            trajectory: None,
            trajectory_format: TrajectoryFormat::default(),
            groundtruth: None,
            marg_data: None,
            debug_directory: None,
            result_path: None,
            metrics_path: None,
            global_result_path: None,
            max_ground_truth_gap_ns: 110_000_000,
            use_imu: true,
            use_double: false,
            mapping: true,
            global_optimization: true,
            optimization: OptimizationOptions::default(),
            loops: LoopOptions::default(),
            map_policy: None,
            max_frames: 0,
        }
    }
}
impl Options {
    fn validate(&self) -> Result<()> {
        if self.global_result_path.is_some() && !(self.mapping && self.global_optimization) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "global_result_path requires mapping and global_optimization",
            )
            .with_operation("rootslam.config.validate"));
        }
        if self.dataset_path.as_os_str().is_empty()
            || self.camera_calibration.as_os_str().is_empty()
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "dataset_path and camera_calibration are required",
            )
            .with_operation("rootslam.config.validate"));
        }
        if !["euroc", "bag", "uzh", "kitti"].contains(&self.dataset_type.as_str()) {
            return Err(
                Error::new(ErrorKind::InvalidArgument, "Unsupported dataset_type")
                    .with_operation("rootslam.config.validate"),
            );
        }
        self.optimization.validate()?;
        self.loops.validate()?;
        if let Some(policy) = &self.map_policy {
            policy.validate()?;
        }
        if self.max_ground_truth_gap_ns <= 0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "max_ground_truth_gap_ns must be positive",
            ));
        }
        Ok(())
    }
}
#[derive(Deserialize)]
struct CalibrationArchive {
    value0: Calibration<f64>,
}

fn main() -> Result<()> {
    let mut config = firefly_config::paths::config("rootslam.toml");
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == "--help" || argument == "-h" {
            writeln!(std::io::stdout(),"firefly-rootslam --config <path.toml>\nRuns the translated original Basalt dataset pipeline.").map_err(io_error)?;
            return Ok(());
        }
        if argument != "--config" {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "Usage: firefly-rootslam --config <path.toml>",
            )
            .with_operation("rootslam.cli"));
        }
        config = arguments
            .next()
            .ok_or_else(|| {
                Error::new(ErrorKind::InvalidArgument, "Missing --config value")
                    .with_operation("rootslam.cli")
            })?
            .into();
    }
    let options = firefly_config::startup(config, firefly_observability::init, Options::validate)?;
    let node = firefly_pubsub::node::create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "rootslam");
    std::thread::scope(|scope| {
        let worker = scope.spawn(|| run(&options));
        while !worker.is_finished() {
            firefly_observability::pump_log_ipc(&log_ipc);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let result = worker
            .join()
            .map_err(|_| {
                Error::new(ErrorKind::Internal, "RootSLAM application worker panicked")
                    .with_operation("rootslam.run")
            })
            .and_then(|result| result);
        if let Err(error) = &result {
            log::error!("{error}");
        }
        firefly_observability::pump_log_ipc(&log_ipc);
        result
    })
}
fn io_error(error: std::io::Error) -> Error {
    Error::new(ErrorKind::Internal, "Dataset application I/O failed")
        .with_operation("rootslam.dataset.io")
        .with_source(error)
}
fn run(options: &Options) -> Result<()> {
    let started = std::time::Instant::now();
    let calibration = serde_json::from_reader::<_, CalibrationArchive>(BufReader::new(
        File::open(&options.camera_calibration).map_err(io_error)?,
    ))
    .map_err(|source| {
        Error::new(
            ErrorKind::InvalidArgument,
            "Invalid Basalt calibration archive",
        )
        .with_operation("rootslam.calibration.load")
        .with_source(source)
    })?
    .value0;
    let mut config = VioConfig::default();
    if let Some(path) = &options.vio_config {
        config.load(path)?;
    }
    let mut reader = DatasetIoFactory::get_dataset_io(&options.dataset_type, false)?;
    reader.read(&options.dataset_path)?;
    let dataset = reader.data().ok_or_else(|| {
        Error::new(ErrorKind::Internal, "Dataset reader returned no data")
            .with_operation("rootslam.dataset.load")
    })?;
    // Readers already provide their reference timestamp convention. The
    // original vio.cpp uses these GT arrays directly, without a second offset.
    let evaluation_data = if options.metrics_path.is_some()
        || options.result_path.is_some()
        || options.groundtruth.is_some()
        || options.debug_directory.is_some()
    {
        let mut dataset = dataset
            .lock()
            .map_err(|_| Error::new(ErrorKind::Internal, "Dataset mutex poisoned"))?;
        let times = dataset.image_timestamps().clone();
        Some((
            times,
            dataset.gt_timestamps().to_vec(),
            dataset.gt_pose_data().to_vec(),
        ))
    } else {
        None
    };
    let mut estimator = VioEstimatorFactory::get_vio_estimator(
        &config,
        &calibration,
        &nalgebra::Vector3::new(0.0, 0.0, -9.81),
        options.use_imu,
        options.use_double,
    )?;
    let mapping = if options.mapping {
        let system_options = firefly_rootslam::system::SystemOptions {
            global_optimization: options.global_optimization,
            optimization: options.optimization.clone(),
            loops: options.loops.clone(),
            map: options
                .map_policy
                .clone()
                .unwrap_or_else(|| firefly_rootslam::system::SystemOptions::default().map),
        };
        let worker =
            firefly_rootslam::system::MappingWorker::spawn(calibration.clone(), system_options)?;
        estimator.outputs.local_map = Some(worker.input());
        Some(worker)
    } else {
        None
    };
    let report = firefly_rootslam::dataset::run_with_options(
        dataset,
        &calibration,
        &config,
        estimator,
        &firefly_rootslam::dataset::RunOptions {
            max_frames: options.max_frames,
            marg_data: options.marg_data.as_deref(),
            debug_directory: options.debug_directory.as_deref(),
        },
    )?;
    let states = report.states;
    save_original_results(options, &states, report.duration, evaluation_data.as_ref())?;
    let mapped = mapping
        .map(firefly_rootslam::system::MappingWorker::finish)
        .transpose()?;
    if let Some(result) = &mapped {
        report_mapping(result);
    }
    save_global_result(options, mapped.as_ref())?;
    save_results(options, &states, mapped.as_ref(), evaluation_data)?;
    let mut memory = firefly_basalt::utils::system_utils::MemoryInfo::default();
    if firefly_basalt::utils::system_utils::get_memory_info(&mut memory) {
        log::info!(
            "Process memory: {} resident bytes, {} peak resident bytes (including mapped input pages)",
            memory.resident_memory,
            memory.resident_memory_peak
        );
    }
    log::info!(
        "Dataset pipeline produced {} states in {:.3} s including input loading and final optimization",
        states.len(),
        started.elapsed().as_secs_f64()
    );
    Ok(())
}

fn save_global_result(
    options: &Options,
    mapped: Option<&firefly_rootslam::system::SystemResult>,
) -> Result<()> {
    let Some(path) = &options.global_result_path else {
        return Ok(());
    };
    let global = mapped
        .and_then(|result| result.global.as_ref())
        .ok_or_else(|| {
            Error::new(
                ErrorKind::InvalidArgument,
                "No global optimization result to export",
            )
            .with_operation("rootslam.global_result.save")
        })?;
    firefly_rootslam_mapping::global_problem::ReplayResult::from_result(
        global,
        options.optimization.clone(),
    )
    .write_new(path)
}

fn report_mapping(result: &firefly_rootslam::system::SystemResult) {
    log::info!(
        "RootSLAM map: {} keyframes, {} landmarks, {} verified loops, {} pose-graph optimizations",
        result.map.snapshot.keyframes.len(),
        result.map.snapshot.landmarks.len(),
        result.loops_verified,
        result.pose_graph_optimizations
    );
    if let Some(global) = &result.global {
        log::info!(
            "Global BA: cost {} -> {}, {} iterations, {} accepted steps, converged={}, {} linear iterations, {} bytes of linear buffers",
            global.initial_cost,
            global.final_cost,
            global.iterations,
            global.accepted_steps,
            global.converged,
            global.linear_iterations,
            global.peak_linear_storage_bytes
        );
    }
}

type EvaluationData = (Vec<i64>, Vec<i64>, Vec<nalgebra::Isometry3<f64>>);
fn save_results(
    options: &Options,
    states: &[std::sync::Arc<firefly_basalt_headers::imu::imu_types::PoseVelBiasState<f64>>],
    mapped: Option<&firefly_rootslam::system::SystemResult>,
    evaluation_data: Option<EvaluationData>,
) -> Result<()> {
    if let Some(path) = &options.trajectory {
        let mut output = BufWriter::new(File::create(path).map_err(io_error)?);
        let estimates: Vec<_> = states
            .iter()
            .map(|state| {
                let pose = &state.state.pose;
                let corrected = mapped
                    .as_ref()
                    .and_then(|result| result.trajectory.get(&pose.t_ns))
                    .unwrap_or(&pose.t_w_i);
                (pose.t_ns, *corrected)
            })
            .collect();
        write_trajectory(&mut output, &estimates, options.trajectory_format).map_err(io_error)?;
        output.flush().map_err(io_error)?;
    }
    if let (Some(path), Some((mut times, gt_times, ground_truth))) =
        (&options.metrics_path, evaluation_data)
    {
        if options.max_frames > 0 {
            times.truncate(options.max_frames);
        }
        let estimates = states
            .iter()
            .map(|state| {
                let pose = &state.state.pose;
                (
                    pose.t_ns,
                    mapped
                        .as_ref()
                        .and_then(|result| result.trajectory.get(&pose.t_ns))
                        .copied()
                        .unwrap_or(pose.t_w_i),
                )
            })
            .collect();
        let metrics = firefly_rootslam::evaluation::evaluate(
            &times,
            &estimates,
            &gt_times,
            &ground_truth,
            options.max_ground_truth_gap_ns,
        )?;
        let mut output = BufWriter::new(File::create(path).map_err(io_error)?);
        serde_json::to_writer_pretty(&mut output, &metrics).map_err(|source| {
            Error::new(ErrorKind::Internal, "Cannot serialize trajectory metrics")
                .with_source(source)
        })?;
        output.write_all(b"\n").map_err(io_error)?;
        output.flush().map_err(io_error)?;
    }
    Ok(())
}

fn save_original_results(
    options: &Options,
    states: &[std::sync::Arc<firefly_basalt_headers::imu::imu_types::PoseVelBiasState<f64>>],
    duration: std::time::Duration,
    data: Option<&EvaluationData>,
) -> Result<()> {
    if options.result_path.is_none()
        && options.groundtruth.is_none()
        && options.debug_directory.is_none()
    {
        return Ok(());
    }
    let (times, gt_times, gt_poses) = data.ok_or_else(|| {
        Error::new(
            ErrorKind::Internal,
            "Missing ground truth for original statistics",
        )
    })?;
    let estimates: Vec<_> = states
        .iter()
        .map(|state| (state.state.pose.t_ns, state.state.pose.t_w_i))
        .collect();
    let mut positions: Vec<_> = gt_poses
        .iter()
        .map(|pose| pose.translation.vector)
        .collect();
    let rms_ate = align_ground_truth(&estimates, gt_times, &mut positions)?;
    if let Some(directory) = &options.debug_directory {
        let mut memory = firefly_basalt::utils::system_utils::MemoryInfo::default();
        let peak = firefly_basalt::utils::system_utils::get_memory_info(&mut memory)
            .then_some(memory.resident_memory_peak);
        firefly_rootslam::dataset_output::original_statistics(
            duration,
            rms_ate,
            states.len(),
            times.len(),
            peak,
        )
        .save_json(directory.join("stats_vio.json"))?;
    }
    if let Some(path) = &options.result_path {
        let result = OriginalResult {
            rms_ate,
            num_frames: times.len(),
            exec_time_ns: duration.as_nanos(),
        };
        let mut output = BufWriter::new(File::create(path).map_err(io_error)?);
        serde_json::to_writer_pretty(&mut output, &result).map_err(|source| {
            Error::new(ErrorKind::Internal, "Cannot serialize original result").with_source(source)
        })?;
        output.flush().map_err(io_error)?;
    }
    if let Some(path) = &options.groundtruth {
        let poses: Vec<_> = gt_times
            .iter()
            .zip(positions)
            .map(|(time, position)| {
                (
                    *time,
                    nalgebra::Isometry3::translation(position.x, position.y, position.z),
                )
            })
            .collect();
        let mut output = BufWriter::new(File::create(path).map_err(io_error)?);
        write_trajectory(&mut output, &poses, TrajectoryFormat::Tum).map_err(io_error)?;
        output.flush().map_err(io_error)?;
    }
    Ok(())
}

#[cfg(test)]
mod global_result_tests {
    use super::Options;

    #[test]
    fn export_cannot_silently_run_without_a_global_solver() {
        let mut options = Options {
            dataset_path: "dataset".into(),
            camera_calibration: "calibration.json".into(),
            global_result_path: Some("global_result.json".into()),
            mapping: false,
            ..Options::default()
        };
        assert!(options.validate().is_err());
        options.mapping = true;
        options.global_optimization = false;
        assert!(options.validate().is_err());
        options.global_optimization = true;
        assert!(options.validate().is_ok());
    }
}
