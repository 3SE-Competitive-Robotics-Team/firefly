use crate::dataset_output::{
    OriginalResult, TrajectoryFormat, align_ground_truth, write_trajectory,
};
mod outputs;
use crate::{
    config,
    input::{SensorModel, Source},
};
use firefly_basalt::vi_estimator::vio_estimator::VioEstimatorFactory;
use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam_mapping::{
    global_optimization::{KrylovPrecision, OptimizationOptions, Precision},
    loop_closure::LoopOptions,
    map::MapPolicy,
};
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    trajectory: Option<PathBuf>,
    /// Causal local posterior, before persistent-map and final BA corrections.
    local_trajectory: Option<PathBuf>,
    trajectory_format: TrajectoryFormat,
    groundtruth: Option<PathBuf>,
    marg_data: Option<PathBuf>,
    debug_directory: Option<PathBuf>,
    result_path: Option<PathBuf>,
    metrics_path: Option<PathBuf>,
    global_result_path: Option<PathBuf>,
    /// Ground-truth interpolation gap, using Basalt's association limit.
    max_ground_truth_gap_ns: i64,
    use_imu: bool,
    global_optimization: bool,
    #[serde(deserialize_with = "deserialize_optimization")]
    optimization: OptimizationOptions,
    #[serde(rename = "loop_options")]
    loops: LoopOptions,
    map_policy: Option<MapPolicy>,
    max_frames: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            trajectory: None,
            local_trajectory: None,
            trajectory_format: TrajectoryFormat::default(),
            groundtruth: None,
            marg_data: None,
            debug_directory: None,
            result_path: None,
            metrics_path: None,
            global_result_path: None,
            max_ground_truth_gap_ns: 110_000_000,
            use_imu: true,
            global_optimization: true,
            optimization: OptimizationOptions::default(),
            loops: LoopOptions::default(),
            map_policy: None,
            max_frames: 0,
        }
    }
}
impl Options {
    fn resolved_optimization(&self, precision: Precision) -> OptimizationOptions {
        OptimizationOptions {
            precision,
            krylov_precision: KrylovPrecision::Native,
            ..self.optimization.clone()
        }
    }
    /// # Errors
    /// Rejects invalid export and global optimization settings.
    pub fn validate(&self, mapping: bool) -> Result<()> {
        if self.local_trajectory.is_some() && self.local_trajectory == self.trajectory {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "local_trajectory and trajectory must use different paths",
            ));
        }
        if self.global_result_path.is_some() && !(mapping && self.global_optimization) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "global_result_path requires mapping and global_optimization",
            )
            .with_operation("rootslam.config.validate"));
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

// Top-level precision selects application arithmetic. Frozen-problem tools
// expose independent controls for numerical comparisons.
fn deserialize_optimization<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> std::result::Result<OptimizationOptions, D::Error> {
    let value = serde_json::Value::deserialize(deserializer)?;
    for key in ["precision", "krylov_precision"] {
        if value.get(key).is_some() {
            return Err(serde::de::Error::custom(format!(
                "dataset.optimization.{key} is not supported; set top-level precision to float32 or float64"
            )));
        }
    }
    serde_json::from_value(value).map_err(serde::de::Error::custom)
}
fn io_error(error: std::io::Error) -> Error {
    Error::new(ErrorKind::Internal, "Dataset application I/O failed")
        .with_operation("rootslam.dataset.io")
        .with_source(error)
}
/// # Errors
/// Propagates dataset input, estimation, mapping and output failures.
pub fn run(
    options: &Options,
    source: &Source,
    model: &SensorModel,
    vio: &config::Options,
    precision: Precision,
    stop: &std::sync::atomic::AtomicBool,
) -> Result<()> {
    options.validate(vio.map)?;
    outputs::validate(options, source, vio.vio_config.as_deref())?;
    let started = std::time::Instant::now();
    let calibration = &model.calibration;
    let config = config::initial_vio_config(vio)?;
    let optimization = options.resolved_optimization(precision);
    let dataset = source.dataset()?;
    let evaluation_data = evaluation_data(options, &dataset, calibration.cam_time_offset_ns)?;
    let mut estimator = VioEstimatorFactory::get_vio_estimator(
        &config,
        calibration,
        &model.gravity,
        options.use_imu,
        precision == Precision::Float64,
    )?;
    let mapping = if vio.map {
        let system_options = crate::system::SystemOptions {
            global_optimization: options.global_optimization,
            loop_closures: vio.loops,
            optimization: optimization.clone(),
            loops: options.loops.clone(),
            map: options
                .map_policy
                .clone()
                .unwrap_or_else(|| crate::system::SystemOptions::default().map),
        };
        let worker = crate::system::MappingWorker::spawn(calibration.clone(), system_options)?;
        estimator.outputs.local_map = Some(worker.input());
        Some(worker)
    } else {
        None
    };
    let report = crate::dataset::run_with_options(
        dataset,
        calibration,
        &config,
        estimator,
        &crate::dataset::RunOptions {
            max_frames: options.max_frames,
            stop: Some(stop),
            marg_data: options.marg_data.as_deref(),
            debug_directory: options.debug_directory.as_deref(),
        },
    );
    let report = match report {
        Ok(report) => report,
        Err(failure) => {
            if let Some(worker) = mapping
                && let Err(cleanup) = worker.cancel()
            {
                return Err(failure.with_context("mapping_cleanup", cleanup));
            }
            return Err(failure);
        }
    };
    let states = report.states;
    if let Some(path) = &options.local_trajectory {
        let estimates: Vec<_> = states
            .iter()
            .map(|state| (state.state.pose.t_ns, state.state.pose.t_w_i))
            .collect();
        save_trajectory(path, &estimates, options.trajectory_format)?;
    }
    if stop.load(std::sync::atomic::Ordering::Acquire) {
        let interrupted = Error::new(ErrorKind::Internal, "Dataset run interrupted")
            .with_operation("rootslam.dataset.run");
        if let Some(worker) = mapping
            && let Err(cleanup) = worker.cancel()
        {
            return Err(interrupted.with_context("mapping_cleanup", cleanup));
        }
        return Err(interrupted);
    }
    save_original_results(options, &states, report.duration, evaluation_data.as_ref())?;
    let mapped = mapping
        .map(|worker| worker.finish_with_stop(stop))
        .transpose()?;
    if let Some(result) = &mapped {
        report_mapping(result);
    }
    save_global_result(options, mapped.as_ref(), optimization)?;
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
    if stop.load(std::sync::atomic::Ordering::Relaxed) {
        return Err(Error::new(ErrorKind::Internal, "Dataset run interrupted")
            .with_operation("rootslam.dataset.run"));
    }
    Ok(())
}

fn save_global_result(
    options: &Options,
    mapped: Option<&crate::system::SystemResult>,
    optimization: OptimizationOptions,
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
    firefly_rootslam_mapping::global_problem::ReplayResult::from_result(global, optimization)
        .write_new(path)
}

fn report_mapping(result: &crate::system::SystemResult) {
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
        log::info!(
            "Global BA numerics: matrix precision {:?}, Krylov precision {:?}",
            global.precision,
            global.krylov_precision
        );
    }
}

type EvaluationData = (Vec<i64>, Vec<i64>, Vec<nalgebra::Isometry3<f64>>);
fn evaluation_data(
    options: &Options,
    dataset: &firefly_basalt::io::dataset_io::VioDatasetPtr,
    camera_offset_ns: i64,
) -> Result<Option<EvaluationData>> {
    if options.metrics_path.is_none()
        && options.result_path.is_none()
        && options.groundtruth.is_none()
        && options.debug_directory.is_none()
    {
        return Ok(None);
    }
    let mut dataset = dataset
        .lock()
        .map_err(|_| Error::new(ErrorKind::Internal, "Dataset mutex poisoned"))?;
    // Evaluation uses the estimator's calibrated camera clock and the supplied GT clock.
    let times = dataset
        .image_timestamps()
        .iter()
        .map(|&time| crate::input::camera_time_in_imu_clock(time, camera_offset_ns))
        .collect::<Result<Vec<_>>>()?;
    Ok(Some((
        times,
        dataset.gt_timestamps().to_vec(),
        dataset.gt_pose_data().to_vec(),
    )))
}
fn save_trajectory(
    path: &std::path::Path,
    estimates: &[(i64, nalgebra::Isometry3<f64>)],
    format: TrajectoryFormat,
) -> Result<()> {
    let mut output = BufWriter::new(File::create_new(path).map_err(io_error)?);
    write_trajectory(&mut output, estimates, format).map_err(io_error)?;
    output.flush().map_err(io_error)
}

fn save_results(
    options: &Options,
    states: &[std::sync::Arc<firefly_basalt::imu::imu_types::PoseVelBiasState<f64>>],
    mapped: Option<&crate::system::SystemResult>,
    evaluation_data: Option<EvaluationData>,
) -> Result<()> {
    if let Some(path) = &options.trajectory {
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
        save_trajectory(path, &estimates, options.trajectory_format)?;
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
        let metrics = crate::evaluation::evaluate(
            &times,
            &estimates,
            &gt_times,
            &ground_truth,
            options.max_ground_truth_gap_ns,
        )?;
        let mut output = BufWriter::new(File::create_new(path).map_err(io_error)?);
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
    states: &[std::sync::Arc<firefly_basalt::imu::imu_types::PoseVelBiasState<f64>>],
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
        crate::dataset_output::original_statistics(
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
        let mut output = BufWriter::new(File::create_new(path).map_err(io_error)?);
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
        let mut output = BufWriter::new(File::create_new(path).map_err(io_error)?);
        write_trajectory(&mut output, &poses, TrajectoryFormat::Tum).map_err(io_error)?;
        output.flush().map_err(io_error)?;
    }
    Ok(())
}

#[cfg(test)]
mod global_result_tests {
    use super::Options;

    #[test]
    fn application_precision_controls_the_actual_solver_and_exported_options() {
        use super::{KrylovPrecision, Precision};
        use firefly_calibration::calibration::Calibration;
        use firefly_rootslam_mapping::{
            global_optimization::optimize_global, global_problem::ReplayResult, map::MapSnapshot,
        };
        let options: Options = toml::from_str("[optimization]\nmax_iterations = 7").unwrap();
        for precision in [Precision::Float32, Precision::Float64] {
            let resolved = options.resolved_optimization(precision);
            assert_eq!(resolved.max_iterations, 7);
            let result =
                optimize_global(&MapSnapshot::default(), &Calibration::default(), &resolved)
                    .unwrap();
            assert_eq!(result.precision, precision);
            assert_eq!(result.krylov_precision, KrylovPrecision::Native);
            let replay = ReplayResult::from_result(&result, resolved);
            assert_eq!(replay.options.precision, result.precision);
            assert_eq!(replay.options.krylov_precision, result.krylov_precision);
        }
    }

    #[test]
    fn local_output_cannot_be_overwritten_by_final_trajectory() {
        let mut options = Options {
            trajectory: Some("trajectory.txt".into()),
            local_trajectory: Some("trajectory.txt".into()),
            ..Options::default()
        };
        assert!(options.validate(true).is_err());
        options.local_trajectory = Some("local_trajectory.txt".into());
        assert!(options.validate(true).is_ok());
    }

    #[test]
    fn export_cannot_silently_run_without_a_global_solver() {
        let mut options = Options {
            global_result_path: Some("global_result.json".into()),
            ..Options::default()
        };
        assert!(options.validate(false).is_err());
        options.global_optimization = false;
        assert!(options.validate(true).is_err());
        options.global_optimization = true;
        assert!(options.validate(true).is_ok());
    }
}
