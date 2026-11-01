//! Headless queue execution of original Basalt `vio_sim.cpp`.
use crate::dataset::EndMarker;
use firefly_basalt::{
    io::marg_data_io::MargDataSaver, utils::imu_types::MargData, utils::vio_config::VioConfig,
    vi_estimator::vio_estimator::VioWorkerOutputs, vio_sim::Simulation,
};
use firefly_basalt_headers::imu::imu_types::PoseVelBiasState;
use firefly_error::{Error, ErrorKind, Result};
use std::sync::{Arc, mpsc};

fn error(message: &str) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("basalt.simulation.run")
}

/// Feeds the original noisy simulated observations and IMU on independent queues.
/// Initialization uses simulation ground truth, as in the original simulator.
/// # Errors
/// Propagates estimator, feeder and worker errors after joining all workers.
pub fn run(
    simulation: &Simulation,
    config: &VioConfig,
    use_imu: bool,
    use_double: bool,
) -> Result<Vec<Arc<PoseVelBiasState<f64>>>> {
    run_inner(simulation, config, use_imu, use_double, None)
}

/// Writes the original marginalization cache and `gt_spline.cereal` archive.
/// # Errors
/// Refuses an existing cache directory and propagates archive or worker errors.
pub fn run_with_cache(
    simulation: &Simulation,
    config: &VioConfig,
    use_imu: bool,
    use_double: bool,
    path: &std::path::Path,
) -> Result<Vec<Arc<PoseVelBiasState<f64>>>> {
    use std::io::Write;
    let saver = MargDataSaver::new(path)?;
    let mut archive = std::io::BufWriter::new(
        std::fs::File::create(path.join("gt_spline.cereal"))
            .map_err(|source| Error::from(source).with_operation("basalt.simulation.archive"))?,
    );
    simulation
        .write_archive(&mut archive)
        .map_err(|source| Error::from(source).with_operation("basalt.simulation.archive"))?;
    archive
        .flush()
        .map_err(|source| Error::from(source).with_operation("basalt.simulation.archive"))?;
    let result = run_inner(
        simulation,
        config,
        use_imu,
        use_double,
        Some(saver.in_marg_queue.clone()),
    );
    let completion = saver.finish();
    result.and_then(|states| completion.map(|()| states))
}

fn run_inner(
    simulation: &Simulation,
    config: &VioConfig,
    use_imu: bool,
    use_double: bool,
    marg: Option<mpsc::SyncSender<Option<Arc<MargData>>>>,
) -> Result<Vec<Arc<PoseVelBiasState<f64>>>> {
    let (sender, receiver) = mpsc::channel();
    let outputs = VioWorkerOutputs {
        state: Some(sender),
        marg_bounded: marg,
        ..VioWorkerOutputs::default()
    };
    let mut estimator = simulation.setup_vio_with_outputs(config, use_imu, use_double, outputs)?;
    estimator.outputs.state.take();
    estimator.outputs.marg_bounded.take();
    let vision = estimator.vision_data_queue().clone();
    let imu = estimator.imu_data_queue().clone();
    let (states, vision_result, imu_result) = std::thread::scope(|scope| {
        let vision_worker = scope.spawn(move || {
            let mut end = EndMarker(Some(vision.clone()));
            let result = simulation.vision_inputs().try_for_each(|value| {
                vision
                    .send(Some(value))
                    .map_err(|_| error("Vision consumer disconnected"))
            });
            result.and(end.finish())
        });
        let imu_worker = scope.spawn(move || {
            let mut end = EndMarker(Some(imu.clone()));
            for value in simulation.imu_inputs() {
                if imu.send(Some(value)).is_err() {
                    return;
                }
            }
            let _ = end.finish();
        });
        let mut states = Vec::new();
        while let Ok(Some(state)) = receiver.recv() {
            states.push(state);
        }
        (states, vision_worker.join(), imu_worker.join())
    });
    estimator.shutdown()?;
    vision_result.map_err(|_| error("Vision producer panicked"))??;
    imu_result.map_err(|_| error("IMU producer panicked"))?;
    Ok(states)
}
