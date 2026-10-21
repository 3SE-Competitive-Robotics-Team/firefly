// SPDX-License-Identifier: BSD-3-Clause
//! Single-process online VIO, loop closure and depth mapping over iceoryx2.
pub mod config;
mod input;
mod output;
#[cfg(test)]
mod tests;

use crate::{
    application::okvis2x_app_synchronous::EstimatorApplication,
    ceres::vi_slam_backend::ViSlamBackend,
    frontend::{
        frame_absolute_pose_sac_problem::OpenGvSolver,
        pipeline::{Frontend, RustBriskFeatureFactory},
    },
    mapping::{
        map_typedefs::SharedOccupancyMaps,
        object_mapping::OccupancyMapBackend,
        submapping_interface::{DepthFrames, SubmappingInterface},
    },
    runtime::{
        threaded_slam::{
            OccupancyMappingRuntime, ProcessingCore, PublicationData, SensorInputConfig,
            ThreadedSlam,
        },
        vi_interface::ViInterface,
    },
    sensors::measurements::{CameraData, Measurement},
};
use config::{Config, invalid};
use firefly_error::Result;
use firefly_supereight2::data::DataConfig;
use std::{
    path::Path,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::Duration,
};

/// Runs until IPC termination or a sensor, estimator or mapping worker fails.
/// # Errors
/// Returns startup, sensor validation, estimator, mapper, publication and worker errors.
pub fn run(config: Config, vocabulary: Option<&Path>) -> Result<()> {
    config.validate()?;
    let parameters = config.parameters()?;
    let vocabulary = load_vocabulary(vocabulary)?;
    let maps = SharedOccupancyMaps::default();
    let submaps = Config::submaps();
    let mut estimator = create_estimator(&parameters, &submaps, vocabulary, maps.clone())?;
    let mut mapping = if config.mapping {
        let mut backend = OccupancyMapBackend::new(config.map(), DataConfig::default());
        backend.shared_maps = maps;
        let mut mapper =
            SubmappingInterface::new(submaps, &parameters, backend).map_err(invalid)?;
        mapper.set_t_bs(parameters.imu.t_bs.clone());
        mapper
            .state
            .lock()
            .map_err(|_| invalid("mapping state poisoned"))?
            .align_callback = Some(estimator.alignment_callback());
        mapper.start().map_err(invalid)?;
        Some(mapper)
    } else {
        None
    };
    let map_inputs = mapping.as_ref().map(SubmappingInterface::input_handle);
    let (publications, posterior_rx) = mpsc::sync_channel(8);
    let (depths, depth_rx) = mpsc::sync_channel(4);
    let (wake, ready) = mpsc::sync_channel(1);
    let stop = Arc::new(AtomicBool::new(false));
    let failure = Arc::new(Mutex::new(None::<String>));
    let stopping = stop.clone();
    let errors = failure.clone();
    estimator.set_optimised_graph_callback(Arc::new(move |state, tracking, updated, landmarks| {
        if let Some(mapper) = &map_inputs
            && let Err(error) =
                mapper.state_update_callback(state.clone(), tracking.clone(), updated.clone())
        {
            latch(&errors, &stopping, error.to_string());
        }
        if stopping.load(Ordering::Acquire) {
            return;
        }
        if publications
            .send(PublicationData {
                state: state.clone(),
                tracking_state: tracking.clone(),
                updated_states: updated,
                landmarks,
            })
            .is_err()
        {
            latch(&errors, &stopping, "IPC posterior receiver stopped".into());
        }
    }));
    let ports = input::Ports {
        topics: input::Topics::default(),
        config,
        sensors: estimator.sensor_input_handle(),
        publications: posterior_rx,
        depths,
        wake,
        stop: stop.clone(),
    };
    let io = spawn_transport(ports, stop.clone())?;
    let inputs = HostInputs {
        depth: depth_rx,
        ready,
        failure: failure.clone(),
        stop: stop.clone(),
    };
    let result = process(&mut estimator, mapping.as_ref(), &inputs);
    stop.store(true, Ordering::Release);
    let transport = io
        .join()
        .map_err(|_| invalid("IPC worker panicked"))
        .and_then(|result| result);
    let estimator = estimator.shutdown();
    let mapper = mapping
        .as_mut()
        .map_or(Ok(()), |mapper| mapper.set_finished().map_err(invalid));
    let callback = failure
        .lock()
        .map_err(|_| invalid("callback failure state poisoned"))?
        .take()
        .map_or(Ok(()), |message| Err(invalid(message)));
    for error in [&transport, &estimator, &mapper, &callback]
        .into_iter()
        .filter_map(|result| result.as_ref().err())
    {
        log::error!("OKVIS shutdown: {error}");
    }
    result
        .and(transport)
        .and(estimator)
        .and(mapper)
        .and(callback)
}
fn latch(failure: &Mutex<Option<String>>, stop: &AtomicBool, message: String) {
    if let Ok(mut failure) = failure.lock() {
        failure.get_or_insert(message);
    }
    stop.store(true, Ordering::Release);
}

fn create_estimator(
    parameters: &crate::config::parameters::ViParameters,
    submaps: &crate::mapping::config_mapping::SubMapConfig,
    vocabulary: Vocabulary,
    maps: SharedOccupancyMaps,
) -> Result<ThreadedSlam> {
    let params = parameters.clone();
    let submap_config = submaps.clone();
    let worker_maps = maps;
    let estimator = ThreadedSlam::new(
        SensorInputConfig::new(parameters, submaps),
        move || {
            let solver: OpenGvSolver = OpenGvSolver::default();
            let frontend = Frontend::with_vocabulary(
                params.n_camera_system.num_cameras(),
                &vocabulary,
                solver,
                RustBriskFeatureFactory,
            );
            let mut core = ProcessingCore::new(
                ViSlamBackend::default(),
                frontend,
                params,
                submap_config,
                OccupancyMappingRuntime::new(worker_maps),
            );
            core.enable_covariance();
            Ok(core)
        },
        None,
    )?;
    estimator.enable_lossless_publications();
    Ok(estimator)
}

struct HostInputs {
    depth: mpsc::Receiver<input::Depth>,
    ready: mpsc::Receiver<()>,
    failure: Arc<Mutex<Option<String>>>,
    stop: Arc<AtomicBool>,
}
fn process(
    estimator: &mut ThreadedSlam,
    mapping: Option<&SubmappingInterface<OccupancyMapBackend>>,
    inputs: &HostInputs,
) -> Result<()> {
    while !inputs.stop.load(Ordering::Acquire) {
        if let Some(error) = inputs
            .failure
            .lock()
            .map_err(|_| invalid("callback failure state poisoned"))?
            .take()
        {
            return Err(invalid(error));
        }
        while let Ok(depth) = inputs.depth.try_recv() {
            if let Some(mapper) = &mapping {
                // The estimator applies image_delay itself; the mapping input adjusts its owned copy.
                estimator.add_depth_measurement(depth.time, &depth.image, None);
                let mut frames = DepthFrames::from([(
                    2,
                    vec![Measurement::new(
                        depth.time,
                        CameraData {
                            depth_image: Some(depth.image),
                            ..Default::default()
                        },
                        2,
                    )],
                )]);
                mapper.add_depth_measurement(&mut frames).map_err(invalid)?;
            }
        }
        let _ = inputs.ready.recv_timeout(Duration::from_millis(5));
        estimator.process_frame()?;
        if let Some(mapper) = &mapping {
            mapper.check_health().map_err(invalid)?;
        }
    }
    Ok(())
}

type Vocabulary =
    firefly_dbow2::templated_vocabulary::TemplatedVocabulary<crate::frontend::f_brisk::FBrisk>;
fn load_vocabulary(directory: Option<&Path>) -> Result<Vocabulary> {
    directory
        .map_or_else(
            || Vocabulary::load_bytes(include_bytes!("../../../resources/small_voc.yml.gz")),
            |directory| Vocabulary::load(directory.join("small_voc.yml.gz")),
        )
        .map_err(invalid)
}

struct StopOnDrop {
    stop: Arc<AtomicBool>,
    sensors: crate::runtime::threaded_slam::SensorInputHandle,
}
impl Drop for StopOnDrop {
    fn drop(&mut self) {
        self.sensors.close_inputs();
        self.stop.store(true, Ordering::Release);
    }
}
fn spawn_transport(
    ports: input::Ports,
    stop: Arc<AtomicBool>,
) -> Result<std::thread::JoinHandle<Result<()>>> {
    std::thread::Builder::new()
        .name("okvis-ipc".into())
        .spawn(move || {
            let _stopping = StopOnDrop {
                stop,
                sensors: ports.sensors.clone(),
            };
            input::run(ports)
        })
        .map_err(|error| {
            firefly_error::Error::new(firefly_error::ErrorKind::Internal, "start IPC worker")
                .with_source(error)
                .with_operation("okvis2_x::ipc")
        })
}
