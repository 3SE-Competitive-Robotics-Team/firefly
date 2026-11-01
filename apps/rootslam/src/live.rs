// Copyright (c) 2019 Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Headless device/estimator and recording paths corresponding to
//! `rs_t265_vio.cpp` and `rs_t265_record.cpp`. The SDK bridge owns acquisition.
use crate::dataset::EndMarker;
use firefly_basalt::{
    device::rs_t265_device::{OutputQueues, RsT265Device},
    optical_flow::OpticalFlowFactory,
    record_t265::RecordingSession,
    utils::vio_config::VioConfig,
    vi_estimator::vio_estimator::VioEstimatorFactory,
};
use firefly_basalt_headers::{calibration::Calibration, imu::imu_types::PoseVelBiasState};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufReader, BufWriter, Write},
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread::JoinHandle,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

#[derive(Default, Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum Task {
    #[default]
    #[serde(skip)]
    Unspecified,
    Vio,
    Record,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "Independent estimator and device switches, not lifecycle states"
)]
pub struct Options {
    pub task: Task,
    pub trajectory: Option<PathBuf>,
    pub dataset_root: Option<PathBuf>,
    pub calibration: Option<PathBuf>,
    pub vio_config: Option<PathBuf>,
    pub use_imu: bool,
    pub use_double: bool,
    pub manual_exposure: bool,
    pub exposure_ms: f64,
    pub skip_frames: i32,
    pub webp_quality: i32,
    pub reset_device: bool,
    pub duration_seconds: Option<f64>,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            task: Task::Unspecified,
            trajectory: None,
            dataset_root: None,
            calibration: None,
            vio_config: None,
            use_imu: true,
            use_double: false,
            manual_exposure: false,
            exposure_ms: 10.,
            skip_frames: 1,
            webp_quality: 90,
            reset_device: false,
            duration_seconds: None,
        }
    }
}
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("basalt.live")
}
fn io_error(error: std::io::Error) -> Error {
    Error::from(error).with_operation("basalt.live.io")
}
impl Options {
    /// # Errors
    /// Rejects missing task/output and invalid acquisition parameters.
    pub fn validate(&self) -> Result<()> {
        match self.task {
            Task::Unspecified => return Err(invalid("A live task (vio or record) is required")),
            Task::Vio if self.trajectory.is_none() => {
                return Err(invalid("VIO requires a trajectory output path"));
            }
            Task::Record if self.dataset_root.is_none() => {
                return Err(invalid("Recording requires dataset_root"));
            }
            _ => {}
        }
        if self.task == Task::Vio && self.dataset_root.is_some() {
            return Err(invalid("dataset_root is only used by the record task"));
        }
        if self.task == Task::Record
            && (self.trajectory.is_some()
                || self.calibration.is_some()
                || self.vio_config.is_some())
        {
            return Err(invalid(
                "trajectory, calibration and vio_config are only used by the vio task",
            ));
        }
        if !self.exposure_ms.is_finite()
            || self.exposure_ms <= 0.
            || self.skip_frames < 1
            || !(0..=101).contains(&self.webp_quality)
        {
            return Err(invalid(
                "Exposure and skip_frames must be positive; webp_quality must be in 0..=101",
            ));
        }
        if self
            .duration_seconds
            .is_some_and(|v| !v.is_finite() || v <= 0.)
        {
            return Err(invalid("duration_seconds must be finite and positive"));
        }
        Ok(())
    }
}
#[derive(Debug, Default)]
pub struct Report {
    pub states: usize,
    pub recording_directory: Option<PathBuf>,
}

trait Source {
    fn is_finished(&self) -> bool;
    fn calibration(&self) -> Result<Arc<Calibration<f64>>>;
    fn outputs(&self, queues: OutputQueues) -> Result<()>;
    fn stop(&mut self) -> Result<()>;
    fn detach(&self) -> Result<()>;
}
impl Source for RsT265Device {
    fn is_finished(&self) -> bool {
        self.is_finished()
    }
    fn calibration(&self) -> Result<Arc<Calibration<f64>>> {
        self.export_calibration()
            .ok_or_else(|| invalid("Device calibration unavailable"))
    }
    fn outputs(&self, queues: OutputQueues) -> Result<()> {
        self.set_output_queues(queues)
    }
    fn stop(&mut self) -> Result<()> {
        self.stop()
    }
    fn detach(&self) -> Result<()> {
        self.detach_output_queues()
    }
}

/// Opens the actual SDK device and runs until SIGINT/SIGTERM cancellation,
/// configured duration, or pipeline completion. No simulated device fallback.
/// # Errors
/// Reports missing hardware, SDK, estimation, recording and output failures.
pub fn run(options: &Options, cancel: &Arc<AtomicBool>) -> Result<Report> {
    options.validate()?;
    let mut device = RsT265Device::new(
        options.manual_exposure,
        options.skip_frames,
        options.webp_quality,
    )?;
    if device.device_count()? == 0 {
        return Err(invalid("No RealSense T265 device is connected"));
    }
    if options.reset_device {
        device.reset_first_device()?;
    }
    device.start(options.exposure_ms)?;
    let result = run_source(options, &mut device, cancel);
    // Also handles setup failures before either task owns its shutdown protocol.
    let detach = device.detach_output_queues();
    let stop = device.stop();
    result.and_then(|report| detach.and(stop).map(|()| report))
}

fn run_source(
    options: &Options,
    source: &mut impl Source,
    cancel: &Arc<AtomicBool>,
) -> Result<Report> {
    match options.task {
        Task::Vio => run_vio(options, source, cancel),
        Task::Record => run_record(options, source, cancel),
        Task::Unspecified => Err(invalid("Missing task")),
    }
}
fn elapsed(options: &Options, start: Instant) -> bool {
    options
        .duration_seconds
        .is_some_and(|seconds| start.elapsed().as_secs_f64() >= seconds)
}

fn run_record(options: &Options, source: &mut impl Source, cancel: &AtomicBool) -> Result<Report> {
    let calibration = source.calibration()?;
    let seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| invalid("System clock is before Unix epoch"))?
        .as_secs();
    let mut session = RecordingSession::start(
        options
            .dataset_root
            .as_ref()
            .ok_or_else(|| invalid("Missing dataset_root"))?,
        i64::try_from(seconds).map_err(|_| invalid("Unix timestamp overflow"))?,
        options.manual_exposure,
        Some(&calibration),
    )
    .map_err(io_error)?;
    session.set_webp_quality(options.webp_quality);
    source.outputs(
        session
            .output_queues()
            .ok_or_else(|| invalid("Recording session has no output queues"))?,
    )?;
    log::info!(
        "Recording T265 dataset to {}",
        session.dataset_dir.display()
    );
    let start = Instant::now();
    while !cancel.load(Ordering::Acquire)
        && !elapsed(options, start)
        && !source.is_finished()
        && !session.needs_shutdown()
    {
        // The original plot stream is also drained in the headless process.
        while session.imu_plot.try_recv().is_ok() {}
        std::thread::sleep(Duration::from_millis(10));
    }
    // Recording workers keep draining while the SDK producer and adapter stop.
    let stop = source.stop();
    let detach = source.detach();
    let recording = session.stop().map_err(io_error);
    stop.and(detach).and(recording)?;
    Ok(Report {
        states: 0,
        recording_directory: Some(session.dataset_dir.clone()),
    })
}

fn relay<T: Send + Sync + 'static>(
    input: crossbeam_channel::Receiver<Option<Arc<T>>>,
    output: mpsc::SyncSender<Option<Arc<T>>>,
    cancel: Arc<AtomicBool>,
) -> JoinHandle<Result<()>> {
    std::thread::spawn(move || {
        let mut ending = EndMarker(Some(output.clone()));
        for sample in input {
            let Some(sample) = sample else { break };
            if output.send(Some(sample)).is_err() {
                break;
            }
        }
        cancel.store(true, Ordering::Release);
        // Exactly one explicit marker; EndMarker covers unwinding as well.
        ending.finish()
    })
}
fn join(thread: JoinHandle<Result<()>>) -> Result<()> {
    thread.join().map_err(|_| invalid("Live feeder panicked"))?
}
fn write_state(output: &mut impl Write, state: &PoseVelBiasState<f64>) -> Result<()> {
    let pose = &state.state.pose;
    let p = pose.t_w_i.translation.vector;
    let q = pose.t_w_i.rotation;
    writeln!(
        output,
        "{:.9} {:.15} {:.15} {:.15} {:.15} {:.15} {:.15} {:.15}",
        pose.t_ns as f64 * 1e-9,
        p.x,
        p.y,
        p.z,
        q.i,
        q.j,
        q.k,
        q.w
    )
    .map_err(io_error)
}
fn estimator_configuration(
    options: &Options,
    source: &impl Source,
) -> Result<(Arc<Calibration<f64>>, VioConfig)> {
    #[derive(Deserialize)]
    struct Archive {
        value0: Calibration<f64>,
    }
    let calibration = if let Some(path) = &options.calibration {
        Arc::new(
            serde_json::from_reader::<_, Archive>(BufReader::new(
                File::open(path).map_err(io_error)?,
            ))
            .map_err(|e| invalid(&e.to_string()))?
            .value0,
        )
    } else {
        source.calibration()?
    };
    if calibration.intrinsics.len() != 2
        || calibration.t_i_c.len() != 2
        || calibration.resolution.len() != 2
    {
        return Err(invalid("T265 requires a two-camera calibration"));
    }
    let mut config = VioConfig::default();
    if let Some(path) = &options.vio_config {
        config.load(path)?;
    } else {
        config.optical_flow_skip_frames = 2;
    }
    Ok((calibration, config))
}
fn run_vio(
    options: &Options,
    source: &mut impl Source,
    cancel: &Arc<AtomicBool>,
) -> Result<Report> {
    let (calibration, config) = estimator_configuration(options, source)?;
    let mut output = BufWriter::new(
        File::create_new(
            options
                .trajectory
                .as_ref()
                .ok_or_else(|| invalid("Missing trajectory output"))?,
        )
        .map_err(io_error)?,
    );
    let mut estimator = VioEstimatorFactory::get_vio_estimator(
        &config,
        &calibration,
        &Vector3::new(0., 0., -9.81),
        options.use_imu,
        options.use_double,
    )?;
    let (state_tx, state_rx) = mpsc::channel();
    estimator.outputs.state = Some(state_tx);
    let flow = OpticalFlowFactory::get_optical_flow_with_output(
        &config,
        &calibration,
        estimator.vision_data_queue().clone(),
    )?;
    estimator.initialize(Vector3::zeros(), Vector3::zeros())?;
    estimator.outputs.state.take();
    let (image_tx, image_rx) = crossbeam_channel::bounded(10);
    let image_thread = relay(image_rx, flow.input_queue().clone(), Arc::clone(cancel));
    let (imu_tx, imu_thread) = if options.use_imu {
        let (tx, rx) = crossbeam_channel::bounded(300);
        (
            Some(tx),
            Some(relay(
                rx,
                estimator.imu_data_queue().clone(),
                Arc::clone(cancel),
            )),
        )
    } else {
        (None, None)
    };
    let setup = source.outputs(OutputQueues {
        image: Some(image_tx),
        imu: imu_tx,
        pose: None,
    });
    let start = Instant::now();
    let mut states: usize = 0;
    let streaming = (|| {
        setup?;
        while !cancel.load(Ordering::Acquire) && !elapsed(options, start) {
            match state_rx.recv_timeout(Duration::from_millis(20)) {
                Ok(Some(state)) => {
                    write_state(&mut output, &state)?;
                    states += 1;
                    if states.is_multiple_of(100) {
                        log::info!("Live estimation: {states} states");
                    }
                }
                Ok(None) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                Err(mpsc::RecvTimeoutError::Timeout) => {}
            }
        }
        Ok(())
    })();
    let stop = source.stop();
    let detach = source.detach();
    let image_result = join(image_thread);
    let imu_result = imu_thread.map_or(Ok(()), join);
    let flow_result = flow.shutdown();
    let estimator_result = estimator.shutdown();
    // State output is unbounded, allowing estimator completion while final
    // acquisition queues drain. Preserve every final state in the trajectory.
    let mut trailing = Ok(());
    while let Ok(Some(state)) = state_rx.recv() {
        if trailing.is_ok() {
            trailing = write_state(&mut output, &state);
            states += 1;
        }
    }
    let flush = output.flush().map_err(io_error);
    streaming
        .and(stop)
        .and(detach)
        .and(image_result)
        .and(imu_result)
        .and(flow_result)
        .and(estimator_result)
        .and(trailing)
        .and(flush)?;
    if states == 0 {
        return Err(invalid("Live estimator produced no states"));
    }
    log::info!("Live estimator stopped after {states} states");
    Ok(Report {
        states,
        recording_directory: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_basalt::{
        device::rs_t265::RsPoseData, io::dataset_io::ImageData, optical_flow::OpticalFlowInput,
    };
    use firefly_basalt_headers::{
        camera::generic_camera::GenericCamera, image::image::ManagedImage, imu::imu_types::ImuData,
    };
    use nalgebra::{Isometry3, Vector2, Vector4};
    use std::sync::{Mutex, atomic::AtomicUsize};

    struct Directory(PathBuf);
    impl Directory {
        fn new(name: &str) -> Self {
            let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../logs")
                .join(format!("basalt-live-{name}-{}", std::process::id()));
            std::fs::create_dir_all(&path).unwrap();
            Self(path)
        }
    }
    impl Drop for Directory {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    fn calibration() -> Calibration<f64> {
        let mut camera = GenericCamera::from_name("pinhole");
        camera.set_from_init(Vector4::new(80., 80., 64., 48.));
        Calibration {
            intrinsics: vec![camera.clone(), camera],
            t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.1, 0., 0.)],
            resolution: vec![Vector2::new(128, 96); 2],
            ..Calibration::default()
        }
    }
    struct InjectedSource {
        failed: bool,
        worker: Mutex<Option<JoinHandle<()>>>,
        queues: Mutex<Option<OutputQueues>>,
        cancel: Arc<AtomicBool>,
        stops: AtomicUsize,
        detaches: AtomicUsize,
    }
    impl InjectedSource {
        fn new(cancel: Arc<AtomicBool>) -> Self {
            Self {
                failed: false,
                worker: Mutex::new(None),
                queues: Mutex::new(None),
                cancel,
                stops: AtomicUsize::new(0),
                detaches: AtomicUsize::new(0),
            }
        }
    }
    impl Source for InjectedSource {
        fn is_finished(&self) -> bool {
            (self.failed && self.queues.lock().unwrap().is_some())
                || self
                    .worker
                    .lock()
                    .unwrap()
                    .as_ref()
                    .is_some_and(JoinHandle::is_finished)
        }
        fn calibration(&self) -> Result<Arc<Calibration<f64>>> {
            Ok(Arc::new(calibration()))
        }
        fn outputs(&self, queues: OutputQueues) -> Result<()> {
            *self.queues.lock().unwrap() = Some(queues.clone());
            if self.failed {
                if let Some(sender) = &queues.image {
                    sender.send(None).unwrap();
                }
                if let Some(sender) = &queues.imu {
                    sender.send(None).unwrap();
                }
                return Ok(());
            }
            let cancel = Arc::clone(&self.cancel);
            *self.worker.lock().unwrap() = Some(std::thread::spawn(move || {
                for index in 0..4 {
                    let t_ns = 100_000_000 + index * 50_000_000;
                    if let Some(sender) = &queues.imu {
                        sender
                            .send(Some(Arc::new(ImuData {
                                t_ns,
                                accel: Vector3::new(0., 0., 9.81),
                                gyro: Vector3::zeros(),
                            })))
                            .unwrap();
                    }
                    if let Some(sender) = &queues.pose {
                        sender
                            .send(RsPoseData {
                                t_ns,
                                data: Isometry3::identity(),
                            })
                            .unwrap();
                    }
                    if let Some(sender) = &queues.image {
                        let img_data = (0..2)
                            .map(|camera| {
                                let mut image = ManagedImage::new(128, 96);
                                for y in 0..96 {
                                    for x in 0..128 {
                                        let shifted = x + camera * 8;
                                        image.data_mut()[y * 128 + x] =
                                            (((shifted * 17 + y * 31 + shifted * y % 71) % 256)
                                                << 8)
                                                as u16;
                                    }
                                }
                                ImageData {
                                    img: Some(Arc::new(image)),
                                    exposure: 0.005,
                                }
                            })
                            .collect();
                        sender
                            .send(Some(Arc::new(OpticalFlowInput { t_ns, img_data })))
                            .unwrap();
                    }
                }
                if let Some(sender) = &queues.image {
                    sender.send(None).unwrap();
                }
                if let Some(sender) = &queues.imu {
                    sender.send(None).unwrap();
                }
                cancel.store(true, Ordering::Release);
            }));
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            self.stops.fetch_add(1, Ordering::Relaxed);
            if let Some(worker) = self.worker.lock().unwrap().take() {
                worker.join().unwrap();
            }
            if self.failed {
                Err(invalid("Injected device processing failure"))
            } else {
                Ok(())
            }
        }
        fn detach(&self) -> Result<()> {
            self.detaches.fetch_add(1, Ordering::Relaxed);
            self.queues.lock().unwrap().take();
            Ok(())
        }
    }
    #[test]
    fn configuration_retains_record_lossless_and_rejects_unknown_fields() {
        let options: Options =
            toml::from_str("task='record'\ndataset_root='capture'\nwebp_quality=101\n").unwrap();
        options.validate().unwrap();
        assert!(
            Options {
                calibration: Some("ignored.json".into()),
                ..options
            }
            .validate()
            .is_err()
        );
        assert!(Options::default().validate().is_err());
        assert!(toml::from_str::<Options>("task='vio'\ntrajectroy='bad'\n").is_err());
        let mut options = Options {
            task: Task::Vio,
            trajectory: Some("pose.tum".into()),
            ..Options::default()
        };
        options.duration_seconds = Some(f64::NAN);
        assert!(options.validate().is_err());
    }
    #[test]
    fn image_pipeline_uses_real_optical_flow_and_vo_vio_factory() {
        for use_imu in [false, true] {
            for use_double in [false, true] {
                let directory = Directory::new("vo");
                let output = directory.0.join(format!("trajectory-{use_imu}.tum"));
                let options = Options {
                    task: Task::Vio,
                    trajectory: Some(output.clone()),
                    use_imu,
                    use_double,
                    ..Options::default()
                };
                let cancel = Arc::new(AtomicBool::new(false));
                let mut source = InjectedSource::new(Arc::clone(&cancel));
                let report = run_source(&options, &mut source, &cancel).unwrap();
                assert_eq!(report.states, 2); // Original live default optical_flow_skip_frames = 2.
                let trajectory = std::fs::read_to_string(output).unwrap();
                assert_eq!(trajectory.lines().count(), 2);
                for line in trajectory.lines() {
                    let fields: Vec<f64> = line
                        .split_whitespace()
                        .map(|v| v.parse().unwrap())
                        .collect();
                    assert_eq!(fields.len(), 8);
                    assert!(fields.iter().all(|v| v.is_finite()));
                }
                assert_eq!(source.stops.load(Ordering::Relaxed), 1);
                assert_eq!(source.detaches.load(Ordering::Relaxed), 1);
            }
        }
    }
    #[test]
    fn recording_stops_producer_then_drains_workers() {
        let directory = Directory::new("record");
        let options = Options {
            task: Task::Record,
            dataset_root: Some(directory.0.clone()),
            webp_quality: 101,
            ..Options::default()
        };
        let cancel = Arc::new(AtomicBool::new(false));
        let mut source = InjectedSource::new(Arc::clone(&cancel));
        let report = run_source(&options, &mut source, &cancel).unwrap();
        let path = report.recording_directory.unwrap();
        for name in ["cam0", "cam1", "imu0", "realsense0"] {
            assert_eq!(
                std::fs::read_to_string(path.join(format!("mav0/{name}/data.csv")))
                    .unwrap()
                    .lines()
                    .count(),
                5
            );
        }
        assert_eq!(
            std::fs::read_dir(path.join("mav0/cam0/data"))
                .unwrap()
                .count(),
            4
        );
        assert_eq!(source.stops.load(Ordering::Relaxed), 1);
        assert_eq!(source.detaches.load(Ordering::Relaxed), 1);
    }
    #[test]
    fn recording_observes_device_failure_without_user_signal() {
        let directory = Directory::new("failure");
        let options = Options {
            task: Task::Record,
            dataset_root: Some(directory.0.clone()),
            ..Options::default()
        };
        let cancel = Arc::new(AtomicBool::new(false));
        let worker_cancel = Arc::clone(&cancel);
        let (tx, rx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            let mut source = InjectedSource::new(Arc::clone(&worker_cancel));
            source.failed = true;
            tx.send(run_source(&options, &mut source, &worker_cancel))
                .unwrap();
        });
        let result = rx.recv_timeout(Duration::from_secs(2));
        cancel.store(true, Ordering::Release);
        worker.join().unwrap();
        assert!(
            result
                .unwrap()
                .unwrap_err()
                .to_string()
                .contains("Injected device processing failure")
        );
    }
    #[test]
    fn writer_failure_stops_recording_without_eos_or_user_signal() {
        struct WriteFailureSource {
            root: PathBuf,
            queues: Mutex<Option<OutputQueues>>,
        }
        impl Source for WriteFailureSource {
            fn is_finished(&self) -> bool {
                false
            }
            fn calibration(&self) -> Result<Arc<Calibration<f64>>> {
                Ok(Arc::new(calibration()))
            }
            fn outputs(&self, queues: OutputQueues) -> Result<()> {
                let dataset = std::fs::read_dir(&self.root)
                    .unwrap()
                    .next()
                    .unwrap()
                    .unwrap()
                    .path();
                std::fs::create_dir(dataset.join("mav0/cam0/data/123.webp")).unwrap();
                let image = Arc::new(ManagedImage::new(16, 16));
                queues
                    .image
                    .as_ref()
                    .unwrap()
                    .send(Some(Arc::new(OpticalFlowInput {
                        t_ns: 123,
                        img_data: vec![
                            ImageData {
                                img: Some(Arc::clone(&image)),
                                exposure: 0.005,
                            },
                            ImageData {
                                img: Some(image),
                                exposure: 0.005,
                            },
                        ],
                    })))
                    .unwrap();
                *self.queues.lock().unwrap() = Some(queues);
                Ok(())
            }
            fn stop(&mut self) -> Result<()> {
                Ok(())
            }
            fn detach(&self) -> Result<()> {
                self.queues.lock().unwrap().take();
                Ok(())
            }
        }
        let directory = Directory::new("write-failure");
        let root = directory.0.clone();
        let options = Options {
            task: Task::Record,
            dataset_root: Some(root.clone()),
            ..Options::default()
        };
        let cancel = Arc::new(AtomicBool::new(false));
        let worker_cancel = Arc::clone(&cancel);
        let (tx, rx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            let mut source = WriteFailureSource {
                root,
                queues: Mutex::new(None),
            };
            tx.send(run_source(&options, &mut source, &worker_cancel))
                .unwrap();
        });
        let result = rx.recv_timeout(Duration::from_secs(2));
        cancel.store(true, Ordering::Release);
        worker.join().unwrap();
        assert!(result.unwrap().is_err());
    }
    #[test]
    fn relay_sends_one_eos_on_marker_and_disconnect() {
        for explicit in [true, false] {
            let (tx, rx) = crossbeam_channel::bounded(2);
            let (out_tx, out_rx) = mpsc::sync_channel(4);
            let cancel = Arc::new(AtomicBool::new(false));
            tx.send(Some(Arc::new(7_u32))).unwrap();
            if explicit {
                tx.send(None).unwrap();
            }
            drop(tx);
            join(relay(rx, out_tx, Arc::clone(&cancel))).unwrap();
            assert_eq!(*out_rx.recv().unwrap().unwrap(), 7);
            assert!(out_rx.recv().unwrap().is_none());
            assert!(out_rx.recv().is_err());
            assert!(cancel.load(Ordering::Acquire));
        }
    }
}
