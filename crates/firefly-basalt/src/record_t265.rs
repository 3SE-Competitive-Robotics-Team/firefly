// Copyright (c) 2019 Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Non-GUI recording workers and dataset layout from `src/rs_t265_record.cpp`.
//! Input detachment and channel closure form a drain barrier; SDK acquisition is
//! owned by the caller and must detach its destinations before stopping a session.
use crate::imu::imu_types::ImuData;
use crate::{device::rs_t265::RsPoseData, optical_flow::OpticalFlowInput};
use crossbeam_channel::{self as mpsc, Sender as SyncSender};
use firefly_calibration::calibration::Calibration;
use std::{
    fs::{self, File},
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicI32, Ordering},
    },
    thread::{self, JoinHandle},
};

pub const NUM_WORKERS: usize = 8;
pub const IMAGE_CAPACITY: usize = 1000;
pub const MOTION_CAPACITY: usize = 10000;
type Errors = Arc<Mutex<Option<io::Error>>>;
type Image = Arc<OpticalFlowInput>;
struct Channels {
    image: SyncSender<Option<Image>>,
    imu: SyncSender<Option<Arc<ImuData<f64>>>>,
    pose: SyncSender<RsPoseData>,
}

/// Clones share the recording's detach gate: after stop all sends return false.
#[derive(Clone)]
pub struct RecordingInputs(Arc<Mutex<Option<Channels>>>);
impl RecordingInputs {
    #[must_use]
    pub fn send_image(&self, image: Image) -> bool {
        self.0.lock().is_ok_and(|state| {
            state
                .as_ref()
                .is_some_and(|s| s.image.send(Some(image)).is_ok())
        })
    }
    #[must_use]
    pub fn send_imu(&self, imu: Arc<ImuData<f64>>) -> bool {
        self.0.lock().is_ok_and(|state| {
            state
                .as_ref()
                .is_some_and(|s| s.imu.send(Some(imu)).is_ok())
        })
    }
    #[must_use]
    pub fn send_pose(&self, pose: RsPoseData) -> bool {
        self.0
            .lock()
            .is_ok_and(|state| state.as_ref().is_some_and(|s| s.pose.send(pose).is_ok()))
    }
}

pub struct RecordingSession {
    pub dataset_dir: PathBuf,
    inputs: RecordingInputs,
    quality: Arc<AtomicI32>,
    errors: Errors,
    workers: Vec<JoinHandle<()>>,
    pending_image_output: Option<SyncSender<Image>>,
    pub imu_plot: mpsc::Receiver<[f32; 3]>,
}
fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}
fn remember(errors: &Errors, result: io::Result<()>) {
    if let Err(error) = result
        && let Ok(mut slot) = errors.lock()
        && slot.is_none()
    {
        *slot = Some(error);
    }
}
fn csv(path: &Path, header: &str) -> io::Result<BufWriter<File>> {
    let mut output = BufWriter::new(File::create(path)?);
    writeln!(output, "{header}")?;
    Ok(output)
}
impl RecordingSession {
    /// Starts the original eight image workers and exposure/IMU/pose workers.
    /// The UTC timestamp is supplied by the caller to keep session naming testable.
    /// # Errors
    /// Reports directory, calibration, CSV or thread creation failures.
    #[expect(
        clippy::too_many_lines,
        reason = "Keeps transactional startup and the original four worker roles together"
    )]
    pub fn start(
        root: &Path,
        unix_seconds: i64,
        manual_exposure: bool,
        calibration: Option<&Calibration<f64>>,
    ) -> io::Result<Self> {
        let dataset_dir = root.join(format!("dataset_{}", utc_date(unix_seconds)?));
        fs::create_dir_all(dataset_dir.join("mav0/cam0/data"))?;
        fs::create_dir_all(dataset_dir.join("mav0/cam1/data"))?;
        fs::create_dir_all(dataset_dir.join("mav0/imu0"))?;
        let mut cam_data = [
            csv(
                &dataset_dir.join("mav0/cam0/data.csv"),
                "#timestamp [ns], filename",
            )?,
            csv(
                &dataset_dir.join("mav0/cam1/data.csv"),
                "#timestamp [ns], filename",
            )?,
        ];
        let mut exposure_data = [
            csv(
                &dataset_dir.join("mav0/cam0/exposure.csv"),
                "#timestamp [ns], exposure time[ns]",
            )?,
            csv(
                &dataset_dir.join("mav0/cam1/exposure.csv"),
                "#timestamp [ns], exposure time[ns]",
            )?,
        ];
        let mut imu_data = csv(
            &dataset_dir.join("mav0/imu0/data.csv"),
            "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]",
        )?;
        let mut pose_data = if manual_exposure {
            None
        } else {
            fs::create_dir_all(dataset_dir.join("mav0/realsense0"))?;
            Some(csv(
                &dataset_dir.join("mav0/realsense0/data.csv"),
                "#timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []",
            )?)
        };
        if let Some(calibration) = calibration {
            save_calibration(&dataset_dir, calibration)?;
        }
        let (image_tx, image_rx) = mpsc::bounded::<Option<Image>>(IMAGE_CAPACITY);
        let (work_tx, work_rx) = mpsc::bounded::<Image>(IMAGE_CAPACITY);
        let (imu_tx, imu_rx) = mpsc::bounded::<Option<Arc<ImuData<f64>>>>(MOTION_CAPACITY);
        let (pose_tx, pose_rx) = mpsc::bounded::<RsPoseData>(MOTION_CAPACITY);
        let (plot_tx, imu_plot) = mpsc::bounded(MOTION_CAPACITY);
        let mut session = Self {
            dataset_dir,
            inputs: RecordingInputs(Arc::new(Mutex::new(Some(Channels {
                image: image_tx,
                imu: imu_tx,
                pose: pose_tx,
            })))),
            quality: Arc::new(AtomicI32::new(90)),
            errors: Arc::new(Mutex::new(None)),
            workers: Vec::new(),
            pending_image_output: Some(work_tx),
            imu_plot,
        };
        let work_rx = Arc::new(Mutex::new(work_rx));
        for index in 0..NUM_WORKERS {
            let (receiver, directory, quality, errors) = (
                Arc::clone(&work_rx),
                session.dataset_dir.clone(),
                Arc::clone(&session.quality),
                Arc::clone(&session.errors),
            );
            session.workers.push(
                thread::Builder::new()
                    .name(format!("t265-image-{index}"))
                    .spawn(move || {
                        loop {
                            let image = match receiver.lock() {
                                Ok(rx) => rx.recv(),
                                Err(_) => break,
                            };
                            let Ok(image) = image else { break };
                            for (camera, data) in image.img_data.iter().take(2).enumerate() {
                                if let Some(raw) = &data.img {
                                    let result = encode_image(raw, quality.load(Ordering::Relaxed))
                                        .and_then(|bytes| {
                                            fs::write(
                                                directory.join(format!(
                                                    "mav0/cam{camera}/data/{}.webp",
                                                    image.t_ns
                                                )),
                                                bytes,
                                            )
                                        });
                                    remember(&errors, result);
                                }
                            }
                        }
                    })?,
            );
        }
        let errors = Arc::clone(&session.errors);
        let work_tx = session
            .pending_image_output
            .take()
            .ok_or_else(|| invalid("Image output not initialized"))?;
        session
            .workers
            .push(
                thread::Builder::new()
                    .name("t265-exposure".into())
                    .spawn(move || {
                        for image in image_rx.into_iter().take_while(Option::is_some).flatten() {
                            if image.img_data.len() != 2 {
                                remember(&errors, Err(invalid("Expected two camera images")));
                                continue;
                            }
                            for camera in 0..2 {
                                remember(
                                    &errors,
                                    writeln!(
                                        cam_data[camera],
                                        "{},{}.webp",
                                        image.t_ns, image.t_ns
                                    ),
                                );
                                remember(
                                    &errors,
                                    writeln!(
                                        exposure_data[camera],
                                        "{},{}",
                                        image.t_ns,
                                        (image.img_data[camera].exposure * 1e9) as i64
                                    ),
                                );
                                // Original std::endl flushes both camera streams.
                                remember(&errors, cam_data[camera].flush());
                                remember(&errors, exposure_data[camera].flush());
                            }
                            if work_tx.send(image).is_err() {
                                remember(&errors, Err(invalid("Image workers disconnected")));
                            }
                        }
                        for file in cam_data.iter_mut().chain(exposure_data.iter_mut()) {
                            remember(&errors, file.flush());
                        }
                    })?,
            );
        let errors = Arc::clone(&session.errors);
        session.workers.push(
            thread::Builder::new()
                .name("t265-imu".into())
                .spawn(move || {
                    for data in imu_rx.into_iter().take_while(Option::is_some).flatten() {
                        let _ = plot_tx.try_send([
                            data.accel.x as f32,
                            data.accel.y as f32,
                            data.accel.z as f32,
                        ]);
                        remember(
                            &errors,
                            writeln!(
                                imu_data,
                                "{},{},{},{},{},{},{}",
                                data.t_ns,
                                scalar(data.gyro.x),
                                scalar(data.gyro.y),
                                scalar(data.gyro.z),
                                scalar(data.accel.x),
                                scalar(data.accel.y),
                                scalar(data.accel.z)
                            ),
                        );
                    }
                    remember(&errors, imu_data.flush());
                })?,
        );
        let errors = Arc::clone(&session.errors);
        session.workers.push(
            thread::Builder::new()
                .name("t265-pose".into())
                .spawn(move || {
                    for pose in pose_rx {
                        if let Some(file) = &mut pose_data {
                            let p = pose.data.translation.vector;
                            let q = pose.data.rotation.quaternion();
                            remember(
                                &errors,
                                writeln!(
                                    file,
                                    "{},{},{},{},{},{},{},{}",
                                    pose.t_ns,
                                    scalar(p.x),
                                    scalar(p.y),
                                    scalar(p.z),
                                    scalar(q.w),
                                    scalar(q.i),
                                    scalar(q.j),
                                    scalar(q.k)
                                ),
                            );
                            remember(&errors, file.flush());
                        }
                    }
                    if let Some(file) = &mut pose_data {
                        remember(&errors, file.flush());
                    }
                })?,
        );
        Ok(session)
    }
    #[must_use]
    pub fn inputs(&self) -> RecordingInputs {
        self.inputs.clone()
    }
    /// Reports a writer failure or an ended worker without consuming its error.
    /// Callers detach acquisition and call `stop` to drain and retrieve the error.
    #[must_use]
    pub fn needs_shutdown(&self) -> bool {
        let failed = match self.errors.try_lock() {
            Ok(error) => error.is_some(),
            Err(std::sync::TryLockError::Poisoned(_)) => true,
            Err(std::sync::TryLockError::WouldBlock) => false,
        };
        failed || self.workers.iter().any(JoinHandle::is_finished)
    }
    /// Clones SDK destinations. Detach these from the device before calling stop
    /// or dropping this session, so its workers can observe channel closure.
    #[cfg(feature = "realsense")]
    #[must_use]
    pub fn output_queues(&self) -> Option<crate::device::rs_t265_device::OutputQueues> {
        let state = self.inputs.0.lock().ok()?;
        let state = state.as_ref()?;
        Some(crate::device::rs_t265_device::OutputQueues {
            image: Some(state.image.clone()),
            imu: Some(state.imu.clone()),
            pose: Some(state.pose.clone()),
        })
    }
    pub fn set_webp_quality(&self, quality: i32) {
        self.quality.store(quality, Ordering::Relaxed);
    }
    /// Atomically detaches all cloned inputs and waits for accepted data to reach disk.
    /// # Errors
    /// Returns the first write/codec error or a worker panic after joining every worker.
    pub fn stop(&mut self) -> io::Result<()> {
        self.inputs
            .0
            .lock()
            .map_err(|_| invalid("Poisoned recording inputs"))?
            .take();
        self.pending_image_output.take();
        for worker in self.workers.drain(..) {
            if worker.join().is_err() {
                remember(&self.errors, Err(invalid("Recording worker panicked")));
            }
        }
        match self
            .errors
            .lock()
            .map_err(|_| invalid("Poisoned recording errors"))?
            .take()
        {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }
}
impl Drop for RecordingSession {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

/// Same u16 high-byte conversion and WebP quality interpretation as `OpenCV`.
/// # Errors
/// Reports invalid image dimensions or codec failure.
pub fn encode_image(
    image: &firefly_calibration::image::image::ManagedImage<u16>,
    quality: i32,
) -> io::Result<Vec<u8>> {
    let view = image.view();
    let width = u32::try_from(view.w).map_err(|_| invalid("Image width overflow"))?;
    let height = u32::try_from(view.h).map_err(|_| invalid("Image height overflow"))?;
    let count = view
        .w
        .checked_mul(view.h)
        .ok_or_else(|| invalid("Image size overflow"))?;
    if count == 0 {
        return Err(invalid("Empty image"));
    }
    let mut rgb = Vec::with_capacity(
        count
            .checked_mul(3)
            .ok_or_else(|| invalid("Image size overflow"))?,
    );
    for &pixel in image.data().iter().take(count) {
        let value = (pixel >> 8) as u8;
        rgb.extend_from_slice(&[value; 3]);
    }
    // Match WebPEncodeLosslessBGR's quality 70 rather than wrapper defaults.
    let encoded = webpx::Encoder::new_rgb(&rgb, width, height)
        .lossless(quality > 100)
        .quality(if quality > 100 {
            70.
        } else {
            quality.max(1) as f32
        })
        .encode(webpx::Unstoppable)
        .map_err(|error| invalid(format!("WebP encode failed: {error:?}")))?;
    Ok(encoded)
}
/// # Errors
/// Propagates calibration serialization or file errors.
pub fn save_calibration(directory: &Path, calibration: &Calibration<f64>) -> io::Result<()> {
    let mut output = BufWriter::new(File::create(directory.join("calibration.json"))?);
    serde_json::to_writer_pretty(&mut output, &serde_json::json!({"value0": calibration}))
        .map_err(io::Error::other)?;
    output.flush()
}
/// C++ default ostream precision: six significant digits and defaultfloat.
#[must_use]
#[expect(
    clippy::missing_panics_doc,
    reason = "Finite f64 scientific formatting always provides a valid integer exponent"
)]
pub fn scalar(value: f64) -> String {
    if value.is_nan() {
        return "nan".into();
    }
    if value.is_infinite() {
        return if value.is_sign_negative() {
            "-inf"
        } else {
            "inf"
        }
        .into();
    }
    let scientific = format!("{value:.5e}");
    let (mantissa, exponent) = scientific
        .split_once('e')
        .expect("Scientific format contains exponent");
    let exponent: i32 = exponent.parse().expect("Scientific exponent is an integer");
    let trim = |text: String| {
        if text.contains('.') {
            text.trim_end_matches('0').trim_end_matches('.').to_owned()
        } else {
            text
        }
    };
    if (-4..6).contains(&exponent) {
        trim(format!("{:.*}", (5 - exponent).max(0) as usize, value))
    } else {
        format!("{}e{exponent:+03}", trim(mantissa.into()))
    }
}
/// # Errors
/// Rejects dates outside 1970..9999, matching the recorder's modern UTC domain.
pub fn utc_date(unix_seconds: i64) -> io::Result<String> {
    if !(0..253_402_300_800).contains(&unix_seconds) {
        return Err(invalid("UTC date outside supported range"));
    }
    let mut days = unix_seconds / 86400;
    let seconds = unix_seconds % 86400;
    let mut year = 1970;
    let leap = |year| year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    loop {
        let length = if leap(year) { 366 } else { 365 };
        if days < length {
            break;
        }
        days -= length;
        year += 1;
    }
    let months = [
        31,
        if leap(year) { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let mut month = 0;
    while days >= months[month] {
        days -= months[month];
        month += 1;
    }
    Ok(format!(
        "{year:04}_{:02}_{:02}_{:02}_{:02}_{:02}",
        month + 1,
        days + 1,
        seconds / 3600,
        seconds / 60 % 60,
        seconds % 60
    ))
}
