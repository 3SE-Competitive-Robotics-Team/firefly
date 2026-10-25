/* OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! Shared deep-learning callback, backpressure and worker lifetime contract.

use super::utils::invalid;
use crate::{
    frames::frame_typedefs::StateId,
    runtime::vi_interface::{State, TrackingState},
    sensors::measurements::{CameraData, CameraMeasurement, Measurement},
};
use firefly_error::{Error, Result};
use jiff::Timestamp;
use purecv::core::DynamicMatrix;
use std::{
    collections::BTreeMap,
    sync::{
        Arc, Mutex, RwLock,
        atomic::{AtomicBool, AtomicUsize, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

pub type Images = BTreeMap<usize, (Timestamp, Arc<DynamicMatrix>)>;
pub type CameraMeasurements = BTreeMap<usize, Vec<CameraMeasurement>>;
pub type DisplayImages = BTreeMap<String, Arc<DynamicMatrix>>;
pub type ImageCallback = Arc<dyn Fn(&mut CameraMeasurements) -> Result<bool> + Send + Sync>;
pub type LiveDepthCallback =
    Arc<dyn Fn(Timestamp, &DynamicMatrix, Option<&DynamicMatrix>) -> Result<bool> + Send + Sync>;
pub type UpdatedStates = Arc<BTreeMap<StateId, State>>;

pub trait DeepLearningProcessor: Send + Sync {
    fn set_blocking(&self, blocking: bool);
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn display(&self, images: &mut DisplayImages) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn add_images(&self, images: &Images, depth_images: &Images) -> Result<bool>;
    fn set_image_callback(&self, callback: ImageCallback);
    fn set_live_depth_image_callback(&self, _callback: LiveDepthCallback) {}
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn state_update_callback(
        &self,
        _latest: &State,
        _tracking: &TrackingState,
        _updated: UpdatedStates,
    ) -> Result<bool> {
        Ok(false)
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn finished_processing(&self) -> Result<bool>;
    fn is_depth_fusion(&self) -> bool {
        false
    }
}

/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn camera_measurements(
    images: &Images,
    depth_images: Option<&Images>,
) -> Result<CameraMeasurements> {
    let mut frames = CameraMeasurements::new();
    for (&id, (time, image)) in images {
        let data = CameraData {
            image: Some(image.clone()),
            ..CameraData::default()
        };
        let sensor = i32::try_from(id)
            .map_err(|_| invalid("camera index exceeds i32", "camera_measurements"))?;
        frames.insert(id, vec![Measurement::new(*time, data, sensor)]);
    }
    if let Some(depths) = depth_images {
        for (&id, (time, depth)) in depths {
            let sensor = i32::try_from(id)
                .map_err(|_| invalid("camera index exceeds i32", "camera_measurements"))?;
            let entries = frames.entry(id).or_default();
            if let Some(measurement) = entries.iter_mut().find(|m| m.time_stamp == *time) {
                measurement.measurement.depth_image = Some(depth.clone());
            } else {
                entries.push(Measurement::new(
                    *time,
                    CameraData {
                        depth_image: Some(depth.clone()),
                        ..CameraData::default()
                    },
                    sensor,
                ));
            }
        }
    }
    Ok(frames)
}

pub struct ProcessingControl {
    pub shutdown: AtomicBool,
    pub blocking: RwLock<Option<bool>>,
    pub is_processing: AtomicBool,
    pub pending: AtomicUsize,
    pub callback: RwLock<Option<ImageCallback>>,
    pub error: Mutex<Option<Error>>,
}
impl ProcessingControl {
    #[must_use]
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            shutdown: AtomicBool::new(false),
            blocking: RwLock::new(None),
            is_processing: AtomicBool::new(true),
            pending: AtomicUsize::new(0),
            callback: RwLock::new(None),
            error: Mutex::new(None),
        })
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn blocking(&self) -> Result<bool> {
        self.blocking.read().unwrap().ok_or_else(|| {
            invalid(
                "set_blocking must initialize the C++ unwritten blocking flag",
                "ProcessingControl::blocking",
            )
        })
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn check_error(&self) -> Result<()> {
        if let Some(error) = self.error.lock().unwrap().as_ref() {
            Err(Error::new(error.kind(), error.message())
                .with_status(error.status())
                .with_operation(
                    error
                        .operation()
                        .unwrap_or("ProcessingControl::check_error"),
                ))
        } else {
            Ok(())
        }
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn invoke(&self, frames: &mut CameraMeasurements) -> Result<()> {
        if let Some(callback) = self.callback.read().unwrap().as_ref() {
            callback(frames)?;
        }
        Ok(())
    }
}

/// The standard bounded channel holds at most ten input batches. The blocking
/// mode additionally limits queued input to one; the nonblocking mode drops the oldest.
pub struct CameraWorker {
    pub control: Arc<ProcessingControl>,
    sender: SyncSender<CameraMeasurements>,
    receiver: Arc<Mutex<Receiver<CameraMeasurements>>>,
    thread: Option<JoinHandle<()>>,
}
impl CameraWorker {
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn start(
        control: Arc<ProcessingControl>,
        mut process: impl FnMut(CameraMeasurements) -> Result<()> + Send + 'static,
    ) -> Self {
        let (sender, receiver) = mpsc::sync_channel(10);
        let receiver = Arc::new(Mutex::new(receiver));
        let c = control.clone();
        let r = receiver.clone();
        let thread = thread::spawn(move || {
            while !c.shutdown.load(Ordering::Acquire) {
                let frame = {
                    let receiver = r.lock().unwrap();
                    match receiver.try_recv() {
                        Ok(frame) => {
                            c.is_processing.store(true, Ordering::Release);
                            c.pending.fetch_sub(1, Ordering::AcqRel);
                            Some(frame)
                        }
                        Err(_) => None,
                    }
                };
                if let Some(frame) = frame {
                    let result =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| process(frame)))
                            .unwrap_or_else(|_| {
                                Err(invalid(
                                    "processing callback panicked",
                                    "CameraWorker::process",
                                ))
                            });
                    c.is_processing.store(false, Ordering::Release);
                    if let Err(error) = result {
                        *c.error.lock().unwrap() = Some(error);
                        c.shutdown.store(true, Ordering::Release);
                    }
                } else {
                    thread::sleep(Duration::from_millis(1));
                }
            }
        });
        Self {
            control,
            sender,
            receiver,
            thread: Some(thread),
        }
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn submit(&self, frames: CameraMeasurements) -> Result<bool> {
        self.control.check_error()?;
        let blocking = self.control.blocking()?;
        loop {
            if self.control.shutdown.load(Ordering::Acquire) {
                return Ok(false);
            }
            let receiver = self.receiver.lock().unwrap();
            let count = self.control.pending.load(Ordering::Acquire);
            if blocking && count >= 1 {
                drop(receiver);
                thread::sleep(Duration::from_millis(1));
                continue;
            }
            let dropped = count >= 10;
            if dropped && receiver.try_recv().is_ok() {
                self.control.pending.fetch_sub(1, Ordering::AcqRel);
            }
            self.control.pending.fetch_add(1, Ordering::AcqRel);
            if self.sender.try_send(frames).is_err() {
                self.control.pending.fetch_sub(1, Ordering::AcqRel);
                return Err(invalid(
                    "input channel disconnected",
                    "CameraWorker::submit",
                ));
            }
            return Ok(!dropped);
        }
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn finished(&self) -> Result<bool> {
        self.control.check_error()?;
        Ok(self.control.pending.load(Ordering::Acquire) == 0
            && !self.control.is_processing.load(Ordering::Acquire))
    }
}
impl Drop for CameraWorker {
    fn drop(&mut self) {
        self.control.shutdown.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            *self.control.error.lock().unwrap() =
                Some(invalid("processing thread panicked", "CameraWorker::drop"));
        }
    }
}
/// Standard channel operation implementing the source's drop-oldest policy.
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
///
/// # Panics
/// Panics if a shared model/queue/configuration mutex has been poisoned.
pub fn send_dropping_oldest<T>(
    sender: &SyncSender<T>,
    receiver: &Mutex<Receiver<T>>,
    value: T,
) -> Result<bool> {
    match sender.try_send(value) {
        Ok(()) => Ok(false),
        Err(TrySendError::Full(value)) => {
            let receiver = receiver.lock().unwrap();
            let dropped = receiver.try_recv().is_ok();
            sender
                .try_send(value)
                .map_err(|_| invalid("output channel unavailable", "send_dropping_oldest"))?;
            Ok(dropped)
        }
        Err(TrySendError::Disconnected(_)) => Err(invalid(
            "output channel disconnected",
            "send_dropping_oldest",
        )),
    }
}
