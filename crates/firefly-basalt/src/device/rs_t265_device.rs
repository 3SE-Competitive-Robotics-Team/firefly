// Copyright (c) 2019 Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Real SDK lifecycle adapter for `device/rs_t265.h/.cpp`.
//! Raw SDK buffering and stream outputs are bounded. Consumers drain outputs
//! during stop; detach cancels pending sends before recording sessions close.
use super::rs_t265::{
    CameraProfile, MotionIntrinsics, RsImuData, RsPoseData, RsT265Processor, VideoFrame,
    export_calibration,
};
use crate::optical_flow::OpticalFlowInput;
use crossbeam_channel::{Sender, TrySendError};
use firefly_basalt_headers::{calibration::Calibration, imu::imu_types::ImuData};
use firefly_basalt_realsense::{Device, Payload, PayloadReceiver};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

#[derive(Clone, Default)]
pub struct OutputQueues {
    pub image: Option<Sender<Option<Arc<OpticalFlowInput>>>>,
    pub imu: Option<Sender<Option<Arc<ImuData<f64>>>>>,
    pub pose: Option<Sender<RsPoseData>>,
}
#[derive(Default)]
struct QueueState {
    generation: usize,
    outputs: OutputQueues,
}
fn failure(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("basalt.rs_t265_device")
}
pub struct RsT265Device {
    sdk: Device,
    processor: Arc<Mutex<RsT265Processor>>,
    queues: Arc<Mutex<QueueState>>,
    worker: Option<JoinHandle<Result<()>>>,
    calibration: Option<Arc<Calibration<f64>>>,
    manual_exposure: bool,
}
impl RsT265Device {
    /// Creates the real SDK context/configuration. Hardware reset is explicit so
    /// callers can present device-discovery failures before resetting a device.
    /// # Errors
    /// Reports SDK discovery/configuration failures.
    pub fn new(manual_exposure: bool, skip_frames: i32, webp_quality: i32) -> Result<Self> {
        Ok(Self {
            sdk: Device::new(manual_exposure)?,
            processor: Arc::new(Mutex::new(RsT265Processor::new(
                manual_exposure,
                skip_frames,
                webp_quality,
            ))),
            queues: Arc::new(Mutex::new(QueueState::default())),
            worker: None,
            calibration: None,
            manual_exposure,
        })
    }
    /// # Errors
    /// Reports SDK enumeration errors.
    pub fn device_count(&self) -> Result<usize> {
        self.sdk.device_count()
    }
    /// # Errors
    /// Reports missing devices, reset failures or a running pipeline.
    pub fn reset_first_device(&self) -> Result<()> {
        self.sdk.reset_first_device()
    }
    /// # Errors
    /// Reports a poisoned queue lock.
    pub fn set_output_queues(&self, queues: OutputQueues) -> Result<()> {
        let mut state = self
            .queues
            .lock()
            .map_err(|_| failure("Poisoned output queues"))?;
        state.generation = state
            .generation
            .checked_add(1)
            .ok_or_else(|| failure("Output queue generation exhausted"))?;
        state.outputs = queues;
        Ok(())
    }
    /// # Errors
    /// Reports a poisoned queue lock.
    pub fn detach_output_queues(&self) -> Result<()> {
        self.set_output_queues(OutputQueues::default())
    }
    /// # Errors
    /// Reports SDK startup/calibration/exposure errors.
    pub fn start(&mut self, exposure_ms: f64) -> Result<()> {
        if self.worker.is_some() {
            return Ok(());
        }
        if self.manual_exposure {
            self.sdk.set_exposure(Some(exposure_ms))?;
        }
        let rx = self.sdk.start()?;
        let setup = (|| {
            let raw = self.sdk.calibration()?;
            let motion = |v: firefly_basalt_realsense::MotionCalibration| MotionIntrinsics {
                data: v.data,
                noise_variances: v.noise_variances,
                bias_variances: v.bias_variances,
            };
            let cameras = raw.cameras.map(|v| CameraProfile {
                width: v.width,
                height: v.height,
                fx: v.fx,
                fy: v.fy,
                ppx: v.ppx,
                ppy: v.ppy,
                coefficients: v.coefficients,
                rotation: v.rotation,
                translation: v.translation,
            });
            self.calibration = Some(Arc::new(export_calibration(
                &motion(raw.gyro),
                &motion(raw.accel),
                &cameras,
            )));
            Ok(())
        })();
        if let Err(error) = setup {
            let _ = self.sdk.stop();
            return Err(error);
        }
        let queues = Arc::clone(&self.queues);
        let processor = Arc::clone(&self.processor);
        self.worker = Some(std::thread::spawn(move || {
            run_worker(rx, &queues, &processor)
        }));
        Ok(())
    }
    /// # Errors
    /// Reports SDK stop or processing failures. Joins the consumer after SDK stop.
    /// Output consumers must continue draining until EOS, or detach their queues
    /// before stopping. Detach cancels blocked sends without enqueuing afterwards.
    pub fn stop(&mut self) -> Result<()> {
        let sdk_result = self.sdk.stop();
        let worker_result = self.worker.take().map_or(Ok(()), |worker| {
            worker
                .join()
                .map_err(|_| failure("T265 processing worker panicked"))?
        });
        sdk_result.and(worker_result)
    }
    #[must_use]
    pub fn export_calibration(&self) -> Option<Arc<Calibration<f64>>> {
        self.calibration.clone()
    }
    /// A finished processing worker can indicate EOS or a decoding/SDK error.
    /// Call `stop` to join it and retrieve the result.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.worker.as_ref().is_some_and(JoinHandle::is_finished)
    }
    /// # Errors
    /// Reports SDK exposure errors.
    pub fn set_exposure(&self, ms: f64) -> Result<bool> {
        if !self.manual_exposure {
            return Ok(false);
        }
        self.sdk.set_exposure(Some(ms))?;
        Ok(true)
    }
    /// # Errors
    /// Reports a poisoned processor lock.
    pub fn set_skip_frames(&self, skip: i32) -> Result<()> {
        self.processor
            .lock()
            .map_err(|_| failure("Poisoned processor"))?
            .set_skip_frames(skip);
        Ok(())
    }
    /// # Errors
    /// Reports a poisoned processor lock.
    pub fn set_webp_quality(&self, quality: i32) -> Result<()> {
        self.processor
            .lock()
            .map_err(|_| failure("Poisoned processor"))?
            .set_webp_quality(quality);
        Ok(())
    }
    /// # Errors
    /// Reports a poisoned processor lock.
    pub fn last_image_data(&self) -> Result<Option<Arc<OpticalFlowInput>>> {
        Ok(self
            .processor
            .lock()
            .map_err(|_| failure("Poisoned processor"))?
            .last_image_data())
    }
}
impl Drop for RsT265Device {
    fn drop(&mut self) {
        let _ = self.detach_output_queues();
        let _ = self.stop();
    }
}

fn run_worker(
    rx: PayloadReceiver,
    queues: &Mutex<QueueState>,
    processor: &Mutex<RsT265Processor>,
) -> Result<()> {
    let result = (|| {
        for event in rx {
            let event = event?;
            let (generation, outputs) = {
                let state = queues
                    .lock()
                    .map_err(|_| failure("Poisoned output queues"))?;
                (state.generation, state.outputs.clone())
            };
            let mut processor = processor
                .lock()
                .map_err(|_| failure("Poisoned processor"))?;
            match event {
                Payload::Gyro { timestamp_ms, xyz } => processor.gyro(RsImuData {
                    timestamp: timestamp_ms,
                    data: Vector3::from(xyz).map(f64::from),
                }),
                Payload::Accel { timestamp_ms, xyz } => {
                    for sample in processor.accel(RsImuData {
                        timestamp: timestamp_ms,
                        data: Vector3::from(xyz).map(f64::from),
                    })? {
                        if let Some(sender) = &outputs.imu {
                            send_output(queues, generation, sender, Some(Arc::new(sample)))?;
                        }
                    }
                }
                Payload::Stereo(frames) => {
                    let frames = frames.each_ref().map(|v| VideoFrame {
                        timestamp: v.timestamp_ms,
                        exposure_us: v.exposure_us,
                        width: v.width,
                        height: v.height,
                        pixels: &v.pixels,
                    });
                    if let Some(image) = processor.images(frames)?
                        && let Some(sender) = &outputs.image
                    {
                        send_output(queues, generation, sender, Some(image))?;
                    }
                }
                Payload::Pose {
                    timestamp_ms,
                    translation,
                    rotation_xyzw,
                } => {
                    if let Some(pose) =
                        RsT265Processor::pose(timestamp_ms, translation, rotation_xyzw)?
                        && let Some(sender) = &outputs.pose
                    {
                        send_output(queues, generation, sender, pose)?;
                    }
                }
            }
        }
        Ok(())
    })();
    // One EOS pair for both normal stop and a failed processing loop.
    let outputs = queues
        .lock()
        .ok()
        .map(|state| (state.generation, state.outputs.clone()));
    if let Some((generation, outputs)) = outputs {
        if let Some(sender) = &outputs.image {
            send_output(queues, generation, sender, None)?;
        }
        if let Some(sender) = &outputs.imu {
            send_output(queues, generation, sender, None)?;
        }
    }
    result
}

fn send_output<T>(
    queues: &Mutex<QueueState>,
    generation: usize,
    sender: &Sender<T>,
    mut value: T,
) -> Result<()> {
    loop {
        {
            let state = queues
                .lock()
                .map_err(|_| failure("Poisoned output queues"))?;
            if state.generation != generation {
                return Ok(());
            }
            match sender.try_send(value) {
                Ok(()) | Err(TrySendError::Disconnected(_)) => return Ok(()),
                Err(TrySendError::Full(pending)) => value = pending,
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn detach_cancels_full_output_without_late_enqueue() {
        let queues = Arc::new(Mutex::new(QueueState::default()));
        let (sender, receiver) = crossbeam_channel::bounded(1);
        sender.send(1).unwrap();
        let (worker_tx, worker_rx) = std::sync::mpsc::channel();
        let worker_queues = Arc::clone(&queues);
        let worker = std::thread::spawn(move || {
            send_output(&worker_queues, 0, &sender, 2).unwrap();
            worker_tx.send(()).unwrap();
        });
        assert!(
            worker_rx
                .recv_timeout(std::time::Duration::from_millis(10))
                .is_err()
        );
        {
            let mut state = queues.lock().unwrap();
            state.generation += 1;
            state.outputs = OutputQueues::default();
        }
        worker_rx
            .recv_timeout(std::time::Duration::from_secs(1))
            .unwrap();
        worker.join().unwrap();
        assert_eq!(receiver.recv().unwrap(), 1);
        assert!(receiver.try_recv().is_err());
    }
    #[test]
    fn owned_payloads_and_single_eos() {
        let (tx, rx) = crossbeam_channel::bounded(8);
        let (image_tx, image_rx) = crossbeam_channel::bounded(8);
        let (imu_tx, imu_rx) = crossbeam_channel::bounded(8);
        let queues = Mutex::new(QueueState {
            generation: 0,
            outputs: OutputQueues {
                image: Some(image_tx),
                imu: Some(imu_tx),
                pose: None,
            },
        });
        let processor = Mutex::new(RsT265Processor::new(false, 1, 90));
        tx.send(Ok(Payload::Accel {
            timestamp_ms: 1.,
            xyz: [0.; 3],
        }))
        .unwrap();
        tx.send(Ok(Payload::Gyro {
            timestamp_ms: 1.5,
            xyz: [1., 2., 3.],
        }))
        .unwrap();
        tx.send(Ok(Payload::Accel {
            timestamp_ms: 2.,
            xyz: [2.; 3],
        }))
        .unwrap();
        let frame = || firefly_basalt_realsense::VideoFrame {
            timestamp_ms: 2.,
            exposure_us: 1000,
            width: 2,
            height: 1,
            pixels: vec![1, 255],
        };
        tx.send(Ok(Payload::Stereo([frame(), frame()]))).unwrap();
        drop(tx);
        run_worker(rx, &queues, &processor).unwrap();
        let sample = imu_rx.recv().unwrap().unwrap();
        assert_eq!(sample.t_ns, 1_500_000);
        assert_eq!(sample.accel, Vector3::repeat(1.));
        assert!(imu_rx.recv().unwrap().is_none());
        assert!(imu_rx.try_recv().is_err());
        let image = image_rx.recv().unwrap().unwrap();
        assert_eq!(
            image.img_data[0].img.as_ref().unwrap().data(),
            &[256, 65280]
        );
        assert!(image_rx.recv().unwrap().is_none());
        assert!(image_rx.try_recv().is_err());
    }
    #[test]
    fn processing_error_also_closes_outputs() {
        let (tx, rx) = crossbeam_channel::bounded(8);
        let (image_tx, image_rx) = crossbeam_channel::bounded(8);
        let (imu_tx, imu_rx) = crossbeam_channel::bounded(8);
        let queues = Mutex::new(QueueState {
            generation: 0,
            outputs: OutputQueues {
                image: Some(image_tx),
                imu: Some(imu_tx),
                pose: None,
            },
        });
        tx.send(Err(failure("injected SDK failure"))).unwrap();
        assert!(run_worker(rx, &queues, &Mutex::new(RsT265Processor::new(false, 1, 90))).is_err());
        assert!(image_rx.recv().unwrap().is_none());
        assert!(imu_rx.recv().unwrap().is_none());
    }
}
