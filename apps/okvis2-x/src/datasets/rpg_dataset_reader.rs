/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! RPG text dataset playback; each camera consumes IMU data through its timestamp.
use crate::{
    datasets::{
        dataset_reader::{ImageCodec, ImageEntry, ImageReadMode},
        x_dataset_reader::{elapsed, invalid, read_image, required_rows, timestamp},
    },
    runtime::vi_interface::ImageMap,
    sensors::vi_sensor_base::{DatasetReaderBase, SensorCallbacks, ViSensorBase},
};
use jiff::Timestamp;
use nalgebra::Vector3;
use std::{
    io,
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread::{self, JoinHandle},
};
/// RPG stores picosecond fractions, divided by 1000 without decimal padding.
fn parse_time(s: &str) -> io::Result<Timestamp> {
    let (a, b) = s
        .split_once('.')
        .ok_or_else(|| invalid("RPG timestamp has no fraction"))?;
    let sec = a
        .parse::<u64>()
        .map_err(|_| invalid("invalid RPG seconds"))?;
    let ns = b
        .parse::<u64>()
        .map_err(|_| invalid("invalid RPG fraction"))?
        / 1000;
    timestamp(
        sec.checked_mul(1_000_000_000)
            .and_then(|s| s.checked_add(ns))
            .ok_or_else(|| invalid("RPG time overflow"))?,
    )
}
pub struct RpgDatasetReader {
    pub callbacks: SensorCallbacks,
    path: PathBuf,
    delay: f64,
    num_cameras: i32,
    codec: Arc<dyn ImageCodec>,
    streaming: Arc<AtomicBool>,
    counter: Arc<AtomicUsize>,
    num_images: usize,
    worker: Option<JoinHandle<io::Result<()>>>,
}
impl RpgDatasetReader {
    pub fn new(path: &Path, delay: f64, num_cameras: i32, codec: Arc<dyn ImageCodec>) -> Self {
        Self {
            callbacks: SensorCallbacks::default(),
            path: path.into(),
            delay,
            num_cameras,
            codec,
            streaming: Arc::new(AtomicBool::new(false)),
            counter: Arc::new(AtomicUsize::new(0)),
            num_images: 0,
            worker: None,
        }
    }
}
impl ViSensorBase for RpgDatasetReader {
    fn callbacks(&self) -> &SensorCallbacks {
        &self.callbacks
    }
    fn callbacks_mut(&mut self) -> &mut SensorCallbacks {
        &mut self.callbacks
    }
    fn is_streaming(&self) -> bool {
        self.streaming.load(Ordering::Acquire)
    }
    fn start_streaming(&mut self) -> io::Result<bool> {
        if self.worker.is_some() {
            return Err(invalid("worker already exists"));
        }
        if self.callbacks.images_callbacks.is_empty() || self.callbacks.imu_callbacks.is_empty() {
            return Err(invalid("image and IMU callbacks required"));
        }
        let imu = required_rows(&self.path.join("imu.txt"))?;
        let cameras = self.discover_cameras()?;
        self.num_images = cameras
            .first()
            .ok_or_else(|| invalid("no camera data"))?
            .len();
        let cb = self.callbacks.clone();
        let codec = self.codec.clone();
        let running = self.streaming.clone();
        let count = self.counter.clone();
        let delay = self.delay;
        running.store(true, Ordering::Release);
        count.store(0, Ordering::Relaxed);
        self.worker = Some(thread::spawn(move || {
            let result = (|| {
                let mut start = Timestamp::UNIX_EPOCH;
                let mut ii = 0;
                let mut index = 0;
                while running.load(Ordering::Acquire) {
                    for (index_value, rotation) in cameras.iter().enumerate() {
                        let Some(residual) = rotation.get(index) else {
                            return Ok(());
                        };
                        if index_value > 0
                            && residual
                                .timestamp_ns
                                .abs_diff(cameras[index_value - 1][index].timestamp_ns)
                                >= 3_000_000
                        {
                            return Err(invalid(format!(
                                "missing images at {}",
                                residual.path.display()
                            )));
                        }
                    }
                    let mut images = ImageMap::new();
                    let mut timestamp_value = Timestamp::UNIX_EPOCH;
                    for (index_value, rotation) in cameras.iter().enumerate() {
                        let residual = &rotation[index];
                        let image = read_image(codec.as_ref(), residual, ImageReadMode::Grayscale)?;
                        timestamp_value = timestamp(residual.timestamp_ns)?;
                        if start == Timestamp::UNIX_EPOCH {
                            start = timestamp_value;
                        }
                        let first = start;
                        loop {
                            let Some(line) = imu.get(ii) else {
                                return Ok(());
                            };
                            ii += 1;
                            let fields: Vec<_> = line.split(' ').collect();
                            let ti = parse_time(
                                fields
                                    .get(1)
                                    .ok_or_else(|| invalid("missing RPG IMU timestamp"))?,
                            )?;
                            let read_vector = |offset| Self::imu_vector(&fields, offset);
                            let gyr = read_vector(2)?;
                            let acc = read_vector(5)?;
                            if elapsed(ti, first) + 1.0 > delay {
                                for cb in &cb.imu_callbacks {
                                    cb(ti, &acc, &gyr);
                                }
                            }
                            if ti > timestamp_value {
                                break;
                            }
                        }
                        if elapsed(timestamp_value, first) > delay {
                            images.insert(index_value, image);
                        }
                    }
                    if images.len() == cameras.len() {
                        for cb in &cb.images_callbacks {
                            cb(timestamp_value, &images, &ImageMap::new());
                        }
                        count.fetch_add(1, Ordering::Relaxed);
                    }
                    index += 1;
                }
                Ok(())
            })();
            running.store(false, Ordering::Release);
            result
        }));
        Ok(true)
    }

    fn stop_streaming(&mut self) -> io::Result<bool> {
        if let Some(w) = self.worker.take() {
            let r = w
                .join()
                .map_err(|_| io::Error::other("RPG worker panicked"))?;
            self.streaming.store(false, Ordering::Release);
            r?;
        }
        Ok(true)
    }
}
impl DatasetReaderBase for RpgDatasetReader {
    fn set_dataset_path(&mut self, p: &Path) -> bool {
        self.path = p.into();
        true
    }
    fn set_starting_delay(&mut self, s: f64) -> bool {
        if self.is_streaming() {
            log::warn!("starting delay ignored, because streaming already started");
            false
        } else {
            self.delay = s;
            true
        }
    }
    fn completion(&self) -> f64 {
        if self.is_streaming() {
            self.counter.load(Ordering::Relaxed) as f64 / self.num_images as f64
        } else {
            0.0
        }
    }
}
impl Drop for RpgDatasetReader {
    fn drop(&mut self) {
        if let Err(e) = self.stop_streaming() {
            log::error!("RPG shutdown: {e}");
        }
    }
}

impl RpgDatasetReader {
    fn discover_cameras(&self) -> io::Result<Vec<Vec<ImageEntry>>> {
        let mut cameras = Vec::new();
        let count_value = if self.num_cameras == -1 {
            2
        } else {
            usize::try_from(self.num_cameras).map_err(|_| invalid("invalid camera count"))?
        };
        for index_value in 0..count_value {
            let path = self.path.join(if index_value == 0 {
                "left_images.txt"
            } else {
                "right_images.txt"
            });
            let entries = match required_rows(&path) {
                Ok(velocity) => velocity,
                Err(residual) if index_value > 0 && residual.kind() == io::ErrorKind::NotFound => {
                    break;
                }
                Err(residual) => return Err(residual),
            };
            cameras.push(
                entries
                    .into_iter()
                    .map(|line| {
                        let fields: Vec<_> = line.split(' ').collect();
                        let timestamp_value = parse_time(
                            fields
                                .get(1)
                                .ok_or_else(|| invalid("missing RPG image timestamp"))?,
                        )?;
                        Ok(ImageEntry {
                            timestamp_ns: u64::try_from(timestamp_value.as_nanosecond())
                                .map_err(|_| invalid("negative RPG timestamp"))?,
                            path: self.path.join(
                                fields
                                    .get(2)
                                    .ok_or_else(|| invalid("missing RPG image name"))?,
                            ),
                        })
                    })
                    .collect::<io::Result<Vec<_>>>()?,
            );
        }
        Ok(cameras)
    }
}
impl RpgDatasetReader {
    fn imu_vector(fields: &[&str], offset: usize) -> io::Result<Vector3<f64>> {
        let mut out = Vector3::zeros();
        for column in 0..3 {
            out[column] = f64::from(
                fields
                    .get(offset + column)
                    .ok_or_else(|| invalid("missing RPG IMU field"))?
                    .parse::<f32>()
                    .map_err(|_| invalid("invalid RPG IMU field"))?,
            );
        }
        Ok(out)
    }
}
