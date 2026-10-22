/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Leica five camera playback with per-camera LiDAR/IMU/GNSS lead-in.
use crate::{
    config::parameters::GpsParameters,
    datasets::{
        dataset_reader::{ImageCodec, ImageEntry, ImageReadMode},
        x_dataset_reader::{
            LidarCallback, elapsed, emit_gps, emit_imu, emit_lidar, gps_rows, invalid, read_image,
            required_rows, synchronize, timestamp,
        },
    },
    runtime::vi_interface::ImageMap,
    sensors::vi_sensor_base::{DatasetReaderBase, SensorCallbacks, ViSensorBase},
};
use jiff::Timestamp;
use std::{
    fs, io,
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread::{self, JoinHandle},
};
pub struct LeicaDatasetReader {
    pub callbacks: SensorCallbacks,
    pub lidar_callback: Option<LidarCallback>,
    path: PathBuf,
    delay: f64,
    enabled: [bool; 5],
    cam_type: String,
    gps_type: Option<String>,
    codec: Arc<dyn ImageCodec>,
    streaming: Arc<AtomicBool>,
    counter: Arc<AtomicUsize>,
    num_images: usize,
    worker: Option<JoinHandle<io::Result<()>>>,
}
impl LeicaDatasetReader {
    /// Enabled camera order is bottom, front, left, right, top; indices are compacted.
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    pub fn new(
        path: &Path,
        delay: f64,
        enabled: [bool; 5],
        cam_type: &str,
        gps: Option<&GpsParameters>,
        codec: Arc<dyn ImageCodec>,
    ) -> io::Result<Self> {
        let gps_type = gps.map(|p| p.r#type.clone());
        if gps_type
            .as_deref()
            .is_some_and(|k| !matches!(k, "cartesian" | "geodetic" | "geodetic-leica"))
        {
            return Err(invalid("unknown GPS data type"));
        }
        Ok(Self {
            callbacks: SensorCallbacks::default(),
            lidar_callback: None,
            path: path.into(),
            delay,
            enabled,
            cam_type: cam_type.into(),
            gps_type,
            codec,
            streaming: Arc::new(AtomicBool::new(false)),
            counter: Arc::new(AtomicUsize::new(0)),
            num_images: 0,
            worker: None,
        })
    }
    pub fn set_lidar_callback(&mut self, c: LidarCallback) {
        self.lidar_callback = Some(c);
    }
}
impl ViSensorBase for LeicaDatasetReader {
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
        if self.gps_type.is_some()
            && self.callbacks.gps_callback.is_none()
            && self.callbacks.geodetic_gps_callback.is_none()
        {
            return Err(invalid("GPS callback required"));
        }
        let lidar = required_rows(&self.path.join("lidar.csv"))?;
        let imu = required_rows(&self.path.join("imu_bottom.csv"))?;
        let gps = gps_rows(&self.path, self.gps_type.as_deref())?;
        let cameras = self.discover_cameras()?;
        self.num_images = cameras
            .first()
            .ok_or_else(|| invalid("no Leica camera selected"))?
            .len();
        let cb = self.callbacks.clone();
        let lcb = self.lidar_callback.clone();
        let codec = self.codec.clone();
        let running = self.streaming.clone();
        let count = self.counter.clone();
        let delay = self.delay;
        let kind = self.gps_type.clone();
        running.store(true, Ordering::Release);
        count.store(0, Ordering::Relaxed);
        self.worker = Some(thread::spawn(move || {
            let result = (|| {
                let mut indices = vec![0; cameras.len()];
                let (mut ii, mut li, mut gi) = (0, 0, 0);
                let mut start = Timestamp::UNIX_EPOCH;
                let mut tg = Timestamp::UNIX_EPOCH;
                while running.load(Ordering::Acquire)
                    && synchronize(&cameras, &mut indices, 10_000_000)?
                {
                    let mut images = ImageMap::new();
                    let mut t = Timestamp::UNIX_EPOCH;
                    for (i, c) in cameras.iter().enumerate() {
                        let e = &c[indices[i]];
                        let image = read_image(codec.as_ref(), e, ImageReadMode::Grayscale)?;
                        t = timestamp(e.timestamp_ns)?;
                        if start == Timestamp::UNIX_EPOCH {
                            start = t;
                        }
                        let first = start;
                        loop {
                            let Some(line) = lidar.get(li) else {
                                return Ok(());
                            };
                            li += 1;
                            if emit_lidar(line, first, delay, lcb.as_ref())? > t {
                                break;
                            }
                        }
                        loop {
                            let Some(line) = imu.get(ii) else {
                                return Ok(());
                            };
                            ii += 1;
                            if emit_imu(line, true, first, delay, &cb)? > t {
                                break;
                            }
                        }
                        if let Some(gps) = &gps {
                            while tg <= t {
                                let Some(line) = gps.get(gi) else {
                                    return Ok(());
                                };
                                gi += 1;
                                tg = emit_gps(
                                    line,
                                    kind.as_deref().ok_or_else(|| invalid("missing GPS type"))?,
                                    first,
                                    delay,
                                    &cb,
                                )?;
                            }
                        }
                        if elapsed(t, first) > delay {
                            images.insert(i, image);
                        }
                        indices[i] += 1;
                    }
                    if images.len() == cameras.len() {
                        for cb in &cb.images_callbacks {
                            cb(t, &images, &ImageMap::new());
                        }
                        count.fetch_add(1, Ordering::Relaxed);
                    }
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
                .map_err(|_| io::Error::other("Leica worker panicked"))?;
            self.streaming.store(false, Ordering::Release);
            r?;
        }
        Ok(true)
    }
}
impl DatasetReaderBase for LeicaDatasetReader {
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
impl Drop for LeicaDatasetReader {
    fn drop(&mut self) {
        if let Err(e) = self.stop_streaming() {
            log::error!("Leica shutdown: {e}");
        }
    }
}

impl LeicaDatasetReader {
    fn discover_cameras(&self) -> io::Result<Vec<Vec<ImageEntry>>> {
        let folder = match self.cam_type.as_str() {
            "pinhole" => "pinhole",
            "eucm" => "imgs",
            _ => return Err(invalid("unknown Leica camera type")),
        };
        let directory = self.path.join(folder);
        let mut all: [Vec<ImageEntry>; 5] = Default::default();
        for entry in fs::read_dir(directory)? {
            let entry = entry?;
            let path = entry.path();
            let stem = path
                .file_stem()
                .and_then(|s| s.to_str())
                .ok_or_else(|| invalid("invalid image filename"))?;
            let mut f = stem.split('_');
            let name = f.next().ok_or_else(|| invalid("missing camera name"))?;
            let stamp = f
                .next()
                .ok_or_else(|| invalid("missing image timestamp"))?
                .parse()
                .map_err(|_| invalid("invalid image timestamp"))?;
            for (i, n) in ["bottom", "front", "left", "right", "top"]
                .iter()
                .enumerate()
            {
                if self.enabled[i] && name.eq_ignore_ascii_case(n) {
                    all[i].push(ImageEntry {
                        timestamp_ns: stamp,
                        path: path.clone(),
                    });
                }
            }
        }
        let mut cameras = Vec::new();
        for (i, mut c) in all.into_iter().enumerate() {
            if self.enabled[i] {
                c.sort_by(|a, b| {
                    a.timestamp_ns
                        .cmp(&b.timestamp_ns)
                        .then(a.path.cmp(&b.path))
                });
                cameras.push(c);
            }
        }
        Ok(cameras)
    }
}
