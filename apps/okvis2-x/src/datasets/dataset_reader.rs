/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! EuRoC-style dataset streaming with camera synchronization and IMU/GPS lead-in.
use crate::{
    config::parameters::GpsParameters,
    runtime::vi_interface::{ImageMap, log_image_transform},
    sensors::vi_sensor_base::{
        DatasetReaderBase, GNSS_LEAP_NANOSECONDS, SensorCallbacks, ViSensorBase,
    },
};
use jiff::Timestamp;
use nalgebra::Vector3;
use purecv::core::{DynamicData, DynamicMatrix, Matrix};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{self, BufRead, BufReader},
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread::{self, JoinHandle},
};
#[derive(Clone, Copy, Debug)]
pub enum ImageReadMode {
    Grayscale,
    Colour,
    Unchanged,
}
/// Decode and encode images with the requested OpenCV-compatible pixel layout.
/// Colour decoding returns BGR; unchanged decoding preserves depth and channels.
pub trait ImageCodec: Send + Sync {
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    fn read(&self, path: &Path, mode: ImageReadMode) -> io::Result<DynamicMatrix>;
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    fn write(&self, path: &Path, image: &DynamicMatrix) -> io::Result<()>;
}
fn invalid(s: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, s.into())
}
pub(crate) fn depth_to_metres(mut image: DynamicMatrix) -> DynamicMatrix {
    if let DynamicData::F32(m) = &mut image.data
        && m.channels == 1
    {
        log_image_transform(&mut m.data);
        return image;
    }
    macro_rules! cvt {
        ($m:expr,$convert:expr) => {{
            let m = $m;
            DynamicMatrix {
                data: DynamicData::F32(Matrix::from_vec(
                    m.rows,
                    m.cols,
                    m.channels,
                    m.data.iter().map(|&v| ($convert)(v) * 0.001).collect(),
                )),
            }
        }};
    }
    match &image.data {
        DynamicData::U8(m) => cvt!(m, f32::from),
        DynamicData::I8(m) => cvt!(m, f32::from),
        DynamicData::U16(m) => cvt!(m, f32::from),
        DynamicData::I16(m) => cvt!(m, f32::from),
        DynamicData::I32(m) => cvt!(m, |v: i32| v as f32),
        DynamicData::F32(m) => cvt!(m, f32::from),
        DynamicData::F64(m) => cvt!(m, |v: f64| v as f32),
    }
}
#[derive(Clone, Debug)]
pub struct ImageEntry {
    pub timestamp_ns: u64,
    pub path: PathBuf,
}
pub struct ImageIterators {
    pub cameras: Vec<Vec<ImageEntry>>,
    pub camera_indices: Vec<usize>,
    pub depths: BTreeMap<usize, Vec<ImageEntry>>,
    pub depth_indices: BTreeMap<usize, usize>,
}
impl ImageIterators {
    pub const TOLERANCE_NS: u64 = 10_000_000;
    #[must_use]
    pub fn arg_min_time(&self) -> Option<(usize, u64, bool)> {
        let mut best = None;
        for (i, entries) in self.cameras.iter().enumerate() {
            if let Some(e) = entries.get(self.camera_indices[i])
                && best.is_none_or(|(_, t, _)| e.timestamp_ns < t)
            {
                best = Some((i, e.timestamp_ns, false));
            }
        }
        best
    }
    #[must_use]
    pub fn arg_min_time_group(&self, group: &BTreeSet<usize>) -> Option<(usize, u64, bool)> {
        let mut best = None;
        for &i in group {
            let e = self.cameras.get(i)?.get(*self.camera_indices.get(i)?)?;
            if best.is_none_or(|(_, t, _)| e.timestamp_ns < t) {
                best = Some((i, e.timestamp_ns, false));
            }
        }
        best
    }
    #[must_use]
    pub fn is_synched(&self, group: &BTreeSet<usize>) -> bool {
        let Some((_, minimum, _)) = self.arg_min_time_group(group) else {
            return false;
        };
        group.iter().all(|&i| {
            self.cameras
                .get(i)
                .and_then(|c| c.get(self.camera_indices[i]))
                .is_some_and(|e| e.timestamp_ns - minimum <= Self::TOLERANCE_NS)
        })
    }
}
pub struct DatasetReader {
    pub callbacks: SensorCallbacks,
    path: PathBuf,
    num_cameras: usize,
    sync_cameras: BTreeSet<usize>,
    starting_delay: f64,
    gps_type: Option<String>,
    streaming: Arc<AtomicBool>,
    counter: Arc<AtomicUsize>,
    last_gps: Arc<Mutex<Timestamp>>,
    num_images: usize,
    processing_thread: Option<JoinHandle<io::Result<()>>>,
    codec: Arc<dyn ImageCodec>,
}
impl DatasetReader {
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    pub fn new(
        path: &Path,
        num_cameras: usize,
        sync_cameras: BTreeSet<usize>,
        starting_delay: f64,
        gps: Option<&GpsParameters>,
        codec: Arc<dyn ImageCodec>,
    ) -> io::Result<Self> {
        let gps_type = gps.map(|p| p.r#type.clone());
        if gps_type
            .as_ref()
            .is_some_and(|s| !matches!(s.as_str(), "cartesian" | "geodetic" | "geodetic-leica"))
        {
            return Err(invalid("Unknown GPS data type specified"));
        }
        Ok(Self {
            callbacks: SensorCallbacks::default(),
            path: path.into(),
            num_cameras,
            sync_cameras,
            starting_delay,
            gps_type,
            streaming: Arc::new(AtomicBool::new(false)),
            counter: Arc::new(AtomicUsize::new(0)),
            last_gps: Arc::new(Mutex::new(Timestamp::UNIX_EPOCH)),
            num_images: 0,
            processing_thread: None,
            codec,
        })
    }
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    pub fn read_camera_image_csv(
        &self,
        folder: &str,
        index: usize,
    ) -> io::Result<Option<Vec<ImageEntry>>> {
        let directory = self.path.join(format!("{folder}{index}"));
        let file = match File::open(directory.join("data.csv")) {
            Ok(f) => f,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(None),
            Err(e) => return Err(e),
        };
        let mut rows = BufReader::new(file).lines();
        rows.next().transpose()?;
        let mut entries = Vec::new();
        for line in rows {
            let line = line?;
            let Some((stamp, name)) = line.split_once(',') else {
                break;
            };
            let name = name.strip_prefix(' ').unwrap_or(name);
            let name = name.strip_suffix('\r').unwrap_or(name);
            if name.is_empty() {
                return Err(invalid("empty camera image filename"));
            }
            entries.push(ImageEntry {
                timestamp_ns: stamp
                    .trim()
                    .parse()
                    .map_err(|_| invalid("invalid camera timestamp"))?,
                path: directory.join("data").join(name),
            });
        }
        Ok(Some(entries))
    }
}
impl ViSensorBase for DatasetReader {
    fn callbacks(&self) -> &SensorCallbacks {
        &self.callbacks
    }
    fn callbacks_mut(&mut self) -> &mut SensorCallbacks {
        &mut self.callbacks
    }
    fn start_streaming(&mut self) -> io::Result<bool> {
        if self.processing_thread.is_some() {
            return Err(invalid("dataset worker already exists"));
        }
        if self.callbacks.images_callbacks.is_empty() || self.callbacks.imu_callbacks.is_empty() {
            return Err(invalid("image and IMU callbacks must be registered"));
        }
        if self.gps_type.is_some()
            && self.callbacks.gps_callback.is_none()
            && self.callbacks.geodetic_gps_callback.is_none()
        {
            return Err(invalid("no add GPS callback registered"));
        }
        let imu = read_sensor_csv(&self.path.join("imu0/data.csv"))?;
        let gps = if let Some(kind) = &self.gps_type {
            let path = match kind.as_str() {
                "cartesian" => self.path.join("gps0/data.csv"),
                "geodetic" => self.path.join("gps0/data_raw.csv"),
                _ => PathBuf::from(format!("{}gnss.csv", self.path.display())),
            };
            Some(read_sensor_csv(&path)?)
        } else {
            None
        };
        let mut cameras = Vec::new();
        let mut depths = BTreeMap::new();
        for i in 0..self.num_cameras {
            let mut entries = self.read_camera_image_csv("cam", i)?;
            if entries.as_ref().is_none_or(Vec::is_empty) {
                entries = self.read_camera_image_csv("rgb", i)?;
            }
            let entries = entries
                .filter(|e| !e.is_empty())
                .ok_or_else(|| invalid(format!("no images found for camera {i}")))?;
            if i == 0 {
                self.num_images = entries.len();
            }
            cameras.push(entries);
            if let Some(d) = self
                .read_camera_image_csv("depth", i)?
                .filter(|d| !d.is_empty())
            {
                depths.insert(i, d);
            }
        }
        let iterators = ImageIterators {
            camera_indices: vec![0; cameras.len()],
            depth_indices: depths.keys().map(|&i| (i, 0)).collect(),
            cameras,
            depths,
        };
        let streaming = self.streaming.clone();
        let count = self.counter.clone();
        let callbacks = self.callbacks.clone();
        let codec = self.codec.clone();
        let sync = self.sync_cameras.clone();
        let delay = self.starting_delay;
        let gps_type = self.gps_type.clone();
        let last_gps = self.last_gps.clone();
        self.counter.store(0, Ordering::Relaxed);
        self.streaming.store(true, Ordering::Release);
        self.processing_thread = Some(thread::spawn(move || {
            let result = processing(
                iterators,
                (imu, gps, gps_type, sync),
                delay,
                &callbacks,
                codec.as_ref(),
                (&streaming, &count, &last_gps),
            );
            streaming.store(false, Ordering::Release);
            result
        }));
        Ok(true)
    }
    fn stop_streaming(&mut self) -> io::Result<bool> {
        if let Some(handle) = self.processing_thread.take() {
            let result = handle
                .join()
                .map_err(|_| io::Error::other("dataset processing thread panicked"))?;
            self.streaming.store(false, Ordering::Release);
            result?;
        }
        Ok(true)
    }
    fn is_streaming(&self) -> bool {
        self.streaming.load(Ordering::Acquire)
    }
}
impl DatasetReaderBase for DatasetReader {
    fn set_dataset_path(&mut self, path: &Path) -> bool {
        self.path = path.into();
        true
    }
    fn set_starting_delay(&mut self, seconds: f64) -> bool {
        if self.is_streaming() {
            log::warn!("starting delay ignored, because streaming already started");
            return false;
        }
        self.starting_delay = seconds;
        true
    }
    fn completion(&self) -> f64 {
        if self.is_streaming() {
            self.counter.load(Ordering::Relaxed) as f64 / self.num_images as f64
        } else {
            0.0
        }
    }
}
impl Drop for DatasetReader {
    fn drop(&mut self) {
        if let Err(e) = self.stop_streaming() {
            log::error!("dataset reader shutdown: {e}");
        }
    }
}
fn read_sensor_csv(path: &Path) -> io::Result<Vec<String>> {
    let mut rows = BufReader::new(File::open(path)?).lines();
    rows.next().transpose()?;
    let rows = rows.collect::<io::Result<Vec<_>>>()?;
    if rows.is_empty() {
        return Err(invalid(format!(
            "no measurements present in {}",
            path.display()
        )));
    }
    Ok(rows)
}
fn timestamp(ns: u64) -> io::Result<Timestamp> {
    Timestamp::from_nanosecond(i128::from(ns)).map_err(|e| invalid(e.to_string()))
}
fn fields(line: &str) -> Vec<&str> {
    line.split(',').map(str::trim).collect()
}
fn field<'a>(fields: &[&'a str], i: usize) -> io::Result<&'a str> {
    fields
        .get(i)
        .copied()
        .ok_or_else(|| invalid("missing measurement field"))
}
fn float(fields: &[&str], i: usize) -> io::Result<f64> {
    field(fields, i)?
        .parse()
        .map_err(|_| invalid("invalid floating-point measurement"))
}
fn float32(fields: &[&str], i: usize) -> io::Result<f64> {
    field(fields, i)?
        .parse::<f32>()
        .map(f64::from)
        .map_err(|_| invalid("invalid floating-point measurement"))
}
fn stamp(fields: &[&str]) -> io::Result<Timestamp> {
    let ns = field(fields, 0)?
        .parse::<u64>()
        .map_err(|_| invalid("invalid measurement timestamp"))?;
    timestamp(
        ns.checked_sub(GNSS_LEAP_NANOSECONDS)
            .ok_or_else(|| invalid("GPS timestamp underflow"))?,
    )
}
fn load_image(
    codec: &dyn ImageCodec,
    path: &Path,
    mode: ImageReadMode,
) -> io::Result<Arc<DynamicMatrix>> {
    let image = codec.read(path, mode)?;
    if image.rows() == 0 || image.cols() == 0 {
        return Err(invalid(format!("missing image {}", path.display())));
    }
    Ok(Arc::new(image))
}
fn camera_mode(path: &Path) -> ImageReadMode {
    let directory = path
        .parent()
        .and_then(Path::parent)
        .and_then(Path::file_name)
        .and_then(|s| s.to_str())
        .unwrap_or("");
    if directory.len() > 3 && directory.starts_with("cam") {
        ImageReadMode::Grayscale
    } else {
        ImageReadMode::Colour
    }
}

fn processing(
    mut it: ImageIterators,
    (imu, gps, gps_type, sync): (
        Vec<String>,
        Option<Vec<String>>,
        Option<String>,
        BTreeSet<usize>,
    ),
    delay: f64,
    callbacks: &SensorCallbacks,
    codec: &dyn ImageCodec,
    (streaming, counter, last_gps): (&AtomicBool, &AtomicUsize, &Mutex<Timestamp>),
) -> io::Result<()> {
    let mut start = Timestamp::UNIX_EPOCH;
    let mut imu_index = 0;
    let mut gps_index = 0;
    let mut t_gps = *last_gps
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    while streaming.load(Ordering::Acquire) {
        let mut need_sync = false;
        let (i_min, minimum) = loop {
            let Some((i, t, _)) = it.arg_min_time() else {
                return Ok(());
            };
            if sync.contains(&i) {
                need_sync = !it.is_synched(&sync);
            }
            if it.depths.contains_key(&i) {
                need_sync |= !it.is_synched(&BTreeSet::from([i]));
            }
            if need_sync {
                log::warn!("image at t={t} without correspondence -- dropping");
                it.camera_indices[i] += 1;
                if it.camera_indices[i] >= it.cameras[i].len() {
                    return Ok(());
                }
            } else {
                break (i, t);
            }
        };
        let Some((images, depth_images)) = read_synchronized_images(&mut it, i_min, &sync, codec)?
        else {
            return Ok(());
        };
        let t = timestamp(minimum)?;
        if start == Timestamp::UNIX_EPOCH {
            start = t;
        }
        if !emit_imu_until(&imu, &mut imu_index, (start, t, delay), callbacks)? {
            return Ok(());
        }
        if let Some(gps) = &gps {
            while t_gps <= t {
                let Some(line) = gps.get(gps_index) else {
                    return Ok(());
                };
                gps_index += 1;
                let f = fields(line);
                t_gps = stamp(&f)?;
                *last_gps
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner) = t_gps;
                let deliver =
                    (t_gps.as_nanosecond() - start.as_nanosecond()) as f64 * 1e-9 + 1.0 > delay;
                match gps_type.as_deref() {
                    Some("cartesian") => {
                        let pos = Vector3::new(float32(&f, 1)?, float32(&f, 2)?, float32(&f, 3)?);
                        let err = Vector3::new(float32(&f, 4)?, float32(&f, 5)?, float32(&f, 6)?);
                        if deliver {
                            callbacks
                                .gps_callback
                                .as_ref()
                                .ok_or_else(|| invalid("cartesian GPS callback missing"))?(
                                t_gps, &pos, &err,
                            );
                        }
                    }
                    Some(kind) => {
                        let offset = if kind == "geodetic-leica" { 6 } else { 1 };
                        let lat = float(&f, offset)?;
                        let lon = float(&f, offset + 1)?;
                        let alt = float(&f, offset + 2)?;
                        let error = if kind == "geodetic-leica" { 10 } else { 4 };
                        let h = float(&f, error)?;
                        let v = float(&f, error + 1)?;
                        if deliver {
                            callbacks
                                .geodetic_gps_callback
                                .as_ref()
                                .ok_or_else(|| invalid("geodetic GPS callback missing"))?(
                                t_gps, lat, lon, alt, h, v,
                            );
                        }
                    }
                    None => return Err(invalid("GPS type missing")),
                }
            }
        }
        for cb in &callbacks.images_callbacks {
            cb(t, &images, &depth_images);
        }
        if images.contains_key(&0) {
            counter.fetch_add(1, Ordering::Relaxed);
        }
    }
    Ok(())
}
fn emit_imu_until(
    imu: &[String],
    imu_index: &mut usize,
    (start, t, delay): (Timestamp, Timestamp, f64),
    callbacks: &SensorCallbacks,
) -> io::Result<bool> {
    loop {
        let Some(line) = imu.get(*imu_index) else {
            return Ok(false);
        };
        *imu_index += 1;
        let f = fields(line);
        let t_imu = timestamp(
            field(&f, 0)?
                .parse()
                .map_err(|_| invalid("invalid IMU timestamp"))?,
        )?;
        let gyr = Vector3::new(float32(&f, 1)?, float32(&f, 2)?, float32(&f, 3)?);
        let acc = Vector3::new(float32(&f, 4)?, float32(&f, 5)?, float32(&f, 6)?);
        if (t_imu.as_nanosecond() - start.as_nanosecond()) as f64 * 1e-9 + 1.0 > delay {
            for cb in &callbacks.imu_callbacks {
                cb(t_imu, &acc, &gyr);
            }
        }
        if t_imu.as_nanosecond() > t.as_nanosecond() + 21_000_000 {
            break;
        }
    }
    Ok(true)
}

fn read_synchronized_images(
    it: &mut ImageIterators,
    i_min: usize,
    sync: &BTreeSet<usize>,
    codec: &dyn ImageCodec,
) -> io::Result<Option<(ImageMap, ImageMap)>> {
    let mut images = ImageMap::new();
    let mut depth_images = ImageMap::new();
    let group = if sync.contains(&i_min) {
        sync.clone()
    } else {
        BTreeSet::from([i_min])
    };
    for i in group {
        let e = it
            .cameras
            .get(i)
            .and_then(|v| v.get(it.camera_indices[i]))
            .ok_or_else(|| invalid("camera synchronization index out of range"))?;
        images.insert(i, load_image(codec, &e.path, camera_mode(&e.path))?);
        it.camera_indices[i] += 1;
    }
    if let Some(depths) = it.depths.get(&i_min) {
        let index = it.depth_indices.get_mut(&i_min).unwrap();
        let Some(e) = depths.get(*index) else {
            return Ok(None);
        };
        let image = codec.read(&e.path, ImageReadMode::Unchanged)?;
        if image.rows() == 0 || image.cols() == 0 {
            return Err(invalid("missing depth image"));
        }
        depth_images.insert(i_min, Arc::new(depth_to_metres(image)));
        *index += 1;
        if !sync.contains(&i_min) {
            let e = it.cameras[i_min]
                .get(it.camera_indices[i_min])
                .ok_or_else(|| {
                    invalid(
                        "unsynchronized depth consumes a second camera image beyond the dataset",
                    )
                })?;
            let mode = if e.path.to_string_lossy().ends_with("png") {
                ImageReadMode::Grayscale
            } else {
                ImageReadMode::Colour
            };
            images.insert(i_min, load_image(codec, &e.path, mode)?);
            it.camera_indices[i_min] += 1;
        }
    }
    Ok(Some((images, depth_images)))
}
