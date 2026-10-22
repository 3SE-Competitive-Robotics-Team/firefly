/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Extended dataset reader with independently timestamped depth, RGB, `LiDAR` and GNSS.
use crate::{
    config::parameters::ViParameters,
    datasets::dataset_reader::{ImageCodec, ImageEntry, ImageReadMode},
    runtime::vi_interface::ImageMap,
    sensors::{
        measurements::{CameraData, CameraMeasurement},
        vi_sensor_base::{DatasetReaderBase, GNSS_LEAP_NANOSECONDS, SensorCallbacks, ViSensorBase},
    },
};
use jiff::Timestamp;
use nalgebra::Vector3;
use purecv::core::{DynamicData, DynamicMatrix, Matrix};
use std::{
    collections::BTreeMap,
    fs::File,
    io::{self, BufRead, BufReader},
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread::{self, JoinHandle},
};
pub type LidarCallback = Arc<dyn Fn(Timestamp, &Vector3<f64>) -> bool + Send + Sync>;
pub type TimedImages = BTreeMap<usize, (Timestamp, Arc<DynamicMatrix>)>;
pub type NetworkImagesCallback = Arc<dyn Fn(&TimedImages, &TimedImages) -> bool + Send + Sync>;
pub(crate) fn invalid(s: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, s.into())
}
pub(crate) fn timestamp(ns: u64) -> io::Result<Timestamp> {
    Timestamp::from_nanosecond(i128::from(ns)).map_err(|e| invalid(e.to_string()))
}
pub(crate) fn rows(path: &Path) -> io::Result<Vec<String>> {
    let mut r = BufReader::new(File::open(path)?).lines();
    r.next().transpose()?;
    r.collect()
}
pub(crate) fn required_rows(path: &Path) -> io::Result<Vec<String>> {
    let r = rows(path)?;
    if r.is_empty() {
        Err(invalid(format!("no measurements in {}", path.display())))
    } else {
        Ok(r)
    }
}
pub(crate) fn csv_images(path: &Path, tail: bool) -> io::Result<Vec<ImageEntry>> {
    rows(&path.join("data.csv"))?
        .into_iter()
        .map(|s| {
            let (a, b) = s
                .split_once(',')
                .ok_or_else(|| invalid("invalid image CSV"))?;
            let b = if tail {
                b
            } else {
                b.split(',').next().unwrap_or("")
            };
            let b = if b.ends_with('\r') || b.ends_with(',') {
                &b[..b.len() - 1]
            } else {
                b
            };
            if b.is_empty() {
                return Err(invalid("empty image filename"));
            }
            Ok(ImageEntry {
                timestamp_ns: a.parse().map_err(|_| invalid("invalid image timestamp"))?,
                path: path.join("data").join(b),
            })
        })
        .collect()
}
pub(crate) fn read_image(
    codec: &dyn ImageCodec,
    e: &ImageEntry,
    mode: ImageReadMode,
) -> io::Result<Arc<DynamicMatrix>> {
    let image = codec.read(&e.path, mode)?;
    if image.rows() == 0 || image.cols() == 0 {
        return Err(invalid(format!("missing image {}", e.path.display())));
    }
    Ok(Arc::new(image))
}
pub(crate) fn floats(s: &str, count: usize) -> io::Result<Vec<f64>> {
    s.split(',')
        .take(count)
        .map(|v| {
            v.trim()
                .parse()
                .map_err(|_| invalid("invalid sensor field"))
        })
        .collect()
}
pub(crate) fn ns_field(s: &str) -> io::Result<u64> {
    s.split(',')
        .next()
        .ok_or_else(|| invalid("missing timestamp"))?
        .trim()
        .parse()
        .map_err(|_| invalid("invalid timestamp"))
}
pub(crate) fn vector(f: &[f64], i: usize) -> io::Result<Vector3<f64>> {
    if f.len() < i + 3 {
        return Err(invalid("missing vector fields"));
    }
    Ok(Vector3::new(f[i], f[i + 1], f[i + 2]))
}
pub(crate) fn elapsed(a: Timestamp, b: Timestamp) -> f64 {
    (a.as_nanosecond() - b.as_nanosecond()) as f64 * 1e-9
}
pub(crate) fn emit_imu(
    line: &str,
    acc_first: bool,
    start: Timestamp,
    delay: f64,
    cb: &SensorCallbacks,
) -> io::Result<Timestamp> {
    let timestamp_value = timestamp(ns_field(line)?)?;
    let fields: Vec<_> = line.split(',').collect();
    let read_vector = |offset: usize| -> io::Result<Vector3<f64>> {
        let mut out = Vector3::zeros();
        for column in 0..3 {
            out[column] = f64::from(
                fields
                    .get(offset + column)
                    .ok_or_else(|| invalid("missing IMU field"))?
                    .trim()
                    .parse::<f32>()
                    .map_err(|_| invalid("invalid IMU field"))?,
            );
        }
        Ok(out)
    };
    let (accelerometer_offset, gyroscope_offset) = if acc_first { (1, 4) } else { (4, 1) };
    let acc = read_vector(accelerometer_offset)?;
    let gyr = read_vector(gyroscope_offset)?;
    if elapsed(timestamp_value, start) + 1.0 > delay {
        for callback in &cb.imu_callbacks {
            callback(timestamp_value, &acc, &gyr);
        }
    }
    Ok(timestamp_value)
}
pub(crate) fn emit_lidar(
    line: &str,
    start: Timestamp,
    delay: f64,
    cb: Option<&LidarCallback>,
) -> io::Result<Timestamp> {
    let timestamp_value = timestamp(ns_field(line)?)?;
    let point = vector(&floats(line, 4)?, 1)?;
    if elapsed(timestamp_value, start) + 1.0 > delay {
        cb.as_ref()
            .ok_or_else(|| invalid("LiDAR callback missing"))?(timestamp_value, &point);
    }
    Ok(timestamp_value)
}
pub(crate) fn emit_gps(
    line: &str,
    kind: &str,
    start: Timestamp,
    delay: f64,
    cb: &SensorCallbacks,
) -> io::Result<Timestamp> {
    let timestamp_value = timestamp(
        ns_field(line)?
            .checked_sub(GNSS_LEAP_NANOSECONDS)
            .ok_or_else(|| invalid("GPS timestamp underflow"))?,
    )?;
    let fields: Vec<&str> = line.split(',').collect();
    let count = |index: usize| -> io::Result<f64> {
        fields
            .get(index)
            .ok_or_else(|| invalid("GPS field missing"))?
            .trim()
            .parse()
            .map_err(|_| invalid("invalid GPS field"))
    };
    if elapsed(timestamp_value, start) + 1.0 > delay {
        if kind == "cartesian" {
            let n32 = |index: usize| -> io::Result<f64> {
                fields
                    .get(index)
                    .ok_or_else(|| invalid("GPS field missing"))?
                    .trim()
                    .parse::<f32>()
                    .map(f64::from)
                    .map_err(|_| invalid("invalid GPS field"))
            };
            let point = Vector3::new(n32(1)?, n32(2)?, n32(3)?);
            let residual = Vector3::new(n32(4)?, n32(5)?, n32(6)?);
            cb.gps_callback
                .as_ref()
                .ok_or_else(|| invalid("Cartesian GPS callback missing"))?(
                timestamp_value,
                &point,
                &residual,
            );
        } else {
            let (occupancy, residual) = if kind == "geodetic-leica" {
                (6, 10)
            } else {
                (1, 4)
            };
            cb.geodetic_gps_callback
                .as_ref()
                .ok_or_else(|| invalid("geodetic GPS callback missing"))?(
                timestamp_value,
                count(occupancy)?,
                count(occupancy + 1)?,
                count(occupancy + 2)?,
                count(residual)?,
                count(residual + 1)?,
            );
        }
    }
    Ok(timestamp_value)
}
pub(crate) fn gps_rows(path: &Path, kind: Option<&str>) -> io::Result<Option<Vec<String>>> {
    kind.map(|k| {
        required_rows(&match k {
            "cartesian" => path.join("gps0/data.csv"),
            "geodetic" => path.join("gps0/data_raw.csv"),
            "geodetic-leica" => PathBuf::from(format!("{}gnss.csv", path.display())),
            _ => return Err(invalid("unknown GPS data type")),
        })
    })
    .transpose()
}
pub(crate) fn synchronize(
    cameras: &[Vec<ImageEntry>],
    indices: &mut [usize],
    tolerance: u64,
) -> io::Result<bool> {
    loop {
        let mut maximum = 0;
        for (c, &i) in cameras.iter().zip(indices.iter()) {
            let Some(e) = c.get(i) else { return Ok(false) };
            maximum = maximum.max(e.timestamp_ns);
        }
        let mut synched = true;
        for (c, i) in cameras.iter().zip(indices.iter_mut()) {
            if c[*i].timestamp_ns < maximum.wrapping_sub(tolerance) {
                *i += 1;
                synched = false;
            }
        }
        if synched {
            let t = cameras[0][indices[0]].timestamp_ns;
            for (c, &i) in cameras.iter().zip(indices.iter()).skip(1) {
                if c[i].timestamp_ns.abs_diff(t) >= tolerance {
                    return Err(invalid("camera timestamp mismatch"));
                }
            }
            return Ok(true);
        }
    }
}
pub(crate) fn to_f32(image: &DynamicMatrix) -> DynamicMatrix {
    macro_rules! c {
        ($m:expr,$convert:expr) => {{
            let m = $m;
            DynamicMatrix {
                data: DynamicData::F32(Matrix::from_vec(
                    m.rows,
                    m.cols,
                    m.channels,
                    m.data.iter().copied().map($convert).collect(),
                )),
            }
        }};
    }
    match &image.data {
        DynamicData::U8(m) => c!(m, f32::from),
        DynamicData::I8(m) => c!(m, f32::from),
        DynamicData::U16(m) => c!(m, f32::from),
        DynamicData::I16(m) => c!(m, f32::from),
        DynamicData::I32(m) => c!(m, |v: i32| v as f32),
        DynamicData::F32(m) => c!(m, f32::from),
        DynamicData::F64(m) => c!(m, |v: f64| v as f32),
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct ImageStreams {
    pub use_network: bool,
    pub use_depth: bool,
    pub use_rgb: bool,
}
pub struct XDatasetReader {
    pub callbacks: SensorCallbacks,
    pub lidar_callback: Option<LidarCallback>,
    pub images_network_callback: Option<NetworkImagesCallback>,
    path: PathBuf,
    delay: f64,
    num_cameras: usize,
    imu: bool,
    gps_type: Option<String>,
    use_lidar: bool,
    pub images: ImageStreams,

    depth_id: Option<usize>,
    rgb_id: Option<usize>,
    codec: Arc<dyn ImageCodec>,
    streaming: Arc<AtomicBool>,
    counter: Arc<AtomicUsize>,
    num_images: usize,
    worker: Option<JoinHandle<io::Result<()>>>,
}
impl XDatasetReader {
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    pub fn new(
        path: &Path,
        delay: f64,
        p: &ViParameters,
        use_lidar: bool,
        (use_network, use_depth, use_rgb): (bool, bool, bool),
        codec: Arc<dyn ImageCodec>,
    ) -> io::Result<Self> {
        let mut depth_id = None;
        let mut rgb_id = None;
        for i in 0..p.n_camera_system.num_cameras() {
            if p.n_camera_system.is_depth_camera(i) {
                depth_id = Some(i);
            }
            if p.n_camera_system.camera_type(i).is_colour {
                rgb_id = Some(i);
            }
        }
        let gps_type = p.gps.as_ref().map(|p| p.r#type.clone());
        if gps_type
            .as_deref()
            .is_some_and(|k| !matches!(k, "cartesian" | "geodetic" | "geodetic-leica"))
        {
            return Err(invalid("unknown GPS data type"));
        }
        if use_depth && depth_id.is_none() {
            return Err(invalid("depth camera is not configured"));
        }
        if use_rgb && rgb_id.is_none() {
            return Err(invalid("RGB camera is not configured"));
        }
        Ok(Self {
            callbacks: SensorCallbacks::default(),
            lidar_callback: None,
            images_network_callback: None,
            path: path.into(),
            delay,
            num_cameras: p.n_camera_system.num_cameras(),
            imu: p.imu.r#use,
            gps_type,
            use_lidar,
            images: crate::datasets::x_dataset_reader::ImageStreams {
                use_network,
                use_depth,
                use_rgb,
            },

            depth_id,
            rgb_id,
            codec,
            streaming: Arc::new(AtomicBool::new(false)),
            counter: Arc::new(AtomicUsize::new(0)),
            num_images: 0,
            worker: None,
        })
    }
    pub fn set_lidar_callback(&mut self, cb: LidarCallback) {
        self.lidar_callback = Some(cb);
    }
    pub fn set_images_network_callback(&mut self, cb: NetworkImagesCallback) {
        self.images_network_callback = Some(cb);
    }
}
impl ViSensorBase for XDatasetReader {
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
        let PlaybackFiles {
            imu,
            lidar,
            gps,
            depths,
            cameras,
        } = self.load_playback_files()?;
        let cb = self.callbacks.clone();
        let lcb = self.lidar_callback.clone();
        let ncb = self.images_network_callback.clone();
        let codec = self.codec.clone();
        let dataset_path = self.path.clone();
        let running = self.streaming.clone();
        let counter = self.counter.clone();
        let (delay, depth_id, rgb_id, use_depth, use_rgb, network, imu_on, lidar_on, kind) = (
            self.delay,
            self.depth_id,
            self.rgb_id,
            self.images.use_depth,
            self.images.use_rgb,
            self.images.use_network,
            self.imu,
            self.use_lidar,
            self.gps_type.clone(),
        );
        self.counter.store(0, Ordering::Relaxed);
        running.store(true, Ordering::Release);
        self.worker = Some(thread::spawn(move || {
            let result = XPlayback {
                cameras,
                imu,
                lidar,
                gps,
                depths,
                cb,
                lcb,
                ncb,
                codec,
                dataset_path,
                running: &running,
                counter: &counter,
                delay,
                depth_id,
                rgb_id,
                images: ImageStreams {
                    use_network: network,
                    use_depth,
                    use_rgb,
                },
                imu_on,
                lidar_on,
                kind,
            }
            .run();
            running.store(false, Ordering::Release);
            result
        }));
        Ok(true)
    }

    fn stop_streaming(&mut self) -> io::Result<bool> {
        if let Some(w) = self.worker.take() {
            let r = w
                .join()
                .map_err(|_| io::Error::other("dataset worker panicked"))?;
            self.streaming.store(false, Ordering::Release);
            r?;
        }
        Ok(true)
    }
}
impl DatasetReaderBase for XDatasetReader {
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
impl Drop for XDatasetReader {
    fn drop(&mut self) {
        if let Err(e) = self.stop_streaming() {
            log::error!("dataset shutdown: {e}");
        }
    }
}

struct PlaybackFiles {
    imu: Vec<String>,
    lidar: Vec<String>,
    gps: Option<Vec<String>>,
    depths: Vec<ImageEntry>,
    cameras: Vec<Vec<ImageEntry>>,
}
impl XDatasetReader {
    fn load_playback_files(&mut self) -> io::Result<PlaybackFiles> {
        if self.worker.is_some() {
            return Err(invalid("worker already exists"));
        }
        if self.callbacks.images_callbacks.is_empty()
            || (self.imu && self.callbacks.imu_callbacks.is_empty())
        {
            return Err(invalid("image/IMU callback missing"));
        }
        if self.gps_type.is_some()
            && self.callbacks.gps_callback.is_none()
            && self.callbacks.geodetic_gps_callback.is_none()
        {
            return Err(invalid("GPS callback missing"));
        }
        if self.use_lidar && self.lidar_callback.is_none() {
            return Err(invalid("LiDAR callback missing"));
        }
        if self.images.use_network && self.images_network_callback.is_none() {
            return Err(invalid("network callback missing"));
        }
        let imu = if self.imu {
            required_rows(&self.path.join("imu0/data.csv"))?
        } else {
            Vec::new()
        };
        let lidar = if self.use_lidar {
            rows(&self.path.join("lidar0/data.csv"))?
        } else {
            Vec::new()
        };
        let gps = gps_rows(&self.path, self.gps_type.as_deref())?;
        let depths = if self.images.use_depth {
            csv_images(&self.path.join("depth0"), true)?
        } else {
            Vec::new()
        };
        let mut cameras = Vec::new();
        for i in 0..self.num_cameras {
            match csv_images(&self.path.join(format!("cam{i}")), false) {
                Ok(c) => cameras.push(c),
                Err(e) if i > 0 && e.kind() == io::ErrorKind::NotFound => break,
                Err(e) => return Err(e),
            }
        }
        if self.images.use_rgb {
            cameras.push(csv_images(
                &self.path.join(format!("rgb{}", self.rgb_id.unwrap())),
                true,
            )?);
        }
        self.num_images = cameras
            .first()
            .ok_or_else(|| invalid("no camera data"))?
            .len();
        Ok(PlaybackFiles {
            imu,
            lidar,
            gps,
            depths,
            cameras,
        })
    }
}

impl XDatasetReader {
    fn read_playback_image(
        codec: &dyn ImageCodec,
        entry: &ImageEntry,
        dataset_path: &Path,
        (i, camera_count): (usize, usize),
        (use_rgb, rgb_id): (bool, Option<usize>),
    ) -> io::Result<Arc<DynamicMatrix>> {
        let rgb = use_rgb && Some(i) == rgb_id;
        let listed_folder = if use_rgb && i + 1 == camera_count {
            format!("rgb{}", rgb_id.unwrap())
        } else {
            format!("cam{i}")
        };
        let filename = entry
            .path
            .strip_prefix(dataset_path.join(listed_folder).join("data"))
            .map_err(|_| invalid("image path is outside its listed camera directory"))?;
        let image_folder = if rgb {
            format!("rgb{i}")
        } else {
            format!("cam{i}")
        };
        let source_entry = ImageEntry {
            timestamp_ns: entry.timestamp_ns,
            path: dataset_path.join(image_folder).join("data").join(filename),
        };
        let mut image = read_image(
            codec,
            &source_entry,
            if rgb {
                ImageReadMode::Colour
            } else {
                ImageReadMode::Grayscale
            },
        )?;
        if rgb {
            let mut im = (*image).clone();
            if let DynamicData::U8(m) = &mut im.data {
                if m.channels != 3 {
                    return Err(invalid("RGB image requires three channels"));
                }
                for px in m.data.as_chunks_mut::<3>().0 {
                    px.swap(0, 2);
                }
            } else {
                return Err(invalid("RGB image requires u8"));
            }
            image = Arc::new(im);
        }
        Ok(image)
    }
}

impl XDatasetReader {
    fn deliver_depth_images(
        (depths, di): (&[ImageEntry], &mut usize),
        (t, first, delay): (Timestamp, Timestamp, f64),
        (use_rgb, rgb_id, depth_id): (bool, Option<usize>, Option<usize>),
        image: &Arc<DynamicMatrix>,
        (depth_images, timed_depth): (&mut ImageMap, &mut TimedImages),
        codec: &dyn ImageCodec,
        cb: &SensorCallbacks,
    ) -> io::Result<bool> {
        loop {
            let Some(d) = depths.get(*di) else {
                return Ok(false);
            };
            let td = timestamp(d.timestamp_ns)?;
            if td > t {
                break;
            }
            let depth = read_image(codec, d, ImageReadMode::Unchanged)?;
            if elapsed(td, first) + 1.0 > delay {
                let depth = Arc::new(to_f32(&depth));
                let id = depth_id.ok_or_else(|| invalid("missing depth camera"))?;
                let mut data = BTreeMap::new();
                let reading = CameraData {
                    depth_image: Some(depth.clone()),
                    ..CameraData::default()
                };
                data.insert(id, vec![CameraMeasurement::new(td, reading, -1)]);
                depth_images.entry(id).or_insert(depth.clone());
                timed_depth.entry(id).or_insert((td, depth));
                if use_rgb {
                    let reading = CameraData {
                        image: Some(image.clone()),
                        ..CameraData::default()
                    };
                    data.insert(
                        rgb_id.unwrap(),
                        vec![CameraMeasurement::new(t, reading, -1)],
                    );
                }
                if let Some(c) = &cb.depth_callback {
                    c(&mut data);
                }
            }
            *di += 1;
        }

        Ok(true)
    }
}

impl XDatasetReader {
    fn deliver_motion_sensors(
        (imu, ii, imu_on): (&[String], &mut usize, bool),
        (gps, gi, t_gps, kind): (Option<&[String]>, &mut usize, &mut Timestamp, Option<&str>),
        (first, t, delay): (Timestamp, Timestamp, f64),
        cb: &SensorCallbacks,
    ) -> io::Result<bool> {
        if imu_on {
            loop {
                let Some(line) = imu.get(*ii) else {
                    return Ok(false);
                };
                *ii += 1;
                if emit_imu(line, false, first, delay, cb)? > t {
                    break;
                }
            }
        }
        if let Some(gps) = &gps {
            while *t_gps <= t {
                let Some(line) = gps.get(*gi) else {
                    return Ok(false);
                };
                *gi += 1;
                *t_gps = emit_gps(
                    line,
                    kind.ok_or_else(|| invalid("missing GPS type"))?,
                    first,
                    delay,
                    cb,
                )?;
            }
        }
        Ok(true)
    }
}

impl XDatasetReader {
    fn deliver_lidar(
        lidar: &[String],
        li: &mut usize,
        lidar_on: bool,
        (first, t, delay): (Timestamp, Timestamp, f64),
        lcb: Option<&LidarCallback>,
    ) -> io::Result<bool> {
        if lidar_on {
            loop {
                let Some(line) = lidar.get(*li) else {
                    return Ok(false);
                };
                *li += 1;
                if emit_lidar(line, first, delay, lcb)? > t {
                    break;
                }
            }
        }
        Ok(true)
    }
}

struct XPlayback<'a> {
    cameras: Vec<Vec<ImageEntry>>,
    imu: Vec<String>,
    lidar: Vec<String>,
    gps: Option<Vec<String>>,
    depths: Vec<ImageEntry>,
    cb: SensorCallbacks,
    lcb: Option<LidarCallback>,
    ncb: Option<NetworkImagesCallback>,
    codec: Arc<dyn ImageCodec>,
    dataset_path: PathBuf,
    running: &'a AtomicBool,
    counter: &'a AtomicUsize,
    delay: f64,
    depth_id: Option<usize>,
    rgb_id: Option<usize>,
    images: ImageStreams,
    imu_on: bool,
    lidar_on: bool,
    kind: Option<String>,
}
impl XPlayback<'_> {
    fn run(self) -> io::Result<()> {
        let Self {
            cameras,
            imu,
            lidar,
            gps,
            depths,
            cb,
            lcb,
            ncb,
            codec,
            dataset_path,
            running,
            counter,
            delay,
            depth_id,
            rgb_id,
            images:
                ImageStreams {
                    use_network: network,
                    use_depth,
                    use_rgb,
                },
            imu_on,
            lidar_on,
            kind,
        } = self;
        let mut indices = vec![0; cameras.len()];
        let (mut ii, mut li, mut gi, mut di) = (0, 0, 0, 0);
        let mut start = Timestamp::UNIX_EPOCH;
        let mut t_gps = Timestamp::UNIX_EPOCH;
        while running.load(Ordering::Acquire) && synchronize(&cameras, &mut indices, 10_000_000)? {
            let mut images = ImageMap::new();
            let mut depth_images = ImageMap::new();
            let mut timed = TimedImages::new();
            let mut timed_depth = TimedImages::new();
            let mut t = Timestamp::UNIX_EPOCH;
            for (i, camera) in cameras.iter().enumerate() {
                let entry = &camera[indices[i]];
                let image = XDatasetReader::read_playback_image(
                    codec.as_ref(),
                    entry,
                    &dataset_path,
                    (i, cameras.len()),
                    (use_rgb, rgb_id),
                )?;
                t = timestamp(entry.timestamp_ns)?;
                if start == Timestamp::UNIX_EPOCH {
                    start = t;
                }
                let first = start;
                if !XDatasetReader::deliver_lidar(
                    &lidar,
                    &mut li,
                    lidar_on,
                    (first, t, delay),
                    lcb.as_ref(),
                )? {
                    return Ok(());
                }
                if use_depth
                    && (!use_rgb || Some(i) == rgb_id)
                    && !XDatasetReader::deliver_depth_images(
                        (&depths, &mut di),
                        (t, first, delay),
                        (use_rgb, rgb_id, depth_id),
                        &image,
                        (&mut depth_images, &mut timed_depth),
                        codec.as_ref(),
                        &cb,
                    )?
                {
                    return Ok(());
                }
                if !XDatasetReader::deliver_motion_sensors(
                    (&imu, &mut ii, imu_on),
                    (gps.as_deref(), &mut gi, &mut t_gps, kind.as_deref()),
                    (first, t, delay),
                    &cb,
                )? {
                    return Ok(());
                }
                if elapsed(t, first) > delay {
                    images.insert(i, image.clone());
                    timed.insert(i, (t, image));
                }
                indices[i] += 1;
            }
            Self::publish_images(
                cameras.len(),
                &cb,
                ncb.as_ref(),
                (t, &images, &depth_images),
                (&timed, &timed_depth),
                network,
                counter,
            )?;
        }
        Ok(())
    }
    fn publish_images(
        camera_count: usize,
        cb: &SensorCallbacks,
        ncb: Option<&NetworkImagesCallback>,
        (t, images, depth_images): (Timestamp, &ImageMap, &ImageMap),
        (timed, timed_depth): (&TimedImages, &TimedImages),
        network: bool,
        counter: &AtomicUsize,
    ) -> io::Result<()> {
        if images.len() == camera_count {
            for c in &cb.images_callbacks {
                c(t, images, depth_images);
            }
            if network {
                ncb.as_ref()
                    .ok_or_else(|| invalid("network callback missing"))?(
                    timed, timed_depth
                );
            }
            counter.fetch_add(1, Ordering::Relaxed);
        }
        Ok(())
    }
}
