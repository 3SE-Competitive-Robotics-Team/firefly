/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! `RealSense` stream configuration, metadata time conversion and inertial interpolation.
use crate::{
    runtime::vi_interface::ImageMap,
    sensors::vi_sensor_base::{SensorCallbacks, ViSensorBase},
};
use jiff::Timestamp;
use nalgebra::Vector3;
use purecv::core::DynamicMatrix;
use std::{
    collections::{BTreeMap, BTreeSet},
    io,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
};
static LAST_FRAMESET_NUMBER: AtomicU64 = AtomicU64::new(0);
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorType {
    D435i,
    D455,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Stream {
    Accelerometer,
    Gyroscope,
    Infrared,
    Colour,
    Depth,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    MotionXyz32f,
    Y8,
    Bgr8,
    Any,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Metadata {
    FrameTimestamp,
    SensorTimestamp,
    FrameEmitterMode,
    ActualExposure,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sensor {
    Depth,
    Motion,
    Colour,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeviceOption {
    EmitterOnOff,
    EmitterEnabled,
    GlobalTimeEnabled,
    Exposure,
    AutoExposure,
    LaserPower,
    WhiteBalance,
    BacklightCompensation,
}
#[derive(Clone, Debug)]
pub struct StreamProfile {
    pub stream: Stream,
    pub index: Option<u32>,
    pub width: usize,
    pub height: usize,
    pub format: Format,
    pub fps: u32,
}
#[derive(Clone, Debug)]
pub struct DeviceInfo {
    pub name: Option<String>,
    pub serial_number: Option<String>,
    pub sensor_streams: Vec<BTreeSet<Stream>>,
}
impl DeviceInfo {
    #[must_use]
    pub fn display_name(&self) -> String {
        format!(
            "{} {}",
            self.name.as_deref().unwrap_or("Unknown Device"),
            self.serial_number
                .as_ref()
                .map_or_else(|| "########".into(), |s| format!("#{s}"))
        )
    }
}
#[derive(Clone, Debug)]
pub struct Frame {
    pub number: u64,
    pub host_timestamp_ms: f64,
    pub metadata: BTreeMap<Metadata, u64>,
    pub data: FrameData,
}
#[derive(Clone, Debug)]
pub enum FrameData {
    Frameset {
        infrared_left: Option<Box<Frame>>,
        infrared_right: Option<Box<Frame>>,
        colour: Option<Box<Frame>>,
        depth: Option<Box<Frame>>,
    },
    Video(Arc<DynamicMatrix>),
    Motion {
        stream: Stream,
        format: Format,
        value: Vector3<f64>,
    },
}
/// SDK operations are capability boundaries. Frame decoding and selection remain in Rust.
pub trait RealSenseSdk: Send {
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn query_devices(&mut self) -> io::Result<Vec<DeviceInfo>>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn enable_stream(&mut self, profile: StreamProfile) -> io::Result<()>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn start_pipeline(&mut self) -> io::Result<()>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn next_event(&mut self) -> io::Result<Option<Frame>>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn set_option(&mut self, sensor: Sensor, option: DeviceOption, value: f64) -> io::Result<()>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn stop_pipeline(&mut self) -> io::Result<()>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn depth_scale(&mut self) -> io::Result<f32>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn align_depth_to_colour(&mut self, frameset: &Frame) -> io::Result<Frame>;
}
pub(crate) fn invalid(s: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, s.into())
}
#[derive(Clone, Copy, Debug, Default)]
pub struct CaptureOptions {
    pub enable_rgb: bool,
    pub has_device_timestamps: bool,
}
pub struct Realsense {
    pub callbacks: SensorCallbacks,
    pub(crate) sdk: Box<dyn RealSenseSdk>,
    pub(crate) streaming: bool,
    pub(crate) started: bool,
    pub(crate) sensor_type: SensorType,
    pub capture: CaptureOptions,
    pub(crate) rgb_exposure: f32,
    pub(crate) ir_size: (usize, usize),
    pub(crate) rgb_size: (usize, usize),

    first_time_us: Option<u64>,
    host_time_offset_us: u64,
    frame_counter: u64,
    pub(crate) image_received_counter: u64,
    last_image_index: Option<u64>,
    last_rgb_index: Option<u64>,
    last_gyro_index: u64,
    last_acc_index: u64,
    gyro: BTreeMap<u64, (Timestamp, Vector3<f64>)>,
    acc: BTreeMap<u64, (Timestamp, Vector3<f64>)>,
}
impl Realsense {
    #[must_use]
    pub fn new(
        sensor_type: SensorType,
        enable_rgb: bool,
        rgb_exposure: f32,
        sdk: Box<dyn RealSenseSdk>,
    ) -> Self {
        Self {
            callbacks: SensorCallbacks::default(),
            sdk,
            streaming: false,
            started: false,
            sensor_type,
            capture: crate::sensors::realsense::CaptureOptions {
                enable_rgb,
                has_device_timestamps: true,
            },
            rgb_exposure,
            ir_size: (640, 480),
            rgb_size: (640, 480),

            first_time_us: None,
            host_time_offset_us: 0,
            frame_counter: 0,
            image_received_counter: 0,
            last_image_index: None,
            last_rgb_index: None,
            last_gyro_index: 0,
            last_acc_index: 0,
            gyro: BTreeMap::new(),
            acc: BTreeMap::new(),
        }
    }
    #[must_use]
    pub fn with_sizes(
        sensor_type: SensorType,
        enable_rgb: bool,
        device_timestamps: bool,
        rgb_size: (usize, usize),
        ir_size: (usize, usize),
        rgb_exposure: f32,
        sdk: Box<dyn RealSenseSdk>,
    ) -> Self {
        let mut s = Self::new(sensor_type, enable_rgb, rgb_exposure, sdk);
        s.capture.has_device_timestamps = device_timestamps;
        s.rgb_size = rgb_size;
        s.ir_size = ir_size;
        s
    }
    pub fn set_sensor_type(&mut self, s: SensorType) {
        self.sensor_type = s;
    }
    pub fn set_has_device_timestamps(&mut self, b: bool) {
        self.capture.has_device_timestamps = b;
    }
    pub fn set_ir_size(&mut self, w: usize, h: usize) {
        self.ir_size = (w, h);
    }
    pub fn set_rgb_size(&mut self, w: usize, h: usize) {
        self.rgb_size = (w, h);
    }
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    pub fn check_support(&mut self) -> io::Result<bool> {
        let (mut gyro, mut acc, mut ir, mut rgb) = (false, false, false, false);
        for d in self.sdk.query_devices()? {
            gyro = false;
            acc = false;
            for p in d.sensor_streams {
                gyro |= p.contains(&Stream::Gyroscope);
                acc |= p.contains(&Stream::Accelerometer);
                ir |= p.contains(&Stream::Infrared);
                rgb |= p.contains(&Stream::Colour);
            }
            if gyro && acc && ir && (!self.capture.enable_rgb || rgb) {
                break;
            }
        }
        Ok(gyro && acc && ir && (!self.capture.enable_rgb || rgb))
    }
    pub(crate) fn configure_streams(&mut self, ir_fps: u32, rgb_fps: u32) -> io::Result<bool> {
        let devices = self.sdk.query_devices()?;
        if devices.is_empty() {
            log::error!("No device connected, please connect a RealSense device");
            return Ok(false);
        }
        for (i, d) in devices.iter().enumerate() {
            log::info!("{i}: {}", d.display_name());
        }
        if !self.check_support()? {
            return Err(invalid("IR stereo or IMU not supported by sensor"));
        }
        if self.callbacks.images_callbacks.is_empty() || self.callbacks.imu_callbacks.is_empty() {
            return Err(invalid("image and IMU callbacks required"));
        }
        for (stream, fps) in [(Stream::Accelerometer, 200), (Stream::Gyroscope, 400)] {
            self.sdk.enable_stream(StreamProfile {
                stream,
                index: None,
                width: 0,
                height: 0,
                format: Format::MotionXyz32f,
                fps,
            })?;
        }
        for index in 1..=2 {
            self.sdk.enable_stream(StreamProfile {
                stream: Stream::Infrared,
                index: Some(index),
                width: self.ir_size.0,
                height: self.ir_size.1,
                format: Format::Y8,
                fps: ir_fps,
            })?;
        }
        if self.capture.enable_rgb {
            if self.sensor_type != SensorType::D455 {
                return Err(invalid("RGB requested but not supported"));
            }
            self.sdk.enable_stream(StreamProfile {
                stream: Stream::Colour,
                index: None,
                width: self.rgb_size.0,
                height: self.rgb_size.1,
                format: Format::Bgr8,
                fps: rgb_fps,
            })?;
        }
        Ok(true)
    }
    pub(crate) fn check_frame_and_update(&mut self, f: &Frame) -> io::Result<bool> {
        if self.capture.has_device_timestamps && !f.metadata.contains_key(&Metadata::FrameTimestamp)
        {
            log::warn!("Device timestamps not available. Switching to host timestamps");
            self.capture.has_device_timestamps = false;
            return Ok(false);
        }
        let host = host_us(f)?;
        let sensor = if self.capture.has_device_timestamps {
            f.metadata[&Metadata::FrameTimestamp]
        } else {
            host
        };
        self.first_time_us.get_or_insert(sensor);
        if !self.started
            && let FrameData::Frameset {
                infrared_left: Some(_),
                infrared_right: Some(_),
                ..
            } = &f.data
        {
            self.started = true;
        }
        let measured = host.wrapping_sub(sensor);
        self.host_time_offset_us = self
            .frame_counter
            .wrapping_mul(self.host_time_offset_us)
            .wrapping_add(measured)
            / (self.frame_counter + 1);
        self.frame_counter = (self.frame_counter + 1).min(1000);
        Ok(true)
    }
    pub(crate) fn compute_timestamp(
        &self,
        f: &Frame,
        metadata: Metadata,
    ) -> io::Result<(u64, Timestamp)> {
        let host = host_us(f)?;
        let sensor = if self.capture.has_device_timestamps {
            *f.metadata
                .get(&metadata)
                .ok_or_else(|| invalid("required RealSense timestamp metadata unavailable"))?
        } else {
            host
        };
        let us = if self.capture.has_device_timestamps {
            sensor.wrapping_add(self.host_time_offset_us)
        } else {
            host
        };
        let ns = us
            .checked_mul(1000)
            .ok_or_else(|| invalid("RealSense timestamp overflow"))?;
        let t = Timestamp::from_nanosecond(i128::from(ns)).map_err(|e| invalid(e.to_string()))?;
        Ok((sensor, t))
    }
    pub(crate) fn process_ir(
        &mut self,
        f: &Frame,
        out: &mut ImageMap,
    ) -> io::Result<Option<Timestamp>> {
        let FrameData::Frameset {
            infrared_left: Some(left),
            infrared_right: Some(right),
            ..
        } = &f.data
        else {
            return Ok(None);
        };
        let (sensor, t) = self.compute_timestamp(f, Metadata::SensorTimestamp)?;
        if left.number != right.number || self.last_image_index.is_some_and(|n| n >= left.number) {
            return Ok(None);
        }
        self.last_image_index = Some(left.number);
        if !self.started {
            self.started = true;
            log::info!(
                "first image received after {} seconds",
                sensor.wrapping_sub(self.first_time_us.unwrap_or(sensor)) as f64 * 1e-6
            );
        }
        out.insert(0, video(left, self.ir_size, 1)?);
        out.insert(1, video(right, self.ir_size, 1)?);
        Ok(Some(t))
    }
    pub(crate) fn process_rgb(
        &mut self,
        f: &Frame,
        out: &mut ImageMap,
    ) -> io::Result<Option<Timestamp>> {
        let FrameData::Frameset {
            colour: Some(rgb), ..
        } = &f.data
        else {
            return Ok(None);
        };
        let (_, t) = self.compute_timestamp(f, Metadata::SensorTimestamp)?;
        if self.last_rgb_index.is_some_and(|n| n >= rgb.number) {
            return Ok(None);
        }
        self.last_rgb_index = Some(rgb.number);
        out.insert(2, video(rgb, self.rgb_size, 3)?);
        Ok(Some(t))
    }
    pub(crate) fn process_imu(&mut self, frame: &Frame) -> io::Result<bool> {
        let (sensor, timestamp) = self.compute_timestamp(frame, Metadata::FrameTimestamp)?;
        let FrameData::Motion {
            stream,
            format: Format::MotionXyz32f,
            value,
        } = &frame.data
        else {
            return Ok(false);
        };
        match stream {
            Stream::Gyroscope => {
                if self.last_gyro_index >= frame.number {
                    log::warn!("gyro sequence number out of order");
                }
                self.last_gyro_index = frame.number;
                self.gyro.insert(sensor, (timestamp, *value));
            }
            Stream::Accelerometer => {
                if self.last_acc_index >= frame.number {
                    log::warn!("accelerometer sequence number out of order");
                }
                self.last_acc_index = frame.number;
                self.acc.insert(sensor, (timestamp, *value));
            }
            _ => return Ok(false),
        }
        if *stream != Stream::Accelerometer {
            return Ok(false);
        }
        let mut success = false;
        loop {
            let mut accs = self.acc.iter();
            let Some((&earlier_stamp, (_, earlier_acceleration))) = accs.next() else {
                break;
            };
            let Some((&later_stamp, (_, later_acceleration))) = accs.next() else {
                break;
            };
            let Some((&gyro_stamp, (gt, gv))) = self.gyro.first_key_value() else {
                break;
            };
            if gyro_stamp < earlier_stamp {
                log::warn!("discarding gyro measurement at t={gt}");
                self.gyro.remove(&gyro_stamp);
                continue;
            }
            if gyro_stamp > later_stamp {
                self.acc.remove(&earlier_stamp);
                continue;
            }
            let ratio = (gyro_stamp - earlier_stamp) as f64 / (later_stamp - earlier_stamp) as f64;
            let ai = (1.0 - ratio) * earlier_acceleration + ratio * later_acceleration;
            let (gt, gv) = (*gt, *gv);
            self.gyro.remove(&gyro_stamp);
            for cb in &self.callbacks.imu_callbacks {
                cb(gt, &ai, &gv);
            }
            success = true;
        }
        Ok(success)
    }
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    pub fn process_frame(&mut self, f: &Frame) -> io::Result<()> {
        if !self.streaming || !self.check_frame_and_update(f)? {
            return Ok(());
        }
        if matches!(&f.data, FrameData::Frameset { .. }) {
            let mut out = ImageMap::new();
            let mut t = self.process_ir(f, &mut out)?;
            if t.is_some() && f.number <= LAST_FRAMESET_NUMBER.fetch_max(f.number, Ordering::AcqRel)
            {
                log::warn!("RealSense duplicate frame time -- skip");
                return Ok(());
            }
            let mut valid = t.is_some();
            if self.capture.enable_rgb {
                let rgb = self.process_rgb(f, &mut out)?;
                valid &= rgb.is_some();
                if rgb.is_some() {
                    t = rgb;
                }
            }
            if valid {
                self.image_received_counter += 1;
                if self.image_received_counter > 15 {
                    for cb in &self.callbacks.images_callbacks {
                        cb(
                            t.ok_or_else(|| invalid("missing image timestamp"))?,
                            &out,
                            &ImageMap::new(),
                        );
                    }
                }
            }
        }
        self.process_imu(f)?;
        Ok(())
    }
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    pub fn poll(&mut self) -> io::Result<bool> {
        if let Some(f) = self.sdk.next_event()? {
            self.process_frame(&f)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}
fn host_us(f: &Frame) -> io::Result<u64> {
    let u = f.host_timestamp_ms * 1000.0;
    if !u.is_finite() || u < 0.0 || u > u64::MAX as f64 {
        return Err(invalid("invalid RealSense host timestamp"));
    }
    Ok(u as u64)
}
pub(crate) fn video(
    f: &Frame,
    size: (usize, usize),
    channels: usize,
) -> io::Result<Arc<DynamicMatrix>> {
    let FrameData::Video(m) = &f.data else {
        return Err(invalid("video frame payload missing"));
    };
    if m.cols() != size.0 || m.rows() != size.1 || m.channels() != channels {
        return Err(invalid("RealSense frame dimensions/format mismatch"));
    }
    Ok(Arc::new((**m).clone()))
}
impl ViSensorBase for Realsense {
    fn callbacks(&self) -> &SensorCallbacks {
        &self.callbacks
    }
    fn callbacks_mut(&mut self) -> &mut SensorCallbacks {
        &mut self.callbacks
    }
    fn is_streaming(&self) -> bool {
        self.streaming
    }
    fn start_streaming(&mut self) -> io::Result<bool> {
        let success = self.configure_streams(15, 30)?;
        if !success {
            return Ok(false);
        }
        self.sdk.start_pipeline()?;
        let result = (|| {
            self.sdk
                .set_option(Sensor::Depth, DeviceOption::EmitterOnOff, 0.0)?;
            self.sdk
                .set_option(Sensor::Depth, DeviceOption::EmitterEnabled, 0.0)?;
            self.sdk
                .set_option(Sensor::Depth, DeviceOption::GlobalTimeEnabled, 1.0)?;
            if self.capture.enable_rgb && self.rgb_exposure > 0.0 {
                self.sdk.set_option(
                    Sensor::Colour,
                    DeviceOption::Exposure,
                    f64::from(self.rgb_exposure),
                )?;
            }
            self.sdk
                .set_option(Sensor::Depth, DeviceOption::AutoExposure, 1.0)?;
            Ok(())
        })();
        if let Err(e) = result {
            self.sdk.stop_pipeline()?;
            return Err(e);
        }
        self.streaming = true;
        Ok(success)
    }
    fn stop_streaming(&mut self) -> io::Result<bool> {
        if self.streaming {
            let option = self
                .sdk
                .set_option(Sensor::Depth, DeviceOption::EmitterOnOff, 0.0);
            let stop = self.sdk.stop_pipeline();
            self.streaming = false;
            self.started = false;
            option?;
            stop?;
        }
        Ok(true)
    }
}
impl Drop for Realsense {
    fn drop(&mut self) {
        if let Err(e) = self.stop_streaming() {
            log::error!("RealSense shutdown: {e}");
        }
    }
}
