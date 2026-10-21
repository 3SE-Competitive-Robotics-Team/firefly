/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Alternating emitter RGB-D selection with depth scaling and SDK alignment.
use crate::{
    runtime::vi_interface::ImageMap,
    sensors::{
        realsense::{
            self, DeviceOption, Format, Frame, FrameData, Metadata, RealSenseSdk, Realsense,
            Sensor, SensorType, Stream, StreamProfile,
        },
        vi_sensor_base::{SensorCallbacks, ViSensorBase},
    },
};
use jiff::Timestamp;
use purecv::core::{DynamicData, DynamicMatrix, Matrix};
use std::{
    io,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
};
static LAST_FRAMESET_NUMBER: AtomicU64 = AtomicU64::new(0);
pub struct RealsenseRgbd {
    pub base: Realsense,
    align_depth_to_rgb: bool,
    emitter_power: f64,
    supports_metadata: bool,
    imu_stream_started: bool,
    depth_scale: Option<f32>,
}
impl RealsenseRgbd {
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    pub fn new(
        sensor: SensorType,
        enable_rgb: bool,
        align_depth_to_rgb: bool,
        emitter_power: f64,
        rgb_exposure: f32,
        sdk: Box<dyn RealSenseSdk>,
    ) -> io::Result<Self> {
        if sensor != SensorType::D455 {
            return Err(realsense::invalid("Depth channel not supported"));
        }
        Ok(Self {
            base: Realsense::new(sensor, enable_rgb, rgb_exposure, sdk),
            align_depth_to_rgb,
            emitter_power,
            supports_metadata: true,
            imu_stream_started: false,
            depth_scale: None,
        })
    }
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    pub fn process_depth(
        &mut self,
        f: &Frame,
        out: &mut ImageMap,
    ) -> io::Result<Option<Timestamp>> {
        let FrameData::Frameset {
            depth: Some(depth), ..
        } = &f.data
        else {
            return Ok(None);
        };
        let (_, t) = self.base.compute_timestamp(f, Metadata::SensorTimestamp)?;
        let (frame, size, id) = if self.align_depth_to_rgb {
            (
                self.base.sdk.align_depth_to_colour(f)?,
                self.base.rgb_size,
                2,
            )
        } else {
            ((**depth).clone(), (640, 480), 0)
        };
        let image = realsense::video(&frame, size, 1)?;
        let DynamicData::U16(m) = &image.data else {
            return Err(realsense::invalid("depth must be u16"));
        };
        let scale = self
            .depth_scale
            .ok_or_else(|| realsense::invalid("depth scale unavailable"))?;
        out.insert(
            id,
            Arc::new(DynamicMatrix {
                data: DynamicData::F32(Matrix::from_vec(
                    m.rows,
                    m.cols,
                    1,
                    m.data.iter().map(|&v| f32::from(v) * scale).collect(),
                )),
            }),
        );
        Ok(Some(t))
    }
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    pub fn process_frame(&mut self, f: &Frame) -> io::Result<()> {
        if !self.base.streaming || !self.base.check_frame_and_update(f)? {
            return Ok(());
        }
        if let FrameData::Frameset {
            infrared_left,
            infrared_right,
            colour,
            depth,
        } = &f.data
        {
            if f.number <= LAST_FRAMESET_NUMBER.fetch_max(f.number, Ordering::AcqRel) {
                log::warn!("RealSense duplicate frame time -- skip");
                return Ok(());
            }
            let mut out = ImageMap::new();
            let mut depths = ImageMap::new();
            let mut t = None;
            let mut emitter_on = false;
            if let (Some(l), Some(r)) = (infrared_left, infrared_right)
                && self.imu_stream_started
            {
                let a = l.metadata.get(&Metadata::FrameEmitterMode);
                let b = r.metadata.get(&Metadata::FrameEmitterMode);
                if self.supports_metadata && (a.is_none() || b.is_none()) {
                    log::warn!("reading emitter mode is not supported, turning off emitter");
                    for (option, value) in [
                        (DeviceOption::EmitterOnOff, 0.0),
                        (DeviceOption::EmitterEnabled, 0.0),
                        (DeviceOption::LaserPower, 0.0),
                    ] {
                        self.base.sdk.set_option(Sensor::Depth, option, value)?;
                    }
                    self.supports_metadata = false;
                    return Ok(());
                }
                if self.supports_metadata {
                    emitter_on = a.is_some_and(|v| *v != 0) && b.is_some_and(|v| *v != 0);
                }
                if self.supports_metadata && !emitter_on {
                    t = self.base.process_ir(f, &mut out)?;
                }
            }
            if colour.is_some() && self.imu_stream_started {
                let rgb = self.base.process_rgb(f, &mut out)?;
                if rgb.is_some() {
                    t = rgb;
                }
            }
            if depth.is_some() && self.imu_stream_started && emitter_on && self.supports_metadata {
                let dt = self.process_depth(f, &mut depths)?;
                if dt.is_some() {
                    t = dt;
                }
            }
            if let Some(t) = t {
                self.base.image_received_counter += 1;
                if self.base.image_received_counter > 15 {
                    for cb in &self.base.callbacks.images_callbacks {
                        cb(t, &out, &depths);
                    }
                }
            }
        }
        let valid = self.base.process_imu(f)?;
        if !self.imu_stream_started && valid {
            self.imu_stream_started = true;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    pub fn poll(&mut self) -> io::Result<bool> {
        if let Some(f) = self.base.sdk.next_event()? {
            self.process_frame(&f)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}
impl ViSensorBase for RealsenseRgbd {
    fn callbacks(&self) -> &SensorCallbacks {
        &self.base.callbacks
    }
    fn callbacks_mut(&mut self) -> &mut SensorCallbacks {
        &mut self.base.callbacks
    }
    fn is_streaming(&self) -> bool {
        self.base.streaming
    }
    fn start_streaming(&mut self) -> io::Result<bool> {
        let success = self.base.configure_streams(30, 15)?;
        if !success {
            return Ok(false);
        }
        self.base.sdk.enable_stream(StreamProfile {
            stream: Stream::Depth,
            index: None,
            width: self.base.ir_size.0,
            height: self.base.ir_size.1,
            format: Format::Any,
            fps: 30,
        })?;
        self.base.sdk.start_pipeline()?;
        let result = (|| {
            for (option, value) in [
                (DeviceOption::EmitterOnOff, 0.0),
                (DeviceOption::LaserPower, self.emitter_power),
                (DeviceOption::EmitterEnabled, 1.0),
                (DeviceOption::EmitterOnOff, 1.0),
            ] {
                self.base.sdk.set_option(Sensor::Depth, option, value)?;
            }
            for sensor in [Sensor::Depth, Sensor::Motion, Sensor::Colour] {
                self.base
                    .sdk
                    .set_option(sensor, DeviceOption::GlobalTimeEnabled, 1.0)?;
            }
            if self.base.rgb_exposure > 0.0 {
                for (option, value) in [
                    (DeviceOption::AutoExposure, 0.0),
                    (DeviceOption::Exposure, f64::from(self.base.rgb_exposure)),
                    (DeviceOption::WhiteBalance, 4500.0),
                    (DeviceOption::BacklightCompensation, 0.0),
                ] {
                    self.base.sdk.set_option(Sensor::Colour, option, value)?;
                }
            }
            self.base
                .sdk
                .set_option(Sensor::Depth, DeviceOption::AutoExposure, 1.0)?;
            self.depth_scale = Some(self.base.sdk.depth_scale()?);
            Ok(())
        })();
        if let Err(e) = result {
            self.base.sdk.stop_pipeline()?;
            return Err(e);
        }
        self.base.streaming = true;
        Ok(success)
    }
    fn stop_streaming(&mut self) -> io::Result<bool> {
        self.base.stop_streaming()
    }
}
