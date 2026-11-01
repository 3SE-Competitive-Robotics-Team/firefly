// Copyright (c) 2019 Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// Complete license: ../../LICENSE.
//! Pure frame-processing and calibration portion of original `device/rs_t265.h/.cpp`.
//! These routines process actual SDK payloads; they do not emulate a device.
use crate::imu::imu_types::ImuData;
use crate::utils::sophus_utils::so3_from_quaternion;
use crate::{io::dataset_io::ImageData, optical_flow::OpticalFlowInput};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::kannala_brandt_camera4::KannalaBrandtCamera4;
use firefly_calibration::image::image::ManagedImage;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{
    Isometry3, Matrix3, Quaternion, Rotation3, SVector, Translation3, UnitQuaternion, Vector2,
    Vector3,
};
use std::{collections::VecDeque, sync::Arc};

#[derive(Clone, Debug)]
pub struct RsImuData {
    pub timestamp: f64,
    pub data: Vector3<f64>,
}
#[derive(Clone, Debug)]
pub struct RsPoseData {
    pub t_ns: i64,
    pub data: Isometry3<f64>,
}
/// SDK motion intrinsic coefficients, retaining the SDK's f32 values.
#[derive(Clone, Debug)]
pub struct MotionIntrinsics {
    pub data: [[f32; 4]; 3],
    pub noise_variances: [f32; 3],
    pub bias_variances: [f32; 3],
}
#[derive(Clone, Debug)]
pub struct CameraProfile {
    pub width: i32,
    pub height: i32,
    pub fx: f32,
    pub fy: f32,
    pub ppx: f32,
    pub ppy: f32,
    pub coefficients: [f32; 5],
    pub rotation: [f32; 9],
    pub translation: [f32; 3],
}
/// Borrowed SDK Y8 video data. The upstream copies width*height bytes directly,
/// without interpreting the SDK row stride; callers must preserve that layout.
pub struct VideoFrame<'a> {
    pub timestamp: f64,
    pub exposure_us: i64,
    pub width: usize,
    pub height: usize,
    pub pixels: &'a [u8],
}
pub struct RsT265Processor {
    pub manual_exposure: bool,
    skip_frames: i32,
    webp_quality: i32,
    frame_counter: u64,
    gyro_data_queue: VecDeque<RsImuData>,
    prev_accel_data: Option<RsImuData>,
    last_img_data: Option<Arc<OpticalFlowInput>>,
}
fn failure(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("basalt.rs_t265")
}
fn timestamp_ns(timestamp: f64) -> Result<i64> {
    let value = timestamp * 1e6;
    if !value.is_finite() || value < i64::MIN as f64 || value >= -(i64::MIN as f64) {
        return Err(failure("SDK timestamp outside i64 nanosecond range"));
    }
    Ok(value as i64)
}
impl RsT265Processor {
    pub const IMU_RATE: usize = 200;
    pub const NUM_CAMS: usize = 2;
    #[must_use]
    pub fn new(manual_exposure: bool, skip_frames: i32, webp_quality: i32) -> Self {
        Self {
            manual_exposure,
            skip_frames,
            webp_quality,
            frame_counter: 0,
            gyro_data_queue: VecDeque::new(),
            prev_accel_data: None,
            last_img_data: None,
        }
    }
    pub fn set_skip_frames(&mut self, skip: i32) {
        self.skip_frames = skip;
    }
    pub fn set_webp_quality(&mut self, quality: i32) {
        self.webp_quality = quality;
    }
    #[must_use]
    pub fn webp_quality(&self) -> i32 {
        self.webp_quality
    }
    /// Exposure conversion passed to the real SDK option setter. Automatic mode
    /// returns None exactly as the original `setExposure` returns false.
    #[must_use]
    pub fn exposure_option(&self, milliseconds: f64) -> Option<f32> {
        self.manual_exposure
            .then_some((milliseconds * 1000.) as f32)
    }
    pub fn gyro(&mut self, sample: RsImuData) {
        self.gyro_data_queue.push_back(sample);
    }
    /// # Errors
    /// Reports non-increasing acceleration timestamps or invalid output timestamps.
    pub fn accel(&mut self, sample: RsImuData) -> Result<Vec<ImuData<f64>>> {
        let Some(previous) = &self.prev_accel_data else {
            self.prev_accel_data = Some(sample);
            return Ok(Vec::new());
        };
        if sample.timestamp <= previous.timestamp || sample.timestamp.is_nan() {
            return Err(failure("Acceleration timestamps must increase"));
        }
        while self
            .gyro_data_queue
            .front()
            .is_some_and(|v| v.timestamp < previous.timestamp)
        {
            self.gyro_data_queue.pop_front();
        }
        let mut output = Vec::new();
        while self
            .gyro_data_queue
            .front()
            .is_some_and(|v| v.timestamp < sample.timestamp)
        {
            let gyro = self
                .gyro_data_queue
                .pop_front()
                .ok_or_else(|| failure("Gyroscope queue unexpectedly empty"))?;
            let duration = sample.timestamp - previous.timestamp;
            let w0 = (sample.timestamp - gyro.timestamp) / duration;
            let w1 = (gyro.timestamp - previous.timestamp) / duration;
            output.push(ImuData {
                t_ns: timestamp_ns(gyro.timestamp)?,
                accel: w0 * previous.data + w1 * sample.data,
                gyro: gyro.data,
            });
        }
        self.prev_accel_data = Some(sample);
        Ok(output)
    }
    /// # Errors
    /// Reports invalid image buffers/dimensions and nonfinite timestamps.
    pub fn images(&mut self, frames: [VideoFrame<'_>; 2]) -> Result<Option<Arc<OpticalFlowInput>>> {
        if frames[0].timestamp != frames[1].timestamp {
            return Ok(None);
        }
        let skip = self.skip_frames.max(1) as u64;
        let counter = self.frame_counter;
        self.frame_counter = self.frame_counter.wrapping_add(1);
        if !counter.is_multiple_of(skip) {
            return Ok(None);
        }
        let mut output = OpticalFlowInput {
            t_ns: timestamp_ns(frames[0].timestamp)?,
            img_data: Vec::with_capacity(2),
        };
        for frame in frames {
            let count = frame
                .width
                .checked_mul(frame.height)
                .ok_or_else(|| failure("Video frame dimensions overflow"))?;
            if frame.pixels.len() < count {
                return Err(failure("Truncated SDK video frame"));
            }
            let mut image = ManagedImage::new(frame.width, frame.height);
            for (dst, &src) in image.data_mut().iter_mut().zip(frame.pixels) {
                *dst = u16::from(src) << 8;
            }
            output.img_data.push(ImageData {
                img: Some(Arc::new(image)),
                exposure: frame.exposure_us as f64 * 1e-6,
            });
        }
        let output = Arc::new(output);
        self.last_img_data = Some(Arc::clone(&output));
        Ok(Some(output))
    }
    #[must_use]
    pub fn last_image_data(&self) -> Option<Arc<OpticalFlowInput>> {
        self.last_img_data.clone()
    }
    /// # Errors
    /// Reports nonfinite or out-of-range timestamps. Invalid geometry is skipped.
    pub fn pose(
        timestamp: f64,
        translation: [f32; 3],
        rotation_xyzw: [f32; 4],
    ) -> Result<Option<RsPoseData>> {
        let translation = Vector3::from(translation).map(f64::from);
        let q = Quaternion::new(
            f64::from(rotation_xyzw[3]),
            f64::from(rotation_xyzw[0]),
            f64::from(rotation_xyzw[1]),
            f64::from(rotation_xyzw[2]),
        );
        let norm2 = (q.i * q.i + q.k * q.k) + (q.j * q.j + q.w * q.w);
        if !translation.iter().all(|x| x.is_finite())
            || !q.coords.iter().all(|x| x.is_finite())
            || norm2 < 1e-12
        {
            return Ok(None);
        }
        // Original explicitly normalizes, then calls the Sophus constructor,
        // which performs its own normalization again.
        Ok(Some(RsPoseData {
            t_ns: timestamp_ns(timestamp)?,
            data: Isometry3::from_parts(
                Translation3::from(translation),
                so3_from_quaternion(q / norm2.sqrt()),
            ),
        }))
    }
}
/// Converts actual gyro/accelerometer and fisheye SDK profiles to Basalt fields.
#[must_use]
pub fn export_calibration(
    gyro: &MotionIntrinsics,
    accel: &MotionIntrinsics,
    cameras: &[CameraProfile; 2],
) -> Calibration<f64> {
    let mut calibration = Calibration {
        imu_update_rate: RsT265Processor::IMU_RATE as f64,
        ..Default::default()
    };
    let g = gyro.data.map(|r| r.map(f64::from));
    let a = accel.data.map(|r| r.map(f64::from));
    *calibration.calib_gyro_bias.param_mut() = SVector::from_row_slice(&[
        g[0][3],
        g[1][3],
        g[2][3],
        g[0][0] - 1.,
        g[1][0],
        g[2][0],
        g[0][1],
        g[1][1] - 1.,
        g[2][1],
        g[0][2],
        g[1][2],
        g[2][2] - 1.,
    ]);
    *calibration.calib_accel_bias.param_mut() = SVector::from_row_slice(&[
        a[0][3],
        a[1][3],
        a[2][3],
        a[0][0] - 1.,
        a[1][0],
        a[2][0],
        a[1][1] - 1.,
        a[2][1],
        a[2][2] - 1.,
    ]);
    calibration.gyro_noise_std = Vector3::from(gyro.noise_variances).map(|v| f64::from(v).sqrt());
    calibration.gyro_bias_std = Vector3::from(gyro.bias_variances).map(|v| f64::from(v).sqrt());
    calibration.accel_noise_std = Vector3::from(accel.noise_variances).map(|v| f64::from(v).sqrt());
    calibration.accel_bias_std = Vector3::from(accel.bias_variances).map(|v| f64::from(v).sqrt());
    for camera in cameras {
        let matrix = Matrix3::from_column_slice(&camera.rotation.map(f64::from));
        let rotation =
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(matrix))
                .into_inner();
        calibration.t_i_c.push(Isometry3::from_parts(
            Translation3::from(Vector3::from(camera.translation).map(f64::from)),
            so3_from_quaternion(rotation),
        ));
        calibration
            .resolution
            .push(Vector2::new(camera.width, camera.height));
        let param = SVector::<f64, 8>::from_row_slice(
            &[
                camera.fx,
                camera.fy,
                camera.ppx,
                camera.ppy,
                camera.coefficients[0],
                camera.coefficients[1],
                camera.coefficients[2],
                camera.coefficients[3],
            ]
            .map(f64::from),
        );
        calibration
            .intrinsics
            .push(GenericCamera::KannalaBrandt4(KannalaBrandtCamera4::new(
                param,
            )));
    }
    calibration
}
#[must_use]
pub fn is_ubuntu() -> bool {
    std::fs::read_to_string("/etc/os-release").is_ok_and(|text| {
        text.lines().any(|line| {
            (line.starts_with("ID=") || line.starts_with("ID_LIKE=")) && line.contains("ubuntu")
        })
    })
}
pub const UBUNTU_UDEV_INSTRUCTIONS: &str = "Ubuntu udev setup may be missing for the RealSense T265.\nInstall the bundled Basalt rules file with:\n  sudo cp ~/.local/etc/basalt/udev/99-realsense-libusb.rules /etc/udev/rules.d/\n  sudo udevadm control --reload-rules && sudo udevadm trigger\nThen unplug and reconnect the T265 and retry.\n";
