/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Depth registration corresponding to `DepthUtils.hpp` and `OpenCV` rgbd.
use nalgebra::{Matrix3, Matrix4, Vector3};
use purecv::core::Matrix;
use std::io;
pub trait DepthValue: Copy + Default + PartialOrd {
    fn no_depth() -> Self;
    fn is_no_depth(self) -> bool;
    fn from_float(value: f32) -> Self;
    fn as_float(self) -> f32;
    fn scale() -> f32;
}
impl DepthValue for u16 {
    fn no_depth() -> Self {
        0
    }
    fn is_no_depth(self) -> bool {
        self == 0
    }
    fn from_float(v: f32) -> Self {
        (v + 0.5) as u16
    }
    fn as_float(self) -> f32 {
        f32::from(self)
    }
    fn scale() -> f32 {
        0.001
    }
}
impl DepthValue for f32 {
    fn no_depth() -> Self {
        f32::NAN
    }
    fn is_no_depth(self) -> bool {
        self.is_nan()
    }
    fn from_float(v: f32) -> Self {
        v
    }
    fn as_float(self) -> f32 {
        self
    }
    fn scale() -> f32 {
        1.0
    }
}
impl DepthValue for f64 {
    fn no_depth() -> Self {
        f64::NAN
    }
    fn is_no_depth(self) -> bool {
        self.is_nan()
    }
    fn from_float(v: f32) -> Self {
        f64::from(v)
    }
    fn as_float(self) -> f32 {
        self as f32
    }
    fn scale() -> f32 {
        1.0
    }
}
#[must_use]
pub fn no_depth_sentinel_value<D: DepthValue>() -> D {
    D::no_depth()
}
pub fn is_equal_to_no_depth_sentinel_value<D: DepthValue>(value: D) -> bool {
    value.is_no_depth()
}
#[must_use]
pub fn float_to_input_depth<D: DepthValue>(value: f32) -> D {
    D::from_float(value)
}
fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message)
}
/// Input and output sizes must be positive. The point-cloud scratch plane has
/// output dimensions, so each input dimension must fit inside that plane.
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn perform_registration<D: DepthValue>(
    input: &Matrix<D>,
    (unregistered_camera, registered_camera): (&Matrix3<f32>, &Matrix3<f32>),
    distortion: &[f32],
    transform: &Matrix4<f32>,
    output_size: (usize, usize),
    (dilation, input_to_metres): (bool, f32),
) -> io::Result<Matrix<D>> {
    let (width, height) = output_size;
    if input.rows == 0 || input.cols == 0 || input.channels != 1 || width == 0 || height == 0 {
        return Err(invalid(
            "depth and output dimensions must be positive and depth must have one channel",
        ));
    }
    if input.cols > width || input.rows > height {
        return Err(invalid("input depth exceeds the point-cloud scratch plane"));
    }
    if !distortion.is_empty() && ![4, 5, 8, 12, 14].contains(&distortion.len()) {
        return Err(invalid(
            "distortion must have 4, 5, 8, 12 or 14 coefficients",
        ));
    }
    let mut out = Matrix::new(height, width, 1);
    out.data.fill(D::no_depth());
    let has_distortion = distortion.iter().any(|&x| x != 0.0);
    let mut k = Matrix4::<f32>::zeros();
    k.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(unregistered_camera);
    k[(3, 3)] = 1.0;
    let inverse = k.try_inverse().unwrap_or_else(Matrix4::zeros);
    let initial = if has_distortion {
        transform * inverse
    } else {
        let mut kr = Matrix4::<f32>::zeros();
        kr.fixed_view_mut::<3, 3>(0, 0).copy_from(registered_camera);
        kr[(3, 3)] = 1.0;
        kr * transform * inverse
    };
    let initial = initial.cast::<f64>();
    for y in 0..height {
        for x in 0..width {
            let mut source = Vector3::<f32>::zeros();
            if y < input.rows && x < input.cols {
                let mut depth = input.data[y * input.cols + x].as_float() * input_to_metres;
                if depth == 0.0 {
                    depth = f32::NAN;
                }
                source = Vector3::new(x as f32 * depth, y as f32 * depth, depth);
            }
            let point = initial * source.cast::<f64>().push(1.0);
            let w = point.w;
            let cloud = if w.abs() > f64::from(f32::EPSILON) {
                (point.fixed_rows::<3>(0) / w).cast::<f32>()
            } else {
                Vector3::zeros()
            };
            let pixel = if has_distortion {
                project_distorted(cloud, registered_camera, distortion)
            } else {
                [cloud.x / cloud.z, cloud.y / cloud.z]
            };
            if pixel[0].is_nan() {
                continue;
            }
            let px = pixel[0].round_ties_even() as i32;
            let py = pixel[1].round_ties_even() as i32;
            if px < 0 || py < 0 || px >= width as i32 || py >= height as i32 {
                continue;
            }
            let depth = D::from_float(cloud.z * (1.0 / input_to_metres));
            for (dx, dy) in if dilation {
                &[(0, 0), (-1, 0), (0, -1), (-1, -1)][..]
            } else {
                &[(0, 0)][..]
            } {
                let xx = px + dx;
                let yy = py + dy;
                if xx < 0 || yy < 0 || xx >= width as i32 || yy >= height as i32 {
                    continue;
                }
                let old = &mut out.data[yy as usize * width + xx as usize];
                if old.is_no_depth() || *old > depth {
                    *old = depth;
                }
            }
        }
    }
    Ok(out)
}
fn project_distorted(point: Vector3<f32>, keypoint: &Matrix3<f32>, depth: &[f32]) -> [f32; 2] {
    let mut coefficients = [0.0; 14];
    for (out, &value) in coefficients.iter_mut().zip(depth) {
        *out = f64::from(value);
    }
    let distance = if point.z == 0.0 {
        1.0
    } else {
        1.0 / f64::from(point.z)
    };
    let value_value = f64::from(point.x) * distance;
    let vertical = f64::from(point.y) * distance;
    let r2 = value_value * value_value + vertical * vertical;
    let r4 = r2 * r2;
    let r6 = r4 * r2;
    let radial = (1.0 + coefficients[0] * r2 + coefficients[1] * r4 + coefficients[4] * r6)
        / (1.0 + coefficients[5] * r2 + coefficients[6] * r4 + coefficients[7] * r6);
    let xx = value_value * radial
        + 2.0 * coefficients[2] * value_value * vertical
        + coefficients[3] * (r2 + 2.0 * value_value * value_value)
        + coefficients[8] * r2
        + coefficients[9] * r4;
    let yy = vertical * radial
        + coefficients[2] * (r2 + 2.0 * vertical * vertical)
        + 2.0 * coefficients[3] * value_value * vertical
        + coefficients[10] * r2
        + coefficients[11] * r4;
    let (sx, cx) = coefficients[12].sin_cos();
    let (sy, cy) = coefficients[13].sin_cos();
    let rx = Matrix3::new(1.0, 0.0, 0.0, 0.0, cx, sx, 0.0, -sx, cx);
    let ry = Matrix3::new(cy, 0.0, -sy, 0.0, 1.0, 0.0, sy, 0.0, cy);
    let ratio = ry * rx;
    let projection = Matrix3::new(
        ratio[(2, 2)],
        0.0,
        -ratio[(0, 2)],
        0.0,
        ratio[(2, 2)],
        -ratio[(1, 2)],
        0.0,
        0.0,
        1.0,
    );
    let tilted = projection * ratio * Vector3::new(xx, yy, 1.0);
    let inv = if tilted.z == 0.0 { 1.0 } else { 1.0 / tilted.z };
    [
        (f64::from(keypoint[(0, 0)]) * tilted.x * inv + f64::from(keypoint[(0, 2)])) as f32,
        (f64::from(keypoint[(1, 1)]) * tilted.y * inv + f64::from(keypoint[(1, 2)])) as f32,
    ]
}

/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn register_depth<D: DepthValue>(
    unregistered_camera: &Matrix3<f32>,
    registered_camera: &Matrix3<f32>,
    distortion: &[f32],
    transform: &Matrix4<f32>,
    input: &Matrix<D>,
    output_size: (usize, usize),
    dilation: bool,
) -> io::Result<Matrix<D>> {
    perform_registration(
        input,
        (unregistered_camera, registered_camera),
        distortion,
        transform,
        output_size,
        (dilation, D::scale()),
    )
}
