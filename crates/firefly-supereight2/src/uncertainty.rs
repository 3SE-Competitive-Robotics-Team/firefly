/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Per-pixel standard deviation used by occupancy fusion.
use crate::image::invalid;
use crate::{
    data_field::{OccupancyConfig, UncertaintyModel},
    image::Image,
};
use firefly_error::Result;
/// # Errors
/// Returns an error if depth storage is inconsistent or the configured minimum sigma exceeds
/// the maximum.
pub fn depth_sigma(
    depth: &Image<f32>,
    map_resolution: f32,
    config: &OccupancyConfig,
) -> Result<Image<f32>> {
    if depth.channels != 1 || depth.rows.checked_mul(depth.cols) != Some(depth.data.len()) {
        return Err(invalid("depth image shape mismatch"));
    }
    let sigma_min = map_resolution * config.sigma_min_factor;
    let sigma_max = map_resolution * config.sigma_max_factor;
    if sigma_min > sigma_max {
        return Err(invalid("reversed depth sigma bounds"));
    }
    let mut sigma = Image::<f32>::new(depth.rows, depth.cols, 1);
    for (out, &value) in sigma.data.iter_mut().zip(&depth.data) {
        *out = if value <= 0.0 || value.is_nan() {
            0.0
        } else {
            let mut s = config.k_sigma * value;
            if config.uncertainty_model == UncertaintyModel::Quadratic {
                s *= value;
            }
            cpp_clamp(s, sigma_min, sigma_max)
        };
    }
    Ok(sigma)
}
pub(crate) fn cpp_min(a: f32, b: f32) -> f32 {
    if b < a { b } else { a }
}
pub(crate) fn cpp_max(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}
pub(crate) fn cpp_clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}
