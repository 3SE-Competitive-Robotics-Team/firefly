/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Octree scale is log2(edge length in finest voxels).
use crate::image::invalid;
use firefly_error::Result;
pub type Scale = i32;
/// # Errors
/// Returns an error unless `size` is a positive power of two.
pub fn size_to_scale(size: i32) -> Result<Scale> {
    if size <= 0 || !(size as u32).is_power_of_two() {
        return Err(invalid("octant size must be a positive power of two"));
    }
    Ok(crate::math_util::log2_const(size))
}
/// # Errors
/// Returns an error if `scale` is outside the supported signed-integer shift range.
pub fn scale_to_size(scale: Scale) -> Result<i32> {
    if !(0..31).contains(&scale) {
        return Err(invalid("octant scale outside signed size range"));
    }
    Ok(1i32 << scale)
}
