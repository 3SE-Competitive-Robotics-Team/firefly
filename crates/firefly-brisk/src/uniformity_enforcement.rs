/*
Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
Stefan Leutenegger, Simon Lynen and Margarita Chli.

Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
Stefan Leutenegger and Simon Lynen.

Copyright (C) 2020  Smart Robotics Lab, Imperial College London
Stefan Leutenegger

Copyright (C) 2024  Smart Robotics Lab, Technical University of Munich
Stefan Leutenegger

All rights reserved.

This is the Author's implementation of BRISK: Binary Robust Invariant
Scalable Keypoints [1]. Various (partly unpublished) extensions are provided,
some of which are described in [2].

[1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary
    Robust Invariant Scalable Keypoints. In Proceedings of the IEEE
    International Conference on Computer Vision (ICCV), 2011.

[2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
    Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

This file is part of BRISK.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//! Score-dependent occupancy suppression using the 31-by-31 LUT.
use crate::{
    harris_score_calculator::PointWithScore,
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use purecv::core::Matrix;
pub trait UniformityPoint: Clone {
    fn x(&self) -> f32;
    fn y(&self) -> f32;
    fn score(&self) -> f64;
    fn score_ratio(&self, maximum: f32) -> f32;
}
macro_rules! point {
    ($scalar:ty, $ratio:expr) => {
        impl UniformityPoint for PointWithScore<$scalar> {
            fn x(&self) -> f32 {
                f32::from(self.x)
            }
            fn y(&self) -> f32 {
                f32::from(self.y)
            }
            fn score(&self) -> f64 {
                f64::from(self.score)
            }
            fn score_ratio(&self, maximum: f32) -> f32 {
                ($ratio)(self.score, maximum)
            }
        }
    };
}
point!(i32, |v: i32, m: f32| v as f32 / m);
point!(f32, |v: f32, m: f32| v / m);
point!(f64, |v: f64, m: f32| (v / f64::from(m)) as f32);
/// # Errors
/// Returns an error for malformed/small lookup tables, nonpositive or non-finite radius,
/// empty points, non-finite point data, or invalid lookup geometry.
/// # Panics
/// Panics if a point supplies a NaN score while sorting. Scores must remain
/// finite and stable throughout the call.
pub fn enforce_key_point_uniformity<P: UniformityPoint>(
    lut: &Matrix<f32>,
    radius: f64,
    imgrows: usize,
    imgcols: usize,
    max_num_keypoints: usize,
    points: &mut Vec<P>,
) -> Result<()> {
    check(lut)?;
    if lut.rows < 31 || lut.cols < 31 || !radius.is_finite() || radius <= 0.0 {
        return Err(invalid("positive radius and a 31-by-31 LUT required"));
    }
    if points.is_empty() {
        return Err(invalid("uniformity requires at least one scored point"));
    }
    if points
        .iter()
        .any(|p| !p.score().is_finite() || !p.x().is_finite() || !p.y().is_finite())
    {
        return Err(invalid("uniformity scores and coordinates must be finite"));
    }
    points.sort_unstable_by(|a, b| b.score().partial_cmp(&a.score()).unwrap());
    let max_score = points[0].score() as f32;
    let scaling = (15.0 / f64::from(radius as f32)) as f32;
    if !scaling.is_finite() {
        return Err(invalid("uniformity scaling is not finite"));
    }
    let scale = scaling.ceil() as usize;
    let rows = imgrows
        .checked_mul(scale)
        .and_then(|n| n.checked_add(32))
        .ok_or_else(|| invalid("occupancy size overflow"))?;
    let cols = imgcols
        .checked_mul(scale)
        .and_then(|n| n.checked_add(32))
        .ok_or_else(|| invalid("occupancy size overflow"))?;
    let size = rows
        .checked_mul(cols)
        .ok_or_else(|| invalid("occupancy size overflow"))?;
    let mut occupancy = vec![0u8; size];
    let mut selected = Vec::with_capacity(points.len());
    for p in points.iter() {
        let cy = (p.y() * scaling + 16.0) as i64;
        let cx = (p.x() * scaling + 16.0) as i64;
        if cy < 15 || cx < 15 || cy + 15 >= rows as i64 || cx + 16 >= cols as i64 {
            return Err(invalid("keypoint occupancy support outside image"));
        }
        let (cy, cx) = (cy as usize, cx as usize);
        let normalized = p.score_ratio(max_score).sqrt().sqrt() * 255.0;
        if !normalized.is_finite() {
            return Err(invalid("uniformity normalized score is not finite"));
        }
        if f64::from(normalized) < f64::from(occupancy[cy * cols + cx]) {
            continue;
        }
        let nsc = 0.99f32 * normalized;
        for y in 0..31 {
            for x in 0..31 {
                let value = (lut.data[y * lut.cols + x] * nsc).ceil();
                if !value.is_finite() || !(0.0..=255.0).contains(&value) {
                    return Err(invalid("LUT mask cannot be represented as a byte"));
                }
                let at = (cy + y - 15) * cols + cx + x - 15;
                occupancy[at] = occupancy[at].saturating_add(value as u8);
            }
        }
        selected.push(p.clone());
        if selected.len() == max_num_keypoints {
            break;
        }
    }
    *points = selected;
    Ok(())
}
