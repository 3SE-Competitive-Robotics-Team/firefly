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

//! BRISK 1/2 patterns, intensity integration, orientation and camera-aware descriptors.
use crate::{
    helper_structures::{BriskLongPair, BriskPatternPoint, BriskShortPair},
    integral_image::integral_image_8,
    pattern_provider::DEFAULT_PATTERN,
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use nalgebra::{Matrix2, SMatrix, SymmetricEigen, Vector3};
use purecv::{core::Matrix, features2d::KeyPoint};
use std::{path::Path, sync::Arc};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BriskVersion {
    V1,
    V2,
}
#[derive(Clone)]
pub struct CameraProperties {
    pub rays: Arc<Matrix<f32>>,
    pub image_jacobians: Arc<Matrix<Option<f32>>>,
    pub virtual_focal_length: f32,
}
pub struct BriskDescriptorExtractor {
    pub rotation_invariance: bool,
    pub scale_invariance: bool,
    pub points: usize,
    pub pattern_points: Vec<BriskPatternPoint>,
    pub scale_list: [f32; 64],
    pub size_list: [u32; 64],
    pub short_pairs: Vec<BriskShortPair>,
    pub long_pairs: Vec<BriskLongPair>,
    pub strings: usize,
    pub d_max: f32,
    pub d_min: f32,
    pub camera: Option<CameraProperties>,
    pub extraction_direction: Vector3<f32>,
}
impl BriskDescriptorExtractor {
    pub const BASIC_SIZE: f32 = 12.0;
    pub const SCALES: usize = 64;
    pub const ROTATIONS: usize = 1024;
    pub const SCALE_RANGE: f32 = 30.0;
    fn empty(rotation: bool, scale: bool) -> Self {
        Self {
            rotation_invariance: rotation,
            scale_invariance: scale,
            points: 0,
            pattern_points: Vec::new(),
            scale_list: [0.0; 64],
            size_list: [0; 64],
            short_pairs: Vec::new(),
            long_pairs: Vec::new(),
            strings: 0,
            d_max: 0.0,
            d_min: 0.0,
            camera: None,
            extraction_direction: Vector3::new(0.0, 1.0, 0.0),
        }
    }
    /// # Errors
    /// Returns an error if the pattern scale or generated pattern is invalid.
    pub fn new(
        rotation_invariant: bool,
        scale_invariant: bool,
        version: BriskVersion,
        pattern_scale: f32,
    ) -> Result<Self> {
        let mut s = Self::empty(rotation_invariant, scale_invariant);
        match version {
            BriskVersion::V2 => s.init_from_stream(DEFAULT_PATTERN, pattern_scale)?,
            BriskVersion::V1 => {
                let f = 0.85f64 * f64::from(pattern_scale);
                let radii = [
                    (f * 0.0) as f32,
                    (f * 2.9) as f32,
                    (f * 4.9) as f32,
                    (f * 7.4) as f32,
                    (f * 10.8) as f32,
                ];
                s.generate_kernel(&radii, &[1, 10, 14, 15, 20], 5.85, 8.2, &[])?;
            }
        }
        Ok(s)
    }
    /// # Errors
    /// Returns an error if the file cannot be read or its pattern data/scale is invalid.
    pub fn from_file(path: &Path, rotation: bool, scale: bool, pattern_scale: f32) -> Result<Self> {
        let input = std::fs::read_to_string(path)?;
        let mut s = Self::empty(rotation, scale);
        s.init_from_stream(&input, pattern_scale)?;
        Ok(s)
    }
    fn allocate_points(&mut self, points: usize) -> Result<()> {
        let total = points
            .checked_mul(Self::SCALES)
            .and_then(|v| v.checked_mul(Self::ROTATIONS))
            .ok_or_else(|| invalid("BRISK pattern allocation overflow"))?;
        self.points = points;
        self.pattern_points.clear();
        self.pattern_points
            .try_reserve_exact(total)
            .map_err(|e| invalid(&e.to_string()))?;
        self.pattern_points.resize(
            total,
            BriskPatternPoint {
                x: 0.0,
                y: 0.0,
                sigma: 0.0,
            },
        );
        Ok(())
    }
    fn initialize_scales(&mut self) {
        let lb = (f64::from(Self::SCALE_RANGE).ln() / 2.0f64.ln()) as f32;
        let step = lb / Self::SCALES as f32;
        for scale in 0..Self::SCALES {
            self.scale_list[scale] = 2.0f64.powf(f64::from(scale as f32 * step)) as f32;
            self.size_list[scale] = 0;
        }
    }
    /// # Errors
    /// Returns an error for incomplete/non-numeric pattern data, non-finite scale/points,
    /// invalid pair indices, zero-distance long pairs, or a short-pair count other than 384.
    pub fn init_from_stream(&mut self, input: &str, pattern_scale: f32) -> Result<()> {
        if !pattern_scale.is_finite() {
            return Err(invalid("finite pattern scale required"));
        }
        let mut tokens = input.split_ascii_whitespace();
        let mut next = || {
            tokens
                .next()
                .ok_or_else(|| invalid("incomplete BRISK pattern"))
        };
        let points = next()?
            .parse::<usize>()
            .map_err(|e| invalid(&e.to_string()))?;
        if points == 0 {
            return Err(invalid("nonempty BRISK pattern required"));
        }
        let mut unscaled = Vec::with_capacity(points);
        for _ in 0..points {
            let x = next()?
                .parse::<f32>()
                .map_err(|e| invalid(&e.to_string()))?
                * pattern_scale;
            let y = next()?
                .parse::<f32>()
                .map_err(|e| invalid(&e.to_string()))?
                * pattern_scale;
            let sigma = next()?
                .parse::<f32>()
                .map_err(|e| invalid(&e.to_string()))?
                * pattern_scale;
            if !x.is_finite() || !y.is_finite() || !sigma.is_finite() {
                return Err(invalid("finite BRISK pattern point required"));
            }
            unscaled.push(BriskPatternPoint { x, y, sigma });
        }
        self.allocate_points(points)?;
        self.initialize_scales();
        self.d_min = 0.0;
        self.d_max = 0.0;
        self.expand_pattern(&unscaled)?;
        let short_count = next()?
            .parse::<usize>()
            .map_err(|e| invalid(&e.to_string()))?;
        self.short_pairs.clear();
        for _ in 0..short_count {
            let i = next()?
                .parse::<u32>()
                .map_err(|e| invalid(&e.to_string()))?;
            let j = next()?
                .parse::<u32>()
                .map_err(|e| invalid(&e.to_string()))?;
            if i as usize >= points || j as usize >= points {
                return Err(invalid("BRISK short-pair point index outside pattern"));
            }
            self.short_pairs.push(BriskShortPair { i, j });
        }
        let long_count = next()?
            .parse::<usize>()
            .map_err(|e| invalid(&e.to_string()))?;
        self.long_pairs.clear();
        for _ in 0..long_count {
            let i = next()?
                .parse::<u32>()
                .map_err(|e| invalid(&e.to_string()))?;
            let j = next()?
                .parse::<u32>()
                .map_err(|e| invalid(&e.to_string()))?;
            let a = unscaled
                .get(i as usize)
                .ok_or_else(|| invalid("BRISK long-pair index outside pattern"))?;
            let b = unscaled
                .get(j as usize)
                .ok_or_else(|| invalid("BRISK long-pair index outside pattern"))?;
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let norm = dx * dx + dy * dy;
            if norm == 0.0 {
                return Err(invalid("zero-distance BRISK long pair"));
            }
            self.long_pairs.push(BriskLongPair {
                i,
                j,
                weighted_dx: (f64::from(dx / norm) * 2048.0 + 0.5) as i32,
                weighted_dy: (f64::from(dy / norm) * 2048.0 + 0.5) as i32,
            });
        }
        self.strings = ((short_count as f32 / 128.0).ceil() as usize) * 16;
        if short_count != 384 {
            return Err(invalid("short pairs must equal descriptor length 384"));
        }
        Ok(())
    }
    fn expand_pattern(&mut self, unscaled: &[BriskPatternPoint]) -> Result<()> {
        let sigma_scale = 1.3f32;
        for scale in 0..Self::SCALES {
            for rot in 0..Self::ROTATIONS {
                let theta = rot as f64 * 2.0 * std::f64::consts::PI / Self::ROTATIONS as f64;
                for (i, p) in unscaled.iter().enumerate() {
                    let x = (f64::from(self.scale_list[scale])
                        * (f64::from(p.x) * theta.cos() - f64::from(p.y) * theta.sin()))
                        as f32;
                    let y = (f64::from(self.scale_list[scale])
                        * (f64::from(p.x) * theta.sin() + f64::from(p.y) * theta.cos()))
                        as f32;
                    let sigma = sigma_scale * self.scale_list[scale] * p.sigma;
                    let size =
                        checked_pattern_size(f64::from(x * x + y * y).sqrt() + f64::from(sigma))?;
                    self.size_list[scale] = self.size_list[scale].max(size);
                    self.pattern_points[(scale * Self::ROTATIONS + rot) * self.points + i] =
                        BriskPatternPoint { x, y, sigma };
                }
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for incompatible rings/counts, overflow, invalid pattern sizes,
    /// or invalid short-pair remapping.
    pub fn generate_kernel(
        &mut self,
        radii: &[f32],
        counts: &[i32],
        d_max: f32,
        d_min: f32,
        index_change: &[usize],
    ) -> Result<()> {
        if radii.is_empty() || radii.len() != counts.len() || counts.iter().any(|n| *n < 0) {
            return Err(invalid("BRISK ring counts/radii mismatch"));
        }
        let points = counts
            .iter()
            .try_fold(0usize, |v, n| v.checked_add(*n as usize))
            .ok_or_else(|| invalid("BRISK point count overflow"))?;
        if points == 0 {
            return Err(invalid("nonempty BRISK rings required"));
        }
        self.allocate_points(points)?;
        self.initialize_scales();
        self.d_max = d_max;
        self.d_min = d_min;
        let sigma_scale = 1.3f32;
        for scale in 0..Self::SCALES {
            for rot in 0..Self::ROTATIONS {
                let theta = rot as f64 * 2.0 * std::f64::consts::PI / Self::ROTATIONS as f64;
                let mut i = 0;
                for (ring, (&radius, &count)) in radii.iter().zip(counts).enumerate() {
                    for num in 0..count {
                        let alpha = f64::from(num) * 2.0 * std::f64::consts::PI / f64::from(count);
                        let x = (f64::from(self.scale_list[scale] * radius) * (alpha + theta).cos())
                            as f32;
                        let y = (f64::from(self.scale_list[scale] * radius) * (alpha + theta).sin())
                            as f32;
                        let sigma = if ring == 0 {
                            (f64::from(sigma_scale * self.scale_list[scale]) * 0.5) as f32
                        } else {
                            (f64::from(sigma_scale * self.scale_list[scale])
                                * f64::from(radius)
                                * (std::f64::consts::PI / f64::from(count)).sin())
                                as f32
                        };
                        let size = checked_pattern_size(f64::from(
                            (self.scale_list[scale] * radius) + sigma,
                        ))?;
                        self.size_list[scale] = self.size_list[scale].max(size);
                        self.pattern_points[(scale * Self::ROTATIONS + rot) * points + i] =
                            BriskPatternPoint { x, y, sigma };
                        i += 1;
                    }
                }
            }
        }
        let max_pairs = points
            .checked_mul(points - 1)
            .ok_or_else(|| invalid("BRISK pair count overflow"))?
            / 2;
        let mut short = vec![None; max_pairs];
        let mut short_count = 0;
        self.long_pairs.clear();
        for i in 1..points {
            for j in 0..i {
                let dx = self.pattern_points[j].x - self.pattern_points[i].x;
                let dy = self.pattern_points[j].y - self.pattern_points[i].y;
                let norm = dx * dx + dy * dy;
                if norm > d_min * d_min {
                    self.long_pairs.push(BriskLongPair {
                        i: i as u32,
                        j: j as u32,
                        weighted_dx: (f64::from(dx / norm) * 2048.0 + 0.5) as i32,
                        weighted_dy: (f64::from(dy / norm) * 2048.0 + 0.5) as i32,
                    });
                }
                if norm < d_max * d_max {
                    let index = if index_change.is_empty() {
                        short_count
                    } else {
                        *index_change
                            .get(short_count)
                            .ok_or_else(|| invalid("short-pair permutation too short"))?
                    };
                    *short
                        .get_mut(index)
                        .ok_or_else(|| invalid("short-pair permutation outside storage"))? =
                        Some(BriskShortPair {
                            i: i as u32,
                            j: j as u32,
                        });
                    short_count += 1;
                }
            }
        }
        self.short_pairs = short
            .into_iter()
            .take(short_count)
            .map(|p| p.ok_or_else(|| invalid("short-pair permutation leaves unwritten entries")))
            .collect::<Result<_>>()?;
        self.strings = ((short_count as f32 / 128.0).ceil() as usize) * 16;
        Ok(())
    }
    /// # Errors
    /// Returns an error if ray/Jacobian storage, channel counts, map shapes,
    /// or virtual focal length are invalid.
    pub fn set_camera_properties(
        &mut self,
        rays: Arc<Matrix<f32>>,
        image_jacobians: Arc<Matrix<Option<f32>>>,
        virtual_focal_length: f32,
    ) -> Result<()> {
        if (!rays.data.is_empty() && rays.channels != 3)
            || (!image_jacobians.data.is_empty() && image_jacobians.channels != 6)
            || rays
                .rows
                .checked_mul(rays.cols)
                .and_then(|n| n.checked_mul(rays.channels))
                != Some(rays.data.len())
            || image_jacobians
                .rows
                .checked_mul(image_jacobians.cols)
                .and_then(|n| n.checked_mul(image_jacobians.channels))
                != Some(image_jacobians.data.len())
        {
            return Err(invalid("invalid camera ray/Jacobian storage"));
        }
        self.camera = Some(CameraProperties {
            rays,
            image_jacobians,
            virtual_focal_length,
        });
        Ok(())
    }
    pub fn set_extraction_direction(&mut self, direction: Vector3<f32>) {
        self.extraction_direction = direction;
    }
    #[must_use]
    pub fn is_camera_aware(&self) -> bool {
        self.camera
            .as_ref()
            .is_some_and(|c| !c.rays.data.is_empty())
    }
    #[must_use]
    pub fn descriptor_size(&self) -> usize {
        self.strings
    }
    #[must_use]
    pub fn descriptor_type(&self) -> i32 {
        0
    }
    /// # Errors
    /// Returns an error if the descriptor allocation size overflows.
    pub fn allocate_descriptors(&self, count: usize) -> Result<Matrix<u8>> {
        count
            .checked_mul(self.strings)
            .ok_or_else(|| invalid("descriptor allocation overflow"))?;
        Ok(Matrix::new(count, self.strings, 1))
    }
    /// # Errors
    /// Returns an error if the descriptor shape, keypoint index, or pattern-value indices are invalid.
    pub fn set_descriptor_bits(
        &self,
        keypoint: usize,
        values: &[i32],
        descriptors: &mut Matrix<u8>,
    ) -> Result<()> {
        check(descriptors)?;
        if values.len() != self.points
            || descriptors.cols != self.strings
            || keypoint >= descriptors.rows
            || self.short_pairs.len() > self.strings * 8
        {
            return Err(invalid("BRISK descriptor/pattern dimensions differ"));
        }
        let row = &mut descriptors.data[keypoint * self.strings..(keypoint + 1) * self.strings];
        for (bit, pair) in self.short_pairs.iter().enumerate() {
            if values[pair.i as usize] > values[pair.j as usize] {
                let word = bit / 32;
                let shifted = 1u32 << (bit % 32);
                let mut bytes = [0u8; 4];
                bytes.copy_from_slice(&row[word * 4..word * 4 + 4]);
                let old = u32::from_ne_bytes(bytes);
                row[word * 4..word * 4 + 4].copy_from_slice(&(old | shifted).to_ne_bytes());
            }
        }
        Ok(())
    }
    fn camera_warp(&self, point: &mut KeyPoint) -> Result<(Option<[f32; 4]>, f32, bool)> {
        let Some(camera) = &self.camera else {
            return Ok((None, 1.0, false));
        };
        if camera.image_jacobians.data.is_empty() {
            return Ok((None, 1.0, false));
        }
        let (x0, y0) = (point.pt.x.floor() as i32, point.pt.y.floor() as i32);
        let rays = &camera.rays;
        if x0 < 0 || y0 < 0 || x0 + 1 >= rays.cols as i32 || y0 + 1 >= rays.rows as i32 {
            return Err(invalid("camera-awareness interpolation outside ray map"));
        }
        let (dx, dy) = (point.pt.x - x0 as f32, point.pt.y - y0 as f32);
        let weights = [
            (1.0f32 - dx) * (1.0f32 - dy),
            dx * (1.0f32 - dy),
            (1.0f32 - dx) * dy,
            dx * dy,
        ];
        let mut directions = [Vector3::<f32>::zeros(); 4];
        for (i, (x, y)) in [(x0, y0), (x0 + 1, y0), (x0, y0 + 1), (x0 + 1, y0 + 1)]
            .into_iter()
            .enumerate()
        {
            let at = (y as usize * rays.cols + x as usize) * 3;
            directions[i] = Vector3::from_row_slice(&rays.data[at..at + 3]);
            let norm = directions[i].dot(&directions[i]);
            if f64::from(norm).partial_cmp(&1e-12) != Some(std::cmp::Ordering::Greater) {
                return Ok((None, 1.0, false));
            }
        }
        let direction = weights[0] * directions[0]
            + weights[1] * directions[1]
            + weights[2] * directions[2]
            + weights[3] * directions[3];
        let mut eu = self.extraction_direction.cross(&direction);
        let directional = f64::from(eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2]) > 0.01;
        eu = (1.0f32 / camera.virtual_focal_length) * cv_normalized(eu);
        let ev = (1.0f32 / camera.virtual_focal_length) * cv_normalized(direction.cross(&eu));
        let (xm, ym) = (point.pt.x.round() as i32, point.pt.y.round() as i32);
        let j = &camera.image_jacobians;
        if xm < 0 || ym < 0 || xm >= j.cols as i32 || ym >= j.rows as i32 {
            return Err(invalid("camera-awareness Jacobian outside map"));
        }
        let at = (ym as usize * j.cols + xm as usize) * 6;
        let mut values = [0f32; 6];
        for (i, value) in values.iter_mut().enumerate() {
            *value =
                j.data[at + i].ok_or_else(|| invalid("camera-awareness Jacobian unimplemented"))?;
        }
        let jacobian = SMatrix::<f32, 2, 3>::from_row_slice(&values);
        let axes = SMatrix::<f32, 3, 2>::from_columns(&[eu, ev]);
        let warp = jacobian * axes;
        if warp.iter().any(|v| !v.is_finite()) {
            return Err(invalid("non-finite camera warp"));
        }
        let eigenvalues = SymmetricEigen::new(Matrix2::new(
            warp[(0, 0)],
            warp[(0, 1)],
            warp[(1, 0)],
            warp[(1, 1)],
        ))
        .eigenvalues;
        let sigma_scale = (0.5f64 * f64::from(eigenvalues[0].abs() + eigenvalues[1].abs())) as f32;
        let warp = [warp[(0, 0)], warp[(0, 1)], warp[(1, 0)], warp[(1, 1)]];
        if directional {
            point.angle = (f64::from(warp[3]).atan2(f64::from(warp[1])) / std::f64::consts::PI
                * 180.0) as f32;
        }
        Ok((Some(warp), sigma_scale, directional))
    }
    /// # Errors
    /// Returns an error for malformed pattern/image storage, invalid pattern indices,
    /// non-finite sampling geometry, or support outside the available image/integral map.
    pub fn compute(&self, image: &Matrix<u8>, keypoints: &mut Vec<KeyPoint>) -> Result<Matrix<u8>> {
        check(image)?;
        let log2 = std::f32::consts::LN_2;
        let lb = (f64::from(Self::SCALE_RANGE).ln() / f64::from(log2)) as f32;
        let basic06 = (f64::from(Self::BASIC_SIZE) * 0.6) as f32;
        let basicscale = if self.scale_invariance {
            0
        } else {
            (f64::from(Self::SCALES as f32 / lb)
                * ((1.45 * f64::from(Self::BASIC_SIZE) / f64::from(basic06)).ln()
                    / f64::from(log2))
                + 0.5) as i32
        };
        let mut scales = Vec::new();
        let mut valid = Vec::new();
        for p in keypoints.iter() {
            if !p.pt.x.is_finite() || !p.pt.y.is_finite() || !p.size.is_finite() || p.size <= 0.0 {
                return Err(invalid("finite positive BRISK keypoint size required"));
            }
            let scale = if self.scale_invariance {
                let n = f64::from(Self::SCALES as f32 / lb)
                    * (f64::from(p.size / basic06).ln() / f64::from(log2));
                ((n + 0.5) as i32).max(0).min(Self::SCALES as i32 - 1) as usize
            } else {
                basicscale.max(0) as usize
            };
            let border = *self
                .size_list
                .get(scale)
                .ok_or_else(|| invalid("BRISK scale outside pattern"))?
                as i32;
            if p.pt.x < border as f32
                || p.pt.x >= (image.cols as i32 - border) as f32
                || p.pt.y < border as f32
                || p.pt.y >= (image.rows as i32 - border) as f32
            {
                continue;
            }
            valid.push(p.clone());
            scales.push(scale);
        }
        *keypoints = valid;
        let mut descriptors = self.allocate_descriptors(keypoints.len())?;
        let integral = integral_image_8(image)?;
        let mut values = vec![0i32; self.points];
        for (k, point) in keypoints.iter_mut().enumerate() {
            let scale = scales[k];
            let (warp, sigma_scale, directional) = self.camera_warp(point)?;
            let mut theta = 0i32;
            if point.angle == -1.0 {
                if self.rotation_invariance {
                    for (i, value) in values.iter_mut().enumerate() {
                        *value = self.smoothed_intensity(
                            image,
                            &integral,
                            [point.pt.x, point.pt.y],
                            [scale, 0, i],
                            None,
                            1.0,
                        )?;
                    }
                    let (mut d0, mut d1) = (0i32, 0i32);
                    for pair in &self.long_pairs {
                        let delta = values[pair.i as usize].wrapping_sub(values[pair.j as usize]);
                        d0 = d0.wrapping_add(delta.wrapping_mul(pair.weighted_dx) / 1024);
                        d1 = d1.wrapping_add(delta.wrapping_mul(pair.weighted_dy) / 1024);
                    }
                    point.angle = (f64::from(d1 as f32).atan2(f64::from(d0 as f32))
                        / std::f64::consts::PI
                        * 180.0) as f32;
                    theta = (f64::from(Self::ROTATIONS as f32 * point.angle) / 360.0 + 0.5) as i32;
                }
            } else if self.rotation_invariance {
                theta = (Self::ROTATIONS as f64 * (f64::from(point.angle) / 360.0) + 0.5) as i32;
            }
            if theta < 0 {
                theta += Self::ROTATIONS as i32;
            }
            if theta >= Self::ROTATIONS as i32 {
                theta -= Self::ROTATIONS as i32;
            }
            if theta < 0 || theta >= Self::ROTATIONS as i32 {
                return Err(invalid("BRISK angle remains outside rotation table"));
            }
            for (i, value) in values.iter_mut().enumerate() {
                *value = self.smoothed_intensity(
                    image,
                    &integral,
                    [point.pt.x, point.pt.y],
                    [scale, if directional { 0 } else { theta as usize }, i],
                    warp.as_ref(),
                    sigma_scale,
                )?;
            }
            self.set_descriptor_bits(k, &values, &mut descriptors)?;
        }
        Ok(descriptors)
    }
    /// # Errors
    /// Returns an error for malformed pattern/image storage, invalid pattern indices,
    /// non-finite sampling geometry, or support outside the available image/integral map.
    pub fn smoothed_intensity<P: IntensityPixel, R: IntensityScalar>(
        &self,
        image: &Matrix<P>,
        integral: &Matrix<R>,
        key: [f32; 2],
        pattern_index: [usize; 3],
        warp: Option<&[f32; 4]>,
        warp_scale: f32,
    ) -> Result<R> {
        let [key_x, key_y] = key;
        let [scale, rot, point] = pattern_index;

        check(image)?;
        check(integral)?;
        if scale >= Self::SCALES
            || rot >= Self::ROTATIONS
            || point >= self.points
            || integral.cols != image.cols + 1
            || integral.rows != image.rows + 1
        {
            return Err(invalid("BRISK intensity pattern/integral shape mismatch"));
        }
        let mut point_value =
            self.pattern_points[(scale * Self::ROTATIONS + rot) * self.points + point];
        if let Some(w) = warp {
            let base = point_value;
            point_value.x = w[0] * base.x + w[1] * base.y;
            point_value.y = w[2] * base.x + w[3] * base.y;
            point_value.sigma = warp_scale * base.sigma;
        }
        let (xf, yf) = (point_value.x + key_x, point_value.y + key_y);
        if !xf.is_finite() || !yf.is_finite() || !point_value.sigma.is_finite() {
            return Err(invalid("non-finite BRISK intensity coordinates"));
        }
        let (x, y) = (xf as i32, yf as i32);
        let sigma = point_value.sigma;
        let area = (4.0f64 * f64::from(sigma) * f64::from(sigma)) as f32;
        if f64::from(sigma) < 0.5 {
            return sample_bilinear(image, [xf, yf], [x, y]);
        }
        let (xlo, xhi, ylo, yhi) = (xf - sigma, xf + sigma, yf - sigma, yf + sigma);
        if xlo < 0.0
            || xhi > (image.cols as i32 - 1) as f32
            || ylo < 0.0
            || yhi > (image.rows as i32 - 1) as f32
        {
            return Ok(R::from_i32(-1));
        }
        let scaling = R::from_f64(4_194_304.0 / f64::from(area));
        let scaling2 = R::from_f64(f64::from(scaling.to_f32() * area) / 1024.0);
        let (xl, yt, xr, yb) = (
            (f64::from(xlo) + 0.5) as i32,
            (f64::from(ylo) + 0.5) as i32,
            (f64::from(xhi) + 0.5) as i32,
            (f64::from(yhi) + 0.5) as i32,
        );
        let (left_fraction, top_fraction, right_fraction, bottom_fraction) = (
            (f64::from(xl as f32 - xlo) + 0.5) as f32,
            (f64::from(yt as f32 - ylo) + 0.5) as f32,
            (f64::from(xhi - xr as f32) + 0.5) as f32,
            (f64::from(yhi - yb as f32) + 0.5) as f32,
        );
        let (dx, dy) = (xr - xl - 1, yb - yt - 1);
        let (leading, quadratic, linear, constant) = (
            R::from_f32(left_fraction * top_fraction * scaling.to_f32()),
            R::from_f32(right_fraction * top_fraction * scaling.to_f32()),
            R::from_f32(right_fraction * bottom_fraction * scaling.to_f32()),
            R::from_f32(left_fraction * bottom_fraction * scaling.to_f32()),
        );
        let (left_weight, top_weight, right_weight, bottom_weight) = (
            R::from_f32(left_fraction * scaling.to_f32()),
            R::from_f32(top_fraction * scaling.to_f32()),
            R::from_f32(right_fraction * scaling.to_f32()),
            R::from_f32(bottom_fraction * scaling.to_f32()),
        );
        let support = IntensitySupport {
            bounds: [xl, yt, xr, yb],
            corners: [leading, quadratic, linear, constant],
            edges: [left_weight, top_weight, right_weight, bottom_weight],
            scaling,
            normalizer: scaling2,
        };
        if dx + dy > 2 {
            sample_integral(image, integral, support)
        } else {
            sample_small_support(image, support)
        }
    }
}
fn checked_pattern_size(value: f64) -> Result<u32> {
    let size = value.ceil();
    if !size.is_finite() || size < 0.0 || size >= f64::from(u32::MAX) {
        return Err(invalid("BRISK pattern radius outside unsigned size range"));
    }
    Ok(size as u32 + 1)
}
fn cv_normalized(v: Vector3<f32>) -> Vector3<f32> {
    let n = (f64::from(v[0]) * f64::from(v[0])
        + f64::from(v[1]) * f64::from(v[1])
        + f64::from(v[2]) * f64::from(v[2]))
    .sqrt();
    let scale = if n > f64::EPSILON { 1.0 / n } else { 0.0 };
    Vector3::new(
        (f64::from(v[0]) * scale) as f32,
        (f64::from(v[1]) * scale) as f32,
        (f64::from(v[2]) * scale) as f32,
    )
}
/// Scalar operations retain the reference integral-image pixel arithmetic.
pub trait IntensityScalar: Copy + Default {
    fn from_i32(v: i32) -> Self;
    fn from_f32(v: f32) -> Self;
    fn from_f64(v: f64) -> Self;
    fn to_i32(self) -> i32;
    fn to_f32(self) -> f32;
    #[must_use]
    fn add(self, b: Self) -> Self;
    #[must_use]
    fn sub(self, b: Self) -> Self;
    #[must_use]
    fn mul(self, b: Self) -> Self;
    /// # Errors
    /// Returns an error if integer division has a zero divisor or overflows.
    fn divide(self, b: Self) -> Result<Self>;
}
impl IntensityScalar for i32 {
    fn from_i32(v: i32) -> Self {
        v
    }
    fn from_f32(v: f32) -> Self {
        v as i32
    }
    fn from_f64(v: f64) -> Self {
        v as i32
    }
    fn to_i32(self) -> i32 {
        self
    }
    fn to_f32(self) -> f32 {
        self as f32
    }
    fn add(self, b: Self) -> Self {
        self.wrapping_add(b)
    }
    fn sub(self, b: Self) -> Self {
        self.wrapping_sub(b)
    }
    fn mul(self, b: Self) -> Self {
        self.wrapping_mul(b)
    }
    fn divide(self, b: Self) -> Result<Self> {
        self.checked_div(b)
            .ok_or_else(|| invalid("invalid BRISK integer intensity divisor"))
    }
}
impl IntensityScalar for f32 {
    fn from_i32(v: i32) -> Self {
        v as f32
    }
    fn from_f32(v: f32) -> Self {
        v
    }
    fn from_f64(v: f64) -> Self {
        v as f32
    }
    fn to_i32(self) -> i32 {
        self as i32
    }
    fn to_f32(self) -> f32 {
        self
    }
    fn add(self, b: Self) -> Self {
        self + b
    }
    fn sub(self, b: Self) -> Self {
        self - b
    }
    fn mul(self, b: Self) -> Self {
        self * b
    }
    fn divide(self, b: Self) -> Result<Self> {
        Ok(self / b)
    }
}
pub trait IntensityPixel: Copy {
    fn to_f64(self) -> f64;
}
impl IntensityPixel for u8 {
    fn to_f64(self) -> f64 {
        f64::from(self)
    }
}
impl IntensityPixel for f32 {
    fn to_f64(self) -> f64 {
        f64::from(self)
    }
}

fn sample_bilinear<P: IntensityPixel, R: IntensityScalar>(
    image: &Matrix<P>,
    position: [f32; 2],
    indices: [i32; 2],
) -> Result<R> {
    let [xf, yf] = position;
    let [x, y] = indices;
    let mut ret = 0i32;

    if x < 0 || x > image.cols as i32 - 2 || y < 0 || y > image.rows as i32 - 2 {
        return Ok(R::from_i32(-1));
    }
    let (rx, ry) = (
        ((xf - x as f32) * 1024.0) as i32,
        ((yf - y as f32) * 1024.0) as i32,
    );
    for (weight, xx, yy) in [
        ((1024 - rx) * (1024 - ry), x, y),
        (rx * (1024 - ry), x + 1, y),
        (rx * ry, x + 1, y + 1),
        ((1024 - rx) * ry, x, y + 1),
    ] {
        ret = R::from_i32(ret)
            .add(R::from_i32(weight).mul(intensity_pixel(image, xx, yy)?))
            .to_i32();
    }
    Ok(R::from_i32(ret / 1024))
}

#[derive(Clone, Copy)]
struct IntensitySupport<R> {
    bounds: [i32; 4],
    corners: [R; 4],
    edges: [R; 4],
    scaling: R,
    normalizer: R,
}
fn sample_integral<P: IntensityPixel, R: IntensityScalar>(
    image: &Matrix<P>,
    integral: &Matrix<R>,
    support: IntensitySupport<R>,
) -> Result<R> {
    let IntensitySupport {
        bounds: [xl, yt, xr, yb],
        corners: [a, b, c, d],
        edges: [left_weight, top_weight, right_weight, bottom_weight],
        scaling,
        normalizer: scaling2,
    } = support;
    let (dx, dy) = (xr - xl - 1, yb - yt - 1);
    let mut ret = 0i32;

    let first = xl as usize + image.cols * yt as usize;
    let second = first + (dx + 1) as usize;
    let third = second + dy as usize * image.cols + 1;
    let fourth = third - (dx + 1) as usize;
    for (weight, at) in [(a, first), (b, second), (c, third), (d, fourth)] {
        let value = image
            .data
            .get(at)
            .ok_or_else(|| invalid("BRISK corner pointer outside image"))?;
        ret = R::from_i32(ret)
            .add(weight.mul(R::from_f64(value.to_f64())))
            .to_i32();
    }
    let mut ptr = i64::from(xl) + integral.cols as i64 * i64::from(yt) + 1;
    let mut tmp = [R::from_i32(0); 12];
    for (i, advance) in [
        0,
        i64::from(dx),
        integral.cols as i64,
        1,
        i64::from(dy) * integral.cols as i64,
        -1,
        integral.cols as i64,
        -i64::from(dx),
        -(integral.cols as i64),
        -1,
        -i64::from(dy) * integral.cols as i64,
        1,
    ]
    .into_iter()
    .enumerate()
    {
        ptr += advance;
        tmp[i] = *integral
            .data
            .get(usize::try_from(ptr).map_err(|_| invalid("negative BRISK integral offset"))?)
            .ok_or_else(|| invalid("BRISK integral support outside image"))?;
    }
    let upper = tmp[2].sub(tmp[1]).add(tmp[0]).sub(tmp[11]).mul(top_weight);
    let middle = tmp[5].sub(tmp[2]).add(tmp[11]).sub(tmp[8]).mul(scaling);
    let left = tmp[8]
        .sub(tmp[11])
        .add(tmp[10])
        .sub(tmp[9])
        .mul(left_weight);
    let right = tmp[4].sub(tmp[3]).add(tmp[2]).sub(tmp[5]).mul(right_weight);
    let bottom = tmp[6]
        .sub(tmp[5])
        .add(tmp[8])
        .sub(tmp[7])
        .mul(bottom_weight);
    R::from_i32(ret)
        .add(upper)
        .add(middle)
        .add(left)
        .add(right)
        .add(bottom)
        .divide(scaling2)
}

fn intensity_pixel<P: IntensityPixel, R: IntensityScalar>(
    image: &Matrix<P>,
    x: i32,
    y: i32,
) -> Result<R> {
    if x < 0 || y < 0 || x >= image.cols as i32 || y >= image.rows as i32 {
        return Err(invalid("BRISK intensity sample outside image"));
    }
    Ok(R::from_f64(
        image.data[y as usize * image.cols + x as usize].to_f64(),
    ))
}

fn sample_small_support<P: IntensityPixel, R: IntensityScalar>(
    image: &Matrix<P>,
    support: IntensitySupport<R>,
) -> Result<R> {
    let IntensitySupport {
        bounds: [xl, yt, xr, yb],
        corners: [leading, quadratic, linear, constant],
        edges: [left_weight, top_weight, right_weight, bottom_weight],
        scaling,
        normalizer: scaling2,
    } = support;
    let mut ret = 0i32;
    for yy in yt..=yb {
        for xx in xl..=xr {
            let weight = if yy == yt {
                if xx == xl {
                    leading
                } else if xx == xr {
                    quadratic
                } else {
                    top_weight
                }
            } else if yy == yb {
                if xx == xl {
                    constant
                } else if xx == xr {
                    linear
                } else {
                    bottom_weight
                }
            } else if xx == xl {
                left_weight
            } else if xx == xr {
                right_weight
            } else {
                scaling
            };
            ret = R::from_i32(ret)
                .add(weight.mul(intensity_pixel(image, xx, yy)?))
                .to_i32();
        }
    }
    R::from_i32(ret).divide(scaling2)
}
