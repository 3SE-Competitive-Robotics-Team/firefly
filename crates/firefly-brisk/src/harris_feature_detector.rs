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

//! Harris feature detector using the reference covariance, NMS and occupancy pipeline.
use crate::{
    harris_score_calculator::HarrisScoreCalculator,
    vectorized_filters::{check, filter_gauss_3_by_3_16s, invalid, sample, starts},
};
use firefly_error::Result;
use purecv::core::{Matrix, types::Point2f};
use purecv::features2d::KeyPoint;
pub struct HarrisFeatureDetector {
    pub radius: f64,
    pub lut: Matrix<f32>,
}
impl HarrisFeatureDetector {
    #[must_use]
    pub fn new(radius: f64) -> Self {
        let mut s = Self {
            radius,
            lut: Matrix::new(31, 31, 1),
        };
        s.set_radius(radius);
        s
    }
    pub fn set_radius(&mut self, radius: f64) {
        self.radius = radius;
        for x in 0..31 {
            for y in 0..31 {
                let value = 1.0
                    - ((radius / 2.0 - x as f64) * (radius / 2.0 - x as f64)
                        + (radius / 2.0 - y as f64) * (radius / 2.0 - y as f64))
                        / (radius / 2.0 * radius / 2.0);
                self.lut.data[y * 31 + x] = if value < 0.0 { 0.0 } else { value as f32 };
            }
        }
    }
    /// # Errors
    /// Returns an error for malformed image/score storage, mismatched covariance shapes,
    /// or invalid uniformity parameters.
    pub fn get_covar_entries(
        image: &Matrix<u8>,
    ) -> Result<(Matrix<i16>, Matrix<i16>, Matrix<i16>)> {
        check(image)?;
        let mut xx = Matrix::<i16>::new(image.rows, image.cols, 1);
        let mut yy = Matrix::<i16>::new(image.rows, image.cols, 1);
        let mut xy = Matrix::<i16>::new(image.rows, image.cols, 1);
        let kernel: [[i16; 3]; 3] = [[24, 0, -24], [80, 0, -80], [24, 0, -24]];
        for row in 0..image.rows.saturating_sub(2) {
            for block in starts(image.cols, 16, 16, 16)? {
                for lane in 0..16 {
                    let (mut dx, mut dy) = (0i16, 0i16);
                    for (x, horizontal_kernel) in kernel.iter().enumerate() {
                        for (y, vertical_kernel) in kernel.iter().enumerate() {
                            // The legacy masks unpack both byte lanes without a bit shift.
                            let pixel = i16::from(sample(image, row + y, block + x + lane)?);
                            dx = dx.wrapping_add(pixel.wrapping_mul(vertical_kernel[x]));
                            dy = dy.wrapping_add(pixel.wrapping_mul(horizontal_kernel[y]));
                        }
                    }
                    let at = (row + 1) * image.cols + block + 1 + lane;
                    xx.data[at] = ((i32::from(dx) * i32::from(dx)) >> 20) as i16;
                    yy.data[at] = ((i32::from(dy) * i32::from(dy)) >> 20) as i16;
                    xy.data[at] = ((i32::from(dx) * i32::from(dy)) >> 20) as i16;
                }
            }
        }
        Ok((xx, yy, xy))
    }
    /// # Errors
    /// Returns an error for malformed image/score storage, mismatched covariance shapes,
    /// or invalid uniformity parameters.
    pub fn corner_harris(
        xx: &Matrix<i16>,
        yy: &Matrix<i16>,
        xy: &Matrix<i16>,
    ) -> Result<Matrix<i32>> {
        HarrisScoreCalculator::corner_harris(xx, yy, xy)
    }
    /// # Errors
    /// Returns an error for malformed image/score storage, mismatched covariance shapes,
    /// or invalid uniformity parameters.
    pub fn nonmax_suppress(scores: &Matrix<i32>, points: &mut Vec<KeyPoint>) -> Result<()> {
        check(scores)?;
        for y in 2..scores.rows.saturating_sub(2) {
            for x in 0..scores.cols.saturating_sub(2) {
                let i = y * scores.cols + x;
                let c = scores.data[i];
                if c < 64 {
                    continue;
                }
                let offsets = [
                    1,
                    -1,
                    scores.cols as isize,
                    -(scores.cols as isize),
                    scores.cols as isize + 1,
                    scores.cols as isize - 1,
                    -(scores.cols as isize) + 1,
                    -(scores.cols as isize) - 1,
                ];
                if offsets
                    .iter()
                    .any(|d| scores.data[(i as isize + d) as usize] > c)
                {
                    continue;
                }
                points.push(KeyPoint::new(
                    Point2f::new((x + 1) as f32, y as f32),
                    10.0,
                    -1.0,
                    c as f32,
                    0,
                    -1,
                ));
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for malformed image/score storage, mismatched covariance shapes,
    /// or invalid uniformity parameters.
    pub fn enforce_uniformity(
        &self,
        scores: &Matrix<i32>,
        points: &mut Vec<KeyPoint>,
    ) -> Result<()> {
        check(scores)?;
        if points
            .iter()
            .any(|p| !p.response.is_finite() || !p.pt.x.is_finite() || !p.pt.y.is_finite())
        {
            return Err(invalid("finite Harris keypoints required"));
        }
        points.sort_unstable_by(|a, b| b.response.total_cmp(&a.response));
        let rows = scores.rows / 2 + 32;
        let cols = scores.cols / 2 + 32;
        let mut occupancy = Matrix::<u8>::new(rows, cols, 1);
        let mut selected = Vec::new();
        for p in points.iter() {
            let cy = (p.pt.x / 2.0 + 16.0) as i32;
            let cx = (p.pt.y / 2.0 + 16.0) as i32;
            if cy < 15 || cx < 15 || cy + 15 >= rows as i32 || cx + 16 >= cols as i32 {
                return Err(invalid(
                    "Harris occupancy support outside image (source swaps x/y)",
                ));
            }
            let center_response = f64::from(occupancy.data[cy as usize * cols + cx as usize]);
            let s1 = center_response * center_response;
            let narrowed = p.response as i32 as i16;
            if f64::from(narrowed) < s1 * s1 {
                continue;
            }
            let nsc = f64::from(p.response).sqrt().sqrt() as f32;
            for y in 0..31 {
                for x in 0..31 {
                    let mask = self.lut.data[y * 31 + x] * nsc;
                    if !mask.is_finite() {
                        return Err(invalid("Harris uniformity mask is not finite"));
                    }
                    let mask = mask as i32 as u8;
                    let at = (cy as usize + y - 15) * cols + cx as usize + x - 15;
                    occupancy.data[at] = occupancy.data[at].saturating_add(mask);
                }
            }
            selected.push(p.clone());
        }
        *points = selected;
        Ok(())
    }
    /// # Errors
    /// Returns an error for malformed image/score storage, mismatched covariance shapes,
    /// or invalid uniformity parameters.
    pub fn detect(
        &self,
        image: &Matrix<u8>,
        points: &mut Vec<KeyPoint>,
        _mask: Option<&Matrix<u8>>,
    ) -> Result<()> {
        check(image)?;
        points.clear();
        let (xx, yy, xy) = Self::get_covar_entries(image)?;
        let (xx, yy, xy) = (
            filter_gauss_3_by_3_16s(&xx)?,
            filter_gauss_3_by_3_16s(&yy)?,
            filter_gauss_3_by_3_16s(&xy)?,
        );
        let scores = Self::corner_harris(&xx, &yy, &xy)?;
        Self::nonmax_suppress(&scores, points)?;
        self.enforce_uniformity(&scores, points)
    }
}
