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

//! Floating Harris covariance, filtering and score interpolation.
use crate::{
    harris_score_calculator::{PointWithScore, interpolate},
    vectorized_filters::{check, filter_gauss_3_by_3_32f, invalid, starts},
};
use firefly_error::Result;
use purecv::core::{DynamicMatrix, Matrix};
use std::sync::Arc;
#[derive(Default)]
pub struct HarrisScoreCalculatorFloat {
    image: Option<Arc<DynamicMatrix>>,
    scores: Option<Matrix<f32>>,
}
impl HarrisScoreCalculatorFloat {
    /// # Errors
    /// Returns an error for unsupported image types, absent image/scores, invalid covariance
    /// shapes, or interpolation support outside the score image.
    pub fn set_image(&mut self, image: Arc<DynamicMatrix>, init_scores: bool) -> Result<()> {
        self.image = Some(image);
        if init_scores {
            self.initialize_scores()?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for unsupported image types, absent image/scores, invalid covariance
    /// shapes, or interpolation support outside the score image.
    pub fn initialize_scores(&mut self) -> Result<()> {
        let (xx, yy, xy) = Self::get_covar_entries(
            self.image
                .as_ref()
                .ok_or_else(|| invalid("Harris image absent"))?,
        )?;
        let a = filter_gauss_3_by_3_32f(&xx)?;
        let b = filter_gauss_3_by_3_32f(&yy)?;
        let c = filter_gauss_3_by_3_32f(&xy)?;
        self.scores = Some(Self::corner_harris(&a, &b, &c)?);
        Ok(())
    }
    #[must_use]
    pub fn scores(&self) -> Option<&Matrix<f32>> {
        self.scores.as_ref()
    }
    /// # Errors
    /// Returns an error for unsupported image types, absent image/scores, invalid covariance
    /// shapes, or interpolation support outside the score image.
    pub fn score_at(&self, u: usize, v: usize) -> Result<f32> {
        crate::vectorized_filters::sample(
            self.scores
                .as_ref()
                .ok_or_else(|| invalid("Harris scores absent"))?,
            v,
            u,
        )
    }
    /// # Errors
    /// Returns an error for unsupported image types, absent image/scores, invalid covariance
    /// shapes, or interpolation support outside the score image.
    pub fn score(&self, u: f64, v: f64) -> Result<f64> {
        interpolate(
            self.scores
                .as_ref()
                .ok_or_else(|| invalid("Harris scores absent"))?,
            u,
            v,
        )
    }
    /// # Errors
    /// Returns an error for unsupported image types, absent image/scores, invalid covariance
    /// shapes, or interpolation support outside the score image.
    pub fn get_2d_maxima(
        &self,
        points: &mut Vec<PointWithScore<f32>>,
        absolute_threshold: f32,
    ) -> Result<()> {
        let m = self
            .scores
            .as_ref()
            .ok_or_else(|| invalid("Harris scores absent"))?;
        for y in 2..m.rows.saturating_sub(2) {
            for x in 0..m.cols.saturating_sub(2) {
                let i = y * m.cols + x;
                let c = m.data[i];
                if c < absolute_threshold {
                    continue;
                }
                let ns = [
                    i + 1,
                    i - 1,
                    i + m.cols,
                    i - m.cols,
                    i + m.cols + 1,
                    i + m.cols - 1,
                    i - m.cols + 1,
                    i - m.cols - 1,
                ];
                if ns.into_iter().any(|n| m.data[n] > c) {
                    continue;
                }
                points.push(PointWithScore::new(c, x as u16, y as u16));
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for unsupported image types, absent image/scores, invalid covariance
    /// shapes, or interpolation support outside the score image.
    pub fn get_covar_entries(
        src: &DynamicMatrix,
    ) -> Result<(Matrix<f32>, Matrix<f32>, Matrix<f32>)> {
        if let Some(m) = src.as_matrix_u8() {
            check(m)?;
            Self::covariance(m.rows, m.cols, |y, x| f32::from(m.data[y * m.cols + x]))
        } else if let purecv::core::DynamicData::U16(m) = &src.data {
            check(m)?;
            Self::covariance(m.rows, m.cols, |y, x| f32::from(m.data[y * m.cols + x]))
        } else {
            Err(invalid("Harris float source must be grayscale u8 or u16"))
        }
    }
    fn covariance(
        rows: usize,
        cols: usize,
        get: impl Fn(usize, usize) -> f32,
    ) -> Result<(Matrix<f32>, Matrix<f32>, Matrix<f32>)> {
        let (mut xx, mut yy, mut xy) = (
            Matrix::new(rows, cols, 1),
            Matrix::new(rows, cols, 1),
            Matrix::new(rows, cols, 1),
        );
        let kernel = [
            [0.09375, 0.0, -0.09375],
            [0.3125, 0.0, -0.3125],
            [0.09375, 0.0, -0.09375],
        ];
        for i in 0..rows.saturating_sub(2) {
            for j in starts(cols, 4, 4, 4)? {
                for k in 0..4 {
                    let (mut dx, mut dy) = (0.0f32, 0.0f32);
                    for (x, horizontal_kernel) in kernel.iter().enumerate() {
                        for (y, vertical_kernel) in kernel.iter().enumerate() {
                            let v = get(i + y, j + x + k);
                            let mx = vertical_kernel[x];
                            let my = horizontal_kernel[y];
                            if mx != 0.0 {
                                dx += v * mx;
                            }
                            if my != 0.0 {
                                dy += v * my;
                            }
                        }
                    }
                    let at = i * cols + j + 1 + k;
                    xx.data[at] = dx * dx;
                    yy.data[at] = dy * dy;
                    xy.data[at] = dy * dx;
                }
            }
        }
        Ok((xx, yy, xy))
    }
    /// # Errors
    /// Returns an error for unsupported image types, absent image/scores, invalid covariance
    /// shapes, or interpolation support outside the score image.
    pub fn corner_harris(
        xx: &Matrix<f32>,
        yy: &Matrix<f32>,
        xy: &Matrix<f32>,
    ) -> Result<Matrix<f32>> {
        check(xx)?;
        check(yy)?;
        check(xy)?;
        if (xx.rows, xx.cols) != (yy.rows, yy.cols) || (xx.rows, xx.cols) != (xy.rows, xy.cols) {
            return Err(invalid("covariance image dimensions differ"));
        }
        let mut dst = Matrix::new(xx.rows, xx.cols, 1);
        for y in 0..xx.rows.saturating_sub(2) {
            for j in starts(xx.cols, 8, 4, 4)? {
                for k in 0..4 {
                    let at = y * xx.cols + j + k;
                    let a = xx.data[at];
                    let b = yy.data[at];
                    let c = xy.data[at];
                    let det = a * b - c * c;
                    let trace = a + b;
                    dst.data[at] = det - (trace * trace) * 0.0625;
                }
            }
        }
        Ok(dst)
    }
}
