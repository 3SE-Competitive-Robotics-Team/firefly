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

//! Integer Harris scoring, interpolation and eight-neighbour maxima.
use crate::{
    harris_scores::harris_scores_sse,
    vectorized_filters::{check, invalid, sample, starts},
};
use firefly_error::Result;
use purecv::core::Matrix;
use std::sync::Arc;
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointWithScore<S> {
    pub score: S,
    pub x: u16,
    pub y: u16,
}
impl<S> PointWithScore<S> {
    pub fn new(score: S, x: u16, y: u16) -> Self {
        Self { score, x, y }
    }
}
#[derive(Default)]
pub struct HarrisScoreCalculator {
    image: Option<Arc<Matrix<u8>>>,
    scores: Option<Matrix<i32>>,
}
impl HarrisScoreCalculator {
    /// # Errors
    /// Returns an error if the image/scores are absent, storage or covariance shapes are invalid,
    /// or the requested interpolation support lies outside the score image.
    pub fn set_image(&mut self, image: Arc<Matrix<u8>>, init_scores: bool) -> Result<()> {
        check(&image)?;
        self.image = Some(image);
        if init_scores {
            self.initialize_scores()?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if the image/scores are absent, storage or covariance shapes are invalid,
    /// or the requested interpolation support lies outside the score image.
    pub fn initialize_scores(&mut self) -> Result<()> {
        self.scores = Some(harris_scores_sse(
            self.image
                .as_ref()
                .ok_or_else(|| invalid("Harris image absent"))?,
        )?);
        Ok(())
    }
    #[must_use]
    pub fn scores(&self) -> Option<&Matrix<i32>> {
        self.scores.as_ref()
    }
    /// # Errors
    /// Returns an error if the image/scores are absent, storage or covariance shapes are invalid,
    /// or the requested interpolation support lies outside the score image.
    pub fn score_at(&self, u: usize, v: usize) -> Result<i32> {
        sample(
            self.scores
                .as_ref()
                .ok_or_else(|| invalid("Harris scores absent"))?,
            v,
            u,
        )
    }
    /// # Errors
    /// Returns an error if the image/scores are absent, storage or covariance shapes are invalid,
    /// or the requested interpolation support lies outside the score image.
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
    /// Returns an error if the image/scores are absent, storage or covariance shapes are invalid,
    /// or the requested interpolation support lies outside the score image.
    pub fn get_2d_maxima(
        &self,
        points: &mut Vec<PointWithScore<i32>>,
        absolute_threshold: i32,
    ) -> Result<()> {
        let scores = self
            .scores
            .as_ref()
            .ok_or_else(|| invalid("Harris scores absent"))?;
        for y in 2..scores.rows.saturating_sub(2) {
            for x in 2..scores.cols.saturating_sub(2) {
                let i = y * scores.cols + x;
                let center = scores.data[i];
                if center < absolute_threshold {
                    continue;
                }
                let neighbours = [
                    i + 1,
                    i - 1,
                    i + scores.cols,
                    i - scores.cols,
                    i + scores.cols + 1,
                    i + scores.cols - 1,
                    i - scores.cols + 1,
                    i - scores.cols - 1,
                ];
                if neighbours.into_iter().any(|n| scores.data[n] > center) {
                    continue;
                }
                points.push(PointWithScore::new(center, x as u16, y as u16));
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if the image/scores are absent, storage or covariance shapes are invalid,
    /// or the requested interpolation support lies outside the score image.
    pub fn get_covar_entries(src: &Matrix<u8>) -> Result<(Matrix<i16>, Matrix<i16>, Matrix<i16>)> {
        check(src)?;
        let n = src.data.len();
        let (mut xx, mut yy, mut xy) = (vec![0i16; n], vec![0i16; n], vec![0i16; n]);
        let kernel = [[24, 0, -24], [80, 0, -80], [24, 0, -24]];
        for i in 0..src.rows.saturating_sub(2) {
            for j in starts(src.cols, 16, 16, 16)? {
                let (mut hi_x, mut lo_x, mut hi_y, mut lo_y) =
                    ([0i16; 8], [0i16; 8], [0i16; 8], [0i16; 8]);
                for (x, horizontal_kernel) in kernel.iter().enumerate() {
                    for (y, vertical_kernel) in kernel.iter().enumerate() {
                        let mx = vertical_kernel[x];
                        let my = horizontal_kernel[y];
                        for k in 0..8 {
                            let hi =
                                (u16::from(sample(src, i + y, j + x + 2 * k + 1)?) << 8) as i16;
                            let lo = if k < 7 {
                                (u16::from(sample(src, i + y, j + x + 2 * k + 2)?) << 8) as i16
                            } else {
                                0
                            };
                            hi_x[k] = hi_x[k].wrapping_add(hi.wrapping_mul(mx));
                            lo_x[k] = lo_x[k].wrapping_add(lo.wrapping_mul(mx));
                            hi_y[k] = hi_y[k].wrapping_add(hi.wrapping_mul(my));
                            lo_y[k] = lo_y[k].wrapping_add(lo.wrapping_mul(my));
                        }
                    }
                }
                for k in 0..8 {
                    for (lane, (dx, dy)) in [(hi_x[k], hi_y[k]), (lo_x[k], lo_y[k])]
                        .into_iter()
                        .enumerate()
                    {
                        let at = (i + 1) * src.cols + j + 1 + 2 * k + lane;
                        xx[at] = ((i32::from(dx) * i32::from(dx)) >> 20) as i16;
                        yy[at] = ((i32::from(dy) * i32::from(dy)) >> 20) as i16;
                        xy[at] = ((i32::from(dx) * i32::from(dy)) >> 20) as i16;
                    }
                }
            }
        }
        Ok((
            Matrix::from_vec(src.rows, src.cols, 1, xx),
            Matrix::from_vec(src.rows, src.cols, 1, yy),
            Matrix::from_vec(src.rows, src.cols, 1, xy),
        ))
    }
    /// # Errors
    /// Returns an error if the image/scores are absent, storage or covariance shapes are invalid,
    /// or the requested interpolation support lies outside the score image.
    pub fn corner_harris(
        xx: &Matrix<i16>,
        yy: &Matrix<i16>,
        xy: &Matrix<i16>,
    ) -> Result<Matrix<i32>> {
        check(xx)?;
        check(yy)?;
        check(xy)?;
        if (xx.rows, xx.cols) != (yy.rows, yy.cols) || (xx.rows, xx.cols) != (xy.rows, xy.cols) {
            return Err(invalid("covariance image dimensions differ"));
        }
        let mut dst = Matrix::new(xx.rows, xx.cols, 1);
        for y in 0..xx.rows.saturating_sub(2) {
            for j in starts(xx.cols, 8, 8, 8)? {
                for k in 0..8 {
                    let at = y * xx.cols + j + k;
                    let a = xx.data[at];
                    let b = yy.data[at];
                    let c = xy.data[at];
                    let trace = (a >> 1).wrapping_add(b >> 1) >> 1;
                    dst.data[at] = i32::from(a)
                        .wrapping_mul(i32::from(b))
                        .wrapping_sub(i32::from(c).wrapping_mul(i32::from(c)))
                        .wrapping_sub(i32::from(trace).wrapping_mul(i32::from(trace)));
                }
            }
        }
        Ok(dst)
    }
}
pub(crate) fn interpolate<T: Copy + Into<f64>>(
    matrix: &Matrix<T>,
    left_vectors: f64,
    right_vectors: f64,
) -> Result<f64> {
    if !left_vectors.is_finite() || !right_vectors.is_finite() {
        return Err(invalid("Harris coordinates must be finite"));
    }
    let ui = left_vectors.trunc();
    let vi = right_vectors.trunc();
    if ui < 0.0 || vi < 0.0 || ui + 1.0 >= matrix.cols as f64 || vi + 1.0 >= matrix.rows as f64 {
        return Ok(0.0);
    }
    let (x, y) = (ui as usize, vi as usize);
    let (ru, rv) = (left_vectors - ui, right_vectors - vi);
    let i = y * matrix.cols + x;
    Ok(
        (1.0 - rv) * ((1.0 - ru) * matrix.data[i].into() + ru * matrix.data[i + 1].into())
            + rv * ((1.0 - ru) * matrix.data[i + matrix.cols].into()
                + ru * matrix.data[i + matrix.cols + 1].into()),
    )
}
