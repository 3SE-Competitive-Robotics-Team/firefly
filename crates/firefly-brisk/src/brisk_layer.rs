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

//! AGAST image-pyramid layer with lazy score caching and reference sampling.
use crate::{
    agast::{agast5_8::AgastDetector58, oast9_16::OastDetector916},
    image_down_sampling::{halfsample_8, twothirdsample_8},
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use purecv::{core::Matrix, features2d::KeyPoint};
use std::cell::RefCell;
pub const USE_THRESHOLD_MAP: bool = false;
#[derive(Clone, Copy, Debug)]
pub enum Sampling {
    Half,
    TwoThirds,
}
pub struct BriskLayer {
    pub image: Matrix<u8>,
    pub scores: RefCell<Matrix<u8>>,
    pub threshold_map: Option<Matrix<u8>>,
    pub scale: f32,
    pub offset: f32,
    pub upper_threshold: u8,
    pub lower_threshold: u8,
    oast: RefCell<OastDetector916>,
    agast58: RefCell<AgastDetector58>,
}
impl BriskLayer {
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn new(
        image: Matrix<u8>,
        upper_threshold: u8,
        lower_threshold: u8,
        scale: f32,
        offset: f32,
    ) -> Result<Self> {
        check(&image)?;
        let (rows, cols) = (image.rows, image.cols);
        let mut layer = Self {
            image,
            scores: RefCell::new(Matrix::new(rows, cols, 1)),
            threshold_map: None,
            scale,
            offset,
            upper_threshold,
            lower_threshold,
            oast: RefCell::new(OastDetector916::new(cols, rows, 0)),
            agast58: RefCell::new(AgastDetector58::new(cols, rows, 0)),
        };
        if USE_THRESHOLD_MAP {
            layer.calculate_threshold_map()?;
        }
        Ok(layer)
    }
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn derived(layer: &Self, sampling: Sampling, upper: u8, lower: u8) -> Result<Self> {
        let (mut image, scale) = match sampling {
            Sampling::Half => (
                Matrix::new(layer.image.rows / 2, layer.image.cols / 2, 1),
                layer.scale * 2.0,
            ),
            Sampling::TwoThirds => (
                Matrix::new(2 * (layer.image.rows / 3), 2 * (layer.image.cols / 3), 1),
                (f64::from(layer.scale) * 1.5) as f32,
            ),
        };
        match sampling {
            Sampling::Half => halfsample_8(&layer.image, &mut image)?,
            Sampling::TwoThirds => twothirdsample_8(&layer.image, &mut image)?,
        }
        Self::new(
            image,
            upper,
            lower,
            scale,
            (0.5 * f64::from(scale) - 0.5) as f32,
        )
    }
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn get_agast_points(&self, threshold: u8, points: &mut Vec<KeyPoint>) -> Result<()> {
        let mut detector = self.oast.borrow_mut();
        detector.set_threshold(
            i32::from(threshold),
            i32::from(self.upper_threshold),
            i32::from(self.lower_threshold),
        );
        if points.is_empty() {
            detector.detect(
                &self.image.data,
                points,
                if USE_THRESHOLD_MAP {
                    self.threshold_map.as_ref()
                } else {
                    None
                },
            )?;
        }
        for point in points {
            let at = (point.pt.x + point.pt.y * self.image.cols as f32) as i64;
            if at < 0 || at >= self.image.data.len() as i64 {
                return Err(invalid("AGAST point outside layer"));
            }
            let at = at as usize;
            if USE_THRESHOLD_MAP {
                let thr = self
                    .threshold_map
                    .as_ref()
                    .ok_or_else(|| invalid("threshold map absent"))?
                    .data[at];
                detector.set_threshold(i32::from(thr), 120, 50);
            } else {
                self.scores.borrow_mut().data[at] =
                    detector.corner_score(&self.image.data, at)? as u8;
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn get_agast_score(&self, x: i32, y: i32, threshold: u8) -> Result<u8> {
        if x < 3 || y < 3 || x >= self.image.cols as i32 - 3 || y >= self.image.rows as i32 - 3 {
            return Ok(0);
        }
        let at = y as usize * self.image.cols + x as usize;
        let score = self.scores.borrow().data[at];
        if score > 2 {
            return Ok(score);
        }
        let mut detector = self.oast.borrow_mut();
        detector.set_threshold(i32::from(threshold) - 1, 120, 50);
        let mut score = detector.corner_score(&self.image.data, at)? as u8;
        if score < threshold {
            score = 0;
        }
        self.scores.borrow_mut().data[at] = score;
        Ok(score)
    }
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn get_agast_score_5_8(&self, x: i32, y: i32, threshold: u8) -> Result<u8> {
        if x < 2 || y < 2 || x >= self.image.cols as i32 - 2 || y >= self.image.rows as i32 - 2 {
            return Ok(0);
        }
        let mut detector = self.agast58.borrow_mut();
        detector.set_threshold(i32::from(threshold) - 1, 120, 50);
        let score = detector
            .corner_score(&self.image.data, y as usize * self.image.cols + x as usize)?
            as u8;
        Ok(if score < threshold { 0 } else { score })
    }
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn get_agast_score_interpolated(
        &self,
        xf: f32,
        yf: f32,
        threshold: u8,
        scale: f32,
    ) -> Result<u8> {
        if !xf.is_finite() || !yf.is_finite() || !scale.is_finite() {
            return Err(invalid("finite AGAST sample required"));
        }
        if scale <= 1.0 {
            let (x, y) = (xf as i32, yf as i32);
            let (right_fraction, bottom_fraction) = (xf - x as f32, yf - y as f32);
            let (rx, ry) = (1.0f32 - right_fraction, 1.0f32 - bottom_fraction);
            let score = rx * ry * f32::from(self.get_agast_score(x, y, threshold)?)
                + right_fraction * ry * f32::from(self.get_agast_score(x + 1, y, threshold)?)
                + rx * bottom_fraction * f32::from(self.get_agast_score(x, y + 1, threshold)?)
                + right_fraction
                    * bottom_fraction
                    * f32::from(self.get_agast_score(x + 1, y + 1, threshold)?);
            Ok(score as i32 as u8)
        } else {
            let half = scale / 2.0f32;
            for x in (xf - half) as i32..=(xf + half + 1.0f32) as i32 {
                for y in (yf - half) as i32..=(yf + half + 1.0f32) as i32 {
                    self.get_agast_score(x, y, threshold)?;
                }
            }
            Self::value(&self.scores.borrow(), xf, yf, scale)
        }
    }
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn value(image: &Matrix<u8>, xf: f32, yf: f32, scale: f32) -> Result<u8> {
        check(image)?;
        if scale >= 1.0 {
            return crate::agast::oast9_16_nms::pattern_value(image, xf, yf, scale);
        }
        if !xf.is_finite() || !yf.is_finite() {
            return Err(invalid("finite layer sample required"));
        }
        let (x, y) = (xf.floor() as i32, yf.floor() as i32);
        if x < 0 || y < 0 || x + 1 >= image.cols as i32 || y + 1 >= image.rows as i32 {
            return Err(invalid("bilinear layer sample outside image"));
        }
        let (rx, ry) = (
            ((xf - x as f32) * 1024.0) as i32,
            ((yf - y as f32) * 1024.0) as i32,
        );
        let at = y as usize * image.cols + x as usize;
        let value = (1024 - rx) * (1024 - ry) * i32::from(image.data[at])
            + rx * (1024 - ry) * i32::from(image.data[at + 1])
            + rx * ry * i32::from(image.data[at + image.cols + 1])
            + (1024 - rx) * ry * i32::from(image.data[at + image.cols]);
        Ok((((value + 512) / 1024 / 1024) & 255) as u8)
    }
    /// # Errors
    /// Returns an error if image storage, detector thresholds, or sampling support is invalid.
    pub fn calculate_threshold_map(&mut self) -> Result<()> {
        let (rows, cols) = (self.image.rows, self.image.cols);
        let mut max_image = Matrix::<u8>::new(rows, cols, 1);
        let mut min_image = Matrix::<u8>::new(rows, cols, 1);
        let mut threshold = Matrix::<u8>::new(rows, cols, 1);
        if rows < 3 || cols < 3 {
            self.threshold_map = Some(threshold);
            return Ok(());
        }
        for y in 1..rows - 1 {
            for x in 1..cols - 1 {
                let (mut hi, mut lo) = (0, 255);
                for yy in y - 1..=y + 1 {
                    for xx in x - 1..=x + 1 {
                        let v = self.image.data[yy * cols + xx];
                        hi = hi.max(v);
                        lo = lo.min(v);
                    }
                }
                max_image.data[y * cols + x] = hi;
                min_image.data[y * cols + x] = lo;
            }
        }
        if rows >= 7 && cols >= 7 {
            for y in 3..rows - 3 {
                for x in 3..cols - 3 {
                    let mut hi = self.image.data[y * cols + x];
                    let mut lo = hi;
                    for (yy, xx) in [
                        (y - 2, x - 2),
                        (y - 2, x + 2),
                        (y + 2, x + 2),
                        (y + 2, x - 2),
                    ] {
                        let v = self.image.data[yy * cols + xx];
                        hi = hi.max(v);
                        lo = lo.min(v);
                    }
                    for (yy, xx) in [(y - 2, x), (y + 2, x), (y, x - 2), (y, x + 2)] {
                        hi = hi.max(max_image.data[yy * cols + xx]);
                        lo = lo.min(min_image.data[yy * cols + xx]);
                    }
                    threshold.data[y * cols + x] = hi - lo;
                }
            }
        }
        self.threshold_map = Some(threshold);
        Ok(())
    }
}
