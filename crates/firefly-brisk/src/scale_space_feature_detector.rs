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

//! Harris-score scale-space detector and per-layer uniformity budgets.
use crate::{
    harris_score_calculator::{HarrisScoreCalculator, PointWithScore},
    scale_space_layer::{ScaleSpaceLayer, ScoreCalculator},
    uniformity_enforcement::UniformityPoint,
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use purecv::{
    core::{DynamicMatrix, Matrix},
    features2d::KeyPoint,
};
use std::sync::Arc;
pub struct ScaleSpaceFeatureDetector<S: ScoreCalculator = HarrisScoreCalculator> {
    pub uniformity_radius: f64,
    pub octaves: usize,
    pub absolute_threshold: f64,
    pub max_keypoints: usize,
    pub layers: Vec<ScaleSpaceLayer<S>>,
}
impl<S: ScoreCalculator> ScaleSpaceFeatureDetector<S>
where
    PointWithScore<S::Score>: UniformityPoint,
{
    /// # Errors
    /// Returns an error for invalid pyramid/image data, a mismatched mask, or failed
    /// score interpolation or uniformity selection.
    pub fn new(
        uniformity_radius: f64,
        octaves: usize,
        absolute_threshold: f64,
        max_keypoints: usize,
    ) -> Result<Self> {
        octaves
            .checked_mul(2)
            .ok_or_else(|| invalid("octave count overflow"))?;
        Ok(Self {
            uniformity_radius,
            octaves,
            absolute_threshold,
            max_keypoints,
            layers: Vec::new(),
        })
    }
    /// # Errors
    /// Returns an error for invalid pyramid/image data, a mismatched mask, or failed
    /// score interpolation or uniformity selection.
    pub fn detect(
        &mut self,
        image: &Matrix<u8>,
        keypoints: &mut Vec<KeyPoint>,
        mask: Option<&Matrix<u8>>,
    ) -> Result<()> {
        check(image)?;
        if image.data.is_empty() {
            return Ok(());
        }
        if let Some(mask) = mask {
            check(mask)?;
            if !mask.data.is_empty() {
                if mask.rows != image.rows || mask.cols != image.cols {
                    return Err(invalid("mask shape mismatch"));
                }
                return Err(invalid("scale-space detector does not support masks"));
            }
        }
        let use_passed = !keypoints.is_empty();
        let mut factor = 1.0f64;
        for i in 1..self.octaves * 2 {
            factor += if i % 2 == 0 {
                1.0 / 2.0f64.powf((i / 2) as f64)
            } else {
                1.0 / (1.5 * 2.0f64.powf((i / 2) as f64))
            };
        }
        self.layers.clear();
        self.layers.push(ScaleSpaceLayer::new(
            Arc::new(
                DynamicMatrix::new_u8(image.rows, image.cols, 1, image.data.clone())
                    .map_err(|e| invalid(&e.to_string()))?,
            ),
            !use_passed,
        )?);
        for _ in 1..self.octaves * 2 {
            ScaleSpaceLayer::derive(&mut self.layers, !use_passed)?;
        }
        for (i, layer) in self.layers.iter_mut().enumerate() {
            let max = if i == 0 {
                self.max_keypoints as f64 / factor
            } else if i % 2 == 0 {
                self.max_keypoints as f64 / (factor * 2.0f64.powf((i / 2) as f64))
            } else {
                self.max_keypoints as f64 / (factor * 1.5 * 2.0f64.powf((i / 2) as f64))
            };
            layer.set_uniformity_radius(self.uniformity_radius);
            layer.max_keypoints = Some(max as usize);
            layer.absolute_threshold = self.absolute_threshold;
        }
        for layer in &self.layers {
            layer.detect_scale_space_maxima(
                &self.layers,
                keypoints,
                true,
                !use_passed,
                use_passed,
            )?;
        }
        Ok(())
    }
}
