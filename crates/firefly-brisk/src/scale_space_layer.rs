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

//! Generic score-pyramid layers, adjacent-scale suppression and subpixel fitting.
use crate::{
    harris_score_calculator::{HarrisScoreCalculator, PointWithScore},
    harris_score_calculator_float::HarrisScoreCalculatorFloat,
    image_down_sampling::{halfsample_8, halfsample_16, twothirdsample_8, twothirdsample_16},
    uniformity_enforcement::{UniformityPoint, enforce_key_point_uniformity},
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use purecv::{
    core::{DynamicData, DynamicMatrix, Matrix, types::Point2f},
    features2d::KeyPoint,
};
use std::sync::Arc;
pub trait ScoreValue: Copy + Into<f64> {
    fn from_f64(v: f64) -> Self;
}
impl ScoreValue for i32 {
    fn from_f64(v: f64) -> Self {
        v as i32
    }
}
impl ScoreValue for f32 {
    fn from_f64(v: f64) -> Self {
        v as f32
    }
}
pub trait ScoreCalculator: Default {
    type Score: ScoreValue;
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    fn set_image(&mut self, image: Arc<DynamicMatrix>, initialize: bool) -> Result<()>;
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    fn score(&self, u: f64, v: f64) -> Result<f64>;
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    fn score_at(&self, u: usize, v: usize) -> Result<Self::Score>;
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    fn maxima(&self, threshold: Self::Score) -> Result<Vec<PointWithScore<Self::Score>>>;
}
impl ScoreCalculator for HarrisScoreCalculator {
    type Score = i32;
    fn set_image(&mut self, image: Arc<DynamicMatrix>, initialize: bool) -> Result<()> {
        let m = image
            .as_matrix_u8()
            .ok_or_else(|| invalid("integer Harris requires u8 image"))?;
        HarrisScoreCalculator::set_image(self, Arc::new(m.clone()), initialize)
    }
    fn score(&self, u: f64, v: f64) -> Result<f64> {
        HarrisScoreCalculator::score(self, u, v)
    }
    fn score_at(&self, u: usize, v: usize) -> Result<i32> {
        HarrisScoreCalculator::score_at(self, u, v)
    }
    fn maxima(&self, threshold: i32) -> Result<Vec<PointWithScore<i32>>> {
        let mut p = Vec::new();
        self.get_2d_maxima(&mut p, threshold)?;
        Ok(p)
    }
}
impl ScoreCalculator for HarrisScoreCalculatorFloat {
    type Score = f32;
    fn set_image(&mut self, image: Arc<DynamicMatrix>, initialize: bool) -> Result<()> {
        HarrisScoreCalculatorFloat::set_image(self, image, initialize)
    }
    fn score(&self, u: f64, v: f64) -> Result<f64> {
        HarrisScoreCalculatorFloat::score(self, u, v)
    }
    fn score_at(&self, u: usize, v: usize) -> Result<f32> {
        HarrisScoreCalculatorFloat::score_at(self, u, v)
    }
    fn maxima(&self, threshold: f32) -> Result<Vec<PointWithScore<f32>>> {
        let mut p = Vec::new();
        self.get_2d_maxima(&mut p, threshold)?;
        Ok(p)
    }
}
pub struct ScaleSpaceLayer<S: ScoreCalculator> {
    pub image: Arc<DynamicMatrix>,
    pub calculator: S,
    pub is_octave: bool,
    pub layer_number: usize,
    pub above: Option<usize>,
    pub below: Option<usize>,
    pub scale: f64,
    pub offset: f64,
    pub scale_above: f64,
    pub scale_below: f64,
    pub offset_above: f64,
    pub offset_below: f64,
    pub radius: f64,
    pub max_keypoints: Option<usize>,
    pub absolute_threshold: f64,
    pub lut: Matrix<f32>,
}
impl<S: ScoreCalculator> ScaleSpaceLayer<S>
where
    PointWithScore<S::Score>: UniformityPoint,
{
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    pub fn new(image: Arc<DynamicMatrix>, initialize: bool) -> Result<Self> {
        let mut calculator = S::default();
        calculator.set_image(image.clone(), initialize)?;
        let mut lut = Matrix::new(31, 31, 1);
        for x in 0..31 {
            for y in 0..31 {
                let d = f64::from(
                    (15 - x as i32) * (15 - x as i32) + (15 - y as i32) * (15 - y as i32),
                );
                lut.data[y * 31 + x] = (1.0
                    - d.sqrt().sqrt().sqrt() / (225.0f64).sqrt().sqrt().sqrt())
                .max(0.0) as f32;
            }
        }
        Ok(Self {
            image,
            calculator,
            is_octave: true,
            layer_number: 0,
            above: None,
            below: None,
            scale: 1.0,
            offset: 0.0,
            scale_above: 2.0 / 3.0,
            scale_below: 4.0 / 3.0,
            offset_above: -0.25,
            offset_below: 1.0 / 6.0,
            radius: 1.0,
            max_keypoints: None,
            absolute_threshold: 0.0,
            lut,
        })
    }
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    pub fn from_image_constructor(image: Arc<DynamicMatrix>, _initialize: bool) -> Result<Self> {
        Self::new(image, true)
    }
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    pub fn derive(layers: &mut Vec<Self>, initialize: bool) -> Result<()> {
        let previous = layers
            .len()
            .checked_sub(1)
            .ok_or_else(|| invalid("lower scale layer absent"))?;
        let lower = &layers[previous];
        let half = (!lower.is_octave) || lower.layer_number >= 2;
        let input = if half {
            &layers[lower
                .below
                .ok_or_else(|| invalid("half-sample source layer absent"))?]
            .image
        } else {
            &lower.image
        };
        let image = Arc::new(
            Self::subsample(input, half)?
                .ok_or_else(|| invalid("u8/u16 pyramid image required"))?,
        );
        let number = lower.layer_number + 1;
        let octave = !lower.is_octave;
        let mut current = Self::new(image, initialize)?;
        current.is_octave = octave;
        current.layer_number = number;
        current.below = Some(previous);
        current.scale = 2.0f64.powf((number / 2) as f64) * if octave { 1.0 } else { 1.5 };
        current.offset = current.scale * 0.5 - 0.5;
        if !octave {
            current.offset_above = -1.0 / 6.0;
            current.offset_below = 0.125;
            current.scale_above = 0.75;
            current.scale_below = 1.5;
        }
        for x in 0..31 {
            for y in 0..31 {
                let d = f64::from(
                    (15 - x as i32) * (15 - x as i32) + (15 - y as i32) * (15 - y as i32),
                );
                current.lut.data[y * 31 + x] = (1.0 - d / 225.0).max(0.0) as f32;
            }
        }
        layers[previous].above = Some(layers.len());
        layers.push(current);
        Ok(())
    }
    pub fn set_uniformity_radius(&mut self, radius: f64) {
        self.radius = if radius == 0.0 { 1.0 } else { radius };
    }
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    pub fn subsample(image: &DynamicMatrix, half: bool) -> Result<Option<DynamicMatrix>> {
        match &image.data {
            DynamicData::U8(m) => {
                check(m)?;
                let mut out = Matrix::new(
                    if half { m.rows / 2 } else { m.rows / 3 * 2 },
                    if half { m.cols / 2 } else { m.cols / 3 * 2 },
                    1,
                );
                if half {
                    halfsample_8(m, &mut out)?;
                } else {
                    twothirdsample_8(m, &mut out)?;
                }
                Ok(Some(
                    DynamicMatrix::new_u8(out.rows, out.cols, 1, out.data)
                        .map_err(|e| invalid(&e.to_string()))?,
                ))
            }
            DynamicData::U16(m) => {
                check(m)?;
                let mut out = Matrix::new(
                    if half { m.rows / 2 } else { m.rows / 3 * 2 },
                    if half { m.cols / 2 } else { m.cols / 3 * 2 },
                    1,
                );
                if half {
                    halfsample_16(m, &mut out)?;
                } else {
                    twothirdsample_16(m, &mut out)?;
                }
                Ok(Some(
                    DynamicMatrix::new_u16(out.rows, out.cols, 1, out.data)
                        .map_err(|e| invalid(&e.to_string()))?,
                ))
            }
            _ => Ok(None),
        }
    }
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    pub fn score_above(&self, layers: &[Self], u: f64, v: f64) -> Result<f64> {
        layers
            .get(self.above.ok_or_else(|| invalid("above layer absent"))?)
            .ok_or_else(|| invalid("above layer index invalid"))?
            .calculator
            .score(
                self.scale_above * (u + self.offset_above),
                self.scale_above * (v + self.offset_above),
            )
    }
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    pub fn score_below(&self, layers: &[Self], u: f64, v: f64) -> Result<f64> {
        layers
            .get(self.below.ok_or_else(|| invalid("below layer absent"))?)
            .ok_or_else(|| invalid("below layer index invalid"))?
            .calculator
            .score(
                self.scale_below * (u + self.offset_below),
                self.scale_below * (v + self.offset_below),
            )
    }
    fn select_adjacent_maxima(
        &self,
        layers: &[Self],
        points: Vec<PointWithScore<S::Score>>,
        threshold: S::Score,
    ) -> Result<Vec<PointWithScore<S::Score>>> {
        let mut selected = Vec::new();
        for point in points {
            let center: f64 = point.score.into();
            if center < threshold.into() {
                continue;
            }
            let mut accepted = true;
            for above in [true, false] {
                if (above && self.above.is_none()) || (!above && self.below.is_none()) {
                    continue;
                }
                let step = (1.0
                    / if above {
                        self.scale_above
                    } else {
                        self.scale_below
                    }) as i32;
                for (dx, dy) in [
                    (0, 0),
                    (step, 0),
                    (-step, 0),
                    (0, step),
                    (0, -step),
                    (step, step),
                    (step, -step),
                    (-step, step),
                    (-step, -step),
                ] {
                    let score = if above {
                        self.score_above(
                            layers,
                            f64::from(point.x) + f64::from(dx),
                            f64::from(point.y) + f64::from(dy),
                        )?
                    } else {
                        self.score_below(
                            layers,
                            f64::from(point.x) + f64::from(dx),
                            f64::from(point.y) + f64::from(dy),
                        )?
                    };
                    if center < score {
                        accepted = false;
                        break;
                    }
                }
                if !accepted {
                    break;
                }
            }
            if accepted {
                selected.push(point);
            }
        }
        Ok(selected)
    }
    /// # Errors
    /// Returns an error for unsupported or malformed images, absent adjacent layers/scores,
    /// or sampling outside the available score support.
    pub fn detect_scale_space_maxima(
        &self,
        layers: &[Self],
        keypoints: &mut Vec<KeyPoint>,
        uniformity: bool,
        refine: bool,
        use_passed: bool,
    ) -> Result<()> {
        let threshold = S::Score::from_f64(self.absolute_threshold);
        let mut points = if use_passed {
            keypoints
                .iter()
                .filter(|p| p.response > 1e6)
                .map(|p| {
                    PointWithScore::new(
                        S::Score::from_f64(f64::from(p.response)),
                        p.pt.x as u16,
                        p.pt.y as u16,
                    )
                })
                .collect()
        } else {
            self.calculator.maxima(threshold)?
        };
        if !use_passed && (self.above.is_some() || self.below.is_some()) {
            points = self.select_adjacent_maxima(layers, points, threshold)?;
        }
        if points.is_empty() {
            return Ok(());
        }
        if uniformity && self.radius > 0.0 {
            let max = self
                .max_keypoints
                .ok_or_else(|| invalid("per-layer maximum keypoints has not been set"))?;
            enforce_key_point_uniformity(
                &self.lut,
                self.radius,
                self.image.rows(),
                self.image.cols(),
                max,
                &mut points,
            )?;
        }
        if use_passed {
            keypoints.clear();
        }
        for point in points {
            let (mut dx, mut dy) = (0.0f32, 0.0f32);
            if refine {
                let u = point.x as usize;
                let v = point.y as usize;
                if u == 0 || v == 0 {
                    return Err(invalid("subpixel support outside image"));
                }
                let mut patch = [0.0f64; 9];
                for x in 0..3 {
                    for y in 0..3 {
                        patch[y * 3 + x] = self.calculator.score_at(u + x - 1, v + y - 1)?.into();
                    }
                }
                let (_, x, y) = layer_subpixel_2d(&[
                    [patch[0], patch[1], patch[2]],
                    [patch[3], patch[4], patch[5]],
                    [patch[6], patch[7], patch[8]],
                ]);
                dx = x;
                dy = y;
            }
            let x = if refine {
                self.scale * (f64::from(f32::from(point.x) + dx) + self.offset)
            } else {
                self.scale * (f64::from(point.x) + self.offset)
            };
            let y = if refine {
                self.scale * (f64::from(f32::from(point.y) + dy) + self.offset)
            } else {
                self.scale * (f64::from(point.y) + self.offset)
            };
            keypoints.push(KeyPoint::new(
                Point2f::new(x as f32, y as f32),
                (self.scale * 12.0) as f32,
                -1.0,
                point.score.into() as f32,
                (self.layer_number / 2) as i32,
                -1,
            ));
        }
        Ok(())
    }
}
#[must_use]
pub fn layer_refine_1d(
    lower_response: f32,
    center_response: f32,
    upper_response: f32,
) -> (f32, f32) {
    let mut max: f32;

    let lower_fixed: i32 = ((1024.0f64 * f64::from(lower_response)) + 0.5f64) as i32;
    let center_fixed: i32 = ((1024.0f64 * f64::from(center_response)) + 0.5f64) as i32;
    let upper_fixed: i32 = ((1024.0f64 * f64::from(upper_response)) + 0.5f64) as i32;
    let three_a: i32 = (((16i32).wrapping_mul(lower_fixed))
        .wrapping_sub((24i32).wrapping_mul(center_fixed)))
    .wrapping_add((8i32).wrapping_mul(upper_fixed));
    if three_a >= 0i32 {
        if (center_response >= lower_response) && (center_response >= upper_response) {
            max = center_response;
            return ((1.0f64 as f32), max);
        }
        if (lower_response >= center_response) && (lower_response >= upper_response) {
            max = lower_response;
            return ((0.75f64 as f32), max);
        }
        if (upper_response >= center_response) && (upper_response >= lower_response) {
            max = upper_response;
            return ((1.5f64 as f32), max);
        }
    }
    let three_b: i32 = ((((40i32).wrapping_neg()).wrapping_mul(lower_fixed))
        .wrapping_add((54i32).wrapping_mul(center_fixed)))
    .wrapping_sub((14i32).wrapping_mul(upper_fixed));
    let mut ret_val: f32 = -(three_b as f32) / ((2i32).wrapping_mul(three_a) as f32);
    if f64::from(ret_val) < 0.75f64 {
        ret_val = 0.75f64 as f32;
    } else if f64::from(ret_val) > 1.5f64 {
        ret_val = 1.5f64 as f32;
    }
    let three_c: i32 = (((24i32).wrapping_mul(lower_fixed))
        .wrapping_sub((27i32).wrapping_mul(center_fixed)))
    .wrapping_add((6i32).wrapping_mul(upper_fixed));
    max = ((three_c as f32) + (((three_a as f32) * ret_val) * ret_val))
        + ((three_b as f32) * ret_val);
    max = (f64::from(max) / 3072.0f64) as f32;
    (ret_val, max)
}
#[must_use]
pub fn layer_refine_1d_1(
    lower_response: f32,
    center_response: f32,
    upper_response: f32,
) -> (f32, f32) {
    let mut max: f32;

    let lower_fixed: i32 = ((1024.0f64 * f64::from(lower_response)) + 0.5f64) as i32;
    let center_fixed: i32 = ((1024.0f64 * f64::from(center_response)) + 0.5f64) as i32;
    let upper_fixed: i32 = ((1024.0f64 * f64::from(upper_response)) + 0.5f64) as i32;
    let two_a: i32 = (((9i32).wrapping_mul(lower_fixed))
        .wrapping_sub((18i32).wrapping_mul(center_fixed)))
    .wrapping_add((9i32).wrapping_mul(upper_fixed));
    if two_a >= 0i32 {
        if (center_response >= lower_response) && (center_response >= upper_response) {
            max = center_response;
            return ((1.0f64 as f32), max);
        }
        if (lower_response >= center_response) && (lower_response >= upper_response) {
            max = lower_response;
            return ((0.666_666_666_666_666_6f64 as f32), max);
        }
        if (upper_response >= center_response) && (upper_response >= lower_response) {
            max = upper_response;
            return ((1.333_333_333_333_333_3f64 as f32), max);
        }
    }
    let two_b: i32 = ((((21i32).wrapping_neg()).wrapping_mul(lower_fixed))
        .wrapping_add((36i32).wrapping_mul(center_fixed)))
    .wrapping_sub((15i32).wrapping_mul(upper_fixed));
    let mut ret_val: f32 = -(two_b as f32) / ((2i32).wrapping_mul(two_a) as f32);
    if f64::from(ret_val) < 0.666_666_666_666_666_6f64 {
        ret_val = 0.666_666_666_666_666_6f64 as f32;
    } else if f64::from(ret_val) > 1.333_333_333_333_333_3f64 {
        ret_val = 1.333_333_333_333_333_3f64 as f32;
    }
    let two_c: i32 = (((12i32).wrapping_mul(lower_fixed))
        .wrapping_sub((16i32).wrapping_mul(center_fixed)))
    .wrapping_add((6i32).wrapping_mul(upper_fixed));
    max = ((two_c as f32) + (((two_a as f32) * ret_val) * ret_val)) + ((two_b as f32) * ret_val);
    max = (f64::from(max) / 2048.0f64) as f32;
    (ret_val, max)
}
#[must_use]
pub fn layer_subpixel_2d(samples: &[[f64; 3]; 3]) -> (f32, f32, f32) {
    let mut delta_x: f32;
    let mut delta_y: f32;

    let tmp1: f64 = (((samples[0][0] + samples[0][2]) - (2_f64 * samples[1][1])) + samples[2][0])
        + samples[2][2];
    let horizontal_sum = samples[1][0] + samples[1][2];
    let coeff1: f64 =
        3_f64 * (((tmp1 + samples[0][1]) - (horizontal_sum / 2.0f64)) + samples[2][1]);
    let vertical_sum = samples[0][1] + samples[2][1];
    let coeff2: f64 = 3_f64 * (((tmp1 - (vertical_sum / 2.0f64)) + samples[1][0]) + samples[1][2]);
    let tmp2: f64 = samples[0][2] - samples[2][0];
    let tmp3: f64 = (samples[0][0] + tmp2) - samples[2][2];
    let tmp4: f64 = tmp3 - (2_f64 * tmp2);
    let coeff3: f64 = f64::from((3i32).wrapping_neg()) * ((tmp3 + samples[0][1]) - samples[2][1]);
    let coeff4: f64 = f64::from((3i32).wrapping_neg()) * ((tmp4 + samples[1][0]) - samples[1][2]);
    let coeff5: f64 = (((samples[0][0] - samples[0][2]) - samples[2][0]) + samples[2][2]) / 4.0f64;
    let coeff6: f64 = -(((((samples[0][0] + samples[0][2])
        - ((((samples[1][0] + samples[0][1]) + samples[1][2]) + samples[2][1]) / 2.0f64))
        - (5_f64 * samples[1][1]))
        + samples[2][0])
        + samples[2][2])
        / 2.01f64;
    let h_det: f64 = ((4_f64 * coeff1) * coeff2) - (coeff5 * coeff5);
    if h_det == f64::from(0i32) {
        delta_x = 0.0f64 as f32;
        delta_y = 0.0f64 as f32;
        return (
            ((f64::from(coeff6 as f32) / 18.0f64) as f32),
            delta_x,
            delta_y,
        );
    }
    if !((h_det > f64::from(0i32)) && (coeff1 < f64::from(0i32))) {
        let mut tmp_max: i32 = ((coeff3 + coeff4) + coeff5) as i32;
        delta_x = 1.0f64 as f32;
        delta_y = 1.0f64 as f32;
        let mut tmp: i32 = ((-(coeff3) + coeff4) - coeff5) as i32;
        if tmp > tmp_max {
            tmp_max = tmp;
            delta_x = -(1.0f64) as f32;
            delta_y = 1.0f64 as f32;
        }
        tmp = ((coeff3 - coeff4) - coeff5) as i32;
        if tmp > tmp_max {
            tmp_max = tmp;
            delta_x = 1.0f64 as f32;
            delta_y = -(1.0f64) as f32;
        }
        tmp = ((-(coeff3) - coeff4) + coeff5) as i32;
        if tmp > tmp_max {
            tmp_max = tmp;
            delta_x = -(1.0f64) as f32;
            delta_y = -(1.0f64) as f32;
        }
        return (
            ((f64::from((((f64::from(tmp_max) + coeff1) + coeff2) + coeff6) as f32) / 18.0f64)
                as f32),
            delta_x,
            delta_y,
        );
    }
    delta_x = ((((2_f64 * coeff2) * coeff3) - (coeff4 * coeff5)) as f32) / (-(h_det) as f32);
    delta_y = ((((2_f64 * coeff1) * coeff4) - (coeff3 * coeff5)) as f32) / (-(h_det) as f32);
    constrain_subpixel_patch(
        [coeff1, coeff2, coeff3, coeff4, coeff5, coeff6],
        delta_x,
        delta_y,
    )
}
fn constrain_subpixel_patch(
    coefficients: [f64; 6],
    mut delta_x: f32,
    mut delta_y: f32,
) -> (f32, f32, f32) {
    let [coeff1, coeff2, coeff3, coeff4, coeff5, coeff6] = coefficients;
    let mut tx: bool = false;
    let mut tx_: bool = false;
    let mut ty: bool = false;
    let mut ty_: bool = false;
    if f64::from(delta_x) > 1.0f64 {
        tx = true;
    } else if f64::from(delta_x) < -(1.0f64) {
        tx_ = true;
    }
    if f64::from(delta_y) > 1.0f64 {
        ty = true;
    }
    if f64::from(delta_y) < -(1.0f64) {
        ty_ = true;
    }
    if ((tx || tx_) || ty) || ty_ {
        let mut horizontal_candidate_x: f32 = 0.0f64 as f32;
        let mut vertical_candidate_x: f32 = 0.0f64 as f32;
        let mut horizontal_candidate_y: f32 = 0.0f64 as f32;
        let mut vertical_candidate_y: f32 = 0.0f64 as f32;
        if tx {
            horizontal_candidate_x = 1.0f64 as f32;
            horizontal_candidate_y = -((coeff4 + coeff5) as f32) / ((2_f64 * coeff2) as f32);
            if f64::from(horizontal_candidate_y) > 1.0f64 {
                horizontal_candidate_y = 1.0f64 as f32;
            } else if f64::from(horizontal_candidate_y) < -(1.0f64) {
                horizontal_candidate_y = -(1.0f64) as f32;
            }
        } else if tx_ {
            horizontal_candidate_x = -(1.0f64) as f32;
            horizontal_candidate_y = -((coeff4 - coeff5) as f32) / ((2_f64 * coeff2) as f32);
            if f64::from(horizontal_candidate_y) > 1.0f64 {
                horizontal_candidate_y = 1.0f64 as f32;
            } else if f64::from(horizontal_candidate_y) < -(1.0f64) {
                horizontal_candidate_y = -(1.0f64) as f32;
            }
        }
        if ty {
            vertical_candidate_y = 1.0f64 as f32;
            vertical_candidate_x = -((coeff3 + coeff5) as f32) / ((2_f64 * coeff1) as f32);
            if f64::from(vertical_candidate_x) > 1.0f64 {
                vertical_candidate_x = 1.0f64 as f32;
            } else if f64::from(vertical_candidate_x) < -(1.0f64) {
                vertical_candidate_x = -(1.0f64) as f32;
            }
        } else if ty_ {
            vertical_candidate_y = -(1.0f64) as f32;
            vertical_candidate_x = -((coeff3 - coeff5) as f32) / ((2_f64 * coeff1) as f32);
            if f64::from(vertical_candidate_x) > 1.0f64 {
                vertical_candidate_x = 1.0f64 as f32;
            } else if f64::from(vertical_candidate_x) < -(1.0f64) {
                vertical_candidate_x = -(1.0f64) as f32;
            }
        }
        let max1: f32 = ((((((((coeff1 * f64::from(horizontal_candidate_x))
            * f64::from(horizontal_candidate_x))
            + ((coeff2 * f64::from(horizontal_candidate_y))
                * f64::from(horizontal_candidate_y)))
            + (coeff3 * f64::from(horizontal_candidate_x)))
            + (coeff4 * f64::from(horizontal_candidate_y)))
            + ((coeff5 * f64::from(horizontal_candidate_x)) * f64::from(horizontal_candidate_y)))
            + coeff6)
            / 18.0f64) as f32;
        let max2: f32 = ((((((((coeff1 * f64::from(vertical_candidate_x))
            * f64::from(vertical_candidate_x))
            + ((coeff2 * f64::from(vertical_candidate_y)) * f64::from(vertical_candidate_y)))
            + (coeff3 * f64::from(vertical_candidate_x)))
            + (coeff4 * f64::from(vertical_candidate_y)))
            + ((coeff5 * f64::from(vertical_candidate_x)) * f64::from(vertical_candidate_y)))
            + coeff6)
            / 18.0f64) as f32;
        if max1 > max2 {
            delta_x = horizontal_candidate_x;
            delta_y = horizontal_candidate_x;
            return (max1, delta_x, delta_y);
        }
        delta_x = vertical_candidate_x;
        delta_y = vertical_candidate_x;
        return (max2, delta_x, delta_y);
    }
    (
        (((((((((coeff1 * f64::from(delta_x)) * f64::from(delta_x))
            + ((coeff2 * f64::from(delta_y)) * f64::from(delta_y)))
            + (coeff3 * f64::from(delta_x)))
            + (coeff4 * f64::from(delta_y)))
            + ((coeff5 * f64::from(delta_x)) * f64::from(delta_y)))
            + coeff6)
            / 18.0f64) as f32),
        delta_x,
        delta_y,
    )
}
