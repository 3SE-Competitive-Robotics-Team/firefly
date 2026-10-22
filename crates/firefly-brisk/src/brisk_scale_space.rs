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

//! BRISK AGAST pyramid, scale-axis maxima and reference quadratic refinement.
use crate::{
    brisk_layer::{BriskLayer, Sampling},
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use purecv::{core::Matrix, features2d::KeyPoint};
pub struct BriskScaleSpace {
    pub layers: u8,
    pub pyramid: Vec<BriskLayer>,
    pub threshold: u8,
    pub suppress_scale_nonmaxima: bool,
}
fn max_f32(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}
impl BriskScaleSpace {
    #[must_use]
    pub fn new(octaves: u8, suppress_scale_nonmaxima: bool) -> Self {
        Self {
            layers: if octaves == 0 {
                1
            } else {
                octaves.wrapping_mul(2)
            },
            pyramid: Vec::new(),
            threshold: 0,
            suppress_scale_nonmaxima,
        }
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn construct_pyramid(
        &mut self,
        image: &Matrix<u8>,
        threshold: u8,
        lower_threshold: u8,
    ) -> Result<()> {
        check(image)?;
        self.pyramid.clear();
        self.threshold = threshold;
        self.pyramid.push(BriskLayer::new(
            image.clone(),
            230,
            lower_threshold,
            1.0,
            0.0,
        )?);
        if self.layers > 1 {
            let next =
                BriskLayer::derived(&self.pyramid[0], Sampling::TwoThirds, 230, lower_threshold)?;
            self.pyramid.push(next);
        }
        for i in (2..self.layers as usize).step_by(2) {
            let a =
                BriskLayer::derived(&self.pyramid[i - 2], Sampling::Half, 230, lower_threshold)?;
            self.pyramid.push(a);
            let b =
                BriskLayer::derived(&self.pyramid[i - 1], Sampling::Half, 230, lower_threshold)?;
            self.pyramid.push(b);
        }
        Ok(())
    }
    fn layer(&self, index: usize) -> Result<&BriskLayer> {
        self.pyramid
            .get(index)
            .ok_or_else(|| invalid("BRISK layer index outside pyramid"))
    }
    fn patch(layer: &BriskLayer, x: i32, y: i32, small: bool) -> Result<[i32; 9]> {
        let mut p = [0; 9];
        for dx in 0..3 {
            for dy in 0..3 {
                p[dx * 3 + dy] = i32::from(if small {
                    layer.get_agast_score_5_8(x + dx as i32 - 1, y + dy as i32 - 1, 1)?
                } else {
                    layer.get_agast_score(x + dx as i32 - 1, y + dy as i32 - 1, 1)?
                });
            }
        }
        Ok(p)
    }
    fn patch_float(layer: &BriskLayer, x: f32, y: f32) -> Result<[i32; 9]> {
        let mut p = [0; 9];
        for (dx, dy) in [
            (0, 0),
            (1, 0),
            (2, 0),
            (2, 1),
            (1, 1),
            (0, 1),
            (0, 2),
            (1, 2),
            (2, 2),
        ] {
            p[dx * 3 + dy] = i32::from(layer.get_agast_score_interpolated(
                x + dx as f32 - 1.0,
                y + dy as f32 - 1.0,
                1,
                1.0,
            )?);
        }
        Ok(p)
    }
    #[must_use]
    pub fn subpixel(p: &[i32; 9]) -> (f32, f32, f32) {
        brisk_subpixel_2d(&[[p[0], p[1], p[2]], [p[3], p[4], p[5]], [p[6], p[7], p[8]]])
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn get_keypoints(&self, keypoints: &mut Vec<KeyPoint>) -> Result<()> {
        let input = keypoints.clone();
        let perform_2d = input.is_empty();
        let mut agast_points = vec![Vec::new(); self.layers as usize];
        for (i, points) in agast_points.iter_mut().enumerate() {
            let layer = self.layer(i)?;
            if !input.is_empty() {
                for keypoint in &input {
                    let mut kp = keypoint.clone();
                    kp.pt.x = keypoint.pt.x / layer.scale - layer.offset;
                    kp.pt.y = keypoint.pt.y / layer.scale - layer.offset;
                    if kp.pt.x < 3.0
                        || kp.pt.y < 3.0
                        || kp.pt.x > (layer.image.cols as i32 - 3) as f32
                        || kp.pt.y > (layer.image.rows as i32 - 3) as f32
                    {
                        continue;
                    }
                    layer.get_agast_score_interpolated(kp.pt.x, kp.pt.y, 0, 1.0)?;
                    points.push(kp);
                }
            }
            layer.get_agast_points(self.threshold, points)?;
        }
        keypoints.clear();
        for (i, points) in agast_points.iter().enumerate() {
            let layer = self.layer(i)?;
            for point in points {
                let (x, y) = (point.pt.x, point.pt.y);
                if perform_2d && !self.is_max_2d(i, x as i32, y as i32)? {
                    continue;
                }
                let mut out = point.clone();
                if !self.suppress_scale_nonmaxima
                    || self.layers == 1
                    || i == self.layers as usize - 1
                {
                    if self.suppress_scale_nonmaxima && self.layers > 1 {
                        let center = i32::from(layer.get_agast_score_interpolated(x, y, 1, 1.0)?);
                        if self
                            .get_score_max_below(i, x as i32, y as i32, center)?
                            .is_none()
                        {
                            continue;
                        }
                    }
                    let patch = Self::patch_float(layer, x, y)?;
                    let (score, dx, dy) = Self::subpixel(&patch);
                    out.pt.x = (x + dx) * layer.scale + layer.offset;
                    out.pt.y = (y + dy) * layer.scale + layer.offset;
                    out.size = 12.0 * layer.scale;
                    out.response = score;
                } else {
                    let Some((score, rx, ry, scale)) = self.refine_3d(i, x as i32, y as i32)?
                    else {
                        continue;
                    };
                    out.pt.x = rx;
                    out.pt.y = ry;
                    out.size = 12.0 * scale;
                    out.response = score;
                }
                out.angle = -1.0;
                out.octave = i as i32;
                keypoints.push(out);
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn get_score_above(&self, layer: usize, x: i32, y: i32) -> Result<i32> {
        if layer + 1 >= self.layers as usize {
            return Err(invalid("above scale layer absent"));
        }
        let l = self.layer(layer + 1)?;
        let (dimension, constant, round) = if layer.is_multiple_of(2) {
            (4, 6, 18)
        } else {
            (6, 8, 32)
        };
        let (sx, sy) = (dimension * x - 1, dimension * y - 1);
        let (xa, ya) = (sx / constant, sy / constant);
        let (rx, ry) = (sx % constant, sy % constant);
        let right_vectors =
            (constant - rx) * (constant - ry) * i32::from(l.get_agast_score(xa, ya, 1)?)
                + rx * (constant - ry) * i32::from(l.get_agast_score(xa + 1, ya, 1)?)
                + (constant - rx) * ry * i32::from(l.get_agast_score(xa, ya + 1, 1)?)
                + rx * ry * i32::from(l.get_agast_score(xa + 1, ya + 1, 1)?)
                + round;
        Ok((right_vectors / (constant * constant)) & 255)
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn get_score_below(&self, layer: usize, x: i32, y: i32) -> Result<i32> {
        let l = self.layer(
            layer
                .checked_sub(1)
                .ok_or_else(|| invalid("below scale layer absent"))?,
        )?;
        let (xf, yf, offs) = if layer.is_multiple_of(2) {
            (
                (f64::from((8 * x + 1) as f32) / 6.0) as f32,
                (f64::from((8 * y + 1) as f32) / 6.0) as f32,
                (3.0 / 4.0) as f32,
            )
        } else {
            (
                (f64::from((6 * x + 1) as f32) / 4.0) as f32,
                (f64::from((6 * y + 1) as f32) / 4.0) as f32,
                (2.0 / 3.0) as f32,
            )
        };
        let area = (4.0 * f64::from(offs) * f64::from(offs)) as f32;
        let scaling = (4_194_304.0 / f64::from(area)) as i32;
        let scaling2 = (scaling as f32 * area) as i32;
        if scaling2 == 0 {
            return Err(invalid("zero below-scale integration divisor"));
        }
        let (xlo, xhi, ylo, yhi) = (xf - offs, xf + offs, yf - offs, yf + offs);
        let (xl, xr, yt, yb) = (
            (f64::from(xlo) + 0.5) as i32,
            (f64::from(xhi) + 0.5) as i32,
            (f64::from(ylo) + 0.5) as i32,
            (f64::from(yhi) + 0.5) as i32,
        );
        let (left_fraction, right_fraction, top_fraction, bottom_fraction) = (
            (f64::from(xl as f32 - xlo) + 0.5) as f32,
            (f64::from(xhi - xr as f32) + 0.5) as f32,
            (f64::from(yt as f32 - ylo) + 0.5) as f32,
            (f64::from(yhi - yb as f32) + 0.5) as f32,
        );
        let (leading, quadratic, linear, constant) = (
            (left_fraction * top_fraction * scaling as f32) as i32,
            (right_fraction * top_fraction * scaling as f32) as i32,
            (right_fraction * bottom_fraction * scaling as f32) as i32,
            (left_fraction * bottom_fraction * scaling as f32) as i32,
        );
        let (dx, dy) = (xr - xl - 1, yb - yt - 1);
        let mut value = 0i32;
        for yy in 0..=dy + 1 {
            for xx in 0..=dx + 1 {
                let weight = if yy == 0 {
                    if xx == 0 {
                        leading
                    } else if xx == dx + 1 {
                        quadratic
                    } else {
                        (top_fraction * scaling as f32) as i32
                    }
                } else if yy == dy + 1 {
                    if xx == 0 {
                        constant
                    } else if xx == dx + 1 {
                        linear
                    } else {
                        (bottom_fraction * scaling as f32) as i32
                    }
                } else if xx == 0 {
                    (left_fraction * scaling as f32) as i32
                } else if xx == dx + 1 {
                    (right_fraction * scaling as f32) as i32
                } else {
                    scaling
                };
                value = value.wrapping_add(weight.wrapping_mul(i32::from(l.get_agast_score(
                    xl + xx,
                    yt + yy,
                    1,
                )?)));
            }
        }
        Ok(value.wrapping_add(scaling2 / 2) / scaling2)
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn is_max_2d(&self, index: usize, x: i32, y: i32) -> Result<bool> {
        let layer = self.layer(index)?;
        if x < 0 || y < 0 || x >= layer.image.cols as i32 || y >= layer.image.rows as i32 {
            return Err(invalid("maximum center outside layer"));
        }
        let center = layer.scores.borrow().data[y as usize * layer.image.cols + x as usize];
        let offsets = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, 1),
            (1, 1),
            (1, -1),
            (-1, -1),
        ];
        let mut scores = [0u8; 8];
        for (i, (dx, dy)) in offsets.iter().enumerate() {
            scores[i] = layer.get_agast_score(x + dx, y + dy, center)?;
            if center < scores[i] {
                return Ok(false);
            }
        }
        let smooth = 4 * i32::from(center)
            + 2 * (i32::from(scores[0])
                + i32::from(scores[1])
                + i32::from(scores[2])
                + i32::from(scores[3]))
            + i32::from(scores[7])
            + i32::from(scores[6])
            + i32::from(scores[4])
            + i32::from(scores[5]);
        let cache = layer.scores.borrow();
        for i in [7, 2, 6, 0, 1, 4, 3, 5] {
            if center != scores[i] {
                continue;
            }
            let (dx, dy) = offsets[i];
            let mut other = 0i32;
            for yy in -1..=1 {
                for xx in -1..=1 {
                    let (px, py) = (x + dx + xx, y + dy + yy);
                    if px < 0 || py < 0 || px >= cache.cols as i32 || py >= cache.rows as i32 {
                        return Err(invalid("maximum tie support outside image"));
                    }
                    let weight = if xx == 0 { 2 } else { 1 } * if yy == 0 { 2 } else { 1 };
                    other += weight * i32::from(cache.data[py as usize * cache.cols + px as usize]);
                }
            }
            if other > smooth {
                return Ok(false);
            }
        }
        Ok(true)
    }
    fn neighbor_sum(layer: &BriskLayer, x: i32, y: i32) -> Result<i32> {
        Ok(2 * (i32::from(layer.get_agast_score(x - 1, y, 1)?)
            + i32::from(layer.get_agast_score(x + 1, y, 1)?)
            + i32::from(layer.get_agast_score(x, y + 1, 1)?)
            + i32::from(layer.get_agast_score(x, y - 1, 1)?))
            + i32::from(layer.get_agast_score(x + 1, y + 1, 1)?)
            + i32::from(layer.get_agast_score(x - 1, y + 1, 1)?)
            + i32::from(layer.get_agast_score(x + 1, y - 1, 1)?)
            + i32::from(layer.get_agast_score(x - 1, y - 1, 1)?))
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn get_score_max_above(
        &self,
        index: usize,
        x: i32,
        y: i32,
        threshold: i32,
    ) -> Result<Option<(f32, f32, f32)>> {
        self.get_score_max(index, x, y, threshold, true)
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn get_score_max_below(
        &self,
        index: usize,
        x: i32,
        y: i32,
        threshold: i32,
    ) -> Result<Option<(f32, f32, f32)>> {
        self.get_score_max(index, x, y, threshold, false)
    }
    fn get_score_max(
        &self,
        index: usize,
        x: i32,
        y: i32,
        threshold: i32,
        above: bool,
    ) -> Result<Option<(f32, f32, f32)>> {
        let neighbor = if above {
            index.checked_add(1)
        } else {
            index.checked_sub(1)
        }
        .ok_or_else(|| invalid("adjacent layer absent"))?;
        let layer = self.layer(neighbor)?;
        let even = index.is_multiple_of(2);
        let bounds = |c: i32| -> (f32, f32) {
            if above {
                if even {
                    (
                        (f64::from((4 * c - 3) as f32) / 6.0) as f32,
                        (f64::from((4 * c + 1) as f32) / 6.0) as f32,
                    )
                } else {
                    ((6 * c - 4) as f32 / 8.0f32, (6 * c + 2) as f32 / 8.0f32)
                }
            } else if even {
                (
                    (f64::from((8 * c - 3) as f32) / 6.0) as f32,
                    (f64::from((8 * c + 5) as f32) / 6.0) as f32,
                )
            } else {
                (
                    (f64::from((6 * c - 2) as f32) / 4.0) as f32,
                    (f64::from((6 * c + 4) as f32) / 4.0) as f32,
                )
            }
        };
        let (xlo, xhi) = bounds(x);
        let (ylo, yhi) = bounds(y);
        let Some((max, mx, my)) =
            Self::scan_score_window(layer, [xlo, xhi, ylo, yhi], threshold, above)?
        else {
            return Ok(None);
        };
        let (refined, dx, dy) = Self::subpixel(&Self::patch(layer, mx, my, false)?);
        let (rx, ry) = (mx as f32 + dx, my as f32 + dy);
        let (mut dx, mut dy) = if above && even {
            (
                (rx * 6.0f32 + 1.0f32) / 4.0f32 - x as f32,
                (ry * 6.0f32 + 1.0f32) / 4.0f32 - y as f32,
            )
        } else if above {
            (
                ((f64::from(rx) * 8.0 + 1.0) / 6.0 - f64::from(x as f32)) as f32,
                ((f64::from(ry) * 8.0 + 1.0) / 6.0 - f64::from(y as f32)) as f32,
            )
        } else if even {
            (
                ((f64::from(rx) * 6.0 + 1.0) / 8.0 - f64::from(x as f32)) as f32,
                ((f64::from(ry) * 6.0 + 1.0) / 8.0 - f64::from(y as f32)) as f32,
            )
        } else {
            (
                ((f64::from(rx) * 4.0 - 1.0) / 6.0 - f64::from(x as f32)) as f32,
                ((f64::from(ry) * 4.0 - 1.0) / 6.0 - f64::from(y as f32)) as f32,
            )
        };
        let mut return_refined = true;
        if dx > 1.0 {
            dx = 1.0;
            return_refined = false;
        }
        if dx < -1.0 {
            dx = -1.0;
            return_refined = false;
        }
        if dy > 1.0 {
            dy = 1.0;
            return_refined = false;
        }
        if dy < -1.0 {
            dy = -1.0;
            return_refined = false;
        }
        Ok(Some((
            if return_refined {
                max_f32(refined, max)
            } else {
                max
            },
            dx,
            dy,
        )))
    }
    fn scan_score_window(
        layer: &BriskLayer,
        bounds: [f32; 4],
        threshold: i32,
        above: bool,
    ) -> Result<Option<(f32, i32, i32)>> {
        let [xlo, xhi, ylo, yhi] = bounds;
        let (mut mx, mut my) = ((f64::from(xlo) + 1.0) as i32, (f64::from(ylo) + 1.0) as i32);
        let mut max = f32::from(layer.get_agast_score_interpolated(xlo, ylo, 1, 1.0)?);
        let limit = threshold + 5;
        if max > limit as f32 {
            return Ok(None);
        }
        for xx in (f64::from(xlo) + 1.0) as i32..=xhi as i32 {
            let v = f32::from(layer.get_agast_score_interpolated(xx as f32, ylo, 1, 1.0)?);
            if v > limit as f32 {
                return Ok(None);
            }
            if v > max {
                max = v;
                mx = xx;
            }
        }
        let v = f32::from(layer.get_agast_score_interpolated(xhi, ylo, 1, 1.0)?);
        if v > limit as f32 {
            return Ok(None);
        }
        if v > max {
            max = v;
            mx = xhi as i32;
        }
        for yy in (f64::from(ylo) + 1.0) as i32..=yhi as i32 {
            let v = f32::from(layer.get_agast_score_interpolated(xlo, yy as f32, 1, 1.0)?);
            if v > limit as f32 {
                return Ok(None);
            }
            if v > max {
                max = v;
                mx = (f64::from(xlo) + 1.0) as i32;
                my = yy;
            }
            for xx in (f64::from(xlo) + 1.0) as i32..=xhi as i32 {
                let v = f32::from(layer.get_agast_score(xx, yy, 1)?);
                if v > limit as f32 {
                    return Ok(None);
                }
                if !above
                    && v == max
                    && Self::neighbor_sum(layer, xx, yy)? > Self::neighbor_sum(layer, mx, my)?
                {
                    mx = xx;
                    my = yy;
                }
                if v > max {
                    max = v;
                    mx = xx;
                    my = yy;
                }
            }
            let v = f32::from(layer.get_agast_score_interpolated(xhi, yy as f32, 1, 1.0)?);
            if v > limit as f32 {
                return Ok(None);
            }
            if v > max {
                max = v;
                mx = xhi as i32;
                my = yy;
            }
        }
        let v = f32::from(layer.get_agast_score_interpolated(xlo, yhi, 1, 1.0)?);
        if v > max {
            max = v;
            mx = (f64::from(xlo) + 1.0) as i32;
            my = yhi as i32;
        }
        for xx in (f64::from(xlo) + 1.0) as i32..=xhi as i32 {
            let v = f32::from(layer.get_agast_score_interpolated(xx as f32, yhi, 1, 1.0)?);
            if v > max {
                max = v;
                mx = xx;
                my = yhi as i32;
            }
        }
        let v = f32::from(layer.get_agast_score_interpolated(xhi, yhi, 1, 1.0)?);
        if v > max {
            max = v;
            mx = xhi as i32;
            my = yhi as i32;
        }
        Ok(Some((max, mx, my)))
    }
    /// # Errors
    /// Returns an error if a required pyramid layer is absent or image/score sampling fails.
    pub fn refine_3d(&self, index: usize, x: i32, y: i32) -> Result<Option<(f32, f32, f32, f32)>> {
        let layer = self.layer(index)?;
        let center = i32::from(layer.get_agast_score(x, y, 1)?);
        let Some((above, ax, ay)) = self.get_score_max_above(index, x, y, center)? else {
            return Ok(None);
        };
        let even = index.is_multiple_of(2);
        let (below, bx, by) = if index == 0 {
            let p = Self::patch(self.layer(0)?, x, y, true)?;
            let (_, dx, dy) = Self::subpixel(&p);
            (
                *p.iter().max().ok_or_else(|| invalid("empty score patch"))? as f32,
                dx,
                dy,
            )
        } else {
            let Some(v) = self.get_score_max_below(index, x, y, center)? else {
                return Ok(None);
            };
            v
        };
        let p = Self::patch(layer, x, y, false)?;
        let mut do_scale = true;
        if index == 0 {
            if p[4] - 1 <= above as i32 {
                do_scale = false;
            }
        } else if ((p[4] - 1) as f32) < above || ((p[4] - 1) as f32) < below {
            if (p[4] - 15) as f32 > above || (p[4] - 15) as f32 > below {
                do_scale = false;
            } else {
                return Ok(None);
            }
        }
        let (max_layer, lx, ly) = Self::subpixel(&p);
        let (scale, score) = if do_scale {
            let middle = max_f32(center as f32, max_layer);
            if index == 0 {
                brisk_refine_1d_2(below, middle, above)
            } else if even {
                brisk_refine_1d(below, middle, above)
            } else {
                brisk_refine_1d_1(below, middle, above)
            }
        } else {
            (1.0, max_layer)
        };
        let (r0, r1, dx, dy) = if scale > 1.0 {
            let r0 = if even {
                ((1.5 - f64::from(scale)) / 0.5) as f32
            } else {
                (4.0 - f64::from(scale) * 3.0) as f32
            };
            (r0, (1.0 - f64::from(r0)) as f32, ax, ay)
        } else {
            let r0 = if index == 0 {
                ((f64::from(scale) - 0.5) / 0.5) as f32
            } else if even {
                ((f64::from(scale) - 0.75) / 0.25) as f32
            } else {
                (f64::from(scale) * 3.0 - 2.0) as f32
            };
            (r0, (1.0 - f64::from(r0)) as f32, bx, by)
        };
        let (mut rx, mut ry) = (r0 * lx + r1 * dx + x as f32, r0 * ly + r1 * dy + y as f32);
        if index != 0 || scale > 1.0 {
            rx = rx * layer.scale + layer.offset;
            ry = ry * layer.scale + layer.offset;
        }
        Ok(Some((score, rx, ry, scale * layer.scale)))
    }
}
#[must_use]
pub fn brisk_refine_1d(
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
pub fn brisk_refine_1d_1(
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
pub fn brisk_refine_1d_2(
    lower_response: f32,
    center_response: f32,
    upper_response: f32,
) -> (f32, f32) {
    let mut max: f32;

    let lower_fixed: i32 = ((1024.0f64 * f64::from(lower_response)) + 0.5f64) as i32;
    let center_fixed: i32 = ((1024.0f64 * f64::from(center_response)) + 0.5f64) as i32;
    let upper_fixed: i32 = ((1024.0f64 * f64::from(upper_response)) + 0.5f64) as i32;
    let a: i32 = (((2i32).wrapping_mul(lower_fixed))
        .wrapping_sub((4i32).wrapping_mul(center_fixed)))
    .wrapping_add((2i32).wrapping_mul(upper_fixed));
    if a >= 0i32 {
        if (center_response >= lower_response) && (center_response >= upper_response) {
            max = center_response;
            return ((1.0f64 as f32), max);
        }
        if (lower_response >= center_response) && (lower_response >= upper_response) {
            max = lower_response;
            return ((0.7f64 as f32), max);
        }
        if (upper_response >= center_response) && (upper_response >= lower_response) {
            max = upper_response;
            return ((1.5f64 as f32), max);
        }
    }
    let b: i32 = ((((5i32).wrapping_neg()).wrapping_mul(lower_fixed))
        .wrapping_add((8i32).wrapping_mul(center_fixed)))
    .wrapping_sub((3i32).wrapping_mul(upper_fixed));
    let mut ret_val: f32 = -(b as f32) / ((2i32).wrapping_mul(a) as f32);
    if f64::from(ret_val) < 0.7f64 {
        ret_val = 0.7f64 as f32;
    } else if f64::from(ret_val) > 1.5f64 {
        ret_val = 1.5f64 as f32;
    }
    let c: i32 = (((3i32).wrapping_mul(lower_fixed))
        .wrapping_sub((3i32).wrapping_mul(center_fixed)))
    .wrapping_add((1i32).wrapping_mul(upper_fixed));
    max = ((c as f32) + (((a as f32) * ret_val) * ret_val)) + ((b as f32) * ret_val);
    max /= 1024_f32;
    (ret_val, max)
}
#[must_use]
pub fn brisk_subpixel_2d(samples: &[[i32; 3]; 3]) -> (f32, f32, f32) {
    let mut delta_x: f32;
    let mut delta_y: f32;

    let tmp1: i32 = ((((samples[0][0]).wrapping_add(samples[0][2]))
        .wrapping_sub((2i32).wrapping_mul(samples[1][1])))
    .wrapping_add(samples[2][0]))
    .wrapping_add(samples[2][2]);
    let coeff1: i32 = (3i32).wrapping_mul(
        (((tmp1).wrapping_add(samples[0][1]))
            .wrapping_sub(((samples[1][0]).wrapping_add(samples[1][2])).wrapping_shl(1i32 as u32)))
        .wrapping_add(samples[2][1]),
    );
    let coeff2: i32 = (3i32).wrapping_mul(
        (((tmp1).wrapping_sub(
            ((samples[0][1]).wrapping_add(samples[2][1])).wrapping_shl(1i32 as u32),
        ))
        .wrapping_add(samples[1][0]))
        .wrapping_add(samples[1][2]),
    );
    let tmp2: i32 = (samples[0][2]).wrapping_sub(samples[2][0]);
    let tmp3: i32 = ((samples[0][0]).wrapping_add(tmp2)).wrapping_sub(samples[2][2]);
    let tmp4: i32 = (tmp3).wrapping_sub((2i32).wrapping_mul(tmp2));
    let coeff3: i32 = ((3i32).wrapping_neg())
        .wrapping_mul(((tmp3).wrapping_add(samples[0][1])).wrapping_sub(samples[2][1]));
    let coeff4: i32 = ((3i32).wrapping_neg())
        .wrapping_mul(((tmp4).wrapping_add(samples[1][0])).wrapping_sub(samples[1][2]));
    let coeff5: i32 = ((((samples[0][0]).wrapping_sub(samples[0][2])).wrapping_sub(samples[2][0]))
        .wrapping_add(samples[2][2]))
    .wrapping_shl(2i32 as u32);
    let coeff6: i32 = (((((((samples[0][0]).wrapping_add(samples[0][2])).wrapping_sub(
        ((((samples[1][0]).wrapping_add(samples[0][1])).wrapping_add(samples[1][2]))
            .wrapping_add(samples[2][1]))
        .wrapping_shl(1i32 as u32),
    ))
    .wrapping_sub((5i32).wrapping_mul(samples[1][1])))
    .wrapping_add(samples[2][0]))
    .wrapping_add(samples[2][2]))
    .wrapping_neg())
    .wrapping_shl(1i32 as u32);
    let h_det: i32 = (((4i32).wrapping_mul(coeff1)).wrapping_mul(coeff2))
        .wrapping_sub((coeff5).wrapping_mul(coeff5));
    if h_det == 0i32 {
        delta_x = 0.0f64 as f32;
        delta_y = 0.0f64 as f32;
        return (
            ((f64::from(coeff6 as f32) / 18.0f64) as f32),
            delta_x,
            delta_y,
        );
    }
    if !((h_det > 0i32) && (coeff1 < 0i32)) {
        let mut tmp_max: i32 = ((coeff3).wrapping_add(coeff4)).wrapping_add(coeff5);
        delta_x = 1.0f64 as f32;
        delta_y = 1.0f64 as f32;
        let mut tmp: i32 = (((coeff3).wrapping_neg()).wrapping_add(coeff4)).wrapping_sub(coeff5);
        if tmp > tmp_max {
            tmp_max = tmp;
            delta_x = -(1.0f64) as f32;
            delta_y = 1.0f64 as f32;
        }
        tmp = ((coeff3).wrapping_sub(coeff4)).wrapping_sub(coeff5);
        if tmp > tmp_max {
            tmp_max = tmp;
            delta_x = 1.0f64 as f32;
            delta_y = -(1.0f64) as f32;
        }
        tmp = (((coeff3).wrapping_neg()).wrapping_sub(coeff4)).wrapping_add(coeff5);
        if tmp > tmp_max {
            tmp_max = tmp;
            delta_x = -(1.0f64) as f32;
            delta_y = -(1.0f64) as f32;
        }
        return (
            ((f64::from(
                (((tmp_max).wrapping_add(coeff1)).wrapping_add(coeff2)).wrapping_add(coeff6) as f32,
            ) / 18.0f64) as f32),
            delta_x,
            delta_y,
        );
    }
    delta_x = ((((2i32).wrapping_mul(coeff2)).wrapping_mul(coeff3))
        .wrapping_sub((coeff4).wrapping_mul(coeff5)) as f32)
        / ((h_det).wrapping_neg() as f32);
    delta_y = ((((2i32).wrapping_mul(coeff1)).wrapping_mul(coeff4))
        .wrapping_sub((coeff3).wrapping_mul(coeff5)) as f32)
        / ((h_det).wrapping_neg() as f32);
    constrain_subpixel_patch(
        [coeff1, coeff2, coeff3, coeff4, coeff5, coeff6],
        delta_x,
        delta_y,
    )
}
fn constrain_subpixel_patch(
    coefficients: [i32; 6],
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
            horizontal_candidate_y =
                -((coeff4).wrapping_add(coeff5) as f32) / ((2i32).wrapping_mul(coeff2) as f32);
            if f64::from(horizontal_candidate_y) > 1.0f64 {
                horizontal_candidate_y = 1.0f64 as f32;
            } else if f64::from(horizontal_candidate_y) < -(1.0f64) {
                horizontal_candidate_y = -(1.0f64) as f32;
            }
        } else if tx_ {
            horizontal_candidate_x = -(1.0f64) as f32;
            horizontal_candidate_y =
                -((coeff4).wrapping_sub(coeff5) as f32) / ((2i32).wrapping_mul(coeff2) as f32);
            if f64::from(horizontal_candidate_y) > 1.0f64 {
                horizontal_candidate_y = 1.0f64 as f32;
            } else if f64::from(horizontal_candidate_y) < -(1.0f64) {
                horizontal_candidate_y = -(1.0f64) as f32;
            }
        }
        if ty {
            vertical_candidate_y = 1.0f64 as f32;
            vertical_candidate_x =
                -((coeff3).wrapping_add(coeff5) as f32) / ((2i32).wrapping_mul(coeff1) as f32);
            if f64::from(vertical_candidate_x) > 1.0f64 {
                vertical_candidate_x = 1.0f64 as f32;
            } else if f64::from(vertical_candidate_x) < -(1.0f64) {
                vertical_candidate_x = -(1.0f64) as f32;
            }
        } else if ty_ {
            vertical_candidate_y = -(1.0f64) as f32;
            vertical_candidate_x =
                -((coeff3).wrapping_sub(coeff5) as f32) / ((2i32).wrapping_mul(coeff1) as f32);
            if f64::from(vertical_candidate_x) > 1.0f64 {
                vertical_candidate_x = 1.0f64 as f32;
            } else if f64::from(vertical_candidate_x) < -(1.0f64) {
                vertical_candidate_x = -(1.0f64) as f32;
            }
        }
        let max1: f32 = (f64::from(
            (((((((coeff1 as f32) * horizontal_candidate_x) * horizontal_candidate_x)
                + (((coeff2 as f32) * horizontal_candidate_y) * horizontal_candidate_y))
                + ((coeff3 as f32) * horizontal_candidate_x))
                + ((coeff4 as f32) * horizontal_candidate_y))
                + (((coeff5 as f32) * horizontal_candidate_x) * horizontal_candidate_y))
                + (coeff6 as f32),
        ) / 18.0f64) as f32;
        let max2: f32 = (f64::from(
            (((((((coeff1 as f32) * vertical_candidate_x) * vertical_candidate_x)
                + (((coeff2 as f32) * vertical_candidate_y) * vertical_candidate_y))
                + ((coeff3 as f32) * vertical_candidate_x))
                + ((coeff4 as f32) * vertical_candidate_y))
                + (((coeff5 as f32) * vertical_candidate_x) * vertical_candidate_y))
                + (coeff6 as f32),
        ) / 18.0f64) as f32;
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
        ((f64::from(
            (((((((coeff1 as f32) * delta_x) * delta_x)
                + (((coeff2 as f32) * delta_y) * delta_y))
                + ((coeff3 as f32) * delta_x))
                + ((coeff4 as f32) * delta_y))
                + (((coeff5 as f32) * delta_x) * delta_y))
                + (coeff6 as f32),
        ) / 18.0f64) as f32),
        delta_x,
        delta_y,
    )
}
