/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Pinhole depth min/max pyramid with conservative image-boundary queries.
use crate::{
    image::{Image, invalid},
    uncertainty::{cpp_max, cpp_min},
};
use firefly_error::Result;
use nalgebra::Vector2;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum Crossing {
    Inside = 0,
    Crossing = 1,
    Outside = 2,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum Known {
    Known = 0,
    PartKnown = 1,
    Unknown = 2,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pixel {
    pub min: f32,
    pub max: f32,
    pub status_crossing: Crossing,
    pub status_known: Known,
}
impl Pixel {
    #[must_use]
    pub fn known() -> Self {
        Self {
            min: f32::MAX,
            max: f32::MIN_POSITIVE,
            status_crossing: Crossing::Inside,
            status_known: Known::Known,
        }
    }
    #[must_use]
    pub fn unknown() -> Self {
        Self {
            status_known: Known::Unknown,
            ..Self::known()
        }
    }
    #[must_use]
    pub fn crossing_known() -> Self {
        Self {
            status_crossing: Crossing::Crossing,
            ..Self::known()
        }
    }
    #[must_use]
    pub fn crossing_part_known() -> Self {
        Self {
            status_known: Known::PartKnown,
            ..Self::crossing_known()
        }
    }
    #[must_use]
    pub fn crossing_unknown() -> Self {
        Self {
            status_known: Known::Unknown,
            ..Self::crossing_known()
        }
    }
    #[must_use]
    pub fn outside() -> Self {
        Self {
            min: 0.0,
            max: 0.0,
            status_crossing: Crossing::Outside,
            status_known: Known::Unknown,
        }
    }
}
pub struct DensePoolingImage {
    pub width: i32,
    pub height: i32,
    pub max_level: i32,
    pub max_value: f32,
    pub levels: Vec<Vec<Pixel>>,
}
impl DensePoolingImage {
    /// # Errors
    /// Returns an error unless image storage is single-channel and contiguous, dimensions fit
    /// `i32`, and both dimensions are at least three.
    pub fn new(image: &Image<f32>) -> Result<Self> {
        if image.channels != 1
            || image.rows.checked_mul(image.cols) != Some(image.data.len())
            || image.rows > i32::MAX as usize
            || image.cols > i32::MAX as usize
        {
            return Err(invalid("pooling image shape mismatch"));
        }
        let (width, height) = (image.cols as i32, image.rows as i32);
        let dim = width.min(height);
        if dim < 3 {
            return Err(invalid(
                "pooling pyramid requires dimensions at least three",
            ));
        }
        let max_level = (f64::from((dim - 1) / 2).log2() + 2.0) as i32 - 1;
        let mut levels = vec![vec![Pixel::unknown(); image.data.len()]; max_level as usize + 1];
        for (pixel, &value) in levels[0].iter_mut().zip(&image.data) {
            *pixel = if value <= 0.0 {
                Pixel::unknown()
            } else {
                Pixel {
                    min: value,
                    max: value,
                    ..Pixel::known()
                }
            };
        }
        pool_first_level(width, height, &mut levels);
        let mut stride = 2;
        for level in 2..=max_level {
            let half = stride / 2;
            for y in 0..height {
                for x in 0..width {
                    let (left, right, top, bottom) = (
                        (x - half).max(0),
                        (x + half).min(width - 1),
                        (y - half).max(0),
                        (y + half).min(height - 1),
                    );
                    let samples = [
                        levels[level as usize - 1][(left + top * width) as usize],
                        levels[level as usize - 1][(right + top * width) as usize],
                        levels[level as usize - 1][(left + bottom * width) as usize],
                        levels[level as usize - 1][(right + bottom * width) as usize],
                    ];
                    let mut pixel = Pixel::crossing_known();
                    pixel.min = cpp_min(
                        cpp_min(samples[0].min, samples[1].min),
                        cpp_min(samples[2].min, samples[3].min),
                    );
                    pixel.max = cpp_max(
                        cpp_max(samples[0].max, samples[1].max),
                        cpp_max(samples[2].max, samples[3].max),
                    );
                    let sum: i32 = samples.iter().map(|v| v.status_known as i32).sum();
                    pixel.status_known = if sum == 8 {
                        Known::Unknown
                    } else if sum > 0 {
                        Known::PartKnown
                    } else {
                        Known::Known
                    };
                    if y >= stride && y < height - stride && x >= stride && x < width - stride {
                        pixel.status_crossing = Crossing::Inside;
                    }
                    levels[level as usize][(x + y * width) as usize] = pixel;
                }
            }
            stride <<= 1;
        }
        let stride = 1 << (max_level - 1);
        let mut max_value = 0.0;
        for y in terminal_samples(stride, height - stride - 1, 2 * stride) {
            for x in terminal_samples(stride, width - stride - 1, 2 * stride) {
                let pixel = levels[max_level as usize][(x + y * width) as usize];
                if max_value < pixel.max {
                    max_value = pixel.max;
                }
            }
        }
        Ok(Self {
            width,
            height,
            max_level,
            max_value,
            levels,
        })
    }
    #[must_use]
    pub fn in_image(&self, u: i32, v: i32) -> bool {
        u >= 0 && u < self.width && v >= 0 && v < self.height
    }
    /// # Errors
    /// Returns an error for out-of-image bounds or a window whose pooling level cannot be
    /// represented; one-coordinate-wide windows are rejected.
    pub fn pool_bounding_box(
        &self,
        u_min: i32,
        u_max: i32,
        v_min: i32,
        v_max: i32,
    ) -> Result<Pixel> {
        if !self.in_image(u_min, v_min)
            || !self.in_image(u_max, v_max)
            || u_min > u_max
            || v_min > v_max
        {
            return Err(invalid("pooling bounds outside image"));
        }
        let side = (u_max - u_min).min(v_max - v_min);
        if side < 2 {
            return Err(invalid(
                "pooling level uses logarithm of zero for box side below two",
            ));
        }
        let level = (f64::from(side / 2).log2().ceil() as i32)
            .min(self.max_level)
            .max(0);
        let s = if level > 0 {
            1 << (level - 1).max(0)
        } else {
            0
        };
        let mut batch = Pixel::known();
        let (mut count, mut unknown, mut partly) = (0, 0, 0);
        for y in terminal_samples(v_min + s, v_max - s, 2 * s + 1) {
            for x in terminal_samples(u_min + s, u_max - s, 2 * s + 1) {
                let p = self.levels[level as usize][(x + y * self.width) as usize];
                if batch.max < p.max {
                    batch.max = p.max;
                }
                if batch.min > p.min {
                    batch.min = p.min;
                }
                if p.status_known == Known::Unknown {
                    unknown += 1;
                    partly += 1;
                } else if p.status_known == Known::PartKnown {
                    partly += 1;
                }
                count += 1;
            }
        }
        batch.status_known = if count == unknown {
            Known::Unknown
        } else if partly > 0 {
            Known::PartKnown
        } else {
            Known::Known
        };
        Ok(batch)
    }
    /// # Errors
    /// Returns an error for out-of-image bounds or a window whose pooling level cannot be
    /// represented; one-coordinate-wide windows are rejected.
    pub fn conservative_query(
        &self,
        bb_min: &Vector2<i32>,
        bb_max: &Vector2<i32>,
    ) -> Result<Pixel> {
        let (mut u_min, mut v_min, mut u_max, mut v_max) = (bb_min.x, bb_min.y, bb_max.x, bb_max.y);
        let (
            mut fully_horizontal,
            mut outside_horizontal,
            mut fully_vertical,
            mut outside_vertical,
        ) = (true, false, true, false);
        if u_min < 0 {
            fully_horizontal = false;
            u_min = 0;
            if u_max < 0 {
                outside_horizontal = true;
            }
        } else if u_min > self.width - 1 {
            outside_horizontal = true;
        }
        if u_max > self.width - 1 {
            fully_horizontal = false;
            u_max = self.width - 1;
        }
        if v_min < 0 {
            fully_vertical = false;
            v_min = 0;
            if v_max < 0 {
                outside_vertical = true;
            }
        } else if v_min > self.height - 1 {
            outside_vertical = true;
        }
        if v_max > self.height - 1 {
            fully_vertical = false;
            v_max = self.height - 1;
        }
        if outside_horizontal || outside_vertical || u_min == u_max || v_min == v_max {
            return Ok(Pixel::outside());
        }
        let mut p = self.pool_bounding_box(u_min, u_max, v_min, v_max)?;
        if !fully_horizontal || !fully_vertical {
            p.status_crossing = Crossing::Crossing;
        }
        Ok(p)
    }
}
fn terminal_samples(start: i32, end: i32, step: i32) -> Vec<i32> {
    let mut result = Vec::new();
    let mut v = start;
    loop {
        if v > end {
            v = end;
        }
        result.push(v);
        if v == end {
            break;
        }
        v += step;
    }
    result
}

fn pool_first_level(width: i32, height: i32, levels: &mut [Vec<Pixel>]) {
    for y in 0..height {
        for x in 0..width {
            let (left, right, top, bottom) = (
                (x - 1).max(0),
                (x + 1).min(width - 1),
                (y - 1).max(0),
                (y + 1).min(height - 1),
            );
            let coords = [
                (x, y),
                (left, top),
                (x, top),
                (right, top),
                (right, y),
                (right, bottom),
                (x, bottom),
                (left, bottom),
                (left, y),
            ];
            let samples = coords.map(|(xx, yy)| levels[0][(xx + yy * width) as usize]);
            let mut pixel = Pixel::crossing_known();
            if y >= 1 && y < height - 1 && x >= 1 && x < width - 1 {
                pixel.status_crossing = Crossing::Inside;
            }
            pixel.min = cpp_min(
                samples[0].min,
                cpp_min(
                    cpp_min(
                        cpp_min(samples[1].min, samples[2].min),
                        cpp_min(samples[3].min, samples[4].min),
                    ),
                    cpp_min(
                        cpp_min(samples[5].min, samples[6].min),
                        cpp_min(samples[7].min, samples[8].min),
                    ),
                ),
            );
            pixel.max = cpp_max(
                samples[0].max,
                cpp_max(
                    cpp_max(
                        cpp_max(samples[1].max, samples[2].max),
                        cpp_max(samples[3].max, samples[4].max),
                    ),
                    cpp_max(
                        cpp_max(samples[5].max, samples[6].max),
                        cpp_max(samples[7].max, samples[8].max),
                    ),
                ),
            );
            let sum: i32 = samples.iter().map(|v| v.status_known as i32).sum();
            pixel.status_known = if sum == 18 {
                Known::Unknown
            } else if sum > 0 {
                Known::PartKnown
            } else {
                Known::Known
            };
            levels[1][(x + y * width) as usize] = pixel;
        }
    }
}
