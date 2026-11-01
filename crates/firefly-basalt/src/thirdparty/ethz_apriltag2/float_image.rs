// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original FloatImage.h/FloatImage.cc.
use super::gaussian::convolve_symmetric_centered;
#[derive(Clone, Debug, Default)]
pub struct FloatImage {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<f32>,
}
impl FloatImage {
    #[must_use]
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            pixels: vec![0.; width * height],
        }
    }
    #[must_use]
    pub fn get(&self, x: usize, y: usize) -> f32 {
        self.pixels[y * self.width + x]
    }
    pub fn set(&mut self, x: usize, y: usize, value: f32) {
        self.pixels[y * self.width + x] = value;
    }
    pub fn decimate_avg(&mut self) {
        let width = self.width / 2;
        let height = self.height / 2;
        for y in 0..height {
            for x in 0..width {
                self.pixels[y * width + x] = self.pixels[2 * y * self.width + 2 * x];
            }
        }
        self.width = width;
        self.height = height;
        self.pixels.truncate(width * height);
    }
    pub fn normalize(&mut self) {
        let minimum = self.pixels.iter().copied().fold(f32::INFINITY, f32::min);
        let maximum = self
            .pixels
            .iter()
            .copied()
            .fold(f32::NEG_INFINITY, f32::max);
        let inverse = 1. / (maximum - minimum);
        for value in &mut self.pixels {
            *value = (*value - minimum) * inverse;
        }
    }
    pub fn filter_factored_centered(&mut self, horizontal: &[f32], vertical: &[f32]) {
        let mut intermediate = self.pixels.clone();
        for y in 0..self.height {
            convolve_symmetric_centered(
                &self.pixels,
                y * self.width,
                self.width,
                horizontal,
                &mut intermediate,
                y * self.width,
            );
        }
        let mut column = vec![0.; self.height];
        let mut filtered = column.clone();
        for x in 0..self.width {
            for y in 0..self.height {
                column[y] = intermediate[y * self.width + x];
            }
            convolve_symmetric_centered(&column, 0, self.height, vertical, &mut filtered, 0);
            for (y, &value) in filtered.iter().enumerate() {
                self.pixels[y * self.width + x] = value;
            }
        }
    }
}
