// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original GrayModel.h/GrayModel.cc.
use nalgebra::{Matrix4, Vector4};
#[derive(Default)]
pub struct GrayModel {
    matrix: Matrix4<f64>,
    vector: Vector4<f64>,
    rhs: Vector4<f64>,
    observations: usize,
    dirty: bool,
}
impl GrayModel {
    pub fn add_observation(&mut self, x: f32, y: f32, gray: f32) {
        let xy = x * y;
        let row = [x, y, xy, 1.];
        for r in 0..4 {
            for c in r..4 {
                self.matrix[(r, c)] += f64::from(row[r] * row[c]);
            }
        }
        self.rhs[0] += f64::from(x * gray);
        self.rhs[1] += f64::from(y * gray);
        self.rhs[2] += f64::from(xy * gray);
        self.rhs[3] += f64::from(gray);
        self.observations += 1;
        self.dirty = true;
    }
    pub fn interpolate(&mut self, x: f32, y: f32) -> f32 {
        if self.dirty {
            self.compute();
        }
        (self.vector[0] * f64::from(x)
            + self.vector[1] * f64::from(y)
            + self.vector[2] * f64::from(x) * f64::from(y)
            + self.vector[3]) as f32
    }
    fn compute(&mut self) {
        self.dirty = false;
        if self.observations >= 6 {
            for i in 0..4 {
                for j in i + 1..4 {
                    self.matrix[(j, i)] = self.matrix[(i, j)];
                }
            }
            if self.matrix.determinant().abs() > 1e-12
                && let Some(inverse) = self.matrix.try_inverse()
            {
                self.vector = inverse * self.rhs;
                return;
            }
            log::warn!("AprilTags gray model matrix inverse underflow");
        }
        self.vector.fill(0.);
        self.vector[3] = self.rhs[3] / self.observations as f64;
    }
}
