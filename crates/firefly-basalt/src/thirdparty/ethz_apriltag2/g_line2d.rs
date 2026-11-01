// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original GLine2D.h/GLine2D.cc.
use super::xy_weight::XYWeight;
#[derive(Clone, Debug, Default)]
pub struct GLine2D {
    pub dx: f32,
    pub dy: f32,
    pub point: (f32, f32),
    normalized_slope: bool,
    normalized_point: bool,
}
impl GLine2D {
    #[must_use]
    pub fn new(dx: f32, dy: f32, point: (f32, f32)) -> Self {
        Self {
            dx,
            dy,
            point,
            normalized_slope: false,
            normalized_point: false,
        }
    }
    #[must_use]
    pub fn from_points(a: (f32, f32), b: (f32, f32)) -> Self {
        Self::new(b.0 - a.0, b.1 - a.1, a)
    }
    pub fn normalize_slope(&mut self) {
        if !self.normalized_slope {
            let magnitude = (self.dx * self.dx + self.dy * self.dy).sqrt();
            self.dx /= magnitude;
            self.dy /= magnitude;
            self.normalized_slope = true;
        }
    }
    pub fn normalize_point(&mut self) {
        if !self.normalized_point {
            self.normalize_slope();
            let dot = -self.dy * self.point.0 + self.dx * self.point.1;
            self.point = (-self.dy * dot, self.dx * dot);
            self.normalized_point = true;
        }
    }
    pub fn line_coordinate(&mut self, point: (f32, f32)) -> f32 {
        self.normalize_slope();
        point.0 * self.dx + point.1 * self.dy
    }
    pub fn point_of_coordinate(&mut self, coordinate: f32) -> (f32, f32) {
        self.normalize_point();
        (
            self.point.0 + coordinate * self.dx,
            self.point.1 + coordinate * self.dy,
        )
    }
    #[must_use]
    pub fn intersection(&self, other: &Self) -> (f32, f32) {
        let determinant = self.dx * (-other.dy) - (-other.dx) * self.dy;
        if determinant.abs() < 1e-10 {
            return (-1., 0.);
        }
        let first = -other.dy / determinant;
        let second = other.dx / determinant;
        let distance =
            first * (other.point.0 - self.point.0) + second * (other.point.1 - self.point.1);
        (
            self.dx * distance + self.point.0,
            self.dy * distance + self.point.1,
        )
    }
    #[must_use]
    pub fn lsq_fit(points: &[XYWeight]) -> Self {
        let (mut mx, mut my, mut mxx, mut myy, mut mxy, mut n) = (0., 0., 0., 0., 0., 0.);
        for p in points {
            my += p.y * p.weight;
            mx += p.x * p.weight;
            myy += p.y * p.y * p.weight;
            mxx += p.x * p.x * p.weight;
            mxy += p.x * p.y * p.weight;
            n += p.weight;
        }
        let ex = mx / n;
        let ey = my / n;
        let cxx = mxx / n - ex * ex;
        let cyy = myy / n - ey * ey;
        let cxy = mxy / n - ex * ey;
        let phi = 0.5 * (-2. * cxy).atan2(cyy - cxx);
        Self::new(-phi.sin(), phi.cos(), (ex, ey))
    }
}
