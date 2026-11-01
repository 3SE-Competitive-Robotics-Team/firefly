// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original `Homography33.h/.cc`, with the upstream enabled `STABLE_H` configuration.
use nalgebra::Matrix3;
use purecv::{
    calib3d::homography::{HomographyMethod, find_homography},
    core::types::Point2f,
};
#[derive(Clone, Debug)]
pub struct Homography33 {
    pub center: (f32, f32),
    pub matrix: Matrix3<f64>,
}
impl Homography33 {
    #[must_use]
    /// # Panics
    /// Panics if the homography backend violates its 3x3 output contract.
    pub fn new(
        source: &[(f32, f32)],
        destination: &[(f32, f32)],
        center: (f32, f32),
    ) -> Option<Self> {
        let source: Vec<_> = source.iter().map(|p| Point2f::new(p.0, p.1)).collect();
        let destination: Vec<_> = destination
            .iter()
            .map(|p| Point2f::new(p.0 - center.0, p.1 - center.1))
            .collect();
        let homography =
            find_homography(&source, &destination, HomographyMethod::None, 3., None).ok()?;
        Some(Self {
            center,
            matrix: Matrix3::from_fn(|r, c| *homography.get(r, c, 0).expect("3x3 homography")),
        })
    }
    #[must_use]
    pub fn project(&self, x: f32, y: f32) -> (f32, f32) {
        let x = f64::from(x);
        let y = f64::from(y);
        let numerator_x =
            (self.matrix[(0, 0)] * x + self.matrix[(0, 1)] * y + self.matrix[(0, 2)]) as f32;
        let numerator_y =
            (self.matrix[(1, 0)] * x + self.matrix[(1, 1)] * y + self.matrix[(1, 2)]) as f32;
        let z = (self.matrix[(2, 0)] * x + self.matrix[(2, 1)] * y + self.matrix[(2, 2)]) as f32;
        (
            numerator_x / z + self.center.0,
            numerator_y / z + self.center.1,
        )
    }
}
