// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See docs/licenses/README.md for the complete notice and advertising acknowledgement.
//! Original TagDetection.h/TagDetection.cc detection geometry.
use super::math_util::distance_2d;
use nalgebra::{Matrix3, Matrix4, Vector3};
#[path = "planar_pose.rs"]
mod planar_pose;
/// Raster backend contract for the original diagnostic `draw` helper.
/// Coordinates must use nearest-even integer conversion, 8-connected lines,
/// and `OpenCV`'s Hershey Plain font. Keeping this boundary explicit avoids
/// substituting a different rasterizer while the runtime uses Rerun overlays.
pub trait TagDrawing {
    type Error;
    /// # Errors
    /// Reports a raster backend failure.
    fn line(&mut self, start: (f32, f32), end: (f32, f32), bgr: [u8; 3])
    -> Result<(), Self::Error>;
    /// # Errors
    /// Reports a raster backend failure.
    fn circle(
        &mut self,
        center: (f32, f32),
        radius: i32,
        thickness: i32,
        bgr: [u8; 3],
    ) -> Result<(), Self::Error>;
    /// # Errors
    /// Reports a raster backend failure.
    fn hershey_plain(
        &mut self,
        origin: (f32, f32),
        text: &str,
        scale: f64,
        bgr: [u8; 3],
    ) -> Result<(), Self::Error>;
}
#[derive(Clone, Debug, Default)]
pub struct TagDetection {
    pub good: bool,
    pub observed_code: u64,
    pub code: u64,
    pub id: i32,
    pub hamming_distance: i32,
    pub rotation: usize,
    pub points: [(f32, f32); 4],
    pub center: (f32, f32),
    pub observed_perimeter: f32,
    pub homography: Matrix3<f64>,
    pub homography_center: (f32, f32),
}
impl TagDetection {
    /// Original diagnostic outline, center and ID drawing calls.
    /// # Errors
    /// Propagates an error from the selected raster backend.
    pub fn draw<D: TagDrawing>(&self, image: &mut D) -> Result<(), D::Error> {
        for (i, color) in [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 0, 255]]
            .into_iter()
            .enumerate()
        {
            image.line(self.points[i], self.points[(i + 1) % 4], color)?;
        }
        image.circle(self.center, 8, 2, [0, 0, 255])?;
        image.hershey_plain(
            (self.center.0 + 10., self.center.1 + 10.),
            &format!("#{}", self.id),
            1.,
            [0, 0, 255],
        )
    }
    /// Original square-tag `solvePnP` pose, mapping tag coordinates into camera.
    /// # Errors
    /// Returns homography initialization or linear solve errors.
    pub fn relative_transform(
        &self,
        tag_size: f64,
        fx: f64,
        fy: f64,
        px: f64,
        py: f64,
    ) -> firefly_error::Result<Matrix4<f64>> {
        planar_pose::solve(&self.points, tag_size, [fx, fy, px, py])
    }
    /// Original translation axis conversion (forward, left, up); rotation retains
    /// the original camera-coordinate convention.
    /// # Errors
    /// Returns pose estimation errors.
    pub fn relative_translation_rotation(
        &self,
        tag_size: f64,
        fx: f64,
        fy: f64,
        px: f64,
        py: f64,
    ) -> firefly_error::Result<(Vector3<f64>, Matrix3<f64>)> {
        let pose = self.relative_transform(tag_size, fx, fy, px, py)?;
        Ok((
            Vector3::new(pose[(2, 3)], -pose[(0, 3)], -pose[(1, 3)]),
            pose.fixed_view::<3, 3>(0, 0).into_owned(),
        ))
    }
    #[must_use]
    pub fn new(id: i32) -> Self {
        Self {
            id,
            ..Default::default()
        }
    }
    #[must_use]
    pub fn interpolate(&self, x: f32, y: f32) -> (f32, f32) {
        let x = f64::from(x);
        let y = f64::from(y);
        let z = (self.homography[(2, 0)] * x
            + self.homography[(2, 1)] * y
            + self.homography[(2, 2)]) as f32;
        if z == 0. {
            return (0., 0.);
        }
        let nx =
            ((self.homography[(0, 0)] * x + self.homography[(0, 1)] * y + self.homography[(0, 2)])
                / f64::from(z)
                + f64::from(self.homography_center.0)) as f32;
        let ny =
            ((self.homography[(1, 0)] * x + self.homography[(1, 1)] * y + self.homography[(1, 2)])
                / f64::from(z)
                + f64::from(self.homography_center.1)) as f32;
        (nx, ny)
    }
    #[must_use]
    pub fn xy_orientation(&self) -> f32 {
        let a = self.interpolate(-1., -1.);
        let b = self.interpolate(1., -1.);
        let angle = (b.1 - a.1).atan2(b.0 - a.0);
        if angle.is_nan() { 0. } else { angle }
    }
    #[must_use]
    pub fn overlaps_too_much(&self, other: &Self) -> bool {
        let mut perimeter = 0.;
        for i in 0..4 {
            perimeter += distance_2d(self.points[i], self.points[(i + 1) % 4]);
        }
        for i in 0..4 {
            perimeter += distance_2d(other.points[i], other.points[(i + 1) % 4]);
        }
        distance_2d(self.center, other.center) < perimeter / 16.
    }
}
