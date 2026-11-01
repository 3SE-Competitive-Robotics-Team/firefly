// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original Quad.h/Quad.cc, with homography interpolation.
use super::{
    g_line2d::GLine2D,
    homography33::Homography33,
    math_util::{distance_2d, mod_2pi},
    segment::Segment,
};
pub struct Quad {
    pub points: [(f32, f32); 4],
    pub segments: [usize; 5],
    pub observed_perimeter: f32,
    pub homography: Homography33,
}
impl Quad {
    #[must_use]
    pub fn new(points: [(f32, f32); 4], center: (f32, f32)) -> Option<Self> {
        Some(Self {
            points,
            segments: [0; 5],
            observed_perimeter: 0.,
            homography: Homography33::new(
                &[(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)],
                &points,
                center,
            )?,
        })
    }
    #[must_use]
    pub fn interpolate(&self, x: f32, y: f32) -> (f32, f32) {
        self.homography.project(x, y)
    }
    #[must_use]
    pub fn interpolate01(&self, x: f32, y: f32) -> (f32, f32) {
        self.interpolate(2. * x - 1., 2. * y - 1.)
    }
    pub fn search(
        segments: &[Segment],
        path: &mut [usize; 5],
        parent: usize,
        depth: usize,
        quads: &mut Vec<Self>,
        center: (f32, f32),
    ) {
        if depth == 4 {
            if path[4] != path[0] {
                return;
            }
            let mut points = [(0., 0.); 4];
            let mut perimeter = 0.;
            for i in 0..4 {
                let first = &segments[path[i]];
                let second = &segments[path[i + 1]];
                points[i] = GLine2D::from_points(first.p0, first.p1)
                    .intersection(&GLine2D::from_points(second.p0, second.p1));
                perimeter += first.length;
                if points[i].0 == -1. {
                    return;
                }
            }
            let angles: [f32; 4] = std::array::from_fn(|i| {
                let next = (i + 1) % 4;
                (points[next].1 - points[i].1).atan2(points[next].0 - points[i].0)
            });
            let turning = mod_2pi(angles[1] - angles[0])
                + mod_2pi(angles[2] - angles[1])
                + mod_2pi(angles[3] - angles[2])
                + mod_2pi(angles[0] - angles[3]);
            if !(-7. ..=-5.).contains(&turning) {
                return;
            }
            let lengths: [f32; 4] =
                std::array::from_fn(|i| distance_2d(points[i], points[(i + 1) % 4]));
            if lengths.iter().any(|&x| x < 6.)
                || distance_2d(points[0], points[2]) < 6.
                || distance_2d(points[1], points[3]) < 6.
            {
                return;
            }
            let maximum = lengths.iter().copied().fold(f32::NEG_INFINITY, f32::max);
            let minimum = lengths.iter().copied().fold(f32::INFINITY, f32::min);
            if maximum > minimum * 32. {
                return;
            }
            if let Some(mut quad) = Self::new(points, center) {
                quad.segments = *path;
                quad.observed_perimeter = perimeter;
                quads.push(quad);
            }
            return;
        }
        for &child in &segments[parent].children {
            if segments[child].theta > segments[path[0]].theta {
                continue;
            }
            path[depth + 1] = child;
            Self::search(segments, path, child, depth + 1, quads, center);
        }
    }
}
