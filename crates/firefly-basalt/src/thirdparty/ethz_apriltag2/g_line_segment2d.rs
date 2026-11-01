// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original GLineSegment2D.h/GLineSegment2D.cc.
use super::{g_line2d::GLine2D, xy_weight::XYWeight};
pub struct GLineSegment2D {
    pub line: GLine2D,
    pub p0: (f32, f32),
    pub p1: (f32, f32),
}
impl GLineSegment2D {
    #[must_use]
    pub fn new(p0: (f32, f32), p1: (f32, f32)) -> Self {
        Self {
            line: GLine2D::from_points(p0, p1),
            p0,
            p1,
        }
    }
    #[must_use]
    pub fn lsq_fit(points: &[XYWeight]) -> Self {
        let mut line = GLine2D::lsq_fit(points);
        let (mut minimum, mut maximum) = (f32::INFINITY, f32::NEG_INFINITY);
        for point in points {
            let coordinate = line.line_coordinate((point.x, point.y));
            minimum = minimum.min(coordinate);
            maximum = maximum.max(coordinate);
        }
        Self::new(
            line.point_of_coordinate(minimum),
            line.point_of_coordinate(maximum),
        )
    }
}
