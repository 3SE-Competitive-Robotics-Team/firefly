// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original TagDetector.h/TagDetector.cc nine-stage detector.
use super::{
    edge::Edge,
    float_image::FloatImage,
    g_line_segment2d::GLineSegment2D,
    g_line2d::GLine2D,
    gaussian::make_gaussian_filter,
    gray_model::GrayModel,
    gridder::Gridder,
    math_util::{distance_2d, mod_2pi},
    quad::Quad,
    segment::Segment,
    tag_detection::TagDetection,
    tag_family::TagFamily,
    union_find_simple::UnionFindSimple,
    xy_weight::XYWeight,
};
use nalgebra::Matrix3;
use std::collections::BTreeMap;
pub struct TagDetector {
    pub family: TagFamily,
}
impl TagDetector {
    #[must_use]
    pub fn new(family: TagFamily) -> Self {
        Self { family }
    }
    #[must_use]
    /// # Panics
    /// Panics if the image dimensions do not match the pixel slice.
    #[expect(
        clippy::too_many_lines,
        clippy::manual_midpoint,
        reason = "Preserve the original nine-stage detector and f32 operation order"
    )]
    pub fn extract_tags(&self, width: usize, height: usize, pixels: &[u8]) -> Vec<TagDetection> {
        assert_eq!(pixels.len(), width * height);
        if width < 3 || height < 3 {
            return Vec::new();
        }
        let original = FloatImage {
            width,
            height,
            pixels: pixels
                .iter()
                .map(|&v| (f64::from(v) / 255.) as f32)
                .collect(),
        };
        let mut segmented = original.clone();
        let filter = make_gaussian_filter(0.8, 3);
        segmented.filter_factored_centered(&filter, &filter);
        let (mut theta, mut magnitude) = (
            FloatImage::new(width, height),
            FloatImage::new(width, height),
        );
        for y in 1..height - 1 {
            for x in 1..width - 1 {
                let ix = segmented.get(x + 1, y) - segmented.get(x - 1, y);
                let iy = segmented.get(x, y + 1) - segmented.get(x, y - 1);
                theta.set(x, y, iy.atan2(ix));
                magnitude.set(x, y, ix * ix + iy * iy);
            }
        }
        let mut forest = UnionFindSimple::new(width * height);
        let mut edges = Vec::with_capacity(width * height * 4);
        let (mut tmin, mut tmax, mut mmin, mut mmax) = (
            vec![0.; width * height],
            vec![0.; width * height],
            vec![0.; width * height],
            vec![0.; width * height],
        );
        for y in 0..height - 1 {
            for x in 0..width - 1 {
                let mag = magnitude.get(x, y);
                if mag < Edge::MIN_MAG {
                    continue;
                }
                let index = y * width + x;
                mmin[index] = mag;
                mmax[index] = mag;
                let angle = theta.get(x, y);
                tmin[index] = angle;
                tmax[index] = angle;
                Edge::calculate(angle, x, y, &theta, &magnitude, &mut edges);
            }
        }
        edges.sort_by_key(|edge| edge.cost);
        Edge::merge(
            &edges,
            &mut forest,
            &mut tmin,
            &mut tmax,
            &mut mmin,
            &mut mmax,
        );
        let mut clusters: BTreeMap<usize, Vec<XYWeight>> = BTreeMap::new();
        for y in 0..height - 1 {
            for x in 0..width - 1 {
                let root = forest.representative(y * width + x);
                if forest.sizes[root] < Segment::MINIMUM_SEGMENT_SIZE {
                    continue;
                }
                clusters.entry(root).or_default().push(XYWeight {
                    x: x as f32,
                    y: y as f32,
                    weight: magnitude.get(x, y),
                });
            }
        }
        let mut segments = Vec::new();
        for points in clusters.values() {
            let fitted = GLineSegment2D::lsq_fit(points);
            let length = distance_2d(fitted.p0, fitted.p1);
            if length < Segment::MINIMUM_LINE_LENGTH {
                continue;
            }
            let dx = fitted.p1.0 - fitted.p0.0;
            let dy = fitted.p1.1 - fitted.p0.1;
            let mut segment = Segment {
                theta: dy.atan2(dx),
                length,
                ..Default::default()
            };
            let (mut flip, mut noflip) = (0., 0.);
            for point in points {
                let angle = theta.get(point.x as usize, point.y as usize);
                let mag = magnitude.get(point.x as usize, point.y as usize);
                if mod_2pi(angle - segment.theta) < 0. {
                    noflip += mag;
                } else {
                    flip += mag;
                }
            }
            if flip > noflip {
                segment.theta += std::f32::consts::PI;
            }
            let dot = dx * segment.theta.cos() + dy * segment.theta.sin();
            if dot > 0. {
                segment.p0 = fitted.p1;
                segment.p1 = fitted.p0;
            } else {
                segment.p0 = fitted.p0;
                segment.p1 = fitted.p1;
            }
            segments.push(segment);
        }
        let mut grid = Gridder::new(0., 0., width as f32, height as f32, 10.);
        for (i, segment) in segments.iter().enumerate() {
            grid.add(segment.p0.0, segment.p0.1, i);
        }
        for i in 0..segments.len() {
            let parent = &segments[i];
            let line = GLine2D::from_points(parent.p0, parent.p1);
            let mut children = Vec::new();
            for child_id in grid.find(parent.p1.0, parent.p1.1, 0.5 * parent.length) {
                let child = &segments[child_id];
                if mod_2pi(child.theta - parent.theta) > 0. {
                    continue;
                }
                let point = line.intersection(&GLine2D::from_points(child.p0, child.p1));
                if point.0 == -1. {
                    continue;
                }
                if distance_2d(point, parent.p1).max(distance_2d(point, child.p0)) > parent.length {
                    continue;
                }
                children.push(child_id);
            }
            segments[i].children = children;
        }
        let center = ((width / 2) as f32, (height / 2) as f32);
        let mut quads = Vec::new();
        let mut path = [0; 5];
        for i in 0..segments.len() {
            path[0] = i;
            Quad::search(&segments, &mut path, i, 0, &mut quads, center);
        }
        let mut detections = Vec::new();
        for quad in &quads {
            let (mut black, mut white) = (GrayModel::default(), GrayModel::default());
            let dimension = (2 * self.family.black_border + self.family.dimension) as i32;
            for iy in -1..=dimension {
                let y = (iy as f32 + 0.5) / dimension as f32;
                for ix in -1..=dimension {
                    let x = (ix as f32 + 0.5) / dimension as f32;
                    let point = quad.interpolate01(x, y);
                    let px = (f64::from(point.0) + 0.5) as i32;
                    let py = (f64::from(point.1) + 0.5) as i32;
                    if px < 0 || py < 0 || px >= width as i32 || py >= height as i32 {
                        continue;
                    }
                    let value = original.get(px as usize, py as usize);
                    if iy == -1 || iy == dimension || ix == -1 || ix == dimension {
                        white.add_observation(x, y, value);
                    } else if iy == 0 || iy == dimension - 1 || ix == 0 || ix == dimension - 1 {
                        black.add_observation(x, y, value);
                    }
                }
            }
            let mut bad = false;
            let mut code = 0_u64;
            for iy in (0..self.family.dimension).rev() {
                let y = (self.family.black_border as f32 + iy as f32 + 0.5) / dimension as f32;
                for ix in 0..self.family.dimension {
                    let x = (self.family.black_border as f32 + ix as f32 + 0.5) / dimension as f32;
                    let point = quad.interpolate01(x, y);
                    let px = (f64::from(point.0) + 0.5) as i32;
                    let py = (f64::from(point.1) + 0.5) as i32;
                    if px < 0 || py < 0 || px >= width as i32 || py >= height as i32 {
                        bad = true;
                        continue;
                    }
                    let threshold = (black.interpolate(x, y) + white.interpolate(x, y)) * 0.5;
                    code <<= 1;
                    if original.get(px as usize, py as usize) > threshold {
                        code |= 1;
                    }
                }
            }
            if bad {
                continue;
            }
            let mut detection = self.family.decode(code);
            detection.homography = quad.homography.matrix;
            detection.homography_center = quad.homography.center;
            let angle = detection.rotation as f32 * std::f32::consts::PI / 2.;
            let c = f64::from(angle.cos());
            let s = f64::from(angle.sin());
            detection.homography *= Matrix3::new(c, -s, 0., s, c, 0., 0., 0., 1.);
            let lower_left = detection.interpolate(-1., -1.);
            let mut best_rotation = 0;
            let mut distance = f32::MAX;
            for (i, &point) in quad.points.iter().enumerate() {
                let d = distance_2d(lower_left, point);
                if d < distance {
                    distance = d;
                    best_rotation = i;
                }
            }
            for i in 0..4 {
                detection.points[i] = quad.points[(i + best_rotation) % 4];
            }
            if detection.good {
                detection.center = quad.interpolate01(0.5, 0.5);
                detection.observed_perimeter = quad.observed_perimeter;
                detections.push(detection);
            }
        }
        let mut good: Vec<TagDetection> = Vec::new();
        for detection in detections {
            let mut new_feature = true;
            for other in &mut good {
                if detection.id != other.id || !detection.overlaps_too_much(other) {
                    continue;
                }
                new_feature = false;
                if detection.hamming_distance > other.hamming_distance {
                    continue;
                }
                if detection.hamming_distance < other.hamming_distance
                    || detection.observed_perimeter > other.observed_perimeter
                {
                    *other = detection.clone();
                }
            }
            if new_feature {
                good.push(detection);
            }
        }
        good
    }
}
