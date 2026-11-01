// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are indexed in docs/licenses/README.md.
//! Original `thirdparty/apriltag/include/basalt/utils/apriltag.h` and wrapper cpp.
use crate::thirdparty::ethz_apriltag2::{
    tag_detector::TagDetector, tag_family::TagFamily, tag36h11::CODES,
};
use firefly_calibration::image::image::ManagedImage;
use firefly_calibration::observation::{AprilTagDetector, CalibCornerData};
use firefly_error::Result;
use nalgebra::Vector2;
use purecv::core::types::Point2f;
#[path = "apriltag_subpixel.rs"]
mod subpixel;
pub struct ApriltagDetector {
    tag_count: usize,
    detector: TagDetector,
}
impl ApriltagDetector {
    #[must_use]
    pub fn new(tag_count: usize) -> Self {
        Self {
            tag_count,
            detector: TagDetector::new(TagFamily::new(36, 11, &CODES, 2)),
        }
    }
    /// # Errors
    /// Returns subpixel refinement errors.
    pub fn detect_tags(
        &mut self,
        image: &ManagedImage<u16>,
    ) -> Result<(CalibCornerData, CalibCornerData)> {
        let pixels: Vec<_> = image
            .data()
            .iter()
            .take(image.w * image.h)
            .map(|&v| (v >> 8) as u8)
            .collect();
        let mut detections = self.detector.extract_tags(image.w, image.h, &pixels);
        detections.retain(|d| {
            d.good
                && d.id < self.tag_count as i32
                && d.points.iter().all(|&(x, y)| {
                    x >= 4. && y >= 4. && x <= image.w as f32 - 4. && y <= image.h as f32 - 4.
                })
        });
        let (mut good, mut bad) = (CalibCornerData::default(), CalibCornerData::default());
        if detections.len() < 4 {
            return Ok((good, bad));
        }
        detections.sort_by_key(|d| d.id);
        if detections.windows(2).any(|pair| pair[0].id == pair[1].id) {
            log::warn!("Wild Apriltag detected. Hide them!");
            return Ok((good, bad));
        }
        for detection in detections {
            let radius = 2.0_f64.max(0.075 * f64::from(detection.observed_perimeter) / 4. - 1.);
            let window = (radius + 1.).ceil() as i32;
            let mut corners: Vec<_> = detection
                .points
                .iter()
                .map(|&(x, y)| Point2f::new(x, y))
                .collect();
            subpixel::refine(&pixels, image.w, image.h, &mut corners, window as usize)?;
            for (i, corner) in corners.iter().enumerate() {
                let x = f64::from(corner.x);
                let y = f64::from(corner.y);
                let displacement = (x - f64::from(detection.points[i].0)).powi(2)
                    + (y - f64::from(detection.points[i].1)).powi(2);
                let output = if displacement <= radius * radius {
                    &mut good
                } else {
                    &mut bad
                };
                output.corners.push(Vector2::new(x, y));
                output.corner_ids.push((detection.id << 2) + i as i32);
                output.radii.push(radius);
            }
        }
        Ok((good, bad))
    }
}
impl AprilTagDetector for ApriltagDetector {
    fn detect_tags(
        &mut self,
        image: &ManagedImage<u16>,
    ) -> Result<(CalibCornerData, CalibCornerData)> {
        Self::detect_tags(self, image)
    }
}
