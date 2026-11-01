// Copyright (c) 2018-2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../LICENSE.
use nalgebra::{Isometry3, Vector2};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, fmt};

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct TimeCamId {
    #[serde(rename = "value0")]
    pub frame_id: i64,
    #[serde(rename = "value1")]
    pub cam_id: usize,
}
impl TimeCamId {
    /// Numeric value of the upstream integer-based `std::hash<TimeCamId>`.
    #[must_use]
    pub fn hash_value(self) -> usize {
        let mut seed = 0;
        firefly_basalt_headers::utils::hash::hash_combine(&mut seed, self.frame_id as usize);
        firefly_basalt_headers::utils::hash::hash_combine(&mut seed, self.cam_id);
        seed
    }

    #[must_use]
    pub const fn new(frame_id: i64, cam_id: usize) -> Self {
        Self { frame_id, cam_id }
    }
}

impl fmt::Display for TimeCamId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}_{}", self.frame_id, self.cam_id)
    }
}

#[derive(Clone, Debug, Default)]
pub struct CalibCornerData {
    pub corners: Vec<Vector2<f64>>,
    pub corner_ids: Vec<i32>,
    pub radii: Vec<f64>,
}

pub type CalibCornerMap = HashMap<TimeCamId, CalibCornerData>;

#[derive(Clone, Debug, Default)]
pub struct ProjectedCornerData {
    pub corners_proj: Vec<Vector2<f64>>,
    pub corners_proj_success: Vec<bool>,
}

#[derive(Clone, Debug, Default)]
pub struct CalibInitPoseData {
    pub t_a_c: Isometry3<f64>,
    pub num_inliers: usize,
    pub reprojected_corners: Vec<Vector2<f64>>,
}

pub type CalibInitPoseMap = HashMap<TimeCamId, CalibInitPoseData>;

/// Detection backend must perform tag identification and upstream subpixel acceptance.
pub trait AprilTagDetector: Send {
    /// # Errors
    /// Returns image or detector errors.
    fn detect_tags(
        &mut self,
        image: &firefly_basalt_headers::image::image::ManagedImage<u16>,
    ) -> firefly_error::Result<(CalibCornerData, CalibCornerData)>;
}

/// Narrow image source contract used by calibration corner detection.
pub type CalibrationImage =
    Option<std::sync::Arc<firefly_basalt_headers::image::image::ManagedImage<u16>>>;

pub trait CalibrationImageSource: Sync {
    /// # Errors
    /// Returns errors while reading the source timeline.
    fn image_timestamps(&self) -> firefly_error::Result<Vec<i64>>;

    /// # Errors
    /// Returns errors while reading images at the requested timestamp.
    fn images_at(&self, timestamp: i64) -> firefly_error::Result<Vec<CalibrationImage>>;
}

/// Detects corners from an injected image source without depending on a dataset crate.
/// # Errors
/// Returns image source or detector errors.
pub fn detect_corners<S, F>(
    source: &S,
    tag_count: usize,
    factory: F,
) -> firefly_error::Result<(CalibCornerMap, CalibCornerMap)>
where
    S: CalibrationImageSource,
    F: Fn(usize) -> Box<dyn AprilTagDetector> + Sync,
{
    let timestamps = source.image_timestamps()?;
    let detections: firefly_error::Result<Vec<_>> = timestamps
        .par_iter()
        .map_init(
            || factory(tag_count),
            |detector, &timestamp| {
                source.images_at(timestamp).map(|images| {
                    images
                        .into_iter()
                        .enumerate()
                        .filter_map(|(camera, image)| image.map(|image| (camera, image)))
                        .map(|(camera, image)| {
                            detector
                                .detect_tags(&image)
                                .map(|(good, bad)| (TimeCamId::new(timestamp, camera), good, bad))
                        })
                        .collect::<firefly_error::Result<Vec<_>>>()
                })?
            },
        )
        .collect();
    let mut accepted = CalibCornerMap::new();
    let mut rejected = CalibCornerMap::new();
    for (id, good, bad) in detections?.into_iter().flatten() {
        accepted.insert(id, good);
        rejected.insert(id, bad);
    }
    Ok((accepted, rejected))
}
