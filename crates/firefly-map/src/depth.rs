//! Local probabilistic mapping from paired depth images and body poses.
//! The caller supplies measurement timestamps for evidence and a monotonic clock
//! for the watchdog, and drives the enabled fading timer every 500 ms.
//! Numerical reference: EGO-Planner-v2, revision 9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010.

mod config;
mod grid;
mod raycast;
mod snapshot;

#[cfg(test)]
mod cpp_reference;
#[cfg(test)]
mod tests;

use crate::{CollisionSnapshot, GridMap};
pub use config::{DepthImage, MappingParameters};
use firefly_error::Result;
use nalgebra::{Isometry3, Vector3};
pub use snapshot::{Bounds, DepthSnapshot};
use std::time::Duration;

/// Work performed for one accepted depth measurement, including an empty image.
#[derive(Clone, Copy, Debug)]
pub struct FrameStats {
    pub measured_at: f64,
    pub projection: Duration,
    pub window: Duration,
    pub raycast: Duration,
    pub inflation: Duration,
    pub projected_rays: usize,
    pub voted_voxels: usize,
    pub cleared_voxels: usize,
}

/// Outcome of submitting one synchronized image to a mapper.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameUpdate {
    /// An equal or newer measurement has already been consumed.
    Stale,
    /// A frame advanced the window and timestamp without producing valid rays.
    Empty,
    /// One or more valid rays contributed online evidence.
    Observed,
}

/// Single writer for the rolling depth layer. Published views share immutable buffers.
pub struct DepthMapper {
    grid: grid::GridMap,
    collision_origin: Vector3<f64>,
    collision_resolution: f64,
    collision_cache: Option<(u64, CollisionSnapshot)>,
    last_frame: Option<FrameStats>,
}

impl DepthMapper {
    /// Configure the sensor and select the output grid's world-anchored lattice.
    /// # Errors
    /// Rejects invalid mapping parameters and camera extrinsics.
    pub fn new(
        parameters: MappingParameters,
        body_from_camera: Isometry3<f64>,
        output: &GridMap,
    ) -> Result<Self> {
        let mut grid = grid::GridMap::new(parameters)?;
        grid.extrinsic_callback(body_from_camera)?;
        Ok(Self {
            grid,
            collision_origin: output.world_origin(),
            collision_resolution: output.resolution(),
            collision_cache: None,
            last_frame: None,
        })
    }

    /// Convert only the pixels required by this mapper's projection settings.
    /// # Errors
    /// Rejects mismatched or empty input dimensions.
    pub fn sample_depth<T: Copy>(
        &self,
        width: usize,
        height: usize,
        source: &[T],
        convert: impl FnMut(T) -> u16,
    ) -> Result<DepthImage> {
        self.grid.sample_depth(width, height, source, convert)
    }

    /// Sample a new image, reusing storage from the last completed integration when available.
    /// # Errors
    /// Rejects mismatched or empty input dimensions.
    pub fn sample_next_depth<T: Copy>(
        &mut self,
        width: usize,
        height: usize,
        source: &[T],
        convert: impl FnMut(T) -> u16,
    ) -> Result<DepthImage> {
        DepthImage::sample(
            width,
            height,
            source,
            self.grid.sampling(),
            self.grid.take_integrated_pixels(),
            convert,
        )
    }

    /// Fuse one depth image using its synchronized body pose in odometry coordinates.
    /// # Errors
    /// Rejects invalid clocks, poses, depth layouts and out-of-range projections.
    pub fn integrate(
        &mut self,
        image: DepthImage,
        odom_from_body: Isometry3<f64>,
        measured_at: f64,
        timer_time: f64,
    ) -> Result<FrameUpdate> {
        if !measured_at.is_finite()
            || measured_at < 0.
            || !timer_time.is_finite()
            || timer_time < 0.
        {
            return Err(config::invalid("invalid depth or timer timestamp"));
        }
        if self
            .last_frame
            .is_some_and(|s| measured_at <= s.measured_at)
        {
            return Ok(FrameUpdate::Stale);
        }
        self.grid.depth_odom_callback(image, odom_from_body)?;
        let stats = self
            .grid
            .update_at(timer_time, measured_at)?
            .ok_or_else(|| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    "paired depth frame was not pending",
                )
            })?;
        self.last_frame = Some(stats);
        Ok(if stats.projected_rays == 0 {
            FrameUpdate::Empty
        } else {
            FrameUpdate::Observed
        })
    }

    /// Timings and voxel counts from the most recent accepted measurement.
    #[must_use]
    pub fn last_frame(&self) -> Option<FrameStats> {
        self.last_frame
    }

    /// Advance the sensor watchdog. Its timeout stays latched for this mapper.
    /// # Errors
    /// Rejects invalid timer values.
    pub fn timed_out_at(&mut self, timer_time: f64) -> Result<bool> {
        self.grid.update_occupancy_callback(timer_time)?;
        Ok(self.grid.odom_depth_timeout())
    }

    /// Retire obstacles at the latest accepted measurement time and apply one decay step.
    pub fn fade(&mut self) {
        self.grid
            .fading_callback(self.last_frame.map(|s| s.measured_at));
    }
}
