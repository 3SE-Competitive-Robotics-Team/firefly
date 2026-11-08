// SPDX-License-Identifier: GPL-3.0-only
use firefly_error::{Error, ErrorKind, Result};

/// `grid_map.cpp::initMap` ROS defaults, before launch-file overrides.
/// Required geometry fields must be supplied before constructing the map.
#[derive(Clone, Debug)]
pub struct MappingParameters {
    pub resolution: f64,
    pub local_update_range: [f64; 3],
    pub obstacles_inflation: f64,
    pub enable_virtual_wall: bool,
    pub virtual_ceil: f64,
    pub virtual_ground: f64,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub odom_depth_timeout: f64,
    pub use_depth_filter: bool,
    pub depth_filter_mindist: f64,
    /// Loaded by C++; its temporal consistency branch is disabled.
    pub depth_filter_tolerance: f64,
    pub depth_filter_margin: i32,
    pub k_depth_scaling_factor: f64,
    pub skip_pixel: i32,
    pub p_hit: f64,
    pub p_miss: f64,
    pub p_min: f64,
    pub p_max: f64,
    pub p_occ: f64,
    pub fading_time: f64,
    /// Loaded by C++; the active raycaster does not use this value.
    pub min_ray_length: f64,
}

impl Default for MappingParameters {
    fn default() -> Self {
        Self {
            resolution: -1.,
            local_update_range: [-1.; 3],
            obstacles_inflation: -1.,
            enable_virtual_wall: false,
            virtual_ceil: -1.,
            virtual_ground: -1.,
            fx: -1.,
            fy: -1.,
            cx: -1.,
            cy: -1.,
            odom_depth_timeout: 1.,
            use_depth_filter: true,
            depth_filter_mindist: 0.1,
            depth_filter_tolerance: -1.,
            depth_filter_margin: -1,
            k_depth_scaling_factor: -1.,
            skip_pixel: -1,
            p_hit: 0.70,
            p_miss: 0.35,
            p_min: 0.12,
            p_max: 0.97,
            p_occ: 0.80,
            fading_time: 1000.,
            min_ray_length: 0.1,
        }
    }
}

impl MappingParameters {
    pub(crate) fn validate(&self) -> Result<()> {
        let positive = [
            self.resolution,
            self.fx,
            self.fy,
            self.k_depth_scaling_factor,
            self.local_update_range[0],
            self.local_update_range[1],
            self.local_update_range[2],
        ];
        let finite = [
            self.cx,
            self.cy,
            self.obstacles_inflation,
            self.virtual_ceil,
            self.virtual_ground,
            self.odom_depth_timeout,
            self.depth_filter_mindist,
            self.depth_filter_tolerance,
            self.fading_time,
            self.min_ray_length,
        ];
        if positive.iter().any(|x| !x.is_finite() || *x <= 0.)
            || finite.iter().any(|x| !x.is_finite())
            || self.obstacles_inflation < 0.
            || self.skip_pixel <= 0
            || self.depth_filter_margin < 0
            || self.odom_depth_timeout < 0.
            || [self.p_hit, self.p_miss, self.p_min, self.p_max, self.p_occ]
                .iter()
                .any(|x| !x.is_finite() || *x <= 0. || *x >= 1.)
            || self.p_min > self.p_occ
            || self.p_occ > self.p_max
        {
            return Err(invalid("invalid mapping parameters"));
        }
        Ok(())
    }
}

/// Contiguous `CV_16UC1` equivalent. Units are divided by `k_depth_scaling_factor`.
#[derive(Clone, Debug)]
pub struct DepthImage {
    pub(crate) width: usize,
    pub(crate) height: usize,
    pub(crate) pixels: Vec<u16>,
}
impl DepthImage {
    /// # Errors
    /// Rejects empty images, overflowing dimensions and mismatched storage.
    pub fn new(width: usize, height: usize, pixels: Vec<u16>) -> Result<Self> {
        if width == 0 || height == 0 || width.checked_mul(height) != Some(pixels.len()) {
            return Err(invalid("invalid depth dimensions"));
        }
        Ok(Self {
            width,
            height,
            pixels,
        })
    }
}

pub(crate) fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("ego_map")
}

pub(crate) fn logit(p: f64) -> f64 {
    (p / (1. - p)).ln()
}
