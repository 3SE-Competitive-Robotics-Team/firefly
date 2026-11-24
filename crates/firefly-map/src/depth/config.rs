use firefly_error::{Error, ErrorKind, Result};

/// `grid_map.cpp::initMap` ROS defaults, before launch-file overrides.
/// Required geometry fields must be supplied before constructing the map.
#[derive(Clone, Debug)]
pub struct MappingParameters {
    pub resolution: f64,
    pub local_update_range: [f64; 3],
    pub obstacles_inflation: f64,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub odom_depth_timeout: f64,
    pub use_depth_filter: bool,
    pub depth_filter_mindist: f64,
    pub depth_filter_margin: i32,
    pub k_depth_scaling_factor: f64,
    pub skip_pixel: i32,
    pub p_hit: f64,
    pub p_miss: f64,
    pub p_min: f64,
    pub p_max: f64,
    pub p_occ: f64,
    pub fading_time: f64,
    /// Maximum measurement age of an online obstacle, in seconds.
    pub occupied_max_age: Option<f64>,
}

impl Default for MappingParameters {
    fn default() -> Self {
        Self {
            resolution: -1.,
            local_update_range: [-1.; 3],
            obstacles_inflation: -1.,
            fx: -1.,
            fy: -1.,
            cx: -1.,
            cy: -1.,
            odom_depth_timeout: 1.,
            use_depth_filter: true,
            depth_filter_mindist: 0.1,
            depth_filter_margin: -1,
            k_depth_scaling_factor: -1.,
            skip_pixel: -1,
            p_hit: 0.70,
            p_miss: 0.35,
            p_min: 0.12,
            p_max: 0.97,
            p_occ: 0.80,
            fading_time: 1000.,
            occupied_max_age: None,
        }
    }
}

impl MappingParameters {
    pub(super) fn validate(&self) -> Result<()> {
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
            self.odom_depth_timeout,
            self.depth_filter_mindist,
            self.fading_time,
        ];
        if positive.iter().any(|x| !x.is_finite() || *x <= 0.)
            || finite.iter().any(|x| !x.is_finite())
            || self
                .occupied_max_age
                .is_some_and(|age| !age.is_finite() || age <= 0.)
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct DepthSampling {
    margin: usize,
    step: usize,
}

impl DepthSampling {
    pub(super) fn for_map(p: &MappingParameters) -> Self {
        Self {
            margin: if p.use_depth_filter {
                p.depth_filter_margin as usize
            } else {
                0
            },
            step: p.skip_pixel as usize,
        }
    }

    fn axis(self, size: usize) -> impl ExactSizeIterator<Item = usize> + Clone {
        (self.margin..size.saturating_sub(self.margin)).step_by(self.step)
    }

    pub(super) fn coordinates(
        self,
        width: usize,
        height: usize,
    ) -> impl Iterator<Item = (usize, usize)> {
        let columns = self.axis(width);
        self.axis(height)
            .flat_map(move |v| columns.clone().map(move |u| (u, v)))
    }
}

/// Depth units in source image coordinates, stored densely or on the map's sampling grid.
#[derive(Clone, Debug)]
pub struct DepthImage {
    pub(super) width: usize,
    pub(super) height: usize,
    pub(super) pixels: Vec<u16>,
    pub(super) sampling: Option<DepthSampling>,
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
            sampling: None,
        })
    }

    pub(super) fn sample<T: Copy>(
        width: usize,
        height: usize,
        source: &[T],
        sampling: DepthSampling,
        mut convert: impl FnMut(T) -> u16,
    ) -> Result<Self> {
        if width == 0 || height == 0 || width.checked_mul(height) != Some(source.len()) {
            return Err(invalid("invalid depth dimensions"));
        }
        let columns = sampling.axis(width).len();
        let rows = sampling.axis(height);
        let mut pixels = vec![0; columns * rows.len()];
        if columns > 0 {
            for (v, output) in rows.zip(pixels.chunks_exact_mut(columns)) {
                let start = v * width + sampling.margin;
                let row = &source[start..(v + 1) * width - sampling.margin];
                if sampling.step == 2 {
                    // Fixed-width pairs let the release build vectorize the default sampling grid.
                    let (pairs, tail) = row.as_chunks::<2>();
                    for (dst, pair) in output.iter_mut().zip(pairs) {
                        *dst = convert(pair[0]);
                    }
                    if let [last] = tail {
                        output[columns - 1] = convert(*last);
                    }
                } else {
                    for (dst, src) in output.iter_mut().zip(row.iter().step_by(sampling.step)) {
                        *dst = convert(*src);
                    }
                }
            }
        }
        Ok(Self {
            width,
            height,
            pixels,
            sampling: Some(sampling),
        })
    }

    pub(super) fn validate_sampling(&self, sampling: DepthSampling) -> Result<()> {
        if self.sampling.is_some_and(|layout| layout != sampling) {
            return Err(invalid("depth sampling grid differs from map"));
        }
        Ok(())
    }
}

pub(super) fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("ego_map")
}

pub(super) fn logit(p: f64) -> f64 {
    (p / (1. - p)).ln()
}
