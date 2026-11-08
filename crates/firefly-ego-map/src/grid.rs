// SPDX-License-Identifier: GPL-3.0-only
//! Numerical state and callbacks corresponding to upstream `grid_map.cpp`.
use crate::{
    DepthImage, MappingParameters,
    config::{invalid, logit},
    raycast::RayCaster,
};
use firefly_error::Result;
use nalgebra::{Isometry3, Matrix3, Vector3};

mod window;

const OBSTACLE_FLAG: u16 = 32767;
type Index = Vector3<i32>;

/// Local probabilistic map. Call depth reception and timer callbacks sequentially.
/// The raw buffer order matches C++: x is the fastest-varying coordinate.
#[expect(
    clippy::struct_excessive_bools,
    reason = "Preserve independent C++ callback flags"
)]
pub struct GridMap {
    parameters: MappingParameters,
    resolution_inv: f64,
    half_range: Index,
    size: Index,
    inf_size: Index,
    inf_grid: i32,
    origin: Index,
    inf_origin: Index,
    low: Index,
    high: Index,
    low_position: Vector3<f64>,
    high_position: Vector3<f64>,
    inf_low: Index,
    inf_high: Index,
    inf_low_position: Vector3<f64>,
    inf_high_position: Vector3<f64>,
    center_last: Index,
    initialized: bool,
    prob_hit_log: f64,
    prob_miss_log: f64,
    clamp_min_log: f64,
    clamp_max_log: f64,
    min_occupancy_log: f64,
    occupancy: Vec<f64>,
    inflate: Vec<u16>,
    count_hit: Vec<i16>,
    count_hit_and_miss: Vec<i16>,
    flag_traverse: Vec<i8>,
    flag_rayend: Vec<i8>,
    raycast_num: i8,
    cache_voxel: Vec<Index>,
    projected: Vec<Vector3<f64>>,
    camera_position: Vector3<f64>,
    camera_rotation: Matrix3<f64>,
    camera_to_body_rotation: Matrix3<f64>,
    camera_to_body_translation: Vector3<f64>,
    depth: Option<DepthImage>,
    has_first_depth: bool,
    has_odom: bool,
    ever_received_depth: bool,
    occupancy_needs_update: bool,
    last_occupancy_update_time: f64,
    depth_odom_timeout: bool,
}

impl GridMap {
    /// `initMap`, including inflation-driven resolution adjustment.
    ///
    /// # Errors
    /// Rejects invalid parameters or dimensions outside the C++ signed-index range.
    pub fn new(mut parameters: MappingParameters) -> Result<Self> {
        parameters.validate()?;
        let mut inf_grid = ((parameters.obstacles_inflation - 1e-5) / parameters.resolution).ceil();
        if inf_grid > 4. {
            inf_grid = 4.;
            parameters.resolution = parameters.obstacles_inflation / inf_grid;
        }
        if !parameters.resolution.is_finite() || parameters.resolution <= 0. || inf_grid < 0. {
            return Err(invalid("invalid effective resolution or inflation"));
        }
        let resolution_inv = 1. / parameters.resolution;
        let ranges =
            Vector3::from(parameters.local_update_range).map(|v| (v * resolution_inv).ceil());
        if ranges
            .iter()
            .any(|v| !v.is_finite() || *v < 1. || *v > f64::from(i32::MAX / 8))
        {
            return Err(invalid("local map dimensions exceed signed-index range"));
        }
        let half_range = ranges.map(|v| v as i32);
        let size = half_range * 2;
        let inf_grid = inf_grid as i32;
        let inf_size = size + Index::repeat(2 * inf_grid);
        let capacity = Self::capacity(size)?;
        let inf_capacity = Self::capacity(inf_size)?;
        let clamp_min_log = logit(parameters.p_min);
        // `initMap` camera-to-body rotation, before extrinsicCallback overrides it.
        let rotation = Matrix3::new(0., 0., 1., -1., 0., 0., 0., -1., 0.);
        Ok(Self {
            prob_hit_log: logit(parameters.p_hit),
            prob_miss_log: logit(parameters.p_miss),
            clamp_min_log,
            clamp_max_log: logit(parameters.p_max),
            min_occupancy_log: logit(parameters.p_occ),
            parameters,
            resolution_inv,
            half_range,
            size,
            inf_size,
            inf_grid,
            origin: Index::zeros(),
            inf_origin: Index::zeros(),
            low: Index::zeros(),
            high: Index::zeros(),
            low_position: Vector3::zeros(),
            high_position: Vector3::zeros(),
            inf_low: Index::zeros(),
            inf_high: Index::zeros(),
            inf_low_position: Vector3::zeros(),
            inf_high_position: Vector3::zeros(),
            center_last: Index::zeros(),
            initialized: false,
            occupancy: vec![clamp_min_log; capacity],
            inflate: vec![0; inf_capacity],
            count_hit: vec![0; capacity],
            count_hit_and_miss: vec![0; capacity],
            flag_traverse: vec![-1; capacity],
            flag_rayend: vec![-1; capacity],
            raycast_num: 0,
            cache_voxel: Vec::new(),
            projected: Vec::new(),
            camera_position: Vector3::zeros(),
            camera_rotation: Matrix3::identity(),
            camera_to_body_rotation: rotation,
            camera_to_body_translation: Vector3::zeros(),
            depth: None,
            has_first_depth: false,
            has_odom: false,
            ever_received_depth: false,
            occupancy_needs_update: false,
            last_occupancy_update_time: 0.,
            depth_odom_timeout: false,
        })
    }

    fn capacity(size: Index) -> Result<usize> {
        size.iter()
            .try_fold(1_usize, |n, &s| n.checked_mul(s as usize))
            .filter(|n| i32::try_from(*n).is_ok())
            .ok_or_else(|| invalid("map buffer exceeds signed-index range"))
    }

    /// `depthPoseCallback`: pose is camera-to-world, and already paired with depth.
    /// # Errors
    /// Rejects non-finite poses or coordinates outside the supported index range.
    pub fn depth_pose_callback(&mut self, depth: DepthImage, pose: Isometry3<f64>) -> Result<()> {
        self.validate_pose(&pose)?;
        self.camera_position = pose.translation.vector;
        self.camera_rotation = pose.rotation.to_rotation_matrix().into_inner();
        self.depth = Some(depth);
        self.occupancy_needs_update = true;
        self.ever_received_depth = true;
        Ok(())
    }

    /// `depthOdomCallback`: compose body-to-world with camera-to-body extrinsics.
    /// # Errors
    /// Rejects invalid poses, as in `depth_pose_callback`.
    pub fn depth_odom_callback(
        &mut self,
        depth: DepthImage,
        body_pose: Isometry3<f64>,
    ) -> Result<()> {
        let (position, rotation) = self.camera_pose_from_body(&body_pose)?;
        self.camera_position = position;
        self.camera_rotation = rotation;
        self.depth = Some(depth);
        self.occupancy_needs_update = true;
        self.ever_received_depth = true;
        Ok(())
    }

    /// `extrinsicCallback`: camera-to-body transform.
    /// # Errors
    /// Rejects non-finite transforms or unsupported coordinate magnitudes.
    pub fn extrinsic_callback(&mut self, camera_to_body: Isometry3<f64>) -> Result<()> {
        self.validate_pose(&camera_to_body)?;
        self.camera_to_body_rotation = camera_to_body.rotation.to_rotation_matrix().into_inner();
        self.camera_to_body_translation = camera_to_body.translation.vector;
        Ok(())
    }

    /// Independent odometry is ignored after the first synchronized depth pair.
    /// # Errors
    /// Rejects invalid poses before depth reception.
    pub fn odom_callback(&mut self, body_pose: Isometry3<f64>) -> Result<()> {
        if self.ever_received_depth {
            return Ok(());
        }
        let (position, rotation) = self.camera_pose_from_body(&body_pose)?;
        self.camera_position = position;
        self.camera_rotation = rotation;
        self.has_odom = true;
        Ok(())
    }

    fn validate_pose(&self, pose: &Isometry3<f64>) -> Result<()> {
        if !pose.to_homogeneous().iter().all(|x| x.is_finite())
            || !self.indexable(pose.translation.vector)
        {
            return Err(invalid("invalid camera pose"));
        }
        Ok(())
    }

    fn camera_pose_from_body(
        &self,
        body_pose: &Isometry3<f64>,
    ) -> Result<(Vector3<f64>, Matrix3<f64>)> {
        self.validate_pose(body_pose)?;
        // C++ composes rotation matrices after converting the body quaternion.
        let body_rotation = body_pose.rotation.to_rotation_matrix().into_inner();
        let position =
            body_rotation * self.camera_to_body_translation + body_pose.translation.vector;
        if !self.indexable(position) {
            return Err(invalid("camera position exceeds signed-index range"));
        }
        Ok((position, body_rotation * self.camera_to_body_rotation))
    }

    fn indexable(&self, point: Vector3<f64>) -> bool {
        point
            .iter()
            .all(|v| v.is_finite() && (v * self.resolution_inv).abs() < f64::from(i32::MAX / 4))
    }

    /// `updateOccupancyCallback`; `now` is timer time in seconds, not a depth stamp.
    /// Returns whether an update was consumed, including the filter's empty first frame.
    /// # Errors
    /// Rejects invalid clock values or projected coordinates outside the signed-index range.
    #[fastrace::trace]
    pub fn update_occupancy_callback(&mut self, now: f64) -> Result<bool> {
        if !now.is_finite() || now < 0. {
            return Err(invalid("invalid map timer time"));
        }
        if self.last_occupancy_update_time < 1. {
            self.last_occupancy_update_time = now;
        }
        if !self.occupancy_needs_update {
            if self.ever_received_depth
                && now - self.last_occupancy_update_time > self.parameters.odom_depth_timeout
            {
                self.depth_odom_timeout = true;
            }
            return Ok(false);
        }
        self.last_occupancy_update_time = now;
        self.move_ring_buffer();
        self.project_depth_image()?;
        if !self.projected.is_empty() {
            self.raycast_process();
            self.clear_and_inflate_local_map();
        }
        self.occupancy_needs_update = false;
        Ok(true)
    }

    #[fastrace::trace]
    fn project_depth_image(&mut self) -> Result<()> {
        self.projected.clear();
        let Some(image) = &self.depth else {
            return Ok(());
        };
        let p = &self.parameters;
        if p.use_depth_filter && !self.has_first_depth {
            self.has_first_depth = true;
            return Ok(());
        }
        let margin = if p.use_depth_filter {
            p.depth_filter_margin as usize
        } else {
            0
        };
        let inv_factor = 1. / p.k_depth_scaling_factor;
        for v in (margin..image.height.saturating_sub(margin)).step_by(p.skip_pixel as usize) {
            for u in (margin..image.width.saturating_sub(margin)).step_by(p.skip_pixel as usize) {
                let raw = f64::from(image.pixels[v * image.width + u]);
                let depth = if p.use_depth_filter {
                    raw * inv_factor
                } else {
                    raw / p.k_depth_scaling_factor
                };
                let minimum = if p.use_depth_filter {
                    p.depth_filter_mindist
                } else {
                    0.1
                };
                if depth < minimum {
                    continue;
                }
                let local = Vector3::new(
                    (u as f64 - p.cx) * depth / p.fx,
                    (v as f64 - p.cy) * depth / p.fy,
                    depth,
                );
                let point = self.camera_rotation * local + self.camera_position;
                if !self.indexable(point) {
                    return Err(invalid("projected point exceeds signed-index range"));
                }
                self.projected.push(point);
            }
        }
        Ok(())
    }

    fn set_cache_occupancy(&mut self, point: Vector3<f64>, occupied: bool) -> usize {
        let id = self.position_to_index(point);
        let address = self.address(id);
        self.count_hit_and_miss[address] = self.count_hit_and_miss[address].wrapping_add(1);
        if self.count_hit_and_miss[address] == 1 {
            self.cache_voxel.push(id);
        }
        if occupied {
            self.count_hit[address] = self.count_hit[address].wrapping_add(1);
        }
        address
    }

    #[fastrace::trace]
    fn raycast_process(&mut self) {
        self.cache_voxel.clear();
        self.raycast_num = self.raycast_num.wrapping_add(1);
        for i in 0..self.projected.len() {
            let mut point = self.projected[i];
            let occupied = self.is_in_buffer(point);
            if !occupied {
                point = self.closest_point_in_map(point);
            }
            let address = self.set_cache_occupancy(point, occupied);
            if self.flag_rayend[address] == self.raycast_num {
                continue;
            }
            self.flag_rayend[address] = self.raycast_num;
            let ray = RayCaster::new(
                point / self.parameters.resolution,
                self.camera_position / self.parameters.resolution,
            );
            for cell in ray {
                let center =
                    (cell.cast::<f64>() + Vector3::repeat(0.5)) * self.parameters.resolution;
                let address = self.set_cache_occupancy(center, false);
                if self.flag_traverse[address] == self.raycast_num {
                    break;
                }
                self.flag_traverse[address] = self.raycast_num;
            }
        }
        for i in 0..self.cache_voxel.len() {
            let address = self.address(self.cache_voxel[i]);
            let hit = i32::from(self.count_hit[address]);
            let all = i32::from(self.count_hit_and_miss[address]);
            let update = if hit >= all - hit {
                self.prob_hit_log
            } else {
                self.prob_miss_log
            };
            self.count_hit[address] = 0;
            self.count_hit_and_miss[address] = 0;
            if (update >= 0. && self.occupancy[address] >= self.clamp_max_log)
                || (update <= 0. && self.occupancy[address] <= self.clamp_min_log)
            {
                continue;
            }
            self.occupancy[address] = (self.occupancy[address] + update)
                .max(self.clamp_min_log)
                .min(self.clamp_max_log);
        }
    }

    #[fastrace::trace]
    fn clear_and_inflate_local_map(&mut self) {
        for i in 0..self.cache_voxel.len() {
            let id = self.cache_voxel[i];
            let address = self.address(id);
            let inf_address = self.inf_address(id);
            if self.inflate[inf_address] < OBSTACLE_FLAG
                && self.occupancy[address] >= self.min_occupancy_log
            {
                self.change_inf_buf(true, id);
            }
            if self.inflate[inf_address] >= OBSTACLE_FLAG
                && self.occupancy[address] < self.min_occupancy_log
            {
                self.change_inf_buf(false, id);
            }
        }
    }

    /// Upstream point-cloud path: world coordinates, insertion only, no free-space rays.
    #[fastrace::trace]
    pub fn cloud_callback(&mut self, points: &[Vector3<f64>]) {
        if !self.has_odom || points.is_empty() {
            return;
        }
        self.move_ring_buffer();
        for &point in points {
            if point.iter().any(|v| v.is_nan()) || !self.is_in_buffer(point) {
                continue;
            }
            let id = self.position_to_index(point);
            let address = self.address(id);
            self.occupancy[address] = self.clamp_max_log;
            if self.inflate[self.inf_address(id)] < OBSTACLE_FLAG
                && self.occupancy[address] >= self.min_occupancy_log
            {
                self.change_inf_buf(true, id);
            }
        }
    }

    /// Call at 2 Hz when `fading_time > 0`; one invocation is one upstream decay step.
    #[fastrace::trace]
    pub fn fading_callback(&mut self) {
        if self.parameters.fading_time <= 0. {
            return;
        }
        let reduce =
            (self.clamp_max_log - self.min_occupancy_log) / (self.parameters.fading_time * 2.);
        let low_threshold = self.clamp_min_log + reduce;
        for address in 0..self.occupancy.len() {
            if self.occupancy[address] > low_threshold {
                let was_occupied = self.occupancy[address] >= self.min_occupancy_log;
                self.occupancy[address] -= reduce;
                if was_occupied && self.occupancy[address] < self.min_occupancy_log {
                    self.change_inf_buf(false, self.address_to_index(address));
                }
            }
        }
    }

    fn change_inf_buf(&mut self, add: bool, id: Index) {
        let center = self.inf_address(id);
        self.inflate[center] = if add {
            self.inflate[center].wrapping_add(OBSTACLE_FLAG)
        } else {
            self.inflate[center].wrapping_sub(OBSTACLE_FLAG)
        };
        for x in -self.inf_grid..=self.inf_grid {
            for y in -self.inf_grid..=self.inf_grid {
                for z in -self.inf_grid..=self.inf_grid {
                    let address = self.inf_address(id + Index::new(x, y, z));
                    if add {
                        self.inflate[address] = self.inflate[address].wrapping_add(1);
                    } else {
                        self.inflate[address] = self.inflate[address].wrapping_sub(1);
                        // Upstream's unsigned underflow recovery, with the same reset scope.
                        if self.inflate[address] > 65000 {
                            self.occupancy.fill(self.clamp_min_log);
                            self.inflate.fill(0);
                        }
                    }
                }
            }
        }
    }

    /// C++ `getOccupancy`: 0 outside the window, -1 for an in-window virtual wall.
    #[must_use]
    pub fn occupancy_at(&self, position: Vector3<f64>) -> i32 {
        if !self.initialized || !self.is_in_buffer(position) {
            return 0;
        }
        if self.virtual_wall(position) {
            return -1;
        }
        i32::from(
            self.occupancy[self.address(self.position_to_index(position))] > self.min_occupancy_log,
        )
    }

    /// C++ `getInflateOccupancy`: raw neighbor count including the obstacle flag.
    #[must_use]
    pub fn inflated_occupancy_at(&self, position: Vector3<f64>) -> i32 {
        if !self.initialized
            || !Self::contains(position, self.inf_low_position, self.inf_high_position)
        {
            return 0;
        }
        if self.virtual_wall(position) {
            return -1;
        }
        i32::from(self.inflate[self.inf_address(self.position_to_index(position))])
    }

    fn virtual_wall(&self, position: Vector3<f64>) -> bool {
        self.parameters.enable_virtual_wall
            && (position.z >= self.parameters.virtual_ceil
                || position.z <= self.parameters.virtual_ground)
    }

    #[must_use]
    pub fn resolution(&self) -> f64 {
        self.parameters.resolution
    }
    #[must_use]
    pub fn initialized(&self) -> bool {
        self.initialized
    }
    #[must_use]
    pub fn odom_depth_timeout(&self) -> bool {
        self.depth_odom_timeout
    }
    #[must_use]
    pub fn projected_points(&self) -> &[Vector3<f64>] {
        &self.projected
    }
    /// Raw C++ ring-buffer layout; use `buffer_index` to locate a world point.
    #[must_use]
    pub fn occupancy_buffer(&self) -> &[f64] {
        &self.occupancy
    }
    #[must_use]
    pub fn inflation_buffer(&self) -> &[u16] {
        &self.inflate
    }
    #[must_use]
    pub fn buffer_index(&self, position: Vector3<f64>) -> Option<usize> {
        (self.initialized && self.is_in_buffer(position))
            .then(|| self.address(self.position_to_index(position)))
    }
    /// Integer bounds are inclusive, matching `ringbuffer_*bound3i_`.
    #[must_use]
    pub fn bounds(&self) -> Option<([i32; 3], [i32; 3])> {
        self.initialized
            .then_some((self.low.into(), self.high.into()))
    }

    /// Visit x-contiguous inflated runs within the occupancy window. Integer bounds are
    /// half-open; callers can preserve the exact lattice without serializing every voxel.
    /// Virtual walls remain a query-time constraint and are not included here.
    pub fn visit_inflated_runs(&self, mut visit: impl FnMut([i32; 3], [i32; 3])) {
        if !self.initialized {
            return;
        }
        for z in self.low.z..=self.high.z {
            for y in self.low.y..=self.high.y {
                let mut start = None;
                for x in self.low.x..=self.high.x {
                    let occupied = self.inflate[self.inf_address(Index::new(x, y, z))] > 0;
                    if occupied && start.is_none() {
                        start = Some(x);
                    } else if !occupied && let Some(begin) = start.take() {
                        visit([begin, y, z], [x, y + 1, z + 1]);
                    }
                }
                if let Some(begin) = start {
                    visit([begin, y, z], [self.high.x + 1, y + 1, z + 1]);
                }
            }
        }
    }
}
