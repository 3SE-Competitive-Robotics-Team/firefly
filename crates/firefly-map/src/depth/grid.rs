use super::{
    DepthImage, FrameStats, MappingParameters,
    config::{DepthSampling, invalid, logit},
    raycast::RayCaster,
};
use crate::{Evidence, ObservedSpace, VoxelBox, observed::EvidenceBuffer};
use firefly_error::Result;
use nalgebra::{Isometry3, Matrix3, Vector3};
use std::time::{Duration, Instant};

mod window;

const OBSTACLE_FLAG: u16 = 32767;
type Index = Vector3<i32>;

#[derive(Default)]
struct RayVotes {
    index: Index,
    hits: usize,
    misses: usize,
    saw_endpoint: bool,
    traversed: bool,
}

/// Local probabilistic map. Call depth reception and timer callbacks sequentially.
/// The raw buffer order matches C++: x is the fastest-varying coordinate.
pub(super) struct GridMap {
    collision_revision: u64,
    parameters: MappingParameters,
    resolution_inv: f64,
    size: Index,
    inf_size: Index,
    inf_grid: i32,
    window: Option<VoxelBox>,
    inf_window: Option<VoxelBox>,
    prob_hit_log: f64,
    prob_miss_log: f64,
    clamp_min_log: f64,
    clamp_max_log: f64,
    min_occupancy_log: f64,
    occupancy: Vec<f64>,
    evidence: EvidenceBuffer,
    inflate: Vec<u16>,
    collision_bits: crate::grid::buffer::Buffer<u64>,
    vote_slots: Vec<u32>,
    frame_votes: Vec<RayVotes>,
    projected: Vec<Vector3<f64>>,
    camera_position: Vector3<f64>,
    camera_rotation: Matrix3<f64>,
    camera_to_body_rotation: Matrix3<f64>,
    camera_to_body_translation: Vector3<f64>,
    depth: Option<DepthImage>,
    has_first_depth: bool,
    #[cfg(test)]
    has_odom: bool,
    #[cfg(test)]
    pub(super) reference_wall: Option<(f64, f64)>,
    depth_phase: DepthPhase,
    last_occupancy_update_time: f64,
    depth_odom_timeout: bool,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DepthPhase {
    Waiting,
    Pending,
    Integrated,
}

impl GridMap {
    /// `initMap`, including inflation-driven resolution adjustment.
    ///
    /// # Errors
    /// Rejects invalid parameters or dimensions outside the C++ signed-index range.
    pub(super) fn new(mut parameters: MappingParameters) -> Result<Self> {
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
            collision_revision: 0,
            prob_hit_log: logit(parameters.p_hit),
            prob_miss_log: logit(parameters.p_miss),
            clamp_min_log,
            clamp_max_log: logit(parameters.p_max),
            min_occupancy_log: logit(parameters.p_occ),
            parameters,
            resolution_inv,
            size,
            inf_size,
            inf_grid,
            window: None,
            inf_window: None,
            occupancy: vec![clamp_min_log; capacity],
            evidence: EvidenceBuffer::new(capacity),
            inflate: vec![0; inf_capacity],
            collision_bits: crate::grid::buffer::Buffer::new(0, inf_capacity.div_ceil(64)),
            vote_slots: vec![0; capacity],
            frame_votes: Vec::new(),
            projected: Vec::new(),
            camera_position: Vector3::zeros(),
            camera_rotation: Matrix3::identity(),
            camera_to_body_rotation: rotation,
            camera_to_body_translation: Vector3::zeros(),
            depth: None,
            has_first_depth: false,
            #[cfg(test)]
            has_odom: false,
            #[cfg(test)]
            reference_wall: None,
            depth_phase: DepthPhase::Waiting,
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

    /// Capture only the source pixels used for projection, in row-major order.
    /// `convert` maps a selected source value to depth units.
    /// # Errors
    /// Rejects empty dimensions and mismatched source storage.
    pub(super) fn sample_depth<T: Copy>(
        &self,
        width: usize,
        height: usize,
        source: &[T],
        convert: impl FnMut(T) -> u16,
    ) -> Result<DepthImage> {
        DepthImage::sample(
            width,
            height,
            source,
            DepthSampling::for_map(&self.parameters),
            convert,
        )
    }

    /// `depthPoseCallback`: pose is camera-to-world, and already paired with depth.
    /// # Errors
    /// Requires a matching sampling grid, a finite pose and indexable coordinates.
    #[cfg(test)]
    pub(super) fn depth_pose_callback(
        &mut self,
        depth: DepthImage,
        pose: Isometry3<f64>,
    ) -> Result<()> {
        depth.validate_sampling(DepthSampling::for_map(&self.parameters))?;
        self.validate_pose(&pose)?;
        self.camera_position = pose.translation.vector;
        self.camera_rotation = pose.rotation.to_rotation_matrix().into_inner();
        self.depth = Some(depth);
        self.depth_phase = DepthPhase::Pending;
        Ok(())
    }

    /// `depthOdomCallback`: compose body-to-world with camera-to-body extrinsics.
    /// # Errors
    /// Checks the sampling grid and pose as in `depth_pose_callback`.
    pub(super) fn depth_odom_callback(
        &mut self,
        depth: DepthImage,
        body_pose: Isometry3<f64>,
    ) -> Result<()> {
        depth.validate_sampling(DepthSampling::for_map(&self.parameters))?;
        let (position, rotation) = self.camera_pose_from_body(&body_pose)?;
        self.camera_position = position;
        self.camera_rotation = rotation;
        self.depth = Some(depth);
        self.depth_phase = DepthPhase::Pending;
        Ok(())
    }

    /// `extrinsicCallback`: camera-to-body transform.
    /// # Errors
    /// Rejects non-finite transforms or unsupported coordinate magnitudes.
    pub(super) fn extrinsic_callback(&mut self, camera_to_body: Isometry3<f64>) -> Result<()> {
        self.validate_pose(&camera_to_body)?;
        self.camera_to_body_rotation = camera_to_body.rotation.to_rotation_matrix().into_inner();
        self.camera_to_body_translation = camera_to_body.translation.vector;
        Ok(())
    }

    /// Independent odometry is ignored after the first synchronized depth pair.
    /// # Errors
    /// Rejects invalid poses before depth reception.
    #[cfg(test)]
    pub(super) fn odom_callback(&mut self, body_pose: Isometry3<f64>) -> Result<()> {
        if self.depth_phase != DepthPhase::Waiting {
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

    /// Tick the occupancy timer. A pending image uses `now` for its measurement stamp.
    /// Use `update_at` when the depth sensor and timer have distinct clocks.
    /// Returns whether an update was consumed, including the filter's empty first frame.
    /// # Errors
    /// Rejects invalid clock values or projected coordinates outside the signed-index range.
    #[fastrace::trace]
    pub(super) fn update_occupancy_callback(&mut self, now: f64) -> Result<bool> {
        Ok(self.update_at(now, now)?.is_some())
    }

    /// Integrate a paired image using its measurement stamp; the watchdog uses timer time.
    /// # Errors
    /// Both clocks must be finite and nonnegative.
    pub(super) fn update_at(&mut self, now: f64, measured_at: f64) -> Result<Option<FrameStats>> {
        if !now.is_finite() || now < 0. || !measured_at.is_finite() || measured_at < 0. {
            return Err(invalid("invalid map timer time"));
        }
        if self.depth_phase != DepthPhase::Pending {
            if self.depth_phase == DepthPhase::Integrated
                && now - self.last_occupancy_update_time > self.parameters.odom_depth_timeout
            {
                self.depth_odom_timeout = true;
            }
            return Ok(None);
        }
        let start = Instant::now();
        if let Err(error) = self.project_depth_image() {
            self.depth_phase = if self.window.is_some() {
                DepthPhase::Integrated
            } else {
                DepthPhase::Waiting
            };
            return Err(error);
        }
        self.last_occupancy_update_time = now;
        let projected = Instant::now();
        let cleared_voxels = self.move_ring_buffer();
        let moved = Instant::now();
        let mut raycast = Duration::ZERO;
        let mut inflation = Duration::ZERO;
        let mut voted_voxels = 0;
        if !self.projected.is_empty() {
            self.raycast_process(measured_at);
            let voted = Instant::now();
            raycast = voted - moved;
            voted_voxels = self.frame_votes.len();
            self.clear_and_inflate_local_map();
            inflation = voted.elapsed();
        }
        self.depth_phase = DepthPhase::Integrated;
        Ok(Some(FrameStats {
            measured_at,
            projection: projected - start,
            window: moved - projected,
            raycast,
            inflation,
            projected_rays: self.projected.len(),
            voted_voxels,
            cleared_voxels,
        }))
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
        let sampling = DepthSampling::for_map(p);
        let inv_factor = 1. / p.k_depth_scaling_factor;
        for (index, (u, v)) in sampling.coordinates(image.width, image.height).enumerate() {
            let address = if image.sampling.is_some() {
                index
            } else {
                v * image.width + u
            };
            let raw = f64::from(image.pixels[address]);
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
        Ok(())
    }

    fn votes_for(&mut self, index: Index) -> &mut RayVotes {
        let address = self.address(index);
        let slot = self.vote_slots[address] as usize;
        // A ring slot is valid only if its entry in this frame names this world voxel.
        if self.frame_votes.get(slot).is_some_and(|v| v.index == index) {
            return &mut self.frame_votes[slot];
        }
        let slot = self.frame_votes.len();
        self.vote_slots[address] = u32::try_from(slot).expect("map capacity fits i32");
        self.frame_votes.push(RayVotes {
            index,
            ..Default::default()
        });
        &mut self.frame_votes[slot]
    }

    #[fastrace::trace]
    fn raycast_process(&mut self, measured_at: f64) {
        self.frame_votes.clear();
        for i in 0..self.projected.len() {
            let mut point = self.projected[i];
            let occupied = self.is_in_buffer(point);
            if !occupied {
                point = self.closest_point_in_map(point);
            }
            let votes = self.votes_for(self.position_to_index(point));
            if occupied {
                votes.hits += 1;
            } else {
                votes.misses += 1;
            }
            if std::mem::replace(&mut votes.saw_endpoint, true) {
                continue;
            }
            let ray = RayCaster::new(
                point / self.parameters.resolution,
                self.camera_position / self.parameters.resolution,
            );
            for cell in ray {
                let votes = self.votes_for(cell);
                votes.misses += 1;
                if std::mem::replace(&mut votes.traversed, true) {
                    break;
                }
            }
        }
        for i in 0..self.frame_votes.len() {
            let votes = &self.frame_votes[i];
            let address = self.address(votes.index);
            let hit = votes.hits;
            let update = if hit >= votes.misses {
                self.prob_hit_log
            } else {
                self.prob_miss_log
            };
            self.reset_expired_obstacle(address, measured_at);
            self.occupancy[address] =
                (self.occupancy[address] + update).clamp(self.clamp_min_log, self.clamp_max_log);
            if self.occupancy[address] >= self.min_occupancy_log {
                if hit > 0 && update > 0. {
                    self.evidence.set(address, Evidence::Occupied(measured_at));
                }
            } else {
                let evidence = if hit == 0 {
                    Evidence::Free(measured_at)
                } else {
                    Evidence::Unknown
                };
                self.evidence.set(address, evidence);
            }
        }
        // Rays start at the sensor, but upstream's reverse iterator excludes that
        // cell. Retain this free-space evidence separately from its probability votes.
        if !self.projected.is_empty() {
            let address = self.address(self.position_to_index(self.camera_position));
            if self.occupancy[address] < self.min_occupancy_log {
                self.evidence.set(address, Evidence::Free(measured_at));
            }
        }
    }

    #[fastrace::trace]
    fn clear_and_inflate_local_map(&mut self) {
        for i in 0..self.frame_votes.len() {
            let id = self.frame_votes[i].index;
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
    #[cfg(test)]
    pub(super) fn cloud_callback(&mut self, points: &[Vector3<f64>]) {
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
            self.evidence.set(address, Evidence::Unknown);
            if self.inflate[self.inf_address(id)] < OBSTACLE_FLAG
                && self.occupancy[address] >= self.min_occupancy_log
            {
                self.change_inf_buf(true, id);
            }
        }
    }

    fn reset_expired_obstacle(&mut self, address: usize, measured_at: f64) -> bool {
        if self.occupancy[address] >= self.min_occupancy_log
            && let Some(age) = self.parameters.occupied_max_age
            && let Evidence::Occupied(stamp) = self.evidence.get(address)
            && measured_at - stamp > age
        {
            self.occupancy[address] = self.clamp_min_log;
            self.evidence.set(address, Evidence::Unknown);
            let id = self.address_to_index(address);
            if self.inflate[self.inf_address(id)] >= OBSTACLE_FLAG {
                self.change_inf_buf(false, id);
            }
            return true;
        }
        false
    }

    /// Retire old online obstacles and apply one configured 2 Hz decay step.
    #[fastrace::trace]
    pub(super) fn fading_callback(&mut self, measured_at: Option<f64>) {
        if self.parameters.fading_time <= 0. && self.parameters.occupied_max_age.is_none() {
            return;
        }
        let reduce = if self.parameters.fading_time > 0. {
            (self.clamp_max_log - self.min_occupancy_log) / (self.parameters.fading_time * 2.)
        } else {
            0.
        };
        let low_threshold = self.clamp_min_log + reduce;
        for address in 0..self.occupancy.len() {
            if measured_at.is_some_and(|now| self.reset_expired_obstacle(address, now)) {
                continue;
            }
            if reduce > 0. && self.occupancy[address] > low_threshold {
                let was_occupied = self.occupancy[address] >= self.min_occupancy_log;
                self.occupancy[address] -= reduce;
                if was_occupied && self.occupancy[address] < self.min_occupancy_log {
                    self.evidence.set(address, Evidence::Unknown);
                    self.change_inf_buf(false, self.address_to_index(address));
                }
            }
        }
    }

    fn set_inflation(&mut self, address: usize, count: u16) {
        let previous = std::mem::replace(&mut self.inflate[address], count);
        if (previous == 0) != (count == 0) {
            self.collision_bits[address / 64] ^= 1 << (address % 64);
            self.collision_revision = self.collision_revision.wrapping_add(1);
        }
    }

    fn change_inf_buf(&mut self, add: bool, id: Index) {
        let center = self.inf_address(id);
        let count = if add {
            self.inflate[center].wrapping_add(OBSTACLE_FLAG)
        } else {
            self.inflate[center].wrapping_sub(OBSTACLE_FLAG)
        };
        self.set_inflation(center, count);
        for x in -self.inf_grid..=self.inf_grid {
            for y in -self.inf_grid..=self.inf_grid {
                for z in -self.inf_grid..=self.inf_grid {
                    let address = self.inf_address(id + Index::new(x, y, z));
                    if add {
                        self.set_inflation(address, self.inflate[address].wrapping_add(1));
                    } else {
                        self.set_inflation(address, self.inflate[address].wrapping_sub(1));
                        // Upstream's unsigned underflow recovery, with the same reset scope.
                        if self.inflate[address] > 65000 {
                            self.occupancy.fill(self.clamp_min_log);
                            self.evidence.clear();
                            self.inflate.fill(0);
                            self.collision_bits.fill(0);
                            self.collision_revision = self.collision_revision.wrapping_add(1);
                        }
                    }
                }
            }
        }
    }

    /// C++ `getOccupancy`: 0 outside the window, -1 for an in-window virtual wall.
    #[must_use]
    #[cfg(test)]
    pub(super) fn occupancy_at(&self, position: Vector3<f64>) -> i32 {
        if !self.is_in_buffer(position) {
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
    #[cfg(test)]
    pub(super) fn inflated_occupancy_at(&self, position: Vector3<f64>) -> i32 {
        if !self
            .inf_window
            .is_some_and(|b| self.contains_position(b, position))
        {
            return 0;
        }
        if self.virtual_wall(position) {
            return -1;
        }
        i32::from(self.inflate[self.inf_address(self.position_to_index(position))])
    }

    #[cfg(test)]
    fn virtual_wall(&self, position: Vector3<f64>) -> bool {
        self.reference_wall
            .is_some_and(|(ground, ceil)| position.z <= ground || position.z >= ceil)
    }

    #[must_use]
    pub(super) fn resolution(&self) -> f64 {
        self.parameters.resolution
    }
    #[must_use]
    #[cfg(test)]
    pub(super) fn initialized(&self) -> bool {
        self.window.is_some()
    }
    #[must_use]
    pub(super) fn odom_depth_timeout(&self) -> bool {
        self.depth_odom_timeout
    }
    #[must_use]
    #[cfg(test)]
    pub(super) fn projected_points(&self) -> &[Vector3<f64>] {
        &self.projected
    }
    /// Raw C++ ring-buffer layout; use `buffer_index` to locate a world point.
    #[must_use]
    #[cfg(test)]
    pub(super) fn occupancy_buffer(&self) -> &[f64] {
        &self.occupancy
    }

    /// Immutable evidence from real rays, retaining each voxel's measurement time.
    #[must_use]
    pub(super) fn observations(&self) -> Option<ObservedSpace> {
        self.window
            .map(|bounds| self.evidence.snapshot(self.resolution(), bounds))
    }

    #[must_use]
    pub(super) fn collision_snapshot(&self, bounds: VoxelBox) -> crate::CollisionSnapshot {
        crate::CollisionSnapshot::from_ring(
            bounds,
            self.inf_size.map(|v| v as usize).into(),
            &self.collision_bits,
        )
    }

    #[must_use]
    pub(super) fn collision_revision(&self) -> u64 {
        self.collision_revision
    }

    #[must_use]
    #[cfg(test)]
    pub(super) fn inflation_buffer(&self) -> &[u16] {
        &self.inflate
    }
    #[must_use]
    #[cfg(test)]
    pub(super) fn buffer_index(&self, position: Vector3<f64>) -> Option<usize> {
        self.is_in_buffer(position)
            .then(|| self.address(self.position_to_index(position)))
    }
    /// Integer bounds are inclusive, matching `ringbuffer_*bound3i_`.
    #[must_use]
    pub(super) fn bounds(&self) -> Option<([i32; 3], [i32; 3])> {
        self.window
            .map(|b| (b.low().map(|v| v as i32), b.end().map(|v| (v - 1) as i32)))
    }

    /// Visit x-contiguous inflated runs within the occupancy window. Integer bounds are
    /// half-open; callers can preserve the exact lattice without serializing every voxel.
    /// Virtual walls remain a query-time constraint and are not included here.
    ///
    /// # Panics
    /// Panics if an internal row offset exceeds the grid's signed 32-bit width.
    pub(super) fn inflated_runs(&self) -> impl Iterator<Item = ([i32; 3], [i32; 3])> + '_ {
        let (low, high) = self.bounds().unwrap_or(([0; 3], [-1; 3]));
        let (low, high) = (Index::from(low), Index::from(high));
        (low.z..=high.z).flat_map(move |z| {
            (low.y..=high.y).flat_map(move |y| {
                let address = self.inf_address(Index::new(low.x, y, z));
                let stride = self.inf_size.x as usize;
                let offset = address % stride;
                let row = &self.inflate[address - offset..][..stride];
                let width = self.size.x as usize;
                let (before, after) = row.split_at(offset);
                let first = &after[..width.min(after.len())];
                let second = &before[..width - first.len()];
                occupied_runs(first, second).map(move |(start, end)| {
                    let x = |offset| low.x + i32::try_from(offset).expect("row width fits i32");
                    ([x(start), y, z], [x(end), y + 1, z + 1])
                })
            })
        })
    }
}

/// The two slices form one logical row across the ring-buffer seam.
fn occupied_runs<'a>(
    first: &'a [u16],
    second: &'a [u16],
) -> impl Iterator<Item = (usize, usize)> + 'a {
    let width = first.len() + second.len();
    let mut cells = first.iter().chain(second).copied().enumerate();
    std::iter::from_fn(move || {
        let (start, _) = cells.find(|&(_, count)| count != 0)?;
        let end = cells
            .find(|&(_, count)| count == 0)
            .map_or(width, |(i, _)| i);
        Some((start, end))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn occupied_runs_preserve_every_boundary_across_the_ring_seam() {
        for mask in 0..256_u32 {
            let row: [u16; 8] = std::array::from_fn(|i| if mask & (1 << i) == 0 { 0 } else { 17 });
            let expected: Vec<_> = (0..row.len())
                .filter(|&i| row[i] != 0 && (i == 0 || row[i - 1] == 0))
                .map(|start| {
                    (
                        start,
                        start + row[start..].iter().take_while(|&&v| v != 0).count(),
                    )
                })
                .collect();
            for seam in 0..=row.len() {
                let (first, second) = row.split_at(seam);
                assert_eq!(occupied_runs(first, second).collect::<Vec<_>>(), expected);
            }
        }
    }
}
