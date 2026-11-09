// SPDX-License-Identifier: GPL-3.0-only
use super::{GridMap, Index, OBSTACLE_FLAG};
use nalgebra::Vector3;

impl GridMap {
    pub(super) fn position_to_index(&self, point: Vector3<f64>) -> Index {
        (point * self.resolution_inv).map(|v| v.floor() as i32)
    }
    fn buffer_address(id: Index, origin: Index, size: Index) -> usize {
        let local = (id - origin).zip_map(&size, i32::rem_euclid);
        (size.x * size.y * local.z + size.x * local.y + local.x) as usize
    }
    pub(super) fn address(&self, id: Index) -> usize {
        Self::buffer_address(id, self.origin, self.size)
    }
    pub(super) fn inf_address(&self, id: Index) -> usize {
        Self::buffer_address(id, self.inf_origin, self.inf_size)
    }
    pub(super) fn address_to_index(&self, address: usize) -> Index {
        let address = address as i32;
        let mut id = Index::new(
            address % self.size.x,
            (address % (self.size.x * self.size.y)) / self.size.x,
            address / (self.size.x * self.size.y),
        ) + self.origin;
        for axis in 0..3 {
            if id[axis] > self.high[axis] {
                id[axis] -= self.size[axis];
            }
        }
        id
    }
    pub(super) fn contains(point: Vector3<f64>, low: Vector3<f64>, high: Vector3<f64>) -> bool {
        point.iter().all(|v| v.is_finite())
            && (0..3).all(|axis| point[axis] >= low[axis] && point[axis] <= high[axis])
    }
    pub(super) fn is_in_buffer(&self, point: Vector3<f64>) -> bool {
        Self::contains(point, self.low_position, self.high_position)
    }
    fn set_bounds(&mut self, center: Index) {
        self.low = center - self.half_range;
        self.high = center + self.half_range;
        self.low_position = self.low.cast::<f64>() * self.parameters.resolution;
        self.high_position = self.high.cast::<f64>() * self.parameters.resolution;
        self.high -= Index::repeat(1);
        self.inf_low = self.low - Index::repeat(self.inf_grid);
        self.inf_high = self.high + Index::repeat(self.inf_grid);
        let extent = Vector3::repeat(f64::from(self.inf_grid) * self.parameters.resolution);
        self.inf_low_position = self.low_position - extent;
        self.inf_high_position = self.high_position + extent;
    }
    fn align_origins(&mut self) {
        for axis in 0..3 {
            while self.origin[axis] < self.low[axis] {
                self.origin[axis] += self.size[axis];
            }
            while self.origin[axis] > self.high[axis] {
                self.origin[axis] -= self.size[axis];
            }
            while self.inf_origin[axis] < self.inf_low[axis] {
                self.inf_origin[axis] += self.inf_size[axis];
            }
            while self.inf_origin[axis] > self.inf_high[axis] {
                self.inf_origin[axis] -= self.inf_size[axis];
            }
        }
    }
    pub(super) fn move_ring_buffer(&mut self) {
        let center = self.position_to_index(self.camera_position);
        if !self.initialized {
            self.initialized = true;
            self.center_last = center;
            self.set_bounds(center);
            self.align_origins();
        }
        let low = center - self.half_range;
        let high = center + self.half_range - Index::repeat(1);
        for axis in 0..3 {
            if center[axis] < self.center_last[axis] {
                self.clear_buffer(axis * 2, high[axis]);
            }
            if center[axis] > self.center_last[axis] {
                self.clear_buffer(axis * 2 + 1, low[axis]);
            }
        }
        // C++ aligns against the existing bounds before assigning the new bounds.
        self.align_origins();
        self.center_last = center;
        self.set_bounds(center);
    }
    fn clear_buffer(&mut self, case: usize, bound: i32) {
        let mut low = self.low;
        let mut high = self.high;
        if case.is_multiple_of(2) {
            low[case / 2] = bound;
        } else {
            high[case / 2] = bound;
        }
        for x in low.x..=high.x {
            for y in low.y..=high.y {
                for z in low.z..=high.z {
                    let id = Index::new(x, y, z);
                    let address = self.address(id);
                    self.count_hit[address] = 0;
                    self.count_hit_and_miss[address] = 0;
                    self.flag_traverse[address] = self.raycast_num;
                    self.flag_rayend[address] = self.raycast_num;
                    self.occupancy[address] = self.clamp_min_log;
                    self.observed_free[address] = false;
                    if self.inflate[self.inf_address(id)] > OBSTACLE_FLAG {
                        self.change_inf_buf(false, id);
                    }
                }
            }
        }
    }
    pub(super) fn closest_point_in_map(&self, point: Vector3<f64>) -> Vector3<f64> {
        let difference = point - self.camera_position;
        let max_tc = self.high_position - self.camera_position;
        let min_tc = self.low_position - self.camera_position;
        let mut min_t = 1_000_000.;
        for axis in 0..3 {
            if difference[axis].abs() > 0. {
                let t1 = max_tc[axis] / difference[axis];
                if t1 > 0. && t1 < min_t {
                    min_t = t1;
                }
                let t2 = min_tc[axis] / difference[axis];
                if t2 > 0. && t2 < min_t {
                    min_t = t2;
                }
            }
        }
        self.camera_position + (min_t - 1e-3) * difference
    }
}
