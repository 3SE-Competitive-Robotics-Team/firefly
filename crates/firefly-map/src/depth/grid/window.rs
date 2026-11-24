use super::{GridMap, Index, OBSTACLE_FLAG};
use crate::{Evidence, VoxelBox};
use nalgebra::Vector3;

impl GridMap {
    pub(super) fn position_to_index(&self, point: Vector3<f64>) -> Index {
        (point / self.parameters.resolution).map(|v| v.floor() as i32)
    }
    fn buffer_address(id: Index, size: Index) -> usize {
        let local = id.zip_map(&size, i32::rem_euclid);
        (size.x * size.y * local.z + size.x * local.y + local.x) as usize
    }
    pub(super) fn address(&self, id: Index) -> usize {
        Self::buffer_address(id, self.size)
    }
    pub(super) fn inf_address(&self, id: Index) -> usize {
        Self::buffer_address(id, self.inf_size)
    }
    pub(super) fn address_to_index(&self, address: usize) -> Index {
        let low = self
            .window
            .expect("fading occupied cells requires an initialized window")
            .low();
        let a = address as i32;
        let local = Index::new(
            a % self.size.x,
            (a / self.size.x) % self.size.y,
            a / (self.size.x * self.size.y),
        );
        Index::from_fn(|i, _| {
            let low = i32::try_from(low[i]).expect("mapper bounds fit i32");
            low + (local[i] - low).rem_euclid(self.size[i])
        })
    }
    pub(super) fn contains_position(&self, bounds: VoxelBox, point: Vector3<f64>) -> bool {
        let r = self.parameters.resolution;
        point
            .iter()
            .enumerate()
            .all(|(i, &v)| v >= bounds.low()[i] as f64 * r && v < bounds.end()[i] as f64 * r)
            && bounds.contains(self.position_to_index(point).map(i64::from).into())
    }
    pub(super) fn is_in_buffer(&self, point: Vector3<f64>) -> bool {
        self.window
            .is_some_and(|b| self.contains_position(b, point))
    }
    pub(super) fn move_ring_buffer(&mut self) -> usize {
        let center = self
            .position_to_index(self.camera_position)
            .map(i64::from)
            .into();
        let bounds = |size: Index| {
            VoxelBox::centered(center, size.map(|n| n as usize).into())
                .expect("validated mapper coordinates and sizes fit i32")
        };
        let next = bounds(self.size);
        if self.window == Some(next) {
            return 0;
        }
        let mut cleared = 0;
        if let Some(old) = self.window {
            old.visit_difference(old.intersection(next), |slab| {
                cleared += self.clear_region(slab);
            });
        }
        self.window = Some(next);
        self.inf_window = Some(bounds(self.inf_size));
        self.collision_revision = self.collision_revision.wrapping_add(1);
        cleared
    }
    fn clear_region(&mut self, region: VoxelBox) -> usize {
        let low = region.low();
        let end = region.end();
        for z in low[2]..end[2] {
            for y in low[1]..end[1] {
                for x in low[0]..end[0] {
                    let id = Index::new(x as i32, y as i32, z as i32);
                    let a = self.address(id);
                    self.occupancy[a] = self.clamp_min_log;
                    self.evidence.set(a, Evidence::Unknown);
                    if self.inflate[self.inf_address(id)] >= OBSTACLE_FLAG {
                        self.change_inf_buf(false, id);
                    }
                }
            }
        }
        (0..3).map(|i| (end[i] - low[i]) as usize).product()
    }
    pub(super) fn closest_point_in_map(&self, point: Vector3<f64>) -> Vector3<f64> {
        let bounds = self.window.expect("ray integration initializes the window");
        let direction = point - self.camera_position;
        let mut t = 1.0_f64;
        for i in 0..3 {
            if direction[i] == 0. {
                continue;
            }
            let edge = if direction[i] > 0. {
                bounds.end()[i]
            } else {
                bounds.low()[i]
            };
            t = t.min(
                (edge as f64 * self.parameters.resolution - self.camera_position[i]) / direction[i],
            );
        }
        // Stay on the ray and strictly inside the last measured voxel.
        self.camera_position
            + direction * (t - (self.parameters.resolution * 1e-6 / direction.amax())).max(0.)
    }
}
