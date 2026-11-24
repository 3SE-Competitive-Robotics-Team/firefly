use std::collections::HashSet;

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Point3, Vector3};

use crate::{CollisionSnapshot, GridMap, MapFile};

#[derive(Clone, Debug)]
pub(crate) struct FixedField {
    pub(crate) geometry: CollisionSnapshot,
    pub(crate) clearance: CollisionSnapshot,
}

impl FixedField {
    fn pack(voxels: &HashSet<[i64; 3]>, padding: i64) -> Result<Option<Self>> {
        let Some(&first) = voxels.iter().next() else {
            return Ok(None);
        };
        let mut low = first;
        let mut high = first;
        for g in voxels {
            for i in 0..3 {
                low[i] = low[i].min(g[i]);
                high[i] = high[i].max(g[i]);
            }
        }
        let mask = |radius| {
            let start = low.map(|v| v - radius);
            let dims = std::array::from_fn(|i| (high[i] + radius + 1 - start[i]) as usize);
            dims.into_iter()
                .try_fold(1usize, usize::checked_mul)
                .ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "field dimensions overflow")
                })?;
            Ok::<_, Error>(CollisionSnapshot::from_runs(
                start,
                dims,
                voxels
                    .iter()
                    .map(|g| (g.map(|v| v - radius), g.map(|v| v + radius + 1))),
            ))
        };
        let geometry = mask(0)?;
        let clearance = if padding == 0 {
            geometry.clone()
        } else {
            mask(padding)?
        };
        Ok(Some(Self {
            geometry,
            clearance,
        }))
    }
}

impl MapFile {
    /// Install fixed field voxels to the planning lattice using a surveyed field-to-odom pose.
    /// A target cell is occupied when its interior intersects a transformed source cell.
    /// Decoration and motion tracks are independent of the permanent geometry.
    ///
    /// # Errors
    /// Invalid geometry or coordinates outside the supported lattice reject the entire import.
    pub fn install_fixed(&self, map: &mut GridMap, odom_from_field: &Isometry3<f64>) -> Result<()> {
        let invalid = || Error::new(ErrorKind::InvalidArgument, "invalid fixed field geometry");
        if !self.resolution.is_finite()
            || self.resolution <= 0.
            || self.dims.contains(&0)
            || !self.origin.iter().all(|v| v.is_finite())
        {
            return Err(invalid());
        }
        let resolution = map.resolution();
        let anchor = map.world_origin();
        let world = [Vector3::x(), Vector3::y(), Vector3::z()];
        let field = world.map(|a| odom_from_field.rotation * a);
        let half = 0.5 * self.resolution;
        let extent = Vector3::from_fn(|i, _| half * field.iter().map(|a| a[i].abs()).sum::<f64>());
        // Two boxes overlap on all three face normals of each box and all edge cross products.
        let axes: Vec<_> = world
            .into_iter()
            .chain(field)
            .chain(world.into_iter().flat_map(|a| field.map(|b| a.cross(&b))))
            .filter_map(|a| a.try_normalize(16. * f64::EPSILON))
            .map(|a| {
                let radius = resolution * 0.5 * a.abs().sum()
                    + half * field.iter().map(|b| a.dot(b).abs()).sum::<f64>();
                (a, radius)
            })
            .collect();
        let mut occupied = HashSet::new();
        let origin = Vector3::from(self.origin);
        for &p in &self.occupied {
            let index = ((Vector3::from(p) - origin) / self.resolution).map(f64::floor);
            if !(0..3).all(|i| index[i] >= 0. && index[i] < self.dims[i] as f64) {
                return Err(invalid());
            }
            let center = origin + (index + Vector3::repeat(0.5)) * self.resolution;
            let center = (odom_from_field * Point3::from(center)).coords - anchor;
            let low = ((center - extent) / resolution).map(f64::floor);
            let end = ((center + extent) / resolution).map(f64::ceil);
            if !low
                .iter()
                .chain(end.iter())
                .all(|v| v.is_finite() && v.abs() < f64::from(i32::MAX))
            {
                return Err(invalid());
            }
            let tolerance = 16. * f64::EPSILON * (center.amax() + resolution + self.resolution);
            for x in low.x as i64..end.x as i64 {
                for y in low.y as i64..end.y as i64 {
                    for z in low.z as i64..end.z as i64 {
                        let delta = Vector3::new(x as f64 + 0.5, y as f64 + 0.5, z as f64 + 0.5)
                            * resolution
                            - center;
                        if axes
                            .iter()
                            .all(|(axis, r)| delta.dot(axis).abs() < r - tolerance)
                        {
                            occupied.insert([x, y, z]);
                        }
                    }
                }
            }
        }
        map.replace_field(FixedField::pack(
            &occupied,
            i64::from(map.inflation_steps()),
        )?);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GridMapBuilder;
    use std::time::Duration;

    #[test]
    fn rotated_cube_covers_faces_and_excludes_corner_contacts() {
        let field: MapFile = "FORMAT firefly-map 1\nRESOLUTION 1\nORIGIN 0 0 0\nDIMS 1 1 1\nOCCUPANCY\n0.5 0.5 0.5\n".parse().unwrap();
        let mut map = GridMapBuilder::new(1., [6; 3])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        map.move_ring_buffer(Vector3::zeros());
        let center = Vector3::repeat(0.5);
        let rotation = Isometry3::rotation(Vector3::z() * std::f64::consts::FRAC_PI_4);
        let offset = center - rotation.rotation * center;
        let transform = Isometry3::from_parts(offset.into(), rotation.rotation);
        field.install_fixed(&mut map, &transform).unwrap();
        for x in -1_i32..=1 {
            for y in -1_i32..=1 {
                for z in -1_i32..=1 {
                    let point = center + Vector3::new(x, y, z).cast::<f64>();
                    let expected = z == 0 && (x == 0 || y == 0);
                    assert_eq!(
                        map.cell_at(point, 0., Duration::ZERO)
                            .unwrap()
                            .fixed_occupied,
                        expected
                    );
                    assert_eq!(map.is_occupied_inflated(point), expected);
                }
            }
        }
    }

    #[test]
    fn fixed_clearance_reaches_into_window_from_outside_and_survives_eviction() {
        let field: MapFile =
            "RESOLUTION 0.5\nORIGIN 0 0 0\nDIMS 4 4 4\nOCCUPANCY\n1.25 0.25 0.25\n"
                .parse()
                .unwrap();
        let mut map = GridMapBuilder::new(0.25, [4; 3])
            .with_obstacles_inflation(0.25)
            .build()
            .unwrap();
        field
            .install_fixed(&mut map, &Isometry3::identity())
            .unwrap();
        let point = Vector3::new(0.875, 0.125, 0.125);
        let sample = map.cell_at(point, 0., Duration::ZERO).unwrap();
        assert!(sample.collision && !sample.fixed_occupied);
        map.clear_observations();
        map.move_ring_buffer(Vector3::repeat(10.));
        map.move_ring_buffer(Vector3::repeat(0.5));
        assert_eq!(map.cell_at(point, 0., Duration::ZERO).unwrap(), sample);
    }
}
