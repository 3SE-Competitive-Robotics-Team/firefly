/*
 * Adapted from KISS-ICP, https://github.com/PRBonn/kiss-icp
 * Vizzo, Ignacio, et al. "Kiss-icp: In defense of point-to-point icp–simple,
 * accurate, and robust registration if done the right way."
 * IEEE Robotics and Automation Letters 8.2 (2023): 1029-1036.
 */
//! Original-point voxel sampling corresponding to `VoxelGridFilter.hpp/.cpp`.
//! Voxel indices truncate toward zero, including at negative coordinates.
use crate::kinematics::transformation::Transformation;
use nalgebra::{RealField, Vector3};
use std::collections::{HashMap, HashSet};

pub type Voxel = [i32; 3];
pub type Point3<T> = Vector3<T>;
pub type Point3Cloud<T> = Vec<Point3<T>>;

/// Source hash arithmetic, with the two distinct Y multipliers retained.
#[must_use]
pub fn voxel_hash(voxel: Voxel, incremental_map: bool) -> u32 {
    let y = if incremental_map {
        19_349_669
    } else {
        19_349_663
    };
    ((voxel[0] as u32).wrapping_mul(73_856_093)
        ^ (voxel[1] as u32).wrapping_mul(y)
        ^ (voxel[2] as u32).wrapping_mul(83_492_791))
        & ((1 << 20) - 1)
}

/// A coordinate outside the C++ floating-to-integer conversion domain is rejected.
pub trait VoxelScalar: RealField + Copy {
    fn voxel_index(self, voxel_size: f64) -> Option<i32>;
}
impl VoxelScalar for f64 {
    fn voxel_index(self, size: f64) -> Option<i32> {
        let value = (self / size).trunc();
        (value.is_finite() && value >= f64::from(i32::MIN) && value <= f64::from(i32::MAX))
            .then_some(value as i32)
    }
}
impl VoxelScalar for f32 {
    fn voxel_index(self, size: f64) -> Option<i32> {
        // Eigen retains the point scalar when dividing a float point by a double.
        let value = (self / size as f32).trunc();
        (value.is_finite()
            && f64::from(value) >= f64::from(i32::MIN)
            && f64::from(value) <= f64::from(i32::MAX))
        .then_some(value as i32)
    }
}
fn voxel<T: VoxelScalar>(point: &Vector3<T>, size: f64) -> Option<Voxel> {
    Some([
        point[0].voxel_index(size)?,
        point[1].voxel_index(size)?,
        point[2].voxel_index(size)?,
    ])
}

/// Keeps the first point in each voxel; hash-table iteration order is unspecified.
pub fn voxel_down_sample<T: VoxelScalar>(
    points: &[Vector3<T>],
    size: f64,
) -> Option<Vec<Vector3<T>>> {
    let mut grid = HashMap::with_capacity(points.len());
    for point in points {
        grid.entry(voxel(point, size)?).or_insert(*point);
    }
    Some(grid.into_values().collect())
}
/// Keeps input order. Invalid indices leave the input unchanged.
pub fn voxel_down_sample_inplace<T: VoxelScalar>(points: &mut Vec<Vector3<T>>, size: f64) -> bool {
    let Some(indices) = points
        .iter()
        .map(|point| voxel(point, size))
        .collect::<Option<Vec<_>>>()
    else {
        return false;
    };
    let mut allocated = HashSet::with_capacity(points.len());
    let mut index = 0;
    points.retain(|_| {
        let keep = allocated.insert(indices[index]);
        index += 1;
        keep
    });
    true
}
#[derive(Clone, Debug)]
pub struct VoxelBlock {
    pub points: Vec<Vector3<f64>>,
    pub num_points: i32,
}
impl VoxelBlock {
    pub fn add_point(&mut self, point: Vector3<f64>) {
        if self.points.len() < self.num_points as usize {
            self.points.push(point);
        }
    }
}
#[derive(Clone, Debug)]
pub struct VoxelHashMap {
    pub voxel_size: f64,
    pub max_distance: f64,
    pub max_points_per_voxel: i32,
    pub map: HashMap<Voxel, VoxelBlock>,
}
impl VoxelHashMap {
    #[must_use]
    pub fn new(voxel_size: f64, max_distance: f64, max_points_per_voxel: i32) -> Self {
        Self {
            voxel_size,
            max_distance,
            max_points_per_voxel,
            map: HashMap::new(),
        }
    }
    pub fn clear(&mut self) {
        self.map.clear();
    }
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }
    #[must_use]
    pub fn pointcloud(&self) -> Vec<Vector3<f64>> {
        self.map
            .values()
            .flat_map(|block| block.points.iter().copied())
            .collect()
    }
    pub fn add_points(&mut self, points: &[Vector3<f64>]) -> bool {
        let Some(indices) = points
            .iter()
            .map(|p| voxel(p, self.voxel_size))
            .collect::<Option<Vec<_>>>()
        else {
            return false;
        };
        for (point, key) in points.iter().zip(indices) {
            match self.map.entry(key) {
                std::collections::hash_map::Entry::Occupied(mut entry) => {
                    entry.get_mut().add_point(*point);
                }
                std::collections::hash_map::Entry::Vacant(entry) => {
                    entry.insert(VoxelBlock {
                        points: vec![*point],
                        num_points: self.max_points_per_voxel,
                    });
                }
            }
        }
        true
    }
    /// Removes a whole voxel according to its first point, with strict `>` comparison.
    pub fn remove_points_far_from_location(&mut self, origin: &Vector3<f64>) {
        let distance2 = self.max_distance * self.max_distance;
        self.map.retain(|_, block| {
            block.points.first().is_none_or(|point| {
                (point - origin).norm_squared().partial_cmp(&distance2)
                    != Some(std::cmp::Ordering::Greater)
            })
        });
    }
    pub fn update(&mut self, points: &[Vector3<f64>], origin: &Vector3<f64>) -> bool {
        if !self.add_points(points) {
            return false;
        }
        self.remove_points_far_from_location(origin);
        true
    }
    pub fn update_with_pose(&mut self, points: &[Vector3<f64>], pose: &Transformation) -> bool {
        let rotation = pose.rotation_matrix();
        let translation = pose.translation();
        let transformed: Vec<_> = points
            .iter()
            .map(|point| rotation * point + translation)
            .collect();
        self.update(&transformed, &pose.translation())
    }
}
