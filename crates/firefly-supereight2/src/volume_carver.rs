/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Pinhole allocation of multiresolution occupancy octants.
use crate::eigen_utils::{inverse_transform, transform_point, transform_points};
use crate::{
    dense_pooling_image::{Crossing, DensePoolingImage, Known},
    image::{Image, invalid},
    map::Map,
    multires_ofusion_core::compute_tau,
    octant::OctantId,
    pinhole_camera::PinholeCamera,
    uncertainty::cpp_min,
};
use firefly_error::Result;
use firefly_srl_projection::ProjectionStatus;
use nalgebra::{IsometryMatrix3, Vector2, Vector3};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VarianceState {
    Constant,
    Gradient,
    Undefined,
}
#[derive(Default)]
pub struct VolumeCarverAllocation {
    pub node_list: Vec<OctantId>,
    pub block_list: Vec<OctantId>,
    pub variance_state_list: Vec<VarianceState>,
    pub projects_inside_list: Vec<bool>,
}
pub struct VolumeCarver<'a> {
    map: &'a mut Map,
    sensor: &'a PinholeCamera,
    depth_pool: DensePoolingImage,
    sigma_pool: DensePoolingImage,
    t_sw: IsometryMatrix3<f32>,
    tau_min: f32,
    tau_max: f32,
    max_depth_value: f32,
    allocation: VolumeCarverAllocation,
}
impl<'a> VolumeCarver<'a> {
    /// # Errors
    /// Returns an error if depth/sigma storage or dimensions do not support the pooling
    /// pyramid.
    pub fn new(
        map: &'a mut Map,
        sensor: &'a PinholeCamera,
        depth: &Image<f32>,
        sigma: &Image<f32>,
        t_ws: &IsometryMatrix3<f32>,
        _timestamp: i32,
    ) -> Result<Self> {
        let depth_pool = DensePoolingImage::new(depth)?;
        let sigma_pool = DensePoolingImage::new(sigma)?;
        let tau_min = map.data_config.field.tau_min_factor * map.resolution;
        let tau_max = map.data_config.field.tau_max_factor * map.resolution;
        let max_depth_value = cpp_min(sensor.far_plane, depth_pool.max_value + tau_max);
        Ok(Self {
            map,
            sensor,
            depth_pool,
            sigma_pool,
            t_sw: inverse_transform(t_ws),
            tau_min,
            tau_max,
            max_depth_value,
            allocation: VolumeCarverAllocation::default(),
        })
    }
    /// # Errors
    /// Returns an error for missing/inconsistent tree children, invalid projections,
    /// unsupported pooling windows, or Morton/scale allocation errors.
    pub fn allocate(mut self) -> Result<VolumeCarverAllocation> {
        let root = self.map.octree.root;
        self.map.octree.allocate_children(root)?;
        let children = self.map.octree.node(root)?.children;
        for child in children {
            self.visit(
                child.ok_or_else(|| invalid("missing allocated root child"))?,
                1,
            )?;
        }
        for &id in &self.allocation.node_list {
            let base = self.map.octree.get(id)?.base();
            if base.child_mask == 0 {
                let (coord, size) = (base.coord, base.size);
                self.map.octree.aabb_extend(&coord, size);
            }
        }
        Ok(self.allocation)
    }
    #[must_use]
    pub fn compute_variance(
        &self,
        depth_min: f32,
        depth_max: f32,
        node_min: f32,
        node_max: f32,
        std_dev_max: f32,
    ) -> VarianceState {
        let diff_max = node_max - depth_min;
        let diff_min = node_min - depth_max;
        let tau = compute_tau(depth_max, self.tau_min, self.tau_max, &self.map.data_config);
        let three_sigma = 3.0f32 * std_dev_max;
        if f64::from(diff_min) > 1.25f64 * f64::from(tau) {
            VarianceState::Undefined
        } else if f64::from(diff_max) < -1.25f64 * f64::from(three_sigma) {
            VarianceState::Constant
        } else {
            VarianceState::Gradient
        }
    }
    fn camera_in_node(&self, coord: &Vector3<i32>, size: i32) -> bool {
        let p = self
            .map
            .point_to_voxel(&inverse_transform(&self.t_sw).translation.vector);
        (0..3).all(|i| p[i] >= coord[i] as f32 && p[i] < (coord[i] + size) as f32)
    }
    fn visit(&mut self, id: OctantId, depth: i32) -> Result<()> {
        let base = self.map.octree.get(id)?.base();
        let (coord, size) = (base.coord, base.size);
        let centre_w = self.map.voxel_to_point(&coord, size);
        let centre_s = transform_point(&self.t_sw, &centre_w);
        let radius = size as f32 * (3.0f32.sqrt() / 2.0f32) * self.map.resolution;
        if centre_s.z - radius > self.max_depth_value || centre_s.z + radius < 1.0e-6 {
            return Ok(());
        }
        let corners = transform_points(&self.t_sw, &self.map.voxel_to_corner_points(&coord, size));
        let front = corners
            .iter()
            .filter(|point| point.z.partial_cmp(&1.0e-6) != Some(std::cmp::Ordering::Less))
            .count();
        if front == 0 {
            return Ok(());
        }
        let projected = corners.map(|p| self.sensor.model.project(&p));
        let (mut split, mut inside, mut variance) = (false, false, VarianceState::Gradient);
        if depth < self.map.octree.block_depth() + 1 {
            if front < 8 {
                if self.camera_in_node(&coord, size)
                    || projected
                        .iter()
                        .any(|p| p.0 == ProjectionStatus::Successful)
                    || self.sensor.sphere_in_frustum_inf(&centre_s, radius)
                {
                    split = true;
                } else {
                    return Ok(());
                }
            } else {
                let mut min = Vector2::repeat(i32::MAX);
                let mut max = Vector2::repeat(i32::MIN);
                let mut dist_min = corners[0].z;
                let mut dist_max = dist_min;
                for (p, corner) in projected.iter().zip(corners) {
                    for axis in 0..2 {
                        let value = p.1[axis].clamp(-1e7, 1e7);
                        if !value.is_finite() {
                            return Err(invalid("non-finite volume projection"));
                        }
                        let value = value as i32;
                        min[axis] = min[axis].min(value);
                        max[axis] = max[axis].max(value);
                    }
                    if corner.z < dist_min {
                        dist_min = corner.z;
                    }
                    if dist_max < corner.z {
                        dist_max = corner.z;
                    }
                }
                let pixel = self.depth_pool.conservative_query(&min, &max)?;
                if pixel.status_known == Known::Unknown || dist_min > pixel.max + self.tau_max {
                    return Ok(());
                }
                let sigma = self.sigma_pool.conservative_query(&min, &max)?;
                variance =
                    self.compute_variance(pixel.min, pixel.max, dist_min, dist_max, sigma.max);
                if variance != VarianceState::Gradient {
                    let octant = self.map.octree.get(id)?;
                    let data = if matches!(octant, crate::octant::Octant::Block(_)) {
                        octant.max_data()
                    } else {
                        octant.data()
                    };
                    if data.field.observed && f64::from(data.get_field()) <= 0.95f64 * (-100.0f64) {
                        return Ok(());
                    }
                }
                split = pixel.status_crossing == Crossing::Crossing
                    || pixel.status_known == Known::PartKnown
                    || variance == VarianceState::Gradient;
                inside = pixel.status_known == Known::Known;
            }
        }
        let is_block = matches!(self.map.octree.get(id)?, crate::octant::Octant::Block(_));
        if split && !is_block {
            self.map.octree.allocate_children(id)?;
            let children = self.map.octree.node(id)?.children;
            for child in children {
                self.visit(
                    child.ok_or_else(|| invalid("missing allocated volume child"))?,
                    depth + 1,
                )?;
            }
        } else if is_block {
            self.allocation.block_list.push(id);
            self.allocation.variance_state_list.push(variance);
            self.allocation.projects_inside_list.push(inside);
        } else if variance == VarianceState::Constant {
            self.allocation.node_list.push(id);
        }
        Ok(())
    }
}
