/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Depth-driven occupancy updates, scale buffers and propagation to the root.
use crate::eigen_utils::{inverse_transform, matrix_vector_product, transform_point};
use crate::{
    block::Block,
    data::OccupancyData,
    image::{Image, invalid},
    map::Map,
    measurement::Measurements,
    multires_ofusion_core::{
        compute_tau, free_voxel, propagate_block_to_coarsest_scale, propagate_to_parent_node,
        update_voxel,
    },
    octant::{Octant, OctantId},
    pinhole_camera::PinholeCamera,
    volume_carver::{VarianceState, VolumeCarverAllocation},
};
use firefly_error::Result;
use firefly_srl_projection::ProjectionStatus;
use nalgebra::{IsometryMatrix3, Vector3};
use std::collections::{BTreeSet, HashSet};
pub struct Updater<'map, 'input> {
    map: &'map mut Map,
    sensor: &'input PinholeCamera,
    depth: &'input Image<f32>,
    sigma: &'input Image<f32>,
    t_cw: IsometryMatrix3<f32>,
    timestamp: i32,
    tau_min: f32,
    tau_max: f32,
    node_sets: Vec<BTreeSet<OctantId>>,
    freed_blocks: Vec<OctantId>,
}
impl<'map, 'input> Updater<'map, 'input> {
    /// # Errors
    /// Returns an error if the measurements do not include a depth sigma image.
    pub fn new(
        map: &'map mut Map,
        timestamp: i32,
        measurements: &Measurements<'input>,
    ) -> Result<Self> {
        let sigma = measurements
            .depth_sigma
            .ok_or_else(|| invalid("occupancy updater requires sigma image"))?;
        let tau_min = map.resolution * map.data_config.field.tau_min_factor;
        let tau_max = map.resolution * map.data_config.field.tau_max_factor;
        let node_sets = vec![BTreeSet::new(); map.octree.block_depth() as usize];
        Ok(Self {
            map,
            sensor: measurements.depth.sensor,
            depth: measurements.depth.image,
            sigma,
            t_cw: inverse_transform(&measurements.depth.t_wc),
            timestamp,
            tau_min,
            tau_max,
            node_sets,
            freed_blocks: Vec::new(),
        })
    }
    /// # Errors
    /// Returns an error for allocation-list length mismatches, invalid tree identities/depths
    /// or buffers, non-finite/out-of-image projections, or invalid integration strides.
    pub fn update(
        &mut self,
        allocation: &VolumeCarverAllocation,
        mut updated: Option<&mut HashSet<OctantId>>,
    ) -> Result<()> {
        if allocation.block_list.len() != allocation.variance_state_list.len()
            || allocation.block_list.len() != allocation.projects_inside_list.len()
        {
            return Err(invalid("allocation vector lengths differ"));
        }
        for &id in &allocation.node_list {
            let depth = self.map.octree.max_scale() - self.map.octree.get(id)?.base().scale();
            self.free_node_recurse(id, depth)?;
        }
        for (i, &id) in allocation.block_list.iter().enumerate() {
            self.update_block(
                id,
                allocation.variance_state_list[i] == VarianceState::Constant,
                allocation.projects_inside_list[i],
            )?;
        }
        for &id in &allocation.block_list {
            propagate_block_to_coarsest_scale(self.map.octree.block_mut(id)?)?;
        }
        for &id in &self.freed_blocks {
            propagate_block_to_coarsest_scale(self.map.octree.block_mut(id)?)?;
        }
        if let Some(set) = updated.as_deref_mut() {
            set.extend(allocation.block_list.iter().copied());
            set.extend(self.freed_blocks.iter().copied());
            for &id in &allocation.node_list {
                if self.map.octree.get(id)?.base().is_leaf() {
                    set.insert(id);
                }
            }
        }
        self.propagate_to_root(&allocation.block_list, updated)
    }
    fn propagate_to_root(
        &mut self,
        blocks: &[OctantId],
        mut updated: Option<&mut HashSet<OctantId>>,
    ) -> Result<()> {
        let block_depth = self.map.octree.block_depth();
        for &id in blocks {
            let parent = self
                .map
                .octree
                .get(id)?
                .base()
                .parent
                .ok_or_else(|| invalid("block has no parent"))?;
            self.node_sets
                .get_mut((block_depth - 1) as usize)
                .ok_or_else(|| invalid("invalid block depth"))?
                .insert(parent);
        }
        for depth in (1..block_depth).rev() {
            let ids: Vec<_> = self.node_sets[depth as usize].iter().copied().collect();
            for id in ids {
                let base = self.map.octree.get(id)?.base();
                if base.timestamp == self.timestamp {
                    continue;
                }
                let parent = base
                    .parent
                    .ok_or_else(|| invalid("nonroot propagation node has no parent"))?;
                let data = propagate_to_parent_node(&mut self.map.octree, id, self.timestamp)?;
                self.node_sets[depth as usize - 1].insert(parent);
                if let Some(set) = updated.as_deref_mut() {
                    set.insert(id);
                }
                if data.field.observed && f64::from(data.get_field()) <= 0.95f64 * (-100.0f64) {
                    if let Some(set) = updated.as_deref_mut() {
                        for child in self.map.octree.node(id)?.children.into_iter().flatten() {
                            set.remove(&child);
                        }
                    }
                    self.map.octree.delete_children(id)?;
                }
            }
        }
        let root = self.map.octree.root;
        propagate_to_parent_node(&mut self.map.octree, root, self.timestamp)?;
        Ok(())
    }
    fn free_block(&mut self, id: OctantId) -> Result<()> {
        let block = self.map.octree.block(id)?;
        let centre = transform_point(
            &self.t_cw,
            &self.map.voxel_to_point(&block.base.coord, block.base.size),
        );
        let last = if block.min_scale == -1 {
            0
        } else {
            block.current_scale
        };
        let computed = self.sensor.block_integration_scale(
            &centre,
            self.map.resolution,
            last,
            block.min_scale,
            block.max_scale,
        );
        let min = if block.min_scale == -1
            || f64::from(block.max_data().field.occupancy)
                < 0.95f64 * f64::from(self.map.data_config.field.log_odd_min)
        {
            self.map.data_config.field.fs_integr_scale
        } else {
            0.max(last - 1)
        };
        let max = if block.min_scale == -1 {
            block.max_scale
        } else {
            block.max_scale.min(last + 1)
        };
        let recommended = min.max(computed).min(max);
        let mut scale = last;
        let config = self.map.data_config;
        let block = self.map.octree.block_mut(id)?;
        if block.min_scale == -1 {
            scale = recommended;
            block.allocate_down_to(scale)?;
            block.current_scale = scale;
            block.init_curr_count()?;
            block.init_data = OccupancyData::default();
        } else if recommended != last {
            if recommended != block.buffer_scale() {
                block.init_buffer(recommended)?;
                if recommended < last {
                    copy_parent_to_buffer(block, last)?;
                }
            }
            let side = block.base.size >> recommended;
            for z in 0..side {
                for y in 0..side {
                    for x in 0..side {
                        let idx = (x + y * side + z * side * side) as usize;
                        let observed = free_voxel(block.buffer_data_mut(idx)?, &config);
                        block.incr_buffer_observed_count(observed)?;
                    }
                }
            }
            block.incr_buffer_integr_count(true)?;
            if block.switch_data()? {
                return Ok(());
            }
        } else {
            block.reset_buffer();
        }
        let side = block.base.size >> scale;
        for z in 0..side {
            for y in 0..side {
                for x in 0..side {
                    let idx = (x + y * side + z * side * side) as usize;
                    let observed = free_voxel(&mut block.data_at_scale_mut(scale)?[idx], &config);
                    block.incr_curr_observed_count(observed)?;
                }
            }
        }
        block.incr_curr_integr_count()?;
        Ok(())
    }
    fn update_block(
        &mut self,
        id: OctantId,
        low_variance: bool,
        project_inside: bool,
    ) -> Result<()> {
        let block = self.map.octree.block(id)?;
        let centre = transform_point(
            &self.t_cw,
            &self.map.voxel_to_point(&block.base.coord, block.base.size),
        );
        let last = if block.min_scale == -1 {
            0
        } else {
            block.current_scale
        };
        let computed = self.sensor.block_integration_scale(
            &centre,
            self.map.resolution,
            last,
            block.min_scale,
            block.max_scale,
        );
        let min = if low_variance
            && (block.min_scale == -1
                || f64::from(block.max_data().field.occupancy)
                    < 0.95f64 * f64::from(self.map.data_config.field.log_odd_min))
        {
            self.map.data_config.field.fs_integr_scale
        } else {
            0.max(last - 1)
        };
        let max = if block.min_scale == -1 {
            block.max_scale
        } else {
            block.max_scale.min(last + 1)
        };
        let recommended = min.max(computed).min(max);
        let mut scale = last;
        let block = self.map.octree.block_mut(id)?;
        if block.min_scale == -1 {
            scale = recommended;
            block.allocate_down_to(scale)?;
            block.current_scale = scale;
            block.init_curr_count()?;
            block.init_data = OccupancyData::default();
        } else if recommended != last {
            if recommended != block.buffer_scale() {
                block.init_buffer(recommended)?;
                if recommended < last {
                    copy_parent_to_buffer(block, last)?;
                }
            }
            self.update_block_data(id, &centre, recommended, low_variance, project_inside, true)?;
            if self.map.octree.block_mut(id)?.switch_data()? {
                return Ok(());
            }
        } else {
            block.reset_buffer();
        }
        self.update_block_data(id, &centre, scale, low_variance, project_inside, false)
    }
    fn update_block_data(
        &mut self,
        id: OctantId,
        centre: &Vector3<f32>,
        scale: i32,
        low_variance: bool,
        project_inside: bool,
        buffer: bool,
    ) -> Result<()> {
        let stride = 1i32
            .checked_shl(scale as u32)
            .ok_or_else(|| invalid("invalid integration stride"))?;
        let block = self.map.octree.block(id)?;
        let side = block.base.size >> scale;
        let base = transform_point(
            &self.t_cw,
            &self.map.voxel_to_point(&block.base.coord, stride),
        );
        let delta = self.t_cw.rotation.matrix()
            * (self.map.resolution
                * nalgebra::Matrix3::from_diagonal(&Vector3::repeat(stride as f32)));
        let tau = compute_tau(
            self.sensor.measurement_from_point(centre),
            self.tau_min,
            self.tau_max,
            &self.map.data_config,
        );
        let config = self.map.data_config;
        for z in 0..side {
            for y in 0..side {
                for x in 0..side {
                    let point = base
                        + matrix_vector_product(
                            &delta,
                            &Vector3::new(x as f32, y as f32, z as f32),
                        );
                    let (status, pixel) = self.sensor.model.project(&point);
                    if status != ProjectionStatus::Successful {
                        continue;
                    }
                    if !pixel.x.is_finite() || !pixel.y.is_finite() {
                        return Err(invalid("non-finite projected depth pixel"));
                    }
                    let (px, py) = ((pixel.x + 0.5f32) as i32, (pixel.y + 0.5f32) as i32);
                    if px < 0
                        || py < 0
                        || px >= self.depth.cols as i32
                        || py >= self.depth.rows as i32
                        || px >= self.sigma.cols as i32
                        || py >= self.sigma.rows as i32
                    {
                        return Err(invalid("projected depth sample outside image"));
                    }
                    let depth = self.depth.data[px as usize + py as usize * self.depth.cols];
                    if depth < self.sensor.near_plane {
                        continue;
                    }
                    let three_sigma =
                        3.0f32 * self.sigma.data[px as usize + py as usize * self.sigma.cols];
                    let idx = (x + y * side + z * side * side) as usize;
                    let block = self.map.octree.block_mut(id)?;
                    let data = if buffer {
                        block.buffer_data_mut(idx)?
                    } else {
                        &mut block.data_at_scale_mut(scale)?[idx]
                    };
                    let newly = if low_variance {
                        free_voxel(data, &config)
                    } else {
                        let measurement = self.sensor.measurement_from_point(&point);
                        // Eigen's three-term reduction groups the final two squares.
                        let range =
                            (point.x * point.x + (point.y * point.y + point.z * point.z)).sqrt();
                        let diff = (measurement - depth) * (range / measurement);
                        let newly = update_voxel(data, diff, tau, three_sigma, &config);
                        if depth > self.sensor.far_plane {
                            continue;
                        }
                        newly
                    };
                    if buffer {
                        block.incr_buffer_observed_count(newly)?;
                    } else {
                        block.incr_curr_observed_count(newly)?;
                    }
                }
            }
        }
        let block = self.map.octree.block_mut(id)?;
        if buffer {
            block.incr_buffer_integr_count(project_inside)?;
        } else {
            block.incr_curr_integr_count()?;
        }
        Ok(())
    }
    fn free_node_recurse(&mut self, id: OctantId, depth: i32) -> Result<()> {
        let node = self.map.octree.node(id)?;
        if node.base.is_leaf() {
            let mut data = node.data();
            data.field.update(
                self.map.data_config.field.log_odd_min,
                self.map.data_config.field.max_weight,
            );
            let parent = node
                .base
                .parent
                .ok_or_else(|| invalid("free-space root has no parent"))?;
            let node = self.map.octree.node_mut(id)?;
            node.min_data = data;
            node.max_data = data;
            self.node_sets
                .get_mut((depth - 1) as usize)
                .ok_or_else(|| invalid("free node depth outside tree"))?
                .insert(parent);
        } else {
            let children = node.children;
            for child in children {
                let child =
                    child.ok_or_else(|| invalid("free-space internal node missing child"))?;
                if matches!(self.map.octree.get(child)?, Octant::Block(_)) {
                    self.free_block(child)?;
                    let parent = self
                        .map
                        .octree
                        .get(child)?
                        .base()
                        .parent
                        .ok_or_else(|| invalid("free block has no parent"))?;
                    self.node_sets
                        .get_mut(depth as usize)
                        .ok_or_else(|| invalid("free block depth outside tree"))?
                        .insert(parent);
                    self.freed_blocks.push(child);
                } else {
                    self.free_node_recurse(child, depth + 1)?;
                }
            }
        }
        Ok(())
    }
}
fn copy_parent_to_buffer(block: &mut Block, parent_scale: i32) -> Result<()> {
    let side = block.base.size >> parent_scale;
    let fine = side * 2;
    let parents = block.data_at_scale(parent_scale)?.to_vec();
    for z in 0..side {
        for y in 0..side {
            for x in 0..side {
                let parent = parents[(x + y * side + z * side * side) as usize];
                for k in 0..2 {
                    for j in 0..2 {
                        for i in 0..2 {
                            let index =
                                ((2 * x + i) + (2 * y + j) * fine + (2 * z + k) * fine * fine)
                                    as usize;
                            let d = block.buffer_data_mut(index)?;
                            d.field.occupancy = parent.field.occupancy;
                            d.field.weight = parent.field.weight;
                            d.field.observed = false;
                        }
                    }
                }
            }
        }
    }
    Ok(())
}
