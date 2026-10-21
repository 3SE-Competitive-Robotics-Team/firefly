/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Occupancy inverse sensor model and multiresolution data propagation.
use crate::{
    block::Block,
    data::{DataConfig, OccupancyData, up_prop_max, up_prop_mean, up_prop_min},
    data_field::UncertaintyModel,
    image::invalid,
    octant::OctantId,
    octree::Octree,
    uncertainty::{cpp_clamp, cpp_min},
};
use firefly_error::Result;
#[must_use]
pub fn compute_three_sigma(depth: f32, sigma_min: f32, sigma_max: f32, config: &DataConfig) -> f32 {
    if config.field.uncertainty_model == UncertaintyModel::Linear {
        3.0 * cpp_clamp(config.field.k_sigma * depth, sigma_min, sigma_max)
    } else {
        3.0 * cpp_clamp(config.field.k_sigma * (depth * depth), sigma_min, sigma_max)
    }
}
#[must_use]
pub fn compute_tau(depth: f32, tau_min: f32, tau_max: f32, config: &DataConfig) -> f32 {
    cpp_clamp(config.field.k_tau * depth, tau_min, tau_max)
}
pub fn update_voxel(
    data: &mut OccupancyData,
    range_diff: f32,
    tau: f32,
    three_sigma: f32,
    config: &DataConfig,
) -> bool {
    let sample = if range_diff < -three_sigma {
        config.field.log_odd_min
    } else if range_diff < tau / 2.0 {
        cpp_min(
            config.field.log_odd_min
                - config.field.log_odd_min / three_sigma * (range_diff + three_sigma),
            config.field.log_odd_max,
        )
    } else if range_diff < tau {
        cpp_min(
            -config.field.log_odd_min * tau / (2.0 * three_sigma),
            config.field.log_odd_max,
        )
    } else {
        return false;
    };
    let observed = !data.field.observed;
    data.field.update(sample, config.field.max_weight);
    observed
}
pub fn free_voxel(data: &mut OccupancyData, config: &DataConfig) -> bool {
    let observed = !data.field.observed;
    data.field
        .update(config.field.log_odd_min, config.field.max_weight);
    observed
}
/// # Errors
/// Returns an error if an octant identity/type is invalid, scale data is absent, or the block
/// level sizes disagree.
pub fn propagate_to_parent_node(
    tree: &mut Octree,
    id: OctantId,
    timestamp: i32,
) -> Result<OccupancyData> {
    let children = tree.node(id)?.children;
    let mut mins = [OccupancyData::default(); 8];
    let mut maxs = mins;
    for (i, child) in children.into_iter().enumerate() {
        if let Some(child) = child {
            mins[i] = tree.get(child)?.min_data();
            maxs[i] = tree.get(child)?.max_data();
        }
    }
    let node = tree.node_mut(id)?;
    node.base.timestamp = timestamp;
    up_prop_min(&mut node.min_data, &mins);
    up_prop_max(&mut node.max_data, &maxs);
    Ok(node.max_data)
}
/// # Errors
/// Returns an error if an octant identity/type is invalid, scale data is absent, or the block
/// level sizes disagree.
pub fn propagate_block_to_coarsest_scale(block: &mut Block) -> Result<()> {
    if block.current_scale == block.max_scale {
        return Ok(());
    }
    let first = block.current_scale + 1;
    for parent_scale in first..=block.max_scale {
        let child_scale = parent_scale - 1;
        let side = block.base.size >> parent_scale;
        let child_side = side * 2;
        let child_mean = block.data_at_scale(child_scale)?.to_vec();
        let preserve_mean = parent_scale == first && block.buffer_scale() > block.current_scale;
        let child_min = if parent_scale == first {
            child_mean.clone()
        } else {
            block.min_data_at_scale(child_scale)?.to_vec()
        };
        let child_max = if parent_scale == first {
            child_mean.clone()
        } else {
            block.max_data_at_scale(child_scale)?.to_vec()
        };
        let count = (side * side * side) as usize;
        let mut means = block.data_at_scale(parent_scale)?.to_vec();
        let mut mins = block.min_data_at_scale(parent_scale)?.to_vec();
        let mut maxs = block.max_data_at_scale(parent_scale)?.to_vec();
        if means.len() != count || mins.len() != count || maxs.len() != count {
            return Err(invalid("block propagation scale storage mismatch"));
        }
        for z in 0..side {
            for y in 0..side {
                for x in 0..side {
                    let p = (x + y * side + z * side * side) as usize;
                    let mut cm = [OccupancyData::default(); 8];
                    let mut cl = cm;
                    let mut ch = cm;
                    for k in 0..2 {
                        for j in 0..2 {
                            for i in 0..2 {
                                let ci = (i + 2 * j + 4 * k) as usize;
                                let idx = ((2 * x + i)
                                    + (2 * y + j) * child_side
                                    + (2 * z + k) * child_side * child_side)
                                    as usize;
                                cm[ci] = child_mean[idx];
                                cl[ci] = child_min[idx];
                                ch[ci] = child_max[idx];
                            }
                        }
                    }
                    if !preserve_mean {
                        up_prop_mean(&mut means[p], &cm);
                    }
                    up_prop_min(&mut mins[p], &cl);
                    up_prop_max(&mut maxs[p], &ch);
                }
            }
        }
        if !preserve_mean {
            block
                .data_at_scale_mut(parent_scale)?
                .copy_from_slice(&means);
        }
        block
            .min_data_at_scale_mut(parent_scale)?
            .copy_from_slice(&mins);
        block
            .max_data_at_scale_mut(parent_scale)?
            .copy_from_slice(&maxs);
    }
    Ok(())
}
