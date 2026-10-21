/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Occupancy queries, multi-resolution interpolation and finite-difference gradients.
use crate::{
    data::OccupancyData,
    fetcher,
    octant::{Octant, invalid},
    octree::Octree,
};
use firefly_error::Result;
use nalgebra::Vector3;
const OFFSETS: [[i32; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [1, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [0, 1, 1],
    [1, 1, 1],
];
/// # Errors
/// Returns an error if tree traversal encounters an absent/incorrect octant or the requested
/// block scale/data is unavailable.
pub fn get_data(tree: &Octree, coord: &Vector3<i32>) -> Result<OccupancyData> {
    let Some(id) = fetcher::leaf(tree, coord, tree.root)? else {
        return Ok(OccupancyData::default());
    };
    match tree.get(id)? {
        Octant::Block(b) => b.data_at(coord, b.current_scale),
        Octant::Node(n) => Ok(n.data()),
    }
}
/// # Errors
/// Returns an error if tree traversal encounters an absent/incorrect octant or the requested
/// block scale/data is unavailable.
pub fn get_data_scaled(
    tree: &Octree,
    coord: &Vector3<i32>,
    desired: i32,
) -> Result<(OccupancyData, Option<i32>)> {
    let Some(id) = fetcher::leaf(tree, coord, tree.root)? else {
        return Ok((OccupancyData::default(), None));
    };
    match tree.get(id)? {
        Octant::Block(b) => {
            let scale = desired.max(b.current_scale);
            Ok((b.data_at(coord, scale)?, Some(scale)))
        }
        Octant::Node(n) => Ok((n.data(), Some(desired))),
    }
}
/// # Errors
/// Returns an error if tree traversal encounters an absent/incorrect octant or the requested
/// block scale/data is unavailable.
pub fn get_min_data(tree: &Octree, coord: &Vector3<i32>, desired: i32) -> Result<OccupancyData> {
    let Some(id) = fetcher::finest_octant(tree, coord, desired, tree.root)? else {
        return Ok(OccupancyData::default());
    };
    match tree.get(id)? {
        Octant::Block(b) => b.min_data_at(coord, desired.max(b.current_scale)),
        Octant::Node(n) => Ok(n.min_data),
    }
}
/// # Errors
/// Returns an error if tree traversal encounters an absent/incorrect octant or the requested
/// block scale/data is unavailable.
pub fn get_max_data(tree: &Octree, coord: &Vector3<i32>, desired: i32) -> Result<OccupancyData> {
    let Some(id) = fetcher::finest_octant(tree, coord, desired, tree.root)? else {
        return Ok(OccupancyData::default());
    };
    match tree.get(id)? {
        Octant::Block(b) => b.max_data_at(coord, desired.max(b.current_scale)),
        Octant::Node(n) => Ok(n.max_data),
    }
}
/// # Errors
/// Returns an error if tree traversal encounters an absent/incorrect octant or the requested
/// block scale/data is unavailable.
pub fn get_field(tree: &Octree, coord: &Vector3<i32>) -> Result<Option<f32>> {
    let d = get_data(tree, coord)?;
    Ok(d.field.valid().then(|| d.get_field()))
}
fn crossmask(coord: &Vector3<i32>, size: i32, stride: i32) -> usize {
    usize::from((coord.x & (size - 1)) == size - stride) << 2
        | usize::from((coord.y & (size - 1)) == size - stride) << 1
        | usize::from((coord.z & (size - 1)) == size - stride)
}
/// # Errors
/// Returns an error if tree traversal encounters an absent/incorrect octant or the requested
/// block scale/data is unavailable.
pub fn get_neighbours(
    tree: &Octree,
    base: &Vector3<i32>,
    scale: i32,
) -> Result<Option<[OccupancyData; 8]>> {
    if !(0..=tree.max_block_scale).contains(&scale) {
        return Err(invalid("interpolation scale outside block"));
    }
    let stride = 1 << scale;
    let upper = base + Vector3::repeat(stride);
    if !tree.contains(base) || !tree.contains(&upper) {
        return Ok(None);
    }
    let Some(base_id) = fetcher::leaf(tree, base, tree.root)? else {
        return Ok(None);
    };
    let o = tree.get(base_id)?;
    let mut mask = crossmask(base, o.base().size, stride);
    if mask != 0 && !o.is_block() {
        mask = crossmask(base, tree.block_size, stride);
    }
    let groups: &[&[usize]] = match mask {
        0 => &[&[0, 1, 2, 3, 4, 5, 6, 7]],
        1 => &[&[0, 1, 2, 3], &[4, 5, 6, 7]],
        2 => &[&[0, 1, 4, 5], &[2, 3, 6, 7]],
        3 => &[&[0, 1], &[2, 3], &[4, 5], &[6, 7]],
        4 => &[&[0, 2, 4, 6], &[1, 3, 5, 7]],
        5 => &[&[0, 2], &[1, 3], &[4, 6], &[5, 7]],
        6 => &[&[0, 4], &[1, 5], &[2, 6], &[3, 7]],
        _ => &[&[0], &[1], &[2], &[3], &[4], &[5], &[6], &[7]],
    };
    let mut output = [OccupancyData::default(); 8];
    for group in groups {
        let offset = Vector3::from(OFFSETS[group[0]]) * stride;
        let Some(id) = fetcher::leaf(tree, &(base + offset), tree.root)? else {
            return Ok(None);
        };
        match tree.get(id)? {
            Octant::Block(b) => {
                if b.current_scale > scale {
                    return Ok(None);
                }
                for &i in *group {
                    output[i] = b.data_at(&(base + Vector3::from(OFFSETS[i]) * stride), scale)?;
                }
            }
            Octant::Node(n) => {
                let d = n.data();
                for &i in *group {
                    output[i] = d;
                }
            }
        }
    }
    Ok(Some(output))
}
fn in_aabb(tree: &Octree, p: &Vector3<i32>) -> bool {
    tree.aabb
        .is_some_and(|(lo, hi)| (0..3).all(|i| p[i] >= lo[i] && p[i] <= hi[i]))
}
/// # Errors
/// Returns an error for non-finite coordinates, negative/invalid interpolation scales, or
/// inconsistent neighbour/block storage.
pub fn interp(
    tree: &Octree,
    coord: &Vector3<f32>,
    desired: i32,
    valid: impl Fn(&OccupancyData) -> bool,
    get: impl Fn(&OccupancyData) -> f32,
) -> Result<Option<(f32, i32)>> {
    if coord.iter().any(|v| !v.is_finite()) || desired < 0 {
        return Err(invalid(
            "finite interpolation coordinates and nonnegative scale required",
        ));
    }
    let Some(id) = fetcher::leaf(tree, &coord.map(|v| v as i32), tree.root)? else {
        return Ok(None);
    };
    let octant = tree.get(id)?;
    let initial = match octant {
        Octant::Block(b) => b.current_scale.max(desired),
        Octant::Node(_) => 0,
    };
    for scale in initial..=tree.max_block_scale {
        let stride = 1 << scale;
        let base_f = (1.0 / stride as f32) * coord - Vector3::repeat(0.5);
        let base = base_f.map(|v| v as i32) * stride;
        if !in_aabb(tree, &base) {
            return Ok(None);
        }
        let Some(data) = get_neighbours(tree, &base, scale)? else {
            continue;
        };
        if data.iter().any(|d| !valid(d)) {
            return Ok(None);
        }
        let t = base_f.map(|v| v - v.floor());
        let tc = Vector3::repeat(1.0) - t;
        let value = ((get(&data[0]) * tc.x + get(&data[1]) * t.x) * tc.y
            + (get(&data[2]) * tc.x + get(&data[3]) * t.x) * t.y)
            * tc.z
            + ((get(&data[4]) * tc.x + get(&data[5]) * t.x) * tc.y
                + (get(&data[6]) * tc.x + get(&data[7]) * t.x) * t.y)
                * t.z;
        return Ok(Some((
            value,
            if octant.is_block() {
                scale
            } else {
                octant.base().scale()
            },
        )));
    }
    Ok(None)
}
/// # Errors
/// Returns an error for non-finite coordinates, negative/invalid interpolation scales, or
/// inconsistent neighbour/block storage.
pub fn interp_field(
    tree: &Octree,
    coord: &Vector3<f32>,
    desired: i32,
) -> Result<Option<(f32, i32)>> {
    interp(
        tree,
        coord,
        desired,
        |d| d.field.valid(),
        OccupancyData::get_field,
    )
}
/// # Errors
/// Returns an error if the gradient base is outside the octree or the scale is outside
/// the voxel-block scale range.
pub fn gradient_sample_coords(
    tree: &Octree,
    base: &Vector3<i32>,
    scale: i32,
) -> Result<[Vector3<i32>; 32]> {
    if !tree.contains(base) || !(0..=tree.max_block_scale).contains(&scale) {
        return Err(invalid("gradient base outside octree"));
    }
    let stride = 1 << scale;
    let lower_lower = (base - Vector3::repeat(stride)).map(|v| v.max(0));
    let lower_upper = *base;
    let upper_lower = (base + Vector3::repeat(stride)).map(|v| v.min(tree.size - 1));
    let upper_upper = (base + Vector3::repeat(2 * stride)).map(|v| v.min(tree.size - 1));
    Ok([
        Vector3::new(lower_lower.x, lower_upper.y, lower_upper.z),
        Vector3::new(lower_lower.x, upper_lower.y, lower_upper.z),
        Vector3::new(lower_lower.x, lower_upper.y, upper_lower.z),
        Vector3::new(lower_lower.x, upper_lower.y, upper_lower.z),
        Vector3::new(lower_upper.x, lower_lower.y, lower_upper.z),
        Vector3::new(lower_upper.x, lower_lower.y, upper_lower.z),
        Vector3::new(lower_upper.x, lower_upper.y, lower_lower.z),
        Vector3::new(lower_upper.x, lower_upper.y, lower_upper.z),
        Vector3::new(lower_upper.x, lower_upper.y, upper_lower.z),
        Vector3::new(lower_upper.x, lower_upper.y, upper_upper.z),
        Vector3::new(lower_upper.x, upper_lower.y, lower_lower.z),
        Vector3::new(lower_upper.x, upper_lower.y, lower_upper.z),
        Vector3::new(lower_upper.x, upper_lower.y, upper_lower.z),
        Vector3::new(lower_upper.x, upper_lower.y, upper_upper.z),
        Vector3::new(lower_upper.x, upper_upper.y, lower_upper.z),
        Vector3::new(lower_upper.x, upper_upper.y, upper_lower.z),
        Vector3::new(upper_lower.x, lower_lower.y, lower_upper.z),
        Vector3::new(upper_lower.x, lower_lower.y, upper_lower.z),
        Vector3::new(upper_lower.x, lower_upper.y, lower_lower.z),
        Vector3::new(upper_lower.x, lower_upper.y, lower_upper.z),
        Vector3::new(upper_lower.x, lower_upper.y, upper_lower.z),
        Vector3::new(upper_lower.x, lower_upper.y, upper_upper.z),
        Vector3::new(upper_lower.x, upper_lower.y, lower_lower.z),
        Vector3::new(upper_lower.x, upper_lower.y, lower_upper.z),
        Vector3::new(upper_lower.x, upper_lower.y, upper_lower.z),
        Vector3::new(upper_lower.x, upper_lower.y, upper_upper.z),
        Vector3::new(upper_lower.x, upper_upper.y, lower_upper.z),
        Vector3::new(upper_lower.x, upper_upper.y, upper_lower.z),
        Vector3::new(upper_upper.x, lower_upper.y, lower_upper.z),
        Vector3::new(upper_upper.x, upper_lower.y, lower_upper.z),
        Vector3::new(upper_upper.x, lower_upper.y, upper_lower.z),
        Vector3::new(upper_upper.x, upper_lower.y, upper_lower.z),
    ])
}
/// # Errors
/// Returns an error if fractions are non-finite/outside `[0,1]` or the scale is outside the
/// supported shift range.
pub fn gradient(t: &Vector3<f32>, data: &[f32; 32], scale: i32) -> Result<Vector3<f32>> {
    if !(0..=30).contains(&scale) || t.iter().any(|v| !v.is_finite() || *v < 0.0 || *v > 1.0) {
        return Err(invalid("invalid gradient fractions or scale"));
    }
    let scaling = 0.5 / (1i32 << scale) as f32;
    let tc = Vector3::repeat(1.0) - t;
    let mut grad = Vector3::<f32>::zeros();
    grad[0] = scaling
        * ((((data[19] - data[0]) * tc.x + (data[28] - data[7]) * t.x) * tc.y
            + ((data[23] - data[1]) * tc.x + (data[29] - data[11]) * t.x) * t.y)
            * tc.z
            + (((data[20] - data[2]) * tc.x + (data[30] - data[8]) * t.x) * tc.y
                + ((data[24] - data[3]) * tc.x + (data[31] - data[12]) * t.x) * t.y)
                * t.z);
    grad[1] = scaling
        * ((((data[11] - data[4]) * tc.x + (data[23] - data[16]) * t.x) * tc.y
            + ((data[14] - data[7]) * tc.x + (data[26] - data[19]) * t.x) * t.y)
            * tc.z
            + (((data[12] - data[5]) * tc.x + (data[24] - data[17]) * t.x) * tc.y
                + ((data[15] - data[8]) * tc.x + (data[27] - data[20]) * t.x) * t.y)
                * t.z);
    grad[2] = scaling
        * ((((data[8] - data[6]) * tc.x + (data[20] - data[18]) * t.x) * tc.y
            + ((data[12] - data[10]) * tc.x + (data[24] - data[22]) * t.x) * t.y)
            * tc.z
            + (((data[9] - data[7]) * tc.x + (data[21] - data[19]) * t.x) * tc.y
                + ((data[13] - data[11]) * tc.x + (data[25] - data[23]) * t.x) * t.y)
                * t.z);
    Ok(grad)
}
/// # Errors
/// Returns an error for out-of-tree/non-finite query coordinates, invalid scales/fractions, or
/// inconsistent gradient samples.
pub fn grad(
    tree: &Octree,
    coord: &Vector3<f32>,
    desired: i32,
    valid: impl Fn(&OccupancyData) -> bool,
    get: impl Fn(&OccupancyData) -> f32,
) -> Result<Option<(Vector3<f32>, i32)>> {
    if coord
        .iter()
        .any(|v| !v.is_finite() || *v < 0.0 || *v >= tree.size as f32)
        || desired < 0
    {
        return Err(invalid("gradient query outside octree"));
    }
    let Some(id) = fetcher::finest_octant(tree, &coord.map(|v| v as i32), desired, tree.root)?
    else {
        return Ok(None);
    };
    let octant = tree.get(id)?;
    let block = match octant {
        Octant::Node(n) => {
            return Ok(n
                .data()
                .field
                .valid()
                .then(|| (Vector3::zeros(), n.base.scale())));
        }
        Octant::Block(b) => b,
    };
    let initial = desired.max(block.current_scale);
    for scale in initial..=tree.max_block_scale {
        let stride = 1 << scale;
        let scaled = (1.0 / stride as f32) * coord - Vector3::repeat(0.5);
        let base = scaled.map(|v| v as i32) * stride;
        let Some(base_id) = fetcher::finest_octant(tree, &base, scale, tree.root)? else {
            continue;
        };
        if let Octant::Node(n) = tree.get(base_id)? {
            if n.base.is_leaf() && n.data().field.valid() {
                return Ok(Some((Vector3::zeros(), n.base.scale())));
            }
            continue;
        }
        let samples = gradient_sample_coords(tree, &base, scale)?;
        let mut values = [0.0; 32];
        let mut good = true;
        for (i, p) in samples.iter().enumerate() {
            let (d, returned) = get_data_scaled(tree, p, scale)?;
            if returned != Some(scale) || !valid(&d) {
                good = false;
                break;
            }
            values[i] = get(&d);
        }
        if !good {
            continue;
        }
        return Ok(Some((
            gradient(&scaled.map(|v| v - v.floor()), &values, scale)?,
            scale,
        )));
    }
    Ok(None)
}
/// # Errors
/// Returns an error for out-of-tree/non-finite query coordinates, invalid scales/fractions, or
/// inconsistent gradient samples.
pub fn grad_field(
    tree: &Octree,
    coord: &Vector3<f32>,
    desired: i32,
) -> Result<Option<(Vector3<f32>, i32)>> {
    grad(
        tree,
        coord,
        desired,
        |d| d.field.valid(),
        OccupancyData::get_field,
    )
}
