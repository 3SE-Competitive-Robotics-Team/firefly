/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Occupancy tree lookup and face-neighbour retrieval.
use crate::{
    octant::{Octant, OctantId, invalid},
    octree::Octree,
};
use firefly_error::Result;
use nalgebra::Vector3;
fn lookup(
    tree: &Octree,
    coord: &Vector3<i32>,
    scale: i32,
    base: OctantId,
    finest: bool,
) -> Result<Option<OctantId>> {
    if !(0..=30).contains(&scale) {
        return Err(invalid("invalid lookup scale"));
    }
    let desired = (1i32 << scale).max(tree.block_size);
    let mut parent = base;
    let mut child_size = tree.node(parent)?.base.size / 2;
    let mut child = None;
    while child_size >= desired {
        let n = tree.node(parent)?;
        let index = usize::from((coord.x & child_size) > 0)
            + 2 * usize::from((coord.y & child_size) > 0)
            + 4 * usize::from((coord.z & child_size) > 0);
        child = n.children[index];
        let Some(id) = child else {
            return Ok(if finest && n.base.is_leaf() {
                Some(parent)
            } else {
                None
            });
        };
        if child_size / 2 >= desired && matches!(tree.get(id)?, Octant::Block(_)) {
            return Err(invalid("lookup requires child node below block"));
        }
        parent = id;
        child_size /= 2;
    }
    Ok(child)
}
/// # Errors
/// Returns an error for an invalid scale, an absent base octant, or descent that requires a
/// node where a block is stored.
pub fn octant(
    tree: &Octree,
    coord: &Vector3<i32>,
    scale: i32,
    base: OctantId,
) -> Result<Option<OctantId>> {
    lookup(tree, coord, scale, base, false)
}
/// # Errors
/// Returns an error for an invalid scale, an absent base octant, or descent that requires a
/// node where a block is stored.
pub fn finest_octant(
    tree: &Octree,
    coord: &Vector3<i32>,
    scale: i32,
    base: OctantId,
) -> Result<Option<OctantId>> {
    lookup(tree, coord, scale, base, true)
}
/// # Errors
/// Returns an error for an invalid scale, an absent base octant, or descent that requires a
/// node where a block is stored.
pub fn block(tree: &Octree, coord: &Vector3<i32>, base: OctantId) -> Result<Option<OctantId>> {
    octant(tree, coord, tree.max_block_scale, base)
}
/// # Errors
/// Returns an error for an invalid scale, an absent base octant, or descent that requires a
/// node where a block is stored.
pub fn leaf(tree: &Octree, coord: &Vector3<i32>, base: OctantId) -> Result<Option<OctantId>> {
    finest_octant(tree, coord, tree.max_block_scale, base)
}
/// # Errors
/// Returns an error for an invalid scale, an absent base octant, or descent that requires a
/// node where a block is stored.
pub fn face_neighbours(tree: &Octree, id: OctantId) -> Result<Vec<Option<OctantId>>> {
    let b = tree.get(id)?.base();
    let mut out = Vec::new();
    for offset in [
        Vector3::new(0, 0, -1),
        Vector3::new(0, -1, 0),
        Vector3::new(-1, 0, 0),
        Vector3::new(1, 0, 0),
        Vector3::new(0, 1, 0),
        Vector3::new(0, 0, 1),
    ] {
        let coord = b.coord + offset * b.size;
        if !tree.contains(&coord) {
            continue;
        }
        let mut neighbour = finest_octant(tree, &coord, b.scale(), tree.root)?;
        if let Some(n) = neighbour {
            let nc = tree.get(n)?.base().coord;
            if (0..3).all(|i| b.coord[i] >= nc[i] && b.coord[i] < nc[i] + b.size) {
                neighbour = None;
            }
        }
        out.push(neighbour);
    }
    Ok(out)
}
