/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Scale-by-scale Morton allocation with source allocation-result semantics.
use crate::{
    key_util,
    octant::{OctantId, invalid},
    octree::Octree,
};
use firefly_error::Result;
use nalgebra::Vector3;
/// # Errors
/// Returns an error for an invalid Morton key/scale, an absent base node, invalid tree descent,
/// or allocation below block size.
pub fn allocate_key(
    key: u64,
    tree: &mut Octree,
    base: OctantId,
) -> Result<(Option<OctantId>, bool)> {
    let code = key_util::key_to_code(key)?;
    let minimum = key_util::key_to_scale(key)?.max(tree.max_block_scale);
    let mut parent = base;
    let scale = tree.node(parent)?.base.scale() - 1;
    let mut result = None;
    let mut allocated = false;
    for s in (minimum..=scale).rev() {
        let index = key_util::code_to_child_index(code, s)?;
        let (id, new) = tree.allocate(parent, index)?;
        parent = id;
        result = Some(id);
        allocated = new;
    }
    Ok((result, allocated))
}
/// # Errors
/// Returns an error for an invalid coordinate or base node, failed tree descent, or allocation
/// that returns no child.
pub fn block(coord: &Vector3<i32>, tree: &mut Octree, base: OctantId) -> Result<OctantId> {
    let (key, _) = key_util::encode_key(coord, 0)?;
    allocate_key(key, tree, base)?
        .0
        .ok_or_else(|| invalid("block allocation returned no child"))
}
/// # Errors
/// Returns an error for an invalid Morton key/scale, an absent base node, invalid tree descent,
/// or allocation below block size.
pub fn blocks(
    coords: &[Vector3<i32>],
    tree: &mut Octree,
    base: OctantId,
    only_allocated: bool,
) -> Result<Vec<OctantId>> {
    let mut keys = std::collections::BTreeSet::new();
    for c in coords {
        keys.insert(key_util::encode_key(c, tree.max_block_scale)?.0);
    }
    blocks_from_keys(
        &mut keys.into_iter().collect::<Vec<_>>(),
        tree,
        base,
        only_allocated,
    )
}
/// # Errors
/// Returns an error for invalid keys/scales or tree state, a key coarser than block scale, or
/// allocation that returns no child.
pub fn blocks_from_keys(
    keys: &mut [u64],
    tree: &mut Octree,
    base: OctantId,
    only_allocated: bool,
) -> Result<Vec<OctantId>> {
    for scale in (tree.max_block_scale + 1..=tree.max_scale()).rev() {
        let mut level = Vec::new();
        key_util::unique_at_scale(keys, scale, &mut level, true)?;
        for key in level {
            allocate_key(key, tree, base)?;
        }
    }
    let mut output = Vec::new();
    for &key in keys.iter() {
        if key_util::key_to_scale(key)? > tree.max_block_scale {
            return Err(invalid("block key exceeds block scale"));
        }
        let (id, new) = allocate_key(key, tree, base)?;
        if !only_allocated || new {
            output.push(id.ok_or_else(|| invalid("block allocation returned no child"))?);
        }
    }
    Ok(output)
}
