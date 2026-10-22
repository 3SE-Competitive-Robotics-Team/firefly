/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Morton ordering of voxel blocks.
use crate::{key_util, octant::OctantId, octree::Octree};
use firefly_error::Result;
/// # Errors
/// Returns an error if a block identity is absent or its coordinate cannot be encoded as a
/// Morton key.
pub fn sort_blocks(tree: &Octree, blocks: &mut [OctantId], _descending: bool) -> Result<()> {
    let mut keys = Vec::with_capacity(blocks.len());
    for &id in blocks.iter() {
        let b = tree.get(id)?.base();
        keys.push((key_util::encode_key(&b.coord, b.scale())?.0, id));
    }
    // Both source Sort specializations compare ascending Morton keys.
    keys.sort_unstable_by_key(|a| a.0);
    for (out, (_, id)) in blocks.iter_mut().zip(keys) {
        *out = id;
    }
    Ok(())
}
