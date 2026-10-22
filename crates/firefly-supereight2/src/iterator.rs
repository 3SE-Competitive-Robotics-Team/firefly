/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Depth-first octant iteration with the source LIFO child order.
use crate::{
    octant::{Octant, OctantId},
    octree::Octree,
};
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Selection {
    All,
    Nodes,
    Blocks,
    Leaves,
}
pub struct OctreeIterator<'a> {
    tree: &'a Octree,
    stack: Vec<OctantId>,
    selection: Selection,
}
impl<'a> OctreeIterator<'a> {
    #[must_use]
    pub fn new(tree: &'a Octree, selection: Selection) -> Self {
        Self {
            tree,
            stack: vec![tree.root],
            selection,
        }
    }
}
impl Iterator for OctreeIterator<'_> {
    type Item = OctantId;
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(id) = self.stack.pop() {
            let o = self.tree.octants.get(id)?.as_ref()?;
            if let Octant::Node(n) = o {
                for child in n.children.into_iter().flatten() {
                    self.stack.push(child);
                }
            }
            let selected = match self.selection {
                Selection::All => true,
                Selection::Nodes => !o.is_block(),
                Selection::Blocks => o.is_block(),
                Selection::Leaves => o.is_block() || o.base().is_leaf(),
            };
            if selected {
                return Some(id);
            }
        }
        None
    }
}
