/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Arena identities and the shared occupancy octant metadata.
use crate::{block::Block, data::OccupancyData, node::Node};
use firefly_error::Result;
use nalgebra::Vector3;
pub type OctantId = usize;
pub(crate) use crate::image::invalid;
#[derive(Clone, Debug)]
pub struct OctantBase {
    pub coord: Vector3<i32>,
    pub size: i32,
    pub parent: Option<OctantId>,
    pub timestamp: i32,
    pub child_mask: u8,
}
impl OctantBase {
    #[must_use]
    pub fn new(coord: Vector3<i32>, size: i32, parent: Option<OctantId>) -> Self {
        Self {
            coord,
            size,
            parent,
            timestamp: -1,
            child_mask: 0,
        }
    }
    #[must_use]
    pub fn is_leaf(&self) -> bool {
        self.child_mask == 0
    }
    #[must_use]
    pub fn scale(&self) -> i32 {
        self.size.ilog2() as i32
    }
}
#[derive(Clone, Debug)]
pub enum Octant {
    Node(Node),
    Block(Block),
}
impl Octant {
    #[must_use]
    pub fn base(&self) -> &OctantBase {
        match self {
            Self::Node(n) => &n.base,
            Self::Block(b) => &b.base,
        }
    }
    pub fn base_mut(&mut self) -> &mut OctantBase {
        match self {
            Self::Node(n) => &mut n.base,
            Self::Block(b) => &mut b.base,
        }
    }
    #[must_use]
    pub fn is_block(&self) -> bool {
        matches!(self, Self::Block(_))
    }
    #[must_use]
    pub fn data(&self) -> OccupancyData {
        match self {
            Self::Node(n) => n.data(),
            Self::Block(b) => b.data(),
        }
    }
    #[must_use]
    pub fn min_data(&self) -> OccupancyData {
        match self {
            Self::Node(n) => n.min_data,
            Self::Block(b) => b.min_data(),
        }
    }
    #[must_use]
    pub fn max_data(&self) -> OccupancyData {
        match self {
            Self::Node(n) => n.max_data,
            Self::Block(b) => b.max_data(),
        }
    }
    /// # Errors
    /// Returns an error if this octant is a block.
    pub fn as_node(&self) -> Result<&Node> {
        if let Self::Node(n) = self {
            Ok(n)
        } else {
            Err(invalid("node required"))
        }
    }
    /// # Errors
    /// Returns an error if this octant is a node.
    pub fn as_block(&self) -> Result<&Block> {
        if let Self::Block(b) = self {
            Ok(b)
        } else {
            Err(invalid("block required"))
        }
    }
}
