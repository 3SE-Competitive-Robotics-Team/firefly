/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Multi-resolution occupancy nodes and child geometry.
use crate::{
    data::OccupancyData,
    octant::{OctantBase, OctantId, invalid},
};
use firefly_error::Result;
use nalgebra::Vector3;
#[derive(Clone, Debug)]
pub struct Node {
    pub base: OctantBase,
    pub children: [Option<OctantId>; 8],
    pub min_data: OccupancyData,
    pub max_data: OccupancyData,
}
impl Node {
    /// # Errors
    /// Returns an error if `size` is not a positive power of two.
    pub fn new(
        coord: Vector3<i32>,
        size: i32,
        parent: Option<OctantId>,
        init_data: OccupancyData,
    ) -> Result<Self> {
        if size <= 0 || (size & (size - 1)) != 0 {
            return Err(invalid("node size must be a positive power of two"));
        }
        Ok(Self {
            base: OctantBase::new(coord, size, parent),
            children: [None; 8],
            min_data: init_data,
            max_data: init_data,
        })
    }
    #[must_use]
    pub fn data(&self) -> OccupancyData {
        if self.max_data.field.observed && self.base.is_leaf() {
            self.max_data
        } else {
            OccupancyData::default()
        }
    }
    /// # Errors
    /// Returns an error if the child index is outside `0..8`.
    pub fn child(&self, index: usize) -> Result<Option<OctantId>> {
        self.children
            .get(index)
            .copied()
            .ok_or_else(|| invalid("child index outside [0,7]"))
    }
    /// # Errors
    /// Returns an error if the child index is outside `0..8`.
    pub fn set_child(&mut self, index: usize, id: Option<OctantId>) -> Result<()> {
        *self
            .children
            .get_mut(index)
            .ok_or_else(|| invalid("child index outside [0,7]"))? = id;
        if id.is_some() {
            self.base.child_mask |= 1 << index;
        } else {
            self.base.child_mask &= !(1 << index);
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if the child index is outside `0..8`.
    pub fn child_coord(&self, index: usize) -> Result<Vector3<i32>> {
        if index >= 8 {
            return Err(invalid("child index outside [0,7]"));
        }
        let half = self.base.size / 2;
        Ok(self.base.coord
            + Vector3::new(
                i32::from(index & 1 != 0),
                i32::from(index & 2 != 0),
                i32::from(index & 4 != 0),
            ) * half)
    }
    #[must_use]
    pub fn child_index(&self, coord: &Vector3<i32>) -> usize {
        let d = coord - self.base.coord;
        let half = self.base.size / 2;
        usize::from((d.x & half) > 0)
            + 2 * usize::from((d.y & half) > 0)
            + 4 * usize::from((d.z & half) > 0)
    }
}
