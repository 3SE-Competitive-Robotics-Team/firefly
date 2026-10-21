/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Arena-backed multi-resolution occupancy octree with stable octant identities.
use crate::{
    block::Block,
    data::OccupancyData,
    node::Node,
    octant::{Octant, OctantId, invalid},
};
use firefly_error::Result;
use nalgebra::Vector3;
#[derive(Clone, Debug)]
pub struct Octree {
    pub root: OctantId,
    pub size: i32,
    pub block_size: i32,
    pub max_block_scale: i32,
    pub octants: Vec<Option<Octant>>,
    pub aabb: Option<(Vector3<i32>, Vector3<i32>)>,
}
impl Octree {
    /// # Errors
    /// Returns an error for an invalid block size, overflowing rounded dimensions, or
    /// dimensions outside the Morton range.
    pub fn new(size: i32, block_size: i32) -> Result<Self> {
        if block_size <= 0 || (block_size & (block_size - 1)) != 0 {
            return Err(invalid("block size must be a power of two"));
        }
        let minimum = block_size
            .checked_mul(2)
            .ok_or_else(|| invalid("octree size overflow"))?;
        let size = (size.max(minimum) as u32)
            .checked_next_power_of_two()
            .ok_or_else(|| invalid("octree size overflow"))?;
        if size > 1 << 19 {
            return Err(invalid("Morton coordinate range exceeded"));
        }
        let node = Node::new(
            Vector3::zeros(),
            size as i32,
            None,
            OccupancyData::default(),
        )?;
        Ok(Self {
            root: 0,
            size: size as i32,
            block_size,
            max_block_scale: block_size.ilog2() as i32,
            octants: vec![Some(Octant::Node(node))],
            aabb: None,
        })
    }
    #[must_use]
    pub fn contains(&self, coord: &Vector3<i32>) -> bool {
        coord.iter().all(|v| *v >= 0 && *v < self.size)
    }
    /// # Errors
    /// Returns an error if the octant identity is absent or has been deleted.
    pub fn get(&self, id: OctantId) -> Result<&Octant> {
        self.octants
            .get(id)
            .and_then(Option::as_ref)
            .ok_or_else(|| invalid("octant identity absent"))
    }
    /// # Errors
    /// Returns an error if the octant identity is absent or has been deleted.
    pub fn get_mut(&mut self, id: OctantId) -> Result<&mut Octant> {
        self.octants
            .get_mut(id)
            .and_then(Option::as_mut)
            .ok_or_else(|| invalid("octant identity absent"))
    }
    /// # Errors
    /// Returns an error if the octant identity is absent or refers to the wrong octant type.
    pub fn node(&self, id: OctantId) -> Result<&Node> {
        self.get(id)?.as_node()
    }
    /// # Errors
    /// Returns an error if the octant identity is absent or refers to the wrong octant type.
    pub fn node_mut(&mut self, id: OctantId) -> Result<&mut Node> {
        match self.get_mut(id)? {
            Octant::Node(n) => Ok(n),
            Octant::Block(_) => Err(invalid("node required")),
        }
    }
    /// # Errors
    /// Returns an error if the octant identity is absent or refers to the wrong octant type.
    pub fn block(&self, id: OctantId) -> Result<&Block> {
        self.get(id)?.as_block()
    }
    /// # Errors
    /// Returns an error if the octant identity is absent or refers to the wrong octant type.
    pub fn block_mut(&mut self, id: OctantId) -> Result<&mut Block> {
        match self.get_mut(id)? {
            Octant::Block(b) => Ok(b),
            Octant::Node(_) => Err(invalid("block required")),
        }
    }
    #[must_use]
    pub fn max_scale(&self) -> i32 {
        self.size.ilog2() as i32
    }
    #[must_use]
    pub fn block_depth(&self) -> i32 {
        self.max_scale() - self.max_block_scale
    }
    /// # Errors
    /// Returns an error if the parent/child identity or type is invalid, the child index is out
    /// of range, or allocation descends below block size.
    pub fn allocate(&mut self, parent: OctantId, index: usize) -> Result<(OctantId, bool)> {
        let init = self.node(parent)?.data();
        self.allocate_with_data(parent, index, init)
    }
    fn allocate_with_data(
        &mut self,
        parent: OctantId,
        index: usize,
        init: OccupancyData,
    ) -> Result<(OctantId, bool)> {
        let p = self.node(parent)?;
        if let Some(id) = p.child(index)? {
            return Ok((id, false));
        }
        let coord = p.child_coord(index)?;
        let size = p.base.size / 2;

        let octant = match size.cmp(&self.block_size) {
            std::cmp::Ordering::Equal => Octant::Block(Block::new(coord, size, parent, init)?),
            std::cmp::Ordering::Greater => {
                Octant::Node(Node::new(coord, size, Some(parent), init)?)
            }
            std::cmp::Ordering::Less => {
                return Err(invalid("cannot allocate below voxel block size"));
            }
        };
        let id = self.octants.len();
        self.octants.push(Some(octant));
        self.node_mut(parent)?.set_child(index, Some(id))?;
        if size == self.block_size {
            self.aabb_extend(&coord, size);
        }
        Ok((id, true))
    }
    /// # Errors
    /// Returns an error if the parent is not a live node or a child cannot be allocated at a
    /// supported size.
    pub fn allocate_children(&mut self, parent: OctantId) -> Result<()> {
        let init = self.node(parent)?.data();
        for index in 0..8 {
            self.allocate_with_data(parent, index, init)?;
        }
        let base = self.get(parent)?.base().clone();
        if base.size == 2 * self.block_size {
            self.aabb_extend(&base.coord, base.size);
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if the parent or a descendant is absent or has an unexpected octant
    /// type.
    pub fn delete_children(&mut self, parent: OctantId) -> Result<()> {
        let children = self.node(parent)?.children;
        for (index, child) in children.into_iter().enumerate() {
            if let Some(id) = child {
                if matches!(self.get(id)?, Octant::Node(_)) {
                    self.delete_children(id)?;
                }
                self.octants[id] = None;
                self.node_mut(parent)?.set_child(index, None)?;
            }
        }
        Ok(())
    }
    pub fn aabb_extend(&mut self, coord: &Vector3<i32>, size: i32) {
        let upper = coord + Vector3::repeat(size - 1);
        self.aabb = Some(if let Some((lo, hi)) = self.aabb {
            (
                lo.zip_map(coord, std::cmp::Ord::min),
                hi.zip_map(&upper, std::cmp::Ord::max),
            )
        } else {
            (*coord, upper)
        });
    }
}

impl Octree {
    /// # Errors
    /// Returns an error for an invalid mesh scale or inconsistent neighbour/block scale
    /// storage.
    pub fn mesh(
        &self,
        t_ov: &nalgebra::Matrix4<f32>,
        min_desired_scale: i32,
    ) -> Result<crate::mesh::TriangleMesh> {
        let mut mesh = crate::marching_cube::marching_cube(self, min_desired_scale)?;
        crate::mesh::transform(&mut mesh, t_ov);
        Ok(mesh)
    }
}
