/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Adaptive dual marching cubes for multi-resolution occupancy blocks.
use crate::{
    block::Block,
    data::OccupancyData,
    edge_tables::TRI_TABLE,
    fetcher,
    image::invalid,
    iterator::{OctreeIterator, Selection},
    mesh::{MeshFace, TriangleMesh},
    octree::Octree,
};
use firefly_error::Result;
use nalgebra::Vector3;
pub const NORMALIZED_DUAL_OFFSETS: [[i32; 3]; 8] = [
    [-1, -1, -1],
    [1, -1, -1],
    [1, -1, 1],
    [-1, -1, 1],
    [-1, 1, -1],
    [1, 1, -1],
    [1, 1, 1],
    [-1, 1, 1],
];
pub const LOGICAL_DUAL_OFFSETS: [[i32; 3]; 8] = [
    [-1, -1, -1],
    [0, -1, -1],
    [0, -1, 0],
    [-1, -1, 0],
    [-1, 0, -1],
    [0, 0, -1],
    [0, 0, 0],
    [-1, 0, 0],
];
pub const EDGE_CORNERS: [[usize; 2]; 12] = [
    [0, 1],
    [1, 2],
    [2, 3],
    [0, 3],
    [4, 5],
    [5, 6],
    [6, 7],
    [4, 7],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
];
#[must_use]
pub fn occupied_index(data: &[OccupancyData; 8]) -> u8 {
    let mut index = 0;
    for (i, d) in data.iter().enumerate() {
        if !d.valid() {
            return 0;
        }
        if d.is_inside() {
            index |= 1 << i;
        }
    }
    index
}
#[must_use]
pub fn free_index(data: &[OccupancyData; 8]) -> u8 {
    data.iter().enumerate().fold(0, |i, (j, d)| {
        i | if d.valid() && !d.is_inside() {
            1 << j
        } else {
            0
        }
    })
}
pub fn frontier_index(data: &[OccupancyData; 8]) -> u8 {
    if data.iter().all(OccupancyData::valid) {
        0
    } else {
        free_index(data)
    }
}
#[must_use]
pub fn dual_intersection(
    a: &OccupancyData,
    b: &OccupancyData,
    p: &Vector3<f32>,
    q: &Vector3<f32>,
) -> Vector3<f32> {
    let va = a.get_field();
    let vb = b.get_field();
    p + (-va) / (vb - va) * (q - p)
}
#[must_use]
pub fn vertex_outside(p: &Vector3<f32>, dim: f32) -> bool {
    p.x <= 0.0 || p.y <= 0.0 || p.z <= 0.0 || p.x > dim || p.y > dim || p.z > dim
}
#[derive(Clone, Copy)]
struct NeighbourGroups {
    lower: &'static [usize],
    higher: &'static [usize],
    corners: &'static [&'static [usize]],
}
const DUAL_CORNER_NEIGHBOURS: [NeighbourGroups; 64] = {
    let mut table = [NeighbourGroups {
        lower: &[],
        higher: &[],
        corners: &[],
    }; 64];
    table[1] = NeighbourGroups {
        lower: &[0],
        higher: &[],
        corners: &[&[2, 3, 6, 7], &[0, 1, 4, 5]],
    };
    table[2] = NeighbourGroups {
        lower: &[0],
        higher: &[],
        corners: &[&[4, 5, 6, 7], &[0, 1, 2, 3]],
    };
    table[4] = NeighbourGroups {
        lower: &[0],
        higher: &[],
        corners: &[&[1, 2, 5, 6], &[0, 3, 4, 7]],
    };
    table[8] = NeighbourGroups {
        lower: &[],
        higher: &[3],
        corners: &[&[0, 1, 4, 5], &[2, 3, 6, 7]],
    };
    table[16] = NeighbourGroups {
        lower: &[],
        higher: &[4],
        corners: &[&[0, 1, 2, 3], &[4, 5, 6, 7]],
    };
    table[32] = NeighbourGroups {
        lower: &[],
        higher: &[1],
        corners: &[&[0, 3, 4, 7], &[1, 2, 5, 6]],
    };
    table[7] = NeighbourGroups {
        lower: &[0, 1, 2, 3, 4, 5, 7],
        higher: &[],
        corners: &[&[6], &[0], &[1], &[2], &[3], &[4], &[5], &[7]],
    };
    table[14] = NeighbourGroups {
        lower: &[0, 1, 2, 3, 4, 7],
        higher: &[6],
        corners: &[&[5], &[0], &[1], &[2], &[3], &[4], &[6], &[7]],
    };
    table[21] = NeighbourGroups {
        lower: &[0, 1, 3, 4, 7],
        higher: &[5, 6],
        corners: &[&[2], &[0], &[1], &[3], &[4], &[5], &[6], &[7]],
    };
    table[28] = NeighbourGroups {
        lower: &[0, 3, 4, 7],
        higher: &[2, 5, 6],
        corners: &[&[1], &[0], &[2], &[3], &[4], &[5], &[6], &[7]],
    };
    table[35] = NeighbourGroups {
        lower: &[0, 3, 4],
        higher: &[1, 2, 5, 6],
        corners: &[&[7], &[0], &[1], &[2], &[3], &[4], &[5], &[6]],
    };
    table[42] = NeighbourGroups {
        lower: &[0, 3],
        higher: &[1, 2, 5, 6, 7],
        corners: &[&[4], &[0], &[1], &[2], &[3], &[5], &[6], &[7]],
    };
    table[49] = NeighbourGroups {
        lower: &[0],
        higher: &[1, 2, 4, 5, 6, 7],
        corners: &[&[3], &[0], &[1], &[2], &[4], &[5], &[6], &[7]],
    };
    table[56] = NeighbourGroups {
        lower: &[],
        higher: &[1, 2, 3, 4, 5, 6, 7],
        corners: &[&[0], &[1], &[2], &[3], &[4], &[5], &[6], &[7]],
    };
    table[3] = NeighbourGroups {
        lower: &[0, 3, 4],
        higher: &[],
        corners: &[&[6, 7], &[0, 1], &[2, 3], &[4, 5]],
    };
    table[5] = NeighbourGroups {
        lower: &[0, 1, 3],
        higher: &[],
        corners: &[&[2, 6], &[0, 4], &[3, 7], &[1, 5]],
    };
    table[6] = NeighbourGroups {
        lower: &[0, 1, 4],
        higher: &[],
        corners: &[&[5, 6], &[0, 3], &[4, 7], &[1, 2]],
    };
    table[10] = NeighbourGroups {
        lower: &[0, 3],
        higher: &[7],
        corners: &[&[4, 5], &[0, 1], &[2, 3], &[6, 7]],
    };
    table[12] = NeighbourGroups {
        lower: &[0, 3],
        higher: &[2],
        corners: &[&[1, 5], &[0, 4], &[3, 7], &[2, 6]],
    };
    table[17] = NeighbourGroups {
        lower: &[0],
        higher: &[4, 7],
        corners: &[&[2, 3], &[0, 1], &[4, 5], &[6, 7]],
    };
    table[20] = NeighbourGroups {
        lower: &[0, 4],
        higher: &[5],
        corners: &[&[1, 2], &[0, 3], &[4, 7], &[5, 6]],
    };
    table[24] = NeighbourGroups {
        lower: &[],
        higher: &[3, 4, 7],
        corners: &[&[0, 1], &[2, 3], &[4, 5], &[6, 7]],
    };
    table[33] = NeighbourGroups {
        lower: &[0],
        higher: &[1, 2],
        corners: &[&[3, 7], &[0, 4], &[1, 5], &[2, 6]],
    };
    table[34] = NeighbourGroups {
        lower: &[0],
        higher: &[1, 5],
        corners: &[&[4, 7], &[0, 3], &[1, 2], &[5, 6]],
    };
    table[40] = NeighbourGroups {
        lower: &[],
        higher: &[1, 2, 3],
        corners: &[&[0, 4], &[3, 7], &[1, 5], &[2, 6]],
    };
    table[48] = NeighbourGroups {
        lower: &[],
        higher: &[1, 4, 5],
        corners: &[&[0, 3], &[4, 7], &[1, 2], &[5, 6]],
    };
    table
};
/// Neighbours are ordered with the owning block first. Priority tests use current block scale.
#[must_use]
pub fn dual_corner_neighbours(
    relative: &Vector3<i32>,
    size: i32,
) -> (Vec<usize>, Vec<usize>, Vec<Vec<usize>>) {
    let mask = u8::from(relative.x == size) << 5
        | u8::from(relative.y == size) << 4
        | u8::from(relative.z == size) << 3
        | u8::from(relative.x == 0) << 2
        | u8::from(relative.y == 0) << 1
        | u8::from(relative.z == 0);
    let groups = &DUAL_CORNER_NEIGHBOURS[usize::from(mask)];
    (
        groups.lower.to_vec(),
        groups.higher.to_vec(),
        groups.corners.iter().map(|group| group.to_vec()).collect(),
    )
}

/// # Errors
/// Returns an error for an invalid mesh scale or inconsistent neighbour/block data, including
/// absent scale storage.
pub fn gather_dual_data(
    tree: &Octree,
    block: &Block,
    scale: i32,
    primal: &Vector3<i32>,
) -> Result<([OccupancyData; 8], [Vector3<f32>; 8])> {
    let mut data = [OccupancyData::default(); 8];
    let mut points = [Vector3::zeros(); 8];
    let size = block.base.size;
    let local = (primal.x % size == 0) || (primal.y % size == 0) || (primal.z % size == 0);
    if !local {
        let half = (1 << scale) as f32 / 2.0;
        for i in 0..8 {
            points[i] = primal.cast::<f32>()
                + Vector3::from(NORMALIZED_DUAL_OFFSETS[i]).cast::<f32>() * half;
            data[i] = block.data_at(&points[i].map(|v| v as i32), scale)?;
        }
        return Ok((data, points));
    }
    let (lower, higher, groups) = dual_corner_neighbours(&(primal - block.base.coord), size);
    for (offset, strict) in lower
        .iter()
        .map(|&i| (i, false))
        .chain(higher.iter().map(|&i| (i, true)))
    {
        let coord = primal + Vector3::from(LOGICAL_DUAL_OFFSETS[offset]);
        if !tree.contains(&coord) {
            return Ok((data, points));
        }
        let Some(id) = fetcher::block(tree, &coord, tree.root)? else {
            return Ok((data, points));
        };
        let neighbour = tree.block(id)?;
        if if strict {
            neighbour.current_scale < block.current_scale
        } else {
            neighbour.current_scale <= block.current_scale
        } {
            return Ok((data, points));
        }
    }
    let Some(own) = groups.first() else {
        return Err(invalid(
            "dual corner has no boundary-neighbour classification",
        ));
    };
    let stride = 1 << scale;
    for &i in own {
        let c = primal + Vector3::from(LOGICAL_DUAL_OFFSETS[i]);
        points[i] = ((c / stride) * stride).cast::<f32>() + Vector3::repeat(stride as f32 * 0.5);
        data[i] = block.data_at(&points[i].map(|v| v as i32), scale)?;
    }
    for group in groups.iter().skip(1) {
        let coord = primal + Vector3::from(LOGICAL_DUAL_OFFSETS[group[0]]);
        let Some(id) = fetcher::block(tree, &coord, tree.root)? else {
            return Err(invalid(
                "dual corner neighbour absent after priority validation",
            ));
        };
        let neighbour = tree.block(id)?;
        let neighbour_scale = neighbour.current_scale.max(scale);
        let stride = 1 << neighbour_scale;
        for &i in group {
            let c = primal + Vector3::from(LOGICAL_DUAL_OFFSETS[i]);
            points[i] =
                ((c / stride) * stride).cast::<f32>() + Vector3::repeat(stride as f32 * 0.5);
            data[i] = neighbour.data_at(&points[i].map(|v| v as i32), neighbour_scale)?;
        }
    }
    Ok((data, points))
}
/// # Errors
/// Returns an error for an invalid mesh scale or inconsistent neighbour/block data, including
/// absent scale storage.
pub fn marching_cube(tree: &Octree, min_desired_scale: i32) -> Result<TriangleMesh> {
    if !(0..=tree.max_block_scale).contains(&min_desired_scale) {
        return Err(invalid("mesh minimum scale outside voxel-block range"));
    }
    let mut mesh = Vec::new();
    for id in OctreeIterator::new(tree, Selection::Blocks) {
        let block = tree.block(id)?;
        let scale = min_desired_scale.clamp(block.current_scale, tree.max_block_scale);
        let stride = 1 << scale;
        let start = block.base.coord;
        let end = (start + Vector3::repeat(block.base.size))
            .map(|vertex_index| vertex_index.min(tree.size - 1));
        let mut x = start.x;
        while x <= end.x {
            let mut y = start.y;
            while y <= end.y {
                let mut z = start.z;
                while z <= end.z {
                    let primal = Vector3::new(x, y, z);
                    z += stride;
                    if (x == end.x || y == end.y || primal.z == end.z)
                        && fetcher::block(tree, &primal, tree.root)?.is_none()
                    {
                        continue;
                    }
                    let (data, points) = gather_dual_data(tree, block, scale, &primal)?;
                    let edges = &TRI_TABLE[occupied_index(&data) as usize];
                    for tri in edges.as_chunks::<3>().0 {
                        if tri[0] < 0 {
                            break;
                        }
                        let mut vertices = [Vector3::zeros(); 3];
                        let mut skip = false;
                        for vertex_index in 0..3 {
                            let [edge_start, edge_end] = EDGE_CORNERS[tri[vertex_index] as usize];
                            vertices[vertex_index] = dual_intersection(
                                &data[edge_start],
                                &data[edge_end],
                                &points[edge_start],
                                &points[edge_end],
                            );
                            if vertex_outside(&vertices[vertex_index], tree.size as f32) {
                                skip = true;
                                break;
                            }
                        }
                        if !skip {
                            mesh.push(MeshFace {
                                vertices,
                                scale: scale as i8,
                            });
                        }
                    }
                }
                y += stride;
            }
            x += stride;
        }
    }
    Ok(mesh)
}
