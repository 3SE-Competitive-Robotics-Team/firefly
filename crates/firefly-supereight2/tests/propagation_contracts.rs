// SPDX-License-Identifier: BSD-3-Clause
//! Borrowed propagation must match allocation-owning evaluation for every stored level.
use firefly_error::{Error, ErrorKind, Result};
use firefly_supereight2::{
    block::Block,
    data::{OccupancyData, up_prop_max, up_prop_mean, up_prop_min},
    data_field::OccupancyField,
    multires_ofusion_core::propagate_block_to_coarsest_scale,
};
use nalgebra::Vector3;

#[test]
fn propagation_matches_reference_with_unknowns_and_coarser_buffer() {
    for size in [2_usize, 4, 8, 16] {
        for scale in 0..=size.ilog2() as i32 {
            for preserve in [false, true] {
                for seed in 0..8 {
                    let mut actual =
                        Block::new(Vector3::zeros(), size as i32, 0, OccupancyData::default())
                            .unwrap();
                    actual.allocate_down_to(scale).unwrap();
                    for level in scale..=actual.max_scale {
                        for (i, value) in actual
                            .data_at_scale_mut(level)
                            .unwrap()
                            .iter_mut()
                            .enumerate()
                        {
                            *value = OccupancyData {
                                field: OccupancyField {
                                    occupancy: ((i * 19 + seed * 31) % 37) as f32 - 18.,
                                    weight: ((i + seed) % 5) as f32,
                                    observed: (i + seed) % 3 != 0,
                                },
                            };
                        }
                    }
                    if preserve && scale < actual.max_scale {
                        actual.init_buffer(scale + 1).unwrap();
                    }
                    let mut expected = actual.clone();
                    allocating_reference(&mut expected).unwrap();
                    propagate_block_to_coarsest_scale(&mut actual).unwrap();
                    for level in scale..=actual.max_scale {
                        assert_eq!(
                            actual.data_at_scale(level).unwrap(),
                            expected.data_at_scale(level).unwrap()
                        );
                        assert_eq!(
                            actual.min_data_at_scale(level).unwrap(),
                            expected.min_data_at_scale(level).unwrap()
                        );
                        assert_eq!(
                            actual.max_data_at_scale(level).unwrap(),
                            expected.max_data_at_scale(level).unwrap()
                        );
                    }
                }
            }
        }
    }
}

fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}

fn allocating_reference(block: &mut Block) -> Result<()> {
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
