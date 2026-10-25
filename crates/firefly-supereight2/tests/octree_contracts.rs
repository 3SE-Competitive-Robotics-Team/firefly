// SPDX-License-Identifier: BSD-3-Clause
//! Analytic fields exercise actual cross-block interpolation, refinement and pruning.
use firefly_supereight2::{
    allocator,
    block::Block,
    data::{OccupancyData, up_prop_mean},
    data_field::OccupancyField,
    octant::Octant,
    octree::Octree,
    visitor,
};
use nalgebra::Vector3;
fn value(x: f32) -> OccupancyData {
    OccupancyData {
        field: OccupancyField {
            occupancy: x,
            weight: 1.0,
            observed: true,
        },
    }
}
fn tree_with_field(scale: i32, field: impl Fn(Vector3<f32>) -> f32) -> Octree {
    let mut tree = Octree::new(32, 8).unwrap();
    let root = tree.root;
    let stride = 1 << scale;
    for z in (0..32).step_by(8) {
        for y in (0..32).step_by(8) {
            for x in (0..32).step_by(8) {
                let id = allocator::block(&Vector3::new(x, y, z), &mut tree, root).unwrap();
                let block = tree.block_mut(id).unwrap();
                block.allocate_down_to(scale).unwrap();
                for k in (0..8).step_by(stride as usize) {
                    for j in (0..8).step_by(stride as usize) {
                        for i in (0..8).step_by(stride as usize) {
                            let coord = Vector3::new(x + i, y + j, z + k);
                            let index = block.voxel_index(&coord, scale).unwrap();
                            block.data_at_scale_mut(scale).unwrap()[index] = value(field(
                                coord.cast::<f32>() + Vector3::repeat(stride as f32 * 0.5),
                            ));
                        }
                    }
                }
            }
        }
    }
    tree
}
#[test]
fn affine_field_interpolation_and_gradients_are_exact_across_block_boundaries_and_scales() {
    let gradient = Vector3::new(0.03125, -0.0625, 0.125);
    for scale in [0, 1] {
        let tree = tree_with_field(scale, |p| gradient.dot(&p) - 0.5);
        for x in [7.25, 7.75, 8.25, 15.75, 16.25, 23.75] {
            let p = Vector3::new(x, 12.25, 14.75);
            let (actual, returned) = visitor::interp(&tree, &p, scale, OccupancyData::valid, |d| {
                d.field.occupancy
            })
            .unwrap()
            .unwrap();
            assert_eq!(returned, scale);
            assert!((actual - (gradient.dot(&p) - 0.5)).abs() < 1e-6);
            let (actual, returned) = visitor::grad(&tree, &p, scale, OccupancyData::valid, |d| {
                d.field.occupancy
            })
            .unwrap()
            .unwrap();
            assert_eq!(returned, scale);
            assert!(
                (actual - gradient).amax() < 1e-6,
                "{p:?}, scale={scale}, grad={actual:?}"
            );
        }
    }
}
#[test]
fn invalid_neighbour_prevents_invented_interpolation_and_nonfinite_queries_are_errors() {
    let mut tree = tree_with_field(0, |p| p.x * 0.125);
    let root = tree.root;
    let coord = Vector3::new(8, 12, 14);
    let id = allocator::block(&coord, &mut tree, root).unwrap();
    let block = tree.block_mut(id).unwrap();
    let index = block.voxel_index(&coord, 0).unwrap();
    block.data_at_scale_mut(0).unwrap()[index] = OccupancyData::default();
    assert!(
        visitor::interp(
            &tree,
            &Vector3::new(8.1, 12.5, 14.5),
            0,
            OccupancyData::valid,
            |d| d.field.occupancy
        )
        .unwrap()
        .is_none()
    );
    assert!(visitor::interp_field(&tree, &Vector3::new(f32::NAN, 12.0, 14.0), 0).is_err());
    assert!(visitor::grad_field(&tree, &Vector3::new(32.0, 12.0, 14.0), 0).is_err());
}
#[test]
fn observed_leaf_refinement_inherits_data_and_pruning_invalidates_all_descendant_ids() {
    let mut tree = Octree::new(32, 8).unwrap();
    let root = tree.root;
    let initial = value(-2.0);
    tree.node_mut(root).unwrap().max_data = initial;
    tree.node_mut(root).unwrap().min_data = initial;
    tree.allocate_children(root).unwrap();
    let children = tree.node(root).unwrap().children;
    for id in children.into_iter().flatten() {
        assert_eq!(tree.node(id).unwrap().data(), initial);
        tree.allocate_children(id).unwrap();
    }
    let old: Vec<_> = tree
        .octants
        .iter()
        .enumerate()
        .filter_map(|(id, o)| (id != root && o.is_some()).then_some(id))
        .collect();
    assert!(
        old.iter()
            .any(|id| matches!(tree.get(*id).unwrap(), Octant::Block(_)))
    );
    tree.delete_children(root).unwrap();
    assert_eq!(tree.node(root).unwrap().base.child_mask, 0);
    for &id in &old {
        assert!(tree.get(id).is_err());
    }
    let (new, created) = tree.allocate(root, 0).unwrap();
    assert!(created);
    assert!(new > *old.last().unwrap());
    assert!(tree.get(root).is_ok());
}
#[test]
fn block_buffer_requires_twenty_integrations_and_ninety_percent_observation_support() {
    let mut block = Block::new(Vector3::zeros(), 8, 0, value(-1.0)).unwrap();
    block.allocate_down_to(1).unwrap();
    block.init_curr_count().unwrap();
    block.curr_observed_count = Some(64);
    block.init_buffer(0).unwrap();
    block.buffer_observed_count = Some(459); // 459/8 < 0.9*64
    block.buffer_integr_count = Some(20);
    assert!(!block.switch_data().unwrap());
    block.buffer_observed_count = Some(461);
    block.buffer_integr_count = Some(19);
    assert!(!block.switch_data().unwrap());
    block.buffer_integr_count = Some(20);
    assert!(block.switch_data().unwrap());
    assert_eq!(block.current_scale, 0);
    assert_eq!(block.data_at_scale(0).unwrap().len(), 512);
    let children = [value(-1.0); 8];
    let mut parent = OccupancyData::default();
    assert_eq!(up_prop_mean(&mut parent, &children), 8);
    assert_eq!(parent.field.occupancy, -1.0);
}
