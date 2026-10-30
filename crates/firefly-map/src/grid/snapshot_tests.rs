use super::*;

fn cells(map: &GridMap) -> Vec<(VoxelState, u16)> {
    let dims = map.dims();
    let mut cells = Vec::new();
    for x in 0..dims[0] {
        for y in 0..dims[1] {
            for z in 0..dims[2] {
                cells.push((map.state([x, y, z]), map.inflate_at([x, y, z])));
            }
        }
    }
    cells
}

#[test]
fn snapshot_survives_concurrent_window_reset_prior_and_wall_changes() {
    let mut map = GridMapBuilder::new(0.25, [32, 32, 16])
        .with_obstacles_inflation(0.5)
        .build()
        .unwrap();
    map.set_prior_occupied([12, 12, 4]);
    map.set_state([16, 16, 5], VoxelState::Occupied);
    map.inflate_obstacles();
    let expected = cells(&map);
    let snapshot = map.clone();
    let origin = snapshot.origin();
    let revision = snapshot.revision();
    std::thread::scope(|scope| {
        let (done, wait) = std::sync::mpsc::sync_channel(1);
        scope.spawn(move || {
            assert_eq!(cells(&snapshot), expected);
            wait.recv().unwrap();
            assert_eq!(cells(&snapshot), expected);
            assert_eq!(snapshot.origin(), origin);
            assert_eq!(snapshot.revision(), revision);
            assert!(snapshot.virtual_wall().is_none());
            assert_eq!(snapshot.prior.len(), 1);
        });
        map.set_state([16, 16, 5], VoxelState::Free);
        map.inflate_obstacles();
        map.move_ring_buffer(Vector3::new(5., 4., 2.));
        map.clear_observations();
        map.set_prior_occupied([1, 1, 1]);
        map.set_virtual_wall(VirtualWall {
            ground: -1.,
            ceil: 2.,
        });
        map.inflate_obstacles();
        done.send(()).unwrap();
    });
}

#[test]
fn snapshot_with_pending_updates_can_be_inflated_independently() {
    let mut map = GridMapBuilder::new(0.25, [32, 32, 16]).build().unwrap();
    map.set_state([12, 12, 4], VoxelState::Occupied);
    let mut snapshot = map.clone();
    map.set_state([12, 12, 4], VoxelState::Free);
    map.set_state([14, 12, 4], VoxelState::Occupied);
    map.inflate_obstacles();
    snapshot.inflate_obstacles();
    assert!(snapshot.is_occupied(snapshot.origin() + Vector3::new(3.125, 3.125, 1.125)));
    assert_eq!(snapshot.state([14, 12, 4]), VoxelState::Free);
    assert_eq!(map.state([12, 12, 4]), VoxelState::Free);
    assert_eq!(map.state([14, 12, 4]), VoxelState::Occupied);
    let expected = cells(&snapshot);
    map.clear_observations();
    assert_eq!(cells(&snapshot), expected);
}

#[test]
fn empty_page_skips_match_voxel_scan_across_ring_wraps() {
    let mut map = GridMapBuilder::new(0.25, [64, 64, 16]).build().unwrap();
    for index in [[0, 0, 0], [15, 63, 15], [32, 32, 8], [63, 63, 15]] {
        map.set_state(index, VoxelState::Occupied);
    }
    map.set_prior_occupied([18, 20, 5]);
    map.inflate_obstacles();
    let mut reference = map.clone();
    for center in [
        Vector3::new(4., 7., 2.),
        Vector3::new(-2., 3., 1.),
        Vector3::new(12., 12., 3.),
        Vector3::new(8., 8., 2.),
    ] {
        // Remove the reference's page summaries without changing voxel values: force scalar scans.
        for address in 0..reference.occupancy.len() {
            std::hint::black_box(&mut reference.occupancy[address]);
        }
        for address in 0..reference.inflate.len() {
            std::hint::black_box(&mut reference.inflate[address]);
        }
        map.move_ring_buffer(center);
        reference.move_ring_buffer(center);
        assert_eq!(map.origin(), reference.origin());
        assert_eq!(cells(&map), cells(&reference));
    }
}
