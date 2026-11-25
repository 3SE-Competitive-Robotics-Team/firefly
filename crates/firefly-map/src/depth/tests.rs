use super::{DepthImage, MappingParameters, grid::GridMap};
use nalgebra::{Isometry3, Vector3};

fn parameters() -> MappingParameters {
    MappingParameters {
        resolution: 1.0,
        local_update_range: [4.0; 3],
        obstacles_inflation: 1.0,
        fx: 2.0,
        fy: 2.0,
        cx: 0.0,
        cy: 0.0,
        depth_filter_margin: 0,
        skip_pixel: 1,
        k_depth_scaling_factor: 1000.0,
        use_depth_filter: false,
        ..MappingParameters::default()
    }
}

#[test]
fn first_point_in_endpoint_voxel_selects_the_path_and_all_points_vote() {
    use super::raycast::FrameRays;

    let left = Vector3::new(3.125, 3.875, 0.5);
    let bottom = Vector3::new(3.875, 3.125, 0.5);
    let address = |cell: Vector3<i32>| usize::try_from(4 * cell.y + cell.x).unwrap();
    let endpoint = address(Vector3::new(3, 3, 0));
    let x_step = 1_u64 << address(Vector3::new(2, 3, 0));
    let y_step = 1_u64 << address(Vector3::new(3, 2, 0));
    for (first, second, expected_step) in [(left, bottom, x_step), (bottom, left, y_step)] {
        let mut rays = FrameRays::new(16, true);
        rays.endpoint(endpoint, first, false);
        rays.endpoint(endpoint, second, true);
        rays.endpoint(endpoint, second, true);
        rays.trace(Vector3::new(0.5, 0.5, 0.5), address);
        assert_eq!(rays.votes(endpoint), (2, 2));
        assert_eq!(rays.visited[0] & (x_step | y_step), expected_step);
    }
}

#[test]
fn rejected_projection_preserves_window_and_allows_retry_with_the_same_stamp() {
    use super::{DepthMapper, FrameUpdate};
    let output = crate::GridMapBuilder::new(1., [8; 3]).build().unwrap();
    let p = MappingParameters {
        fx: 1e-20,
        ..parameters()
    };
    let pose = Isometry3::identity();
    let mut mapper = DepthMapper::new(p, pose, &output).unwrap();
    assert_eq!(
        mapper.integrate(depth(1000), pose, 10., 0.).unwrap(),
        FrameUpdate::Observed
    );
    let before = mapper.snapshot().unwrap().unwrap();
    let invalid = DepthImage::new(2, 1, vec![1000, 1000]).unwrap();
    assert!(
        mapper
            .integrate(invalid, Isometry3::translation(8., 0., 0.), 11., 0.8)
            .is_err()
    );
    let after = mapper.snapshot().unwrap().unwrap();
    assert_eq!(after.bounds, before.bounds);
    assert!(after.collision.shares_data(&before.collision));
    assert!(!mapper.timed_out_at(1.).unwrap());
    assert!(mapper.timed_out_at(1.001).unwrap());
    assert_eq!(
        mapper.integrate(depth(1000), pose, 11., 1.1).unwrap(),
        FrameUpdate::Observed
    );
    assert_eq!(
        mapper.integrate(depth(1000), pose, 11., 1.2).unwrap(),
        FrameUpdate::Stale
    );
}

fn logit(probability: f64) -> f64 {
    (probability / (1.0 - probability)).ln()
}

fn depth(raw: u16) -> DepthImage {
    DepthImage::new(1, 1, vec![raw]).unwrap()
}

#[test]
fn free_only_rays_respect_positive_miss_updates_and_inclusive_inflation() {
    let point = Vector3::new(0.25, 0.25, 1.25);
    for (p_miss, p_occ) in [(0.75, 0.6), (0.35, 0.5)] {
        let mut map = GridMap::new(MappingParameters {
            p_min: 0.5,
            p_miss,
            p_occ,
            ..parameters()
        })
        .unwrap();
        map.depth_pose_callback(depth(3000), Isometry3::translation(0.25, 0.25, 0.25))
            .unwrap();
        map.update_occupancy_callback(1.).unwrap();
        let address = map.buffer_index(point).unwrap();
        assert_eq!(map.occupancy_buffer()[address], logit(p_miss).max(0.));
        assert!(map.inflated_occupancy_at(point) >= 32767);
    }
}

#[test]
fn rays_crossing_later_endpoints_keep_the_terminating_miss() {
    use super::raycast::FrameRays;
    let mut rays = FrameRays::new(8, true);
    let pos = |x| Vector3::new(x, 0.5, 0.5);
    rays.endpoint(5, pos(5.25), true);
    rays.endpoint(3, pos(3.75), true);
    rays.endpoint(3, pos(3.25), true);
    rays.trace(pos(0.25), |cell| usize::try_from(cell.x).unwrap());
    assert_eq!(rays.votes(3), (2, 2));
    assert_eq!(rays.cell_count, 5);
}

#[test]
#[ignore = "release-mode replay with a live reader holding the previous frame"]
fn measured_raycast_replay() {
    let mut map = GridMap::new(MappingParameters {
        resolution: 0.1,
        local_update_range: [9., 9., 5.],
        obstacles_inflation: 0.4,
        fx: 160.,
        fy: 160.,
        cx: 159.5,
        cy: 119.5,
        skip_pixel: 2,
        depth_filter_margin: 2,
        occupied_max_age: Some(1.),
        ..parameters()
    })
    .unwrap();
    let mut timings = Vec::new();
    let mut counts = Vec::new();
    for frame in 0..64 {
        let stamp = f64::from(frame) / 30.;
        let pixels: Vec<u16> = (0..320 * 240)
            .map(|i| {
                let x = i % 320;
                let y = i / 320;
                if (x + y) % 29 == 0 {
                    0
                } else if (x / 32 + y / 24 + frame / 8) % 3 == 0 {
                    1800 + x * 2
                } else {
                    4000 + y * 20 + x * 9
                }
            })
            .map(|raw| u16::try_from(raw).unwrap())
            .collect();
        let depth = map.sample_depth(320, 240, &pixels, |raw| raw).unwrap();
        let pose = Isometry3::translation(-2.0 + stamp, 0.05 * f64::from(frame % 5), -0.6);
        let reader = map.observations();
        map.depth_pose_callback(depth, pose).unwrap();
        let stats = map.update_at(stamp, stamp).unwrap().unwrap();
        std::hint::black_box(reader);
        counts.push(stats.voted_voxels);
        if frame >= 8 {
            timings.push(stats.raycast.as_secs_f64() * 1000.);
        }
    }
    timings.sort_by(f64::total_cmp);
    let hash = map
        .occupancy_buffer()
        .iter()
        .map(|v| v.to_bits())
        .chain(map.inflation_buffer().iter().map(|&v| u64::from(v)))
        .chain(counts.into_iter().map(|n| n as u64))
        .fold(0_u64, |hash, word| hash.wrapping_mul(31).wrapping_add(word));
    println!(
        "raycast n={} p50={:.3}ms p95={:.3}ms map={hash:016x}",
        timings.len(),
        timings[28],
        timings[53]
    );
}

#[test]
fn returning_ray_refreshes_free_evidence_on_its_first_frame() {
    use crate::{Evidence, OnlineState};
    use std::{f64::consts::PI, time::Duration};

    let origin = Vector3::repeat(0.25);
    let front = Isometry3::translation(origin.x, origin.y, origin.z);
    let back = Isometry3::new(origin, Vector3::y() * PI);
    let probe = origin + Vector3::z();
    for gap in [255, 256, 257, 512] {
        for returning_depth in [2000, 3000] {
            let mut map = GridMap::new(parameters()).unwrap();
            for frame in 0..gap {
                let pose = if frame == 0 { front } else { back };
                map.depth_pose_callback(depth(2000), pose).unwrap();
                let time = f64::from(frame);
                map.update_at(time, time).unwrap();
            }
            let time = f64::from(gap);
            assert_eq!(
                map.observations()
                    .unwrap()
                    .sample(probe, time, Duration::ZERO),
                OnlineState::Expired(Evidence::Free(0.))
            );
            map.depth_pose_callback(depth(returning_depth), front)
                .unwrap();
            map.update_at(time, time).unwrap();
            assert_eq!(
                map.observations()
                    .unwrap()
                    .sample(probe, time, Duration::ZERO),
                OnlineState::Free(time),
                "gap {gap}, returning depth {returning_depth}"
            );
        }
    }
}

#[test]
fn dense_rays_to_one_cell_apply_one_probability_update() {
    let p = MappingParameters {
        fx: 1e8,
        fy: 1e8,
        ..parameters()
    };
    let pose = Isometry3::translation(0.25, 0.25, 0.25);
    let mut dense = GridMap::new(p.clone()).unwrap();
    let mut single = GridMap::new(p).unwrap();
    let (width, height) = (257, 256);
    let image = DepthImage::new(width, height, vec![2000; width * height]).unwrap();
    dense.depth_pose_callback(image, pose).unwrap();
    single.depth_pose_callback(depth(2000), pose).unwrap();
    dense.update_at(1., 1.).unwrap();
    single.update_at(1., 1.).unwrap();
    assert_eq!(dense.occupancy_buffer(), single.occupancy_buffer());
}

#[test]
fn sampled_depth_matches_dense_projection_and_occupancy() {
    let (width, height) = (13, 9);
    let addresses: Vec<_> = (0..width * height).collect();
    let pixels: Vec<_> = (0..width * height)
        .map(|i| if i % 11 == 0 { 0 } else { (300 + 7 * i) as u16 })
        .collect();
    for (filter, margin, step) in [(true, 2, 2), (false, 2, 3), (true, 0, 1), (true, 5, 2)] {
        let p = MappingParameters {
            use_depth_filter: filter,
            depth_filter_margin: margin,
            skip_pixel: step,
            ..parameters()
        };
        let mut dense = GridMap::new(p.clone()).unwrap();
        let mut sampled = GridMap::new(p).unwrap();
        for frame in 0..4 {
            let pose = Isometry3::translation(f64::from(frame) * 0.1, 0., 0.);
            let timestamp = 1. + f64::from(frame) * 0.032;
            let full = DepthImage::new(width, height, pixels.clone()).unwrap();
            let mut indices = Vec::new();
            let packed = sampled
                .sample_depth(width, height, &addresses, |i| {
                    indices.push(i);
                    pixels[i]
                })
                .unwrap();
            let border = if filter { margin as usize } else { 0 };
            let expected: Vec<_> = (border..height.saturating_sub(border))
                .step_by(step as usize)
                .flat_map(|y| {
                    (border..width.saturating_sub(border))
                        .step_by(step as usize)
                        .map(move |x| y * width + x)
                })
                .collect();
            assert_eq!(indices, expected);
            dense.depth_odom_callback(full, pose).unwrap();
            sampled.depth_odom_callback(packed, pose).unwrap();
            dense.update_occupancy_callback(timestamp).unwrap();
            sampled.update_occupancy_callback(timestamp).unwrap();
            assert_eq!(sampled.projected_points(), dense.projected_points());
            assert_eq!(sampled.occupancy_buffer(), dense.occupancy_buffer());
            assert_eq!(sampled.inflation_buffer(), dense.inflation_buffer());
            let observed = sampled.observations().unwrap();
            let expected = dense.observations().unwrap();
            let (low, high) = dense.bounds().unwrap();
            for x in low[0]..=high[0] {
                for y in low[1]..=high[1] {
                    for z in low[2]..=high[2] {
                        let p = (Vector3::new(x, y, z).cast::<f64>() + Vector3::repeat(0.5))
                            * dense.resolution();
                        assert_eq!(
                            observed.sample(p, timestamp, std::time::Duration::MAX),
                            expected.sample(p, timestamp, std::time::Duration::MAX)
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn next_depth_reuses_only_a_completed_image() {
    let grid = crate::GridMapBuilder::new(1., [4; 3]).build().unwrap();
    let mut mapper = super::DepthMapper::new(parameters(), Isometry3::identity(), &grid).unwrap();
    let first = mapper.sample_next_depth(3, 3, &[1000; 9], |p| p).unwrap();
    let ptr = first.pixels.as_ptr();
    mapper
        .integrate(first, Isometry3::identity(), 1., 1.)
        .unwrap();
    let second = mapper.sample_next_depth(3, 3, &[1250; 9], |p| p).unwrap();
    assert_eq!(ptr, second.pixels.as_ptr());
    let pending = mapper.sample_next_depth(3, 3, &[1500; 9], |p| p).unwrap();
    assert!(second.pixels.iter().all(|p| *p == 1250));
    assert_ne!(ptr, pending.pixels.as_ptr());
}

#[test]
fn incompatible_depth_sampling_is_rejected_at_ingress() {
    let source = GridMap::new(parameters()).unwrap();
    let mut other = GridMap::new(MappingParameters {
        skip_pixel: 2,
        ..parameters()
    })
    .unwrap();
    let image = source.sample_depth(3, 3, &[1250; 9], |raw| raw).unwrap();
    assert!(
        other
            .depth_pose_callback(image.clone(), Isometry3::identity())
            .is_err()
    );
    assert!(
        other
            .depth_odom_callback(image, Isometry3::identity())
            .is_err()
    );
    assert!(!other.update_occupancy_callback(1.).unwrap());
}

#[test]
fn single_ray_updates_endpoint_and_traversed_cells() {
    let mut map = GridMap::new(parameters()).unwrap();
    for frame in 0..5 {
        map.depth_pose_callback(depth(2000), Isometry3::translation(0.25, 0.25, 0.25))
            .unwrap();
        assert!(
            map.update_occupancy_callback(10.0 + f64::from(frame) * 0.032)
                .unwrap()
        );
    }
    let hit = Vector3::new(0.25, 0.25, 2.25);
    let free = Vector3::new(0.25, 0.25, 1.25);
    let camera = Vector3::new(0.25, 0.25, 0.25);
    // One hit and one traversal vote at the endpoint tie in favor of a hit.
    let expected_hit = (logit(0.12) + 5.0 * logit(0.70)).min(logit(0.97));
    assert!((map.occupancy_buffer()[map.buffer_index(hit).unwrap()] - expected_hit).abs() < 1e-12);
    assert_eq!(
        map.occupancy_buffer()[map.buffer_index(free).unwrap()],
        logit(0.12)
    );
    // RayCaster excludes its end (the camera's cell).
    assert_eq!(
        map.occupancy_buffer()[map.buffer_index(camera).unwrap()],
        logit(0.12)
    );
    assert_eq!(map.occupancy_at(hit), 1);
    assert_eq!(map.inflated_occupancy_at(hit), 32768);
    assert_eq!(map.inflated_occupancy_at(free), 1);
    let observations = map.observations().unwrap();
    let observed = |p| {
        matches!(
            observations.sample(p, 11., std::time::Duration::from_secs(2)),
            crate::OnlineState::Free(_)
        )
    };
    assert!(observed(free));
    assert!(observed(camera));
    assert!(!observed(hit));
    let unseen = Vector3::new(2.25, 0.25, 1.25);
    assert!(!observed(unseen));
}

#[test]
fn filtered_first_frame_is_skipped_but_counts_as_a_timer_update() {
    let mut config = parameters();
    config.use_depth_filter = true;
    let mut map = GridMap::new(config).unwrap();
    map.depth_pose_callback(depth(2000), Isometry3::identity())
        .unwrap();
    assert!(map.update_occupancy_callback(10.0).unwrap());
    assert!(map.initialized());
    assert_eq!(map.projected_points(), &[] as &[Vector3<f64>]);
    assert!(map.occupancy_buffer().iter().all(|&p| p == logit(0.12)));
    map.depth_pose_callback(depth(2000), Isometry3::identity())
        .unwrap();
    assert!(map.update_occupancy_callback(10.032).unwrap());
    assert_eq!(map.projected_points(), &[Vector3::new(0.0, 0.0, 2.0)]);
}

#[test]
fn filter_margin_stride_and_projection_follow_pixel_coordinates() {
    let mut config = parameters();
    config.use_depth_filter = true;
    config.depth_filter_margin = 1;
    config.skip_pixel = 2;
    config.depth_filter_mindist = 0.2;
    config.cx = 2.0;
    config.cy = 2.0;
    let mut map = GridMap::new(config).unwrap();
    let mut pixels = vec![1000; 36];
    pixels[7] = 199;
    pixels[9] = 200;
    let image = DepthImage::new(6, 6, pixels).unwrap();
    for i in 0..2 {
        map.depth_pose_callback(image.clone(), Isometry3::identity())
            .unwrap();
        map.update_occupancy_callback(10.0 + f64::from(i)).unwrap();
    }
    assert_eq!(
        map.projected_points(),
        &[
            Vector3::new(0.1, -0.1, 0.2),
            Vector3::new(-0.5, 0.5, 1.0),
            Vector3::new(0.5, 0.5, 1.0),
        ]
    );
}

#[test]
fn upper_face_cannot_alias_opposite_voxel_and_shift_preserves_evidence() {
    use crate::{Evidence, OnlineState};
    let mut map = GridMap::new(MappingParameters {
        resolution: 0.5,
        local_update_range: [1.5; 3],
        ..parameters()
    })
    .unwrap();
    let age = std::time::Duration::from_secs(1);
    for frame in 0..8 {
        map.depth_pose_callback(depth(1500), Isometry3::identity())
            .unwrap();
        map.update_at(20. + f64::from(frame), f64::from(frame))
            .unwrap();
    }
    let opposite = Vector3::new(0., 0., -1.25);
    assert_eq!(map.occupancy_at(opposite), 0);
    assert_eq!(map.buffer_index(Vector3::new(0., 0., 1.5)), None);
    assert_eq!(
        map.observations().unwrap().sample(opposite, 7., age),
        OnlineState::Unknown
    );
    // Move the upper face down by one cell. The last retained plane contains this hit.
    for i in 8..16 {
        map.depth_pose_callback(depth(750), Isometry3::identity())
            .unwrap();
        map.update_at(20. + f64::from(i), f64::from(i)).unwrap();
    }
    let hit = Vector3::new(0., 0., 0.75);
    map.depth_pose_callback(depth(0), Isometry3::translation(0., 0., -0.5))
        .unwrap();
    map.update_at(40., 20.).unwrap();
    assert_eq!(map.occupancy_at(hit), 1);
    assert_eq!(
        map.observations().unwrap().sample(hit, 20., age),
        OnlineState::Expired(Evidence::Occupied(15.))
    );
}

#[test]
fn outside_endpoint_clears_rays_without_inserting_a_boundary_obstacle() {
    let mut map = GridMap::new(parameters()).unwrap();
    for i in 0..8 {
        map.depth_pose_callback(depth(10000), Isometry3::translation(0.25, 0.25, 0.25))
            .unwrap();
        map.update_occupancy_callback(10.0 + f64::from(i)).unwrap();
    }
    assert!(map.occupancy_buffer().iter().all(|&p| p == logit(0.12)));
    assert!(map.inflation_buffer().iter().all(|&p| p == 0));
}

#[test]
fn cloud_requires_independent_odom_and_does_not_remove_obstacles() {
    let mut map = GridMap::new(parameters()).unwrap();
    let a = Vector3::new(0.25, 0.25, 1.25);
    let b = Vector3::new(1.25, 0.25, 1.25);
    map.cloud_callback(&[a]);
    assert!(!map.initialized());
    map.odom_callback(Isometry3::identity()).unwrap();
    map.cloud_callback(&[a]);
    map.cloud_callback(&[b]);
    assert_eq!(map.occupancy_at(a), 1);
    assert_eq!(map.occupancy_at(b), 1);
    assert_eq!(map.inflated_occupancy_at(a), 32769);
    map.cloud_callback(&[]);
    assert_eq!(map.occupancy_at(a), 1);
}

#[test]
fn overlapping_radius_four_inflation_matches_world_cells_across_ring_seams() {
    let mut map = GridMap::new(MappingParameters {
        local_update_range: [9.; 3],
        obstacles_inflation: 4.,
        ..parameters()
    })
    .unwrap();
    map.odom_callback(Isometry3::identity()).unwrap();
    let centers = [Vector3::new(-1, 0, 1), Vector3::new(3, -2, 0)];
    map.cloud_callback(&centers.map(|p| p.cast::<f64>() + Vector3::repeat(0.25)));
    for z in -8..9 {
        for y in -8..9 {
            for x in -8..9 {
                let id = Vector3::new(x, y, z);
                let expected: i32 = centers
                    .iter()
                    .map(|c| i32::from((id - c).abs().max() <= 4) + 32767 * i32::from(id == *c))
                    .sum();
                assert_eq!(
                    map.inflated_occupancy_at(id.cast::<f64>() + Vector3::repeat(0.25)),
                    expected
                );
            }
        }
    }
    map.depth_pose_callback(depth(0), Isometry3::translation(25., 0., 0.))
        .unwrap();
    map.update_at(10., 10.).unwrap();
    assert!(map.inflation_buffer().iter().all(|&count| count == 0));
}

#[test]
fn virtual_wall_is_checked_after_window_membership() {
    let mut map = GridMap::new(parameters()).unwrap();
    map.reference_wall = Some((-0.1, 3.));
    map.depth_pose_callback(depth(0), Isometry3::identity())
        .unwrap();
    map.update_occupancy_callback(10.0).unwrap();
    assert_eq!(map.occupancy_at(Vector3::new(0.0, 0.0, 3.0)), -1);
    assert_eq!(map.inflated_occupancy_at(Vector3::new(0.0, 0.0, 3.0)), -1);
    assert_eq!(map.occupancy_at(Vector3::new(100.0, 0.0, 3.0)), 0);
    assert_eq!(map.inflated_occupancy_at(Vector3::new(100.0, 0.0, 3.0)), 0);
}

#[test]
fn timeout_is_latched_and_uses_strict_greater_than() {
    let mut map = GridMap::new(parameters()).unwrap();
    assert!(!map.update_occupancy_callback(10.0).unwrap());
    assert!(!map.update_occupancy_callback(20.0).unwrap());
    assert!(!map.odom_depth_timeout());
    map.depth_pose_callback(depth(0), Isometry3::identity())
        .unwrap();
    map.update_occupancy_callback(20.0).unwrap();
    map.update_occupancy_callback(21.0).unwrap();
    assert!(!map.odom_depth_timeout());
    map.update_occupancy_callback(21.001).unwrap();
    assert!(map.odom_depth_timeout());
    map.depth_pose_callback(depth(2000), Isometry3::identity())
        .unwrap();
    map.update_occupancy_callback(21.01).unwrap();
    assert!(map.odom_depth_timeout());
}

#[test]
fn independent_odometry_stops_changing_camera_after_depth_reception() {
    let mut map = GridMap::new(parameters()).unwrap();
    map.depth_pose_callback(depth(2000), Isometry3::identity())
        .unwrap();
    map.odom_callback(Isometry3::translation(10.0, 0.0, 0.0))
        .unwrap();
    map.update_occupancy_callback(10.0).unwrap();
    assert_eq!(map.projected_points(), &[Vector3::new(0.0, 0.0, 2.0)]);
}

#[test]
fn depth_odom_composes_camera_extrinsics() {
    let mut map = GridMap::new(parameters()).unwrap();
    map.extrinsic_callback(Isometry3::translation(0.5, 0.0, 0.0))
        .unwrap();
    map.depth_odom_callback(depth(2000), Isometry3::translation(1.0, 0.0, 0.0))
        .unwrap();
    map.update_occupancy_callback(10.0).unwrap();
    assert_eq!(map.projected_points(), &[Vector3::new(1.5, 0.0, 2.0)]);
}

#[test]
fn fading_removes_inflation_on_the_threshold_crossing() {
    let mut config = parameters();
    config.fading_time = 1.0;
    let mut map = GridMap::new(config).unwrap();
    let point = Vector3::new(0.25, 0.25, 1.25);
    map.odom_callback(Isometry3::identity()).unwrap();
    map.cloud_callback(&[point]);
    let step = (logit(0.97) - logit(0.80)) / 2.0;
    map.fading_callback(None);
    assert!(
        (map.occupancy_buffer()[map.buffer_index(point).unwrap()] - (logit(0.97) - step)).abs()
            < 1e-12
    );
    for _ in 0..3 {
        map.fading_callback(None);
    }
    assert_eq!(map.occupancy_at(point), 0);
    assert_eq!(map.inflated_occupancy_at(point), 0);
}

#[test]
fn invalid_inputs_have_explicit_rust_errors() {
    assert!(GridMap::new(MappingParameters::default()).is_err());
    assert!(DepthImage::new(0, 1, vec![]).is_err());
    assert!(DepthImage::new(2, 2, vec![0; 3]).is_err());
    let mut map = GridMap::new(parameters()).unwrap();
    assert!(map.update_occupancy_callback(f64::NAN).is_err());
    assert!(
        map.depth_pose_callback(depth(1000), Isometry3::translation(f64::INFINITY, 0.0, 0.0))
            .is_err()
    );
}

#[test]
fn occupancy_query_and_inflation_keep_the_cpp_threshold_comparisons() {
    let mut config = parameters();
    config.p_min = 0.5;
    config.p_occ = config.p_hit;
    let mut map = GridMap::new(config).unwrap();
    map.depth_pose_callback(depth(2000), Isometry3::translation(0.25, 0.25, 0.25))
        .unwrap();
    map.update_occupancy_callback(10.0).unwrap();
    let point = Vector3::new(0.25, 0.25, 2.25);
    assert_eq!(
        map.occupancy_buffer()[map.buffer_index(point).unwrap()],
        logit(0.70)
    );
    assert_eq!(map.occupancy_at(point), 0); // getOccupancy uses >.
    assert_eq!(map.inflated_occupancy_at(point), 32768); // Inflation uses >=.
}

#[test]
fn snapshot_matches_live_voxels_after_fusion_fading_and_window_moves() {
    use super::DepthMapper;
    use std::time::Duration;
    let output = crate::GridMapBuilder::new(0.25, [8; 3]).build().unwrap();
    let config = MappingParameters {
        resolution: 0.25,
        local_update_range: [1.5; 3],
        obstacles_inflation: 0.25,
        fading_time: 0.51,
        p_hit: 0.9,
        p_min: 0.1,
        p_occ: 0.5,
        ..parameters()
    };
    let mut mapper = DepthMapper::new(config, Isometry3::identity(), &output).unwrap();
    for (i, (x, units)) in [
        (0., 500),
        (0., 500),
        (0., 500),
        (0., 500),
        (0., 500),
        (0., 1250),
        (0.5, 1250),
        (2.25, 500),
        (-1., 500),
        (-1., 500),
    ]
    .into_iter()
    .enumerate()
    {
        let timestamp = 1. + i as f64 * 0.1;
        mapper
            .integrate(
                depth(units),
                Isometry3::translation(x, 0., 0.),
                timestamp,
                timestamp,
            )
            .unwrap();
        for fade in [false, true] {
            if fade {
                mapper.fade();
            }
            let snapshot = mapper.snapshot().unwrap().unwrap();
            let (low, high) = mapper.grid.bounds().unwrap();

            for z in low[2]..=high[2] {
                for y in low[1]..=high[1] {
                    for x in low[0]..=high[0] {
                        let point = (Vector3::new(x, y, z).cast::<f64>() + Vector3::repeat(0.5))
                            * mapper.grid.resolution();
                        let global = ((point - mapper.collision_origin)
                            / mapper.collision_resolution)
                            .map(|v| v.floor() as i64);
                        assert_eq!(
                            snapshot.collision.is_occupied(global.into()),
                            mapper.grid.inflated_occupancy_at(point) > 0
                        );
                        if let crate::OnlineState::Occupied(_) =
                            snapshot
                                .observed
                                .sample(point, timestamp, Duration::from_secs(1))
                        {
                            assert!(mapper.grid.inflated_occupancy_at(point) > 0);
                        }
                    }
                }
            }
        }
    }
}
