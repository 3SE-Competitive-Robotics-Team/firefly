use firefly_ego_map::{DepthImage, GridMap, MappingParameters};
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

fn logit(probability: f64) -> f64 {
    (probability / (1.0 - probability)).ln()
}

fn depth(raw: u16) -> DepthImage {
    DepthImage::new(1, 1, vec![raw]).unwrap()
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
    let (observed, _, _) = map.observed_free_buffer();
    assert!(observed[map.buffer_index(free).unwrap()]);
    assert!(observed[map.buffer_index(camera).unwrap()]);
    assert!(!observed[map.buffer_index(hit).unwrap()]);
    let unseen = Vector3::new(2.25, 0.25, 1.25);
    assert!(!observed[map.buffer_index(unseen).unwrap()]);
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
fn virtual_wall_is_checked_after_window_membership() {
    let mut config = parameters();
    config.enable_virtual_wall = true;
    config.virtual_ground = -0.1;
    config.virtual_ceil = 3.0;
    let mut map = GridMap::new(config).unwrap();
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
    map.fading_callback();
    assert!(
        (map.occupancy_buffer()[map.buffer_index(point).unwrap()] - (logit(0.97) - step)).abs()
            < 1e-12
    );
    for _ in 0..3 {
        map.fading_callback();
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
