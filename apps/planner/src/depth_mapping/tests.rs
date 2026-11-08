use super::*;
use crate::map_input::MapInput;
use firefly_pubsub::odom::OdomMessage;
use nalgebra::UnitQuaternion;

fn navigation(timestamp: f64, x: f64, revision: u64) -> NavigationMessage {
    NavigationMessage {
        odom: OdomMessage {
            timestamp,
            position_x: x,
            is_initialized: true,
            ..Default::default()
        },
        rotation: [0., 0., 0., 1.],
        revision,
        ..Default::default()
    }
}

fn options() -> DepthOptions {
    DepthOptions {
        intrinsics: [1., 1., 0., 0.],
        translation: [0.; 3],
        rotation: [0., 0., 0., 1.],
        depth_filter_margin: 0,
        skip_pixel: 1,
        window_margin: 0.5,
        p_hit: 0.9,
        p_min: 0.1,
        p_occ: 0.5,
        p_max: 0.97,
        ..Default::default()
    }
}

fn source() -> firefly_map::GridMap {
    firefly_map::GridMapBuilder::new(0.25, [8; 3])
        .with_obstacles_inflation(0.25)
        .build()
        .unwrap()
}

fn frame(timestamp: f64, units: u16, received: Instant) -> DepthFrame {
    DepthFrame {
        timestamp,
        received,
        image: DepthImage::new(1, 1, vec![units]).unwrap(),
        trace: TraceContext::default(),
    }
}

#[test]
fn mapping_timers_are_independent_and_keep_their_phase() {
    let start = Instant::now();
    let mut occupancy = start + OCCUPANCY_PERIOD;
    let mut fading = start + FADING_PERIOD;
    for millis in [32, 64, 480] {
        assert!(timer_due(
            &mut occupancy,
            OCCUPANCY_PERIOD,
            start + Duration::from_millis(millis)
        ));
    }
    let now = start + Duration::from_millis(500);
    assert!(!timer_due(&mut occupancy, OCCUPANCY_PERIOD, now));
    assert!(timer_due(&mut fading, FADING_PERIOD, now));
    assert_eq!(fading, start + Duration::from_secs(1));

    // A delayed callback skips missed ticks without permanently shifting the cadence.
    let delayed = start + Duration::from_millis(1555);
    assert!(timer_due(&mut fading, FADING_PERIOD, delayed));
    assert_eq!(fading, start + Duration::from_secs(2));
    assert!(!timer_due(&mut fading, FADING_PERIOD, delayed));
}

#[test]
fn pairing_drops_old_frames_and_retains_future_frames() {
    let now = Instant::now();
    let poses = VecDeque::from([navigation(1., 0., 0), navigation(1.04, 2., 0)]);
    let mut pending = VecDeque::from([
        frame(0.9, 500, now),
        frame(1.01, 500, now),
        frame(1.02, 500, now),
        frame(1.06, 500, now),
    ]);
    let (paired, pose) = take_pair(&mut pending, &poses, now, 0.05).unwrap();
    assert_eq!(paired.timestamp, 1.02);
    assert!((pose.translation.vector.x - 1.).abs() < 1e-12);
    assert_eq!(pending.len(), 1);
    assert_eq!(pending.front().unwrap().timestamp, 1.06);
    assert!(take_pair(&mut pending, &poses, now, 0.05).is_none());
    assert!(take_pair(&mut pending, &poses, now + Duration::from_secs(2), 0.05).is_none());
    assert!(pending.is_empty());
}

#[test]
fn interpolation_uses_measurement_time_and_never_crosses_versions_or_gaps() {
    let a = navigation(1., 0., 0);
    let mut b = navigation(1.04, 2., 0);
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
    let q = rotation.quaternion();
    b.odom.quat_x = q.i;
    b.odom.quat_y = q.j;
    b.odom.quat_z = q.k;
    b.odom.quat_w = q.w;
    let mut history = VecDeque::from([a, b]);
    let pose = interpolate_pose(&history, 1.02, 0.05).unwrap();
    assert!((pose.translation.vector.x - 1.).abs() < 1e-12);
    assert!((pose.rotation.angle() - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
    assert!(interpolate_pose(&history, 0.99, 0.05).is_none());
    assert!(interpolate_pose(&history, 1.05, 0.05).is_none());
    assert!(interpolate_pose(&history, 1.02, 0.01).is_none());
    history[1].revision = 1;
    assert!(interpolate_pose(&history, 1.02, 0.05).is_none());
}

#[test]
fn metric_depth_conversion_rounds_even_saturates_and_rejects_missing_values() {
    assert_eq!(depth_units(1.25, 1000.), 1250);
    assert_eq!(depth_units(0.5, 1.), 0);
    assert_eq!(depth_units(1.5, 1.), 2);
    assert_eq!(depth_units(2.5, 1.), 2);
    assert_eq!(depth_units(100., 1000.), u16::MAX);
    for invalid in [0., -1., f32::NAN, f32::INFINITY] {
        assert_eq!(depth_units(invalid, 1000.), 0);
    }
}

#[test]
fn full_depth_to_collision_path_preserves_native_inflation_and_clears_obstacles() {
    let source = source();
    let mut fusion = DepthFusion::new(&source, options(), 0.25).unwrap();
    let mut collision = collision_grid(source, false).unwrap();
    let mut input = MapInput::default();
    let now = Instant::now();
    for i in 0..4 {
        let stamp = 1. + f64::from(i) * 0.1;
        let nav = navigation(stamp, 0., 0);
        fusion.align(&nav).unwrap();
        fusion
            .integrate(frame(stamp, 500, now), Isometry3::identity(), stamp)
            .unwrap();
        if i == 0 {
            assert!(fusion.snapshot().unwrap().is_none());
        }
    }
    let snapshot = fusion.snapshot().unwrap().unwrap();
    let nav = navigation(1.3, 0., 0);
    input.observe_local(snapshot).unwrap();
    input.prepare_region(&mut collision, &nav, true).unwrap();
    assert!(input.ready(&nav, now, Duration::from_secs(1)));
    input.apply(&mut collision, &nav, true).unwrap();
    assert!(collision.is_occupied_inflated(Vector3::new(0.125, 0.125, 0.625)));
    // Every planning voxel agrees with the native inflated grid: no second inflation,
    // no axis/layout conversion error, no extra voxel from floating-boundary rounding.
    for x in 0..collision.dims()[0] {
        for y in 0..collision.dims()[1] {
            for z in 0..collision.dims()[2] {
                let point = collision.origin()
                    + Vector3::new(x as f64 + 0.5, y as f64 + 0.5, z as f64 + 0.5)
                        * collision.resolution();
                assert_eq!(
                    collision.is_occupied_inflated(point),
                    fusion.map.inflated_occupancy_at(point) > 0,
                    "{point:?}"
                );
            }
        }
    }
    for i in 0..12 {
        let stamp = 1.4 + f64::from(i) * 0.1;
        fusion.align(&navigation(stamp, 0., 0)).unwrap();
        fusion
            .integrate(frame(stamp, 1250, now), Isometry3::identity(), stamp)
            .unwrap();
    }
    let snapshot = fusion.snapshot().unwrap().unwrap();
    let nav = navigation(2.6, 0., 0);
    input.observe_local(snapshot).unwrap();
    input.apply(&mut collision, &nav, true).unwrap();
    assert!(!collision.is_occupied_inflated(Vector3::new(0.125, 0.125, 0.625)));
}

#[test]
fn empty_images_and_fading_cannot_refresh_map_age_and_revision_requires_new_depth() {
    let mut fusion = DepthFusion::new(&source(), options(), 0.25).unwrap();
    let now = Instant::now();
    fusion.align(&navigation(1., 0., 0)).unwrap();
    fusion
        .integrate(frame(1., 500, now), Isometry3::identity(), 1.)
        .unwrap();
    fusion
        .integrate(frame(1.1, 500, now), Isometry3::identity(), 1.1)
        .unwrap();
    fusion
        .integrate(
            frame(1.2, 0, now + Duration::from_millis(200)),
            Isometry3::identity(),
            1.2,
        )
        .unwrap();
    fusion.map.fading_callback();
    let snapshot = fusion.snapshot().unwrap().unwrap();
    assert_eq!(snapshot.timestamp, 1.1);
    assert_eq!(snapshot.fused_at, now);
    let mut collision = collision_grid(source(), false).unwrap();
    let mut input = MapInput::default();
    input.observe_local(snapshot).unwrap();
    let nav = navigation(1.2, 0., 0);
    input.prepare_region(&mut collision, &nav, true).unwrap();
    assert!(!input.ready(
        &nav,
        now + Duration::from_millis(1001),
        Duration::from_secs(1)
    ));
    let revised = navigation(1.3, 0., 1);
    fusion.align(&revised).unwrap();
    assert!(fusion.snapshot().unwrap().is_none());
    assert!(!input.ready(&revised, now, Duration::from_secs(1)));
    fusion
        .integrate(frame(1.3, 500, now), Isometry3::identity(), 1.3)
        .unwrap();
    assert!(fusion.snapshot().unwrap().is_none());
    fusion
        .integrate(frame(1.4, 500, now), Isometry3::identity(), 1.4)
        .unwrap();
    assert_eq!(fusion.snapshot().unwrap().unwrap().revision, 1);
}

#[test]
fn local_map_requires_exact_coverage_even_with_external_alignment() {
    let mut fusion = DepthFusion::new(&source(), options(), 0.25).unwrap();
    let now = Instant::now();
    let mut nav = navigation(1.1, 0., 0);
    nav.external_map = 1;
    fusion.align(&nav).unwrap();
    fusion
        .integrate(frame(1., 500, now), Isometry3::identity(), 1.)
        .unwrap();
    fusion
        .integrate(frame(1.1, 500, now), Isometry3::identity(), 1.1)
        .unwrap();
    let mut snapshot = fusion.snapshot().unwrap().unwrap();
    // Partial depth-map coverage must not authorize an uncovered planning window.
    snapshot.bounds.max[0] = 0.5;
    let mut input = MapInput::default();
    let mut collision = collision_grid(source(), false).unwrap();
    input.observe_local(snapshot).unwrap();
    input.prepare_region(&mut collision, &nav, true).unwrap();
    assert!(!input.ready(&nav, now, Duration::from_secs(1)));
    nav.revision = 1;
    assert!(!input.ready(&nav, now, Duration::from_secs(1)));
}

#[test]
fn depth_collision_grid_keeps_inflated_static_priors_after_reset() {
    let mut source = source();
    source.set_prior_occupied([3, 3, 3]);
    let mut collision = collision_grid(source, true).unwrap();
    collision.clear_observations();
    assert!(collision.is_inflated([2, 3, 3]));
    assert!(collision.is_inflated([3, 3, 3]));
    assert!(!collision.is_inflated([1, 3, 3]));
}

#[test]
fn prior_inflation_outside_the_initial_window_survives_window_motion() {
    let mut source = source();
    source.set_prior_occupied([0, 3, 3]);
    let mut collision = collision_grid(source, true).unwrap();
    collision.move_ring_buffer(Vector3::new(0.75, 1., 1.));
    assert!(collision.is_occupied_inflated(Vector3::new(-0.125, 0.875, 0.875)));
}

#[test]
fn calibration_errors_are_rejected_before_starting_workers() {
    let mut invalid = options();
    invalid.rotation = [0.; 4];
    assert!(DepthFusion::new(&source(), invalid, 0.25).is_err());
    invalid = options();
    invalid.max_pose_gap = f64::NAN;
    assert!(DepthFusion::new(&source(), invalid, 0.25).is_err());
    invalid = options();
    invalid.depth_filter_margin = IMAGE_HEIGHT as i32;
    assert!(DepthFusion::new(&source(), invalid, 0.25).is_err());
}
