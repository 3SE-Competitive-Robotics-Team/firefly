use super::*;
use crate::map_input::MapInput;
use firefly_pubsub::odom::OdomMessage;
use nalgebra::{UnitQuaternion, Vector3};

mod mujoco_revisit;

fn odometry(timestamp: f64, x: f64) -> OdomMessage {
    OdomMessage {
        timestamp,
        position_x: x,
        is_initialized: true,
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
#[ignore = "Release timing of repeated fused depth snapshots"]
fn repeated_snapshot_timing() {
    let mapping = crate::config::MappingOptions::default();
    let source = mapping.build(0.4).unwrap();
    let mut fusion = DepthFusion::new(&source, mapping.depth, 0.4).unwrap();
    let now = Instant::now();
    for moving in [false, true] {
        let mut times = Vec::new();
        let mut totals = Vec::new();
        for i in 0..50 {
            let stamp = 1. + f64::from(i) * 0.032 + if moving { 2. } else { 0. };
            let frame = DepthFrame {
                timestamp: stamp,
                received: now,
                image: DepthImage::new(320, 240, vec![2000; 320 * 240]).unwrap(),
                trace: TraceContext::default(),
            };
            let pose =
                Isometry3::translation(if moving { 0.11 * f64::from(i % 2) } else { 0. }, 0., 0.);
            let all = Instant::now();
            fusion.integrate(frame, pose, stamp).unwrap();
            let start = Instant::now();
            let snapshot = std::hint::black_box(fusion.snapshot().unwrap());
            if i > 9 {
                assert_eq!(snapshot.unwrap().last_observed_at, stamp);
                times.push(start.elapsed().as_secs_f64() * 1e6);
                totals.push(all.elapsed().as_secs_f64() * 1e6);
            }
        }
        times.sort_by(f64::total_cmp);
        totals.sort_by(f64::total_cmp);
        println!(
            "moving={moving} snapshot_us p50={} p95={} fuse_and_snapshot_us p50={} p95={}",
            times[20], times[37], totals[20], totals[37]
        );
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

    let delayed = start + Duration::from_millis(1555);
    assert!(timer_due(&mut fading, FADING_PERIOD, delayed));
    assert_eq!(fading, start + Duration::from_secs(2));
    assert!(!timer_due(&mut fading, FADING_PERIOD, delayed));
}

#[test]
fn pairing_drops_old_frames_and_retains_future_frames() {
    let now = Instant::now();
    let poses = VecDeque::from([odometry(1., 0.), odometry(1.04, 2.)]);
    let mut pending = VecDeque::from([
        frame(0.9, 500, now),
        frame(1.01, 500, now),
        frame(1.02, 500, now),
        frame(1.06, 500, now),
    ]);
    let (paired, bracket) = take_pair(&mut pending, &poses, now, 0.05).unwrap();
    let pose = bracket.pose().unwrap();
    assert_eq!(paired.timestamp, 1.02);
    assert!((pose.translation.vector.x - 1.).abs() < 1e-12);
    assert_eq!(pending.len(), 1);
    assert_eq!(pending.front().unwrap().timestamp, 1.06);
    assert!(take_pair(&mut pending, &poses, now, 0.05).is_none());
    assert!(take_pair(&mut pending, &poses, now + Duration::from_secs(2), 0.05).is_none());
    assert!(pending.is_empty());
}

#[test]
fn interpolation_uses_measurement_time_without_extrapolation_or_large_gaps() {
    let a = odometry(1., 0.);
    let mut b = odometry(1.04, 2.);
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
    let q = rotation.quaternion();
    b.quat_x = q.i;
    b.quat_y = q.j;
    b.quat_z = q.k;
    b.quat_w = q.w;
    let history = VecDeque::from([a, b]);
    let pose = PoseBracket::find(&history, 1.02, 0.05)
        .unwrap()
        .pose()
        .unwrap();
    assert!((pose.translation.vector.x - 1.).abs() < 1e-12);
    assert!((pose.rotation.angle() - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
    assert!(PoseBracket::find(&history, 0.99, 0.05).is_none());
    assert!(PoseBracket::find(&history, 1.05, 0.05).is_none());
    assert!(PoseBracket::find(&history, 1.02, 0.01).is_none());
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
#[ignore = "release comparison of full and sampled depth ingestion"]
fn benchmark_depth_ingestion() {
    use std::hint::black_box;
    let mapping = crate::config::MappingOptions::default();
    let source = mapping.build(0.4).unwrap();
    let fusion = DepthFusion::new(&source, mapping.depth, 0.4).unwrap();
    let scale = mapping.depth.k_depth_scaling_factor;
    let message = DepthImageMessage {
        width: IMAGE_WIDTH as u32,
        height: IMAGE_HEIGHT as u32,
        data: std::array::from_fn(|i| match i % 31 {
            0 => f32::NAN,
            1 => 0.,
            _ => (200 + (i * 37 % 7800)) as f32 / 1000.,
        }),
        ..Default::default()
    };
    let mut full = Vec::new();
    let mut sampled = Vec::new();
    for round in 0..10 {
        for packed in [round % 2 == 0, round % 2 != 0] {
            let start = Instant::now();
            for _ in 0..128 {
                let input = black_box(&message);
                let image = if packed {
                    fusion
                        .map
                        .sample_depth(IMAGE_WIDTH, IMAGE_HEIGHT, &input.data, |metres| {
                            depth_units(metres, scale)
                        })
                        .unwrap()
                } else {
                    let units = input.data.iter().map(|&d| depth_units(d, scale)).collect();
                    DepthImage::new(IMAGE_WIDTH, IMAGE_HEIGHT, units).unwrap()
                };
                black_box(image);
            }
            if round > 0 {
                let times = if packed { &mut sampled } else { &mut full };
                times.push(start.elapsed().as_secs_f64() * 1e6 / 128.);
            }
        }
    }
    full.sort_by(f64::total_cmp);
    sampled.sort_by(f64::total_cmp);
    println!(
        "depth ingestion median microseconds/frame: full={} sampled={}",
        full[4], sampled[4]
    );
}

#[test]
fn ground_depth_does_not_observe_air_above_the_camera() {
    let mapping = crate::config::MappingOptions::default();
    let source = mapping.build(0.4).unwrap();
    let options = mapping.depth;
    let mut fusion = DepthFusion::new(&source, options, 0.4).unwrap();
    let body = Isometry3::translation(0.0177, -0.0071, 0.9888);
    let camera = body * options.extrinsic().unwrap();
    let [fx, fy, cx, cy] = options.intrinsics;
    let pixels = (0..IMAGE_WIDTH * IMAGE_HEIGHT)
        .map(|i| {
            let direction = camera.rotation
                * Vector3::new(
                    (i % IMAGE_WIDTH) as f64 / fx - cx / fx,
                    (i / IMAGE_WIDTH) as f64 / fy - cy / fy,
                    1.,
                );
            let depth = -camera.translation.vector.z / direction.z;
            if (0.2..=8.).contains(&depth) {
                depth_units(depth as f32, options.k_depth_scaling_factor)
            } else {
                0
            }
        })
        .collect::<Vec<_>>();
    assert!(pixels.iter().any(|&depth| depth > 0));
    let image = DepthImage::new(IMAGE_WIDTH, IMAGE_HEIGHT, pixels).unwrap();
    for i in 0..4 {
        let stamp = 10. + f64::from(i) * 0.032;
        fusion
            .integrate(
                DepthFrame {
                    timestamp: stamp,
                    received: Instant::now(),
                    image: image.clone(),
                    trace: TraceContext::default(),
                },
                body,
                stamp,
            )
            .unwrap();
    }
    let snapshot = fusion.snapshot().unwrap().unwrap();
    let is_free = |point| {
        matches!(
            snapshot
                .map
                .observed
                .sample(point, snapshot.last_observed_at, Duration::from_secs(1)),
            firefly_map::OnlineState::Free(_)
        )
    };
    assert!(is_free(camera.translation.vector));
    assert!(is_free(Vector3::new(0.5, -0.0071, 0.8)));
    let above = Vector3::new(0.0454, -0.0075, 1.0011);
    assert!(!is_free(above));
    assert!(
        !snapshot
            .map
            .collision
            .is_occupied(source.pos_to_global_index(above))
    );
}

#[test]
fn full_depth_to_collision_path_preserves_native_inflation_and_clears_obstacles() {
    let source = crate::config::MappingOptions {
        resolution: 0.25,
        range: [2.; 3],
        ..Default::default()
    }
    .build(0.25)
    .unwrap();
    let mut fusion = DepthFusion::new(&source, options(), 0.25).unwrap();
    let mut collision = source.clone();
    assert_eq!(collision.origin(), source.origin());
    assert_eq!(collision.global_index([0; 3]), source.global_index([0; 3]));
    let mut input = MapInput::default();
    let now = Instant::now();
    for i in 0..4 {
        let stamp = 1. + f64::from(i) * 0.1;

        fusion
            .integrate(frame(stamp, 500, now), Isometry3::identity(), stamp)
            .unwrap();
        if i == 0 {
            assert!(fusion.snapshot().unwrap().is_none());
        }
    }
    let snapshot = fusion.snapshot().unwrap().unwrap();
    let nav = odometry(1.3, 0.);
    input.observe_local(snapshot).unwrap();
    assert_eq!(
        input.synchronize(&mut collision, &nav, now, Duration::from_secs(1)),
        crate::map_input::MapStatus::Ready
    );
    assert_eq!(
        input.synchronize(&mut collision, &nav, now, Duration::from_secs(1)),
        crate::map_input::MapStatus::Ready
    );
    assert!(collision.is_occupied_inflated(Vector3::new(0.125, 0.125, 0.625)));
    for x in 0..collision.dims()[0] {
        for y in 0..collision.dims()[1] {
            for z in 0..collision.dims()[2] {
                let point = collision.origin()
                    + Vector3::new(x as f64 + 0.5, y as f64 + 0.5, z as f64 + 0.5)
                        * collision.resolution();
                assert_eq!(
                    collision.is_occupied_inflated(point),
                    (point - Vector3::new(0.125, 0.125, 0.625)).amax() <= 0.25,
                    "{point:?}"
                );
            }
        }
    }
    for i in 0..12 {
        let stamp = 1.4 + f64::from(i) * 0.1;

        fusion
            .integrate(frame(stamp, 1250, now), Isometry3::identity(), stamp)
            .unwrap();
    }
    let snapshot = fusion.snapshot().unwrap().unwrap();
    let nav = odometry(2.6, 0.);
    input.observe_local(snapshot).unwrap();
    assert_eq!(
        input.synchronize(&mut collision, &nav, now, Duration::from_secs(1)),
        crate::map_input::MapStatus::Ready
    );
    assert!(!collision.is_occupied_inflated(Vector3::new(0.125, 0.125, 0.625)));
}

#[test]
fn moved_box_and_unseen_free_space_in_a_registered_field() {
    use firefly_map::{Evidence, MapFile, OnlineState};
    let mut map = crate::config::MappingOptions {
        resolution: 0.25,
        range: [2.; 3],
        ..Default::default()
    }
    .build(0.25)
    .unwrap();
    let field: MapFile = "FORMAT firefly-map 1\nRESOLUTION 0.25\nORIGIN 0 0 0\nDIMS 8 8 8\nOCCUPANCY\n0.125 0.125 1.125\n".parse().unwrap();
    field
        .install_fixed(&mut map, &Isometry3::identity())
        .unwrap();
    let box_at = Vector3::new(0.125, 0.125, 0.625);
    let wall = Vector3::new(0.125, 0.125, 1.125);
    let mut fusion = DepthFusion::new(&map, options(), 0.25).unwrap();
    let mut input = MapInput::default();
    let received = Instant::now();
    let mut fuse = |map: &mut firefly_map::GridMap, stamp, units, pose| {
        fusion
            .integrate(frame(stamp, units, received), pose, 100. + stamp)
            .unwrap();
        if let Some(snapshot) = fusion.snapshot().unwrap() {
            input.observe_local(snapshot).unwrap();
            assert_eq!(
                input.synchronize(map, &odometry(stamp, 0.), received, Duration::from_secs(1)),
                crate::map_input::MapStatus::Ready
            );
        }
    };
    let max_age = Duration::from_secs(1);
    for i in 0..5 {
        fuse(&mut map, f64::from(i), 500, Isometry3::identity());
    }
    let before = map.clone();
    let original = before.cell_at(box_at, 4., max_age).unwrap();
    assert!(original.collision && !original.fixed_occupied);
    assert_eq!(original.online, OnlineState::Occupied(4.));

    for i in 5..20 {
        fuse(&mut map, f64::from(i), 1750, Isometry3::identity());
    }
    let empty = map.cell_at(box_at, 19., max_age).unwrap();
    assert!(!empty.collision && !empty.fixed_occupied);
    assert_eq!(empty.online, OnlineState::Free(19.));
    let fixed = map.cell_at(wall, 19., max_age).unwrap();
    assert!(fixed.fixed_occupied && fixed.collision);
    assert_eq!(fixed.online, OnlineState::Free(19.));

    let side = Isometry3::rotation(Vector3::y() * std::f64::consts::FRAC_PI_2);
    fuse(&mut map, 21., 500, side);
    assert_eq!(
        map.cell_at(box_at, 21., max_age).unwrap().online,
        OnlineState::Expired(Evidence::Free(19.))
    );
    assert!(map.cell_at(wall, 21., max_age).unwrap().collision);
    assert_eq!(before.cell_at(box_at, 4., max_age).unwrap(), original);
}

#[test]
fn occupied_lifetime_retracts_only_online_geometry_on_maintenance_or_reobservation() {
    let box_at = Vector3::new(0.125, 0.125, 0.625);
    for revisit in [false, true] {
        let mut map = source();
        map.move_ring_buffer(Vector3::zeros());
        let wall = [3, 0, 2];
        map.set_prior_global_occupied(wall);
        map.inflate_obstacles();
        // A single fresh hit stays below threshold after the saturated prior expires.
        let opts = DepthOptions {
            p_hit: 0.7,
            occupied_max_age: Some(2.),
            fading_time: 0.,
            ..options()
        };
        let mut fusion = DepthFusion::new(&map, opts, 0.25).unwrap();
        let mut input = MapInput::default();
        let now = Instant::now();
        for i in 0..10 {
            let t = f64::from(i) * 0.125;
            fusion
                .integrate(frame(t, 500, now), Isometry3::identity(), t)
                .unwrap();
        }
        let t = 9. * 0.125;
        input
            .observe_local(fusion.snapshot().unwrap().unwrap())
            .unwrap();
        assert_eq!(
            input.synchronize(&mut map, &odometry(t, 0.), now, Duration::from_secs(1)),
            crate::map_input::MapStatus::Ready
        );
        let old = map.clone();
        assert!(old.is_occupied_inflated(box_at));
        let side = Isometry3::rotation(Vector3::y() * std::f64::consts::FRAC_PI_2);
        for (age, expected) in [(2., true), (2.125, false)] {
            let stamp = t + age;
            let pose = if revisit && !expected {
                Isometry3::identity()
            } else {
                side
            };
            fusion
                .integrate(frame(stamp, 500, now), pose, stamp)
                .unwrap();
            if !revisit || expected {
                fusion.map.fade();
            }
            input
                .observe_local(fusion.snapshot().unwrap().unwrap())
                .unwrap();
            assert_eq!(
                input.synchronize(&mut map, &odometry(stamp, 0.), now, Duration::from_secs(1)),
                crate::map_input::MapStatus::Ready
            );
            assert_eq!(map.is_occupied_inflated(box_at), expected);
            assert!(
                map.cell_at(Vector3::new(0.875, 0.125, 0.625), stamp, Duration::ZERO)
                    .unwrap()
                    .fixed_occupied
            );
        }
        assert!(old.is_occupied_inflated(box_at));
    }
}

#[test]
fn empty_images_and_fading_cannot_refresh_map_age() {
    let mut fusion = DepthFusion::new(&source(), options(), 0.25).unwrap();
    let now = Instant::now();

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
    for stamp in [1.2, 1.15] {
        assert!(
            !fusion
                .integrate(frame(stamp, 500, now), Isometry3::identity(), 2.)
                .unwrap()
        );
    }
    fusion.map.fade();
    let snapshot = fusion.snapshot().unwrap().unwrap();
    assert_eq!(snapshot.last_observed_at, 1.1);
    assert_eq!(snapshot.fused_at, now);
    let mut collision = source();
    let mut input = MapInput::default();
    input.observe_local(snapshot).unwrap();
    assert_eq!(
        input.synchronize(
            &mut collision,
            &odometry(1.15, 0.),
            now,
            Duration::from_secs(1)
        ),
        crate::map_input::MapStatus::Waiting
    );
    let nav = odometry(1.2, 0.);
    assert_eq!(
        input.synchronize(
            &mut collision,
            &nav,
            now + Duration::from_millis(1001),
            Duration::from_secs(1)
        ),
        crate::map_input::MapStatus::Unavailable
    );
}

#[test]
fn published_sequence_and_measurement_time_survive_fading_and_window_moves() {
    let options = DepthOptions {
        fading_time: 0.51,
        ..options()
    };
    let mut fusion = DepthFusion::new(&source(), options, 0.25).unwrap();
    let mut last_sequence = 0;
    let now = Instant::now();
    for (i, (x, depth)) in [
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
        fusion
            .integrate(
                frame(timestamp, depth, now),
                Isometry3::translation(x, 0., 0.),
                timestamp,
            )
            .unwrap();
        if i == 0 {
            continue;
        }
        for fade in [false, true] {
            if fade {
                fusion.map.fade();
            }
            let snapshot = fusion.snapshot().unwrap().unwrap();
            assert_eq!(snapshot.sequence, last_sequence + 1);
            assert_eq!(snapshot.last_observed_at, timestamp);
            assert_eq!(snapshot.fused_at, now);
            last_sequence = snapshot.sequence;
        }
    }
}

#[test]
fn local_map_requires_exact_coverage() {
    let mut fusion = DepthFusion::new(&source(), options(), 0.25).unwrap();
    let now = Instant::now();
    let nav = odometry(1.1, 0.);

    fusion
        .integrate(frame(1., 500, now), Isometry3::identity(), 1.)
        .unwrap();
    fusion
        .integrate(frame(1.1, 500, now), Isometry3::identity(), 1.1)
        .unwrap();
    let mut snapshot = fusion.snapshot().unwrap().unwrap();
    snapshot.map.bounds.max[0] = 0.5;
    let mut input = MapInput::default();
    let mut collision = source();
    input.observe_local(snapshot).unwrap();
    assert_ne!(
        input.synchronize(&mut collision, &nav, now, Duration::from_secs(1)),
        crate::map_input::MapStatus::Ready
    );
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

#[test]
fn depth_uses_body_odometry_and_camera_extrinsics_in_order() {
    let mut options = options();
    options.translation = [0.25, 0., 0.];
    options.rotation = [
        0.,
        std::f64::consts::FRAC_1_SQRT_2,
        0.,
        std::f64::consts::FRAC_1_SQRT_2,
    ];
    options.use_depth_filter = false;
    let mut fusion = DepthFusion::new(&source(), options, 0.25).unwrap();
    let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
    let mut odom = odometry(1., 0.4);
    odom.position_y = 0.3;
    odom.position_z = 0.2;
    odom.quat_z = q.quaternion().k;
    odom.quat_w = q.quaternion().w;
    let pose = PoseBracket::find(&VecDeque::from([odom]), 1., 0.05)
        .unwrap()
        .pose()
        .unwrap();
    let now = Instant::now();
    for i in 0..4 {
        let time = 1. + f64::from(i) * 0.1;
        fusion.integrate(frame(time, 500, now), pose, time).unwrap();
    }
    let expected = Vector3::new(0.4, 1.05, 0.2);
    let snapshot = fusion.snapshot().unwrap().unwrap();
    assert_eq!(
        snapshot.map.observed.sample(expected, 1.3, Duration::ZERO),
        firefly_map::OnlineState::Occupied(1.3)
    );
    assert!(
        snapshot
            .map
            .collision
            .is_occupied(source().pos_to_global_index(expected))
    );
}
