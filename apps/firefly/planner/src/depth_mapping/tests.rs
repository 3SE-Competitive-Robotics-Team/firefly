use super::*;
use crate::map_input::MapInput;
use firefly_pubsub::odom::OdomMessage;
use nalgebra::{UnitQuaternion, Vector3};

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
    let pose = interpolate_pose(&history, 1.02, 0.05).unwrap();
    assert!((pose.translation.vector.x - 1.).abs() < 1e-12);
    assert!((pose.rotation.angle() - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
    assert!(interpolate_pose(&history, 0.99, 0.05).is_none());
    assert!(interpolate_pose(&history, 1.05, 0.05).is_none());
    assert!(interpolate_pose(&history, 1.02, 0.01).is_none());
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
    assert!(snapshot.observed.is_free(camera.translation.vector));
    assert!(snapshot.observed.is_free(Vector3::new(0.5, -0.0071, 0.8)));
    let above = Vector3::new(0.0454, -0.0075, 1.0011);
    assert!(!snapshot.observed.is_free(above));
    assert_eq!(fusion.map.inflated_occupancy_at(above), 0);
    let (native, _, _) = fusion.map.observed_free_buffer();
    assert!(!native[fusion.map.buffer_index(above).unwrap()]);
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
    let mut collision = collision_grid(&source).unwrap();
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
    input.prepare_region(&mut collision, &nav, true);
    assert_eq!(
        input.status(&nav, now, Duration::from_secs(1)),
        crate::map_input::MapStatus::Ready
    );
    input.apply(&mut collision, &nav, true).unwrap();
    assert!(collision.is_occupied_inflated(Vector3::new(0.125, 0.125, 0.625)));
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

        fusion
            .integrate(frame(stamp, 1250, now), Isometry3::identity(), stamp)
            .unwrap();
    }
    let snapshot = fusion.snapshot().unwrap().unwrap();
    let nav = odometry(2.6, 0.);
    input.observe_local(snapshot).unwrap();
    input.apply(&mut collision, &nav, true).unwrap();
    assert!(!collision.is_occupied_inflated(Vector3::new(0.125, 0.125, 0.625)));
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
    fusion.map.fading_callback();
    let snapshot = fusion.snapshot().unwrap().unwrap();
    assert_eq!(snapshot.timestamp, 1.1);
    assert_eq!(snapshot.fused_at, now);
    let mut collision = collision_grid(&source()).unwrap();
    let mut input = MapInput::default();
    input.observe_local(snapshot).unwrap();
    let nav = odometry(1.2, 0.);
    input.prepare_region(&mut collision, &nav, true);
    assert_eq!(
        input.status(
            &nav,
            now + Duration::from_millis(1001),
            Duration::from_secs(1)
        ),
        crate::map_input::MapStatus::Unavailable
    );
}

#[test]
fn cached_masks_match_live_voxels_after_fusion_fading_and_window_moves() {
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
                fusion.map.fading_callback();
            }
            let snapshot = fusion.snapshot().unwrap().unwrap();
            assert_eq!(snapshot.sequence, last_sequence + 1);
            assert_eq!(snapshot.timestamp, timestamp);
            assert_eq!(snapshot.fused_at, now);
            last_sequence = snapshot.sequence;
            let (low, high) = fusion.map.bounds().unwrap();
            let (free, _, _) = fusion.map.observed_free_buffer();
            for z in low[2]..=high[2] {
                for y in low[1]..=high[1] {
                    for x in low[0]..=high[0] {
                        let point = (Vector3::new(x, y, z).cast::<f64>() + Vector3::repeat(0.5))
                            * fusion.map.resolution();
                        let global = ((point - fusion.collision_origin)
                            / fusion.collision_resolution)
                            .map(|v| v.floor() as i64);
                        assert_eq!(
                            snapshot.collision.is_occupied(global.into()),
                            fusion.map.inflated_occupancy_at(point) > 0
                        );
                        assert_eq!(
                            snapshot.observed.is_free(point),
                            free[fusion.map.buffer_index(point).unwrap()]
                        );
                    }
                }
            }
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
    snapshot.bounds.max[0] = 0.5;
    let mut input = MapInput::default();
    let mut collision = collision_grid(&source()).unwrap();
    input.observe_local(snapshot).unwrap();
    input.prepare_region(&mut collision, &nav, true);
    assert_ne!(
        input.status(&nav, now, Duration::from_secs(1)),
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
    let pose = interpolate_pose(&VecDeque::from([odom]), 1., 0.05).unwrap();
    let now = Instant::now();
    for i in 0..4 {
        let time = 1. + f64::from(i) * 0.1;
        fusion.integrate(frame(time, 500, now), pose, time).unwrap();
    }
    let expected = Vector3::new(0.4, 1.05, 0.2);
    assert!((fusion.map.projected_points()[0] - expected).norm() < 1e-12);
    assert!(fusion.map.inflated_occupancy_at(expected) > 0);
}
