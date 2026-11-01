use firefly_basalt::{
    optical_flow::{AffineCompact2, OpticalFlowResult},
    vi_estimator::{landmark_database::Keypoint, local_map::LocalMapSnapshot},
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::pinhole_camera::PinholeCamera;
use firefly_calibration::camera::stereographic_param::StereographicParam;
use firefly_calibration::observation::TimeCamId;
use firefly_rootslam_mapping::{
    loop_closure::{LoopDetector, LoopOptions},
    map::{
        MapFeature, MapKeyframe, MapLandmark, MapObservation, MapPolicy, MapSnapshot,
        ObservationKey, PersistentMap,
    },
};
use nalgebra::{Isometry3, SMatrix, Vector2, Vector3, Vector4};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

fn calibration() -> Calibration<f64> {
    let camera = GenericCamera::Pinhole(PinholeCamera::new(Vector4::new(300., 300., 320., 240.)));
    Calibration {
        intrinsics: vec![camera.clone(), camera],
        t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.2, 0., 0.)],
        ..Calibration::default()
    }
}
fn descriptor(id: usize) -> [u64; 4] {
    let mut state = id as u64 + 1;
    std::array::from_fn(|_| {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state = state.wrapping_mul(0x9e37_79b9_7f4a_7c15);
        state
    })
}
fn scene() -> Vec<Vector3<f64>> {
    (0..20)
        .map(|i| {
            Vector3::new(
                f64::from(i % 5) * 0.25 - 0.5,
                f64::from(i / 5) * 0.2 - 0.3,
                2.5 + f64::from(i % 3) * 0.3,
            )
        })
        .collect()
}
fn pixel(point: &Vector3<f64>, body_x: f64, camera: usize) -> Vector2<f64> {
    Vector2::new(
        300. * (point.x - body_x - camera as f64 * 0.2) / point.z + 320.,
        300. * point.y / point.z + 240.,
    )
}
fn closure_map() -> MapSnapshot {
    let mut map = MapSnapshot::default();
    let points = scene();
    for (time, body_x, estimated_x, offset) in
        [(0, 0., 0., 0_usize), (3_000_000_000, 0.02, 0.27, 100)]
    {
        let features = (0..2)
            .map(|camera| {
                points
                    .iter()
                    .enumerate()
                    .map(|(id, p)| {
                        (
                            id + offset,
                            MapFeature {
                                pixel: pixel(p, body_x, camera),
                                descriptor: Some(descriptor(id)),
                                landmark: Some((id + offset) as u64),
                            },
                        )
                    })
                    .collect()
            })
            .collect();
        map.keyframes.insert(
            time,
            MapKeyframe {
                id: time,
                t_w_b: Isometry3::translation(estimated_x, 0., 0.),
                active: time != 0,
                features,
                images: None,
            },
        );
        for (id, p) in points.iter().enumerate() {
            let observations = (0..2)
                .map(|camera| {
                    (
                        ObservationKey {
                            keyframe: time,
                            camera,
                            feature: id + offset,
                        },
                        MapObservation {
                            pixel: pixel(p, body_x, camera),
                        },
                    )
                })
                .collect();
            map.landmarks.insert(
                (id + offset) as u64,
                MapLandmark {
                    id: (id + offset) as u64,
                    host: time,
                    position: p + Vector3::new(estimated_x - body_x, 0., 0.),
                    observations,
                },
            );
        }
    }
    map
}
#[test]
fn hashbow_to_generalized_p3p_verifies_real_multicamera_loop() {
    let map = closure_map();
    let mut detector = LoopDetector::new(calibration(), LoopOptions::default());
    assert!(detector.process(&map, 0).unwrap().candidate.is_none());
    let result = detector.process(&map, 3_000_000_000).unwrap();
    let closure = result
        .candidate
        .expect("known revisit must pass geometric verification");
    assert!((closure.verified_t_w_b.translation.vector - Vector3::new(0.02, 0., 0.)).norm() < 1e-6);
    assert!(closure.verified_t_w_b.rotation.angle() < 1e-6);
    assert!((closure.drift_m - 0.25).abs() < 1e-6);
    assert_eq!(closure.landmark_pairs.len(), 20);
    assert_eq!(closure.associations.len(), 40);
    let mut persistent = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    persistent.snapshot = map;
    assert!(persistent.apply_updates(
        0,
        &BTreeMap::from([(0, Isometry3::translation(0.25, 0., 0.))]),
        &BTreeMap::new()
    ));
    persistent.apply_loop_associations(&closure);
    assert_eq!(persistent.snapshot.landmarks.len(), 20);
    for point in persistent.snapshot.landmarks.values() {
        assert_eq!(point.observations.len(), 4);
    }
    assert_eq!(persistent.covisibility()[&(0, 3_000_000_000)], 20);
}
#[test]
fn stale_loop_geometry_is_revalidated_before_map_mutation() {
    let initial = closure_map();
    let mut detector = LoopDetector::new(calibration(), LoopOptions::default());
    let _ = detector.process(&initial, 0).unwrap();
    let closure = detector
        .process(&initial, 3_000_000_000)
        .unwrap()
        .candidate
        .unwrap();
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    map.snapshot = initial;
    for point in map.snapshot.landmarks.values_mut().filter(|p| p.host == 0) {
        point.position.y += 1.;
    }
    map.snapshot.revision += 1;
    assert!(map.revalidate_loop(&closure).is_none());
    map.apply_loop_associations(&closure);
    assert_eq!(map.snapshot.landmarks.len(), 40);
    for point in map.snapshot.landmarks.values() {
        assert_eq!(point.observations.len(), 2);
    }
}

#[test]
fn corrected_loop_replaces_existing_measurement_with_same_endpoints() {
    let initial = closure_map();
    let mut detector = LoopDetector::new(calibration(), LoopOptions::default());
    let _ = detector.process(&initial, 0).unwrap();
    let closure = detector
        .process(&initial, 3_000_000_000)
        .unwrap()
        .candidate
        .unwrap();
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    map.snapshot = initial;
    let mut existing = closure.edge.clone();
    existing.measurement = Isometry3::translation(0.4, 0., 0.);
    map.snapshot.edges.push(existing);
    assert!(map.apply_loop_correction(
        0,
        &BTreeMap::from([(0, Isometry3::translation(0.25, 0., 0.))]),
        &BTreeMap::new(),
        &closure,
    ));
    let matching: Vec<_> = map
        .snapshot
        .edges
        .iter()
        .filter(|edge| {
            edge.kind == closure.edge.kind
                && edge.from == closure.edge.from
                && edge.to == closure.edge.to
        })
        .collect();
    assert_eq!(matching.len(), 1);
    assert!(
        (matching[0].measurement.translation.vector - closure.edge.measurement.translation.vector)
            .norm()
            < 1e-12
    );
    assert!(
        matching[0]
            .measurement
            .rotation
            .angle_to(&closure.edge.measurement.rotation)
            < 1e-12
    );
}
fn local(translation: f64) -> LocalMapSnapshot {
    let p = Vector3::new(0.1, 0.2, 3.);
    let direction = StereographicParam::project(&Vector4::new(p.x, p.y, p.z, 1.), None);
    let mut point = Keypoint::new(direction, 1. / p.norm(), TimeCamId::new(0, 0));
    point.obs.insert(TimeCamId::new(0, 0), pixel(&p, 0., 0));
    let flow = Arc::new(OpticalFlowResult {
        t_ns: 0,
        observations: vec![BTreeMap::from([(
            0,
            AffineCompact2 {
                translation: pixel(&p, 0., 0).map(|v| v as f32),
                ..Default::default()
            },
        )])],
        ..Default::default()
    });
    LocalMapSnapshot {
        t_ns: 0,
        poses: BTreeMap::from([(0, Isometry3::translation(translation, 0., 0.))]),
        keyframes: BTreeSet::from([0]),
        landmarks: BTreeMap::from([(0, point)]),
        frames: BTreeMap::from([(0, flow)]),
        rejected_landmarks: BTreeSet::new(),
        rejected_observations: BTreeSet::new(),
    }
}

#[test]
fn local_outlier_removal_retracts_active_observation_but_retains_marginalized_geometry() {
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    let mut input = local(0.);
    input.keyframes.insert(1);
    input.poses.insert(1, Isometry3::identity());
    input.frames.insert(1, input.frames[&0].clone());
    let pixel = input.landmarks[&0].obs[&TimeCamId::new(0, 0)];
    input
        .landmarks
        .get_mut(&0)
        .unwrap()
        .obs
        .insert(TimeCamId::new(1, 0), pixel);
    map.ingest(&input);
    let previous = input
        .landmarks
        .iter()
        .map(|(&id, point)| (id, point.obs.keys().copied().collect()))
        .collect();
    input
        .landmarks
        .get_mut(&0)
        .unwrap()
        .obs
        .remove(&TimeCamId::new(0, 0));
    input.record_rejections(&previous);
    map.ingest(&input);
    let rejected = ObservationKey {
        keyframe: 0,
        camera: 0,
        feature: 0,
    };
    assert!(
        !map.snapshot.landmarks[&0]
            .observations
            .contains_key(&rejected)
    );
    assert!(
        map.snapshot.keyframes[&0].features[0][&0]
            .landmark
            .is_none()
    );
    assert_eq!(map.snapshot.landmarks[&0].observations.len(), 1);
    input.keyframes.clear();
    input.landmarks.clear();
    input.poses.clear();
    input.frames.clear();
    input.rejected_observations.clear();
    map.ingest(&input);
    assert_eq!(
        map.snapshot.landmarks[&0].observations.len(),
        1,
        "marginalization retains map geometry"
    );
}

#[test]
fn local_outlier_point_removal_does_not_leave_a_persistent_active_point() {
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    let mut input = local(0.);
    map.ingest(&input);
    let previous = input
        .landmarks
        .iter()
        .map(|(&id, point)| (id, point.obs.keys().copied().collect()))
        .collect();
    input.landmarks.clear();
    input.record_rejections(&previous);
    map.ingest(&input);
    assert!(map.snapshot.landmarks.is_empty());
    assert!(
        map.snapshot.keyframes[&0].features[0][&0]
            .landmark
            .is_none()
    );
}

#[test]
fn lost_landmark_marginalization_keeps_geometry_with_an_active_host() {
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    let mut input = local(0.);
    map.ingest(&input);
    input.landmarks.clear();
    map.ingest(&input);
    assert_eq!(map.snapshot.landmarks.len(), 1);
    assert_eq!(map.snapshot.keyframes[&0].features[0][&0].landmark, Some(0));
}

#[test]
fn local_triangulation_reassigns_single_observation_owner() {
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    let input = local(0.);
    let mut untriangulated = input.clone();
    untriangulated.landmarks.clear();
    map.ingest(&untriangulated);
    let key = ObservationKey {
        keyframe: 0,
        camera: 0,
        feature: 0,
    };
    let pixel = map.snapshot.keyframes[&0].features[0][&0].pixel;
    // A verified loop can associate a tracked corner before local triangulation.
    map.snapshot.landmarks.insert(
        9,
        MapLandmark {
            id: 9,
            host: 0,
            position: Vector3::new(0.1, 0.2, 3.),
            observations: BTreeMap::from([(key, MapObservation { pixel })]),
        },
    );
    map.snapshot.keyframes.get_mut(&0).unwrap().features[0]
        .get_mut(&0)
        .unwrap()
        .landmark = Some(9);
    map.ingest(&input);
    assert_eq!(map.snapshot.keyframes[&0].features[0][&0].landmark, Some(0));
    assert!(map.snapshot.landmarks[&0].observations.contains_key(&key));
    assert!(!map.snapshot.landmarks[&9].observations.contains_key(&key));
}
#[test]
fn stale_results_cannot_overwrite_local_geometry_or_a_newer_optimizer() {
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    map.ingest(&local(0.));
    let source = map.snapshot.revision;
    map.ingest(&local(1.));
    let expected = map.snapshot.landmarks[&0].position;
    assert!(map.apply_updates(
        source,
        &BTreeMap::from([(0, Isometry3::translation(5., 0., 0.))]),
        &BTreeMap::from([(0, Vector3::zeros())])
    ));
    assert_eq!(map.snapshot.keyframes[&0].t_w_b.translation.vector.x, 1.);
    assert_eq!(map.snapshot.landmarks[&0].position, expected);
    map.finish_local();
    let source = map.snapshot.revision;
    assert!(map.apply_updates(
        source,
        &BTreeMap::from([(0, Isometry3::translation(2., 0., 0.))]),
        &BTreeMap::from([(0, Vector3::new(2.1, 0.2, 3.))])
    ));
    assert!(!map.apply_updates(
        source,
        &BTreeMap::from([(0, Isometry3::identity())]),
        &BTreeMap::new()
    ));
    assert_eq!(map.snapshot.keyframes[&0].t_w_b.translation.vector.x, 2.);
    assert_eq!(
        map.snapshot.landmarks[&0].position,
        Vector3::new(2.1, 0.2, 3.)
    );
}
#[test]
fn pose_graph_correction_moves_hosted_geometry_but_preserves_active_window() {
    let mut map = PersistentMap::new(
        calibration(),
        MapPolicy::paper_thresholds(SMatrix::identity()),
    );
    map.snapshot = closure_map();
    let original = map.snapshot.landmarks[&0].position;
    assert!(map.apply_updates(
        0,
        &BTreeMap::from([
            (0, Isometry3::translation(1., 0., 0.)),
            (3_000_000_000, Isometry3::translation(8., 0., 0.))
        ]),
        &BTreeMap::new()
    ));
    assert!((map.snapshot.landmarks[&0].position - original - Vector3::x()).norm() < 1e-12);
    assert_eq!(
        map.snapshot.keyframes[&3_000_000_000]
            .t_w_b
            .translation
            .vector
            .x,
        0.27
    );
}
#[test]
fn culling_preserves_chained_full_trajectory_anchors() {
    let mut policy = MapPolicy::paper_thresholds(SMatrix::identity());
    policy.max_keyframes = 2;
    policy.cull_batch = 1;
    let mut map = PersistentMap::new(calibration(), policy);
    let frame = |id| MapKeyframe {
        id,
        t_w_b: Isometry3::translation(id as f64, 0., 0.),
        active: false,
        features: Vec::new(),
        images: None,
    };
    for id in 0..3 {
        map.snapshot.keyframes.insert(id, frame(id));
    }
    let empty = LocalMapSnapshot {
        t_ns: 0,
        poses: BTreeMap::new(),
        keyframes: BTreeSet::new(),
        landmarks: BTreeMap::new(),
        frames: BTreeMap::new(),
        rejected_landmarks: BTreeSet::new(),
        rejected_observations: BTreeSet::new(),
    };
    map.ingest(&empty);
    assert!(!map.snapshot.keyframes.contains_key(&0));
    assert_eq!(map.resolved_pose(0).unwrap().translation.vector.x, 0.);
    map.snapshot.keyframes.insert(3, frame(3));
    map.ingest(&empty);
    assert!(!map.snapshot.keyframes.contains_key(&1));
    let revision = map.snapshot.revision;
    assert!(map.apply_updates(
        revision,
        &BTreeMap::from([(2, Isometry3::translation(7., 0., 0.))]),
        &BTreeMap::new()
    ));
    assert_eq!(map.resolved_pose(0).unwrap().translation.vector.x, 5.);
    assert_eq!(map.resolved_pose(1).unwrap().translation.vector.x, 6.);
}
#[test]
fn fixed_rig_stereo_landmarks_continue_until_their_own_steps_converge() {
    use firefly_rootslam_mapping::global_optimization::{
        OptimizationOptions, Precision, optimize_global,
    };
    let mut map = closure_map();
    map.keyframes.retain(|&id, _| id == 0);
    map.landmarks.retain(|_, point| point.host == 0);
    for point in map.landmarks.values_mut() {
        point.position += Vector3::new(0.2, -0.1, 0.4);
    }
    let options = OptimizationOptions {
        precision: Precision::Float64,
        max_iterations: 30,
        step_tolerance: 1e-10,
        relative_cost_tolerance: 1e-10,
        pose_graph_terms: false,
        ..Default::default()
    };
    let result = optimize_global(&map, &calibration(), &options).unwrap();
    assert!(
        result.accepted_steps > 1,
        "landmark-only problem incorrectly stopped after one pose-empty step"
    );
    assert!(
        result.final_cost < result.initial_cost * 1e-8,
        "{} -> {}",
        result.initial_cost,
        result.final_cost
    );
}
#[test]
fn pose_graph_residual_and_jacobian_use_the_papers_right_relative_error() {
    use firefly_rootslam_mapping::pose_graph::{linearize_pose_graph_edge, se3_log};
    let first = Isometry3::new(Vector3::new(2., -1., 0.5), Vector3::new(0.2, -0.1, 0.3));
    let second = Isometry3::new(Vector3::new(-0.3, 1.5, 2.), Vector3::new(-0.2, 0.4, 0.1));
    let measurement = Isometry3::new(Vector3::new(-1., 0.2, 1.4), Vector3::new(0.1, 0.2, -0.15));
    let error = |a: &Isometry3<f64>, b: &Isometry3<f64>| {
        se3_log(&((a.inverse() * b).inverse() * measurement))
    };
    let (residual, ja, jb) = linearize_pose_graph_edge(&first, &second, &measurement);
    assert!((residual - error(&first, &second)).norm() < 1e-12);
    let perturb = |pose: &Isometry3<f64>, axis: usize, step: f64| {
        let mut result = *pose;
        if axis < 3 {
            result.translation.vector[axis] += step;
        } else {
            let mut rotation = Vector3::zeros();
            rotation[axis - 3] = 2. * step;
            result.rotation =
                nalgebra::UnitQuaternion::from_scaled_axis(rotation) * result.rotation;
        }
        result
    };
    for axis in 0..6 {
        let h = 1e-7;
        let numeric = (error(&perturb(&first, axis, h), &second)
            - error(&perturb(&first, axis, -h), &second))
            / (2. * h);
        assert!(
            (numeric - ja.column(axis)).norm() < 1e-7,
            "first axis {axis}"
        );
        let numeric = (error(&first, &perturb(&second, axis, h))
            - error(&first, &perturb(&second, axis, -h)))
            / (2. * h);
        assert!(
            (numeric - jb.column(axis)).norm() < 1e-7,
            "second axis {axis}"
        );
    }
}
