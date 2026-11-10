use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::pinhole_camera::PinholeCamera;
use firefly_rootslam_mapping::{
    global_optimization::{
        DampingCoordinates, LinearSolver, OptimizationOptions, Precision, optimize_global,
    },
    map::{MapKeyframe, MapLandmark, MapObservation, MapSnapshot, ObservationKey},
};
use nalgebra::{DMatrix, DVector, Isometry3, Vector2, Vector3, Vector4};
use std::collections::BTreeMap;

fn scene() -> (MapSnapshot, Calibration<f64>) {
    let camera = GenericCamera::Pinhole(PinholeCamera::new(Vector4::new(300., 300., 0., 0.)));
    let calibration = Calibration {
        intrinsics: vec![camera.clone(), camera],
        t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.2, 0., 0.)],
        ..Calibration::default()
    };
    let mut map = MapSnapshot::default();
    map.keyframes.insert(
        0,
        MapKeyframe {
            id: 0,
            t_w_b: Isometry3::identity(),
            active: false,
            features: vec![BTreeMap::new(); 2],
            images: None,
        },
    );
    let true_point = Vector3::new(0.1, -0.08, 3.);
    let observations = (0..2)
        .map(|camera| {
            let x = true_point.x - 0.2 * camera as f64;
            (
                ObservationKey {
                    keyframe: 0,
                    camera,
                    feature: 10,
                },
                MapObservation {
                    pixel: Vector2::new(
                        300. * x / true_point.z,
                        300. * true_point.y / true_point.z,
                    ),
                },
            )
        })
        .collect();
    map.landmarks.insert(
        10,
        MapLandmark {
            id: 10,
            host: 0,
            position: Vector3::new(0.2, 0.1, 4.),
            observations,
        },
    );
    (map, calibration)
}

#[test]
fn one_global_landmark_step_matches_independent_augmented_svd() {
    let (map, calibration) = scene();
    let initial = map.landmarks[&10].position;
    for (linear_solver, damping_coordinates, lambda) in [
        (LinearSolver::Rootba, DampingCoordinates::Physical),
        (LinearSolver::Rootba, DampingCoordinates::Scaled),
        (LinearSolver::LsqrReference, DampingCoordinates::Physical),
    ]
    .into_iter()
    .flat_map(|(solver, coordinates)| {
        [1e-6_f64, 1e-2, 10., 1e4].map(|lambda| (solver, coordinates, lambda))
    }) {
        let mut jacobian = DMatrix::zeros(7, 3);
        let mut residual = DVector::zeros(7);
        for camera in 0..2 {
            let x = initial.x - 0.2 * camera as f64;
            let y = initial.y;
            let z = initial.z;
            let pixel = map.landmarks[&10].observations[&ObservationKey {
                keyframe: 0,
                camera,
                feature: 10,
            }]
                .pixel;
            residual[camera * 2] = 300. * x / z - pixel.x;
            residual[camera * 2 + 1] = 300. * y / z - pixel.y;
            jacobian[(camera * 2, 0)] = 300. / z;
            jacobian[(camera * 2, 2)] = -300. * x / (z * z);
            jacobian[(camera * 2 + 1, 1)] = 300. / z;
            jacobian[(camera * 2 + 1, 2)] = -300. * y / (z * z);
        }
        let mut scaling = Vector3::repeat(1.0);
        for axis in 0..3 {
            if linear_solver == LinearSolver::Rootba {
                scaling[axis] = 1.0 / (1e-5 + jacobian.column(axis).norm());
                jacobian.column_mut(axis).scale_mut(scaling[axis]);
            }
            jacobian[(4 + axis, axis)] = lambda.sqrt()
                * if damping_coordinates == DampingCoordinates::Physical {
                    scaling[axis]
                } else {
                    1.0
                };
        }
        let svd = jacobian.clone().svd(true, true);
        let mut reference = svd.solve(&(-&residual), 1e-14).unwrap();
        // Refine against the original augmented rows and certify the reference gradient.
        for _ in 0..3 {
            reference += svd
                .solve(&(-(&jacobian * &reference + &residual)), 1e-14)
                .unwrap();
        }
        assert!((jacobian.transpose() * (&jacobian * &reference + &residual)).norm() < 1e-10);
        let options = OptimizationOptions {
            linear_solver,
            damping_coordinates,
            precision: Precision::Float64,
            max_iterations: 1,
            initial_damping: lambda,
            minimum_damping: lambda,
            pose_graph_terms: false,
            ..OptimizationOptions::default()
        };
        let result = optimize_global(&map, &calibration, &options).unwrap();
        assert_eq!(result.accepted_steps, 1);
        let returned = (result.points[&10] - initial).component_div(&scaling);
        let difference =
            result.points[&10] - initial - reference.fixed_rows::<3>(0).component_mul(&scaling);
        assert!(
            difference.norm() < 1e-10,
            "lambda={lambda}, difference={difference:?}, svd_gradient={}, returned_gradient={}",
            (jacobian.transpose() * (&jacobian * &reference + &residual)).norm(),
            (jacobian.transpose() * (&jacobian * returned + &residual)).norm()
        );
    }
}

#[test]
fn positive_damping_that_rounds_to_zero_in_storage_precision_is_rejected() {
    let (map, calibration) = scene();
    let options = OptimizationOptions {
        precision: Precision::Float32,
        initial_damping: 1e-300,
        minimum_damping: 1e-300,
        max_iterations: 1,
        ..OptimizationOptions::default()
    };
    assert!(
        optimize_global(&map, &calibration, &options).is_err(),
        "f32 must not silently solve the undamped problem requested with positive damping"
    );
}

#[test]
fn nonfinite_measurement_is_not_returned_as_a_successful_optimization() {
    let (mut map, calibration) = scene();
    map.landmarks
        .get_mut(&10)
        .unwrap()
        .observations
        .values_mut()
        .next()
        .unwrap()
        .pixel
        .x = f64::NAN;
    assert!(optimize_global(&map, &calibration, &OptimizationOptions::default()).is_err());
}

#[test]
fn cost_increasing_step_is_rejected_without_committing() {
    let (mut map, calibration) = scene();
    for observation in map
        .landmarks
        .get_mut(&10)
        .unwrap()
        .observations
        .values_mut()
    {
        observation.pixel *= 3.;
    }
    // At z=4, the undamped linearized stereo depth step toward z=1 is -12.
    // The finite damped step still crosses the cameras' positive-depth domain.
    let options = OptimizationOptions {
        precision: Precision::Float64,
        max_iterations: 1,
        initial_damping: 1e-6,
        minimum_damping: 1e-6,
        pose_graph_terms: false,
        ..OptimizationOptions::default()
    };
    let result = optimize_global(&map, &calibration, &options).unwrap();
    assert_eq!(result.accepted_steps, 0);
    assert_eq!(result.points[&10], map.landmarks[&10].position);
    assert_eq!(result.initial_cost, result.final_cost);
    assert!(!result.converged);
}

#[test]
fn invalid_initial_camera_domain_is_an_error_instead_of_a_dropped_factor() {
    let (mut map, calibration) = scene();
    map.landmarks.get_mut(&10).unwrap().position.z = -4.0;
    assert!(optimize_global(&map, &calibration, &OptimizationOptions::default()).is_err());
}

#[test]
fn singular_projection_is_an_error_instead_of_a_dropped_factor() {
    let (mut map, calibration) = scene();
    map.landmarks.get_mut(&10).unwrap().position.z = 0.0;
    assert!(optimize_global(&map, &calibration, &OptimizationOptions::default()).is_err());
}

#[test]
fn single_view_landmarks_are_deferred_and_follow_their_host_correction() {
    use firefly_rootslam_mapping::map::{MapPolicy, PersistentMap};
    use firefly_rootslam_mapping::pose_graph::{EdgeKind, PoseGraphEdge};
    let (mut map, calibration) = scene();
    let mut moving = map.keyframes[&0].clone();
    moving.id = 1;
    moving.t_w_b = Isometry3::translation(0.5, 0.0, 0.0);
    map.keyframes.insert(1, moving);
    map.edges.push(PoseGraphEdge {
        from: 0,
        to: 1,
        measurement: Isometry3::translation(1.0, 0.0, 0.0),
        kind: EdgeKind::Temporal,
        sqrt_information: nalgebra::SMatrix::identity(),
    });
    let expected = optimize_global(&map, &calibration, &OptimizationOptions::default()).unwrap();
    let position = Vector3::new(0.5, 0.0, 1e-5);
    map.landmarks.insert(
        20,
        MapLandmark {
            id: 20,
            host: 1,
            position,
            observations: BTreeMap::from([(
                ObservationKey {
                    keyframe: 1,
                    camera: 0,
                    feature: 20,
                },
                MapObservation {
                    pixel: Vector2::new(100.0, 50.0),
                },
            )]),
        },
    );
    let result = optimize_global(&map, &calibration, &OptimizationOptions::default()).unwrap();
    assert!(
        !result.points.contains_key(&20),
        "single-view depth is unobservable"
    );
    assert_eq!(result.initial_cost, expected.initial_cost);
    assert_eq!(result.final_cost, expected.final_cost);
    assert_eq!(result.poses, expected.poses);
    assert!(
        result.points.contains_key(&10),
        "same-rig stereo remains observable"
    );
    let mut invalid = map.clone();
    let landmark = invalid.landmarks.get_mut(&20).unwrap();
    let (mut key, observation) = landmark.observations.pop_first().unwrap();
    key.camera = calibration.intrinsics.len();
    landmark.observations.insert(key, observation);
    assert!(optimize_global(&invalid, &calibration, &OptimizationOptions::default()).is_err());
    let before = map.keyframes[&1].t_w_b.inverse() * nalgebra::Point3::from(position);
    let mut persistent = PersistentMap::new(calibration, MapPolicy::paper_thresholds()).unwrap();
    persistent.snapshot = map;
    assert!(persistent.apply_updates(result.revision, &result.poses, &result.points));
    let after = persistent.snapshot.keyframes[&1].t_w_b.inverse()
        * nalgebra::Point3::from(persistent.snapshot.landmarks[&20].position);
    assert!((before - after).norm() < 1e-12);
    assert_eq!(persistent.snapshot.landmarks[&20].observations.len(), 1);
}

#[test]
fn inverse_right_jacobian_matches_group_difference_around_series_boundary() {
    use firefly_rootslam_mapping::pose_graph::{right_jacobian_inverse, se3_exp, se3_log};
    use nalgebra::SVector;
    let axis = Vector3::new(1., 2., -0.5).normalize();
    for angle in [0.001, 0.0099, 0.0101, 0.03, 0.1, 0.4999, 0.5001, 1., 3.] {
        let tangent =
            SVector::<f64, 6>::new(1., -2., 0.3, angle * axis.x, angle * axis.y, angle * axis.z);
        // Differentiate the group operation in f64 at exactly the quantized f32 input.
        let quantized = tangent.cast::<f32>();
        let point = quantized.cast::<f64>();
        let transform = se3_exp(&point);
        let analytic = right_jacobian_inverse(&quantized).cast::<f64>();
        let exact = right_jacobian_inverse(&point);
        for column in 0..6 {
            let mut delta = SVector::<f64, 6>::zeros();
            delta[column] = 1e-6;
            let numerical = (se3_log(&(transform * se3_exp(&delta)))
                - se3_log(&(transform * se3_exp(&-delta))))
                / (2e-6);
            assert!(
                (exact.column(column) - numerical).amax() < 1e-7,
                "double angle={angle} col={column}"
            );
            assert!(
                (analytic.column(column) - numerical).amax() < 1e-5,
                "float angle={angle} col={column} difference={:?}",
                analytic.column(column) - numerical
            );
        }
    }
}
