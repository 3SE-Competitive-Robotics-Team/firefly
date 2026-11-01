use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::pinhole_camera::PinholeCamera;
use firefly_rootslam_mapping::{
    global_optimization::{OptimizationOptions, Precision, optimize_global},
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
    for lambda in [1e-6_f64, 1e-2, 10., 1e4] {
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
        for axis in 0..3 {
            jacobian[(4 + axis, axis)] = lambda.sqrt();
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
            precision: Precision::Float64,
            max_iterations: 1,
            initial_damping: lambda,
            minimum_damping: lambda,
            pose_graph_terms: false,
            ..OptimizationOptions::default()
        };
        let result = optimize_global(&map, &calibration, &options).unwrap();
        assert_eq!(result.accepted_steps, 1);
        let difference = result.points[&10] - initial - reference.fixed_rows::<3>(0);
        assert!(
            difference.norm() < 1e-10,
            "lambda={lambda}, difference={difference:?}, svd_gradient={}, returned_gradient={}",
            (jacobian.transpose() * (&jacobian * &reference + &residual)).norm(),
            (jacobian.transpose() * (&jacobian * (result.points[&10] - initial) + &residual))
                .norm()
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
fn proposal_crossing_pinhole_projection_domain_is_rejected_without_committing() {
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
