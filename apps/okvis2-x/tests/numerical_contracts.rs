// SPDX-License-Identifier: BSD-3-Clause

//! Rust-native mathematical contracts: independent geometry, differential checks and analytic motion.
use firefly_okvis_x::{
    cameras::{
        camera_base::{CameraBase, ProjectionStatus},
        distortion_base::DistortionBase,
        equidistant_distortion::EquidistantDistortion,
        eucm_camera::EucmCamera,
        no_distortion::NoDistortion,
        pinhole_camera::PinholeCamera,
        radial_tangential_distortion::RadialTangentialDistortion,
        radial_tangential_distortion8::RadialTangentialDistortion8,
    },
    ceres::{
        depth_error::{DepthJacobians, OneSidedDepthError},
        imu_error::{Covariance, ImuError, IntegrationError, Jacobian},
        pose_local_parameterization::PoseManifold,
        speed_and_bias_error::SpeedAndBiasError,
        speed_and_bias_parameter_block::SpeedAndBiasParameterBlock,
        vi_graph::{
            GraphFactor, GraphProblem, GraphSolver, ManifoldKind, Parameter, RobustLoss,
            RustGraphSolver, SolverOptions,
        },
    },
    config::parameters::ImuParameters,
    kinematics::transformation::{Transformation, delta_q},
    sensors::measurements::{ImuMeasurement, ImuMeasurementDeque, ImuSensorReadings},
};
use jiff::Timestamp;
use nalgebra::{Matrix2, SMatrix, SVector, Vector2, Vector3, Vector4};
use proptest::prelude::*;
use std::{cell::RefCell, rc::Rc};

proptest! {
    #![proptest_config(ProptestConfig::with_failure_persistence(
        proptest::test_runner::FileFailurePersistence::WithSource("proptest-regressions")
    ))]
    #[test]
    fn pose_composition_and_inverse_match_homogeneous_matrix_geometry(
        a in prop::array::uniform6(-1.5f64..1.5), b in prop::array::uniform6(-1.5f64..1.5),
        point in prop::array::uniform3(-10.0f64..10.0)
    ) {
        let make = |v:[f64;6]|Transformation::new(Vector3::new(v[0],v[1],v[2]),delta_q(&Vector3::new(v[3],v[4],v[5])));
        let a = make(a); let b = make(b); let p = Vector4::new(point[0],point[1],point[2],1.0);
        let combined = a.compose(&b);
        prop_assert!((combined.transform_homogeneous(&p)-a.matrix()*b.matrix()*p).amax()<2e-14);
        prop_assert!((a.inverse().transform_homogeneous(&a.transform_homogeneous(&p))-p).amax()<2e-14);
        // Directions (homogeneous w=0) must not receive a translation.
        let direction=Vector4::new(point[0],point[1],point[2],0.0);
        let transformed=a.transform_homogeneous(&direction);
        prop_assert!((transformed.fixed_rows::<3>(0)-a.rotate_vector(&Vector3::from(point))).norm()<1e-14);
        prop_assert_eq!(transformed.w,0.0);
    }

    #[test]
    fn pinhole_obeys_analytic_perspective_and_positive_scale(
        x in -0.5f64..0.5, y in -0.4f64..0.4, z in 0.1f64..50.0, scale in 0.01f64..100.0
    ) {
        let camera=PinholeCamera::new((640,480),(320.0,300.0,319.5,239.5),NoDistortion,0);
        let p=Vector3::new(x*z,y*z,z); let mut pixel=Vector2::zeros();
        prop_assert_eq!(camera.project(&p,&mut pixel),ProjectionStatus::Successful);
        prop_assert!((pixel-Vector2::new(320.0*x+319.5,300.0*y+239.5)).amax()<2e-13);
        let mut scaled=Vector2::zeros();
        prop_assert_eq!(camera.project(&(p*scale),&mut scaled),ProjectionStatus::Successful);
        prop_assert!((pixel-scaled).amax()<2e-13);
    }
}

#[test]
fn camera_projection_jacobians_and_pixel_roundtrips_cover_all_models() {
    let cameras: Vec<Box<dyn CameraBase>> = vec![
        Box::new(PinholeCamera::<NoDistortion>::test_object()),
        Box::new(PinholeCamera::<EquidistantDistortion>::test_object()),
        Box::new(PinholeCamera::<RadialTangentialDistortion>::test_object()),
        Box::new(PinholeCamera::<RadialTangentialDistortion8>::test_object()),
        Box::new(EucmCamera::test_object()),
    ];
    for (model, camera) in cameras.iter().enumerate() {
        for x in [-0.25, 0.0, 0.23] {
            for y in [-0.19, 0.0, 0.17] {
                for z in [0.5, 2.0, 10.0] {
                    let point = Vector3::new(x * z, y * z, z);
                    let mut pixel = Vector2::zeros();
                    let mut j = SMatrix::<f64, 2, 3>::zeros();
                    assert_eq!(
                        camera.project_with_jacobians(&point, &mut pixel, &mut j, None),
                        ProjectionStatus::Successful,
                        "model={model}"
                    );
                    for axis in 0..3 {
                        let step = 1e-5 * z;
                        let mut plus = point;
                        let mut minus = point;
                        plus[axis] += step;
                        minus[axis] -= step;
                        let mut p_plus = Vector2::zeros();
                        let mut p_minus = Vector2::zeros();
                        assert_eq!(
                            camera.project(&plus, &mut p_plus),
                            ProjectionStatus::Successful
                        );
                        assert_eq!(
                            camera.project(&minus, &mut p_minus),
                            ProjectionStatus::Successful
                        );
                        let numeric = (p_plus - p_minus) / (2.0 * step);
                        // Central difference error combines O(h^2) curvature and O(epsilon/h) rounding.
                        assert!(
                            (numeric - j.column(axis)).amax() < 2e-6,
                            "model={model}, axis={axis}, point={point:?}"
                        );
                    }
                    let mut ray = Vector3::zeros();
                    assert!(camera.back_project(&pixel, &mut ray));
                    assert!(
                        (ray.normalize() - point.normalize()).norm() < 2e-9,
                        "model={model}, point={point:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn distortion_jacobians_and_inverse_chain_agree_at_axis_and_off_axis() {
    let models: Vec<Box<dyn DistortionBase>> = vec![
        Box::new(EquidistantDistortion::test_object()),
        Box::new(RadialTangentialDistortion::test_object()),
        Box::new(RadialTangentialDistortion8::test_object()),
    ];
    for (index, model) in models.iter().enumerate() {
        for point in [
            Vector2::zeros(),
            Vector2::new(1e-8, -1e-8),
            Vector2::new(0.25, -0.18),
            Vector2::new(-0.4, 0.3),
        ] {
            let mut distorted = Vector2::zeros();
            let mut forward = Matrix2::zeros();
            assert!(model.distort_with_jacobians(&point, &mut distorted, &mut forward, None));
            let mut recovered = Vector2::zeros();
            let mut inverse = Matrix2::zeros();
            assert!(model.undistort_with_jacobian(&distorted, &mut recovered, &mut inverse));
            assert!(
                (recovered - point).norm() < 2e-9,
                "model={index}, point={point:?}"
            );
            assert!(
                (inverse * forward - Matrix2::identity()).amax() < 2e-7,
                "model={index}"
            );
            for axis in 0..2 {
                let mut plus = point;
                let mut minus = point;
                plus[axis] += 1e-6;
                minus[axis] -= 1e-6;
                let mut p = Vector2::zeros();
                let mut m = Vector2::zeros();
                assert!(model.distort(&plus, &mut p));
                assert!(model.distort(&minus, &mut m));
                assert!(
                    ((p - m) / 2e-6 - forward.column(axis)).amax() < 1e-7,
                    "model={index}"
                );
            }
        }
    }
}

#[test]
fn pose_manifold_differential_matches_central_difference_and_inverse_tangent() {
    for angle in [
        Vector3::zeros(),
        Vector3::new(0.3, -0.5, 0.2),
        Vector3::new(3.0, 0.1, -0.1),
    ] {
        let pose = Transformation::new(Vector3::new(1.0, -2.0, 0.3), delta_q(&angle));
        let x = pose.coeffs();
        let j = PoseManifold::plus_jacobian(x);
        assert!(
            (PoseManifold::minus_jacobian(x) * j - SMatrix::<f64, 6, 6>::identity()).amax() < 1e-14
        );
        for axis in 0..6 {
            let mut step = SVector::<f64, 6>::zeros();
            step[axis] = 1e-6;
            let plus = PoseManifold::plus(x, &step);
            let minus = PoseManifold::plus(x, &-step);
            assert!(((plus - minus) / 2e-6 - j.column(axis)).amax() < 3e-10);
            assert!((PoseManifold::minus(&plus, x) - step).amax() < 1e-12);
        }
    }
}

fn time(milliseconds: i64) -> Timestamp {
    Timestamp::from_millisecond(milliseconds).unwrap()
}
fn imu_parameters() -> ImuParameters {
    ImuParameters {
        r#use: true,
        t_bs: Transformation::identity(),
        a_max: 100.0,
        g_max: 20.0,
        sigma_g_c: 0.002,
        sigma_bg: 0.01,
        sigma_a_c: 0.02,
        sigma_ba: 0.02,
        sigma_gw_c: 0.0002,
        sigma_aw_c: 0.002,
        g0: Vector3::zeros(),
        a0: Vector3::zeros(),
        g: 9.81,
        s_a: Vector3::repeat(1.0),
    }
}
fn imu_samples(gyro: Vector3<f64>, acc: Vector3<f64>) -> ImuMeasurementDeque {
    (0..=100)
        .map(|i| ImuMeasurement::new(time(i * 10), ImuSensorReadings::new(gyro, acc), -1))
        .collect()
}

#[test]
fn imu_residual_only_preserves_cost_and_cache_across_bias_updates() {
    for count in [21, 50, 101] {
        let samples = (0..count)
            .map(|i| {
                let t = i as f64 * 0.01;
                ImuMeasurement::new(
                    time(i * 10),
                    ImuSensorReadings::new(
                        Vector3::new(0.1 + t, -0.2 * t, 0.3 - t),
                        Vector3::new(t, 0.5 - t, 9.81 + 0.2 * t),
                    ),
                    -1,
                )
            })
            .collect();
        let mut cost_factor =
            ImuError::new(samples, imu_parameters(), time(0), time((count - 1) * 10)).unwrap();
        let mut full_factor = cost_factor.clone();
        let first = Transformation::new(
            Vector3::new(0.1, -0.2, 0.3),
            delta_q(&Vector3::new(0.2, -0.1, 0.3)),
        );
        let second = Transformation::new(
            Vector3::new(-0.4, 0.2, 0.1),
            delta_q(&Vector3::new(-0.1, 0.2, 0.1)),
        );
        let initial =
            SVector::<f64, 9>::from_row_slice(&[0.1, -0.2, 0.3, 0.01, -0.02, 0.03, 0.1, -0.2, 0.3]);
        let end = -initial;
        for (trial, bias_delta) in [0.0, 0.0001, 0.0005, 0.0001].into_iter().enumerate() {
            let mut start = initial;
            start[3] += bias_delta;
            start[6] += trial as f64 * 0.01;
            let cost = cost_factor
                .evaluate_residual(first.coeffs(), &start, second.coeffs(), &end)
                .unwrap();
            let full = full_factor
                .evaluate(first.coeffs(), &start, second.coeffs(), &end)
                .unwrap();
            assert!(cost.norm() > 0.0 && cost.iter().all(|v| v.is_finite()));
            assert_eq!(cost, full.residual, "count={count}, trial={trial}");
            assert_eq!(cost_factor.redo, full_factor.redo);
            assert_eq!(cost_factor.redo_counter, full_factor.redo_counter);
            let expected_redos = if count < 50 { trial.max(1) } else { 1 };
            assert_eq!(cost_factor.redo_counter, expected_redos);
            assert_eq!(cost_factor.redo, count >= 50 && trial >= 2);
            assert_eq!(cost_factor.information, full_factor.information);
            assert_eq!(
                cost_factor.square_root_information,
                full_factor.square_root_information
            );
            let after = cost_factor
                .evaluate(first.coeffs(), &start, second.coeffs(), &end)
                .unwrap();
            assert_eq!(after.residual, full.residual);
            assert_eq!(after.pose0, full.pose0);
            assert_eq!(after.pose1, full.pose1);
            assert_eq!(after.speed_bias0, full.speed_bias0);
            assert_eq!(after.speed_bias1, full.speed_bias1);
            assert_eq!(after.pose0_minimal, full.pose0_minimal);
            assert_eq!(after.pose1_minimal, full.pose1_minimal);
            assert_eq!(after.speed_bias0_minimal, full.speed_bias0_minimal);
            assert_eq!(after.speed_bias1_minimal, full.speed_bias1_minimal);
        }
    }
}

#[test]
fn imu_residual_only_preserves_integration_and_covariance_errors() {
    let pose = Transformation::identity();
    let sb = SVector::<f64, 9>::zeros();
    let mut factor = ImuError::new(
        imu_samples(Vector3::zeros(), Vector3::z() * 9.81),
        imu_parameters(),
        time(0),
        time(1000),
    )
    .unwrap();
    let mut missing = factor.clone();
    missing.imu_measurements.clear();
    assert_eq!(
        missing.evaluate_residual(pose.coeffs(), &sb, pose.coeffs(), &sb),
        Err(IntegrationError::MissingMeasurements)
    );
    assert_eq!(
        missing
            .evaluate(pose.coeffs(), &sb, pose.coeffs(), &sb)
            .unwrap_err(),
        IntegrationError::MissingMeasurements
    );
    factor
        .evaluate_residual(pose.coeffs(), &sb, pose.coeffs(), &sb)
        .unwrap();
    factor.square_root_information = None;
    assert_eq!(
        factor.evaluate_residual(pose.coeffs(), &sb, pose.coeffs(), &sb),
        Err(IntegrationError::InvalidCovariance)
    );
    assert_eq!(
        factor
            .evaluate(pose.coeffs(), &sb, pose.coeffs(), &sb)
            .unwrap_err(),
        IntegrationError::InvalidCovariance
    );
}

#[test]
fn imu_constant_acceleration_and_bias_cancellation_follow_analytic_motion() {
    let parameters = imu_parameters();
    for acceleration in [Vector3::zeros(), Vector3::new(0.4, -0.2, 0.1)] {
        let gyro_bias = Vector3::new(0.01, -0.02, 0.03);
        let acc_bias = Vector3::new(0.1, 0.2, -0.1);
        let samples = imu_samples(
            gyro_bias,
            acceleration + Vector3::new(0.0, 0.0, parameters.g) + acc_bias,
        );
        let mut pose = Transformation::identity();
        let mut state = SVector::<f64, 9>::zeros();
        let v0 = Vector3::new(0.2, -0.1, 0.3);
        state.fixed_rows_mut::<3>(0).copy_from(&v0);
        state.fixed_rows_mut::<3>(3).copy_from(&gyro_bias);
        state.fixed_rows_mut::<3>(6).copy_from(&acc_bias);
        let steps = ImuError::propagation(
            &samples,
            &parameters,
            &mut pose,
            &mut state,
            (time(0), time(1000)),
            (None, None),
        )
        .unwrap();
        assert_eq!(steps, 100);
        assert!((pose.translation() - (v0 + 0.5 * acceleration)).norm() < 1e-13);
        assert!((state.fixed_rows::<3>(0) - (v0 + acceleration)).norm() < 1e-13);
        assert!((pose.rotation_matrix() - SMatrix::<f64, 3, 3>::identity()).norm() < 1e-14);
        assert_eq!(state.fixed_rows::<3>(3), gyro_bias);
        assert_eq!(state.fixed_rows::<3>(6), acc_bias);
    }
}

#[test]
fn imu_constant_yaw_and_clipped_time_window_preserve_physical_units() {
    let params = imu_parameters();
    let rate = 0.4;
    let samples = imu_samples(
        Vector3::new(0.0, 0.0, rate),
        Vector3::new(0.0, 0.0, params.g),
    );
    let mut pose = Transformation::identity();
    let mut state = SVector::<f64, 9>::zeros();
    let start = time(35);
    let end = time(965);
    let duration = 0.93;
    ImuError::propagation(
        &samples,
        &params,
        &mut pose,
        &mut state,
        (start, end),
        (None, None),
    )
    .unwrap();
    let (sin, cos) = (rate * duration).sin_cos();
    let expected = SMatrix::<f64, 3, 3>::new(cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0);
    assert!((pose.rotation_matrix() - expected).norm() < 2e-14);
    assert!(pose.translation().norm() < 1e-13);
    assert!(state.fixed_rows::<3>(0).norm() < 1e-13);
}

#[test]
fn imu_covariance_is_symmetric_positive_semidefinite_and_scales_with_noise_variance() {
    let params = imu_parameters();
    let mut doubled = params.clone();
    doubled.sigma_g_c *= 2.0;
    doubled.sigma_a_c *= 2.0;
    doubled.sigma_gw_c *= 2.0;
    doubled.sigma_aw_c *= 2.0;
    let samples = imu_samples(
        Vector3::new(0.03, -0.04, 0.02),
        Vector3::new(0.2, -0.1, params.g),
    );
    let run = |p: &ImuParameters| {
        let mut pose = Transformation::identity();
        let mut state = SVector::<f64, 9>::zeros();
        let mut covariance = Covariance::zeros();
        let mut jacobian = Jacobian::zeros();
        ImuError::propagation(
            &samples,
            p,
            &mut pose,
            &mut state,
            (time(0), time(1000)),
            (Some(&mut covariance), Some(&mut jacobian)),
        )
        .unwrap();
        (pose, state, covariance, jacobian)
    };
    let (pose, state, covariance, jacobian) = run(&params);
    let (pose2, state2, covariance2, jacobian2) = run(&doubled);
    assert_eq!(pose.coeffs(), pose2.coeffs());
    assert_eq!(state, state2);
    assert_eq!(jacobian, jacobian2);
    assert!((covariance - covariance.transpose()).amax() < 1e-18);
    assert!((covariance2 - 4.0 * covariance).amax() < 1e-17);
    assert!(covariance.iter().all(|v| v.is_finite()));
    let eigenvalues = covariance.symmetric_eigen().eigenvalues;
    assert!(eigenvalues.min() > -1e-16);
    assert!(eigenvalues.max() > 0.0);
}

#[test]
fn imu_missing_time_coverage_returns_specific_errors_without_mutating_state() {
    let parameters = imu_parameters();
    let samples = imu_samples(Vector3::zeros(), Vector3::z() * parameters.g);
    for (data, start, end, expected) in [
        (
            ImuMeasurementDeque::new(),
            time(0),
            time(1000),
            IntegrationError::MissingMeasurements,
        ),
        (
            samples.clone(),
            time(-1),
            time(500),
            IntegrationError::StartNotCovered,
        ),
        (
            samples,
            time(0),
            time(1001),
            IntegrationError::EndNotCovered,
        ),
    ] {
        let mut pose = Transformation::new(
            Vector3::new(1.0, 2.0, 3.0),
            delta_q(&Vector3::new(0.1, 0.2, 0.3)),
        );
        let before = *pose.coeffs();
        let mut state = SVector::<f64, 9>::repeat(0.1);
        let original = state;
        assert_eq!(
            ImuError::propagation(
                &data,
                &parameters,
                &mut pose,
                &mut state,
                (start, end),
                (None, None)
            ),
            Err(expected)
        );
        assert_eq!(*pose.coeffs(), before);
        assert_eq!(state, original);
    }
}

#[test]
fn one_sided_depth_has_analytic_hinge_and_zero_jacobians_for_infinite_points() {
    let error = OneSidedDepthError::new(2.0, 0.5);
    let pose = Transformation::identity();
    for depth in [1.0, 2.0, 3.0] {
        let point = Vector4::new(0.2, -0.3, depth, 1.0);
        let mut residual = f64::NAN;
        let mut j = SMatrix::<f64, 1, 4>::repeat(99.0);
        assert!(error.evaluate(
            (pose.coeffs(), &point, pose.coeffs()),
            &mut residual,
            Some(DepthJacobians {
                point: Some(&mut j),
                ..Default::default()
            })
        ));
        assert_eq!(residual, 2.0 * (2.0 - depth).max(0.0));
        assert_eq!(j[(0, 2)], if depth > 2.0 { 0.0 } else { -2.0 });
    }
    let point = Vector4::new(0.0, 0.0, 1.0, 0.0);
    let mut residual = 99.0;
    let mut j = SMatrix::<f64, 1, 4>::repeat(99.0);
    assert!(error.evaluate(
        (pose.coeffs(), &point, pose.coeffs()),
        &mut residual,
        Some(DepthJacobians {
            point: Some(&mut j),
            ..Default::default()
        })
    ));
    assert_eq!(residual, 0.0);
    assert_eq!(j, SMatrix::<f64, 1, 4>::zeros());
}

#[test]
fn weighted_prior_solver_reaches_analytic_optimum_for_every_backend_and_preserves_fixed_blocks() {
    for trust_region in ["DOGLEG", "LEVENBERG_MARQUARDT"] {
        for linear_solver in ["SPARSE_NORMAL_CHOLESKY", "DENSE_QR", "DENSE_SCHUR"] {
            let state = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
                SVector::repeat(-3.0),
                1,
                time(0),
            )));
            let fixed = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
                SVector::repeat(5.0),
                2,
                time(0),
            )));
            let mut problem = GraphProblem::default();
            problem.register(Parameter::SpeedBias(state.clone()), ManifoldKind::Euclidean);
            problem.register(Parameter::SpeedBias(fixed.clone()), ManifoldKind::Euclidean);
            Parameter::SpeedBias(fixed.clone()).set_fixed(true);
            for (measurement, variance) in [(1.0, 1.0), (3.0, 0.25)] {
                problem.add(
                    GraphFactor::SpeedBiasPrior(Box::new(SpeedAndBiasError::from_variances(
                        SVector::repeat(measurement),
                        variance,
                        variance,
                        variance,
                    ))),
                    vec![Parameter::SpeedBias(state.clone())],
                    None,
                );
            }
            problem.add(
                GraphFactor::SpeedBiasPrior(Box::new(SpeedAndBiasError::from_variances(
                    SVector::zeros(),
                    1.0,
                    1.0,
                    1.0,
                ))),
                vec![Parameter::SpeedBias(fixed.clone())],
                None,
            );
            let summary = RustGraphSolver
                .solve(
                    &mut problem,
                    &SolverOptions {
                        trust_region,
                        linear_solver,
                        function_tolerance: 1e-12,
                        ..Default::default()
                    },
                )
                .unwrap();
            // Each coordinate minimizes (x-1)^2 + 4(x-3)^2: x=13/5.
            // Cost comparisons include the constant fixed-block cost. f64 cancellation
            // permits ~1e-7 coordinate uncertainty near this minimum (quadratic curvature 5).
            let actual = Parameter::SpeedBias(state).values().unwrap();
            assert!(
                actual.iter().all(|v| (v - 2.6).abs() < 1e-7),
                "{trust_region}/{linear_solver}: {actual:?}"
            );
            assert_eq!(Parameter::SpeedBias(fixed).values().unwrap(), vec![5.0; 9]);
            let expected =
                0.5 * 9.0 * ((2.6f64 - 1.0).powi(2) + 4.0 * (2.6f64 - 3.0).powi(2) + 25.0);
            assert!((summary.final_cost.unwrap() - expected).abs() < 1e-10);
            assert!(summary.final_cost.unwrap() < summary.initial_cost.unwrap());
        }
    }
}

#[test]
fn huber_loss_limits_planted_outlier_influence() {
    let state = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
        SVector::zeros(),
        1,
        time(0),
    )));
    let mut problem = GraphProblem::default();
    problem.register(Parameter::SpeedBias(state.clone()), ManifoldKind::Euclidean);
    for measurement in [0.0, 0.0, 0.0, 0.0, 100.0] {
        problem.add(
            GraphFactor::SpeedBiasPrior(Box::new(SpeedAndBiasError::from_variances(
                SVector::repeat(measurement),
                1.0,
                1.0,
                1.0,
            ))),
            vec![Parameter::SpeedBias(state.clone())],
            Some(RobustLoss::Huber(1.0)),
        );
    }
    let summary = RustGraphSolver
        .solve(
            &mut problem,
            &SolverOptions {
                max_iterations: 100,
                function_tolerance: 1e-12,
                ..Default::default()
            },
        )
        .unwrap();
    // Four quadratic 9-D inliers balance the saturated radial gradient of one outlier:
    // 4 * 9 * x = sqrt(9), so x = 1/12, instead of the least-squares mean 20.
    let actual = Parameter::SpeedBias(state).values().unwrap();
    assert!(
        actual.iter().all(|v| (v - 1.0 / 12.0).abs() < 1e-6),
        "{actual:?}"
    );
    assert!(summary.final_cost.unwrap().is_finite());
}
