// SPDX-License-Identifier: BSD-3-Clause

//! Metric pose-graph closure and inertial factors on independently constructed trajectories.
use firefly_okvis_x::{
    ceres::{
        imu_error::ImuError,
        pose_local_parameterization::PoseManifold,
        pose_parameter_block::PoseParameterBlock,
        relative_pose_error::RelativePoseError,
        vi_graph::{
            GraphFactor, GraphProblem, GraphSolver, ManifoldKind, Parameter, RustGraphSolver,
            SolverOptions,
        },
    },
    config::parameters::ImuParameters,
    kinematics::transformation::Transformation,
    sensors::measurements::{ImuMeasurement, ImuSensorReadings},
};
use jiff::Timestamp;
use nalgebra::{Isometry3, SMatrix, SVector, UnitQuaternion, Vector3};
use std::{cell::RefCell, rc::Rc};

#[test]
fn anchored_loop_recovers_metric_poses_with_both_trust_region_strategies() {
    let truth = [
        Isometry3::identity(),
        Isometry3::new(Vector3::new(1.0, 0.1, 0.0), Vector3::new(0.05, 0.02, 0.1)),
        Isometry3::new(Vector3::new(1.0, 1.0, 0.2), Vector3::new(0.1, -0.1, 0.2)),
        Isometry3::new(Vector3::new(0.0, 1.0, 0.1), Vector3::new(-0.1, 0.2, 0.3)),
    ];
    for trust_region in ["DOGLEG", "LEVENBERG_MARQUARDT"] {
        let blocks: Vec<_> = truth
            .iter()
            .enumerate()
            .map(|(i, t)| {
                let mut initial = Transformation::from_matrix(&t.to_homogeneous());
                if i != 0 {
                    initial.oplus(&SVector::<f64, 6>::new(0.1, -0.05, 0.02, 0.02, -0.03, 0.01));
                }
                Rc::new(RefCell::new(PoseParameterBlock::new(
                    &initial,
                    i as u64,
                    Timestamp::UNIX_EPOCH,
                )))
            })
            .collect();
        let mut graph = GraphProblem::default();
        for block in &blocks {
            graph.register(Parameter::Pose(block.clone()), ManifoldKind::Pose);
        }
        Parameter::Pose(blocks[0].clone()).set_fixed(true);
        for (first, second) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
            let measurement = truth[first].inverse() * truth[second];
            graph.add(
                GraphFactor::RelativePose(Box::new(RelativePoseError::from_variances(
                    0.01,
                    0.01,
                    measurement,
                ))),
                vec![
                    Parameter::Pose(blocks[first].clone()),
                    Parameter::Pose(blocks[second].clone()),
                ],
                None,
            );
        }
        let summary = RustGraphSolver
            .solve(
                &mut graph,
                &SolverOptions {
                    trust_region,
                    function_tolerance: 1e-12,
                    ..Default::default()
                },
            )
            .unwrap();
        assert!(
            summary.final_cost.unwrap() < 1e-14,
            "{trust_region}: {summary:?}"
        );
        for (block, expected) in blocks.iter().zip(&truth) {
            let values = SVector::<f64, 7>::from_column_slice(
                &Parameter::Pose(block.clone()).values().unwrap(),
            );
            let mut actual = Transformation::identity();
            assert!(actual.set_coeffs(&values));
            assert!(
                (actual.matrix() - expected.to_homogeneous()).amax() < 1e-8,
                "{trust_region}"
            );
        }
        assert_eq!(
            Parameter::Pose(blocks[0].clone()).values().unwrap(),
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        );
    }
}

#[test]
fn relative_pose_factor_jacobians_match_manifold_differences_away_from_zero_residual() {
    let first = Transformation::new(
        Vector3::new(0.2, -0.1, 0.3),
        UnitQuaternion::from_euler_angles(0.2, -0.3, 0.1).into_inner(),
    );
    let second = Transformation::new(
        Vector3::new(1.0, 0.4, -0.2),
        UnitQuaternion::from_euler_angles(-0.1, 0.2, 0.4).into_inner(),
    );
    let factor = RelativePoseError::from_variances(
        0.5,
        0.2,
        Isometry3::new(Vector3::new(0.5, 0.1, -0.2), Vector3::new(0.1, 0.2, 0.3)),
    );
    let mut residual = SVector::<f64, 6>::zeros();
    let mut ambient_first = SMatrix::<f64, 6, 7>::zeros();
    let mut ambient_second = ambient_first;
    let mut minimal_first = SMatrix::<f64, 6, 6>::zeros();
    let mut minimal_second = minimal_first;
    assert!(factor.evaluate_with_minimal_jacobians(
        [first.coeffs(), second.coeffs()],
        &mut residual,
        [Some(&mut ambient_first), Some(&mut ambient_second)],
        [Some(&mut minimal_first), Some(&mut minimal_second)]
    ));
    assert!(residual.norm() > 0.1);
    for (block, expected) in [minimal_first, minimal_second].iter().enumerate() {
        for axis in 0..6 {
            let mut step = SVector::<f64, 6>::zeros();
            step[axis] = 1e-6;
            let input = if block == 0 {
                first.coeffs()
            } else {
                second.coeffs()
            };
            let plus = PoseManifold::plus(input, &step);
            let minus = PoseManifold::plus(input, &-step);
            let mut high = SVector::<f64, 6>::zeros();
            let mut low = high;
            let args = |changed| {
                if block == 0 {
                    [changed, second.coeffs()]
                } else {
                    [first.coeffs(), changed]
                }
            };
            assert!(factor.evaluate(args(&plus), &mut high, [None, None]));
            assert!(factor.evaluate(args(&minus), &mut low, [None, None]));
            assert!(
                ((high - low) / 2e-6 - expected.column(axis)).amax() < 2e-9,
                "block={block}, axis={axis}"
            );
        }
    }
}

#[test]
fn inertial_factor_vanishes_on_independently_integrated_constant_acceleration() {
    let params = ImuParameters {
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
    };
    let acceleration = Vector3::new(0.4, -0.2, 0.1);
    let initial_velocity = Vector3::new(0.2, 0.1, -0.1);
    let samples = (0..=100)
        .map(|i| {
            ImuMeasurement::new(
                Timestamp::from_millisecond(i * 10).unwrap(),
                ImuSensorReadings::new(
                    Vector3::zeros(),
                    acceleration + Vector3::new(0.0, 0.0, params.g),
                ),
                -1,
            )
        })
        .collect();
    let first = Transformation::identity();
    let second = Transformation::new(
        initial_velocity + acceleration * 0.5,
        UnitQuaternion::identity().into_inner(),
    );
    let mut start = SVector::<f64, 9>::zeros();
    start.fixed_rows_mut::<3>(0).copy_from(&initial_velocity);
    let mut end = start;
    end.fixed_rows_mut::<3>(0)
        .copy_from(&(initial_velocity + acceleration));
    let mut factor = ImuError::new(
        samples,
        params,
        Timestamp::UNIX_EPOCH,
        Timestamp::from_second(1).unwrap(),
    )
    .unwrap();
    let evaluation = factor
        .evaluate(first.coeffs(), &start, second.coeffs(), &end)
        .unwrap();
    assert!(
        evaluation.residual.norm() < 1e-9,
        "{:?}",
        evaluation.residual
    );
    assert!(evaluation.integration_success);
    // Velocity and bias columns are checked against the factor's local residual evaluations;
    // the trajectory above supplies the independent final-state oracle.
    for (block, expected) in [evaluation.speed_bias0, evaluation.speed_bias1]
        .iter()
        .enumerate()
    {
        for axis in 0..9 {
            let input = if block == 0 { start } else { end };
            let mut plus = input;
            let mut minus = input;
            plus[axis] += 1e-6;
            minus[axis] -= 1e-6;
            let mut evaluate = |changed| {
                if block == 0 {
                    factor
                        .evaluate(first.coeffs(), changed, second.coeffs(), &end)
                        .unwrap()
                        .residual
                } else {
                    factor
                        .evaluate(first.coeffs(), &start, second.coeffs(), changed)
                        .unwrap()
                        .residual
                }
            };
            let numeric = (evaluate(&plus) - evaluate(&minus)) / 2e-6;
            assert!(
                (numeric - expected.column(axis)).amax() < 1e-5,
                "block={block}, axis={axis}"
            );
        }
    }
}
