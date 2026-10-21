// SPDX-License-Identifier: BSD-3-Clause
//! Landmark elimination is compared with an independently assembled least-squares projector.
use firefly_okvis_x::{
    cameras::{no_distortion::NoDistortion, pinhole_camera::PinholeCamera},
    ceres::{
        component::ComponentState, error_interface::ErrorInterface,
        homogeneous_point_parameter_block::HomogeneousPointParameterBlock,
        pose_local_parameterization::PoseManifold, pose_parameter_block::PoseParameterBlock,
        reprojection_error::ReprojectionError,
        speed_and_bias_parameter_block::SpeedAndBiasParameterBlock,
        two_pose_graph_error::TwoPoseStandardGraphError, vi_graph::GraphState,
        vi_graph_estimator::ViGraphEstimator,
    },
    frames::frame_typedefs::{KeypointIdentifier, StateId},
    kinematics::transformation::Transformation,
};
use firefly_okvis_x::{
    ceres::{
        imu_error::ImuError,
        vi_graph::{AnyState, GraphFactor, ManifoldKind, Parameter},
    },
    config::parameters::ImuParameters,
    sensors::measurements::{ImuMeasurement, ImuMeasurementDeque, ImuSensorReadings},
};
use jiff::Timestamp;
use nalgebra::{
    DMatrix, DVector, Matrix2, SMatrix, SVector, UnitQuaternion, Vector2, Vector3, Vector4,
};
use std::{cell::RefCell, rc::Rc, sync::Arc};
fn shared(p: &Transformation, id: u64) -> Rc<RefCell<PoseParameterBlock>> {
    Rc::new(RefCell::new(PoseParameterBlock::new(
        p,
        id,
        Timestamp::UNIX_EPOCH,
    )))
}
fn projected(pose: &SVector<f64, 7>, point: &Vector4<f64>) -> Vector2<f64> {
    let mut t = Transformation::identity();
    assert!(t.set_coeffs(pose));
    let p = t.inverse().transform_homogeneous(point);
    Vector2::new(320.0 * p.x / p.z + 319.5, 300.0 * p.y / p.z + 239.5)
}
struct System {
    factor: TwoPoseStandardGraphError,
    expected_h: DMatrix<f64>,
    expected_b: DVector<f64>,
    first: Transformation,
    second: Transformation,
}
fn system() -> System {
    let camera = Arc::new(PinholeCamera::new(
        (640, 480),
        (320.0, 300.0, 319.5, 239.5),
        NoDistortion,
        0,
    ));
    let first = Transformation::identity();
    let second = Transformation::new(
        Vector3::new(0.5, 0.1, 0.02),
        UnitQuaternion::from_euler_angles(0.02, -0.03, 0.01).into_inner(),
    );
    let p0 = shared(&first, 1);
    let p1 = shared(&second, 2);
    let ext = shared(&Transformation::identity(), 3);
    let mut factor = TwoPoseStandardGraphError::new(StateId::new(1), StateId::new(2), 1, true);
    let mut expected_h = DMatrix::zeros(6, 6);
    let mut expected_b = DVector::zeros(6);
    for i in 0..18 {
        let point = Vector4::new(
            f64::from(i % 6) * 0.2 - 0.5,
            f64::from(i / 6) * 0.25 - 0.25,
            4.0 + f64::from(i % 4) * 0.4,
            1.0,
        );
        let block = Rc::new(RefCell::new(HomogeneousPointParameterBlock::new(
            point,
            i as u64 + 10,
            true,
        )));
        let mut a = SMatrix::<f64, 4, 6>::zeros();
        let mut b = SMatrix::<f64, 4, 3>::zeros();
        let mut residual = SVector::<f64, 4>::zeros();
        for (view, t, p) in [(0, &first, p0.clone()), (1, &second, p1.clone())] {
            let offset = Vector2::new(
                (f64::from(i % 3) - 1.0) * 0.1,
                (f64::from(i % 5) - 2.0) * 0.08,
            );
            let observation = ReprojectionError::new(
                camera.clone(),
                0,
                projected(t.coeffs(), &point) + offset,
                Matrix2::identity(),
            );
            factor
                .add_observation(
                    KeypointIdentifier::new(view as u64 + 1, 0, i as usize),
                    &observation,
                    None,
                    (p, block.clone(), ext.clone()),
                    false,
                    1.0,
                )
                .unwrap();
            residual.fixed_rows_mut::<2>(view * 2).copy_from(&offset);
            // The point derivative is the independent perspective derivative in the local camera.
            let local = t.inverse().transform_homogeneous(&point);
            let perspective = SMatrix::<f64, 2, 3>::new(
                -320.0 / local.z,
                0.0,
                320.0 * local.x / local.z.powi(2),
                0.0,
                -300.0 / local.z,
                300.0 * local.y / local.z.powi(2),
            );
            b.fixed_view_mut::<2, 3>(view * 2, 0)
                .copy_from(&(perspective * t.rotation_matrix().transpose()));
            if view == 1 {
                for axis in 0..6 {
                    let mut delta = SVector::<f64, 6>::zeros();
                    delta[axis] = 1e-6;
                    let j = -(projected(&PoseManifold::plus(t.coeffs(), &delta), &point)
                        - projected(&PoseManifold::plus(t.coeffs(), &-delta), &point))
                        / 2e-6;
                    a.fixed_view_mut::<2, 1>(2, axis).copy_from(&j);
                }
            }
        }
        let projection = SMatrix::<f64, 4, 4>::identity()
            - b * (b.transpose() * b).try_inverse().unwrap() * b.transpose();
        expected_h += a.transpose() * projection * a;
        expected_b -= a.transpose() * projection * residual;
    }
    factor.compute().unwrap();
    System {
        factor,
        expected_h,
        expected_b,
        first,
        second,
    }
}
#[test]
fn eliminated_visual_hessian_and_gradient_match_full_least_squares_projection() {
    let s = system();
    assert!(
        (&s.factor.h00 - &s.expected_h).amax() < 3e-5,
        "H error={}",
        (&s.factor.h00 - &s.expected_h).amax()
    );
    assert!((&s.factor.b0 - &s.expected_b).amax() < 1e-7);
    assert!((&s.factor.h00 - s.factor.h00.transpose()).amax() < 1e-10);
    let spectrum = s.factor.h00.clone().symmetric_eigen().eigenvalues;
    assert!(spectrum.min() > -1e-8);
    assert!(spectrum.max() > 1.0);
}
#[test]
fn frozen_prior_reproduces_the_reduced_quadratic_without_reintegrating_observations() {
    let mut s = system();
    let before = s.factor.h00.clone();
    s.factor.compute().unwrap();
    assert_eq!(s.factor.h00, before);
    let frozen = s.factor.clone_two_pose_graph_error_const();
    let first = *s.first.coeffs();
    let second = *s.second.coeffs();
    let evaluate = |p: &SVector<f64, 7>| {
        frozen
            .evaluate_with_minimal_jacobians(
                &[first.as_slice(), p.as_slice()],
                Some(&[true, true]),
                Some(&[true, true]),
            )
            .unwrap()
    };
    let e = evaluate(&second);
    let j = e.minimal_jacobians[1].as_ref().unwrap();
    assert!((j.transpose() * j - &s.expected_h).amax() < 3e-5);
    for axis in 0..6 {
        let mut delta = SVector::<f64, 6>::zeros();
        delta[axis] = 1e-6;
        let numeric = (evaluate(&PoseManifold::plus(&second, &delta)).residuals
            - evaluate(&PoseManifold::plus(&second, &-delta)).residuals)
            / 2e-6;
        assert!((numeric - j.column(axis)).amax() < 1e-6);
    }
    let mut observations = Vec::new();
    let mut duplicates = Vec::new();
    s.factor
        .convert_to_reprojection_errors(&mut observations, &mut duplicates)
        .unwrap();
    assert_eq!(observations.len(), 36);
    assert_eq!(duplicates.len(), 0);
    assert_eq!(s.factor.observations.len(), 0);
    assert!(s.factor.h00.iter().all(|v| *v == 0.0));
    assert_eq!(
        evaluate(&second).residuals,
        e.residuals,
        "frozen prior owns its linearization"
    );
}
fn estimator() -> ViGraphEstimator {
    let mut e = ViGraphEstimator::new();
    for i in 1..=3 {
        let id = StateId::new(i);
        let pose = Transformation::new(
            Vector3::new(i as f64, 0.0, 0.0),
            UnitQuaternion::identity().into_inner(),
        );
        e.states.insert(
            id,
            ComponentState {
                timestamp: Some(Timestamp::UNIX_EPOCH),
                pose: Some(shared(&pose, i)),
                speed_and_bias: Some(Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
                    SVector::repeat(0.1),
                    i,
                    Timestamp::UNIX_EPOCH,
                )))),
                ..Default::default()
            },
        );
        e.state_info.insert(
            id,
            GraphState {
                is_keyframe: i != 2,
                ..Default::default()
            },
        );
    }
    e
}
#[test]
fn realtime_and_full_graph_state_checks_detect_pose_bias_keyframe_and_missing_state_changes() {
    let first = estimator();
    let mut second = estimator();
    assert!(first.check_synched_states(&second).is_ok());
    assert!(first.is_observationless_synched(&second).unwrap());
    let id = StateId::new(2);
    second.set_pose(id, &Transformation::identity()).unwrap();
    assert!(first.check_synched_states(&second).is_err());
    second = estimator();
    second.set_speed_and_bias(id, SVector::repeat(0.2)).unwrap();
    assert!(first.check_synched_states(&second).is_err());
    second = estimator();
    second.set_keyframe(id, true).unwrap();
    assert!(first.check_synched_states(&second).is_err());
    second = estimator();
    second.states.remove(&id);
    assert!(second.check_synched_states(&first).is_err());
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

fn stationary_window() -> (ViGraphEstimator, ImuMeasurementDeque) {
    let measurements: ImuMeasurementDeque = (0..=30)
        .map(|i| {
            ImuMeasurement::new(
                Timestamp::from_millisecond(i * 10).unwrap(),
                ImuSensorReadings::new(Vector3::zeros(), Vector3::new(0.0, 0.0, 9.81)),
                -1,
            )
        })
        .collect();
    let mut graph = ViGraphEstimator::new();
    for index in 0..4 {
        let id = StateId::new(index + 1);
        let timestamp = Timestamp::from_millisecond(index as i64 * 100).unwrap();
        let pose = shared(&Transformation::identity(), index + 1);
        let speed_bias = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
            SVector::zeros(),
            index + 1,
            timestamp,
        )));
        graph
            .problem
            .register(Parameter::Pose(pose.clone()), ManifoldKind::Pose);
        graph.problem.register(
            Parameter::SpeedBias(speed_bias.clone()),
            ManifoldKind::Euclidean,
        );
        graph.states.insert(
            id,
            ComponentState {
                timestamp: Some(timestamp),
                pose: Some(pose),
                speed_and_bias: Some(speed_bias),
                ..Default::default()
            },
        );
        graph.state_info.insert(id, GraphState::default());
        graph.any_state.insert(
            id,
            AnyState {
                keyframe_id: Some(StateId::new(1)),
                timestamp,
                t_sk_s: Transformation::identity(),
                v_sk: Vector3::zeros(),
            },
        );
    }
    for index in 0..3 {
        let first = StateId::new(index + 1);
        let second = StateId::new(index + 2);
        let start = graph.states[&first].timestamp.unwrap();
        let end = graph.states[&second].timestamp.unwrap();
        let samples = measurements
            .iter()
            .filter(|m| m.time_stamp >= start && m.time_stamp <= end)
            .cloned()
            .collect();
        let mut factor = ImuError::new(samples, imu_parameters(), start, end).unwrap();
        factor
            .evaluate(
                Transformation::identity().coeffs(),
                &SVector::zeros(),
                Transformation::identity().coeffs(),
                &SVector::zeros(),
            )
            .unwrap();
        let parameters = vec![
            Parameter::Pose(graph.states[&first].pose.clone().unwrap()),
            Parameter::SpeedBias(graph.states[&first].speed_and_bias.clone().unwrap()),
            Parameter::Pose(graph.states[&second].pose.clone().unwrap()),
            Parameter::SpeedBias(graph.states[&second].speed_and_bias.clone().unwrap()),
        ];
        let link = graph.problem.add(
            GraphFactor::Imu(Rc::new(RefCell::new(factor))),
            parameters,
            None,
        );
        graph.state_info.get_mut(&first).unwrap().next_imu = Some(link);
        graph.state_info.get_mut(&second).unwrap().previous_imu = Some(link);
    }
    (graph, measurements)
}
#[test]
fn sliding_window_merges_imu_factors_and_preserves_endpoints_and_historical_states() {
    let (mut graph, samples) = stationary_window();
    let first = StateId::new(1);
    let last = StateId::new(4);
    assert!(graph.eliminate_state_by_imu_merge(first, first).is_err());
    assert!(graph.eliminate_state_by_imu_merge(last, first).is_err());
    assert_eq!(graph.states.len(), 4);
    for id in [StateId::new(2), StateId::new(3)] {
        assert!(graph.eliminate_state_by_imu_merge(id, first).unwrap());
        assert!(!graph.states.contains_key(&id));
        assert!(!graph.state_info.contains_key(&id));
        assert_eq!(graph.any_state[&id].keyframe_id, Some(first));
        assert_eq!(
            graph.any_state[&id].t_sk_s.matrix(),
            Transformation::identity().matrix()
        );
    }
    assert_eq!(graph.states.len(), 2);
    assert_eq!(graph.problem.residuals.len(), 1);
    assert_eq!(graph.problem.parameters.len(), 4);
    let link = graph.state_info[&first].next_imu.unwrap();
    assert_eq!(graph.state_info[&last].previous_imu, Some(link));
    let GraphFactor::Imu(factor) = graph.problem.residuals[&link].factor.as_ref() else {
        panic!("IMU link required");
    };
    let mut factor = factor.borrow_mut();
    assert_eq!(factor.imu_measurements.len(), samples.len());
    assert!(
        factor
            .imu_measurements
            .iter()
            .map(|m| m.time_stamp)
            .eq(samples.iter().map(|m| m.time_stamp))
    );
    let pose = Transformation::identity();
    let bias = SVector::zeros();
    let cached_information = factor.information.unwrap();
    let merged = factor
        .evaluate(pose.coeffs(), &bias, pose.coeffs(), &bias)
        .unwrap();
    let mut direct = ImuError::new(
        samples,
        imu_parameters(),
        Timestamp::UNIX_EPOCH,
        Timestamp::from_millisecond(300).unwrap(),
    )
    .unwrap();
    let direct_evaluation = direct
        .evaluate(pose.coeffs(), &bias, pose.coeffs(), &bias)
        .unwrap();
    assert!((cached_information - direct.information.unwrap()).amax() < 1e-6);
    assert!(merged.residual.amax() < 1e-9);
    assert!((merged.residual - direct_evaluation.residual).amax() < 1e-12);
    assert!((merged.pose0 - direct_evaluation.pose0).amax() < 1e-9);
}
