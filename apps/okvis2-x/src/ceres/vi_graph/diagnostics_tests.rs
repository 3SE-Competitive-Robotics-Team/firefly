use super::*;
use crate::{
    cameras::{
        n_camera_system::{CameraType, DistortionType, NCameraSystem},
        no_distortion::NoDistortion,
        pinhole_camera::PinholeCamera,
    },
    ceres::{
        component::ComponentState,
        pose_parameter_block::PoseParameterBlock,
        reprojection_error::ReprojectionError,
        vi_graph::{
            KeypointIdentifier, LandmarkId, ManifoldKind, Parameter, RobustLoss, SolverSummary,
            StateId, ViGraph,
        },
    },
    frames::multi_frame::MultiFrame,
    kinematics::transformation::Transformation,
};
use jiff::Timestamp;
use nalgebra::{Matrix2, Vector2, Vector4};
use purecv::{core::types::Point2f, features2d::KeyPoint};
use std::{cell::RefCell, rc::Rc, sync::Arc};

#[test]
fn support_distinguishes_ids_graph_depth_projection_and_nonzero_influence() {
    let camera = Arc::new(PinholeCamera::new(
        (64, 48),
        (60., 60., 31.5, 23.5),
        NoDistortion,
        0,
    ));
    let mut rig = NCameraSystem::new();
    rig.add_camera(
        &Arc::new(Transformation::identity()),
        camera.clone(),
        DistortionType::NoDistortion,
        true,
        CameraType::default(),
    );
    let mut frame = MultiFrame::new(&rig, Timestamp::UNIX_EPOCH, 1);
    frame.reset_keypoints(
        0,
        &vec![KeyPoint::new(Point2f::new(33.5, 20.5), 12., -1., 1., 0, -1); 7],
    );
    let mut graph = ViGraph::new();
    let pose = Rc::new(RefCell::new(PoseParameterBlock::new(
        &Transformation::identity(),
        1,
        Timestamp::UNIX_EPOCH,
    )));
    let extrinsics = Rc::new(RefCell::new(PoseParameterBlock::new(
        &Transformation::identity(),
        0,
        Timestamp::UNIX_EPOCH,
    )));
    graph
        .problem
        .register(Parameter::Pose(pose.clone()), ManifoldKind::Pose);
    graph
        .problem
        .register(Parameter::Pose(extrinsics.clone()), ManifoldKind::Pose);
    graph.states.insert(
        StateId::new(1),
        ComponentState {
            pose: Some(pose),
            extrinsics: vec![Some(extrinsics)],
            ..Default::default()
        },
    );
    for index in 1..7 {
        let id = LandmarkId::new(index as u64);
        frame.set_landmark_id(0, index, id.value());
        graph
            .add_landmark(
                id,
                Vector4::new(0., 0., if index == 6 { -2. } else { 2. }, 1.),
                index != 2,
            )
            .unwrap();
        if index == 1 {
            continue;
        }
        let information = if index == 3 {
            Matrix2::zeros()
        } else {
            Matrix2::new(4., 0., 0., 9.)
        };
        let factor =
            ReprojectionError::new(camera.clone(), 0, Vector2::new(33.5, 20.5), information);
        let key = KeypointIdentifier::new(1, 0, index);
        graph
            .add_external_observation(&factor, id, key, true)
            .unwrap();
        if index == 4 {
            let residual = graph.observations[&key].residual;
            graph.problem.residuals.get_mut(&residual).unwrap().loss = Some(RobustLoss::Tukey(1.0));
        }
    }
    let points = graph.frame_observations(&frame).unwrap();
    assert_eq!(points.len(), 7);
    assert_eq!(points[0].flags, 0);
    assert_eq!(points[1].flags, DEPTH_READY | PROJECTABLE);
    assert!(points[1].robust_cost.is_nan());
    assert_eq!(points[2].flags, GRAPH | PROJECTABLE | EFFECTIVE);
    assert_eq!(points[3].flags, GRAPH | PROJECTABLE | DEPTH_READY);
    assert_eq!(points[4].flags, GRAPH | PROJECTABLE | DEPTH_READY);
    assert_eq!(points[4].robust_derivative, 0.0);
    assert_eq!(
        points[5].flags,
        GRAPH | PROJECTABLE | DEPTH_READY | EFFECTIVE
    );
    assert_eq!(points[6].flags, GRAPH);
    assert_eq!(points[5].residual, [2., -3.]);
    assert_eq!(points[5].sqrt_information, [[2., 0.], [0., 3.]]);
    assert_eq!(points[5].whitened_squared, 97.);
    assert!((points[5].robust_derivative - 1. / 98.).abs() < 1e-14);
    assert!((points[5].robust_cost - 0.5 * 98_f64.ln()).abs() < 1e-14);
    assert_eq!(points[5].depth_z, 2.);
    assert_eq!(points[5].range, 2.);
    assert_eq!(points[5].inverse_range, 0.5);
    assert_eq!(residual_summary(&points), [2., 13_f64.sqrt(), 2., -3.]);
}

#[test]
fn serialized_solver_records_fit_the_ipc_log_and_preserve_rejected_state() {
    let cost = FactorCosts {
        visual: 1.234_567_890_123_456_7e200,
        imu: 1.234_567_890_123_456_7e-200,
        marginalization: 1.234_567_890_123_456_7e100,
        prior: 1.234_567_890_123_456_7e-100,
    };
    let summary = SolverSummary {
        iterations: usize::MAX,
        diagnostic_initial: Some(cost),
        diagnostic_final: Some(cost),
        diagnostics: vec![SolverIteration {
            iteration: usize::MAX,
            before: cost,
            candidate: Some(FactorCosts { visual: 7., ..cost }),
            accepted: false,
            reason: "Function tolerance reached",
            predicted_decrease: Some(-1.234_567_890_123_456_7e-200),
            quality: Some(-1.234_567_890_123_456_7e-200),
            pose_steps: vec![PoseStep {
                key: 5,
                translation: [-1.234_567_890_123_456_7e-200; 3],
                rotation: [-1.234_567_890_123_456_7e-200; 3],
            }],
        }],
        report: "Optimization time callback terminated successfully".into(),
        ..Default::default()
    };
    let mut records = frame_records(&[
        FrameEvent::Solve {
            frame_ns: i64::MAX.into(),
            solve: "newest",
            pose_key: 5,
            summary,
        },
        FrameEvent::State {
            frame_ns: i64::MAX.into(),
            stage: "after_frontend",
            position: [-1.234_567_890_123_456_7e-200; 3],
            quaternion_xyzw: [-1.234_567_890_123_456_7e-200; 4],
            speed_bias: [-1.234_567_890_123_456_7e-200; 9],
        },
    ]);
    for (index, record) in records.iter().enumerate() {
        assert_eq!(record["ri"], serde_json::json!(index));
        assert_eq!(record["rn"], serde_json::json!(records.len()));
    }
    for record in &mut records {
        if record.get("solve_index").is_some() {
            record["solve_index"] = serde_json::json!(usize::MAX);
        }
        record["ri"] = serde_json::json!(usize::MAX);
        record["rn"] = serde_json::json!(usize::MAX);
    }
    for record in &records {
        let encoded = record.to_string();
        assert!(encoded.len() < 512, "{} bytes: {encoded}", encoded.len());
    }
    let after = records.iter().find(|r| r["stage"] == "after").unwrap();
    assert_eq!(after["visual"], serde_json::json!(cost.visual));
    assert!(records.iter().any(|r| r["kind"] == "solver_stop"));
    let state = state_records(
        i64::MAX.into(),
        "after_frontend",
        usize::MAX,
        &[-1.234_567_890_123_456_7e-200; 3],
        &[-1.234_567_890_123_456_7e-200; 4],
        &[-1.234_567_890_123_456_7e-200; 9],
    );
    for record in state {
        assert!(record.to_string().len() < 512);
    }
}
