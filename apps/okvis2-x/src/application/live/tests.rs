// SPDX-License-Identifier: BSD-3-Clause
use firefly_pubsub::{
    attitude::AttitudeMessage,
    camera::DepthImageMessage,
    imu::{IMU_SUBSCRIPTION, ImuMessage},
    navigation::NavigationMessage,
    node::create_node,
    odom::OdomMessage,
    publish::Publisher,
    subscriber::Subscriber,
};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};

use super::{
    config::Config,
    input::Pairs,
    output::{self, Output},
};
use crate::{
    frames::frame_typedefs::{MapPointVector, StateId},
    kinematics::transformation::Transformation,
    runtime::{
        threaded_slam::PublicationData,
        vi_interface::{BackendTrackingState, State, TrackingQuality, TrackingState},
    },
};
use firefly_pubsub::camera::{GrayImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH};
use nalgebra::{UnitQuaternion, Vector3};
use std::{collections::BTreeMap, sync::Arc};
fn publication(time: f64, id: u64, pose: Transformation, revision: u64) -> PublicationData {
    let state = State::new(
        pose,
        Vector3::zeros(),
        (Vector3::zeros(), Vector3::zeros()),
        Vector3::zeros(),
        output::time(time).unwrap(),
        StateId::new(id),
        true,
    );
    PublicationData {
        tracking_state: TrackingState {
            id: state.id,
            is_keyframe: true,
            is_lidar_keyframe: false,
            tracking_quality: TrackingQuality::Good,
            backend: BackendTrackingState {
                revision,
                ..Default::default()
            },
            current_keyframe_id: state.id,
        },
        state,
        updated_states: Arc::new(BTreeMap::new()),
        landmarks: Arc::new(MapPointVector::new()),
    }
}
fn frame(time: f64, id: i32) -> Box<GrayImageMessage> {
    let mut message = Box::<GrayImageMessage>::default();
    message.timestamp = time;
    message.sensor_id = id;
    message.width = IMAGE_WIDTH as u32;
    message.height = IMAGE_HEIGHT as u32;
    message.data.fill(id as u8 + 17);
    message
}
fn imu(output: &mut Output, time: f64) -> Option<super::output::Sample> {
    output
        .imu(
            super::output::time(time).unwrap(),
            Vector3::new(0.0, 0.0, 9.81),
            Vector3::zeros(),
        )
        .unwrap()
}
#[test]
fn pairing_uses_owned_images_and_requires_both_cameras_and_imu_coverage() {
    let mut pairs = Pairs::new(0.001);
    let mut left = frame(1.0, 0);
    pairs.push(&left, 0).unwrap();
    left.data.fill(99);
    assert!(pairs.next(Some(output::time(2.0).unwrap()), 0.0).is_none());
    pairs.push(&frame(1.0, 1), 1).unwrap();
    assert!(pairs.next(Some(output::time(0.9).unwrap()), 0.0).is_none());
    let (time, images) = pairs.next(Some(output::time(1.0).unwrap()), 0.0).unwrap();
    assert_eq!(time, output::time(1.0).unwrap());
    assert_eq!(images[&0].as_matrix_u8().unwrap().data[0], 17);
    assert!(pairs.next(Some(output::time(2.0).unwrap()), 0.0).is_none());
    pairs.push(&frame(1.0, 0), 0).unwrap();
    pairs.push(&frame(1.0, 1), 1).unwrap();
    assert!(pairs.next(Some(output::time(2.0).unwrap()), 0.0).is_none());
    pairs.push(&frame(2.0, 0), 0).unwrap();
    pairs.push(&frame(2.05, 1), 1).unwrap();
    pairs.push(&frame(2.05, 0), 0).unwrap();
    assert_eq!(
        pairs.next(Some(output::time(2.1).unwrap()), 0.0).unwrap().0,
        output::time(2.05).unwrap()
    );
}
#[test]
fn sensor_configuration_rejects_unknown_fields_nonfinite_geometry_and_invalid_noise() {
    assert!(Config::default().validate().is_ok());
    assert!(toml::from_str::<Config>("unused_key = true").is_err());
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut config = Config::default();
        config.imu.gyro_noise = value;
        assert!(config.validate().is_err());
    }
    let mut config = Config::default();
    config.right.extrinsic.rotation = [0.0; 4];
    assert!(config.validate().is_err());
    assert!(output::time(f64::MAX).is_err());
    assert!(output::time(f64::NAN).is_err());
    let mut bad = frame(1.0, 0);
    bad.width = 0;
    assert!(Pairs::new(0.001).push(&bad, 0).is_err());
    assert!(Pairs::new(0.001).push(&frame(1.0, 1), 0).is_err());
}
#[test]
fn posterior_imu_prediction_preserves_stationary_geometry_and_never_renews_a_stale_posterior() {
    let config = Config {
        max_posterior_age: 0.05,
        max_imu_gap: 0.02,
        ..Default::default()
    };
    let mut predictor = Output::new(config, 7).unwrap();
    assert!(imu(&mut predictor, 1.0).is_none());
    predictor
        .posterior(&publication(1.0, 1, Transformation::identity(), 0))
        .unwrap();
    for index in 1..=10 {
        let sample = imu(&mut predictor, 1.0 + f64::from(index) * 0.01);
        if index <= 5 {
            let sample = sample.unwrap();
            assert!(sample.odom.position_z.abs() < 1e-12);
            assert!(sample.odom.velocity_z.abs() < 1e-12);
            let attitude = sample.attitude.unwrap();
            assert_eq!(attitude.session, 7);
            assert_eq!(attitude.posterior_time, 1.0);
        } else {
            assert!(sample.is_none());
        }
    }
    predictor
        .posterior(&publication(1.08, 2, Transformation::identity(), 0))
        .unwrap();
    assert!(imu(&mut predictor, 1.11).is_some());
}
#[test]
fn loop_commit_changes_map_coordinates_atomically_without_moving_local_odometry() {
    let mut predictor = Output::new(Config::default(), 1).unwrap();
    imu(&mut predictor, 1.0);
    predictor
        .posterior(&publication(1.0, 1, Transformation::identity(), 0))
        .unwrap();
    let before = imu(&mut predictor, 1.01).unwrap();
    let correction = Transformation::new(
        Vector3::new(3.0, -2.0, 0.4),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.2).into_inner(),
    );
    let mut changed = publication(1.01, 2, correction.clone(), 1);
    changed.updated_states = Arc::new(BTreeMap::from([(
        StateId::new(1),
        publication(1.0, 1, correction.clone(), 1).state,
    )]));
    predictor.posterior(&changed).unwrap();
    let after = imu(&mut predictor, 1.02).unwrap();
    assert!((after.odom.position_x - before.odom.position_x).abs() < 1e-12);
    assert!((after.odom.position_y - before.odom.position_y).abs() < 1e-12);
    assert!((after.odom.quat_w - 1.0).abs() < 1e-12);
    assert_eq!(after.navigation.revision, 1);
    assert!((after.corrected.position_x - 3.0).abs() < 1e-12);
    let expected = after
        .navigation
        .alignment()
        .unwrap()
        .compose(&after.odom.body_pose(firefly_base::FrameId::ODOM).unwrap())
        .unwrap();
    assert!(
        (expected.matrix()
            - after
                .corrected
                .body_pose(firefly_base::FrameId::MAP)
                .unwrap()
                .matrix())
        .amax()
            < 1e-12
    );
    assert!(
        predictor
            .posterior(&publication(1.03, 3, correction, 2))
            .is_err()
    );
}
#[test]
fn lost_tracking_or_uncovered_imu_gap_stops_predictions_without_resetting_session() {
    let mut predictor = Output::new(Config::default(), 1).unwrap();
    imu(&mut predictor, 1.0);
    let mut posterior = publication(1.0, 1, Transformation::identity(), 0);
    predictor.posterior(&posterior).unwrap();
    // 上游不计算协方差，姿态照常发布，不做置信度门控。
    assert!(imu(&mut predictor, 1.01).unwrap().attitude.is_some());
    let session_before = predictor.session_id();
    posterior.tracking_state.tracking_quality = TrackingQuality::Lost;
    predictor.posterior(&posterior).unwrap();
    assert!(imu(&mut predictor, 1.02).is_none());
    assert_eq!(
        predictor.session_id(),
        session_before,
        "跟踪丢失只停预测，不得改动进程会话标识"
    );
    let posterior = publication(1.02, 2, Transformation::identity(), 0);
    predictor.posterior(&posterior).unwrap();
    assert!(imu(&mut predictor, 1.1).is_none());
    assert!(imu(&mut predictor, 1.11).is_none());
}

#[test]
fn prediction_reuses_calibration_across_samples_and_posterior_replay() {
    let mut config = Config::default();
    config.imu.extrinsic.translation = [0.1, -0.2, 0.3];
    let rotation = UnitQuaternion::from_euler_angles(0.2, -0.1, 0.3);
    let q = rotation.quaternion();
    config.imu.extrinsic.rotation = [q.i, q.j, q.k, q.w];
    config.imu.accelerometer_scale = [2., 0.5, 1.];
    let sensor_pose = config.imu.extrinsic.pose().unwrap();
    let raw_acceleration = sensor_pose
        .inverse()
        .rotate_vector(&Vector3::new(0., 0., config.imu.gravity))
        .component_div(&Vector3::from(config.imu.accelerometer_scale));
    let mut predictor = Output::new(config, 3).unwrap();
    for index in 0..=40 {
        let time = 1. + index as f64 * 0.001;
        if index == 10 || index == 30 {
            predictor
                .posterior(&publication(time - 0.005, index, sensor_pose.clone(), 0))
                .unwrap();
        }
        let sample = predictor
            .imu(
                output::time(time).unwrap(),
                raw_acceleration,
                Vector3::zeros(),
            )
            .unwrap();
        if index < 10 {
            assert!(sample.is_none());
        } else {
            let sample = sample.unwrap();
            assert!(sample.odom.position_x.abs() < 1e-12);
            assert!(sample.odom.position_y.abs() < 1e-12);
            assert!(sample.odom.position_z.abs() < 1e-12);
            assert!(sample.odom.velocity_z.abs() < 1e-12);
            assert!((sample.odom.quat_w - 1.).abs() < 1e-12);
            assert_eq!(sample.attitude.unwrap().session, 3);
        }
    }
    let mut invalid = Config::default();
    invalid.imu.extrinsic.rotation = [0.; 4];
    assert!(Output::new(invalid, 1).is_err());
}
#[test]
fn marginal_covariance_matches_analytic_independent_priors_and_rejects_unobserved_parameters() {
    use crate::ceres::{
        speed_and_bias_error::SpeedAndBiasError,
        speed_and_bias_parameter_block::SpeedAndBiasParameterBlock,
        vi_graph::{GraphFactor, GraphProblem, ManifoldKind, Parameter, RustGraphSolver},
    };
    use nalgebra::SVector;
    use std::{cell::RefCell, rc::Rc};
    let block = Parameter::SpeedBias(Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
        SVector::zeros(),
        1,
        output::time(0.0).unwrap(),
    ))));
    let mut problem = GraphProblem::default();
    problem.register(block.clone(), ManifoldKind::Euclidean);
    assert!(
        RustGraphSolver::marginal_covariance(&problem, std::slice::from_ref(&block))
            .unwrap()
            .is_none()
    );
    problem.add(
        GraphFactor::SpeedBiasPrior(Box::new(SpeedAndBiasError::from_variances(
            SVector::zeros(),
            0.25,
            0.04,
            0.09,
        ))),
        vec![block.clone()],
        None,
    );
    let covariance = RustGraphSolver::marginal_covariance(&problem, &[block])
        .unwrap()
        .unwrap();
    for index in 0..9 {
        let expected = [0.25, 0.04, 0.09][index / 3];
        assert!((covariance[(index, index)] - expected).abs() < 1e-12);
        for other in 0..index {
            assert_eq!(covariance[(index, other)], 0.0);
        }
    }
}

#[test]
fn real_ipc_transports_sensor_inputs_and_publishes_one_consistent_navigation_snapshot() {
    let config = Config {
        diagnostics: false,
        ..Default::default()
    };
    let prefix = format!(
        "Firefly/TestOkvis/{}",
        jiff::Timestamp::now().as_nanosecond()
    );
    let topics = super::input::Topics {
        imu: format!("{prefix}/imu"),
        left: format!("{prefix}/left"),
        right: format!("{prefix}/right"),
        depth: format!("{prefix}/depth"),
        odometry: format!("{prefix}/odom"),
        corrected: format!("{prefix}/corrected"),
        navigation: format!("{prefix}/nav"),
        attitude: format!("{prefix}/attitude"),
    };
    let clients = IpcClients::new(&topics);
    let parameters = config.parameters().unwrap();
    let mut estimator = super::create_estimator(
        &parameters,
        &Config::submaps(),
        super::load_vocabulary(None).unwrap(),
        Arc::default(),
        false,
    )
    .unwrap();
    let handle = estimator.sensor_input_handle();
    let (posterior, publications) = mpsc::sync_channel(8);
    let (depths, depth_rx) = mpsc::sync_channel(4);
    let (wake, ready) = mpsc::sync_channel(1);
    let stop = Arc::new(AtomicBool::new(false));
    let stopping = stop.clone();
    let ports = super::input::Ports {
        config,
        sensors: handle.clone(),
        publications,
        depths,
        wake,
        stop: stopping,
        topics,
    };
    let worker = std::thread::spawn(move || super::input::run(ports));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        clients.exercise(&handle, &posterior, &depth_rx, &ready);
    }));
    stop.store(true, Ordering::Release);
    let transport = worker.join().unwrap();
    let shutdown = estimator.shutdown();
    if let Err(panic) = result {
        std::panic::resume_unwind(panic);
    }
    transport.unwrap();
    shutdown.unwrap();
}

struct IpcClients {
    node: firefly_pubsub::node::IpcNode,
    imu_pub: Publisher<ImuMessage>,
    left: Publisher<GrayImageMessage>,
    right: Publisher<GrayImageMessage>,
    depth: Publisher<DepthImageMessage>,
    odom: Subscriber<OdomMessage>,
    nav: Subscriber<NavigationMessage>,
    attitude: Subscriber<AttitudeMessage>,
}
impl IpcClients {
    fn new(topics: &super::input::Topics) -> Self {
        let node = create_node().unwrap();
        let imu_pub = Publisher::<ImuMessage>::with_topic_and_buffer(
            &node,
            &topics.imu,
            IMU_SUBSCRIPTION.service_max_buffer_size,
        )
        .unwrap();
        let left = Publisher::<GrayImageMessage>::with_topic(&node, &topics.left).unwrap();
        let right = Publisher::<GrayImageMessage>::with_topic(&node, &topics.right).unwrap();
        let depth = Publisher::<DepthImageMessage>::with_topic(&node, &topics.depth).unwrap();
        let odom = Subscriber::<OdomMessage>::with_topic(&node, &topics.odometry).unwrap();
        let nav = Subscriber::<NavigationMessage>::with_topic(&node, &topics.navigation).unwrap();
        let attitude = Subscriber::<AttitudeMessage>::with_topic(&node, &topics.attitude).unwrap();
        Self {
            node,
            imu_pub,
            left,
            right,
            depth,
            odom,
            nav,
            attitude,
        }
    }
    fn exercise(
        &self,
        handle: &crate::runtime::threaded_slam::SensorInputHandle,
        posterior: &std::sync::mpsc::SyncSender<PublicationData>,
        depth_rx: &std::sync::mpsc::Receiver<super::input::Depth>,
        ready: &std::sync::mpsc::Receiver<()>,
    ) {
        let message = ImuMessage {
            timestamp: 1.0,
            linear_acceleration_z: 9.81,
            ..Default::default()
        };
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.imu_pub.publish_counted(message).unwrap().0 == 0 {
            assert!(Instant::now() < deadline, "IPC input startup deadline");
            self.node.wait(Duration::from_millis(1)).unwrap();
        }
        while handle.pending_measurements().unwrap()[0] == 0 {
            assert!(Instant::now() < deadline);
            self.node.wait(Duration::from_millis(1)).unwrap();
        }
        assert!(self.odom.receive().unwrap().is_none());
        self.left
            .publish_with(|m| {
                m.timestamp = 1.0;
                m.sensor_id = 0;
                m.width = IMAGE_WIDTH as u32;
                m.height = IMAGE_HEIGHT as u32;
                m.data.fill(17);
            })
            .unwrap();
        self.right
            .publish_with(|m| {
                m.timestamp = 1.0;
                m.sensor_id = 1;
                m.width = IMAGE_WIDTH as u32;
                m.height = IMAGE_HEIGHT as u32;
                m.data.fill(18);
            })
            .unwrap();
        ready.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(handle.pending_measurements().unwrap()[1], 1);
        self.depth
            .publish_with(|m| {
                m.timestamp = 1.0;
                m.sensor_id = 0;
                m.width = IMAGE_WIDTH as u32;
                m.height = IMAGE_HEIGHT as u32;
                m.data.fill(2.5);
                m.data[0] = f32::NAN;
            })
            .unwrap();
        let received_depth = depth_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(received_depth.image.as_matrix_f32().unwrap().data[0], 0.0);
        assert_eq!(received_depth.image.as_matrix_f32().unwrap().data[1], 2.5);
        posterior
            .send(publication(1.0, 1, Transformation::identity(), 0))
            .unwrap();
        for step in 1..=4 {
            self.imu_pub
                .publish(ImuMessage {
                    timestamp: 1.0 + f64::from(step) * 0.01,
                    ..message
                })
                .unwrap();
            self.node.wait(Duration::from_millis(3)).unwrap();
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let snapshot = loop {
            if let Some(message) = self.nav.receive().unwrap() {
                break *message;
            }
            assert!(Instant::now() < deadline, "navigation output deadline");
            self.node.wait(Duration::from_millis(1)).unwrap();
        };
        let local = *self.odom.receive().unwrap().unwrap();
        let attitude = *self.attitude.receive().unwrap().unwrap();
        assert_eq!(snapshot.odom.timestamp, local.timestamp);
        assert_eq!(attitude.timestamp, local.timestamp);
        assert_eq!(attitude.posterior_time, 1.0);
        assert_eq!(snapshot.revision, 0);
        assert!(local.is_initialized);
        assert!(local.position_z.abs() < 1e-12);
    }
}

#[test]
fn marginal_covariance_includes_relative_pose_coupling_and_fixed_gauge() {
    use crate::ceres::{
        pose_error::PoseError,
        pose_parameter_block::PoseParameterBlock,
        relative_pose_error::RelativePoseError,
        vi_graph::{GraphFactor, GraphProblem, ManifoldKind, Parameter, RustGraphSolver},
    };
    use nalgebra::{Isometry3, SMatrix};
    use std::{cell::RefCell, rc::Rc};
    let first = Parameter::Pose(Rc::new(RefCell::new(PoseParameterBlock::new(
        &Transformation::identity(),
        1,
        output::time(0.0).unwrap(),
    ))));
    let second = Parameter::Pose(Rc::new(RefCell::new(PoseParameterBlock::new(
        &Transformation::identity(),
        2,
        output::time(0.0).unwrap(),
    ))));
    let mut problem = GraphProblem::default();
    problem.register(first.clone(), ManifoldKind::Pose);
    problem.register(second.clone(), ManifoldKind::Pose);
    problem.add(
        GraphFactor::PosePrior(Box::new(PoseError::new(
            Isometry3::identity(),
            SMatrix::<f64, 6, 6>::identity() * 4.0,
        ))),
        vec![first.clone()],
        None,
    );
    problem.add(
        GraphFactor::RelativePose(Box::new(RelativePoseError::new(
            SMatrix::<f64, 6, 6>::identity() * 2.0,
            Isometry3::identity(),
        ))),
        vec![first.clone(), second.clone()],
        None,
    );
    let joint = RustGraphSolver::marginal_covariance(&problem, &[first.clone(), second.clone()])
        .unwrap()
        .unwrap();
    for axis in 0..6 {
        assert!((joint[(axis, axis)] - 0.25).abs() < 1e-12);
        assert!((joint[(axis, axis + 6)] - 0.25).abs() < 1e-12);
        assert!((joint[(axis + 6, axis + 6)] - 0.75).abs() < 1e-12);
    }
    first.set_fixed(true);
    second.set_fixed(true);
    let fixed = RustGraphSolver::marginal_covariance(&problem, &[first, second])
        .unwrap()
        .unwrap();
    assert_eq!(fixed.amax(), 0.0);
}
#[test]
fn embedded_vocabulary_is_valid_without_any_runtime_resource_path() {
    let dictionary = super::load_vocabulary(None).unwrap();
    assert!(!dictionary.empty());
    assert!(firefly_dbow2::templated_vocabulary::TemplatedVocabulary::<crate::frontend::f_brisk::FBrisk>::load_bytes(b"not a vocabulary").is_err());
}

#[test]
fn incomplete_imu_coverage_returns_without_blocking_and_pending_inputs_close_cleanly() {
    use crate::runtime::vi_interface::{ImageMap, ViInterface};
    let config = Config::default();
    let parameters = config.parameters().unwrap();
    let mut estimator = super::create_estimator(
        &parameters,
        &Config::submaps(),
        super::load_vocabulary(None).unwrap(),
        Arc::default(),
        false,
    )
    .unwrap();
    let mut input = estimator.sensor_input_handle();
    for timestamp in [0.47, 0.50] {
        assert!(input.add_imu_measurement(
            output::time(timestamp).unwrap(),
            &Vector3::new(0.0, 0.0, 9.81),
            &Vector3::zeros()
        ));
    }
    let image = Arc::new(
        purecv::core::DynamicMatrix::new_u8(
            IMAGE_HEIGHT,
            IMAGE_WIDTH,
            1,
            vec![17; IMAGE_WIDTH * IMAGE_HEIGHT],
        )
        .unwrap(),
    );
    assert!(input.add_images(
        output::time(0.5).unwrap(),
        &ImageMap::from([(0, image.clone()), (1, image)]),
        &ImageMap::new()
    ));
    let (done, completion) = std::sync::mpsc::channel();
    let (resume, paused) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        done.send(estimator.process_frame()).unwrap();
        paused.recv().unwrap();
        done.send(estimator.shutdown().map(|()| false)).unwrap();
    });
    assert!(
        !completion
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
    );
    assert_eq!(input.pending_measurements().unwrap()[1], 1);
    input.close_inputs();
    resume.send(()).unwrap();
    completion
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    worker.join().unwrap();
}
