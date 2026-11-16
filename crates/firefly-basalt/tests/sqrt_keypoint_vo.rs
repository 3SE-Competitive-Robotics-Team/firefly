use firefly_basalt::{
    optical_flow::{AffineCompact2, OpticalFlowResult},
    utils::vio_config::{LinearizationType, VioConfig},
    vi_estimator::{
        sqrt_keypoint_vo::SqrtKeypointVoEstimator,
        vio_estimator::{VioEstimatorEngine, VioStatus, VioWorkerOutputs},
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::pinhole_camera::PinholeCamera;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};
use std::{collections::BTreeMap, sync::Arc};

fn retained_queries<S: BasaltScalar>() {
    use firefly_basalt::{
        utils::imu_types::PoseStateWithLin,
        vi_estimator::sqrt_keypoint_vio::SqrtKeypointVioEstimator,
    };
    let config = VioConfig::default();
    let calibration = Calibration::default();
    let mut vo = SqrtKeypointVoEstimator::<S>::new(&calibration, &config);
    let mut vio =
        SqrtKeypointVioEstimator::<S>::new(&Vector3::new(0., 0., -9.81), &calibration, &config);
    let pose = Isometry3::translation(1., 2., 3.);
    for engine in [&mut vo as &mut dyn VioEstimatorEngine, &mut vio] {
        engine.initialize_state(
            17,
            pose,
            Vector3::repeat(0.5),
            Vector3::repeat(0.25),
            Vector3::repeat(0.125),
        );
    }
    let pose_only = Isometry3::translation(4., 5., 6.);
    for base in [&mut vo.base, &mut vio.ba] {
        base.frame_poses
            .insert(23, PoseStateWithLin::new(23, pose_only, false).cast::<S>());
    }
    let state = VioEstimatorEngine::state_at(&vio, 17).unwrap();
    assert_eq!(state.state.pose.t_ns, 17);
    assert_eq!(state.state.pose.t_w_i, pose);
    assert_eq!(state.state.vel_w_i, Vector3::repeat(0.5));
    assert_eq!(state.bias_gyro, Vector3::repeat(0.25));
    assert_eq!(state.bias_accel, Vector3::repeat(0.125));
    assert!(
        vo.state_at(17).is_none(),
        "VO does not estimate velocity or IMU biases"
    );
    for engine in [&vo as &dyn VioEstimatorEngine, &vio] {
        assert_eq!(engine.pose_at(17), Some(pose));
        assert_eq!(engine.pose_at(23), Some(pose_only));
        assert!(
            engine.state_at(23).is_none(),
            "a pose-only frame has no velocity or bias estimate"
        );
        assert!(engine.state_at(99).is_none());
        assert!(
            engine.pose_at(99).is_none(),
            "an absent pose must not become an identity pose"
        );
        assert_eq!(
            engine.collect_poses(),
            BTreeMap::from([(17, pose), (23, pose_only)])
        );
    }
    assert_eq!(vo.latest_timestamp_ns(), 17);
    assert_eq!(vio.latest_timestamp_ns(), 17);
    assert_eq!(vo.latest_pose().translation.vector[0].as_f64(), 1.);
    assert_eq!(vio.latest_velocity()[0].as_f64(), 0.5);
}

#[test]
fn retained_queries_distinguish_complete_pose_only_and_absent_frames_in_both_precisions() {
    retained_queries::<f32>();
    retained_queries::<f64>();
}

#[test]
fn worker_join_preserves_initialized_state_and_rejects_restart() {
    use firefly_basalt::vi_estimator::vio_estimator::VioEstimatorFactory;
    for use_imu in [false, true] {
        for use_double in [false, true] {
            let mut estimator = VioEstimatorFactory::create(
                &VioConfig::default(),
                &Calibration::default(),
                &Vector3::new(0., 0., -9.81),
                use_imu,
                use_double,
            )
            .unwrap();
            let pose = Isometry3::translation(1., 2., 3.);
            estimator
                .initialize_state(
                    17,
                    pose,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .unwrap();
            assert!(estimator.engine().is_err());
            // VIO worker 启动时阻塞在第一条 IMU 测量上,所以测量必须先于 vision 的结束标记送达。
            if use_imu {
                estimator
                    .imu_data_queue()
                    .send(Some(Arc::new(firefly_basalt::imu::imu_types::ImuData {
                        t_ns: 17,
                        accel: Vector3::new(0., 0., 9.81),
                        gyro: Vector3::zeros(),
                    })))
                    .unwrap();
            }
            estimator.vision_data_queue().send(None).unwrap();
            // worker 取到 vision 的结束标记后立即退出,并随之丢弃 IMU 接收端;
            // 这一发是否送达取决于调度,IMU 的结束标记不参与 worker 契约。
            let _ = estimator.imu_data_queue().send(None);
            estimator.maybe_join().unwrap();
            let engine = estimator.engine().unwrap();
            if use_imu {
                assert_eq!(engine.state_at(17).unwrap().state.pose.t_w_i, pose);
            } else {
                assert!(engine.state_at(17).is_none());
            }
            assert_eq!(engine.pose_at(17), Some(pose));
            assert!(engine.state_at(18).is_none());
            assert!(engine.pose_at(18).is_none());
            assert_eq!(engine.collect_poses()[&17], pose);
            assert!(
                estimator
                    .initialize(Vector3::zeros(), Vector3::zeros())
                    .is_err()
            );
            estimator.maybe_join().unwrap();
        }
    }
}

fn stereo_sequence<S: BasaltScalar>(mode: LinearizationType, tolerance: f64) {
    let camera = GenericCamera::Pinhole(PinholeCamera::new(Vector4::new(300., 300., 320., 240.)));
    let calibration = Calibration {
        intrinsics: vec![camera.clone(), camera],
        t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.2, 0., 0.)],
        ..Calibration::default()
    };
    let config = VioConfig {
        vio_linearization_type: mode,
        vio_max_kfs: 3,
        vio_min_frames_after_kf: -1,
        vio_new_kf_keypoints_thresh: 2.,
        vio_max_iterations: 8,
        ..VioConfig::default()
    };
    let mut estimator = SqrtKeypointVoEstimator::<S>::new(&calibration, &config);
    estimator.initialize_state(
        0,
        Isometry3::identity(),
        Vector3::zeros(),
        Vector3::zeros(),
        Vector3::zeros(),
    );
    let outputs = VioWorkerOutputs::default();
    let status = VioStatus::new(3, 3);
    let scene: Vec<_> = (0..18)
        .map(|i| {
            Vector3::new(
                f64::from(i % 6) * 0.2 - 0.5,
                f64::from(i / 6) * 0.2 - 0.2,
                2.5 + f64::from(i % 3) * 0.25,
            )
        })
        .collect();
    for frame in 0..6 {
        let x = f64::from(frame) * 0.02;
        let observations = (0..2)
            .map(|cam| {
                scene
                    .iter()
                    .enumerate()
                    .map(|(id, p)| {
                        let pixel = Vector2::new(
                            (300. * (p.x - x - f64::from(cam) * 0.2) / p.z + 320.) as f32,
                            (300. * p.y / p.z + 240.) as f32,
                        );
                        (
                            id,
                            AffineCompact2 {
                                translation: pixel,
                                ..Default::default()
                            },
                        )
                    })
                    .collect::<BTreeMap<_, _>>()
            })
            .collect();
        let flow = Arc::new(OpticalFlowResult {
            t_ns: i64::from(frame) * 50_000_000,
            observations,
            ..Default::default()
        });
        estimator
            .measure(flow, frame > 0, &outputs, &status)
            .unwrap();
        assert_eq!(estimator.base.lmdb.num_landmarks(), scene.len());
        let time = i64::from(frame) * 50_000_000;
        let pose = estimator.base.frame_poses[&time].pose();
        assert!(
            (pose.translation.vector.x.as_f64() - x).abs() < tolerance,
            "{mode:?} frame={frame}: {} vs {x}",
            pose.translation.vector.x.as_f64()
        );
        assert!(pose.rotation.angle().as_f64().abs() < tolerance);
    }
    assert!(estimator.base.frame_poses.len() <= 3);
    assert_eq!(
        estimator.marg_data.h.ncols(),
        estimator.marg_data.order.total_size
    );
}
#[test]
fn stereo_vo_window_double_all_linearizations() {
    for mode in [
        LinearizationType::AbsQr,
        LinearizationType::AbsSc,
        LinearizationType::RelSc,
    ] {
        stereo_sequence::<f64>(mode, 1e-4);
    }
}
#[test]
fn stereo_vo_window_float_all_linearizations() {
    for mode in [
        LinearizationType::AbsQr,
        LinearizationType::AbsSc,
        LinearizationType::RelSc,
    ] {
        stereo_sequence::<f32>(mode, 1e-4);
    }
}
