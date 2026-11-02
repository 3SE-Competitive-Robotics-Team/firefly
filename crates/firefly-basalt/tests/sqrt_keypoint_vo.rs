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

#[test]
fn worker_join_preserves_initialized_state_and_rejects_restart() {
    use firefly_basalt::vi_estimator::vio_estimator::VioEstimatorFactory;
    for use_imu in [false, true] {
        for use_double in [false, true] {
            let mut estimator = VioEstimatorFactory::get_vio_estimator(
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
            assert_eq!(engine.get_state_at(17).state.pose.t_w_i, pose);
            assert_eq!(engine.get_all_poses_map()[&17], pose);
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
