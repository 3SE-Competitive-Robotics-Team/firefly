use firefly_basalt::{
    utils::vio_config::VioConfig,
    vi_estimator::{
        sqrt_keypoint_vio::SqrtKeypointVioEstimator, vio_estimator::VioEstimatorEngine,
    },
};
use firefly_calibration::calibration::Calibration;
use nalgebra::{Isometry3, Vector3};

fn check_initial_bias_prior(is_sqrt: bool, gyro: bool) {
    let config = VioConfig {
        vio_sqrt_marg: is_sqrt,
        vio_init_bg_weight: 10_000.,
        vio_init_ba_weight: 100.,
        ..VioConfig::default()
    };
    let mut estimator = SqrtKeypointVioEstimator::<f64>::new(
        &Vector3::new(0., 0., -9.81),
        &Calibration::default(),
        &config,
    );
    estimator.initialize_state(
        0,
        Isometry3::identity(),
        Vector3::zeros(),
        Vector3::zeros(),
        Vector3::zeros(),
    );
    assert_eq!(
        estimator.ba.compute_marg_prior_error(&estimator.marg_data),
        0.
    );

    // Name the physical state being changed; do not repeat its increment indices.
    let initial = estimator.get_state().clone();
    let mut changed = initial.clone();
    let perturbation = Vector3::new(0.01, -0.02, 0.03);
    let information = if gyro {
        changed.bias_gyro += perturbation;
        config.vio_init_bg_weight
    } else {
        changed.bias_accel += perturbation;
        config.vio_init_ba_weight
    };
    estimator
        .ba
        .frame_states
        .get_mut(&0)
        .unwrap()
        .apply_inc(&initial.diff(&changed));

    let actual = estimator.ba.compute_marg_prior_error(&estimator.marg_data);
    // The configured information must penalize its named bias, in either storage mode.
    let expected = 0.5 * information * perturbation.norm_squared();
    assert!(
        (actual - expected).abs() < 1e-12,
        "sqrt={is_sqrt}, gyro={gyro}: prior cost {actual}, expected {expected}"
    );
}

#[test]
fn square_root_prior_uses_gyroscope_bias_information() {
    check_initial_bias_prior(true, true);
}

#[test]
fn square_root_prior_uses_accelerometer_bias_information() {
    check_initial_bias_prior(true, false);
}

#[test]
fn hessian_prior_uses_gyroscope_bias_information() {
    check_initial_bias_prior(false, true);
}

#[test]
fn hessian_prior_uses_accelerometer_bias_information() {
    check_initial_bias_prior(false, false);
}

#[test]
#[expect(
    clippy::too_many_lines,
    reason = "Keeps the analytic trajectory, camera observations and IMU model together as the test oracle"
)]
fn accelerating_stereo_preserves_pose_and_bias_through_marginalization() {
    use firefly_basalt::{
        imu::{imu_types::ImuData, preintegration::IntegratedImuMeasurement},
        optical_flow::{AffineCompact2, OpticalFlowResult},
        vi_estimator::vio_estimator::{VioStatus, VioWorkerOutputs},
    };
    use firefly_calibration::camera::generic_camera::GenericCamera;
    use nalgebra::{Matrix2, Translation3, UnitQuaternion, Vector2, Vector4};
    use std::sync::Arc;

    let focal = 120.0 / (70.88_f64.to_radians() * 0.5).tan();
    let mut camera = GenericCamera::from_name("pinhole");
    camera.set_from_init(Vector4::new(focal, focal, 159.5, 119.5));
    let calibration = Calibration::<f64> {
        intrinsics: vec![camera.clone(), camera],
        t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.05, 0., 0.)],
        imu_update_rate: 1000.,
        gyro_noise_std: Vector3::repeat(0.0002),
        accel_noise_std: Vector3::repeat(0.002),
        gyro_bias_std: Vector3::repeat(0.000_019),
        accel_bias_std: Vector3::repeat(0.003),
        ..Calibration::default()
    };
    let config = VioConfig {
        vio_init_yaw_weight: Some(100.),
        vio_init_bg_weight: 10_000.,
        vio_init_ba_weight: 100.,
        ..VioConfig::default()
    };
    let gravity = Vector3::new(0., 0., -9.81);
    let attitude = |t: f64| UnitQuaternion::from_euler_angles(0., 0.1, 0.1 * t);
    let position = |t: f64| Vector3::new(0.3 * t * t, 0., 0.);
    let mut estimator = SqrtKeypointVioEstimator::<f64>::new(&gravity, &calibration, &config);
    estimator.initialize_state(
        0,
        Isometry3::from_parts(Translation3::identity(), attitude(0.)),
        Vector3::zeros(),
        Vector3::zeros(),
        Vector3::zeros(),
    );
    let status = VioStatus::new(config.vio_max_states as usize, config.vio_max_kfs as usize);
    let outputs = VioWorkerOutputs::default();
    for frame in 0..=400 {
        let time = frame * 20_000_000;
        let t = time as f64 * 1e-9;
        let body = Isometry3::from_parts(Translation3::from(position(t)), attitude(t));
        let observations = (0..2)
            .map(|camera| {
                let camera_from_world = (body * calibration.t_i_c[camera]).inverse();
                (0..1200)
                    .filter_map(|id| {
                        let world = nalgebra::Point3::new(
                            (id % 100) as f64 * 0.3 - 3.,
                            (id / 100) as f64 * 0.3 - 1.5,
                            3. + (id % 3) as f64 * 0.3,
                        );
                        let local = camera_from_world * world;
                        let pixel = Vector2::new(
                            focal * local.x / local.z + 159.5,
                            focal * local.y / local.z + 119.5,
                        );
                        (local.z > 0.
                            && pixel.x > 5.
                            && pixel.x < 315.
                            && pixel.y > 5.
                            && pixel.y < 235.)
                            .then_some((
                                id,
                                AffineCompact2 {
                                    linear: Matrix2::identity(),
                                    translation: pixel.cast::<f32>(),
                                },
                            ))
                    })
                    .collect()
            })
            .collect();
        let flow = Arc::new(OpticalFlowResult {
            t_ns: time,
            observations,
            ..OpticalFlowResult::default()
        });
        let measurement = (frame > 0).then(|| {
            let state = estimator.get_state();
            let mut measurement =
                IntegratedImuMeasurement::new(time - 20_000_000, state.bias_gyro, state.bias_accel);
            for sample in 1..=20 {
                let sample_time = time - 20_000_000 + sample * 1_000_000;
                let midpoint = sample_time as f64 * 1e-9 - 0.0005;
                measurement.integrate(
                    &ImuData {
                        t_ns: sample_time,
                        accel: attitude(midpoint).inverse() * (Vector3::new(0.6, 0., 0.) - gravity),
                        gyro: attitude(0.).inverse() * Vector3::new(0., 0., 0.1),
                    },
                    &Vector3::repeat(0.004),
                    &Vector3::repeat(0.00004),
                );
            }
            measurement
        });
        estimator
            .measure(&flow, measurement, &outputs, &status)
            .unwrap();
        let state = estimator.get_state();
        assert!(
            (state.state.pose.t_w_i.translation.vector - position(t)).norm() < 1e-3,
            "frame {frame}: position {:?}, expected {:?}, bias {:?}, velocity {:?}, rotation {:?}",
            state.state.pose.t_w_i.translation.vector,
            position(t),
            state.bias_accel,
            state.state.vel_w_i,
            (state.state.pose.t_w_i.rotation * attitude(t).inverse()).scaled_axis()
        );
        assert!((state.state.vel_w_i - Vector3::new(0.6 * t, 0., 0.)).norm() < 1e-3);
        assert!(
            state.bias_accel.norm() < 1e-3,
            "frame {frame}: bias {:?}",
            state.bias_accel
        );
        assert!((state.state.pose.t_w_i.rotation * attitude(t).inverse()).angle() < 1e-3);
    }
    assert!(estimator.optimization_started());
    assert!(estimator.ba.frame_states.len() < config.vio_max_states as usize);
    assert!(
        estimator
            .marg_data
            .order
            .abs_order_map
            .keys()
            .all(|&time| time > 0),
        "initial keyframe still retained: {:?}",
        estimator.kf_ids
    );
}
