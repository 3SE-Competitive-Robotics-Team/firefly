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
