//! 独立解析、有限差分与最终输出契约；时间单位秒，角度单位 rad。
#![allow(clippy::float_cmp)]
use firefly_imu::{
    Aid, AttitudeState, Estimator, Matrix6, Options, Sample, gravity_jacobian, right_jacobian,
    transition,
};
use nalgebra::{Matrix3, SVector, UnitQuaternion, Vector3};
fn state() -> AttitudeState {
    AttitudeState {
        rotation: UnitQuaternion::identity(),
        bias: Vector3::zeros(),
        covariance: Matrix6::identity() * 0.001,
    }
}
fn sample(t: f64) -> Sample {
    Sample {
        timestamp: t,
        gyro: Vector3::zeros(),
        accel: Vector3::z() * 9.81,
    }
}
fn initialized() -> Estimator {
    let mut e = Estimator::new(Options::default()).unwrap();
    for i in 0..=210 {
        e.observe(sample(f64::from(i) * 0.01), true).unwrap();
    }
    assert!(e.state().is_some());
    e
}
fn perturb(s: &AttitudeState, delta: SVector<f64, 6>) -> AttitudeState {
    AttitudeState {
        rotation: s.rotation
            * UnitQuaternion::from_scaled_axis(delta.fixed_rows::<3>(0).into_owned()),
        bias: s.bias + delta.fixed_rows::<3>(3),
        covariance: s.covariance,
    }
}
fn error(a: &AttitudeState, b: &AttitudeState) -> SVector<f64, 6> {
    let mut e = SVector::<f64, 6>::zeros();
    e.fixed_rows_mut::<3>(0)
        .copy_from(&(a.rotation.inverse() * b.rotation).scaled_axis());
    e.fixed_rows_mut::<3>(3).copy_from(&(b.bias - a.bias));
    e
}
#[test]
fn zero_time_identity_and_zero_rate_noise_have_closed_form() {
    let mut s = state();
    let original = s.clone();
    let o = Options::default();
    s.predict(Vector3::z(), 0., &o).unwrap();
    assert_eq!(s.covariance, original.covariance);
    assert_eq!(s.rotation, original.rotation);
    let dt = 0.01;
    s.predict(Vector3::zeros(), dt, &o).unwrap();
    for i in 0..3 {
        let angular = 0.001
            + 0.001 * dt * dt
            + o.gyro_density.powi(2) * dt
            + o.bias_walk.powi(2) * dt.powi(3) / 3.;
        let cross = -0.001 * dt - o.bias_walk.powi(2) * dt * dt / 2.;
        assert!((s.covariance[(i, i)] - angular).abs() < 1e-14);
        assert!((s.covariance[(i, i + 3)] - cross).abs() < 1e-14);
        assert!((s.covariance[(i + 3, i + 3)] - (0.001 + o.bias_walk.powi(2) * dt)).abs() < 1e-14);
    }
}
#[test]
fn transition_matches_independent_nominal_dynamics_difference() {
    let o = Options::default();
    let gyro = Vector3::new(0.3, -0.5, 0.8);
    let dt = 0.02;
    let mut before = state();
    before.rotation = UnitQuaternion::from_euler_angles(0.4, -0.2, 0.6);
    before.bias = Vector3::new(0.01, -0.02, 0.03);
    let mut nominal = before.clone();
    nominal.predict(gyro, dt, &o).unwrap();
    let expected = transition(gyro - before.bias, dt);
    let eps = 1e-6;
    for axis in 0..6 {
        let mut d = SVector::<f64, 6>::zeros();
        d[axis] = eps;
        let mut plus = perturb(&before, d);
        let mut minus = perturb(&before, -d);
        plus.rotation *= UnitQuaternion::from_scaled_axis((gyro - plus.bias) * dt);
        minus.rotation *= UnitQuaternion::from_scaled_axis((gyro - minus.bias) * dt);
        let fd = (error(&nominal, &plus) - error(&nominal, &minus)) / (2. * eps);
        assert!((fd - expected.column(axis)).amax() < 1e-9);
    }
}
#[test]
fn gravity_and_injection_reset_jacobians_match_central_differences() {
    let rotation = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.7);
    let predicted = rotation.inverse() * Vector3::z() * 9.81;
    let h = gravity_jacobian(predicted);
    let delta = Vector3::new(0.1, -0.2, 0.15);
    let eps = 1e-6;
    for i in 0..3 {
        let d = Vector3::ith(i, eps);
        let hp = (rotation * UnitQuaternion::from_scaled_axis(d)).inverse() * Vector3::z() * 9.81;
        let hm = (rotation * UnitQuaternion::from_scaled_axis(-d)).inverse() * Vector3::z() * 9.81;
        assert!(((hp - hm) / (2. * eps) - h.column(i)).amax() < 1e-8);
        let reset = UnitQuaternion::from_scaled_axis(-delta);
        let rp = (reset * UnitQuaternion::from_scaled_axis(delta + d)).scaled_axis();
        let rm = (reset * UnitQuaternion::from_scaled_axis(delta - d)).scaled_axis();
        assert!(((rp - rm) / (2. * eps) - right_jacobian(delta).column(i)).amax() < 1e-9);
    }
}
#[test]
fn zero_innovation_still_reduces_tilt_covariance_without_observing_yaw() {
    let mut s = state();
    let p = s.covariance;
    let o = Options::default();
    let gate = s
        .correct_accel(Vector3::z() * o.gravity, 0.01, true, &o)
        .unwrap();
    assert!(gate.accepted);
    assert_eq!(s.rotation, UnitQuaternion::identity());
    assert!(s.covariance[(0, 0)] < p[(0, 0)]);
    assert_eq!(s.covariance[(2, 2)], p[(2, 2)]);
    let r = o.accel_density.powi(2) / 0.01 + o.accel_model_std.powi(2);
    let analytic = 1. / (1. / p[(0, 0)] + o.gravity.powi(2) / r);
    assert!((s.covariance[(0, 0)] - analytic).abs() < 1e-14);
}
#[test]
fn tilted_estimate_corrects_toward_gravity_and_dynamic_acceleration_cannot_level_it() {
    let o = Options::default();
    let mut s = state();
    s.rotation = UnitQuaternion::from_scaled_axis(Vector3::y() * 0.02);
    assert!(
        s.correct_accel(Vector3::z() * 9.81, 0.01, true, &o)
            .unwrap()
            .accepted
    );
    assert!(s.rotation.angle() < 0.002);
    s.rotation = UnitQuaternion::from_scaled_axis(Vector3::y() * 0.35);
    let before = s.clone();
    let acceleration = Vector3::z() * (9.81 / 0.35_f64.cos());
    assert!(
        !s.correct_accel(acceleration, 0.01, false, &o)
            .unwrap()
            .accepted
    );
    assert_eq!(s.rotation, before.rotation);
    assert_eq!(s.covariance, before.covariance);
}
#[test]
fn nis_matches_scalar_analytic_value_and_rejects_outliers_without_forced_recovery() {
    let o = Options::default();
    let mut s = state();
    let original = s.clone();
    let a = Vector3::new(1., 0., 9.81);
    let expected = 1.
        / (9.81_f64.powi(2) * 0.001 + o.accel_density.powi(2) / 0.01 + o.accel_model_std.powi(2));
    assert!((s.correct_accel(a, 0.01, false, &o).unwrap().nis - expected).abs() < 1e-12);
    for _ in 0..100 {
        let gate = s
            .correct_accel(Vector3::new(5., 0., 9.81), 0.01, true, &o)
            .unwrap();
        assert!(gate.nis > o.accel_nis_limit);
        assert!(!gate.accepted);
    }
    assert_eq!(s.covariance, original.covariance);
    // 巨大 P 可以让错误方向通过 NIS；空中禁用条件仍必须拒绝。
    s.covariance = Matrix6::identity() * 100.;
    let gate = s
        .correct_accel(Vector3::new(1., 0., 9.81), 0.01, false, &o)
        .unwrap();
    assert!(gate.nis < o.accel_nis_limit);
    assert!(!gate.accepted);
}
#[test]
fn correlated_identical_estimates_do_not_double_count_information() {
    let mut a = state();
    let original = a.clone();
    for _ in 0..20 {
        assert!(a.intersect(&original).unwrap());
    }
    assert!((a.covariance - original.covariance).amax() < 1e-12);
    let mut b = original.clone();
    b.rotation = UnitQuaternion::from_scaled_axis(Vector3::y() * 0.01);
    b.covariance *= 0.1;
    assert!(a.intersect(&b).unwrap());
    assert!((a.rotation.inverse() * b.rotation).angle() < 1e-8);
    a.validate().unwrap();
    b.rotation = UnitQuaternion::from_scaled_axis(Vector3::x());
    assert!(!a.intersect(&b).unwrap());
}
#[test]
fn stationary_initialization_estimates_tilt_bias_and_rejects_motion() {
    let o = Options::default();
    let mut e = Estimator::new(o).unwrap();
    for i in 0..300 {
        let mut s = sample(f64::from(i) * 0.01);
        s.accel.x = if i % 2 == 0 { 1. } else { -1. };
        e.observe(s, true).unwrap();
    }
    assert!(e.state().is_none());
    let rotation = UnitQuaternion::from_euler_angles(0.2, -0.1, 0.);
    let bias = Vector3::new(0.01, -0.02, 0.005);
    for i in 300..550 {
        let mut s = sample(f64::from(i) * 0.01);
        s.gyro = bias;
        s.accel = rotation.inverse() * Vector3::z() * 9.81;
        e.observe(s, true).unwrap();
    }
    let state = e.state().unwrap();
    assert!((state.bias - bias).norm() < 1e-12);
    assert!(
        (state.rotation.inverse() * Vector3::z() - rotation.inverse() * Vector3::z()).norm()
            < 1e-10
    );
    assert!(!e.aligned());
}
#[test]
fn constant_gyro_follows_analytic_rotation_at_multiple_sample_rates() {
    for hz in [100., 1000.] {
        let mut s = state();
        s.bias = Vector3::new(0.01, -0.02, 0.005);
        let o = Options::default();
        let omega = Vector3::new(0.2, -0.3, 0.4);
        for _ in 0..hz as usize {
            s.predict(omega + s.bias, 1. / hz, &o).unwrap();
        }
        assert!((s.rotation.inverse() * UnitQuaternion::from_scaled_axis(omega)).angle() < 1e-12);
        s.validate().unwrap();
    }
}
#[test]
fn timestamps_gap_and_session_changes_cannot_silently_reset_estimator() {
    let mut e = initialized();
    let before = e.state().unwrap().clone();
    assert!(!e.observe(sample(2.1), true).unwrap());
    assert!(!e.observe(sample(1.), true).unwrap());
    assert_eq!(e.state().unwrap().covariance, before.covariance);
    let aid = Aid {
        timestamp: 2.1,
        session: 1,
        state: state(),
    };
    assert!(e.aid(&aid).unwrap());
    assert!(e.aligned());
    assert!(
        e.aid(&Aid {
            session: 2,
            ..aid.clone()
        })
        .is_err()
    );
    assert!(!e.aligned());
    assert!(e.aid(&aid).is_err());
    assert!(e.observe(sample(3.), false).is_err());
    assert!(e.state().is_none());
}
#[test]
fn delayed_aid_replay_equals_on_time_update_and_rejects_uncovered_time() {
    let mut a = initialized();
    let mut b = initialized();
    let first = Aid {
        timestamp: 2.1,
        session: 7,
        state: state(),
    };
    assert!(a.aid(&first).unwrap());
    assert!(b.aid(&first).unwrap());
    let mut aid = first.clone();
    aid.timestamp = 2.205;
    aid.state.rotation = UnitQuaternion::from_scaled_axis(Vector3::z() * 0.015);
    // 2.205 位于两个 IMU 样本间；a 最早在 2.21 收到，b 在 2.50 收到。
    for i in 211..=250 {
        let mut s = sample(f64::from(i) * 0.01);
        s.gyro = Vector3::z() * 0.1;
        a.observe(s, false).unwrap();
        b.observe(s, false).unwrap();
        if i == 221 {
            assert!(a.aid(&aid).unwrap());
        }
    }
    assert!(b.aid(&aid).unwrap());
    assert!((a.state().unwrap().rotation.inverse() * b.state().unwrap().rotation).angle() < 1e-12);
    assert!((a.state().unwrap().covariance - b.state().unwrap().covariance).amax() < 1e-12);
    assert!(!b.aid(&aid).unwrap());
    aid.timestamp = 10.;
    assert!(!b.aid(&aid).unwrap());
}
#[test]
fn consecutive_aids_inside_one_imu_interval_preserve_the_first_posterior() {
    let mut e = initialized();
    for i in 211..=221 {
        let mut s = sample(f64::from(i) * 0.01);
        s.gyro = Vector3::z() * 0.1;
        e.observe(s, false).unwrap();
    }
    let mut first = state();
    first.rotation = UnitQuaternion::from_scaled_axis(Vector3::z() * 0.2);
    first.covariance *= 0.1;
    assert!(
        e.aid(&Aid {
            timestamp: 2.205,
            session: 7,
            state: first
        })
        .unwrap()
    );
    let mut second = state();
    second.rotation = UnitQuaternion::from_scaled_axis(Vector3::z() * 0.3);
    assert!(
        e.aid(&Aid {
            timestamp: 2.207,
            session: 7,
            state: second
        })
        .unwrap()
    );
    // 首份分布各方向均更精确，CI 取其端点；角度由恒定角速度解析积分。
    let expected = UnitQuaternion::from_scaled_axis(Vector3::z() * (0.2 + 0.1 * 0.005));
    assert!((e.state().unwrap().rotation.inverse() * expected).angle() < 1e-12);
    assert!(e.state().unwrap().covariance.trace() < 0.001);
}

#[test]
fn invalid_configuration_measurements_and_covariances_are_rejected() {
    assert!(
        Estimator::new(Options {
            gyro_density: f64::NAN,
            ..Options::default()
        })
        .is_err()
    );
    let mut e = initialized();
    let mut bad = sample(2.11);
    bad.gyro.x = f64::NAN;
    assert!(e.observe(bad, false).is_err());
    let mut s = state();
    s.covariance[(0, 0)] = -1.;
    assert!(s.validate().is_err());
    s = state();
    s.covariance[(0, 1)] = 0.1;
    assert!(s.validate().is_err());
    let h = gravity_jacobian(Vector3::z() * 9.81);
    assert_eq!(h.fixed_view::<3, 3>(0, 3), Matrix3::zeros());
}
