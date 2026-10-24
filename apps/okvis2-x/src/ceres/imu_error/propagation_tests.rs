// SPDX-License-Identifier: BSD-3-Clause
use super::*;
use crate::sensors::measurements::{ImuMeasurement, ImuSensorReadings};

fn time(microseconds: i64) -> Timestamp {
    Timestamp::from_nanosecond(1_700_000_000_000_000_000 + i128::from(microseconds) * 1000).unwrap()
}

fn parameters() -> ImuParameters {
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

fn measurements() -> ImuMeasurementDeque {
    (0..=100)
        .map(|i| {
            let t = f64::from(i) * 0.001;
            ImuMeasurement::new(
                time(i64::from(i) * 1000 + i64::from(i % 3) * 50),
                ImuSensorReadings::new(
                    Vector3::new(0.1 + t, -0.2 * t, 0.3 - t),
                    Vector3::new(t, 0.5 - t, 9.81 + 0.2 * t),
                ),
                -1,
            )
        })
        .collect()
}

#[test]
fn nominal_propagation_matches_all_derivative_output_modes_exactly() {
    let mut samples = measurements();
    samples.insert(20, samples[20].clone());
    let initial_pose = Transformation::new(
        Vector3::new(0.1, -0.2, 0.3),
        delta_q(&Vector3::new(0.2, -0.1, 0.3)),
    );
    let initial_sb =
        SpeedAndBias::from_row_slice(&[0.1, -0.2, 0.3, 0.01, -0.02, 0.03, 0.1, -0.2, 0.3]);
    for limit_scale in [1.0, 0.001] {
        let mut params = parameters();
        params.a_max *= limit_scale;
        params.g_max *= limit_scale;
        for interval in [
            (time(0), time(100_050)),
            (time(125), time(99_725)),
            (time(1130), time(1850)),
            (time(0), time(500)),
            (time(3000), time(3000)),
        ] {
            let mut expected_pose = initial_pose.clone();
            let mut expected_sb = initial_sb;
            let mut expected_cov = Covariance::zeros();
            let mut expected_jac = Jacobian::zeros();
            let expected_steps = ImuError::propagation(
                &samples,
                &params,
                &mut expected_pose,
                &mut expected_sb,
                interval,
                (Some(&mut expected_cov), Some(&mut expected_jac)),
            )
            .unwrap();
            for (with_cov, with_jac) in [(false, false), (true, false), (false, true)] {
                let mut pose = initial_pose.clone();
                let mut sb = initial_sb;
                let mut cov = Covariance::zeros();
                let mut jac = Jacobian::zeros();
                let steps = ImuError::propagation(
                    &samples,
                    &params,
                    &mut pose,
                    &mut sb,
                    interval,
                    (with_cov.then_some(&mut cov), with_jac.then_some(&mut jac)),
                )
                .unwrap();
                assert_eq!(steps, expected_steps);
                assert_eq!(pose.coeffs(), expected_pose.coeffs());
                assert_eq!(sb, expected_sb);
                if with_cov {
                    assert_eq!(cov, expected_cov);
                }
                if with_jac {
                    assert_eq!(jac, expected_jac);
                }
            }
        }
    }
}

#[test]
fn nominal_propagation_follows_independent_constant_acceleration() {
    let params = parameters();
    let gyro_bias = Vector3::new(0.01, -0.02, 0.03);
    let acc_bias = Vector3::new(0.1, 0.2, -0.1);
    let acceleration = Vector3::new(0.4, -0.2, 0.1);
    let v0 = Vector3::new(0.2, -0.1, 0.3);
    let samples = (0..=100)
        .map(|i| {
            ImuMeasurement::new(
                time(i * 10_000),
                ImuSensorReadings::new(
                    gyro_bias,
                    acceleration + Vector3::new(0.0, 0.0, params.g) + acc_bias,
                ),
                -1,
            )
        })
        .collect();
    for interval in [(time(0), time(1_000_000)), (time(123_000), time(987_000))] {
        let mut pose = Transformation::identity();
        let mut sb = SpeedAndBias::zeros();
        sb.fixed_rows_mut::<3>(0).copy_from(&v0);
        sb.fixed_rows_mut::<3>(3).copy_from(&gyro_bias);
        sb.fixed_rows_mut::<3>(6).copy_from(&acc_bias);
        let steps = ImuError::propagation(
            &samples,
            &params,
            &mut pose,
            &mut sb,
            interval,
            (None, None),
        )
        .unwrap();
        let dt = seconds(interval.1, interval.0);
        assert!(steps > 0);
        assert!((pose.translation() - (v0 * dt + 0.5 * acceleration * dt * dt)).norm() < 1e-13);
        assert!((sb.fixed_rows::<3>(0) - (v0 + acceleration * dt)).norm() < 1e-13);
        assert!((pose.rotation_matrix() - Matrix3::identity()).norm() < 1e-14);
        assert_eq!(sb.fixed_rows::<3>(3), gyro_bias);
        assert_eq!(sb.fixed_rows::<3>(6), acc_bias);
    }
}

#[test]
fn propagation_coverage_errors_leave_all_requested_outputs_unchanged() {
    let params = parameters();
    let initial_pose = Transformation::new(
        Vector3::new(0.1, -0.2, 0.3),
        delta_q(&Vector3::new(0.2, -0.1, 0.3)),
    );
    let initial_sb = SpeedAndBias::repeat(0.01);
    for (samples, interval, error) in [
        (
            ImuMeasurementDeque::new(),
            (time(0), time(1000)),
            IntegrationError::MissingMeasurements,
        ),
        (
            measurements(),
            (time(-1), time(1000)),
            IntegrationError::StartNotCovered,
        ),
        (
            measurements(),
            (time(0), time(100_051)),
            IntegrationError::EndNotCovered,
        ),
    ] {
        for (with_cov, with_jac) in [(false, false), (true, false), (false, true), (true, true)] {
            let mut pose = initial_pose.clone();
            let mut sb = initial_sb;
            let mut cov = Covariance::repeat(7.0);
            let mut jac = Jacobian::repeat(8.0);
            assert_eq!(
                ImuError::propagation(
                    &samples,
                    &params,
                    &mut pose,
                    &mut sb,
                    interval,
                    (with_cov.then_some(&mut cov), with_jac.then_some(&mut jac)),
                ),
                Err(error)
            );
            assert_eq!(pose.coeffs(), initial_pose.coeffs());
            assert_eq!(sb, initial_sb);
            assert_eq!(cov, Covariance::repeat(7.0));
            assert_eq!(jac, Jacobian::repeat(8.0));
        }
    }
}
