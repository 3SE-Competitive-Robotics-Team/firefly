use firefly_rootslam::evaluation::evaluate;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::collections::BTreeMap;

#[test]
fn rigid_world_change_has_zero_ate_and_rte() {
    let times: Vec<_> = (0_i64..18).map(|i| i * 50_000_000).collect();
    let truth: Vec<_> = (0_i32..18)
        .map(|i| {
            let x = f64::from(i) * 0.1;
            Isometry3::from_parts(
                Translation3::new(x, x.sin(), x * x),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.0, x * 0.1, 0.0)),
            )
        })
        .collect();
    let world = Isometry3::from_parts(
        Translation3::new(10.0, -3.0, 2.0),
        UnitQuaternion::from_scaled_axis(Vector3::new(0.2, -0.4, 0.6)),
    );
    let estimates = times
        .iter()
        .zip(&truth)
        .map(|(&time, pose)| (time, world * pose))
        .collect();
    let result = evaluate(&times, &estimates, &times, &truth, 100_000_000).unwrap();
    assert!(result.ate_rmse_m < 1e-12);
    assert!(result.rte_rmse_m.unwrap() < 1e-12);
    assert_eq!(result.relative_pairs, 12);
    assert_eq!(result.success_fraction, 1.0);
}

#[test]
fn relative_error_uses_camera_frame_spacing_even_when_estimates_are_missing() {
    let times: Vec<_> = (0_i64..14).map(|i| i * 50_000_000).collect();
    let truth: Vec<_> = (0_i32..14)
        .map(|i| Isometry3::translation(f64::from(i), 0.0, 0.0))
        .collect();
    let mut estimates: BTreeMap<_, _> = times
        .iter()
        .zip(&truth)
        .map(|(&time, pose)| {
            (
                time,
                Isometry3::translation(pose.translation.x * 1.1, 0.0, 0.0),
            )
        })
        .collect();
    estimates.remove(&times[7]);
    estimates.insert(-1, Isometry3::identity());
    let result = evaluate(&times, &estimates, &times, &truth, 100_000_000).unwrap();
    assert_eq!(result.relative_pairs, 6);
    assert_eq!(result.estimated_frames, 13);
    assert!((result.rte_rmse_m.unwrap() - 0.6).abs() < 1e-12);
    assert!((result.success_fraction - 13.0 / 14.0).abs() < 1e-15);
    assert!(
        result.ate_rmse_m > 0.1,
        "SE3 alignment must not absorb scale drift"
    );
}

#[test]
fn ground_truth_interpolation_respects_gaps_and_rejects_invalid_timestamps() {
    let times: Vec<_> = (0_i64..9).collect();
    let gt_times = vec![0, 2, 4, 6, 8];
    let truth: Vec<_> = [0_i32, 2, 4, 6, 8]
        .into_iter()
        .map(|i| Isometry3::translation(f64::from(i), 0.0, 0.0))
        .collect();
    let estimates = (0_i32..9)
        .map(|i| (i64::from(i), Isometry3::translation(f64::from(i), 0.0, 0.0)))
        .collect();
    let all = evaluate(&times, &estimates, &gt_times, &truth, 2).unwrap();
    assert_eq!(all.associated_frames, 9);
    assert!(all.ate_rmse_m < 1e-14);
    let exact = evaluate(&times, &estimates, &gt_times, &truth, 1).unwrap();
    assert_eq!(exact.associated_frames, 5);
    assert_eq!(exact.relative_pairs, 2);
    assert!(evaluate(&[0, 0, 1], &estimates, &gt_times, &truth, 2).is_err());
}
