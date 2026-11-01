use firefly_basalt::kitti_eval::{eval_kitti, read_poses, write_results};
use firefly_basalt::utils::sophus_utils::so3_exp;
use nalgebra::{Isometry3, Translation3, Vector3};
use serde_json::Value;

#[test]
fn original_segment_lengths_and_error_metrics() {
    let reference: Value = serde_json::from_str(include_str!("reference/kitti_eval.json")).unwrap();
    let mut gt = Vec::new();
    let mut estimated = Vec::new();
    for i in 0..101 {
        let i = f64::from(i);
        let x = i * 1.25;
        gt.push(Isometry3::from_parts(
            Translation3::new(x, 0.125 * (0.1 * i).sin(), 0.),
            so3_exp(&Vector3::new(0., 0., 0.001 * i)),
        ));
        estimated.push(Isometry3::from_parts(
            Translation3::new(x * 1.02, 0.125 * (0.1 * i).sin() + 0.001 * i, 0.0001 * i),
            so3_exp(&Vector3::new(0.0001 * i, 0., 0.0012 * i)),
        ));
    }
    let result = eval_kitti(&[5., 10., 25., 50., 200.], &gt, &estimated);
    for (key, value) in &result {
        assert_eq!(
            value.num_meas as u64,
            reference[key]["num_meas"].as_f64().unwrap() as u64
        );
        for (actual, field) in [
            (value.trans_error, "trans_error"),
            (value.rot_error, "rot_error"),
        ] {
            if reference[key][field].is_null() {
                assert!(actual.is_nan());
            } else {
                let expected = reference[key][field].as_f64().unwrap();
                assert!(
                    (actual - expected).abs() <= 1e-10 * (1. + expected.abs()),
                    "{key} {field}: {actual} != {expected}"
                );
            }
        }
    }
    let mut output = Vec::new();
    write_results(&mut output, &result).unwrap();
    let archive = String::from_utf8(output).unwrap();
    assert!(archive.contains("\"trans_error\":NaN"));
}
#[test]
fn pose_rows_and_invalid_input() {
    let poses = read_poses(&b"# comment\n1 0 0 1 0 1 0 2 0 0 1 3\n1 0 0 4 0 -1 0 5 0 0 -1 6\n"[..])
        .unwrap();
    assert_eq!(poses.len(), 2);
    assert_eq!(poses[0].translation.vector, Vector3::new(1., 2., 3.));
    assert!((poses[1].rotation.angle() - std::f64::consts::PI).abs() < 1e-10);
    assert!(read_poses(&b"1 0\n"[..]).is_err());
}
