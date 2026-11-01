use firefly_basalt_headers::utils::sophus_utils::so3_exp;
use firefly_rootslam::{
    dataset::write_ground_truth,
    dataset_output::{OriginalResult, TrajectoryFormat, align_ground_truth, write_trajectory},
};
use nalgebra::{Isometry3, Translation3, Vector3};

#[test]
fn native_ground_truth_bytes_and_original_trajectory_oracle() {
    let reference: serde_json::Value = serde_json::from_str(include_str!(
        "../../../crates/firefly-basalt/tests/reference/mapper.json"
    ))
    .unwrap();
    let estimates: Vec<_> = (0..3)
        .map(|i| {
            let v = f64::from(i);
            (
                1_234_567_890_123_456 + i64::from(i) * 123_456_789,
                Isometry3::from_parts(
                    Translation3::new(0.25 * v, -0.125 * v, 1.5),
                    so3_exp(&Vector3::new(0.1 * v, -0.03 * v, 0.04 * v)),
                ),
            )
        })
        .collect();
    let mut bytes = Vec::new();
    write_ground_truth(
        &mut bytes,
        &estimates.iter().map(|v| v.0).collect::<Vec<_>>(),
        &estimates.iter().map(|v| v.1).collect::<Vec<_>>(),
    )
    .unwrap();
    assert_eq!(
        bytes,
        serde_json::from_value::<Vec<u8>>(reference["gt_binary"].clone()).unwrap()
    );
    for (format, name) in [
        (TrajectoryFormat::Tum, "tum"),
        (TrajectoryFormat::Euroc, "euroc"),
    ] {
        let mut output = Vec::new();
        write_trajectory(&mut output, &estimates, format).unwrap();
        let actual = String::from_utf8(output).unwrap();
        let expected = reference[name].as_str().unwrap();
        assert_eq!(actual.lines().next(), expected.lines().next());
        assert_eq!(actual.lines().count(), expected.lines().count());
        for (a, e) in actual.lines().skip(1).zip(expected.lines().skip(1)) {
            for (a, e) in a.split([' ', ',']).zip(e.split([' ', ','])) {
                assert_eq!(a.parse::<f64>().unwrap(), e.parse::<f64>().unwrap());
            }
        }
    }
    let mut kitti = Vec::new();
    write_trajectory(
        &mut kitti,
        &[(1, Isometry3::translation(1., 2., 3.))],
        TrajectoryFormat::Kitti,
    )
    .unwrap();
    let values: Vec<f64> = std::str::from_utf8(&kitti)
        .unwrap()
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    assert_eq!(values, [1., 0., 0., 1., 0., 1., 0., 2., 0., 0., 1., 3.]);
}

#[test]
fn original_alignment_and_result_schema() {
    let times = [0, 50_000_000, 100_000_000, 150_000_000];
    let positions = [Vector3::zeros(), Vector3::x(), Vector3::y(), Vector3::z()];
    let transform = Isometry3::new(Vector3::new(3., -4., 5.), Vector3::new(0.2, -0.3, 0.1));
    let mut ground_truth: Vec<_> = positions
        .iter()
        .map(|p| transform.transform_point(&(*p).into()).coords)
        .collect();
    let estimates: Vec<_> = times
        .iter()
        .zip(positions)
        .map(|(t, p)| (*t, Isometry3::translation(p.x, p.y, p.z)))
        .collect();
    let error = align_ground_truth(&estimates, &times, &mut ground_truth).unwrap();
    assert!(error < 1e-14);
    for (a, e) in ground_truth.iter().zip(positions) {
        assert!((a - e).norm() < 1e-14);
    }
    assert!(align_ground_truth(&[], &times, &mut ground_truth).is_err());
    let value = serde_json::to_value(OriginalResult {
        rms_ate: error,
        num_frames: 100,
        exec_time_ns: 123,
    })
    .unwrap();
    assert_eq!(value.as_object().unwrap().len(), 3);
    assert_eq!(value["num_frames"], 100);
    assert_eq!(value["exec_time_ns"], 123);
    let stats = firefly_rootslam::dataset_output::original_statistics(
        std::time::Duration::from_millis(1250),
        0.01,
        150,
        2821,
        Some(4096),
    );
    assert_eq!(
        stats.to_json(),
        serde_json::json!({
            "exec_time_s": [1.25], "ate_rmse": [0.01], "ate_num_kfs": [150.0],
            "num_frames": [2821.0], "resident_memory_peak": [4096.0]
        })
    );
    assert!(
        firefly_rootslam::dataset_output::original_statistics(
            std::time::Duration::ZERO,
            0.,
            0,
            0,
            None
        )
        .get("resident_memory_peak")
        .is_none()
    );
}
