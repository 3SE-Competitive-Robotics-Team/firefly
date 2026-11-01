use firefly_basalt::utils::sophus_utils::so3_exp;
use firefly_basalt::{
    mapper::{Mapper, read_ground_truth},
    utils::{imu_types::PoseStateWithLin, vio_config::VioConfig},
    vi_estimator::nfr_mapper::NfrMapper,
};
use firefly_calibration::calibration::Calibration;
use nalgebra::{Isometry3, Translation3, Vector3};
use std::collections::BTreeMap;

#[test]
fn original_ground_truth_archive_and_both_trajectory_formats() {
    let expected: serde_json::Value =
        serde_json::from_str(include_str!("reference/mapper.json")).unwrap();
    let bytes: Vec<u8> = serde_json::from_value(expected["gt_binary"].clone()).unwrap();
    let (gt_timestamps, gt_positions) = read_ground_truth(&mut bytes.as_slice()).unwrap();
    assert_eq!(gt_timestamps.len(), 3);
    let mut mapper = Mapper {
        mapper: NfrMapper::new(&Calibration::default(), &VioConfig::default()),
        marg_data: BTreeMap::new(),
        gt_timestamps,
        gt_positions,
        image_timestamps: Vec::new(),
        points: Vec::new(),
        point_ids: Vec::new(),
        edges: Vec::new(),
        roll_pitch_edges: Vec::new(),
        relative_edges: Vec::new(),
        num_opt_iter: 10,
        outlier_threshold: 3.,
    };
    for i in 0..3 {
        let value = f64::from(i);
        let time = 1_234_567_890_123_456 + i64::from(i) * 123_456_789;
        let pose = Isometry3::from_parts(
            Translation3::new(0.25 * value, -0.125 * value, 1.5),
            so3_exp(&Vector3::new(0.1 * value, -0.03 * value, 0.04 * value)),
        );
        mapper
            .mapper
            .base
            .frame_poses
            .insert(time, PoseStateWithLin::new(time, pose, false));
        assert_eq!(mapper.gt_timestamps[i as usize], time);
        assert_eq!(mapper.gt_positions[i as usize], pose.translation.vector);
    }
    for (tum, field) in [(true, "tum"), (false, "euroc")] {
        let mut output = Vec::new();
        mapper.save_trajectory(&mut output, tum).unwrap();
        let actual = String::from_utf8(output).unwrap();
        let expected = expected[field].as_str().unwrap();
        assert_eq!(actual.lines().next(), expected.lines().next());
        for (actual, expected) in actual.lines().skip(1).zip(expected.lines().skip(1)) {
            let values = |line: &str| {
                line.split([' ', ','])
                    .map(|v| v.parse::<f64>().unwrap())
                    .collect::<Vec<_>>()
            };
            let a = values(actual);
            let e = values(expected);
            assert_eq!(a.len(), 8);
            for (a, e) in a.iter().zip(e) {
                assert!((a - e).abs() <= 1e-10 * e.abs().max(1.));
            }
        }
        assert_eq!(actual.lines().count(), 4);
    }
    for length in [0, 7, bytes.len() - 1] {
        assert!(read_ground_truth(&mut &bytes[..length]).is_err());
    }
}
