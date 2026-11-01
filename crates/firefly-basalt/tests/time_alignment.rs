use firefly_basalt::time_alignment::{RotationSamples, compute_error, search};
use nalgebra::Vector3;
use serde_json::Value;

fn samples() -> (RotationSamples, RotationSamples) {
    let gyro = RotationSamples {
        timestamps: (0..100).map(|i| i * 1_000_000).collect(),
        values: (0..100)
            .map(|i| {
                let i = f64::from(i);
                Vector3::new((0.17 * i).sin(), (0.07 * i).cos(), 0.01 * i)
            })
            .collect(),
    };
    let mocap = RotationSamples {
        timestamps: (5..95).map(|i| i * 1_000_000 - 1_234_567).collect(),
        values: gyro.values[5..95].to_vec(),
    };
    (gyro, mocap)
}
#[test]
fn original_grid_and_jacobi_svd_refinement() {
    let reference: Value =
        serde_json::from_str(include_str!("reference/time_alignment.json")).unwrap();
    let (gyro, mocap) = samples();
    let result = search(gyro, mocap, 0.004);
    assert_eq!(
        i64::from(result.best_error_index),
        reference["best_index"].as_i64().unwrap()
    );
    assert_eq!(
        i64::from(result.best_offset_ns),
        reference["best_offset"].as_i64().unwrap()
    );
    assert_eq!(
        i64::from(result.best_offset_refined_ns),
        reference["refined"].as_i64().unwrap()
    );
    for (actual, expected) in result
        .errors
        .iter()
        .zip(reference["errors"].as_array().unwrap())
    {
        let expected = expected.as_f64().unwrap();
        assert!((actual - expected).abs() <= 1e-10 * (1. + expected.abs()));
    }
    for i in 0..3 {
        let expected = reference["coefficients"][i].as_f64().unwrap();
        assert!(
            (result.coefficients[i] - expected).abs() <= 1e-10 * (1. + expected.abs()),
            "coefficient {i}: {} != {expected}",
            result.coefficients[i]
        );
    }
    let mut offsets = Vec::new();
    result.write_offsets(&mut offsets, 123).unwrap();
    let offsets: Value = serde_json::from_slice(&offsets).unwrap();
    assert_eq!(
        offsets["mocap_to_imu_total_offset_ns"].as_i64().unwrap(),
        123 + i64::from(result.best_offset_refined_ns)
    );
    let mut series = Vec::new();
    result.write_error_series(&mut series).unwrap();
    assert_eq!(String::from_utf8(series).unwrap().lines().count(), 83);
    assert!(result.write_mocap(Vec::new(), 0).is_err());
}
#[test]
fn association_ties_endpoints_and_missing_overlap() {
    let gyro = RotationSamples {
        timestamps: vec![0, 10_000_000, 20_000_000],
        values: vec![Vector3::zeros(), Vector3::repeat(1.), Vector3::repeat(2.)],
    };
    let mut mocap = RotationSamples {
        timestamps: vec![5_000_000],
        values: vec![Vector3::zeros()],
    };
    assert_eq!(compute_error(0, &gyro, &mocap), 0.);
    mocap.timestamps[0] = 0;
    assert!(compute_error(0, &gyro, &mocap).is_infinite());
    mocap.timestamps[0] = 20_000_000;
    mocap.values[0] = Vector3::repeat(2.);
    assert_eq!(compute_error(0, &gyro, &mocap), 0.);
    assert!(compute_error(100_000_000, &gyro, &mocap).is_infinite());
    let empty = search(RotationSamples::default(), RotationSamples::default(), 0.);
    assert_eq!(empty.best_error_index, -1);
    assert_eq!(empty.best_offset_refined_ns, 0);
}

#[derive(Default)]
struct Dataset {
    images: Vec<i64>,
    gyro: Vec<firefly_basalt::io::dataset_io::GyroData>,
    times: Vec<i64>,
    poses: Vec<nalgebra::Isometry3<f64>>,
}
impl firefly_basalt::io::dataset_io::VioDataset for Dataset {
    fn num_cams(&self) -> usize {
        0
    }
    fn image_timestamps(&mut self) -> &mut Vec<i64> {
        &mut self.images
    }
    fn accel_data(&self) -> &[firefly_basalt::io::dataset_io::AccelData] {
        &[]
    }
    fn gyro_data(&self) -> &[firefly_basalt::io::dataset_io::GyroData] {
        &self.gyro
    }
    fn gt_timestamps(&self) -> &[i64] {
        &self.times
    }
    fn gt_pose_data(&self) -> &[nalgebra::Isometry3<f64>] {
        &self.poses
    }
    fn mocap_to_imu_offset_ns(&self) -> i64 {
        123
    }
    fn image_data(
        &mut self,
        _: i64,
    ) -> firefly_error::Result<Vec<firefly_basalt::io::dataset_io::ImageData>> {
        Ok(Vec::new())
    }
}
#[test]
fn calibrated_streams_filter_saturation_gaps_and_write_corrected_poses() {
    use firefly_basalt::{
        io::dataset_io::GyroData,
        time_alignment::{align, write_aligned_poses},
    };
    use firefly_basalt_headers::{
        calibration::{Calibration, MocapCalibration},
        utils::sophus_utils::so3_exp,
    };
    use nalgebra::{Isometry3, Translation3};
    let mut dataset = Dataset {
        times: vec![0, 5_000_000, 10_000_000, 15_000_000, 40_000_000],
        ..Dataset::default()
    };
    dataset.poses = dataset
        .times
        .iter()
        .map(|&time| {
            Isometry3::from_parts(
                Translation3::new(1., 2., 3.),
                so3_exp(&Vector3::new(0., 0., time as f64 * 1e-9 * 0.5)),
            )
        })
        .collect();
    dataset.gyro = vec![
        GyroData {
            timestamp_ns: 0,
            data: Vector3::new(0., 0., 0.75),
        },
        GyroData {
            timestamp_ns: 5_000_000,
            data: Vector3::repeat(100.),
        },
        GyroData {
            timestamp_ns: 10_000_000,
            data: Vector3::new(0., 0., 0.75),
        },
    ];
    let mut calibration = Calibration::default();
    calibration.calib_gyro_bias.param_mut()[2] = 0.25;
    let mocap = MocapCalibration::default();
    let result = align(&dataset, Some((&calibration, &mocap)), 0.);
    assert_eq!(result.saturated_gyro, 1);
    assert_eq!(result.mocap.timestamps, vec![5_000_000, 10_000_000]);
    for value in result.gyro.values.iter().chain(&result.mocap.values) {
        assert!((value - Vector3::new(0., 0., 0.5)).norm() < 1e-10);
    }
    let mut output = Vec::new();
    write_aligned_poses(&mut output, &dataset, 25, Some(&mocap)).unwrap();
    let output = String::from_utf8(output).unwrap();
    assert_eq!(output.lines().count(), 6);
    assert_eq!(output.lines().nth(1).unwrap(), "25,1,2,3,1,0,0,0");
}
