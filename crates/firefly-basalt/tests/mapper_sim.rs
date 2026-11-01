use firefly_basalt::imu::imu_types::PoseVelBiasState;
use firefly_basalt::utils::sophus_utils::se3_expd;
use firefly_basalt::utils::sophus_utils::se3_matrix;
use firefly_basalt::{
    mapper_sim::{extract_nonlinear_factors, process_marg_data},
    utils::imu_types::{MargData, PoseStateWithLin, PoseVelBiasStateWithLin},
};
use nalgebra::{DMatrix, DVector, SVector, Vector3};
use serde_json::Value;

fn matrix(value: &Value) -> DMatrix<f64> {
    let rows = value.as_array().unwrap();
    let columns = rows[0].as_array().unwrap().len();
    DMatrix::from_iterator(
        rows.len(),
        columns,
        (0..columns).flat_map(|c| rows.iter().map(move |row| row[c].as_f64().unwrap())),
    )
}
fn check(actual: &DMatrix<f64>, expected: &Value) {
    let expected = matrix(expected);
    assert_eq!(actual.shape(), expected.shape());
    for (a, b) in actual.iter().zip(expected.iter()) {
        assert!((a - b).abs() <= 1e-10 * (1. + b.abs()), "{a} != {b}");
    }
}
#[test]
fn standalone_mapper_callbacks_match_original_translation_unit() {
    let fixtures: Value = serde_json::from_str(include_str!("reference/mapper_sim.json")).unwrap();
    for (test, fixture) in fixtures.as_array().unwrap().iter().enumerate() {
        let mut data = MargData {
            abs_h: matrix(&fixture["h"]),
            abs_b: DVector::from_iterator(
                matrix(&fixture["b"]).len(),
                matrix(&fixture["b"]).iter().copied(),
            ),
            ..MargData::default()
        };
        data.aom.total_size = data.abs_h.ncols();
        for i in 0..if test == 2 { 3 } else { 2 } {
            let time = 10 + i * 10;
            let x = i as f64;
            let pose = se3_expd(&SVector::<f64, 6>::new(
                0.2 * x,
                -0.1 * x,
                0.3,
                0.03 * x,
                -0.02,
                0.04,
            ));
            let size = if i == 0 || test == 0 { 6 } else { 15 };
            let offset = if i == 0 {
                0
            } else if i == 1 {
                6
            } else {
                21
            };
            data.aom.abs_order_map.insert(time, (offset, size));
            if size == 6 {
                data.frame_poses
                    .insert(time, PoseStateWithLin::new(time, pose, false));
            } else {
                data.frame_states.insert(
                    time,
                    PoseVelBiasStateWithLin::new(
                        PoseVelBiasState::new(
                            time,
                            pose,
                            Vector3::zeros(),
                            Vector3::zeros(),
                            Vector3::zeros(),
                        ),
                        false,
                    ),
                );
            }
        }
        data.kfs_all.extend([10, 20]);
        data.kfs_to_marg.insert(10);
        if test > 0 {
            process_marg_data(&mut data);
        }
        check(&data.abs_h, &fixture["reduced_h"]);
        check(
            &DMatrix::from_column_slice(data.abs_b.len(), 1, data.abs_b.as_slice()),
            &fixture["reduced_b"],
        );
        assert!(data.frame_states.is_empty());
        assert_eq!(data.aom.total_size, 12);
        let (roll, relative) = extract_nonlinear_factors(&data);
        assert_eq!(roll.len(), 1);
        assert_eq!(relative.len(), 1);
        check(
            &DMatrix::from_column_slice(2, 2, roll[0].cov_inv.as_slice()),
            &fixture["roll_information"],
        );
        check(
            &DMatrix::from_column_slice(6, 6, relative[0].cov_inv.as_slice()),
            &fixture["relative_information"],
        );
        check(
            &DMatrix::from_column_slice(4, 4, se3_matrix(&relative[0].t_i_j).as_slice()),
            &fixture["relative_pose"],
        );
    }
}
