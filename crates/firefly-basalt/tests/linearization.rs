use firefly_basalt::imu::imu_types::ImuData;
use firefly_basalt::imu::imu_types::PoseVelBiasState;
use firefly_basalt::imu::preintegration::IntegratedImuMeasurement;
use firefly_basalt::{
    linearization::{
        landmark_block,
        linearization_base::{LinearizationBase, Options},
    },
    utils::{
        common_types::TimeCamId,
        imu_types::{
            AbsOrderMap, ImuLinData, MargLinData, PoseStateWithLin, PoseVelBiasStateWithLin,
        },
        vio_config::LinearizationType,
    },
    vi_estimator::{
        ba_base::BundleAdjustmentBase,
        landmark_database::{Keypoint, KeypointObservation},
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, Isometry3, SVector, Vector2, Vector3, Vector4};
use serde_json::Value;
use std::collections::BTreeMap;

fn check<S: BasaltScalar>(actual: S, expected: f64, label: &str) {
    let tolerance = if std::mem::size_of::<S>() == 4 {
        1e-5
    } else {
        1e-10
    };
    let actual = actual.as_f64();
    assert!(
        (actual - expected).abs() <= tolerance * (1. + expected.abs()),
        "{label}: {actual} != {expected}"
    );
}

/// Dot-product forward error, including the measured perturbation of J/r inputs.
/// Each product and sum contributes one operation; no fitted tolerance is used.
fn check_dot<S: BasaltScalar>(
    actual: S,
    expected: f64,
    left: &[(f64, f64)],
    right: &[(f64, f64)],
    label: &str,
) {
    let unit_roundoff = S::default_epsilon().as_f64() / 2.;
    let operations = (2 * left.len()) as f64;
    let gamma = operations * unit_roundoff / (1. - operations * unit_roundoff);
    let mut input_error = 0.;
    let mut magnitude = 0.;
    for (&(a, x), &(b, y)) in left.iter().zip(right) {
        input_error += (a - x).abs() * b.abs() + x.abs() * (b - y).abs();
        magnitude += (a * b).abs() + (x * y).abs();
    }
    let bound_operations = (12 * left.len() + 8) as f64;
    let bound_gamma = bound_operations * f64::EPSILON / (1. - bound_operations * f64::EPSILON);
    let bound = (input_error + gamma * magnitude) * (1. + bound_gamma);
    assert!(
        (actual.as_f64() - expected).abs() <= bound,
        "{label}: {} != {expected}; forward bound {bound}",
        actual.as_f64()
    );
}

#[expect(
    clippy::too_many_lines,
    reason = "Same ordered window construction and intermediate checks as the independent C++ probe"
)]
fn run<S: BasaltScalar>(text: &str, mode: Option<LinearizationType>) {
    let reference: Value = serde_json::from_str(text).unwrap();
    let scalar = S::from_f64_lossy;
    let cases = reference.as_array().unwrap();
    for expected in cases
        .iter()
        .filter(|case| case["with_prior"] == false)
        .chain(cases.iter().filter(|case| case["with_prior"] == true))
    {
        let with_prior = expected["with_prior"].as_bool().unwrap();
        let with_imu = expected["with_imu"].as_bool().unwrap_or(false);
        let kind = [
            LinearizationType::AbsQr,
            LinearizationType::AbsSc,
            LinearizationType::RelSc,
        ][usize::try_from(expected["mode"].as_u64().unwrap()).unwrap()];
        if mode.is_some_and(|mode| mode != kind) {
            continue;
        }
        let mut camera = GenericCamera::from_name("pinhole");
        camera.set_from_init(Vector4::new(320., 315., 320., 240.).map(scalar));
        let calibration = Calibration {
            intrinsics: vec![camera.clone(), camera],
            t_i_c: vec![
                Isometry3::identity(),
                Isometry3::translation(scalar(0.125), S::zero(), S::zero()),
            ],
            ..Calibration::default()
        };
        let mut ba = BundleAdjustmentBase::new(calibration, scalar(1.5), scalar(2.));
        let mut order = AbsOrderMap::default();
        for i in 0..3_i32 {
            let timestamp = i64::from((i + 1) * 10);
            ba.frame_poses.insert(
                timestamp,
                PoseStateWithLin::new(
                    timestamp,
                    Isometry3::translation(
                        scalar(f64::from(i) * 0.25),
                        scalar(f64::from(i) * -0.0625),
                        S::zero(),
                    ),
                    with_prior,
                ),
            );
            if with_prior {
                let i = f64::from(i);
                ba.frame_poses.get_mut(&timestamp).unwrap().apply_inc(
                    &SVector::<f64, 6>::new(
                        0.001 * i,
                        -0.002 * i,
                        0.0005 * i,
                        0.000_125 * i,
                        -0.00025 * i,
                        0.000_375 * i,
                    )
                    .map(scalar),
                );
            }
            order.abs_order_map.insert(timestamp, (i * 6, 6));
        }
        order.total_size = 18;
        order.items = 3;
        let mut measurement =
            IntegratedImuMeasurement::new(1_000_000_000, Vector3::zeros(), Vector3::zeros());
        let gravity = Vector3::zeros();
        let gyro_weight = Vector3::repeat(scalar(2.));
        let accel_weight = Vector3::repeat(scalar(3.));
        if with_imu {
            for i in 1..=2 {
                measurement.integrate(
                    &ImuData {
                        t_ns: 1_000_000_000 + i * 50_000_000,
                        accel: Vector3::zeros(),
                        gyro: Vector3::zeros(),
                    },
                    &Vector3::repeat(scalar(0.01)),
                    &Vector3::repeat(scalar(0.001)),
                );
            }
            for i in 0..2_i32 {
                let timestamp = 1_000_000_000 + i64::from(i) * 100_000_000;
                let state = PoseVelBiasState::new(
                    timestamp,
                    Isometry3::translation(
                        scalar(0.001 * f64::from(i)),
                        scalar(-0.002 * f64::from(i)),
                        scalar(0.003 * f64::from(i)),
                    ),
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                );
                ba.frame_states
                    .insert(timestamp, PoseVelBiasStateWithLin::new(state, false));
                order.abs_order_map.insert(timestamp, (18 + i * 15, 15));
            }
            order.total_size = 48;
            order.items = 5;
        }
        let imu = ImuLinData {
            g: &gravity,
            gyro_bias_weight_sqrt: &gyro_weight,
            accel_bias_weight_sqrt: &accel_weight,
            imu_meas: BTreeMap::from([(1_000_000_000, &measurement)]),
        };
        for id in 0..3_i32 {
            let point = Keypoint::new(
                Vector2::new(
                    0.125 + f64::from(id) * 0.03125,
                    -0.125 + f64::from(id) * 0.015_625,
                )
                .map(scalar),
                scalar(0.25 + f64::from(id) * 0.0625),
                TimeCamId::new(10, 0),
            );
            ba.lmdb.add_landmark(usize::try_from(id).unwrap(), &point);
            for i in 0..3_i32 {
                for cam in 0..2_i32 {
                    ba.lmdb.add_observation(
                        TimeCamId::new(i64::from((i + 1) * 10), usize::try_from(cam).unwrap()),
                        &KeypointObservation {
                            kpt_id: id,
                            pos: Vector2::new(
                                400. + f64::from(id) * 8.
                                    - f64::from(i) * 12.
                                    - f64::from(cam) * 7.,
                                160. + f64::from(id) * 4. + f64::from(i) * 3.,
                            )
                            .map(scalar),
                        },
                    );
                }
            }
        }
        let options = Options {
            linearization_type: kind,
            lb_options: landmark_block::Options {
                obs_std_dev: scalar(1.5),
                huber_parameter: scalar(2.),
                ..landmark_block::Options::default()
            },
        };
        if with_prior && !expected["schur"].is_null() {
            let (blocks, _) = firefly_basalt::vi_estimator::sc_ba_base::linearize_helper(
                ba.lmdb.observations(),
                &ba,
            );
            let block = &blocks[0];
            let verify =
                |actual: &[S], rows: usize, cols: usize, reference: &Value, label: &str| {
                    for row in 0..rows {
                        for col in 0..cols {
                            check(
                                actual[col * rows + row],
                                reference["values"][row * cols + col].as_f64().unwrap(),
                                &format!("{label}[{row},{col}]"),
                            );
                        }
                    }
                };
            for id in 0..3 {
                let reference = &expected["schur"]["landmarks"][id];
                verify(
                    block.hll[&id].as_slice(),
                    3,
                    3,
                    &reference["hll"],
                    "Schur Hll",
                );
                verify(
                    block.hll_inv[&id].as_slice(),
                    3,
                    3,
                    &reference["inverse"],
                    "Schur inverse",
                );
                verify(block.bl[&id].as_slice(), 3, 1, &reference["bl"], "Schur bl");
            }
            for (i, frame) in block.hpppl.iter().enumerate() {
                let reference = &expected["schur"]["frames"][i];
                if !reference["transform"].is_null() {
                    let (host, target) = block.base.order[i];
                    let transform = if host == target {
                        nalgebra::Matrix4::identity()
                    } else {
                        firefly_basalt::utils::sophus_utils::se3_matrix(
                            &firefly_basalt::utils::ba_utils::compute_rel_pose(
                                ba.get_pose_state_with_lin(host.frame_id).pose(),
                                &ba.calib.t_i_c[host.cam_id],
                                ba.get_pose_state_with_lin(target.frame_id).pose(),
                                &ba.calib.t_i_c[target.cam_id],
                                None,
                                None,
                            ),
                        )
                    };
                    verify(
                        transform.as_slice(),
                        4,
                        4,
                        &reference["transform"],
                        "Schur current transform",
                    );
                    for (j, &id) in frame.lm_id.iter().enumerate() {
                        let point = ba.lmdb.landmark(id);
                        let mut residual = Vector2::zeros();
                        let mut jp = nalgebra::SMatrix::<S, 2, 6>::zeros();
                        let mut jl = nalgebra::SMatrix::<S, 2, 3>::zeros();
                        assert!(firefly_basalt::utils::ba_utils::linearize_point(
                            &point.obs[&target],
                            point,
                            &transform,
                            &ba.calib.intrinsics[target.cam_id],
                            &mut residual,
                            Some(&mut jp),
                            Some(&mut jl),
                            None,
                        ));
                        let observation = &reference["observations"][j];
                        verify(
                            residual.as_slice(),
                            2,
                            1,
                            &observation["residual"],
                            "Schur reprojection residual",
                        );
                        verify(
                            jp.as_slice(),
                            2,
                            6,
                            &observation["jp"],
                            "Schur pose Jacobian",
                        );
                        verify(
                            jl.as_slice(),
                            2,
                            3,
                            &observation["jl"],
                            "Schur landmark Jacobian",
                        );
                    }
                }
                verify(
                    block.base.d_rel_d_h[i].as_slice(),
                    6,
                    6,
                    &reference["dh"],
                    "Schur dh",
                );
                verify(
                    block.base.d_rel_d_t[i].as_slice(),
                    6,
                    6,
                    &reference["dt"],
                    "Schur dt",
                );
                verify(frame.hpp.as_slice(), 6, 6, &reference["hpp"], "Schur Hpp");
                verify(frame.bp.as_slice(), 6, 1, &reference["bp"], "Schur bp");
                for (j, cross) in frame.hpl.iter().enumerate() {
                    verify(cross.as_slice(), 6, 3, &reference["cross"][j], "Schur Hpl");
                }
            }
            let mut exact = block.clone();
            let copy = |target: &mut [S], rows: usize, cols: usize, source: &Value| {
                for row in 0..rows {
                    for col in 0..cols {
                        target[col * rows + row] =
                            scalar(source["values"][row * cols + col].as_f64().unwrap());
                    }
                }
            };
            for id in 0..3 {
                let reference = &expected["schur"]["landmarks"][id];
                copy(
                    exact.hll_inv.get_mut(&id).unwrap().as_mut_slice(),
                    3,
                    3,
                    &reference["inverse"],
                );
                copy(
                    exact.bl.get_mut(&id).unwrap().as_mut_slice(),
                    3,
                    1,
                    &reference["bl"],
                );
            }
            for (i, frame) in exact.hpppl.iter_mut().enumerate() {
                let reference = &expected["schur"]["frames"][i];
                copy(frame.hpp.as_mut_slice(), 6, 6, &reference["hpp"]);
                copy(frame.bp.as_mut_slice(), 6, 1, &reference["bp"]);
                for (j, cross) in frame.hpl.iter_mut().enumerate() {
                    copy(cross.as_mut_slice(), 6, 3, &reference["cross"][j]);
                }
            }
            if !expected["schur"]["reduced_b"].is_null() {
                let (h, b) = firefly_basalt::vi_estimator::sc_ba_base::linearize_rel(&exact);
                verify(
                    b.as_slice(),
                    b.len(),
                    1,
                    &expected["schur"]["reduced_b"],
                    "Schur exact-input reduced rhs",
                );
                verify(
                    h.as_slice(),
                    h.nrows(),
                    h.ncols(),
                    &expected["schur"]["reduced_h"],
                    "Schur exact-input reduced H",
                );
            }
        }
        let prior = MargLinData {
            is_sqrt: true,
            order: order.clone(),
            h: DMatrix::identity(18, 18) * scalar(0.5),
            b: DVector::from_iterator(
                18,
                (0..18)
                    .map(|i| scalar(-0.125) + scalar(f64::from(i)) * (scalar(0.25) / scalar(17.))),
            ),
        };
        let mut linearization = LinearizationBase::<S, 6>::create(
            &ba,
            &order,
            options,
            with_prior.then_some(&prior),
            with_imu.then_some(&imu),
            None,
            None,
            i64::MAX,
        );
        let (error, valid) = linearization.linearize_problem(&ba);
        assert_eq!(valid, expected["valid"].as_bool().unwrap());
        check(error, expected["error"].as_f64().unwrap(), "error");
        linearization.perform_qr();
        let mut root_pairs = None;
        if kind == LinearizationType::AbsQr && !expected["root"].is_null() {
            let (jacobian, residual) = linearization.get_dense_q2jp_q2r(&ba);
            let root = &expected["root"];
            let mut unmatched: Vec<_> = (0..jacobian.nrows()).collect();
            let mut columns = vec![Vec::new(); jacobian.ncols() + 1];
            for row in 0..jacobian.nrows() {
                // Landmark hash iteration changes row order, but not each block's QR.
                let nearest = unmatched
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        let distance = |candidate: usize| {
                            (0..jacobian.ncols())
                                .map(|col| {
                                    (jacobian[(row, col)].as_f64()
                                        - root["jacobian"]["values"]
                                            [candidate * jacobian.ncols() + col]
                                            .as_f64()
                                            .unwrap())
                                    .powi(2)
                                })
                                .sum::<f64>()
                        };
                        distance(**a).total_cmp(&distance(**b))
                    })
                    .unwrap()
                    .0;
                let expected_row = unmatched.remove(nearest);
                for col in 0..jacobian.ncols() {
                    let value = root["jacobian"]["values"][expected_row * jacobian.ncols() + col]
                        .as_f64()
                        .unwrap();
                    check(
                        jacobian[(row, col)],
                        value,
                        &format!("root J[{row},{col}] prior={with_prior}"),
                    );
                    columns[col].push((jacobian[(row, col)].as_f64(), value));
                }
                let value = root["residual"]["values"][expected_row].as_f64().unwrap();
                check(residual[row], value, "root residual");
                columns[jacobian.ncols()].push((residual[row].as_f64(), value));
            }
            root_pairs = Some(columns);
        }
        let (h, b) = linearization.get_dense_h_b(&ba);
        let size = order.total_size;
        for row in 0..size {
            for col in 0..size {
                if let Some(columns) = &root_pairs {
                    check_dot(
                        h[(row, col)],
                        expected["h"]["values"][row * size + col].as_f64().unwrap(),
                        &columns[row],
                        &columns[col],
                        "normal Hessian",
                    );
                    continue;
                }
                check(
                    h[(row, col)],
                    expected["h"]["values"][row * size + col].as_f64().unwrap(),
                    &format!("{kind:?} H[{row},{col}]"),
                );
            }
            if let Some(columns) = &root_pairs {
                check_dot(
                    b[row],
                    expected["b"]["values"][row].as_f64().unwrap(),
                    &columns[row],
                    &columns[size],
                    "normal rhs",
                );
            } else {
                check(
                    b[row],
                    expected["b"]["values"][row].as_f64().unwrap(),
                    &format!("{kind:?} prior={with_prior} rhs[{row}]"),
                );
            }
        }
        let increment =
            DVector::from_iterator(size, (0..size).map(|i| scalar((i as f64 - 8.) * 0.000_125)));
        let decrease = linearization.back_substitute(&mut ba, &increment);
        check(
            decrease,
            expected["model_decrease"].as_f64().unwrap(),
            "model decrease",
        );
        for id in 0..3 {
            let point = ba.lmdb.landmark(id);
            let expected = &expected["points"][id];
            for coordinate in 0..2 {
                check(
                    point.direction[coordinate],
                    expected["direction"]["values"][coordinate]
                        .as_f64()
                        .unwrap(),
                    "direction",
                );
            }
            check(
                point.inv_dist,
                expected["inverse_distance"].as_f64().unwrap(),
                "inverse distance",
            );
        }
    }
}

#[test]
fn f64_three_modes_match_original_multicamera_window() {
    run::<f64>(include_str!("reference/linearization.json"), None);
}

#[test]
fn f32_abs_qr_matches_original_multicamera_window() {
    run::<f32>(
        include_str!("reference/linearization_f32.json"),
        Some(LinearizationType::AbsQr),
    );
}
#[test]
fn f32_abs_sc_matches_original_multicamera_window() {
    run::<f32>(
        include_str!("reference/linearization_f32.json"),
        Some(LinearizationType::AbsSc),
    );
}
#[test]
fn f32_rel_sc_matches_original_multicamera_window() {
    run::<f32>(
        include_str!("reference/linearization_f32.json"),
        Some(LinearizationType::RelSc),
    );
}
