// Nonlinear factor Jacobian checks corresponding to upstream test_nfr.cpp.
use firefly_basalt::imu::imu_types::PoseState;
use firefly_basalt::utils::nfr::{abs_position_error, rel_pose_error, roll_pitch_error, yaw_error};
use firefly_basalt::utils::sophus_utils::se3_expd;
use nalgebra::{SMatrix, SVector, Vector3};

#[test]
fn nonlinear_factor_jacobians() {
    let a = se3_expd(&SVector::<f64, 6>::new(0.2, -0.7, 0.4, 0.3, -0.2, 0.4));
    let b = se3_expd(&SVector::<f64, 6>::new(-0.5, 0.1, 0.9, -0.4, 0.2, -0.1));
    let measured = se3_expd(&SVector::<f64, 6>::repeat(0.003)) * a.inverse() * b;
    let mut ja = SMatrix::<f64, 6, 6>::zeros();
    let mut jb = ja;
    let _ = rel_pose_error(&measured, &a, &b, Some(&mut ja), Some(&mut jb));
    let mut position_j = SMatrix::<f64, 3, 6>::zeros();
    let mut yaw_j = SMatrix::<f64, 1, 6>::zeros();
    let mut rp_j = SMatrix::<f64, 2, 6>::zeros();
    let _ = abs_position_error(&a, &b.translation.vector, Some(&mut position_j));
    let _ = yaw_error(&a, &Vector3::x(), Some(&mut yaw_j));
    let _ = roll_pitch_error(&a, &b.rotation, Some(&mut rp_j));
    let epsilon = 1e-6;
    for col in 0..6 {
        let mut delta = SVector::<f64, 6>::zeros();
        delta[col] = epsilon;
        let (mut plus, mut minus) = (a, a);
        PoseState::inc_pose(&delta, &mut plus);
        PoseState::inc_pose(&(-delta), &mut minus);
        let numerical = (rel_pose_error(&measured, &plus, &b, None, None)
            - rel_pose_error(&measured, &minus, &b, None, None))
            / (2. * epsilon);
        assert!((numerical - ja.column(col)).amax() < 1e-8);
        let position = (abs_position_error(&plus, &b.translation.vector, None)
            - abs_position_error(&minus, &b.translation.vector, None))
            / (2. * epsilon);
        assert!((position - position_j.column(col)).amax() < 1e-8);
        let yaw = (yaw_error(&plus, &Vector3::x(), None) - yaw_error(&minus, &Vector3::x(), None))
            / (2. * epsilon);
        assert!((yaw - yaw_j[col]).abs() < 1e-8);
        let rp = (roll_pitch_error(&plus, &b.rotation, None)
            - roll_pitch_error(&minus, &b.rotation, None))
            / (2. * epsilon);
        assert!((rp - rp_j.column(col)).amax() < 1e-8);
        let (mut plus, mut minus) = (b, b);
        PoseState::inc_pose(&delta, &mut plus);
        PoseState::inc_pose(&(-delta), &mut minus);
        let numerical = (rel_pose_error(&measured, &a, &plus, None, None)
            - rel_pose_error(&measured, &a, &minus, None, None))
            / (2. * epsilon);
        assert!((numerical - jb.column(col)).amax() < 1e-8);
    }
}

#[test]
fn full_pivot_qr_rank_and_rectangular_solution() {
    use firefly_basalt::utils::full_piv_householder_qr::FullPivHouseholderQr;
    use nalgebra::DMatrix;
    let matrix = DMatrix::from_row_slice(4, 3, &[0., 1., 0., 0., 0., 4., 2., 0., 0., 1., -1., 2.]);
    let solution = DMatrix::from_row_slice(3, 2, &[2., -1., 0.2, 4., 3., 0.5]);
    let rhs = &matrix * &solution;
    let qr = FullPivHouseholderQr::new(matrix);
    assert_eq!(qr.rank(), 3);
    assert!((qr.solve(&rhs) - solution).amax() < 1e-14);
    let matrix = DMatrix::from_row_slice(3, 3, &[1., 2., 3., 2., 4., 6., -1., -2., -3.]);
    let qr = FullPivHouseholderQr::new(matrix.clone());
    assert_eq!(qr.rank(), 1);
    assert!((&matrix * qr.solve(&matrix) - matrix).amax() < 1e-14);
    assert_eq!(FullPivHouseholderQr::new(DMatrix::zeros(3, 3)).rank(), 0);
}

#[test]
fn mapper_extracts_covariances_and_optimizes_factors() {
    use firefly_basalt::{
        utils::{
            imu_types::{MargData, PoseStateWithLin},
            vio_config::VioConfig,
        },
        vi_estimator::nfr_mapper::NfrMapper,
    };
    use firefly_calibration::calibration::Calibration;
    use nalgebra::{DMatrix, DVector, Isometry3};
    let mut data = MargData {
        abs_h: DMatrix::identity(12, 12),
        abs_b: DVector::zeros(12),
        use_imu: true,
        ..Default::default()
    };
    data.aom.total_size = 12;
    data.aom.abs_order_map.insert(10, (0, 6));
    data.aom.abs_order_map.insert(20, (6, 6));
    data.kfs_all.extend([10, 20]);
    data.kfs_to_marg.insert(10);
    for id in [10, 20] {
        data.frame_poses
            .insert(id, PoseStateWithLin::new(id, Isometry3::identity(), false));
    }
    for use_lm in [false, true] {
        let config = VioConfig {
            mapper_use_factors: true,
            mapper_use_lm: use_lm,
            ..Default::default()
        };
        let mut mapper = NfrMapper::new(&Calibration::default(), &config);
        mapper.add_marg_data(&mut data.clone());
        assert_eq!(mapper.rel_pose_factors.len(), 1);
        assert_eq!(mapper.roll_pitch_factors.len(), 1);
        assert!(
            (mapper.rel_pose_factors[0].cov_inv - SMatrix::<f64, 6, 6>::identity() * 0.5).amax()
                < 1e-14
        );
        assert!(
            (mapper.roll_pitch_factors[0].cov_inv - SMatrix::<f64, 2, 2>::identity()).amax()
                < 1e-14
        );
        mapper.base.frame_poses.insert(
            20,
            PoseStateWithLin::new(
                20,
                se3_expd(&SVector::<f64, 6>::new(0.1, -0.2, 0.3, 0.01, -0.02, 0.03)),
                false,
            ),
        );
        let before = mapper.compute_rel_pose() + mapper.compute_roll_pitch();
        mapper.optimize(10);
        let after = mapper.compute_rel_pose() + mapper.compute_roll_pitch();
        assert!(after < before * 1e-8, "{use_lm}: {before} -> {after}");
    }
    data.abs_h.fill(0.);
    let mut mapper = NfrMapper::new(&Calibration::default(), &VioConfig::default());
    assert!(!mapper.extract_nonlinear_factors(&data));
    assert_eq!(mapper.rel_pose_factors.len(), 0);
}

#[test]
fn original_cpp_nfr_and_qr_reference() {
    use firefly_basalt::utils::full_piv_householder_qr::FullPivHouseholderQr;
    use nalgebra::DMatrix;
    fn matrix(value: &serde_json::Value) -> DMatrix<f64> {
        let rows = value.as_array().unwrap();
        DMatrix::from_fn(rows.len(), rows[0].as_array().unwrap().len(), |r, c| {
            rows[r][c].as_f64().unwrap()
        })
    }
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/nfr.json")).unwrap();
    let a = se3_expd(&SVector::<f64, 6>::new(0.2, -0.7, 0.4, 0.3, -0.2, 0.4));
    let b = se3_expd(&SVector::<f64, 6>::new(-0.5, 0.1, 0.9, -0.4, 0.2, -0.1));
    let measured = se3_expd(&SVector::<f64, 6>::repeat(0.003)) * a.inverse() * b;
    let (mut ja, mut jb) = (SMatrix::<f64, 6, 6>::zeros(), SMatrix::<f64, 6, 6>::zeros());
    let residual = rel_pose_error(&measured, &a, &b, Some(&mut ja), Some(&mut jb));
    assert!((residual - matrix(&fixture["residual"]).column(0)).amax() < 2e-14);
    assert!((ja - matrix(&fixture["ja"])).amax() < 2e-14);
    assert!((jb - matrix(&fixture["jb"])).amax() < 2e-14);
    let mut jp = SMatrix::<f64, 3, 6>::zeros();
    let mut jy = SMatrix::<f64, 1, 6>::zeros();
    let mut jr = SMatrix::<f64, 2, 6>::zeros();
    let position = abs_position_error(&a, &b.translation.vector, Some(&mut jp));
    let yaw = yaw_error(&a, &Vector3::x(), Some(&mut jy));
    let rp = roll_pitch_error(&a, &b.rotation, Some(&mut jr));
    assert!((position - matrix(&fixture["position"]).column(0)).amax() < 2e-14);
    assert!((yaw - fixture["yaw"].as_f64().unwrap()).abs() < 2e-14);
    assert!((rp - matrix(&fixture["roll_pitch"]).column(0)).amax() < 2e-14);
    assert!((jp - matrix(&fixture["position_j"])).amax() < 2e-14);
    assert!((jy - matrix(&fixture["yaw_j"])).amax() < 2e-14);
    assert!((jr - matrix(&fixture["roll_pitch_j"])).amax() < 2e-14);
    for case in fixture["qr"].as_array().unwrap() {
        let a = matrix(&case["matrix"]);
        let b = matrix(&case["rhs"]);
        let qr = FullPivHouseholderQr::new(a.clone());
        assert_eq!(qr.rank(), case["rank"].as_u64().unwrap() as usize);
        let solution = qr.solve(&b);
        let residual = (&a * &solution - &b).norm();
        let scale = a.norm() * solution.norm() + b.norm();
        assert!(residual <= 128. * f64::EPSILON * scale);
    }
}
