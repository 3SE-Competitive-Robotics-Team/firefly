//! MLE final-state, projection-type and covariance rejection contracts.
use super::*;
use firefly_vio_core::cam::SharedCamera;
use nalgebra::SVector;
use std::sync::Arc;

fn fixture(
    camera: &SharedCamera,
    invalid_covariance: bool,
    iterations: usize,
) -> Option<(GnState, DMatrix<f64>)> {
    let mut params = InitOptions {
        init_dyn_mle_max_iter: iterations,
        ..InitOptions::default()
    };
    params.camera_intrinsics.insert(0, camera.clone());
    params.camera_intrinsics.insert(1, camera.clone());
    params
        .camera_extrinsics
        .insert(0, SVector::from_row_slice(&[0., 0., 0., 1., 0., 0., 0.]));
    params
        .camera_extrinsics
        .insert(1, SVector::from_row_slice(&[0., 0., 0., 1., -0.1, 0., 0.]));
    let mut times = TimeSet::default();
    let mut orientations = HashMap::new();
    let mut positions = HashMap::new();
    let mut velocities = HashMap::new();
    let mut integrations = HashMap::new();
    let gravity = Vector3::z() * 9.81;
    for i in 0..4 {
        let t = f64::from(i) * 0.2;
        times.insert(t);
        orientations.insert(Tk::of(t), Vector4::new(0., 0., 0., 1.));
        positions.insert(
            Tk::of(t),
            Vector3::new(0.2 * t, if i == 0 { 0. } else { 0.02 }, 0.),
        );
        velocities.insert(Tk::of(t), Vector3::x() * 0.2);
        if i > 0 {
            let mut cpi = CpiV1::new(
                params.sigma_w,
                params.sigma_wb,
                params.sigma_a,
                params.sigma_ab,
                true,
            );
            for j in 0..20 {
                cpi.feed_imu(
                    f64::from(j) * 0.01,
                    f64::from(j + 1) * 0.01,
                    &Vector3::zeros(),
                    &gravity,
                    &Vector3::zeros(),
                    &gravity,
                );
            }
            if invalid_covariance {
                cpi.p_meas.fill(0.);
            }
            integrations.insert(Tk::of(t), Some(cpi));
        }
    }
    let mut features = HashMap::new();
    let mut points = HashMap::new();
    let mut counts = HashMap::new();
    let k = camera.value();
    for id in 0..12 {
        let point = Vector3::new(
            (id % 4) as f64 * 0.5 - 0.75,
            (id / 4) as f64 * 0.4 - 0.4,
            3. + (id % 3) as f64,
        );
        points.insert(id, point + Vector3::new(0.01, 0.02, -0.03));
        let mut feature = Feature {
            featid: id,
            ..Feature::default()
        };
        for cam_id in 0..2 {
            for &t in times.iter() {
                let local = point - Vector3::x() * (0.2 * t + cam_id as f64 * 0.1);
                let x = local.x / local.z;
                let y = local.y / local.z;
                // Independent pinhole/equidistant projection, without production CameraModel calls.
                let scale = if camera.is_fisheye() {
                    let r = x.hypot(y);
                    let th = r.atan();
                    if r == 0. {
                        1.
                    } else {
                        th / r
                            * (1.
                                + k[4] * th.powi(2)
                                + k[5] * th.powi(4)
                                + k[6] * th.powi(6)
                                + k[7] * th.powi(8))
                    }
                } else {
                    1.
                };
                feature.timestamps.entry(cam_id).or_default().push(t);
                feature.uvs.entry(cam_id).or_default().push(Vector2::new(
                    (k[0] * x * scale + k[2]) as f32,
                    (k[1] * y * scale + k[3]) as f32,
                ));
            }
        }
        features.insert(id, feature);
        counts.insert(id, 8);
    }
    run_mle(
        &params,
        &features,
        &times,
        &integrations,
        &counts,
        2,
        &orientations,
        &positions,
        &velocities,
        &points,
        Vector3::zeros(),
        Vector3::zeros(),
        gravity,
        &BTreeMap::from([(0, true), (1, true)]),
    )
}

#[test]
fn mle_uses_declared_camera_type_even_when_probe_projections_agree() {
    let k = [
        200.,
        200.,
        160.,
        120.,
        -4.732_079_062_486_056,
        9.904_673_872_151_434,
        0.,
        0.,
    ];
    let camera: SharedCamera = Arc::new(CamEqui::new(320, 240, &k));
    let rad = CamRadtan::new(320, 240, &k);
    for p in [Vector2::new(0.2, -0.3), Vector2::new(-0.4, 0.1)] {
        assert!((camera.distort_d(p) - rad.distort_d(p)).norm() < 0.01);
    }
    let (state, _) = fixture(&camera, false, 0).unwrap();
    assert!(state.calibs[&0].is_fisheye);
    assert_eq!(
        state.parameters(false).len(),
        state.poses.len() * 16 + state.features.len() * 3
    );
    assert_eq!(
        state.parameters(true).len() - state.parameters(false).len(),
        30
    );
    let p = Vector3::new(2., 1.6, 4.);
    let pose = GnPose {
        q: Vector4::new(0., 0., 0., 1.),
        p: Vector3::zeros(),
        v: Vector3::zeros(),
        bg: Vector3::zeros(),
        ba: Vector3::zeros(),
    };
    let radius = 0.5_f64.hypot(0.4);
    let theta = radius.atan();
    let scale = theta / radius * (1. + k[4] * theta.powi(2) + k[5] * theta.powi(4));
    let measured = Vector2::new(160. + 100. * scale, 120. + 80. * scale);
    let (res, _) = reproj_factor_residual_jacobian(
        &InitOptions::default(),
        &pose,
        &p,
        &state.calibs[&0],
        measured,
    );
    assert!(res.norm() < 2e-5);
    assert!((rad.distort_d(Vector2::new(0.5, 0.4)) - measured).norm() > 35.);
}

#[test]
fn covariance_whitening_preserves_correlations_and_quadratic_gradient() {
    // P=[[4,2],[2,3]], P^-1=[[3,-2],[-2,4]]/8.
    let p = DMatrix::from_row_slice(2, 2, &[4., 2., 2., 3.]);
    let expected = DMatrix::from_row_slice(2, 2, &[3., -2., -2., 4.]) / 8.;
    let w = sqrt_info(&p).unwrap();
    assert!((w.transpose() * &w - &expected).amax() < 1e-14);
    let r = DVector::from_vec(vec![1., -2.]);
    assert!(((&w * &r).norm_squared() - r.dot(&(&expected * &r))).abs() < 1e-14);
    let gradient = w.transpose() * (&w * &r);
    for axis in 0..2 {
        let mut plus = r.clone();
        let mut minus = r.clone();
        plus[axis] += 1e-5;
        minus[axis] -= 1e-5;
        let fd = 0.5 * ((&w * plus).norm_squared() - (&w * minus).norm_squared()) / 2e-5;
        assert!((fd - gradient[axis]).abs() < 1e-10);
    }
}

#[test]
fn invalid_covariance_rejects_whitening_and_entire_mle() {
    for p in [
        DMatrix::zeros(2, 2),
        DMatrix::from_diagonal(&DVector::from_vec(vec![1., -1.])),
        DMatrix::from_row_slice(2, 2, &[1., 0.2, 0., 1.]),
        DMatrix::from_element(2, 2, f64::NAN),
    ] {
        assert!(sqrt_info(&p).is_none());
    }
    let cam: SharedCamera = Arc::new(CamRadtan::new(
        320,
        240,
        &[200., 200., 160., 120., 0., 0., 0., 0.],
    ));
    assert!(fixture(&cam, true, 20).is_none());
}

#[test]
fn dynamic_mle_recovers_metric_motion_and_positive_marginal_covariance() {
    for camera in [
        Arc::new(CamRadtan::new(
            320,
            240,
            &[200., 200., 160., 120., 0., 0., 0., 0.],
        )) as SharedCamera,
        Arc::new(CamEqui::new(
            320,
            240,
            &[200., 200., 160., 120., 0., 0., 0., 0.],
        )) as SharedCamera,
    ] {
        let (state, information) = fixture(&camera, false, 100).expect("metric MLE must converge");
        for (i, p) in state.poses.iter().enumerate() {
            assert!(
                (p.p - Vector3::x() * 0.2 * (i as f64 * 0.2)).norm() < 1e-3,
                "position {:?}",
                p.p
            );
            assert!(
                (p.v - Vector3::x() * 0.2).norm() < 1e-3,
                "velocity {:?}",
                p.v
            );
            assert!(p.bg.norm() < 1e-4);
        }
        let cov = reconstruct_covariance(&InitOptions::default(), &information, 3).unwrap();
        assert!(cov.cholesky().is_some());
    }
}
