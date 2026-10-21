// SPDX-License-Identifier: BSD-3-Clause

//! Analytic pinhole geometry and explicit pixel/mask boundaries, independent of C++.
use firefly_srl_projection::{
    pinhole_camera::ProjectionPinholeCamera,
    projection_base::{ProjectionBase, ProjectionStatus},
};
use nalgebra::{SMatrix, Vector2, Vector3};
use proptest::prelude::*;
use purecv::core::Matrix;
use std::sync::Arc;

fn camera() -> ProjectionPinholeCamera {
    ProjectionPinholeCamera::new(640, 480, 320.0, 300.0, 319.5, 239.5)
}

proptest! {
    #![proptest_config(ProptestConfig::with_failure_persistence(
        proptest::test_runner::FileFailurePersistence::WithSource("proptest-regressions")
    ))]
    #[test]
    fn projection_agrees_with_perspective_and_is_depth_scale_invariant(
        x in -0.8f32..0.8, y in -0.6f32..0.6, z in 0.1f32..30.0, scale in 0.1f32..10.0
    ) {
        let c = camera();
        let p = Vector3::new(x*z, y*z, z);
        let (status, pixel) = c.project(&p);
        prop_assert_eq!(status, ProjectionStatus::Successful);
        let expected = Vector2::new(320.0*x+319.5, 300.0*y+239.5);
        // f32 perspective at VGA resolution: a few arithmetic-rounding ULPs, not pixel-sized error.
        prop_assert!((pixel - expected).amax() < 1e-4);
        let (scaled_status, scaled_pixel) = c.project(&(p*scale));
        prop_assert_eq!(scaled_status, status);
        prop_assert!((scaled_pixel - pixel).amax() < 1e-4);
        let ray = c.back_project(&pixel);
        prop_assert!((ray - Vector3::new(x, y, 1.0)).amax() < 5e-7);
    }
}

#[test]
fn back_projection_has_analytic_jacobian_and_central_difference() {
    let c = camera();
    let pixel = Vector2::new(101.25, 390.75);
    let (ray, j) = c.back_project_with_jacobian(&pixel);
    assert_eq!(
        j,
        SMatrix::<f32, 3, 2>::new(1.0 / 320.0, 0.0, 0.0, 1.0 / 300.0, 0.0, 0.0)
    );
    assert_eq!(ray.z, 1.0);
    for axis in 0..2 {
        let mut plus = pixel;
        let mut minus = pixel;
        plus[axis] += 0.25;
        minus[axis] -= 0.25;
        assert!(
            ((c.back_project(&plus) - c.back_project(&minus)) / 0.5 - j.column(axis)).amax() < 2e-7
        );
    }
}

#[test]
fn half_pixel_bounds_and_nonzero_mask_convention_are_explicit() {
    let mut base = ProjectionBase::new(4, 3);
    assert!(base.is_in_image(&Vector2::new(-0.5, -0.5)));
    assert!(!base.is_in_image(&Vector2::new((-0.5f32).next_down(), 0.0)));
    assert!(!base.is_in_image(&Vector2::new(3.5, 0.0)));
    assert!(!base.is_in_image(&Vector2::new(0.0, 2.5)));
    let mut mask = Matrix::<u8>::new(3, 4, 1);
    mask.data[6] = 255;
    assert!(base.set_mask(Arc::new(mask)));
    assert!(base.is_masked(&Vector2::new(2.9, 1.9)).unwrap());
    assert!(!base.is_masked(&Vector2::new(1.9, 1.9)).unwrap());
    assert!(base.is_masked(&Vector2::new(f32::NAN, 1.0)).is_err());
    assert!(!base.set_mask(Arc::new(Matrix::<u8>::new(4, 3, 1))));
    assert!(base.has_mask(), "invalid replacement retains valid mask");
    assert!(base.remove_mask());
    assert!(!base.has_mask());
}

#[test]
fn singular_projection_retains_output_and_batch_retains_status_order() {
    let c = camera();
    for z in [0.0, -0.0, 1e-13, -1e-13] {
        let mut pixel = Vector2::new(17.0, 19.0);
        assert_eq!(
            c.project_into(&Vector3::new(0.0, 0.0, z), &mut pixel),
            ProjectionStatus::Invalid
        );
        assert_eq!(pixel, Vector2::new(17.0, 19.0));
    }
    let points = [
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(10.0, 0.0, 1.0),
        Vector3::zeros(),
    ];
    let batch = c.project_batch(&points);
    assert_eq!(
        batch.iter().map(|p| p.0).collect::<Vec<_>>(),
        [
            ProjectionStatus::Successful,
            ProjectionStatus::Behind,
            ProjectionStatus::OutsideImage,
            ProjectionStatus::Invalid
        ]
    );
    assert!(batch[3].1.iter().all(|v| v.is_nan()));
}
