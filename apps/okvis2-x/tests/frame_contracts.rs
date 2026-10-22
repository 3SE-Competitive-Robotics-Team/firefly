// SPDX-License-Identifier: BSD-3-Clause
//! Public frame/cache and multi-camera overlap contracts use analytic pinhole geometry.
use firefly_okvis_x::{
    cameras::{
        n_camera_system::{CameraType, DistortionType, NCameraSystem},
        no_distortion::NoDistortion,
        pinhole_camera::PinholeCamera,
    },
    frames::{
        frame::{Frame, KeyPoint},
        multi_frame::MultiFrame,
    },
    kinematics::transformation::Transformation,
};
use jiff::Timestamp;
use nalgebra::{UnitQuaternion, Vector3, Vector4};
use purecv::core::{DynamicMatrix, types::Point2f};
use std::sync::Arc;
fn rig() -> NCameraSystem {
    let mut rig = NCameraSystem::new();
    for yaw in [0.0, 0.05, std::f64::consts::PI] {
        let t = Transformation::new(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.0, yaw, 0.0).into_inner(),
        );
        rig.add_camera(
            &Arc::new(t),
            Arc::new(PinholeCamera::new(
                (64, 48),
                (60.0, 60.0, 31.5, 23.5),
                NoDistortion,
                0,
            )),
            DistortionType::NoDistortion,
            true,
            CameraType::default(),
        );
    }
    rig
}
#[test]
fn overlap_is_symmetric_for_parallel_cameras_and_absent_for_opposite_views() {
    let rig = rig();
    assert!(rig.overlap_computation_valid());
    for c in 0..3 {
        assert!(rig.has_overlap(c, c));
    }
    assert!(rig.has_overlap(0, 1) && rig.has_overlap(1, 0));
    assert!(!rig.has_overlap(0, 2) && !rig.has_overlap(2, 0));
    assert_eq!((rig.overlap(0, 1).rows, rig.overlap(0, 1).cols), (48, 64));
}
#[test]
fn frame_ray_and_landmark_caches_follow_keypoint_reset_contract() {
    let rig = rig();
    let mut frame = Frame::default();
    frame.set_geometry(rig.camera_geometry(0).clone());
    let points = [
        KeyPoint::new(Point2f::new(31.5, 23.5), 12.0, -1.0, 1.0, 0, -1),
        KeyPoint::new(Point2f::new(43.5, 17.5), 12.0, -1.0, 1.0, 0, -1),
    ];
    frame.reset_keypoints(&points);
    assert_eq!(frame.compute_back_projections(), 2);
    for (i, expected) in [Vector3::z(), Vector3::new(0.2, -0.1, 1.0)]
        .into_iter()
        .enumerate()
    {
        let mut ray = Vector3::zeros();
        assert!(frame.get_back_projection(i, &mut ray));
        assert!((ray - expected).norm() < 1e-14);
    }
    assert!(frame.set_landmark_id(1, 42));
    assert!(frame.set_landmark(1, Vector4::new(1.0, 2.0, 3.0, 1.0), true));
    assert_eq!(frame.landmark_id(1), 42);
    frame.reset_keypoints(&points[..1]);
    assert_eq!(frame.num_keypoints(), 1);
    assert_eq!(frame.landmark_id(0), 0);
    let copy = frame.clone();
    assert_eq!(copy.num_keypoints(), 0);
    assert!(Arc::ptr_eq(frame.geometry(), copy.geometry()));
}
#[test]
fn multiframe_calibration_reset_preserves_identity_and_separates_camera_storage() {
    let rig = rig();
    let stamp = Timestamp::from_second(12).unwrap();
    let mut multi = MultiFrame::new(&rig, stamp, 7);
    let image = Arc::new(DynamicMatrix::new_u8(48, 64, 1, vec![23; 48 * 64]).unwrap());
    multi.set_image(1, image.clone());
    assert!(multi.frame(0).image().is_none());
    assert!(Arc::ptr_eq(multi.frame(1).image().unwrap(), &image));
    multi.frame_mut(0).reset_keypoints(&[KeyPoint::new(
        Point2f::new(31.5, 23.5),
        12.0,
        -1.0,
        1.0,
        0,
        -1,
    )]);
    assert_eq!(multi.total_keypoints(), 1);
    multi.reset_camera_system_and_frames(&rig);
    assert_eq!(multi.total_keypoints(), 0);
    assert!(multi.frame(1).image().is_none());
    assert_eq!(multi.timestamp(), stamp);
    assert_eq!(multi.id(), 7);
    assert_eq!(multi.num_frames(), 3);
}
