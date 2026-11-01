use firefly_basalt::device::rs_t265::{
    CameraProfile, MotionIntrinsics, RsImuData, RsT265Processor, VideoFrame, export_calibration,
};
use nalgebra::Vector3;

#[test]
fn interpolation_preserves_queue_boundaries() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/rs_t265.json")).unwrap();
    let mut p = RsT265Processor::new(false, 1, 90);
    for t in [9., 10., 10.25, 11.5, 12.] {
        p.gyro(RsImuData {
            timestamp: t,
            data: Vector3::repeat(t),
        });
    }
    assert_eq!(
        p.accel(RsImuData {
            timestamp: 10.,
            data: Vector3::new(1., -2., 3.)
        })
        .unwrap()
        .len(),
        0
    );
    let mut values = p
        .accel(RsImuData {
            timestamp: 12.,
            data: Vector3::new(5., 6., -1.),
        })
        .unwrap();
    assert_eq!(values.len(), 3); // exact upper endpoint remains queued
    values.extend(
        p.accel(RsImuData {
            timestamp: 13.,
            data: Vector3::zeros(),
        })
        .unwrap(),
    );
    for (actual, expected) in values
        .iter()
        .zip(fixture["interpolation"].as_array().unwrap())
    {
        assert_eq!(actual.t_ns, expected["t_ns"].as_i64().unwrap());
        for axis in 0..3 {
            assert_eq!(
                actual.accel[axis],
                expected["accel"][axis].as_f64().unwrap()
            );
        }
    }
    assert!(
        p.accel(RsImuData {
            timestamp: 13.,
            data: Vector3::zeros()
        })
        .is_err()
    );
}

fn frame(timestamp: f64) -> VideoFrame<'static> {
    VideoFrame {
        timestamp,
        exposure_us: 1250,
        width: 2,
        height: 2,
        pixels: &[0, 1, 128, 255],
    }
}
#[test]
fn image_pair_skip_exposure_and_pose() {
    let mut p = RsT265Processor::new(true, 2, 90);
    assert!(p.images([frame(1.), frame(2.)]).unwrap().is_none());
    let first = p.images([frame(3.), frame(3.)]).unwrap().unwrap();
    assert_eq!(first.t_ns, 3_000_000);
    assert_eq!(first.img_data[0].exposure, 0.00125);
    assert_eq!(
        first.img_data[0].img.as_ref().unwrap().data(),
        &[0, 256, 32768, 65280]
    );
    assert!(p.images([frame(4.), frame(4.)]).unwrap().is_none());
    assert_eq!(p.last_image_data().unwrap().t_ns, first.t_ns);
    assert!(p.images([frame(5.), frame(5.)]).unwrap().is_some());
    assert_eq!(p.exposure_option(2.5), Some(2500.));
    assert_eq!(
        RsT265Processor::new(false, 1, 90).exposure_option(2.5),
        None
    );
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/rs_t265.json")).unwrap();
    for (i, input) in [[0.1, -0.2, 0.3, 0.9], [2., 3., 4., 5.]]
        .into_iter()
        .enumerate()
    {
        let pose = RsT265Processor::pose(1., [1., 2., 3.], input)
            .unwrap()
            .unwrap();
        for axis in 0..4 {
            assert!(
                (pose.data.rotation.coords[axis]
                    - fixture["quaternions"][i][axis].as_f64().unwrap())
                .abs()
                    < 1e-15
            );
        }
    }
    assert!(
        RsT265Processor::pose(1., [0.; 3], [0.; 4])
            .unwrap()
            .is_none()
    );
    assert!(
        RsT265Processor::pose(1., [f32::NAN, 0., 0.], [0., 0., 0., 1.])
            .unwrap()
            .is_none()
    );
}

#[test]
fn sdk_calibration_layout() {
    let motion = MotionIntrinsics {
        data: [[1., 2., 3., 4.], [5., 6., 7., 8.], [9., 10., 11., 12.]],
        noise_variances: [1., 4., 9.],
        bias_variances: [16., 25., 36.],
    };
    let camera = CameraProfile {
        width: 848,
        height: 800,
        fx: 285.,
        fy: 286.,
        ppx: 424.,
        ppy: 400.,
        coefficients: [1., 2., 3., 4., 99.],
        rotation: [0., 1., 0., -1., 0., 0., 0., 0., 1.],
        translation: [0.125, -0.25, 0.5],
    };
    let c = export_calibration(&motion, &motion, &[camera.clone(), camera]);
    assert_eq!(c.imu_update_rate, 200.);
    assert_eq!(
        c.calib_gyro_bias.param().as_slice(),
        &[4., 8., 12., 0., 5., 9., 2., 5., 10., 3., 7., 10.]
    );
    assert_eq!(
        c.calib_accel_bias.param().as_slice(),
        &[4., 8., 12., 0., 5., 9., 5., 10., 10.]
    );
    assert_eq!(c.gyro_noise_std, Vector3::new(1., 2., 3.));
    assert_eq!(c.accel_bias_std, Vector3::new(4., 5., 6.));
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/rs_t265.json")).unwrap();
    for axis in 0..4 {
        assert!(
            (c.t_i_c[0].rotation.coords[axis]
                - fixture["extrinsic_quaternion"][axis].as_f64().unwrap())
            .abs()
                < 1e-15
        );
    }
    assert_eq!(c.intrinsics.len(), 2);
    assert_eq!(c.resolution[0].x, 848);
}
