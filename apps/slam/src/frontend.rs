//! 关键帧外观与双目几何由地图进程计算，不占用 VIO 相机更新线程。
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use firefly_landmarks::{
    Feature, Frame, descriptor,
    geometry::{self, CAMERA, Camera},
};
use firefly_pubsub::landmarks::{self, Keyframe};
use firefly_vio_core::{
    cam::{CamEqui, CamRadtan, CameraModel},
    sensor::GrayImage,
    track,
};
use nalgebra::{Matrix3, UnitQuaternion, Vector2};

pub fn convert(message: &Keyframe) -> Result<Frame> {
    let count = message.count as usize;
    if count > landmarks::MAX_POINTS
        || message.width != 320
        || message.height != 240
        || !message.odom.is_initialized
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "invalid VIO keyframe",
        ));
    }
    let observations = &message.observations[..count];
    let descriptors = descriptor::extract(
        &message.image,
        320,
        240,
        &observations.iter().map(|f| f.pixel).collect::<Vec<_>>(),
    )?;
    let rotation = Matrix3::from_row_slice(&message.camera_rotation);
    if !rotation.iter().all(|v| v.is_finite())
        || (rotation.transpose() * rotation - Matrix3::identity()).norm() > 1e-6
        || (rotation.determinant() - 1.).abs() > 1e-6
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "invalid camera rotation",
        ));
    }
    let q = UnitQuaternion::from_matrix(&rotation);
    let camera = Camera {
        camera_from_body: RigidTransform::from_parts(
            CAMERA,
            FrameId::BODY,
            message.camera_translation,
            [q.i, q.j, q.k, q.w],
        )?,
        intrinsics: message.intrinsics[..4].try_into().unwrap(),
    };
    let raw = message.odom.body_pose(FrameId::ODOM)?;
    let mut frame = Frame {
        timestamp: message.odom.timestamp,
        pose: RigidTransform::from_matrix(FrameId::MAP, FrameId::BODY, &raw.matrix())?,
        raw,
        camera,
        right_camera: None,
        features: observations
            .iter()
            .zip(descriptors)
            .filter_map(|(f, d)| {
                Some(Feature {
                    track: f.track,
                    pixel: f.pixel,
                    normalized: f.normalized,
                    right_normalized: f.has_right.then_some(f.right_normalized),
                    descriptor: d?,
                    seed_body: f.has_point.then_some(f.point_body),
                })
            })
            .collect(),
    };
    supplement(message, &mut frame)?;
    Ok(frame)
}

fn model(intrinsics: &[f64; 8], fisheye: bool) -> Result<Box<dyn CameraModel>> {
    if !intrinsics.iter().all(|v| v.is_finite()) || intrinsics[0] <= 0. || intrinsics[1] <= 0. {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "invalid camera intrinsics",
        ));
    }
    let camera: Box<dyn CameraModel> = if fisheye {
        Box::new(CamEqui::new(320, 240, intrinsics))
    } else {
        Box::new(CamRadtan::new(320, 240, intrinsics))
    };
    Ok(camera)
}

#[fastrace::trace]
fn supplement(message: &Keyframe, frame: &mut Frame) -> Result<()> {
    let left = GrayImage {
        width: 320,
        height: 240,
        data: message.image.to_vec(),
    };
    let detected = descriptor::detect(&message.image, 320, 240)?;
    let mut cells = [0u8; 48];
    let camera = model(&message.intrinsics, message.fisheye)?;
    let mut features: Vec<Feature> = Vec::new();
    for detected in detected {
        let [x, y] = detected.pixel;
        let cell = y as usize / 40 * 8 + x as usize / 40;
        if cell >= cells.len()
            || cells[cell] >= 12
            || frame
                .features
                .iter()
                .chain(&features)
                .any(|f| (f.pixel[0] - x).hypot(f.pixel[1] - y) < 2.)
        {
            continue;
        }
        cells[cell] += 1;
        let uv = camera.undistort_f(detected.pixel.into());
        features.push(Feature {
            track: u64::MAX,
            pixel: detected.pixel,
            normalized: [f64::from(uv.x), f64::from(uv.y)],
            right_normalized: None,
            descriptor: detected.descriptor,
            seed_body: None,
        });
    }
    if message.stereo {
        let right_model = model(&message.right_intrinsics, message.right_fisheye)?;
        let rotation = Matrix3::from_row_slice(&message.right_rotation);
        if !rotation.iter().all(|v| v.is_finite())
            || (rotation.transpose() * rotation - Matrix3::identity()).norm() > 1e-6
            || (rotation.determinant() - 1.).abs() > 1e-6
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid stereo rotation",
            ));
        }
        let q = UnitQuaternion::from_matrix(&rotation);
        let right_camera = Camera {
            camera_from_body: RigidTransform::from_parts(
                CAMERA,
                FrameId::BODY,
                message.right_translation,
                [q.i, q.j, q.k, q.w],
            )?,
            intrinsics: message.right_intrinsics[..4].try_into().unwrap(),
        };
        let right = GrayImage {
            width: 320,
            height: 240,
            data: message.right_image.to_vec(),
        };
        let pixels: Vec<Vector2<f32>> = features.iter().map(|f| f.pixel.into()).collect();
        let (tracked, valid) = track::lk_optical_flow(&left, &right, &pixels, &pixels);
        for ((feature, pixel), valid) in features.iter_mut().zip(tracked).zip(valid) {
            if !valid || pixel.x < 0. || pixel.y < 0. || pixel.x >= 320. || pixel.y >= 240. {
                continue;
            }
            let uv = right_model.undistort_f(pixel);
            let observations = [
                (&frame.pose, &frame.camera, feature.normalized),
                (
                    &frame.pose,
                    &right_camera,
                    [f64::from(uv.x), f64::from(uv.y)],
                ),
            ];
            if let Some(point) = geometry::triangulate(&observations) {
                feature.seed_body = Some(frame.pose.inverse().point(point.into()).coords.into());
                feature.right_normalized = Some([f64::from(uv.x), f64::from(uv.y)]);
            }
        }
        frame.right_camera = Some(right_camera);
    }
    frame.features.extend(features);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_pubsub::{landmarks::Observation, odom::OdomMessage};
    #[test]
    #[allow(clippy::large_stack_arrays)]
    fn calibrated_stereo_builds_geometry_at_its_own_pixels() {
        let mut message = Keyframe {
            session: 1,
            camera_time: 1.,
            odom: OdomMessage {
                timestamp: 1.,
                is_initialized: true,
                ..Default::default()
            },
            camera_rotation: [1., 0., 0., 0., 1., 0., 0., 0., 1.],
            camera_translation: [0.; 3],
            intrinsics: [100., 100., 160., 120., 0., 0., 0., 0.],
            fisheye: false,
            stereo: true,
            right_rotation: [1., 0., 0., 0., 1., 0., 0., 0., 1.],
            right_translation: [-0.1, 0., 0.],
            right_intrinsics: [100., 100., 160., 120., 0., 0., 0., 0.],
            right_fisheye: false,
            image: [0; 320 * 240],
            right_image: [0; 320 * 240],
            width: 320,
            height: 240,
            count: 1,
            observations: [Observation {
                track: 7,
                pixel: [100., 100.],
                normalized: [-0.6, -0.2],
                right_normalized: [-0.65, -0.2],
                has_right: true,
                point_body: [-1.2, -0.4, 2.],
                has_point: true,
            }; landmarks::MAX_POINTS],
        };
        let mut random = 42u64;
        for pixel in &mut message.image {
            random = random
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            *pixel = (random >> 32) as u8;
        }
        let texture = smooth_texture(&message.image);
        message.image.copy_from_slice(&texture);
        for y in 0..240 {
            for x in 0..315 {
                message.right_image[y * 320 + x] = message.image[y * 320 + x + 5];
            }
        }
        let frame = convert(&message).unwrap();
        assert_eq!(frame.features[0].track, 7);
        assert_eq!(frame.features[0].seed_body, Some([-1.2, -0.4, 2.]));
        let points: Vec<_> = frame
            .features
            .iter()
            .skip(1)
            .filter_map(|f| f.seed_body.map(|p| (f, p)))
            .collect();
        assert!(points.len() > 20, "stereo points={}", points.len());
        for (feature, point) in points {
            assert!(
                (point[2] - 2.).abs() < 0.05,
                "pixel={:?}, point={point:?}",
                feature.pixel
            );
            assert!((point[0] / point[2] - feature.normalized[0]).abs() < 0.005);
        }
        message.camera_rotation[0] = f64::NAN;
        assert!(convert(&message).is_err());
    }

    fn smooth_texture(image: &[u8]) -> Vec<u8> {
        // 相机纹理先经过像素带宽限制，已知整数视差仍严格为 5px。
        (0..240)
            .flat_map(|y| {
                (0..320).map(move |x| {
                    let mut value = 0u32;
                    for dy in -1i32..=1 {
                        for dx in -1i32..=1 {
                            let u = (x + dx).clamp(0, 319) as usize;
                            let v = (y + dy).clamp(0, 239) as usize;
                            value += u32::from(image[v * 320 + u]);
                        }
                    }
                    (value / 9) as u8
                })
            })
            .collect()
    }
}
