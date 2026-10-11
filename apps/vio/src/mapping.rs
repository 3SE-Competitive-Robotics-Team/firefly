//! 地图关键帧发布；图像只经 IPC 交给地图描述子提取，不写入录制。
use firefly_pubsub::landmarks::{Keyframe, MAX_POINTS, Observation};
use firefly_vio::mapping::Snapshot;
use firefly_vio_core::sensor::CameraData;

#[allow(clippy::large_stack_arrays)]
pub fn message(snapshot: &Snapshot, camera: &CameraData, session: u64) -> Option<Keyframe> {
    if !snapshot.camera_time.total_cmp(&camera.timestamp).is_eq()
        || session == 0
        || snapshot.observations.len() > MAX_POINTS
    {
        return None;
    }
    let i = camera.sensor_ids.iter().position(|&id| id == 0)?;
    let image = camera.images.get(i)?;
    if image.width != 320 || image.height != 240 || image.data.len() != 320 * 240 {
        return None;
    }
    let q = snapshot.quaternion;
    let p = snapshot.position;
    let mut message = Keyframe {
        session,
        camera_time: snapshot.camera_time,
        odom: firefly_pubsub::odom::OdomMessage {
            timestamp: snapshot.imu_time,
            position_x: p.x,
            position_y: p.y,
            position_z: p.z,
            quat_x: q.x,
            quat_y: q.y,
            quat_z: q.z,
            quat_w: q.w,
            is_initialized: true,
            ..Default::default()
        },
        camera_rotation: std::array::from_fn(|i| snapshot.camera_rotation[(i / 3, i % 3)]),
        camera_translation: snapshot.camera_translation.into(),
        intrinsics: snapshot.intrinsics,
        fisheye: snapshot.fisheye,
        stereo: false,
        right_rotation: [0.; 9],
        right_translation: [0.; 3],
        right_intrinsics: [0.; 8],
        right_fisheye: false,
        right_image: [0; 320 * 240],
        width: 320,
        height: 240,
        image: [0; 320 * 240],
        count: snapshot.observations.len() as u32,
        observations: [Observation {
            track: 0,
            pixel: [0.; 2],
            normalized: [0.; 2],
            right_normalized: [0.; 2],
            has_right: false,
            point_body: [0.; 3],
            has_point: false,
        }; MAX_POINTS],
    };
    message.image.copy_from_slice(&image.data);
    if let Some(right) = &snapshot.right_camera
        && let Some(i) = camera.sensor_ids.iter().position(|&id| id == 1)
        && let Some(image) = camera.images.get(i)
        && image.width == 320
        && image.height == 240
        && image.data.len() == 320 * 240
    {
        message.stereo = true;
        message.right_image.copy_from_slice(&image.data);
        message.right_rotation = std::array::from_fn(|i| right.rotation[(i / 3, i % 3)]);
        message.right_translation = right.translation.into();
        message.right_intrinsics = right.intrinsics;
        message.right_fisheye = right.fisheye;
    }
    for (dst, src) in message.observations.iter_mut().zip(&snapshot.observations) {
        *dst = Observation {
            track: src.id as u64,
            pixel: src.pixel,
            normalized: src.normalized,
            right_normalized: src.right_normalized.unwrap_or([0.; 2]),
            has_right: src.right_normalized.is_some() && message.stereo,
            point_body: src.point_body.unwrap_or([0.; 3]),
            has_point: src.point_body.is_some(),
        };
    }
    Some(message)
}
