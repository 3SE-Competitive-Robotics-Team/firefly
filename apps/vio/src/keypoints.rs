//! 当前 KLT 活动点的只读可视化快照（不依赖可能已被滤波更新清理的特征历史）。

use firefly_pubsub::viz::{VIO_KEYPOINT_ENTITIES, VizMessage, VizPublisher, kind};
use firefly_vio_core::sensor::CameraData;
use firefly_vio_core::track::{KeyPoint, TrackKlt};

#[fastrace::trace]
pub fn publish(publisher: &VizPublisher, camera: &CameraData, tracker: &TrackKlt) {
    if !camera.is_valid() || camera.images.len() > 2 {
        return;
    }
    for (&sensor, image) in camera.sensor_ids.iter().zip(&camera.images) {
        let Ok(index @ 0..=1) = usize::try_from(sensor) else {
            continue;
        };
        let msg = snapshot(
            camera.timestamp,
            VIO_KEYPOINT_ENTITIES[index],
            image.width,
            image.height,
            tracker.last_points(sensor),
        );
        if let Err(error) = publisher.publish(msg) {
            log::debug!("VIO 关键点发布失败：{error}");
        }
    }
}

fn snapshot(
    timestamp: f64,
    entity: &str,
    width: usize,
    height: usize,
    points: &[KeyPoint],
) -> VizMessage {
    let mut msg = VizMessage::base(kind::KEYPOINTS_2D, timestamp, entity);
    msg.color = [64, 255, 128];
    msg.scalars[..3].copy_from_slice(&[width as f64, height as f64, points.len() as f64]);
    msg.scalar_count = 3;
    let n = points.len().min(msg.points.len());
    for (dst, point) in msg.points.iter_mut().zip(points) {
        *dst = [f64::from(point.x), f64::from(point.y), 0.0];
    }
    msg.point_count = n as u32;
    msg
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_pubsub::viz::POINTS_MAX;

    #[test]
    fn snapshot_preserves_camera_time_pixels_and_reports_truncation() {
        let points = vec![
            KeyPoint {
                x: 12.25,
                y: 41.5,
                response: 99.0
            };
            POINTS_MAX + 3
        ];
        let msg = snapshot(1.25, VIO_KEYPOINT_ENTITIES[1], 320, 240, &points);
        assert_eq!(msg.timestamp, 1.25);
        assert_eq!(msg.entity(), VIO_KEYPOINT_ENTITIES[1]);
        assert_eq!(msg.points[0], [12.25, 41.5, 0.0]);
        assert_eq!(msg.point_count as usize, POINTS_MAX);
        assert_eq!(msg.scalars[..3], [320.0, 240.0, (POINTS_MAX + 3) as f64]);
        let empty = snapshot(1.5, VIO_KEYPOINT_ENTITIES[0], 320, 240, &[]);
        assert_eq!(empty.point_count, 0);
        assert_eq!(empty.scalars[2], 0.0);
    }
}
