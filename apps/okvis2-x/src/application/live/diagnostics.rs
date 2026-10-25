// SPDX-License-Identifier: BSD-3-Clause
//! Bounded current-frame feature and optimized-state diagnostics on the graph owner.
use crate::{frames::multi_frame::MultiFrame, runtime::vi_interface::State};
use firefly_error::Result;
use firefly_pubsub::{
    node::{IpcNode, create_node},
    viz::{OKVIS_KEYPOINT_ENTITIES, POINTS_MAX, VizMessage, VizPublisher, kind},
};
use nalgebra::Vector2;

pub(super) struct Diagnostics {
    publisher: VizPublisher,
    _node: IpcNode,
    image_delay: f64,
}
impl Diagnostics {
    pub fn new(image_delay: f64) -> Result<Self> {
        let node = create_node()?;
        Ok(Self {
            publisher: VizPublisher::new(&node)?,
            _node: node,
            image_delay,
        })
    }
    pub fn publish(&mut self, frame: &MultiFrame, state: &State) {
        let stamp = frame.timestamp().as_nanosecond() as f64 * 1e-9 + self.image_delay;
        for camera in 0..frame.num_frames().min(OKVIS_KEYPOINT_ENTITIES.len()) {
            if let Some(message) = keypoints(frame, camera, stamp) {
                let _ = self.publisher.publish(message);
            }
        }
        let pose = state.t_ws.matrix().transpose();
        let _ = self.publisher.publish(VizMessage::matrix(
            state.timestamp.as_nanosecond() as f64 * 1e-9,
            "okvis/debug/pose_imu",
            4,
            4,
            pose.as_slice(),
        ));
    }
}

fn keypoints(frame: &MultiFrame, camera: usize, stamp: f64) -> Option<VizMessage> {
    let image = frame.image(camera)?;
    let mut message = VizMessage::base(kind::KEYPOINTS_2D, stamp, OKVIS_KEYPOINT_ENTITIES[camera]);
    message.color = [64, 255, 128];
    let total = frame.num_keypoints(camera);
    let retained = total.min(POINTS_MAX);
    message.scalars[..3].copy_from_slice(&[image.cols() as f64, image.rows() as f64, total as f64]);
    message.scalar_count = 3;
    message.point_count = retained as u32;
    for index in 0..retained {
        let keypoint = index * total / retained;
        let mut pixel = Vector2::zeros();
        frame.get_keypoint(camera, keypoint, &mut pixel);
        message.points[index] = [pixel.x, pixel.y, frame.landmark_id(camera, keypoint) as f64];
    }
    Some(message)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{application::live::config::Config, frames::frame::KeyPoint};
    use jiff::Timestamp;
    use purecv::core::{DynamicMatrix, types::Point2f};
    use std::sync::Arc;

    #[test]
    fn feature_diagnostics_bound_sampling_and_preserve_capture_geometry_and_ids() {
        let parameters = Config::default().parameters().unwrap();
        let mut frame = MultiFrame::new(&parameters.n_camera_system, Timestamp::UNIX_EPOCH, 1);
        let image = Arc::new(DynamicMatrix::new_u8(240, 320, 1, vec![0; 240 * 320]).unwrap());
        frame.set_image(0, image);
        let total = POINTS_MAX * 2 + 1;
        let points = (0..total)
            .map(|i| KeyPoint {
                pt: Point2f::new((i % 320) as f32, (i / 320) as f32),
                ..KeyPoint::default()
            })
            .collect::<Vec<_>>();
        frame.reset_keypoints(0, &points);
        for i in 0..total {
            frame.set_landmark_id(0, i, i as u64 + 1);
        }
        let message = keypoints(&frame, 0, 0.125).unwrap();
        assert_eq!(message.entity(), "okvis/debug/keypoints/left");
        assert_eq!(message.timestamp, 0.125);
        assert_eq!(message.point_count as usize, POINTS_MAX);
        assert_eq!(&message.scalars[..3], &[320., 240., total as f64]);
        for i in 0..POINTS_MAX {
            let selected = i * total / POINTS_MAX;
            assert_eq!(
                message.points[i],
                [
                    (selected % 320) as f64,
                    (selected / 320) as f64,
                    selected as f64 + 1.
                ]
            );
        }
        assert!(keypoints(&frame, 1, 0.125).is_none());
        frame.reset_keypoints(0, &[]);
        let empty = keypoints(&frame, 0, 0.25).unwrap();
        assert_eq!(empty.point_count, 0);
        assert_eq!(&empty.scalars[..3], &[320., 240., 0.]);
    }
}
