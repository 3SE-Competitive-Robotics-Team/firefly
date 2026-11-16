//! 图像时刻里程计配对与静止己方机库跟踪；世界真值不进入此链。
use firefly_landing::Detection;
#[cfg(test)]
use firefly_landing::geometry::refine_corners;
use firefly_landing::geometry::{Geometry, corner_candidates};
use firefly_pubsub::{landing::LandingTargetMessage, odom::OdomMessage};
use nalgebra::{Isometry3, Translation3, Vector3};
use std::collections::VecDeque;

pub fn pose_at(history: &VecDeque<OdomMessage>, stamp: f64) -> Option<Isometry3<f64>> {
    if !stamp.is_finite() {
        return None;
    }
    let before = history.iter().rev().find(|p| p.timestamp <= stamp)?;
    let after = history.iter().find(|p| p.timestamp >= stamp)?;
    if after.timestamp - before.timestamp > 0.05 || !before.is_initialized || !after.is_initialized
    {
        return None;
    }
    let fraction = if after.timestamp == before.timestamp {
        0.
    } else {
        (stamp - before.timestamp) / (after.timestamp - before.timestamp)
    };
    let left = before.body_pose(firefly_base::FrameId::ODOM).ok()?;
    let right = after.body_pose(firefly_base::FrameId::ODOM).ok()?;
    Some(Isometry3::from_parts(
        Translation3::from(
            left.isometry()
                .translation
                .vector
                .lerp(&right.isometry().translation.vector, fraction),
        ),
        left.isometry()
            .rotation
            .slerp(&right.isometry().rotation, fraction),
    ))
}

#[derive(Default)]
pub struct Tracker {
    session: u64,
    stamp: Option<f64>,
    target: Option<Vector3<f64>>,
    accepted: usize,
    confirmed: bool,
    last_accepted: Option<f64>,
    last_confidence: f64,
}
impl Tracker {
    /// 每幅图发布一条完整有效/无效观测，采集时间保持原值。
    #[allow(clippy::too_many_arguments)]
    pub fn process(
        &mut self,
        session: u64,
        team: u32,
        stamp: f64,
        image: &[u8],
        detections: &[Detection],
        pose: Option<Isometry3<f64>>,
        geometry: &Geometry,
        mounting: &Isometry3<f64>,
        radius: f64,
        contact_offset: f64,
    ) -> LandingTargetMessage {
        if session != self.session {
            *self = Self {
                session,
                ..Self::default()
            };
        }
        let mut message = LandingTargetMessage {
            timestamp: stamp,
            session,
            team,
            ..LandingTargetMessage::default()
        };
        if session == 0
            || !stamp.is_finite()
            || self.stamp.is_some_and(|previous| stamp <= previous)
        {
            return message;
        }
        if self
            .last_accepted
            .is_some_and(|previous| stamp - previous > 2.)
        {
            self.accepted = 0;
            self.target = None;
            self.confirmed = false;
        }
        self.stamp = Some(stamp);
        let Some(body) = pose else {
            log::debug!(target:"landing::gates","stamp={stamp:.3} no capture-time odometry");
            self.accepted = 0;
            return message;
        };
        let camera = body * mounting;
        let mut best: Option<(Vector3<f64>, f64, f64)> = None;
        let mut gates = [0; 3];
        // Acquired H geometry remains observable when the growing board leaves
        // the detector's training scale. Tracking uses the same metric gates.
        let tracked = self.target.map(|_| Detection {
            bounds: [
                0.,
                0.,
                firefly_pubsub::camera::DOWN_WIDTH as f32,
                firefly_pubsub::camera::DOWN_HEIGHT as f32,
            ],
            confidence: self.last_confidence as f32,
        });
        for detection in detections.iter().copied().take(8).chain(tracked) {
            let Ok(candidates) = corner_candidates(
                image,
                firefly_pubsub::camera::DOWN_WIDTH,
                firefly_pubsub::camera::DOWN_HEIGHT,
                detection,
            ) else {
                continue;
            };
            for corners in candidates {
                gates[0] += 1;
                let Some((mut target, reprojection)) = geometry.horizontal_target(corners, &camera)
                else {
                    continue;
                };
                gates[1] += 1;
                target.z += contact_offset;
                if target.xy().norm() > radius
                    || !(0.03..=3.).contains(&(body.translation.vector.z - target.z))
                {
                    continue;
                }
                let reference = self.target.unwrap_or_else(Vector3::zeros);
                if self.target.is_some() && (target - reference).norm() > 0.15 {
                    continue;
                }
                gates[2] += 1;
                if best.as_ref().is_none_or(|(previous, _, _)| {
                    (target - reference).norm() < (previous - reference).norm()
                }) {
                    best = Some((target, reprojection, f64::from(detection.confidence)));
                }
            }
        }
        log::debug!(target:"landing::gates","stamp={stamp:.3} detections={} corners={} plane={} accepted={}",detections.len(),gates[0],gates[1],gates[2]);
        let Some((target, error, confidence)) = best else {
            self.accepted = 0;
            return message;
        };
        let target = self
            .target
            .map_or(target, |previous| previous.lerp(&target, 0.35));
        self.target = Some(target);
        self.accepted += 1;
        self.last_accepted = Some(stamp);
        self.last_confidence = confidence;
        message.position = target.into();
        message.reprojection_px = error;
        message.confidence = confidence;
        self.confirmed |= self.accepted >= 3;
        message.valid = u32::from(self.confirmed);
        message
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn acquired_geometry_tracks_without_detector_and_session_reset_requires_detection() {
        use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
        let mut image = vec![0; DOWN_WIDTH * DOWN_HEIGHT];
        for y in 119..168 {
            for x in 155..196 {
                if !(165..186).contains(&x) || (138..149).contains(&y) {
                    image[y * DOWN_WIDTH + x] = 255;
                }
            }
        }
        let geometry = Geometry::default();
        let mounting = Isometry3::from_parts(
            Translation3::new(-0.03, 0., 0.),
            nalgebra::UnitQuaternion::from_matrix(
                &(firefly_base::rig::down_axes_in_body() * firefly_base::rig::CV_FROM_RENDER),
            ),
        );
        let pose = Some(Isometry3::translation(0., 0., 0.8));
        let detection = Detection {
            bounds: [145., 109., 206., 178.],
            confidence: 0.9,
        };
        let mut tracker = Tracker::default();
        let mut process = |session, stamp, detections: &[Detection]| {
            tracker.process(
                session, 0, stamp, &image, detections, pose, &geometry, &mounting, 0.8, 0.02,
            )
        };
        for i in 0..3 {
            assert_eq!(
                process(7, f64::from(i) * 0.1, &[detection]).valid,
                u32::from(i == 2)
            );
        }
        let message = process(7, 0.3, &[]);
        assert_eq!(message.valid, 1);
        assert!((message.position[0] + 0.03).abs() < 0.01);
        assert_eq!(process(7, 0.3, &[]).valid, 0);
        assert_eq!(process(7, 0.4, &[]).valid, 1);
        assert_eq!(process(8, 0.4, &[]).valid, 0);
        assert_eq!(process(8, 0.5, &[detection]).valid, 0);
        assert_eq!(process(8, 3., &[]).valid, 0);
    }

    #[test]
    #[ignore = "requires FIREFLY_LANDING_FRAMES containing recorded CIF 14.png/15.png/16.png"]
    fn recorded_glossy_h_keeps_its_four_corners_during_approach() {
        use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
        let directory = std::path::PathBuf::from(std::env::var("FIREFLY_LANDING_FRAMES").unwrap());
        for name in ["14.png", "15.png", "16.png"] {
            let image = image::open(directory.join(name)).unwrap().to_luma8();
            let corners = refine_corners(
                image.as_raw(),
                DOWN_WIDTH,
                DOWN_HEIGHT,
                Detection {
                    bounds: [0., 0., DOWN_WIDTH as f32, DOWN_HEIGHT as f32],
                    confidence: 1.,
                },
            )
            .unwrap();
            assert!(corners.is_some(), "white H lost in {name}");
        }
    }

    #[test]
    fn interpolation_uses_capture_time_and_rejects_extrapolation_or_gaps() {
        let history = VecDeque::from([
            OdomMessage {
                timestamp: 1.,
                is_initialized: true,
                ..OdomMessage::default()
            },
            OdomMessage {
                timestamp: 1.02,
                position_x: 0.2,
                is_initialized: true,
                ..OdomMessage::default()
            },
        ]);
        assert!((pose_at(&history, 1.01).unwrap().translation.x - 0.1).abs() < 1e-10);
        assert!(pose_at(&history, 0.99).is_none());
        assert!(pose_at(&history, 1.03).is_none());
    }
}
