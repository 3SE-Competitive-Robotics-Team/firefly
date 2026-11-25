//! 图像时刻里程计配对与静止己方机库跟踪；世界真值不进入此链。
use firefly_landing::Detection;
#[cfg(test)]
use firefly_landing::geometry::refine_corners;
use firefly_landing::geometry::{Geometry, HOutline, MarkerPose, outline_candidates};
use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
use firefly_pubsub::landing::LandingTargetMessage;
use nalgebra::{Isometry3, Vector3};

/// 本次激活周期持有的标定与几何门限。
pub(crate) struct Settings {
    pub geometry: Geometry,
    pub camera_in_body: Isometry3<f64>,
    pub acquisition_radius_m: f64,
    pub contact_offset_m: f64,
}

/// 同一幅图像的身份、测量时刻、检测和插值位姿。
#[derive(Clone, Copy)]
pub(crate) struct Observation<'a> {
    pub session: u64,
    pub team: u32,
    pub timestamp: f64,
    pub image: &'a [u8],
    pub detections: &'a [Detection],
    pub pose: Option<Isometry3<f64>>,
}

#[derive(Clone, Copy)]
struct Accepted {
    target: Vector3<f64>,
    timestamp: f64,
    confidence: f64,
}

#[derive(Default)]
enum Acquisition {
    #[default]
    Searching,
    Confirming {
        accepted: Accepted,
        consecutive: u8,
    },
    Tracking(Accepted),
}

impl Acquisition {
    fn accepted(&self) -> Option<&Accepted> {
        match self {
            Self::Searching => None,
            Self::Confirming { accepted, .. } | Self::Tracking(accepted) => Some(accepted),
        }
    }

    fn missed(&mut self) {
        if let Self::Confirming { consecutive, .. } = self {
            *consecutive = 0;
        }
    }

    fn accept(&mut self, accepted: Accepted) -> bool {
        let consecutive = match self {
            Self::Searching => 1,
            Self::Confirming { consecutive, .. } => *consecutive + 1,
            Self::Tracking(_) => 3,
        };
        let confirmed = consecutive >= 3;
        *self = if confirmed {
            Self::Tracking(accepted)
        } else {
            Self::Confirming {
                accepted,
                consecutive,
            }
        };
        confirmed
    }
}

struct Candidate {
    outline: HOutline,
    target: Vector3<f64>,
    reprojection_px: f64,
    confidence: f64,
}

pub(crate) struct Tracker {
    settings: Settings,
    session: u64,
    stamp: Option<f64>,
    acquisition: Acquisition,
}

impl Tracker {
    pub fn new(settings: Settings) -> Self {
        Self {
            settings,
            session: 0,
            stamp: None,
            acquisition: Acquisition::Searching,
        }
    }

    /// 每幅图发布一条完整有效/无效观测，采集时间保持原值。
    pub fn process(&mut self, observation: Observation<'_>) -> LandingTargetMessage {
        let Observation {
            session,
            team,
            timestamp: stamp,
            pose,
            ..
        } = observation;
        if session != self.session {
            self.session = session;
            self.stamp = None;
            self.acquisition = Acquisition::Searching;
        }
        let mut message = LandingTargetMessage {
            timestamp: stamp,
            session,
            team,
            ..Default::default()
        };
        if !stamp.is_finite() || self.stamp.is_some_and(|previous| stamp <= previous) {
            return message;
        }
        if self
            .acquisition
            .accepted()
            .is_some_and(|previous| stamp - previous.timestamp > 2.)
        {
            self.acquisition = Acquisition::Searching;
        }
        self.stamp = Some(stamp);
        let Some(body) = pose.filter(|_| session != 0) else {
            log::debug!(target: "landing::gates", "stamp={stamp:.3} no capture-time odometry");
            let full = Detection {
                bounds: [0., 0., DOWN_WIDTH as f32, DOWN_HEIGHT as f32],
                confidence: 1.,
            };
            if let Ok(outlines) =
                outline_candidates(observation.image, DOWN_WIDTH, DOWN_HEIGHT, full)
                && let Some(v) = outlines
                    .iter()
                    .filter_map(|o| self.settings.geometry.marker_pose(o))
                    .min_by(|a, b| a.reprojection_px.total_cmp(&b.reprojection_px))
            {
                self.set_raw_pose(&mut message, &v);
            }
            self.acquisition.missed();
            return message;
        };
        let Some(candidate) = self.select_candidate(observation, &body) else {
            self.acquisition.missed();
            return message;
        };
        let target = self
            .acquisition
            .accepted()
            .map_or(candidate.target, |value| {
                value.target.lerp(&candidate.target, 0.35)
            });
        let confirmed = self.acquisition.accept(Accepted {
            target,
            timestamp: stamp,
            confidence: candidate.confidence,
        });
        message.position = target.into();
        message.reprojection_px = candidate.reprojection_px;
        message.confidence = candidate.confidence;
        message.valid = u32::from(confirmed);
        if let Some(visual) = self.settings.geometry.marker_pose(&candidate.outline) {
            self.set_raw_pose(&mut message, &visual);
        }
        message
    }

    fn set_raw_pose(&self, message: &mut LandingTargetMessage, visual: &MarkerPose) {
        let pose = self.settings.camera_in_body * visual.camera_from_h;
        message.body_from_h_translation = pose.translation.vector.into();
        message.body_from_h_xyzw = pose.rotation.coords.into();
        message.h_pose_reprojection_px = visual.reprojection_px;
        message.h_pose_valid = 1;
    }

    fn select_candidate(
        &self,
        observation: Observation<'_>,
        body: &Isometry3<f64>,
    ) -> Option<Candidate> {
        let Observation {
            image,
            detections,
            timestamp: stamp,
            ..
        } = observation;
        let camera = body * self.settings.camera_in_body;
        let previous = self.acquisition.accepted().copied();
        let reference = previous.map_or_else(Vector3::zeros, |value| value.target);
        let mut best: Option<Candidate> = None;
        let mut gates = [0; 3];
        // 已获得的 H 几何可在全图延续身份，检测与跟踪共用米制门控。
        let tracked = previous.map(|value| Detection {
            bounds: [
                0.,
                0.,
                firefly_pubsub::camera::DOWN_WIDTH as f32,
                firefly_pubsub::camera::DOWN_HEIGHT as f32,
            ],
            confidence: value.confidence as f32,
        });
        for detection in detections.iter().copied().take(8).chain(tracked) {
            let Ok(candidates) = outline_candidates(
                image,
                firefly_pubsub::camera::DOWN_WIDTH,
                firefly_pubsub::camera::DOWN_HEIGHT,
                detection,
            ) else {
                continue;
            };
            for outline in candidates {
                gates[0] += 1;
                let Some(plane) = self
                    .settings
                    .geometry
                    .horizontal_target(outline.rectangle, &camera)
                else {
                    continue;
                };
                gates[1] += 1;
                let mut target = plane.center;
                target.z += self.settings.contact_offset_m;
                if target.xy().norm() > self.settings.acquisition_radius_m
                    || !(0.03..=3.).contains(&(body.translation.vector.z - target.z))
                    || previous.is_some() && (target - reference).norm() > 0.15
                {
                    continue;
                }
                gates[2] += 1;
                if best
                    .as_ref()
                    .is_none_or(|old| (target - reference).norm() < (old.target - reference).norm())
                {
                    best = Some(Candidate {
                        outline,
                        target,
                        reprojection_px: plane.reprojection_px,
                        confidence: f64::from(detection.confidence),
                    });
                }
            }
        }
        log::debug!(target:"landing::gates","stamp={stamp:.3} detections={} corners={} plane={} accepted={}",detections.len(),gates[0],gates[1],gates[2]);
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_odometry::PoseBracket;
    use firefly_pubsub::odom::OdomMessage;
    use std::collections::VecDeque;

    #[test]
    fn acquisition_requires_consecutive_confirmation_and_retains_tracked_identity() {
        let accepted = Accepted {
            target: Vector3::zeros(),
            timestamp: 1.,
            confidence: 0.9,
        };
        let mut acquisition = Acquisition::Searching;
        assert!(!acquisition.accept(accepted));
        assert!(!acquisition.accept(accepted));
        acquisition.missed();
        assert!(!acquisition.accept(accepted));
        assert!(!acquisition.accept(accepted));
        assert!(acquisition.accept(accepted));
        acquisition.missed();
        assert!(matches!(acquisition, Acquisition::Tracking(_)));
        assert!(acquisition.accept(accepted));
    }

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
        let mounting =
            *firefly_base::rig::body_from_optical(firefly_base::FrameId::DOWN_CAMERA).isometry();
        let pose = Some(Isometry3::translation(0., 0., 0.8));
        let detection = Detection {
            bounds: [145., 109., 206., 178.],
            confidence: 0.9,
        };
        let mut tracker = Tracker::new(Settings {
            geometry,
            camera_in_body: mounting,
            acquisition_radius_m: 0.8,
            contact_offset_m: 0.02,
        });
        let mut process = |session, stamp, detections: &[Detection]| {
            tracker.process(Observation {
                session,
                team: 0,
                timestamp: stamp,
                image: &image,
                detections,
                pose,
            })
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
    fn recorded_glossy_h_keeps_its_four_corners_during_approach() {
        use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
        let directory =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/reference/flight3");
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
    fn raw_pose_survives_without_vio_while_contact_target_waits() {
        let image = image::open(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/reference/flight3/15.png"
        ))
        .unwrap()
        .to_luma8();
        let mut tracker = Tracker::new(crate::config::Config::default().tracking());
        let msg = tracker.process(Observation {
            timestamp: 1.,
            session: 0,
            team: 0,
            image: image.as_raw(),
            detections: &[],
            pose: None,
        });
        assert_eq!((msg.h_pose_valid, msg.valid), (1, 0));
        assert!(msg.body_from_h_translation[2] < 0.);
        assert!((msg.body_from_h_xyzw.iter().map(|x| x * x).sum::<f64>() - 1.).abs() < 1e-8);
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
        assert!(
            (PoseBracket::find(&history, 1.01, 0.05)
                .and_then(|pair| pair.pose())
                .unwrap()
                .translation
                .x
                - 0.1)
                .abs()
                < 1e-10
        );
        assert!(PoseBracket::find(&history, 0.99, 0.05).is_none());
        assert!(PoseBracket::find(&history, 1.03, 0.05).is_none());
    }
}
