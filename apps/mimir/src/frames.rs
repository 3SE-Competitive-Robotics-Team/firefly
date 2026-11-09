//! Display-only frame relations. No truth-derived value leaves the recorder.
use std::collections::BTreeMap;

use firefly_base::FrameId;
use firefly_error::Result;
use firefly_pubsub::{navigation::NavigationMessage, odom::OdomMessage};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{invalid, schema};

const WORLD: FrameId = FrameId(1024);
const MATCH_CAPACITY: usize = 256;

#[derive(Debug)]
pub(crate) struct Anchor {
    pub stamp: f64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

#[derive(Default)]
pub(crate) struct Frames {
    navigation: Option<NavigationMessage>,
    truth: BTreeMap<i64, OdomMessage>,
    map: BTreeMap<i64, OdomMessage>,
    closed: bool,
    saw_truth: bool,
    anchored: bool,
}

impl Frames {
    /// Store atomic map←odom; reject a session reset instead of joining incompatible origins.
    pub fn navigation(&mut self, msg: NavigationMessage) -> Result<(bool, Option<Anchor>)> {
        schema::timestamp(msg.odom.timestamp)?;
        let alignment = msg.alignment()?;
        let pose = msg.odom.body_pose(FrameId::MAP)?;
        let changed = if let Some(previous) = self.navigation {
            if msg.odom.timestamp < previous.odom.timestamp
                || msg.revision < previous.revision
                || msg.external_map != previous.external_map
            {
                return Err(invalid("navigation session changed; start a new recording"));
            }
            let changed =
                msg.translation != previous.translation || msg.rotation != previous.rotation;
            if changed && msg.revision == previous.revision {
                return Err(invalid("navigation alignment changed without a revision"));
            }
            changed || msg.revision != previous.revision
        } else {
            true
        };
        self.navigation = Some(msg);
        if self.closed {
            return Ok((changed, None));
        }
        let local = alignment.inverse().compose(&pose)?;
        // Restrict anchoring to stationary samples near the local origin. Observed motion
        // closes this window; without a session-start marker a late return is indistinguishable.
        if msg.external_map != 0
            || !stationary(&msg.odom)
            || local.isometry().translation.vector.norm() > 0.1
        {
            self.close();
            return Ok((changed, None));
        }
        insert(&mut self.map, msg.odom)?;
        Ok((
            changed,
            self.match_at(schema::timestamp(msg.odom.timestamp)?)?,
        ))
    }

    pub fn truth(&mut self, msg: OdomMessage) -> Result<Option<Anchor>> {
        self.saw_truth |= msg.is_initialized;
        if self.closed || !msg.is_initialized || !stationary(&msg) {
            return Ok(None);
        }
        msg.body_pose(WORLD)?;
        insert(&mut self.truth, msg)?;
        self.match_at(schema::timestamp(msg.timestamp)?)
    }

    fn match_at(&mut self, time: i64) -> Result<Option<Anchor>> {
        let (Some(truth), Some(map)) = (self.truth.get(&time), self.map.get(&time)) else {
            return Ok(None);
        };
        // Both histories use measurement nanoseconds, matching VIO's clock precision.
        // The simulator's accumulated f64 seconds need not survive the ns round-trip.
        let truth_pose = truth.body_pose(WORLD)?;
        let map_pose = map.body_pose(FrameId::MAP)?;
        let yaw = map_pose.isometry().rotation.euler_angles().2
            - truth_pose.isometry().rotation.euler_angles().2;
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
        let translation = map_pose.isometry().translation.vector
            - rotation * truth_pose.isometry().translation.vector;
        let q = rotation.quaternion();
        let anchor = Anchor {
            stamp: map.timestamp,
            translation: translation.into(),
            rotation: [q.i, q.j, q.k, q.w],
        };
        self.close();
        self.anchored = true;
        Ok(Some(anchor))
    }

    pub fn truth_unaligned(&self) -> bool {
        self.saw_truth && !self.anchored
    }

    fn close(&mut self) {
        self.closed = true;
        self.truth.clear();
        self.map.clear();
    }
}

fn stationary(msg: &OdomMessage) -> bool {
    Vector3::new(msg.velocity_x, msg.velocity_y, msg.velocity_z).norm() <= 0.1
}

fn insert(history: &mut BTreeMap<i64, OdomMessage>, msg: OdomMessage) -> Result<()> {
    history.insert(schema::timestamp(msg.timestamp)?, msg);
    if history.len() > MATCH_CAPACITY {
        history.pop_first();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nav(stamp: f64) -> NavigationMessage {
        NavigationMessage {
            odom: OdomMessage {
                timestamp: stamp,
                is_initialized: true,
                ..Default::default()
            },
            rotation: [0., 0., 0., 1.],
            ..Default::default()
        }
    }

    #[test]
    fn startup_anchor_matches_nanoseconds_after_vio_time_conversion() {
        // Simulator time accumulates physics steps; VIO round-trips through integer ns.
        let physics_time: f64 = (0..350).map(|_| 0.001).sum();
        let navigation_time = schema::timestamp(physics_time).unwrap() as f64 * 1e-9;
        assert_ne!(physics_time, navigation_time);
        assert_eq!(schema::timestamp(physics_time).unwrap(), 350_000_000);
        for truth_first in [true, false] {
            let mut frames = Frames::default();
            let truth = OdomMessage {
                position_x: -13.,
                position_z: 0.4,
                ..nav(physics_time).odom
            };
            let anchor = if truth_first {
                assert!(frames.truth(truth).unwrap().is_none());
                // A genuinely different nanosecond must still not match.
                assert!(
                    frames
                        .navigation(nav(navigation_time - 1e-9))
                        .unwrap()
                        .1
                        .is_none()
                );
                frames.navigation(nav(navigation_time)).unwrap().1
            } else {
                assert!(frames.navigation(nav(navigation_time)).unwrap().1.is_none());
                assert!(
                    frames
                        .truth(OdomMessage {
                            timestamp: physics_time - 1e-9,
                            ..truth
                        })
                        .unwrap()
                        .is_none()
                );
                frames.truth(truth).unwrap()
            };
            assert_eq!(anchor.unwrap().translation, [13., 0., -0.4]);
            assert!(!frames.truth_unaligned());
        }
    }

    #[test]
    fn anchor_requires_exact_time_and_preserves_later_drift() {
        for truth_first in [true, false] {
            let mut frames = Frames::default();
            let truth = OdomMessage {
                position_x: -13.,
                position_z: 0.4,
                ..nav(1.).odom
            };
            if truth_first {
                assert!(frames.truth(truth).unwrap().is_none());
                assert!(frames.navigation(nav(1.01)).unwrap().1.is_none());
                // Use a later matching time to keep navigation chronological.
                frames
                    .truth(OdomMessage {
                        timestamp: 1.02,
                        ..truth
                    })
                    .unwrap();
            } else {
                assert!(frames.navigation(nav(1.)).unwrap().1.is_none());
                assert!(
                    frames
                        .truth(OdomMessage {
                            timestamp: 0.99,
                            ..truth
                        })
                        .unwrap()
                        .is_none()
                );
            }
            let anchor = if truth_first {
                frames.navigation(nav(1.02)).unwrap().1.unwrap()
            } else {
                frames.truth(truth).unwrap().unwrap()
            };
            assert_eq!(anchor.translation, [13., 0., -0.4]);
            assert_eq!(anchor.rotation, [0., 0., 0., 1.]);
            let mut drifted = nav(2.);
            drifted.odom.position_z = 0.026;
            assert!(frames.navigation(drifted).unwrap().1.is_none());
            assert!(
                frames
                    .truth(OdomMessage {
                        timestamp: 2.,
                        ..truth
                    })
                    .unwrap()
                    .is_none()
            );
        }
    }

    #[test]
    fn alignment_is_yaw_only_and_handles_a_rotated_map() {
        let mut frames = Frames::default();
        let mut message = nav(1.);
        message.translation = [2., 3., 0.];
        message.rotation = [0., 0., (0.5_f64).sin(), (0.5_f64).cos()];
        message.odom.position_x = 2.;
        message.odom.position_y = 3.;
        let rotation = UnitQuaternion::from_euler_angles(0.02, -0.03, 1.);
        let q = rotation.quaternion();
        message.odom.quat_x = q.i;
        message.odom.quat_y = q.j;
        message.odom.quat_z = q.k;
        message.odom.quat_w = q.w;
        frames.navigation(message).unwrap();
        let truth = OdomMessage {
            position_x: -13.,
            position_z: 0.4,
            ..nav(1.).odom
        };
        let anchor = frames.truth(truth).unwrap().unwrap();
        assert_eq!(anchor.rotation[..2], [0., 0.]);
        let actual = Vector3::from(anchor.translation)
            + UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.) * Vector3::new(-13., 0., 0.4);
        assert!((actual - Vector3::new(2., 3., 0.)).norm() < 1e-12);
    }

    #[test]
    fn no_anchor_for_late_recording_or_external_map_and_no_session_reset() {
        for external in [false, true] {
            let mut frames = Frames::default();
            let mut message = nav(1.);
            if external {
                message.external_map = 1;
            } else {
                message.odom.position_z = 1.;
            }
            frames.navigation(message).unwrap();
            assert!(frames.truth(nav(1.).odom).unwrap().is_none());
            assert!(frames.closed);
        }
        let mut frames = Frames::default();
        frames.navigation(nav(1.)).unwrap();
        assert!(frames.navigation(nav(0.5)).is_err());
        let mut revised = nav(2.);
        revised.translation[0] = 1.;
        assert!(frames.navigation(revised).is_err());
        revised.revision = 1;
        assert!(frames.navigation(revised).unwrap().0);
        assert!(frames.navigation(nav(3.)).is_err());
    }

    #[test]
    fn matching_history_is_bounded() {
        let mut frames = Frames::default();
        for i in 0..1024 {
            frames.truth(nav(f64::from(i)).odom).unwrap();
        }
        assert_eq!(frames.truth.len(), MATCH_CAPACITY);
    }
}
