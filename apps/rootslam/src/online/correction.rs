//! Apply a committed backend snapshot only to future navigation publications.
use firefly_base::{FrameId, RigidTransform};
use firefly_error::Result;
use firefly_pubsub::{navigation::NavigationMessage, odom::OdomMessage};

use super::error;
use crate::system::navigation::Correction;

pub(super) struct Navigation {
    session: u64,
    revision: u64,
    latest: Option<Correction>,
    pending: Option<Correction>,
    alignment: RigidTransform,
}

impl Navigation {
    pub fn new(session: u64) -> Result<Self> {
        Ok(Self {
            session,
            revision: 0,
            latest: None,
            pending: None,
            alignment: RigidTransform::from_parts(
                FrameId::MAP,
                FrameId::ODOM,
                [0.; 3],
                [0., 0., 0., 1.],
            )?,
        })
    }

    pub fn observe(&mut self, correction: Correction) -> Result<()> {
        if correction.session != self.session {
            return Err(error("map correction belongs to another estimator session"));
        }
        if correction.anchor_ns < 0 || correction.source_ns < correction.anchor_ns {
            return Err(error("map correction has invalid measurement times"));
        }
        if correction.revision == 0 {
            return Ok(());
        }
        if let Some(old) = self.latest {
            if correction.revision < old.revision {
                return Ok(());
            }
            if correction.revision == old.revision {
                return if correction == old {
                    Ok(())
                } else {
                    Err(error("map correction changed without a new revision"))
                };
            }
            if correction.source_ns < old.source_ns
                || correction.source_revision < old.source_revision
                || correction.anchor_ns != old.anchor_ns
            {
                return Err(error("map correction source regressed or changed origin"));
            }
        }
        let q = correction.map_from_odom.rotation.quaternion();
        RigidTransform::from_parts(
            FrameId::MAP,
            FrameId::ODOM,
            correction.map_from_odom.translation.vector.into(),
            [q.i, q.j, q.k, q.w],
        )?;
        self.pending = Some(correction);
        self.latest = Some(correction);
        Ok(())
    }

    pub fn output(&mut self, odom: &OdomMessage, time_ns: i128) -> Result<NavigationMessage> {
        if let Some(correction) = self.pending
            && i128::from(correction.source_ns) <= time_ns
        {
            let q = correction.map_from_odom.rotation.quaternion();
            self.alignment = RigidTransform::from_parts(
                FrameId::MAP,
                FrameId::ODOM,
                correction.map_from_odom.translation.vector.into(),
                [q.i, q.j, q.k, q.w],
            )?;
            self.revision = correction.revision;
            self.pending = None;
            log::info!(
                "RootSLAM navigation prepared: session={} revision={} backend_revision={} anchor_ns={} source_ns={} publish_ns={}",
                self.session,
                self.revision,
                correction.source_revision,
                correction.anchor_ns,
                correction.source_ns,
                time_ns
            );
        }
        Ok(NavigationMessage::new(
            odom.in_map(&self.alignment)?,
            &self.alignment,
            self.revision,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};

    fn correction() -> Correction {
        Correction {
            session: 7,
            revision: 1,
            source_revision: 10,
            anchor_ns: 1,
            source_ns: 2_000_000_000,
            map_from_odom: Isometry3::from_parts(
                Translation3::new(10., 2., 0.5),
                UnitQuaternion::from_euler_angles(0., 0., std::f64::consts::FRAC_PI_2),
            ),
        }
    }

    #[test]
    fn accepted_map_changes_navigation_only_at_a_covered_imu_time() {
        let mut nav = Navigation::new(7).unwrap();
        let odom = OdomMessage {
            timestamp: 2.,
            position_x: 1.,
            velocity_x: 3.,
            is_initialized: true,
            ..Default::default()
        };
        nav.observe(correction()).unwrap();
        assert_eq!(nav.output(&odom, 1_000_000_000).unwrap().revision, 0);
        let out = nav.output(&odom, 2_000_000_000).unwrap();
        assert_eq!(out.revision, 1);
        assert!((out.odom.position_x - 10.).abs() < 1e-12);
        assert!((out.odom.position_y - 3.).abs() < 1e-12);
        assert!((out.odom.velocity_y - 3.).abs() < 1e-12);
        assert_eq!(out.odom.timestamp, odom.timestamp);
        assert_eq!(odom.position_x, 1.);
        assert_eq!(odom.quat_w, 1.);
        nav.observe(Correction {
            revision: 0,
            ..correction()
        })
        .unwrap();
        assert_eq!(
            nav.output(&odom, 3_000_000_000).unwrap().translation,
            out.translation
        );
        assert!(
            nav.observe(Correction {
                session: 8,
                ..correction()
            })
            .is_err()
        );
        assert!(
            nav.observe(Correction {
                revision: 2,
                source_revision: 9,
                ..correction()
            })
            .is_err()
        );
    }

    #[test]
    fn pending_and_applied_revisions_have_immutable_monotonic_sources() {
        let mut nav = Navigation::new(7).unwrap();
        nav.observe(correction()).unwrap();
        nav.observe(correction()).unwrap();
        for update in [
            Correction {
                source_ns: 1_000_000_000,
                revision: 2,
                ..correction()
            },
            Correction {
                source_revision: 9,
                revision: 2,
                ..correction()
            },
            Correction {
                anchor_ns: 0,
                revision: 2,
                ..correction()
            },
            Correction {
                map_from_odom: Isometry3::identity(),
                ..correction()
            },
        ] {
            assert!(nav.observe(update).is_err());
        }
        nav.output(&OdomMessage::default(), 2_000_000_000).unwrap();
        assert!(
            nav.observe(Correction {
                source_revision: 11,
                ..correction()
            })
            .is_err()
        );
        nav.observe(Correction {
            revision: 2,
            source_revision: 11,
            ..correction()
        })
        .unwrap();
        assert_eq!(
            nav.output(&OdomMessage::default(), 2_000_000_000)
                .unwrap()
                .revision,
            2
        );
    }
}
