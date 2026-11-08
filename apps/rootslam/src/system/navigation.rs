//! Navigation gauge for a pose graph whose active VIO window stays fixed.
use std::sync::{Arc, RwLock};

use firefly_error::Result;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use super::{MapWorkerState, error};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Correction {
    pub session: u64,
    pub revision: u64,
    pub source_revision: u64,
    pub anchor_ns: i64,
    pub source_ns: i64,
    pub map_from_odom: Isometry3<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct MappingUpdate {
    pub session: u64,
    pub detected: usize,
    pub verified: usize,
    pub accepted: usize,
    pub rejected: usize,
    pub correction: Option<Correction>,
}

/// One atomic committed snapshot; no optimizer work runs under this lock.
#[derive(Clone)]
pub struct MappingUpdates(Arc<RwLock<MappingUpdate>>);

impl Default for MappingUpdates {
    fn default() -> Self {
        Self(Arc::new(RwLock::new(MappingUpdate {
            session: (std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64)
                .max(1),
            verified: 0,
            detected: 0,
            accepted: 0,
            rejected: 0,
            correction: None,
        })))
    }
}

impl MappingUpdates {
    /// # Errors
    /// Returns a poisoned snapshot lock.
    pub fn snapshot(&self) -> Result<MappingUpdate> {
        self.0
            .read()
            .map(|value| *value)
            .map_err(|_| error("mapping update poisoned"))
    }
}

impl MapWorkerState {
    pub(super) fn publish_navigation(&self, accepted_revision: Option<u64>) -> Result<()> {
        let mut update = self.updates.snapshot()?;
        update.verified = self.loops_verified;
        update.detected = self.detections_processed;
        update.accepted = self.pose_graph_optimizations;
        update.rejected = self.corrections_rejected;
        if let Some(source_revision) = accepted_revision
            && let Some((anchor_ns, initial)) = self.navigation_origin
        {
            let corrected = self
                .map
                .resolved_pose(anchor_ns)
                .ok_or_else(|| error("navigation origin lost from persistent map"))?;
            let (&active_ns, active) = self
                .map
                .snapshot
                .keyframes
                .iter()
                .rev()
                .find(|(_, frame)| frame.active)
                .ok_or_else(|| error("online correction has no active VIO anchor"))?;
            let local = self
                .original_poses
                .get(&active_ns)
                .ok_or_else(|| error("online correction has no same-time local pose"))?;
            // §III-B fixes the active window in the local VIO gauge. Re-express
            // that graph in a start-anchored, gravity-aligned navigation gauge.
            let map_from_graph = gravity_alignment(&initial, &corrected);
            let map_from_odom = map_from_graph * active.t_w_b * local.inverse();
            if !map_from_odom.to_homogeneous().iter().all(|v| v.is_finite()) {
                return Err(error("nonfinite committed navigation correction"));
            }
            update.correction = Some(Correction {
                session: update.session,
                revision: update.correction.map_or(1, |value| value.revision + 1),
                source_revision,
                anchor_ns,
                source_ns: self.latest_local_ns,
                map_from_odom,
            });
        }
        *self
            .updates
            .0
            .write()
            .map_err(|_| error("mapping update poisoned"))? = update;
        Ok(())
    }
}

fn gravity_alignment(initial: &Isometry3<f64>, corrected: &Isometry3<f64>) -> Isometry3<f64> {
    let yaw = initial.rotation.euler_angles().2 - corrected.rotation.euler_angles().2;
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
    Isometry3::from_parts(
        Translation3::from(initial.translation.vector - rotation * corrected.translation.vector),
        rotation,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn navigation_gauge_restores_origin_without_tilting_gravity() {
        let initial = Isometry3::translation(0., 0., 0.);
        let corrected = Isometry3::from_parts(
            Translation3::new(1., 2., 0.3),
            UnitQuaternion::from_euler_angles(0.1, -0.2, 0.5),
        );
        let alignment = gravity_alignment(&initial, &corrected);
        assert!((alignment * corrected).translation.vector.norm() < 1e-12);
        assert!((alignment.rotation * Vector3::z() - Vector3::z()).norm() < 1e-12);
        assert!((alignment.rotation.euler_angles().2 + 0.5).abs() < 1e-12);
        let latest_local = Isometry3::translation(22., 0., 1.);
        let active_graph = latest_local;
        assert!(
            (active_graph * latest_local.inverse())
                .translation
                .vector
                .norm()
                < 1e-12
        );
        assert!(
            (alignment * active_graph * latest_local.inverse())
                .translation
                .vector
                .norm()
                > 1.
        );
    }
}
