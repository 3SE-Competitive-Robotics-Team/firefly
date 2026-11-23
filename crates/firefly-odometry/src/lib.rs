mod receiver;
mod state_input;

pub use receiver::{OdometryHistory, OdometryReceiver};
pub use state_input::{Snapshot, StateInput};

use firefly_pubsub::odom::OdomMessage;
use nalgebra::Isometry3;
use std::collections::VecDeque;

/// Validated samples around a measurement time, copied while the history is locked.
pub struct PoseBracket {
    before: OdomMessage,
    after: OdomMessage,
    fraction: f64,
}

impl PoseBracket {
    #[must_use]
    pub fn find(history: &VecDeque<OdomMessage>, stamp: f64, max_gap: f64) -> Option<Self> {
        if !stamp.is_finite() {
            return None;
        }
        let index = history.partition_point(|sample| sample.timestamp < stamp);
        let after = *history.get(index)?;
        let (before, fraction) = if after.timestamp == stamp {
            (after, 0.)
        } else {
            let before = *history.get(index.checked_sub(1)?)?;
            let gap = after.timestamp - before.timestamp;
            if gap <= 0. || gap > max_gap {
                return None;
            }
            (before, (stamp - before.timestamp) / gap)
        };
        (before.is_initialized && after.is_initialized).then_some(Self {
            before,
            after,
            fraction,
        })
    }

    #[must_use]
    pub fn pose(&self) -> Option<Isometry3<f64>> {
        let left = self.before.body_pose(firefly_base::FrameId::ODOM).ok()?;
        if self.fraction == 0. {
            return Some(*left.isometry());
        }
        let right = self.after.body_pose(firefly_base::FrameId::ODOM).ok()?;
        Some(left.isometry().lerp_slerp(right.isometry(), self.fraction))
    }
}
