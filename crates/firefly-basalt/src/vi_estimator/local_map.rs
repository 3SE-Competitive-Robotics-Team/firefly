//! Optional integration snapshot of optimized local geometry before marginalization.
//! This output extends the original estimator API without changing its prior archive.
use super::{ba_base::BundleAdjustmentBase, landmark_database::Keypoint};
use crate::optical_flow::OpticalFlowResult;
use crate::utils::common_types::TimeCamId;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::Isometry3;
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

#[derive(Clone, Debug)]
pub struct LocalMapSnapshot {
    pub t_ns: i64,
    /// Body-to-world poses at the current optimized state, including non-keyframes.
    pub poses: BTreeMap<i64, Isometry3<f64>>,
    pub keyframes: BTreeSet<i64>,
    /// Host-camera inverse-distance geometry and all retained observations.
    pub landmarks: BTreeMap<usize, Keypoint<f64>>,
    pub frames: BTreeMap<i64, Arc<OpticalFlowResult>>,
    /// Explicit optimizer removals, separate from subsequent marginalization.
    pub rejected_landmarks: BTreeSet<usize>,
    pub rejected_observations: BTreeSet<(usize, TimeCamId)>,
}
impl LocalMapSnapshot {
    #[must_use]
    pub fn observation_ids<S: BasaltScalar>(
        base: &BundleAdjustmentBase<S>,
    ) -> BTreeMap<usize, BTreeSet<TimeCamId>> {
        base.lmdb
            .landmarks()
            .iter()
            .map(|(&id, point)| (id, point.obs.keys().copied().collect()))
            .collect()
    }
    /// Compare only snapshots immediately before and after optimization, with
    /// no intervening marginalization. Missing marginalized points are retained
    /// by the persistent map and must not be classified as rejected measurements.
    pub fn record_rejections(&mut self, previous: &BTreeMap<usize, BTreeSet<TimeCamId>>) {
        self.rejected_landmarks.clear();
        self.rejected_observations.clear();
        for (&id, observations) in previous {
            if let Some(point) = self
                .landmarks
                .get(&id)
                .filter(|point| point.inv_dist > 0. && point.inv_dist.is_finite())
            {
                for target in observations {
                    if !point.obs.contains_key(target) {
                        self.rejected_observations.insert((id, *target));
                    }
                }
            } else {
                self.rejected_landmarks.insert(id);
            }
        }
    }
    #[must_use]
    pub fn capture<S: BasaltScalar>(
        t_ns: i64,
        base: &BundleAdjustmentBase<S>,
        keyframes: &BTreeSet<i64>,
        frames: &BTreeMap<i64, Arc<OpticalFlowResult>>,
    ) -> Self {
        let mut poses: BTreeMap<_, _> = base
            .frame_states
            .iter()
            .map(|(&id, state)| (id, state.cast::<f64>().state().state.pose.t_w_i))
            .collect();
        poses.extend(
            base.frame_poses
                .iter()
                .map(|(&id, state)| (id, *state.cast::<f64>().pose())),
        );
        let landmarks = base
            .lmdb
            .landmarks()
            .iter()
            .map(|(&id, point)| {
                let mut output = Keypoint::new(
                    point.direction.map(BasaltScalar::as_f64),
                    point.inv_dist.as_f64(),
                    point.host_kf_id,
                );
                output.obs = point
                    .obs
                    .iter()
                    .map(|(&target, pixel)| (target, pixel.map(BasaltScalar::as_f64)))
                    .collect();
                (id, output)
            })
            .collect();
        Self {
            t_ns,
            poses,
            keyframes: keyframes.clone(),
            landmarks,
            frames: frames.clone(),
            rejected_landmarks: BTreeSet::new(),
            rejected_observations: BTreeSet::new(),
        }
    }
}
