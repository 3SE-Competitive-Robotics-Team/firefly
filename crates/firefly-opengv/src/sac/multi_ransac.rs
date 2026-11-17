use super::{
    multi_sample_consensus::MultiSampleConsensus,
    multi_sample_consensus_problem::MultiSampleConsensusProblem,
};
use std::ops::{Deref, DerefMut};

pub struct MultiRansac<P: MultiSampleConsensusProblem>(pub MultiSampleConsensus<P>);
impl<P: MultiSampleConsensusProblem> Deref for MultiRansac<P> {
    type Target = MultiSampleConsensus<P>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<P: MultiSampleConsensusProblem> DerefMut for MultiRansac<P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl<P: MultiSampleConsensusProblem> MultiRansac<P> {
    pub fn new(problem: P, max_iterations: i32, threshold: f64, probability: f64) -> Self {
        Self(MultiSampleConsensus::new(
            problem,
            max_iterations,
            threshold,
            probability,
        ))
    }
    pub fn compute_model(&mut self, debug_verbosity_level: i32) -> bool {
        let state = &mut self.0;
        state.iterations = 0;
        let mut best = -i64::from(i32::MAX);
        let mut root_product = 1.0;
        let mut selection = Vec::new();
        let mut skipped = 0_u32;
        let max_skip = state.max_iterations.wrapping_mul(10) as u32;
        while f64::from(state.iterations) < root_product && skipped < max_skip {
            state
                .sac_model
                .draw_samples(&mut state.iterations, &mut selection);
            if selection.is_empty() {
                log::error!("No samples could be selected");
                break;
            }
            let Some(coeff) = state.sac_model.compute_model_coefficients(&selection) else {
                skipped += 1;
                continue;
            };
            let count = state
                .sac_model
                .count_within_distance(&coeff, state.threshold) as i64;
            if count > best {
                best = count;
                state.model = selection.clone();
                state.model_coefficients = Some(coeff);
                let dimension = state
                    .sac_model
                    .indices()
                    .iter()
                    .map(Vec::len)
                    .sum::<usize>();
                let matrix = selection.iter().map(Vec::len).sum::<usize>();
                let root_weight = best as f64 / dimension as f64;
                let point = 1.0 - root_weight.powf(matrix as f64);
                let point = if f64::EPSILON < point {
                    point
                } else {
                    f64::EPSILON
                };
                let point = if point < 1.0 - f64::EPSILON {
                    point
                } else {
                    1.0 - f64::EPSILON
                };
                root_product = (1.0 - state.probability).ln() / point.ln();
            }
            state.iterations += 1;
            if debug_verbosity_level > 1 {
                log::info!(
                    "Trial {} out of {root_product}: {count} inliers (best: {best})",
                    state.iterations
                );
            }
            if state.iterations > state.max_iterations {
                if debug_verbosity_level > 0 {
                    log::info!("RANSAC reached the maximum number of trials");
                }
                break;
            }
        }
        if debug_verbosity_level > 0 {
            log::info!(
                "Model: {} size, {best} inliers",
                state.model.iter().map(Vec::len).sum::<usize>()
            );
        }
        if state.model.is_empty() {
            state.inliers.clear();
            return false;
        }
        let Some(coeff) = state.model_coefficients.as_ref() else {
            return false;
        };
        state.inliers = state
            .sac_model
            .select_within_distance(coeff, state.threshold);
        true
    }
}
