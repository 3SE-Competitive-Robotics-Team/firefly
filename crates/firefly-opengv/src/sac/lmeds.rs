/******************************************************************************
 * Authors:  Johannes Mikulasch                                               *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

use super::{sample_consensus::SampleConsensus, sample_consensus_problem::SampleConsensusProblem};
use std::ops::{Deref, DerefMut};

pub struct Lmeds<P: SampleConsensusProblem>(pub SampleConsensus<P>);
impl<P: SampleConsensusProblem> Deref for Lmeds<P> {
    type Target = SampleConsensus<P>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<P: SampleConsensusProblem> DerefMut for Lmeds<P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl<P: SampleConsensusProblem> Lmeds<P> {
    pub fn new(problem: P, max_iterations: i32, threshold: f64, probability: f64) -> Self {
        Self(SampleConsensus::new(
            problem,
            max_iterations,
            threshold,
            probability,
        ))
    }
    /// NaN distances cannot satisfy the source sort's strict weak ordering.
    /// # Panics
    /// Panics if ordering a distance fails despite the explicit rejection of NaNs.
    pub fn compute_model(&mut self, debug_verbosity_level: i32) -> bool {
        let s = &mut self.0;
        if s.threshold == f64::MAX {
            log::error!("No threshold set");
            return false;
        }
        s.iterations = 0;
        let mut best = f64::MAX;
        let mut selection = Vec::new();
        let mut skipped = 0_u32;
        let max_skip = s.max_iterations.wrapping_mul(10) as u32;
        if debug_verbosity_level > 1 {
            log::info!(
                "Starting Least Median of Squares; max_iterations: {}, max_skip: {max_skip}",
                s.max_iterations
            );
        }
        while s.iterations < s.max_iterations && skipped < max_skip {
            s.sac_model.get_samples(&mut s.iterations, &mut selection);
            if selection.is_empty() {
                log::error!("No samples could be selected");
                break;
            }
            let Some(coeff) = s.sac_model.compute_model_coefficients(&selection) else {
                skipped += 1;
                continue;
            };
            let mut distances = s.sac_model.distances_to_model(&coeff);
            if distances.is_empty() {
                skipped += 1;
                continue;
            }
            if !square_distances(&mut distances) {
                return false;
            }
            distances.sort_unstable_by(|a, b| a.partial_cmp(b).expect("NaN rejected"));
            let n = s.sac_model.indices().len();
            let mid = n / 2;
            if mid >= distances.len() {
                skipped += 1;
                continue;
            }
            let penalty = if n.is_multiple_of(2) {
                let Some(previous) = mid.checked_sub(1) else {
                    return false;
                };
                {
                    let middle_sum = distances[previous] + distances[mid];
                    middle_sum / 2.0
                }
            } else {
                distances[mid]
            };
            if penalty < best {
                best = penalty;
                s.model = selection.clone();
                s.model_coefficients = Some(coeff);
            }
            s.iterations += 1;
            if debug_verbosity_level > 1 {
                log::info!(
                    "Trial {} out of {}; best penalty: {best}, current penalty: {penalty}",
                    s.iterations,
                    s.max_iterations
                );
            }
        }
        if s.model.is_empty() {
            if debug_verbosity_level > 0 {
                log::info!("Unable to find a solution");
            }
            return false;
        }
        let Some(coeff) = s.model_coefficients.as_ref() else {
            return false;
        };
        let distances = s.sac_model.distances_to_model(coeff);
        if distances.is_empty() {
            log::error!("The model found failed to verify against the given constraints");
            return false;
        }
        if distances.len() != s.sac_model.indices().len() {
            log::error!(
                "Estimated distances ({}) differs from the number of indices ({})",
                distances.len(),
                s.sac_model.indices().len()
            );
            return false;
        }
        s.inliers = distances
            .iter()
            .zip(s.sac_model.indices())
            .filter_map(|(&d, &i)| (d <= s.threshold).then_some(i))
            .collect();
        if debug_verbosity_level > 0 {
            log::info!("Model: {} size, {} inliers", s.model.len(), s.inliers.len());
        }
        true
    }
}

fn square_distances(distances: &mut [f64]) -> bool {
    for d in distances.iter_mut() {
        if *d < 0.0 {
            *d = 0.0;
        }
        *d *= *d;
    }
    if distances.iter().any(|d| d.is_nan()) {
        return false;
    }
    true
}
