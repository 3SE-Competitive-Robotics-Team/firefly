/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
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

use crate::{
    relative_pose::{methods, relative_adapter_base::RelativeAdapterBase},
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
};
use nalgebra::Matrix3;
pub struct RotationOnlySacProblem<'a, A: RelativeAdapterBase> {
    pub adapter: &'a mut A,
    pub sampling: SamplingState,
}
impl<'a, A: RelativeAdapterBase> RotationOnlySacProblem<'a, A> {
    pub fn new(adapter: &'a mut A, random_seed: bool) -> Self {
        let mut sampling = SamplingState::new(random_seed);
        sampling.set_uniform_indices(adapter.number_correspondences());
        Self { adapter, sampling }
    }
}
impl<A: RelativeAdapterBase> SampleConsensusProblem for RotationOnlySacProblem<'_, A> {
    type Model = Matrix3<f64>;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        2
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Self::Model> {
        Some(methods::twopt_rotation_only(self.adapter, indices))
    }
    fn selected_distances_to_model(&mut self, model: &Self::Model, indices: &[usize]) -> Vec<f64> {
        indices
            .iter()
            .map(|&i| {
                1.0 - self
                    .adapter
                    .bearing_vector1(i)
                    .dot(&(model * self.adapter.bearing_vector2(i)))
            })
            .collect()
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        _: &Self::Model,
    ) -> Option<Self::Model> {
        Some(methods::rotation_only(self.adapter, inliers))
    }
}
