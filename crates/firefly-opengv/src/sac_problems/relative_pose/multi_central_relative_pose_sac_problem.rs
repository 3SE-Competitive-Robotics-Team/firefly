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

use super::central_relative_pose_sac_problem::{decompose, reprojections};
use crate::{
    optimization_functor::NumericalBackend,
    relative_pose::{methods, relative_multi_adapter_base::RelativeMultiAdapterBase},
    sac::multi_sample_consensus_problem::{MultiSampleConsensusProblem, MultiSamplingState},
    types::Transformation,
};
pub struct MultiCentralRelativePoseSacProblem<'a, A: RelativeMultiAdapterBase> {
    pub adapter: &'a mut A,
    pub sample_size: usize,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: MultiSamplingState,
}
impl<'a, A: RelativeMultiAdapterBase> MultiCentralRelativePoseSacProblem<'a, A> {
    pub fn new(
        adapter: &'a mut A,
        sample_size: usize,
        backend: &'a dyn NumericalBackend,
        random_seed: bool,
    ) -> Self {
        let mut sampling = MultiSamplingState::new(random_seed);
        sampling.set_uniform_indices(
            (0..adapter.number_pairs())
                .map(|g| adapter.number_correspondences_at(g))
                .collect(),
        );
        Self {
            adapter,
            sample_size,
            backend,
            sampling,
        }
    }
}
impl<A: RelativeMultiAdapterBase> MultiSampleConsensusProblem
    for MultiCentralRelativePoseSacProblem<'_, A>
{
    type Model = Vec<Transformation>;
    fn state(&self) -> &MultiSamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut MultiSamplingState {
        &mut self.sampling
    }
    fn sample_sizes(&self) -> Vec<usize> {
        vec![self.sample_size; self.adapter.number_pairs()]
    }
    /// The source decomposes essentials but supplies no output transformation.
    fn compute_model_coefficients(&mut self, indices: &[Vec<usize>]) -> Option<Self::Model> {
        let mut translations = Vec::new();
        let mut rotations = Vec::new();
        if indices.len() < self.adapter.number_pairs() {
            return None;
        }
        for (g, values) in indices.iter().enumerate().take(self.adapter.number_pairs()) {
            let serialized: Vec<_> = values
                .iter()
                .map(|&i| self.adapter.convert_multiindex(g, i))
                .collect();
            let essential = methods::eightpt(self.adapter, &serialized, self.backend)?;
            let candidates = decompose(&essential, self.backend)?;
            translations.push([
                candidates[0].column(3).into_owned(),
                candidates[2].column(3).into_owned(),
            ]);
            rotations.push([
                candidates[0].fixed_view::<3, 3>(0, 0).into_owned(),
                candidates[1].fixed_view::<3, 3>(0, 0).into_owned(),
            ]);
        }
        None
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Self::Model,
        indices: &[Vec<usize>],
    ) -> Vec<Vec<f64>> {
        indices
            .iter()
            .enumerate()
            .map(|(g, values)| {
                values
                    .iter()
                    .enumerate()
                    .map(|(position, &i)| {
                        self.adapter.set_t12(model[g].column(3).into_owned());
                        self.adapter
                            .set_r12(model[g].fixed_view::<3, 3>(0, 0).into_owned());
                        let (p, q) =
                            reprojections(self.adapter, self.adapter.convert_multiindex(g, i));
                        (1.0 - self.adapter.bearing_vector1_at(g, position).dot(&p))
                            + (1.0 - self.adapter.bearing_vector2_at(g, position).dot(&q))
                    })
                    .collect()
            })
            .collect()
    }
    fn optimize_model_coefficients(
        &mut self,
        _: &[Vec<usize>],
        model: &Self::Model,
    ) -> Option<Self::Model> {
        Some(model.clone())
    }
}
