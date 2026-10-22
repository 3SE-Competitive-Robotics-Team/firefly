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

use super::{
    central_relative_pose_sac_problem::{
        Algorithm as CentralAlgorithm, CentralRelativePoseSacProblem, reprojections, transformation,
    },
    noncentral_relative_pose_sac_problem::{Algorithm, sixpt_model},
};
use crate::sac_problems::absolute_pose::multi_noncentral_absolute_pose_sac_problem::{
    distribute, random_start,
};
use crate::{
    optimization_functor::NumericalBackend,
    relative_pose::{methods, relative_multi_adapter_base::RelativeMultiAdapterBase},
    sac::{
        multi_sample_consensus_problem::{MultiSampleConsensusProblem, MultiSamplingState},
        sample_consensus_problem::SampleConsensusProblem,
    },
    types::Transformation,
};
pub struct MultiNoncentralRelativePoseSacProblem<'a, A: RelativeMultiAdapterBase> {
    pub adapter: &'a mut A,
    pub algorithm: Algorithm,
    pub as_central: bool,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: MultiSamplingState,
}
impl<'a, A: RelativeMultiAdapterBase> MultiNoncentralRelativePoseSacProblem<'a, A> {
    pub fn new(
        adapter: &'a mut A,
        algorithm: Algorithm,
        as_central: bool,
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
            algorithm,
            as_central,
            backend,
            sampling,
        }
    }
}
/// Triangulation uses each selected index; the bearing lookup uses its list position.
pub(crate) fn grouped_scores<A: RelativeMultiAdapterBase>(
    adapter: &mut A,
    model: &Transformation,
    indices: &[Vec<usize>],
) -> Vec<Vec<f64>> {
    indices
        .iter()
        .enumerate()
        .map(|(g, values)| {
            let offset = adapter.cam_offset(g);
            let rotation = adapter.cam_rotation(g);
            let t = model.column(3);
            let r = model.fixed_view::<3, 3>(0, 0);
            adapter.set_t12(rotation.transpose() * ((t - offset) + r * offset));
            adapter.set_r12(rotation.transpose() * r * rotation);
            values
                .iter()
                .enumerate()
                .map(|(position, &i)| {
                    let (p, q) = reprojections(adapter, adapter.convert_multiindex(g, i));
                    (1.0 - adapter.bearing_vector1_at(g, position).dot(&p))
                        + (1.0 - adapter.bearing_vector2_at(g, position).dot(&q))
                })
                .collect()
        })
        .collect()
}
impl<A: RelativeMultiAdapterBase> MultiSampleConsensusProblem
    for MultiNoncentralRelativePoseSacProblem<'_, A>
{
    type Model = Transformation;
    fn state(&self) -> &MultiSamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut MultiSamplingState {
        &mut self.sampling
    }
    fn sample_sizes(&self) -> Vec<usize> {
        let n = if self.as_central {
            8
        } else {
            match self.algorithm {
                Algorithm::Seventeenpt => 17,
                Algorithm::Ge => 8,
                Algorithm::Sixpt => 9,
            }
        };
        distribute(self.adapter.number_pairs(), n)
    }
    fn compute_model_coefficients(&mut self, indices: &[Vec<usize>]) -> Option<Transformation> {
        if self.as_central {
            let serialized = self.adapter.convert_multi_indices(indices);
            let mut central = CentralRelativePoseSacProblem::new(
                self.adapter,
                CentralAlgorithm::Stewenius,
                self.backend,
                true,
            );
            let m = central.compute_model_coefficients(&serialized)?;
            let r = m.fixed_view::<3, 3>(0, 0);
            let t = m.column(3);
            let bc = self.adapter.cam_rotation(0);
            let offset = self.adapter.cam_offset(0);
            let r = bc * r * bc.transpose();
            return Some(transformation(r, offset + bc * t - r * offset));
        }
        match self.algorithm {
            Algorithm::Seventeenpt => methods::seventeenpt(
                self.adapter,
                &self.adapter.convert_multi_indices(indices),
                self.backend,
            ),
            Algorithm::Ge => {
                let out = methods::ge(
                    self.adapter,
                    &self.adapter.convert_multi_indices(indices),
                    false,
                    self.backend,
                )?;
                Some(transformation(
                    out.rotation,
                    out.translation.fixed_rows::<3>(0).into_owned(),
                ))
            }
            Algorithm::Sixpt => {
                let mut bin = random_start(self.adapter.number_pairs())?;
                if bin >= indices.len() {
                    return None;
                }
                let mut positions = vec![0; indices.len()];
                let mut serialized = Vec::new();
                while positions.iter().zip(indices).any(|(&i, v)| i < v.len()) {
                    while positions[bin] >= indices[bin].len() {
                        bin += 1;
                        if bin >= indices.len() {
                            bin = 0;
                        }
                    }
                    serialized.push(
                        self.adapter
                            .convert_multiindex(bin, indices[bin][positions[bin]]),
                    );
                    positions[bin] += 1;
                    bin += 1;
                    if bin >= indices.len() {
                        bin = 0;
                    }
                }
                sixpt_model(self.adapter, &serialized, serialized.len(), self.backend)
            }
        }
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Transformation,
        indices: &[Vec<usize>],
    ) -> Vec<Vec<f64>> {
        grouped_scores(self.adapter, model, indices)
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[Vec<usize>],
        model: &Transformation,
    ) -> Option<Transformation> {
        self.adapter.set_t12(model.column(3).into_owned());
        self.adapter
            .set_r12(model.fixed_view::<3, 3>(0, 0).into_owned());
        methods::optimize_nonlinear(
            self.adapter,
            &self.adapter.convert_multi_indices(inliers),
            self.backend,
        )
    }
}
