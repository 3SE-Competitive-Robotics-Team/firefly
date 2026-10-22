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

use super::absolute_pose_sac_problem::{best_solution, body_frame};
use crate::{
    absolute_pose::{absolute_multi_adapter_base::AbsoluteMultiAdapterBase, methods},
    optimization_functor::NumericalBackend,
    sac::multi_sample_consensus_problem::{MultiSampleConsensusProblem, MultiSamplingState},
    types::Transformation,
};
use rand::RngExt;
pub struct MultiNoncentralAbsolutePoseSacProblem<'a, A: AbsoluteMultiAdapterBase> {
    pub adapter: &'a mut A,
    pub as_central: bool,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: MultiSamplingState,
}
impl<'a, A: AbsoluteMultiAdapterBase> MultiNoncentralAbsolutePoseSacProblem<'a, A> {
    pub fn new(
        adapter: &'a mut A,
        as_central: bool,
        backend: &'a dyn NumericalBackend,
        random_seed: bool,
    ) -> Self {
        let mut sampling = MultiSamplingState::new(random_seed);
        sampling.set_uniform_indices(
            (0..adapter.number_frames())
                .map(|g| adapter.number_correspondences_at(g))
                .collect(),
        );
        Self {
            adapter,
            as_central,
            backend,
            sampling,
        }
    }
}
/// C `rand()/RAND_MAX` includes one: its invalid terminal bin produces no sample.
pub(crate) fn random_start(groups: usize) -> Option<usize> {
    let draw = rand::rng().random_range(0..=i32::MAX);
    let start = (groups as f64 * f64::from(draw) / f64::from(i32::MAX)).floor() as usize;
    (start < groups).then_some(start)
}
pub(crate) fn distribute(groups: usize, size: usize) -> Vec<usize> {
    let Some(mut bin) = random_start(groups) else {
        return Vec::new();
    };
    let mut sizes = vec![0; groups];
    for _ in 0..size {
        sizes[bin] += 1;
        bin += 1;
        if bin >= groups {
            bin = 0;
        }
    }
    sizes
}
impl<A: AbsoluteMultiAdapterBase> MultiSampleConsensusProblem
    for MultiNoncentralAbsolutePoseSacProblem<'_, A>
{
    type Model = Transformation;
    fn state(&self) -> &MultiSamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut MultiSamplingState {
        &mut self.sampling
    }
    fn sample_sizes(&self) -> Vec<usize> {
        distribute(self.adapter.number_frames(), 4)
    }
    fn compute_model_coefficients(&mut self, indices: &[Vec<usize>]) -> Option<Transformation> {
        let serialized = self.adapter.convert_multi_indices(indices);
        let mut solutions = if self.as_central {
            methods::p3p_kneip(self.adapter, &serialized)?
        } else {
            methods::gp3p(self.adapter, &serialized, self.backend)?
        };
        if self.as_central {
            let t = self.adapter.cam_offset(0);
            let r = self.adapter.cam_rotation(0);
            for s in &mut solutions {
                *s = body_frame(s, t, r);
            }
        }
        if solutions.len() == 1 {
            return Some(solutions[0]);
        }
        best_solution(self.adapter, solutions, *serialized.get(3)?)
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Transformation,
        indices: &[Vec<usize>],
    ) -> Vec<Vec<f64>> {
        let r = model.fixed_view::<3, 3>(0, 0);
        let t = model.column(3);
        let mut inverse = Transformation::zeros();
        inverse
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&r.transpose());
        inverse.set_column(3, &(-r.transpose() * t));
        indices
            .iter()
            .enumerate()
            .map(|(g, v)| {
                v.iter()
                    .map(|&i| {
                        let point = self.adapter.point_at(g, i);
                        let body =
                            inverse * nalgebra::Vector4::new(point[0], point[1], point[2], 1.0);
                        let p = self.adapter.multi_cam_rotation(g).transpose()
                            * (body - self.adapter.multi_cam_offset(g));
                        1.0 - (p / p.norm()).dot(&self.adapter.bearing_vector_at(g, i))
                    })
                    .collect()
            })
            .collect()
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[Vec<usize>],
        model: &Transformation,
    ) -> Option<Transformation> {
        self.adapter.set_t(model.column(3).into_owned());
        self.adapter
            .set_r(model.fixed_view::<3, 3>(0, 0).into_owned());
        methods::optimize_nonlinear(
            self.adapter,
            &self.adapter.convert_multi_indices(inliers),
            self.backend,
        )
    }
}
