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
    absolute_pose::{absolute_adapter_base::AbsoluteAdapterBase, methods},
    optimization_functor::NumericalBackend,
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
    types::Transformation,
};
use nalgebra::{Matrix3, Vector3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    Twopt,
    Kneip,
    Gao,
    Epnp,
    Gp3p,
}
pub struct AbsolutePoseSacProblem<'a, A: AbsoluteAdapterBase> {
    pub adapter: &'a mut A,
    pub algorithm: Algorithm,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: SamplingState,
}
impl<'a, A: AbsoluteAdapterBase> AbsolutePoseSacProblem<'a, A> {
    pub fn new(
        adapter: &'a mut A,
        algorithm: Algorithm,
        backend: &'a dyn NumericalBackend,
        random_seed: bool,
    ) -> Self {
        let mut sampling = SamplingState::new(random_seed);
        sampling.set_uniform_indices(adapter.number_correspondences());
        Self {
            adapter,
            algorithm,
            backend,
            sampling,
        }
    }
}
pub(crate) fn body_frame(
    solution: &Transformation,
    offset: Vector3<f64>,
    cam_rotation: Matrix3<f64>,
) -> Transformation {
    let r = solution.fixed_view::<3, 3>(0, 0).into_owned();
    let t = solution.column(3).into_owned();
    let mut out = *solution;
    out.set_column(3, &(t - r * cam_rotation.transpose() * offset));
    out.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(r * cam_rotation.transpose()));
    out
}
pub(crate) fn score<A: AbsoluteAdapterBase>(
    adapter: &A,
    model: &Transformation,
    index: usize,
) -> f64 {
    let r = model.fixed_view::<3, 3>(0, 0);
    let mut inverse = Transformation::zeros();
    inverse
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&r.transpose());
    inverse.set_column(3, &(-r.transpose() * model.column(3)));
    let point = adapter.point(index);
    let body = inverse * nalgebra::Vector4::new(point[0], point[1], point[2], 1.0);
    let reprojection = adapter.cam_rotation(index).transpose() * (body - adapter.cam_offset(index));
    1.0 - (reprojection / reprojection.norm()).dot(&adapter.bearing_vector(index))
}
pub(crate) fn best_solution<A: AbsoluteAdapterBase>(
    adapter: &A,
    solutions: Vec<Transformation>,
    fourth: usize,
) -> Option<Transformation> {
    if solutions.len() == 1 {
        return Some(solutions[0]);
    }
    let mut min = 1_000_000.0;
    let mut best = None;
    for solution in solutions {
        let value = score(adapter, &solution, fourth);
        if value < min {
            min = value;
            best = Some(solution);
        }
    }
    best
}
impl<A: AbsoluteAdapterBase> SampleConsensusProblem for AbsolutePoseSacProblem<'_, A> {
    type Model = Transformation;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        match self.algorithm {
            Algorithm::Twopt => 2,
            Algorithm::Epnp => 6,
            _ => 4,
        }
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Transformation> {
        let mut solutions = match self.algorithm {
            Algorithm::Twopt => {
                let t = methods::p2p(self.adapter, indices)?;
                let mut m = Transformation::zeros();
                m.set_column(3, &t);
                m.fixed_view_mut::<3, 3>(0, 0).copy_from(&self.adapter.r());
                vec![m]
            }
            Algorithm::Kneip => methods::p3p_kneip(self.adapter, indices)?,
            Algorithm::Gao => methods::p3p_gao(self.adapter, indices)?,
            Algorithm::Epnp => vec![methods::epnp(self.adapter, indices)?],
            Algorithm::Gp3p => methods::gp3p(self.adapter, indices, self.backend)?,
        };
        if self.algorithm != Algorithm::Gp3p {
            let t = self.adapter.cam_offset(indices[0]);
            let r = self.adapter.cam_rotation(indices[0]);
            for s in &mut solutions {
                *s = body_frame(s, t, r);
            }
        }
        if solutions.len() == 1 {
            return Some(solutions[0]);
        }
        best_solution(self.adapter, solutions, *indices.get(3)?)
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Transformation,
        indices: &[usize],
    ) -> Vec<f64> {
        indices
            .iter()
            .map(|&i| score(self.adapter, model, i))
            .collect()
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        model: &Transformation,
    ) -> Option<Transformation> {
        self.adapter.set_t(model.column(3).into_owned());
        self.adapter
            .set_r(model.fixed_view::<3, 3>(0, 0).into_owned());
        methods::optimize_nonlinear(self.adapter, inliers, self.backend)
    }
}
