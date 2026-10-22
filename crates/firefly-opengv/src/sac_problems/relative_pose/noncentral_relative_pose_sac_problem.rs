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

use super::central_relative_pose_sac_problem::{
    Algorithm as CentralAlgorithm, CentralRelativePoseSacProblem, current_score, transformation,
};
use crate::{
    optimization_functor::NumericalBackend,
    relative_pose::{methods, relative_adapter_base::RelativeAdapterBase},
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
    types::Transformation,
};
use nalgebra::{DMatrix, Matrix3, Matrix4, Vector3, Vector4};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    Seventeenpt,
    Ge,
    Sixpt,
}
pub struct NoncentralRelativePoseSacProblem<'a, A: RelativeAdapterBase> {
    pub adapter: &'a mut A,
    pub algorithm: Algorithm,
    pub as_central: bool,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: SamplingState,
}
impl<'a, A: RelativeAdapterBase> NoncentralRelativePoseSacProblem<'a, A> {
    pub fn new(
        adapter: &'a mut A,
        algorithm: Algorithm,
        as_central: bool,
        backend: &'a dyn NumericalBackend,
        random_seed: bool,
    ) -> Self {
        let mut sampling = SamplingState::new(random_seed);
        sampling.set_uniform_indices(adapter.number_correspondences());
        Self {
            adapter,
            algorithm,
            as_central,
            backend,
            sampling,
        }
    }
}
pub(crate) fn direct_pose<A: RelativeAdapterBase>(
    adapter: &mut A,
    model: &Transformation,
    i: usize,
) {
    let t = model.column(3);
    let r = model.fixed_view::<3, 3>(0, 0);
    let t1 = adapter.cam_offset1(i);
    let r1 = adapter.cam_rotation1(i);
    let t2 = adapter.cam_offset2(i);
    let r2 = adapter.cam_rotation2(i);
    adapter.set_t12(r1.transpose() * ((t - t1) + r * t2));
    adapter.set_r12(r1.transpose() * r * r2);
}
pub(crate) fn noncentral_scores<A: RelativeAdapterBase>(
    adapter: &mut A,
    model: &Transformation,
    indices: &[usize],
) -> Vec<f64> {
    indices
        .iter()
        .map(|&i| {
            direct_pose(adapter, model, i);
            current_score(adapter, i)
        })
        .collect()
}
fn skew(t: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -t[2], t[1], t[2], 0.0, -t[0], -t[1], t[0], 0.0)
}
pub(crate) fn sixpt_model<A: RelativeAdapterBase>(
    adapter: &mut A,
    indices: &[usize],
    score_end: usize,
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    let rotations = methods::sixpt(adapter, indices.get(..6)?, backend)?;
    let mut transformations = Vec::new();
    for r in rotations {
        let mut gsum = Matrix4::zeros();
        for &i in &indices[..6] {
            let f1 = adapter.cam_rotation1(i) * adapter.bearing_vector1(i);
            let f2 = adapter.cam_rotation2(i) * adapter.bearing_vector2(i);
            let n = f1.cross(&(r * f2));
            let mut g = Vector4::zeros();
            g.fixed_rows_mut::<3>(0).copy_from(&n);
            g[3] = (f1.transpose()
                * (skew(adapter.cam_offset1(i)) * r - r * skew(adapter.cam_offset2(i)))
                * f2)[0];
            gsum += g * g.transpose();
        }
        let (_, v) = backend.eigenpairs(&DMatrix::from_column_slice(4, 4, gsum.as_slice()))?;
        let factor = v[(3, 0)].re;
        let t = Vector3::new(
            (1.0 / factor) * v[(0, 0)].re,
            (1.0 / factor) * v[(1, 0)].re,
            (1.0 / factor) * v[(2, 0)].re,
        );
        transformations.push(transformation(r, t));
    }
    let mut best_quality = 1_000_000.0;
    let mut best = None;
    for m in transformations {
        let mut quality = 0.0;
        for &i in indices.get(6..score_end)? {
            direct_pose(adapter, &m, i);
            quality += current_score(adapter, i);
        }
        if quality < best_quality {
            best_quality = quality;
            best = Some(m);
        }
    }
    best
}
impl<A: RelativeAdapterBase> SampleConsensusProblem for NoncentralRelativePoseSacProblem<'_, A> {
    type Model = Transformation;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        if self.as_central {
            8
        } else {
            match self.algorithm {
                Algorithm::Seventeenpt => 17,
                Algorithm::Ge => 8,
                Algorithm::Sixpt => 9,
            }
        }
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Transformation> {
        if self.as_central {
            let mut central = CentralRelativePoseSacProblem::new(
                self.adapter,
                CentralAlgorithm::Stewenius,
                self.backend,
                true,
            );
            let m = central.compute_model_coefficients(indices)?;
            let r = m.fixed_view::<3, 3>(0, 0);
            let t = m.column(3);
            let r1 = self.adapter.cam_rotation1(indices[0]);
            let r2 = self.adapter.cam_rotation2(indices[0]);
            let t1 = self.adapter.cam_offset1(indices[0]);
            let t2 = self.adapter.cam_offset2(indices[0]);
            let r = r1 * r * r2.transpose();
            return Some(transformation(r, t1 + r1 * t - r * t2));
        }
        match self.algorithm {
            Algorithm::Seventeenpt => methods::seventeenpt(self.adapter, indices, self.backend),
            Algorithm::Ge => {
                let out = methods::ge(self.adapter, indices, false, self.backend)?;
                Some(transformation(
                    out.rotation,
                    out.translation.fixed_rows::<3>(0).into_owned(),
                ))
            }
            Algorithm::Sixpt => {
                sixpt_model(self.adapter, indices, self.sample_size(), self.backend)
            }
        }
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Transformation,
        indices: &[usize],
    ) -> Vec<f64> {
        noncentral_scores(self.adapter, model, indices)
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        model: &Transformation,
    ) -> Option<Transformation> {
        self.adapter.set_t12(model.column(3).into_owned());
        self.adapter
            .set_r12(model.fixed_view::<3, 3>(0, 0).into_owned());
        methods::optimize_nonlinear(self.adapter, inliers, self.backend)
    }
}
