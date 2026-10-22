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
    optimization_functor::NumericalBackend,
    relative_pose::{methods, relative_adapter_base::RelativeAdapterBase},
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
    types::Transformation,
};
use nalgebra::{DMatrix, Matrix3, Vector3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    Nister,
    Stewenius,
    Sevenpt,
    Eightpt,
}
pub struct CentralRelativePoseSacProblem<'a, A: RelativeAdapterBase> {
    pub adapter: &'a mut A,
    pub algorithm: Algorithm,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: SamplingState,
}
impl<'a, A: RelativeAdapterBase> CentralRelativePoseSacProblem<'a, A> {
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
pub(crate) fn transformation(r: Matrix3<f64>, t: Vector3<f64>) -> Transformation {
    let mut m = Transformation::zeros();
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    m.set_column(3, &t);
    m
}
struct TriangulationAdapter<'a, A>(&'a A);
impl<A: RelativeAdapterBase> crate::triangulation::methods::RelativeAdapterBase
    for TriangulationAdapter<'_, A>
{
    fn t12(&self) -> Vector3<f64> {
        self.0.t12()
    }
    fn r12(&self) -> Matrix3<f64> {
        self.0.r12()
    }
    fn bearing_vector1(&self, i: usize) -> Vector3<f64> {
        self.0.bearing_vector1(i)
    }
    fn bearing_vector2(&self, i: usize) -> Vector3<f64> {
        self.0.bearing_vector2(i)
    }
}
pub(crate) fn reprojections<A: RelativeAdapterBase>(
    adapter: &A,
    index: usize,
) -> (Vector3<f64>, Vector3<f64>) {
    let p = crate::triangulation::methods::triangulate2(&TriangulationAdapter(adapter), index);
    let inverse_rotation = adapter.r12().transpose();
    let mut inverse = Transformation::zeros();
    inverse
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&inverse_rotation);
    inverse.set_column(3, &(-inverse_rotation * adapter.t12()));
    let q = inverse * nalgebra::Vector4::new(p[0], p[1], p[2], 1.0);
    (p / p.norm(), q / q.norm())
}
pub(crate) fn current_score<A: RelativeAdapterBase>(adapter: &A, index: usize) -> f64 {
    let (p, q) = reprojections(adapter, index);
    (1.0 - adapter.bearing_vector1(index).dot(&p)) + (1.0 - adapter.bearing_vector2(index).dot(&q))
}
pub(crate) fn central_scores<A: RelativeAdapterBase>(
    adapter: &mut A,
    model: &Transformation,
    indices: &[usize],
) -> Vec<f64> {
    adapter.set_t12(model.column(3).into_owned());
    adapter.set_r12(model.fixed_view::<3, 3>(0, 0).into_owned());
    indices.iter().map(|&i| current_score(adapter, i)).collect()
}
pub(crate) fn decompose(
    essential: &Matrix3<f64>,
    backend: &dyn NumericalBackend,
) -> Option<[Transformation; 4]> {
    let (u, values, v) =
        backend.full_svd(&DMatrix::from_column_slice(3, 3, essential.as_slice()))?;
    let u = Matrix3::from_column_slice(u.as_slice());
    let v = Matrix3::from_column_slice(v.as_slice());
    let w = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    let mut ra = u * w * v.transpose();
    let mut rb = u * w.transpose() * v.transpose();
    if ra.determinant() < 0.0 {
        ra = -ra;
    }
    if rb.determinant() < 0.0 {
        rb = -rb;
    }
    let t = values[0] * u.column(2).into_owned();
    Some([
        transformation(ra, t),
        transformation(rb, t),
        transformation(ra, -t),
        transformation(rb, -t),
    ])
}
impl<A: RelativeAdapterBase> SampleConsensusProblem for CentralRelativePoseSacProblem<'_, A> {
    type Model = Transformation;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        match self.algorithm {
            Algorithm::Nister | Algorithm::Stewenius => 8,
            Algorithm::Sevenpt | Algorithm::Eightpt => 9,
        }
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Transformation> {
        let essentials = match self.algorithm {
            Algorithm::Nister => {
                methods::fivept_nister(self.adapter, indices.get(..5)?, self.backend)?
            }
            Algorithm::Stewenius => {
                methods::fivept_stewenius(self.adapter, indices.get(..5)?, self.backend)?
                    .into_iter()
                    .map(|e| e.map(|x| x.re))
                    .collect()
            }
            Algorithm::Sevenpt => methods::sevenpt(self.adapter, indices.get(..7)?, self.backend)?,
            Algorithm::Eightpt => vec![methods::eightpt(
                self.adapter,
                indices.get(..8)?,
                self.backend,
            )?],
        };
        let mut best_quality = 1_000_000.0;
        let mut best = None;
        for (i, e) in essentials.iter().enumerate() {
            let candidates = decompose(e, self.backend)?;
            for (j, m) in candidates.iter().enumerate() {
                self.adapter.set_t12(m.column(3).into_owned());
                self.adapter
                    .set_r12(m.fixed_view::<3, 3>(0, 0).into_owned());
                let mut quality = 0.0;
                for &k in indices.get(..self.sample_size())? {
                    quality += current_score(self.adapter, k);
                }
                if quality < best_quality {
                    best_quality = quality;
                    best = Some((i, j));
                }
            }
        }
        let (i, j) = best?;
        Some(decompose(&essentials[i], self.backend)?[j])
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Transformation,
        indices: &[usize],
    ) -> Vec<f64> {
        central_scores(self.adapter, model, indices)
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
