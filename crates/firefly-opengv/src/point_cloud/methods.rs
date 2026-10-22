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
    math::{
        arun::arun,
        cayley::{cayley2rot, rot2cayley},
    },
    optimization_functor::{LmParameters, NumericalBackend},
    point_cloud::point_cloud_adapter_base::PointCloudAdapterBase,
    types::Transformation,
};
use nalgebra::{DVector, Matrix3, Vector3};
/// # Panics
/// Panics if fewer than three indices are provided, or if an adapter index is invalid.
pub fn threept_arun(a: &impl PointCloudAdapterBase, idx: &[usize]) -> Transformation {
    assert!(idx.len() > 2);
    let (mut c1, mut c2) = (Vector3::zeros(), Vector3::zeros());
    for &i in idx {
        c1 += a.point1(i);
        c2 += a.point2(i);
    }
    c1 /= idx.len() as f64;
    c2 /= idx.len() as f64;
    let mut h = Matrix3::zeros();
    for &i in idx {
        h += (a.point2(i) - c2) * (a.point1(i) - c1).transpose();
    }
    let r = arun(&h);
    let t = c1 - r * c2;
    let mut out = Transformation::zeros();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    out.set_column(3, &t);
    out
}
pub fn optimize_nonlinear(
    leading: &impl PointCloudAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    let mut x = DVector::zeros(6);
    x.fixed_rows_mut::<3>(0).copy_from(&leading.t12());
    x.fixed_rows_mut::<3>(3)
        .copy_from(&rot2cayley(&leading.r12()));
    let mut residual = |x: &DVector<f64>, out: &mut DVector<f64>| {
        if x.len() != 6 || out.len() != idx.len() {
            return None;
        }
        let rotation = cayley2rot(&x.fixed_rows::<3>(3).into_owned());
        let translation = x.fixed_rows::<3>(0);
        for (i, &root_product) in idx.iter().enumerate() {
            let mut transformation = Transformation::zeros();
            transformation
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&rotation);
            transformation.column_mut(3).copy_from(&translation);
            let point = leading.point2(root_product);
            let p_hom = nalgebra::Vector4::new(point[0], point[1], point[2], 1.0);
            let point_value = transformation * p_hom;
            out[i] = (leading.point1(root_product) - point_value).norm();
        }
        Some(())
    };
    backend.levenberg_marquardt(
        &mut x,
        idx.len(),
        &mut residual,
        LmParameters {
            ftol: 1E10 * f64::EPSILON,
            xtol: 1E10 * f64::EPSILON,
            maxfev: 1000,
            ..Default::default()
        },
    )?;
    let mut out = Transformation::zeros();
    out.set_column(3, &x.fixed_rows::<3>(0));
    out.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&cayley2rot(&x.fixed_rows::<3>(3).into_owned()));
    Some(out)
}
