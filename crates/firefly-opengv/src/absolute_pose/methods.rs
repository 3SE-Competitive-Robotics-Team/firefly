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

use crate::types::Transformation;
use crate::{
    absolute_pose::{
        absolute_adapter_base::AbsoluteAdapterBase,
        modules::{epnp::Epnp, main as modules},
    },
    math::{cayley, quaternion},
    optimization_functor::{LmParameters, NumericalBackend},
};
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3};

fn valid(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
    minimum: usize,
) -> Option<()> {
    if indices.len() < minimum
        || indices
            .iter()
            .any(|&i| i >= adapter.number_correspondences())
    {
        None
    } else {
        Some(())
    }
}
fn transform(r: &Matrix3<f64>, t: &Vector3<f64>) -> Transformation {
    let mut result = Transformation::zeros();
    result.fixed_view_mut::<3, 3>(0, 0).copy_from(r);
    result.column_mut(3).copy_from(t);
    result
}
pub fn p2p(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
) -> Option<Vector3<f64>> {
    valid(adapter, indices.get(..2)?, 2)?;
    let (i0, i1) = (indices[0], indices[1]);
    let e1 = adapter.bearing_vector(i0);
    let mut e3 = adapter.bearing_vector(i1);
    e3 = e1.cross(&e3);
    e3 = e3 / e3.norm();
    let e2 = e3.cross(&e1);
    let t = Matrix3::from_rows(&[e1.transpose(), e2.transpose(), e3.transpose()]);
    let mut n1 = adapter.point(i1) - adapter.point(i0);
    n1 = n1 / n1.norm();
    let mut n3 = Vector3::zeros();
    if n1[0].abs() > n1[1].abs() && n1[0].abs() > n1[2].abs() {
        n3[1] = 1.0;
        n3[2] = 0.0;
        n3[0] = -n1[1] / n1[0];
    } else if n1[1].abs() > n1[0].abs() && n1[1].abs() > n1[2].abs() {
        n3[2] = 1.0;
        n3[0] = 0.0;
        n3[1] = -n1[2] / n1[1];
    } else {
        n3[0] = 1.0;
        n3[1] = 0.0;
        n3[2] = -n1[0] / n1[2];
    }
    n3 = n3 / n3.norm();
    let n2 = n3.cross(&n1);
    let n = Matrix3::from_rows(&[n1.transpose(), n2.transpose(), n3.transpose()]);
    let q = t * adapter.r().transpose() * n.transpose();
    let temp1 = adapter.point(i1) - adapter.point(i0);
    let d12 = temp1.norm();
    let temp2 = adapter.bearing_vector(i1);
    let cos_beta = e1.dot(&temp2);
    let mut b = 1.0 / (1.0 - cos_beta.powf(2.0)) - 1.0;
    b = if cos_beta < 0.0 { -b.sqrt() } else { b.sqrt() };
    let temp3 = d12 * (q[(1, 0)] * b - q[(0, 0)]);
    let mut solution = -temp3 * q.row(0).transpose();
    solution = adapter.point(i0) + n.transpose() * solution;
    if solution[0].is_nan() || solution[1].is_nan() || solution[2].is_nan() {
        solution = Vector3::zeros();
    }
    Some(solution)
}
pub fn p3p_kneip(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
) -> Option<Vec<Transformation>> {
    valid(adapter, indices.get(..3)?, 3)?;
    let f: Vec<_> = indices[..3]
        .iter()
        .map(|&i| adapter.bearing_vector(i))
        .collect();
    let p: Vec<_> = indices[..3].iter().map(|&i| adapter.point(i)).collect();
    let mut solutions = vec![];
    modules::p3p_kneip_main(&f, &p, &mut solutions)?;
    Some(solutions)
}
pub fn p3p_gao(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
) -> Option<Vec<Transformation>> {
    valid(adapter, indices.get(..3)?, 3)?;
    let f: Vec<_> = indices[..3]
        .iter()
        .map(|&i| adapter.bearing_vector(i))
        .collect();
    let p: Vec<_> = indices[..3].iter().map(|&i| adapter.point(i)).collect();
    let mut solutions = vec![];
    modules::p3p_gao_main(&f, &p, &mut solutions)?;
    Some(solutions)
}
pub fn gp3p(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Vec<Transformation>> {
    valid(adapter, indices.get(..3)?, 3)?;
    let mut bearing = Matrix3::zeros();
    let mut right_vectors = Matrix3::zeros();
    let mut point = Matrix3::zeros();
    for (i, &j) in indices.iter().take(3).enumerate() {
        bearing.column_mut(i).copy_from(&adapter.bearing_vector(j));
        let rotation = adapter.cam_rotation(j);
        let fi = rotation * bearing.column(i);
        bearing.column_mut(i).copy_from(&fi);
        right_vectors
            .column_mut(i)
            .copy_from(&adapter.cam_offset(j));
        point.column_mut(i).copy_from(&adapter.point(j));
    }
    let mut solutions = vec![];
    modules::gp3p_main(&bearing, &right_vectors, &point, &mut solutions, backend)?;
    Some(solutions)
}
pub fn epnp(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
) -> Option<Transformation> {
    valid(adapter, indices, 6)?;
    let mut pnp = Epnp::new();
    pnp.set_maximum_number_of_correspondences(indices.len());
    pnp.reset_correspondences();
    for &i in indices {
        let p = adapter.point(i);
        let f = adapter.bearing_vector(i);
        pnp.add_correspondence(p[0], p[1], p[2], f[0], f[1], f[2])?;
    }
    let (_, r, t) = pnp.compute_pose()?;
    let rotation = r.transpose();
    let translation = -rotation * t;
    Some(transform(&rotation, &translation))
}
pub fn gpnp(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    valid(adapter, indices, 6)?;
    let mut c0 = Vector3::zeros();
    for &i in indices {
        c0 += adapter.point(i);
    }
    c0 /= indices.len() as f64;
    let point = DMatrix::from_fn(3, indices.len(), |rotation_value, i| {
        adapter.point(indices[i])[rotation_value] - c0[rotation_value]
    });
    let svd = point.clone().svd(true, true);
    let left_vectors = svd.u?;
    let mut linear = vec![c0];
    for j in 0..3 {
        linear.push(c0 + 15.0 * left_vectors.column(j));
    }
    let e1 = linear[1] - c0;
    let e1dote1 = e1.dot(&e1);
    let e2 = linear[2] - c0;
    let e2dote2 = e2.dot(&e2);
    let e3 = linear[3] - c0;
    let e3dote3 = e3.dot(&e3);
    let mut weights = DMatrix::zeros(4, indices.len());
    for i in 0..indices.len() {
        let temp = point.column(i);
        weights[(1, i)] = temp.dot(&e1) / e1dote1;
        weights[(2, i)] = temp.dot(&e2) / e2dote2;
        weights[(3, i)] = temp.dot(&e3) / e3dote3;
        weights[(0, i)] = 1.0 - (weights[(1, i)] + weights[(2, i)] + weights[(3, i)]);
    }
    let mut a_matrix = DMatrix::zeros(2 * indices.len(), 12);
    let mut quadratic = DVector::zeros(2 * indices.len());
    for i in 0..indices.len() {
        let offset = adapter.cam_offset(indices[i]);
        let rotation = adapter.cam_rotation(indices[i]);
        let bearing = rotation * adapter.bearing_vector(indices[i]);
        for j in 0..4 {
            a_matrix[(2 * i, 3 * j)] = weights[(j, i)] * bearing[2];
            a_matrix[(2 * i, 3 * j + 2)] = -weights[(j, i)] * bearing[0];
            a_matrix[(2 * i + 1, 3 * j + 1)] = weights[(j, i)] * bearing[2];
            a_matrix[(2 * i + 1, 3 * j + 2)] = -weights[(j, i)] * bearing[1];
        }
        quadratic[2 * i] = bearing[2] * offset[0] - bearing[0] * offset[2];
        quadratic[2 * i + 1] = bearing[2] * offset[1] - bearing[1] * offset[2];
    }
    let svd2 = a_matrix.svd(true, true);
    let mut invd = DMatrix::zeros(12, 12);
    for i in 0..12 {
        invd[(i, i)] = if svd2.singular_values[i] > 1.0e-6 {
            1.0 / svd2.singular_values[i]
        } else {
            0.0
        }
    }
    let right_vectors = svd2.v_t?.transpose();
    let pinva = &right_vectors * invd * svd2.u?.transpose();
    let leading: SVector<f64, 12> = SVector::from_column_slice((pinva * quadratic).as_slice());
    let v_fixed = SMatrix::<f64, 12, 12>::from_column_slice(right_vectors.as_slice());
    modules::gpnp_main(&leading, &v_fixed, &linear, backend)
}
#[must_use]
pub fn fill3x10(x: &Vector3<f64>) -> SMatrix<f64, 3, 10> {
    let (x1, x2, x3) = (x[0], x[1], x[2]);
    SMatrix::from_row_slice(&[
        x1,
        x1,
        -x1,
        -x1,
        0.0,
        2.0 * x3,
        -2.0 * x2,
        2.0 * x2,
        2.0 * x3,
        0.0,
        x2,
        -x2,
        x2,
        -x2,
        -2.0 * x3,
        0.0,
        2.0 * x1,
        2.0 * x1,
        0.0,
        2.0 * x3,
        x3,
        -x3,
        -x3,
        x3,
        2.0 * x2,
        -2.0 * x1,
        0.0,
        0.0,
        2.0 * x1,
        2.0 * x2,
    ])
}
#[must_use]
pub fn f(m: &SMatrix<f64, 10, 10>, c: &SMatrix<f64, 1, 10>, _: f64) -> Vector3<f64> {
    let mut f = Vector3::<f64>::zeros();
    f[0] = 2.0 * m[(0, 4)] + 2.0 * c[(0, 4)];
    f[1] = 2.0 * m[(0, 5)] + 2.0 * c[(0, 5)];
    f[2] = 2.0 * m[(0, 6)] + 2.0 * c[(0, 6)];
    f
}
#[must_use]
pub fn jac(m: &SMatrix<f64, 10, 10>, c: &SMatrix<f64, 1, 10>, _: f64) -> Matrix3<f64> {
    let mut jac = Matrix3::<f64>::zeros();
    jac[(0, 0)] =
        2.0 * m[(4, 4)] + 4.0 * m[(0, 1)] - 4.0 * m[(0, 0)] + 4.0 * c[(0, 1)] - 4.0 * c[(0, 0)];
    jac[(0, 1)] = 2.0 * m[(5, 4)] + 2.0 * m[(0, 7)] + 2.0 * c[(0, 7)];
    jac[(0, 2)] = 2.0 * m[(6, 4)] + 2.0 * m[(0, 8)] + 2.0 * c[(0, 8)];
    jac[(1, 0)] = 2.0 * m[(4, 5)] + 2.0 * m[(0, 7)] + 2.0 * c[(0, 7)];
    jac[(1, 1)] =
        2.0 * m[(5, 5)] + 4.0 * m[(0, 2)] - 4.0 * m[(0, 0)] + 4.0 * c[(0, 2)] - 4.0 * c[(0, 0)];
    jac[(1, 2)] = 2.0 * m[(6, 5)] + 2.0 * m[(0, 9)] + 2.0 * c[(0, 9)];
    jac[(2, 0)] = 2.0 * m[(4, 6)] + 2.0 * m[(0, 8)] + 2.0 * c[(0, 8)];
    jac[(2, 1)] = 2.0 * m[(5, 6)] + 2.0 * m[(0, 9)] + 2.0 * c[(0, 9)];
    jac[(2, 2)] =
        2.0 * m[(6, 6)] + 4.0 * m[(0, 3)] - 4.0 * m[(0, 0)] + 4.0 * c[(0, 3)] - 4.0 * c[(0, 0)];
    jac
}
pub fn upnp(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Vec<Transformation>> {
    valid(adapter, indices, 3)?;
    let mut bearing_moment = Matrix3::zeros();
    for &i in indices {
        let bearing = adapter.cam_rotation(i) * adapter.bearing_vector(i);
        bearing_moment += bearing * bearing.transpose();
    }
    let h_inv = indices.len() as f64 * Matrix3::identity() - bearing_moment;
    let inverse_projection = h_inv.try_inverse()?;
    let mut projection_i = SMatrix::<f64, 3, 10>::zeros();
    let mut projection_j = Vector3::zeros();
    for &i in indices {
        let bearing = adapter.cam_rotation(i) * adapter.bearing_vector(i);
        let vk = inverse_projection * (bearing * bearing.transpose() - Matrix3::identity());
        let point = adapter.point(i);
        let right_vectors = adapter.cam_offset(i);
        let phi = fill3x10(&point);
        projection_i += vk * phi;
        projection_j += vk * right_vectors;
    }
    let mut matrix = SMatrix::<f64, 10, 10>::zeros();
    let mut linear = SMatrix::<f64, 1, 10>::zeros();
    let mut gamma = 0.0;
    for &i in indices {
        let bearing = adapter.cam_rotation(i) * adapter.bearing_vector(i);
        let right_vectors = adapter.cam_offset(i);
        let point = adapter.point(i);
        let phi = fill3x10(&point);
        let temp = bearing * bearing.transpose() - Matrix3::identity();
        let ai = temp * (phi + projection_i);
        let bi = -temp * (right_vectors + projection_j);
        matrix += ai.transpose() * ai;
        linear += bi.transpose() * ai;
        gamma += (bi.transpose() * bi)[0];
    }
    let mut quaternions1 = vec![];
    if indices.len() > 4 {
        modules::upnp_main_sym(&matrix, &linear, gamma, &mut quaternions1, backend)?;
    } else {
        modules::upnp_main(&matrix, &linear, gamma, &mut quaternions1, backend)?;
    }
    let mut transformations = vec![];
    let mut quaternions2 = vec![];
    for &candidate in &quaternions1 {
        let rinv = quaternion::quaternion2rot(&candidate.1);
        let state = modules::upnp_fill_s(&candidate.1);
        let tinv = projection_i * state - projection_j;
        if transformations.is_empty() {
            transformations.push(inverse_pose(&rinv, &tinv));
        }
        let count_negative = count_negative_depths(adapter, indices, &rinv, &tinv);
        if f64::from(count_negative) < (0.2 * indices.len() as f64 + 0.5).floor() {
            quaternions2.push(candidate);
        }
    }
    if quaternions2.is_empty() {
        return Some(transformations);
    }
    transformations.clear();
    for candidate in &quaternions2 {
        let mut i_cay = SMatrix::<f64, 3, 10>::zeros();
        let rinv = quaternion::quaternion2rot(&candidate.1);
        for &i in indices {
            let bearing = adapter.cam_rotation(i) * adapter.bearing_vector(i);
            let vk = inverse_projection * (bearing * bearing.transpose() - Matrix3::identity());
            let point = rinv * adapter.point(i);
            let phi = fill3x10(&point);
            i_cay += vk * phi;
        }
        let mut m_cay = SMatrix::<f64, 10, 10>::zeros();
        let mut c_cay = SMatrix::<f64, 1, 10>::zeros();
        let mut gamma_cay = 0.0;
        for &i in indices {
            let bearing = adapter.cam_rotation(i) * adapter.bearing_vector(i);
            let right_vectors = adapter.cam_offset(i);
            let point = rinv * adapter.point(i);
            let phi = fill3x10(&point);
            let temp = bearing * bearing.transpose() - Matrix3::identity();
            let ai = temp * (phi + i_cay);
            let bi = -temp * (right_vectors + projection_j);
            m_cay += ai.transpose() * ai;
            c_cay += bi.transpose() * ai;
            gamma_cay += (bi.transpose() * bi)[0];
        }
        let val = f(&m_cay, &c_cay, gamma_cay);
        let jacobian = jac(&m_cay, &c_cay, gamma_cay);
        let cay = -jacobian.try_inverse()? * val;
        let rinv2 = cayley::cayley2rot(&cay) * rinv;
        let quaternion_value = quaternion::rot2quaternion(&rinv2);
        let state = modules::upnp_fill_s(&quaternion_value);
        let tinv = projection_i * state - projection_j;
        transformations.push(inverse_pose(&rinv2, &tinv));
    }
    if transformations.is_empty() {
        let quaternion_value = quaternions2[0].1;
        let state = modules::upnp_fill_s(&quaternion_value);
        let tinv = projection_i * state - projection_j;
        let rinv = quaternion::quaternion2rot(&quaternion_value);
        transformations.push(inverse_pose(&rinv, &tinv));
    }
    Some(transformations)
}
pub fn optimize_nonlinear(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    valid(adapter, indices, 0)?;
    let mut x = DVector::zeros(6);
    x.fixed_rows_mut::<3>(0).copy_from(&adapter.t());
    x.fixed_rows_mut::<3>(3)
        .copy_from(&cayley::rot2cayley(&adapter.r()));
    backend.levenberg_marquardt(
        &mut x,
        indices.len(),
        &mut |x, fvec| {
            if x.len() != 6 || fvec.len() != indices.len() {
                return None;
            }
            let translation = x.fixed_rows::<3>(0).into_owned();
            let cay = x.fixed_rows::<3>(3).into_owned();
            let rotation = cayley::cayley2rot(&cay);
            let inverse_r = rotation.transpose();
            let inverse_t = -inverse_r * translation;
            let mut p_hom = nalgebra::Vector4::zeros();
            p_hom[3] = 1.0;
            let inverse = transform(&inverse_r, &inverse_t);
            for (i, &index) in indices.iter().enumerate() {
                p_hom
                    .fixed_rows_mut::<3>(0)
                    .copy_from(&adapter.point(index));
                let body_reprojection = inverse * p_hom;
                let mut reprojection = adapter.cam_rotation(index).transpose()
                    * (body_reprojection - adapter.cam_offset(index));
                reprojection = reprojection / reprojection.norm();
                let factor = 1.0;
                fvec[i] = factor * (1.0 - reprojection.dot(&adapter.bearing_vector(index)));
            }
            Some(())
        },
        LmParameters {
            ftol: 1.0e1 * f64::EPSILON,
            xtol: 1.0e1 * f64::EPSILON,
            maxfev: 1000,
            ..LmParameters::default()
        },
    )?;
    Some(transform(
        &cayley::cayley2rot(&x.fixed_rows::<3>(3).into_owned()),
        &x.fixed_rows::<3>(0).into_owned(),
    ))
}
pub fn epnp_all(adapter: &(impl AbsoluteAdapterBase + ?Sized)) -> Option<Transformation> {
    epnp(
        adapter,
        &(0..adapter.number_correspondences()).collect::<Vec<_>>(),
    )
}
pub fn gpnp_all(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    gpnp(
        adapter,
        &(0..adapter.number_correspondences()).collect::<Vec<_>>(),
        backend,
    )
}
pub fn upnp_all(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    backend: &dyn NumericalBackend,
) -> Option<Vec<Transformation>> {
    upnp(
        adapter,
        &(0..adapter.number_correspondences()).collect::<Vec<_>>(),
        backend,
    )
}
pub fn optimize_nonlinear_all(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    optimize_nonlinear(
        adapter,
        &(0..adapter.number_correspondences()).collect::<Vec<_>>(),
        backend,
    )
}

fn count_negative_depths(
    adapter: &(impl AbsoluteAdapterBase + ?Sized),
    indices: &[usize],
    rinv: &Matrix3<f64>,
    tinv: &Vector3<f64>,
) -> i32 {
    let mut count_negative = 0;
    for &j in indices {
        let bearing = adapter.cam_rotation(j) * adapter.bearing_vector(j);
        let point = adapter.point(j);
        let right_vectors = adapter.cam_offset(j);
        let p_est = rinv * point + tinv - right_vectors;
        if p_est.dot(&bearing) < 0.0 {
            count_negative += 1;
        }
    }
    count_negative
}

fn inverse_pose(rinv: &Matrix3<f64>, tinv: &Vector3<f64>) -> Transformation {
    let r = rinv.transpose();
    transform(&r, &(-r * tinv))
}
