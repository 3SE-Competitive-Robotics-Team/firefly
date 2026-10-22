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
    relative_pose::{
        modules::main::{self as modules, EigensolverTerms, GeTerms},
        relative_adapter_base::RelativeAdapterBase,
    },
    triangulation::methods as triangulation,
    types::{
        BearingVector, ComplexEssentials, EigensolverOutput, Essential, Essentials, GeOutput,
        Rotation, Rotations, Transformation, Translation,
    },
};
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, Vector3};
/// # Panics
/// Panics if fewer than two indices are provided or an adapter index is invalid.
pub fn twopt(a: &impl RelativeAdapterBase, unrotate: bool, idx: &[usize]) -> Translation {
    assert!(idx.len() > 1);
    twopt_indices(a, unrotate, idx[0], idx[1])
}
pub fn twopt_indices(
    a: &impl RelativeAdapterBase,
    unrotate: bool,
    i: usize,
    j: usize,
) -> Translation {
    let f1 = a.bearing_vector1(i);
    let mut f1p = a.bearing_vector2(i);
    let f2 = a.bearing_vector1(j);
    let mut f2p = a.bearing_vector2(j);
    if unrotate {
        f1p = a.r12() * f1p;
        f2p = a.r12() * f2p;
    }
    let n1 = f1.cross(&f1p);
    let n2 = f2.cross(&f2p);
    let mut t = n1.cross(&n2);
    t = t / t.norm();
    if (f1 - f1p).dot(&t) < 0.0 {
        t = -t;
    }
    t
}
/// # Panics
/// Panics if fewer than two indices are provided or an adapter index is invalid.
pub fn twopt_rotation_only(a: &impl RelativeAdapterBase, idx: &[usize]) -> Rotation {
    assert!(idx.len() > 1);
    let c1 = (a.bearing_vector1(idx[0]) + a.bearing_vector1(idx[1])) / 3.0;
    let c2 = (a.bearing_vector2(idx[0]) + a.bearing_vector2(idx[1])) / 3.0;
    let mut h = Matrix3::zeros();
    for &i in &idx[..2] {
        h += (a.bearing_vector2(i) - c2) * (a.bearing_vector1(i) - c1).transpose();
    }
    arun(&h)
}
/// # Panics
/// Panics if fewer than three indices are provided or an adapter index is invalid.
pub fn rotation_only(a: &impl RelativeAdapterBase, idx: &[usize]) -> Rotation {
    assert!(idx.len() > 2);
    alignment(a, idx, false)
}
fn alignment(a: &impl RelativeAdapterBase, idx: &[usize], unrotate: bool) -> Rotation {
    let mut c1 = Vector3::zeros();
    let mut c2 = Vector3::zeros();
    for &i in idx {
        let (mut f1, mut f2) = (a.bearing_vector1(i), a.bearing_vector2(i));
        if unrotate {
            f1 = a.cam_rotation1(i) * f1;
            f2 = a.cam_rotation2(i) * f2;
        }
        c1 += f1;
        c2 += f2;
    }
    c1 /= idx.len() as f64;
    c2 /= idx.len() as f64;
    let mut h = Matrix3::zeros();
    for &i in idx {
        let (mut f1, mut f2) = (a.bearing_vector1(i), a.bearing_vector2(i));
        if unrotate {
            f1 = a.cam_rotation1(i) * f1;
            f2 = a.cam_rotation2(i) * f2;
        }
        h += (f2 - c2) * (f1 - c1).transpose();
    }
    arun(&h)
}
fn epipolar_matrix(a: &impl RelativeAdapterBase, idx: &[usize]) -> DMatrix<f64> {
    DMatrix::from_fn(idx.len(), 9, |i, j| {
        let f = a.bearing_vector2(idx[i]);
        let fp = a.bearing_vector1(idx[i]);
        f[j % 3] * fp[j / 3]
    })
}
fn nullspace5(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<SMatrix<f64, 9, 4>> {
    assert!(idx.len() > 4);
    let q = epipolar_matrix(a, idx);
    let (_, _, v) = backend.full_svd(&q)?;
    if v.shape() != (9, 9) {
        return None;
    }
    Some(v.fixed_view::<9, 4>(0, 5).into_owned())
}
pub fn fivept_stewenius(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<ComplexEssentials> {
    modules::fivept_stewenius_main(&nullspace5(a, idx, backend)?, backend)
}
pub fn fivept_nister(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Essentials> {
    modules::fivept_nister_main(&nullspace5(a, idx, backend)?, backend)
}
/// # Panics
/// Panics if fewer than five indices are provided or an adapter index is invalid.
pub fn fivept_kneip(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Rotations> {
    assert_eq!(idx.len(), 5);
    let f1 = SMatrix::from_fn(|r, c| a.bearing_vector1(idx[c])[r]);
    let f2 = SMatrix::from_fn(|r, c| a.bearing_vector2(idx[c])[r]);
    modules::fivept_kneip_main(&f1, &f2, backend)
}
/// # Panics
/// Panics if fewer than seven indices are provided or an adapter index is invalid.
pub fn sevenpt(
    leading: &impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Essentials> {
    assert!(idx.len() > 6);
    let (_, _, right_vectors) = backend.full_svd(&epipolar_matrix(leading, idx))?;
    if right_vectors.shape() != (9, 9) {
        return None;
    }
    let f1 = Matrix3::from_fn(|rotation, linear| right_vectors[(3 * rotation + linear, 8)]);
    let f2 = Matrix3::from_fn(|rotation, linear| right_vectors[(3 * rotation + linear, 7)]);
    let eps = 0.000_000_01;
    if f1.determinant().abs() < eps || idx.len() > 7 {
        return Some(vec![f1]);
    }
    let matrix = f2.try_inverse()? * f1;
    let (constant, _) = backend.eigenpairs(&DMatrix::from_column_slice(3, 3, matrix.as_slice()))?;
    if constant.len() != 3 {
        return None;
    }
    let val = [
        constant[0].im.abs(),
        constant[1].im.abs(),
        constant[2].im.abs(),
    ];
    if val.iter().all(|x| *x < eps) {
        Some((0..3).map(|i| f1 - constant[i].re * f2).collect())
    } else {
        let (mut min, mut root_product) = (val[0], 0);
        if val[1] < min {
            min = val[1];
            root_product = 1;
        }
        if val[2] < min {
            root_product = 2;
        }
        Some(vec![f1 - constant[root_product].re * f2])
    }
}
/// # Panics
/// Panics if fewer than eight indices are provided or an adapter index is invalid.
pub fn eightpt(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Essential> {
    assert!(idx.len() > 7);
    let (_, _, v) = backend.full_svd(&epipolar_matrix(a, idx))?;
    if v.shape() != (9, 9) {
        return None;
    }
    let f = Matrix3::from_fn(|r, c| v[(3 * r + c, 8)]);
    let svd = f.svd(true, true);
    let mut s = Matrix3::zeros();
    s[(0, 0)] = svd.singular_values[0];
    s[(1, 1)] = svd.singular_values[1];
    Some(svd.u? * s * svd.v_t?)
}
fn add_central(t: &mut EigensolverTerms, f1: Vector3<f64>, f2: Vector3<f64>, weight: f64) {
    let f = f2 * f2.transpose();
    let reduction_term = weight * f1[0] * f1[0] * f;
    t.xx += reduction_term;
    let reduction_term = weight * f1[1] * f1[1] * f;
    t.yy += reduction_term;
    let reduction_term = weight * f1[2] * f1[2] * f;
    t.zz += reduction_term;
    let reduction_term = weight * f1[0] * f1[1] * f;
    t.xy += reduction_term;
    let reduction_term = weight * f1[1] * f1[2] * f;
    t.yz += reduction_term;
    let reduction_term = weight * f1[2] * f1[0] * f;
    t.zx += reduction_term;
}
pub fn eigensolver(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    use_weights: bool,
    backend: &dyn NumericalBackend,
) -> Option<EigensolverOutput> {
    eigensolver_with_rotation(a, idx, &a.r12(), use_weights, backend)
}
/// # Panics
/// Panics if fewer than five indices are provided or an adapter index is invalid.
pub fn eigensolver_with_rotation(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    start: &Rotation,
    use_weights: bool,
    backend: &dyn NumericalBackend,
) -> Option<EigensolverOutput> {
    assert!(idx.len() > 4);
    let mut norm = 0.0;
    for &i in idx {
        norm += a.weight(i).powf(2.0);
    }
    norm = norm.sqrt();
    let mut t = EigensolverTerms::zeros();
    for &i in idx {
        let weight = if use_weights { a.weight(i) / norm } else { 1.0 };
        add_central(&mut t, a.bearing_vector1(i), a.bearing_vector2(i), weight);
    }
    let mut output = modules::eigensolver_main(&t, start, backend)?;
    let flow = a.bearing_vector1(idx[0]) - output.rotation * a.bearing_vector2(idx[0]);
    if flow.dot(&output.translation) < 0.0 {
        output.translation = -output.translation;
    }
    Some(output)
}
/// # Panics
/// Panics unless exactly six indices are provided or an adapter index is invalid.
pub fn sixpt(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Rotations> {
    assert_eq!(idx.len(), 6);
    let (mut l1, mut l2) = (SMatrix::<f64, 6, 6>::zeros(), SMatrix::<f64, 6, 6>::zeros());
    for (i, &k) in idx.iter().enumerate() {
        let f1 = a.cam_rotation1(k) * a.bearing_vector1(k);
        let f2 = a.cam_rotation2(k) * a.bearing_vector2(k);
        l1.fixed_view_mut::<3, 1>(0, i).copy_from(&f1);
        l2.fixed_view_mut::<3, 1>(0, i).copy_from(&f2);
        l1.fixed_view_mut::<3, 1>(3, i)
            .copy_from(&f1.cross(&a.cam_offset1(k)));
        l2.fixed_view_mut::<3, 1>(3, i)
            .copy_from(&f2.cross(&a.cam_offset2(k)));
    }
    modules::sixpt_main(&l1, &l2, backend)
}
/// # Panics
/// Panics if fewer than six indices are provided or an adapter index is invalid.
pub fn ge(
    a: &impl RelativeAdapterBase,
    idx: &[usize],
    use_weights: bool,
    backend: &dyn NumericalBackend,
) -> Option<GeOutput> {
    assert!(idx.len() > 5);
    let mut norm = 0.0;
    for &i in idx {
        norm += a.weight(i).powf(2.0);
    }
    norm = norm.sqrt();
    let mut t = GeTerms::zeros();
    for &i in idx {
        let weight = if use_weights { a.weight(i) / norm } else { 1.0 };
        let f1 = a.cam_rotation1(i) * a.bearing_vector1(i);
        let f2 = a.cam_rotation2(i) * a.bearing_vector2(i);
        add_central(&mut t.central, f1, f2, weight);
        let t1 = a.cam_offset1(i);
        let t2 = a.cam_offset2(i);
        let cross1 = f1.cross(&t1);
        let cross2 = f2.cross(&t2);
        let f2_19 = SMatrix::<f64, 1, 9>::from_fn(|_, j| f2[j % 3] * cross1[j / 3]);
        let f1_19 = SMatrix::<f64, 1, 9>::from_fn(|_, j| f1[j % 3] * cross2[j / 3]);
        if use_weights {
            let reduction_term = ((weight * f1[0]) * f2) * f1_19;
            t.x1 += reduction_term;
            let reduction_term = ((weight * f1[1]) * f2) * f1_19;
            t.y1 += reduction_term;
            let reduction_term = ((weight * f1[2]) * f2) * f1_19;
            t.z1 += reduction_term;
            let reduction_term = ((weight * f1[0]) * f2) * f2_19;
            t.x2 += reduction_term;
            let reduction_term = ((weight * f1[1]) * f2) * f2_19;
            t.y2 += reduction_term;
            let reduction_term = ((weight * f1[2]) * f2) * f2_19;
            t.z2 += reduction_term;
            let reduction_term = (weight * f1_19.transpose()) * f1_19;
            t.m11 -= reduction_term;
            let reduction_term = (weight * f2_19.transpose()) * f2_19;
            t.m22 -= reduction_term;
            let reduction_term = (weight * f2_19.transpose()) * f1_19;
            t.m12 -= reduction_term;
        } else {
            let reduction_term = (f1[0] * f2) * f1_19;
            t.x1 += reduction_term;
            let reduction_term = (f1[1] * f2) * f1_19;
            t.y1 += reduction_term;
            let reduction_term = (f1[2] * f2) * f1_19;
            t.z1 += reduction_term;
            let reduction_term = f1[0] * f2 * f2_19;
            t.x2 += reduction_term;
            let reduction_term = f1[1] * f2 * f2_19;
            t.y2 += reduction_term;
            let reduction_term = f1[2] * f2 * f2_19;
            t.z2 += reduction_term;
            let reduction_term = f1_19.transpose() * f1_19;
            t.m11 -= reduction_term;
            let reduction_term = f2_19.transpose() * f2_19;
            t.m22 -= reduction_term;
            let reduction_term = f2_19.transpose() * f1_19;
            t.m12 -= reduction_term;
        }
    }
    let start = alignment(a, idx, true);
    modules::ge_main2(&t, &rot2cayley(&start), backend)
}
fn pinv(m: &DMatrix<f64>, n: usize) -> Option<DMatrix<f64>> {
    let svd = m.clone().svd(true, true);
    let sigma = svd.singular_values;
    let tolerance = sigma[0] * n as f64 * f64::EPSILON;
    let mut inverse = DMatrix::zeros(sigma.len(), sigma.len());
    for i in 0..sigma.len() {
        if sigma[i] > tolerance {
            inverse[(i, i)] = 1.0 / sigma[i];
        }
    }
    Some(svd.v_t?.transpose() * inverse * svd.u?.transpose())
}
/// # Panics
/// Panics if fewer than seventeen indices are provided or an adapter index is invalid.
pub fn seventeenpt(
    leading: &impl RelativeAdapterBase,
    idx: &[usize],
    _: &dyn NumericalBackend,
) -> Option<Transformation> {
    assert!(idx.len() > 16);
    let dimension = idx.len();
    let (mut ae, mut ar) = (DMatrix::zeros(dimension, 9), DMatrix::zeros(dimension, 9));
    for (i, &root_product) in idx.iter().enumerate() {
        let d1 = leading.cam_rotation1(root_product) * leading.bearing_vector1(root_product);
        let d2 = leading.cam_rotation2(root_product) * leading.bearing_vector2(root_product);
        let m1 = leading.cam_offset1(root_product).cross(&d1);
        let m2 = leading.cam_offset2(root_product).cross(&d2);
        for j in 0..9 {
            ae[(i, j)] = d2[j / 3] * d1[j % 3];
            ar[(i, j)] = d2[j / 3] * m1[j % 3] + m2[j / 3] * d1[j % 3];
        }
    }
    let arp = pinv(&ar, dimension)?;
    let quadratic = -DMatrix::<f64>::identity(dimension, dimension) + ar * arp;
    let linear = quadratic * ae;
    let svd = linear.svd(false, true);
    let vt = svd.v_t?;
    let quartic_offset = Matrix3::from_fn(|rotation, linear| vt[(8, 3 * rotation + linear)]);
    let svd = quartic_offset.svd(true, true);
    let left_vectors = svd.u?;
    let vt = svd.v_t?;
    let root_weight = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    let mut ra = left_vectors * root_weight * vt;
    let mut rb = left_vectors * root_weight.transpose() * vt;
    if ra.determinant() < 0.0 {
        ra = -ra;
    }
    if rb.determinant() < 0.0 {
        rb = -rb;
    }
    ra.transpose_mut();
    rb.transpose_mut();
    let (mut aa, mut ab) = (DMatrix::zeros(dimension, 3), DMatrix::zeros(dimension, 3));
    let (mut ba, mut bb) = (DVector::zeros(dimension), DVector::zeros(dimension));
    for (i, &root_product) in idx.iter().enumerate() {
        let d1 = leading.cam_rotation1(root_product) * leading.bearing_vector1(root_product);
        let d2 = leading.cam_rotation2(root_product) * leading.bearing_vector2(root_product);
        let v1 = leading.cam_offset1(root_product);
        let v2 = leading.cam_offset2(root_product);
        aa[(i, 0)] =
            d1[2] * d2[0] * ra[(1, 0)] + d1[2] * d2[1] * ra[(1, 1)] + d1[2] * d2[2] * ra[(1, 2)]
                - d1[1] * d2[0] * ra[(2, 0)]
                - d1[1] * d2[1] * ra[(2, 1)]
                - d1[1] * d2[2] * ra[(2, 2)];
        aa[(i, 1)] =
            d1[0] * d2[0] * ra[(2, 0)] + d1[0] * d2[1] * ra[(2, 1)] + d1[0] * d2[2] * ra[(2, 2)]
                - d1[2] * d2[0] * ra[(0, 0)]
                - d1[2] * d2[1] * ra[(0, 1)]
                - d1[2] * d2[2] * ra[(0, 2)];
        aa[(i, 2)] =
            d1[1] * d2[0] * ra[(0, 0)] + d1[1] * d2[1] * ra[(0, 1)] + d1[1] * d2[2] * ra[(0, 2)]
                - d1[0] * d2[0] * ra[(1, 0)]
                - d1[0] * d2[1] * ra[(1, 1)]
                - d1[0] * d2[2] * ra[(1, 2)];

        ab[(i, 0)] =
            d1[2] * d2[0] * rb[(1, 0)] + d1[2] * d2[1] * rb[(1, 1)] + d1[2] * d2[2] * rb[(1, 2)]
                - d1[1] * d2[0] * rb[(2, 0)]
                - d1[1] * d2[1] * rb[(2, 1)]
                - d1[1] * d2[2] * rb[(2, 2)];
        ab[(i, 1)] =
            d1[0] * d2[0] * rb[(2, 0)] + d1[0] * d2[1] * rb[(2, 1)] + d1[0] * d2[2] * rb[(2, 2)]
                - d1[2] * d2[0] * rb[(0, 0)]
                - d1[2] * d2[1] * rb[(0, 1)]
                - d1[2] * d2[2] * rb[(0, 2)];
        ab[(i, 2)] =
            d1[1] * d2[0] * rb[(0, 0)] + d1[1] * d2[1] * rb[(0, 1)] + d1[1] * d2[2] * rb[(0, 2)]
                - d1[0] * d2[0] * rb[(1, 0)]
                - d1[0] * d2[1] * rb[(1, 1)]
                - d1[0] * d2[2] * rb[(1, 2)];

        let temp1 = v1.cross(&d1);
        let temp2 = v2.cross(&d2);
        ba[i] = -d1.dot(&(ra * temp2)) - temp1.dot(&(ra * d2));
        bb[i] = -d1.dot(&(rb * temp2)) - temp1.dot(&(rb * d2));
    }
    let ta = pinv(&aa, dimension)? * &ba;
    let tb = pinv(&ab, dimension)? * &bb;
    let fita = aa * &ta - ba;
    let translation_residual = ab * &tb - bb;
    let mut out = Transformation::zeros();
    if fita.norm() < translation_residual.norm() {
        out.fixed_view_mut::<3, 3>(0, 0).copy_from(&ra);
        out.column_mut(3).copy_from(&ta);
    } else {
        out.fixed_view_mut::<3, 3>(0, 0).copy_from(&rb);
        out.column_mut(3).copy_from(&tb);
    }
    Some(out)
}
struct TriangulationInput {
    t: Translation,
    r: Rotation,
    f1: BearingVector,
    f2: BearingVector,
}
impl triangulation::RelativeAdapterBase for TriangulationInput {
    fn t12(&self) -> Translation {
        self.t
    }
    fn r12(&self) -> Rotation {
        self.r
    }
    fn bearing_vector1(&self, _: usize) -> BearingVector {
        self.f1
    }
    fn bearing_vector2(&self, _: usize) -> BearingVector {
        self.f2
    }
}
pub fn optimize_nonlinear(
    a: &mut impl RelativeAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    let mut x = DVector::zeros(6);
    x.fixed_rows_mut::<3>(0).copy_from(&a.t12());
    x.fixed_rows_mut::<3>(3).copy_from(&rot2cayley(&a.r12()));
    let mut residual = |x: &DVector<f64>, out: &mut DVector<f64>| {
        if x.len() != 6 || out.len() != idx.len() {
            return None;
        }
        let translation = x.fixed_rows::<3>(0).into_owned();
        let rotation = cayley2rot(&x.fixed_rows::<3>(3).into_owned());
        for (i, &k) in idx.iter().enumerate() {
            let c1 = a.cam_offset1(k);
            let r1 = a.cam_rotation1(k);
            let c2 = a.cam_offset2(k);
            let r2 = a.cam_rotation2(k);
            let direct_t = r1.transpose() * ((translation - c1) + rotation * c2);
            let direct_r = r1.transpose() * rotation * r2;
            a.set_t12(direct_t);
            a.set_r12(direct_r);
            let f1 = a.bearing_vector1(k);
            let f2 = a.bearing_vector2(k);
            let p = triangulation::triangulate2(
                &TriangulationInput {
                    t: direct_t,
                    r: direct_r,
                    f1,
                    f2,
                },
                0,
            );
            let mut reproj1 = p;
            let mut inverse_solution = Transformation::zeros();
            inverse_solution
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&direct_r.transpose());
            let inverse_t = -inverse_solution.fixed_view::<3, 3>(0, 0) * direct_t;
            inverse_solution.set_column(3, &inverse_t);
            let p_hom = nalgebra::Vector4::new(p[0], p[1], p[2], 1.0);
            let mut reproj2 = inverse_solution * p_hom;
            reproj1 = reproj1 / reproj1.norm();
            reproj2 = reproj2 / reproj2.norm();
            let err1 = 1.0 - f1.dot(&reproj1);
            let err2 = 1.0 - f2.dot(&reproj2);
            out[i] = 1.0 * (err1 + err2);
        }
        Some(())
    };
    backend.levenberg_marquardt(
        &mut x,
        idx.len(),
        &mut residual,
        LmParameters {
            ftol: 1E1 * f64::EPSILON,
            xtol: 1E1 * f64::EPSILON,
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
