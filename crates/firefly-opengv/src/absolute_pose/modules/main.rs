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

use super::{gp3p, gpnp1, gpnp2, gpnp3, gpnp4, gpnp5, upnp2, upnp4};
use crate::{
    math::{arun, cayley, roots},
    optimization_functor::{LmParameters, NumericalBackend},
};
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3, Vector4};
pub fn p3p_kneip_main(
    bearing: &[Vector3<f64>],
    point: &[Vector3<f64>],
    solutions: &mut Vec<SMatrix<f64, 3, 4>>,
) -> Option<()> {
    if bearing.len() < 3 || point.len() < 3 {
        return None;
    }
    let mut p1 = point[0];
    let mut p2 = point[1];
    let mut p3 = point[2];

    let temp1 = p2 - p1;
    let temp2 = p3 - p1;

    if temp1.cross(&(temp2)).norm() == 0.0 {
        return Some(());
    }

    let mut f1 = bearing[0];
    let mut f2 = bearing[1];
    let mut f3 = bearing[2];

    let mut e1 = f1;
    let mut e3 = f1.cross(&(f2));
    e3 = e3 / e3.norm();
    let mut e2 = e3.cross(&(e1));

    let mut translation = Matrix3::<f64>::zeros();
    translation.row_mut(0).copy_from(&(e1.transpose()));
    translation.row_mut(1).copy_from(&(e2.transpose()));
    translation.row_mut(2).copy_from(&(e3.transpose()));

    f3 = translation * f3;

    if f3[(2, 0)] > 0.0 {
        f1 = bearing[1];
        f2 = bearing[0];
        f3 = bearing[2];

        e1 = f1;
        e3 = f1.cross(&(f2));
        e3 = e3 / e3.norm();
        e2 = e3.cross(&(e1));

        translation.row_mut(0).copy_from(&(e1.transpose()));
        translation.row_mut(1).copy_from(&(e2.transpose()));
        translation.row_mut(2).copy_from(&(e3.transpose()));

        f3 = translation * f3;

        p1 = point[1];
        p2 = point[0];
        p3 = point[2];
    }

    let mut n1 = p2 - p1;
    n1 = n1 / n1.norm();
    let mut n3 = n1.cross(&(p3 - p1));
    n3 = n3 / n3.norm();
    let n2 = n3.cross(&(n1));

    let mut dimension = Matrix3::<f64>::zeros();
    dimension.row_mut(0).copy_from(&(n1.transpose()));
    dimension.row_mut(1).copy_from(&(n2.transpose()));
    dimension.row_mut(2).copy_from(&(n3.transpose()));

    p3 = dimension * (p3 - p1);

    let d_12 = temp1.norm();
    let f_1 = f3[(0, 0)] / f3[(2, 0)];
    let f_2 = f3[(1, 0)] / f3[(2, 0)];
    let p_1 = p3[(0, 0)];
    let p_2 = p3[(1, 0)];

    let cos_beta = f1.dot(&(f2));
    let mut quadratic = 1.0 / (1.0 - (cos_beta).powf(2.0)) - 1.0;

    if cos_beta < 0.0 {
        quadratic = -(quadratic).sqrt();
    } else {
        quadratic = (quadratic).sqrt();
    }

    let factors = kneip_quartic(f_1, f_2, p_1, p_2, d_12, quadratic);
    let real_roots = roots::o4_roots(factors.as_slice());
    if real_roots.len() < 4 {
        return None;
    }

    for &root in real_roots.iter().take(4) {
        let cot_alpha = (-f_1 * p_1 / f_2 - root * p_2 + d_12 * quadratic)
            / (-f_1 * root * p_2 / f_2 + p_1 - d_12);

        let cos_theta = root;
        let sin_theta = (1.0 - (root).powf(2.0)).sqrt();
        let sin_alpha = (1.0 / ((cot_alpha).powf(2.0) + 1.0)).sqrt();
        let mut alpha_cosine = (1.0 - (sin_alpha).powf(2.0)).sqrt();

        if cot_alpha < 0.0 {
            alpha_cosine = -alpha_cosine;
        }

        let mut linear = Vector3::<f64>::zeros();
        linear[(0, 0)] = d_12 * alpha_cosine * (sin_alpha * quadratic + alpha_cosine);
        linear[(1, 0)] = cos_theta * d_12 * sin_alpha * (sin_alpha * quadratic + alpha_cosine);
        linear[(2, 0)] = sin_theta * d_12 * sin_alpha * (sin_alpha * quadratic + alpha_cosine);

        linear = p1 + dimension.transpose() * linear;

        let mut rotation = Matrix3::<f64>::zeros();
        rotation[(0, 0)] = -alpha_cosine;
        rotation[(0, 1)] = -sin_alpha * cos_theta;
        rotation[(0, 2)] = -sin_alpha * sin_theta;
        rotation[(1, 0)] = sin_alpha;
        rotation[(1, 1)] = -alpha_cosine * cos_theta;
        rotation[(1, 2)] = -alpha_cosine * sin_theta;
        rotation[(2, 0)] = 0.0;
        rotation[(2, 1)] = -sin_theta;
        rotation[(2, 2)] = cos_theta;

        rotation = dimension.transpose() * rotation.transpose() * translation;

        let mut solution = SMatrix::<f64, 3, 4>::zeros();
        solution.column_mut(3).copy_from(&linear);
        solution.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);

        solutions.push(solution);
    }
    Some(())
}
pub fn p3p_gao_main(
    bearing: &[Vector3<f64>],
    points: &[Vector3<f64>],
    solutions: &mut Vec<SMatrix<f64, 3, 4>>,
) -> Option<()> {
    if bearing.len() < 3 {
        return None;
    }

    if points.len() < 3 {
        return None;
    }
    let leading = points[0];
    let quadratic = points[1];
    let linear = points[2];

    let mut tempp = leading - quadratic;
    let ab = tempp.norm();
    tempp = quadratic - linear;
    let bc = tempp.norm();
    tempp = leading - linear;
    let ac = tempp.norm();

    let f1 = bearing[0];
    let f2 = bearing[1];
    let f3 = bearing[2];

    let cosalpha = f2.dot(&f3);
    let cosbeta = f1.dot(&f3);
    let cosgamma = f1.dot(&f2);

    let leading = (bc / ab).powf(2.0);
    let quadratic = (ac / ab).powf(2.0);
    let point = 2.0 * cosalpha;
    let quaternion = 2.0 * cosbeta;
    let rotation = 2.0 * cosgamma;

    let a_sq = leading * leading;
    let b_sq = quadratic * quadratic;
    let p_sq = point * point;
    let q_sq = quaternion * quaternion;
    let r_sq = rotation * rotation;

    if (p_sq + q_sq + r_sq - point * quaternion * rotation - 1.0) == 0.0 {
        return Some(());
    }

    let terms = GaoTerms {
        leading,
        quadratic,
        point,
        quaternion,
        rotation,
        a_sq,
        b_sq,
        p_sq,
        q_sq,
        r_sq,
    };
    let Some(factors) = gao_quartic(&terms) else {
        return Some(());
    };
    let x_temp = roots::o4_roots(factors.as_slice());
    if x_temp.len() < 4 {
        return None;
    }
    let mut x = SMatrix::<f64, 4, 1>::zeros();
    for i in 0..4 {
        x[i] = x_temp[i];
    }

    let (y, squared_x_roots) = gao_back_substitution(&terms, &x);
    let mut second_coordinate_squared = SMatrix::<f64, 4, 1>::zeros();
    second_coordinate_squared[0] = (y[0]).powf(2.0);
    second_coordinate_squared[1] = (y[1]).powf(2.0);
    second_coordinate_squared[2] = (y[2]).powf(2.0);
    second_coordinate_squared[3] = (y[3]).powf(2.0);

    let mut temp_xy = SMatrix::<f64, 4, 1>::zeros();
    temp_xy[0] = x[0] * y[0];
    temp_xy[1] = x[1] * y[1];
    temp_xy[2] = x[2] * y[2];
    temp_xy[3] = x[3] * y[3];

    let right_vectors: SMatrix<f64, 4, 1> =
        squared_x_roots + second_coordinate_squared - rotation * temp_xy;

    let mut depth = SMatrix::<f64, 4, 1>::zeros();
    depth[0] = ab / (right_vectors[0]).sqrt();
    depth[1] = ab / (right_vectors[1]).sqrt();
    depth[2] = ab / (right_vectors[2]).sqrt();
    depth[3] = ab / (right_vectors[3]).sqrt();

    let mut scaled_x = SMatrix::<f64, 4, 1>::zeros();
    scaled_x[0] = x[0] * depth[0];
    scaled_x[1] = x[1] * depth[1];
    scaled_x[2] = x[2] * depth[2];
    scaled_x[3] = x[3] * depth[3];

    let mut scaled_y = SMatrix::<f64, 4, 1>::zeros();
    scaled_y[0] = y[0] * depth[0];
    scaled_y[1] = y[1] * depth[1];
    scaled_y[2] = y[2] * depth[2];
    scaled_y[3] = y[3] * depth[3];

    for i in 0..4 {
        let p_cam = vec![scaled_x[i] * f1, scaled_y[i] * f2, depth[i] * f3];

        let solution = arun::arun_complete(points, &p_cam);
        solutions.push(solution);
    }
    Some(())
}
pub fn gp3p_main(
    bearing: &Matrix3<f64>,
    right_vectors: &Matrix3<f64>,
    point: &Matrix3<f64>,
    solutions: &mut Vec<SMatrix<f64, 3, 4>>,
    backend: &dyn NumericalBackend,
) -> Option<()> {
    let mut gram = SMatrix::<f64, 48, 85>::zeros();
    gp3p::init::init(&mut gram, bearing, right_vectors, point);
    gp3p::code::compute(&mut gram);
    let mut matrix = DMatrix::zeros(8, 8);
    matrix
        .view_mut((0, 0), (6, 8))
        .copy_from(&(-gram.fixed_view::<6, 8>(36, 77)));
    matrix[(6, 0)] = 1.0;
    matrix[(7, 6)] = 1.0;
    let (constant, vectors) = backend.eigenpairs(&matrix)?;
    if constant.len() != 8 || vectors.shape() != (8, 8) {
        return None;
    }
    for linear in 0..8 {
        if constant[linear].im < 0.0001 {
            let mut cay = Vector3::zeros();
            let mut dimension = Vector3::zeros();
            for i in 0..3 {
                cay[2 - i] = (vectors[(i + 4, linear)] / vectors[(7, linear)]).re;
                dimension[2 - i] = (vectors[(i + 1, linear)] / vectors[(7, linear)]).re;
            }
            let rotation = cayley::cayley2rot(&cay).transpose();
            let mut center_cam = Vector3::zeros();
            let mut center_world = Vector3::zeros();
            for i in 0..3 {
                let temp = rotation * (dimension[i] * bearing.column(i) + right_vectors.column(i));
                center_cam += temp;
                center_world += point.column(i);
            }
            center_cam /= bearing.ncols() as f64;
            center_world /= bearing.ncols() as f64;
            let translation = center_world - center_cam;
            let mut transformation = SMatrix::<f64, 3, 4>::zeros();
            transformation
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&rotation);
            transformation.column_mut(3).copy_from(&translation);
            solutions.push(transformation);
        }
    }
    Some(())
}
#[must_use]
pub fn gpnp_evaluate(
    solution: &SVector<f64, 12>,
    c: &[Vector3<f64>],
) -> Option<(f64, SMatrix<f64, 3, 4>)> {
    if c.len() < 4 {
        return None;
    }
    let ccam: Vec<Vector3<f64>> = (0..4)
        .map(|i| solution.fixed_rows::<3>(i * 3).into_owned())
        .collect();
    let transformation = arun::arun_complete(c, &ccam);
    let t = transformation.column(3);
    let r = transformation.fixed_view::<3, 3>(0, 0);
    let mut error = 0.0;
    for i in 0..4 {
        let reprojected = r.transpose() * (c[i] - t);
        error += 1.0 - reprojected.dot(&ccam[i]) / (ccam[i].norm() * reprojected.norm());
    }
    Some((error, transformation))
}
pub fn gpnp_optimize(
    a: &SVector<f64, 12>,
    v: &SMatrix<f64, 12, 12>,
    c: &[Vector3<f64>],
    factors: &mut [f64],
    backend: &dyn NumericalBackend,
) -> Option<()> {
    let dim = factors.len();
    if dim > 12 || c.len() < 4 {
        return None;
    }
    let mut x = DVector::from_column_slice(factors);
    backend.levenberg_marquardt(
        &mut x,
        6,
        &mut |x, fvec| {
            if x.len() != dim || fvec.len() != 6 {
                return None;
            }
            let mut solution = *a;
            for i in 0..dim {
                solution += x[i] * v.column(12 - dim + i);
            }
            let ccam: Vec<Vector3<f64>> = (0..4)
                .map(|i| solution.fixed_rows::<3>(3 * i).into_owned())
                .collect();
            let mut index = 0;
            for i in 0..3 {
                for j in i + 1..4 {
                    let diffw = c[i] - c[j];
                    let camera_difference = ccam[i] - ccam[j];
                    fvec[index] = diffw.dot(&diffw) - camera_difference.dot(&camera_difference);
                    index += 1;
                }
            }
            Some(())
        },
        LmParameters {
            ftol: 1.0e10 * f64::EPSILON,
            xtol: 1.0e10 * f64::EPSILON,
            maxfev: 1000,
            ..LmParameters::default()
        },
    )?;
    factors.copy_from_slice(x.as_slice());
    Some(())
}
pub fn gpnp_main(
    a: &SVector<f64, 12>,
    v: &SMatrix<f64, 12, 12>,
    c: &[Vector3<f64>],
    backend: &dyn NumericalBackend,
) -> Option<SMatrix<f64, 3, 4>> {
    if c.len() < 4 {
        return None;
    }

    let vec_5 = v.column(7).into_owned();
    let vec_4 = v.column(8).into_owned();
    let vec_3 = v.column(9).into_owned();
    let vec_2 = v.column(10).into_owned();
    let vec_1 = v.column(11).into_owned();

    let c0 = c[0];
    let c1 = c[0];
    let c2 = c[0];
    let c3 = c[0];

    let mut factors = Vec::new();
    let mut results = Vec::new();
    let solution = *a;
    results.push(gpnp_evaluate(&solution, c)?);

    let mut groebner_matrix1 = SMatrix::<f64, 5, 3>::zeros();
    gpnp1::init::init(&mut groebner_matrix1, a, &vec_1, &c0, &c1, &c2, &c3);
    gpnp1::code::compute(&mut groebner_matrix1);
    factors.push(-groebner_matrix1[(3, 2)] / groebner_matrix1[(3, 1)]);
    results.push(refine_gpnp_candidate(a, v, c, &mut factors, backend)?);

    let mut groebner_matrix2 = SMatrix::<f64, 10, 6>::zeros();
    gpnp2::init::init(
        &mut groebner_matrix2,
        [a, &vec_2, &vec_1],
        [&c0, &c1, &c2, &c3],
    );
    gpnp2::code::compute(&mut groebner_matrix2);
    factors[0] = -groebner_matrix2[(8, 5)] / groebner_matrix2[(8, 4)];
    factors.push(
        -(groebner_matrix2[(7, 4)] * factors[0] + groebner_matrix2[(7, 5)])
            / groebner_matrix2[(7, 3)],
    );
    results.push(refine_gpnp_candidate(a, v, c, &mut factors, backend)?);

    let mut groebner_matrix3 = SMatrix::<f64, 15, 18>::zeros();
    gpnp3::init::init(
        &mut groebner_matrix3,
        [a, &vec_3, &vec_2, &vec_1],
        [&c0, &c1, &c2, &c3],
    );
    gpnp3::code::compute(&mut groebner_matrix3);
    factors[0] = -groebner_matrix3[(13, 17)] / groebner_matrix3[(13, 16)];
    factors[1] = -(groebner_matrix3[(12, 16)] * factors[0] + groebner_matrix3[(12, 17)])
        / groebner_matrix3[(12, 15)];
    factors.push(
        -(groebner_matrix3[(11, 15)] * factors[1]
            + groebner_matrix3[(11, 16)] * factors[0]
            + groebner_matrix3[(11, 17)])
            / groebner_matrix3[(11, 14)],
    );
    results.push(refine_gpnp_candidate(a, v, c, &mut factors, backend)?);

    let mut groebner_matrix4 = SMatrix::<f64, 25, 37>::zeros();
    gpnp4::init::init(
        &mut groebner_matrix4,
        [a, &vec_4, &vec_3, &vec_2, &vec_1],
        [&c0, &c1, &c2, &c3],
    );
    gpnp4::code::compute(&mut groebner_matrix4);
    factors[0] = -groebner_matrix4[(23, 36)] / groebner_matrix4[(23, 35)];
    factors[1] = -(groebner_matrix4[(22, 35)] * factors[0] + groebner_matrix4[(22, 36)])
        / groebner_matrix4[(22, 34)];
    factors[2] = -(groebner_matrix4[(21, 34)] * factors[1]
        + groebner_matrix4[(21, 35)] * factors[0]
        + groebner_matrix4[(21, 36)])
        / groebner_matrix4[(21, 33)];
    factors.push(
        -(groebner_matrix4[(20, 33)] * factors[2]
            + groebner_matrix4[(20, 34)] * factors[1]
            + groebner_matrix4[(20, 35)] * factors[0]
            + groebner_matrix4[(20, 36)])
            / groebner_matrix4[(20, 32)],
    );
    results.push(refine_gpnp_candidate(a, v, c, &mut factors, backend)?);

    seed_gpnp_five_factors(
        a,
        [&vec_5, &vec_4, &vec_3, &vec_2, &vec_1],
        [&c0, &c1, &c2, &c3],
        &mut factors,
    );
    results.push(refine_gpnp_candidate(a, v, c, &mut factors, backend)?);

    let mut smallest_error = results[0].0;
    let mut minimumindex = 0;
    for (i, result) in results.iter().enumerate().take(6).skip(1) {
        if result.0 < smallest_error {
            smallest_error = result.0;
            minimumindex = i;
        }
    }
    Some(results[minimumindex].1)
}
#[must_use]
pub fn upnp_fill_s(q: &Vector4<f64>) -> SVector<f64, 10> {
    SVector::from_row_slice(&[
        q[0] * q[0],
        q[1] * q[1],
        q[2] * q[2],
        q[3] * q[3],
        q[0] * q[1],
        q[0] * q[2],
        q[0] * q[3],
        q[1] * q[2],
        q[1] * q[3],
        q[2] * q[3],
    ])
}
pub fn upnp_main(
    matrix: &SMatrix<f64, 10, 10>,
    linear: &SMatrix<f64, 1, 10>,
    gamma: f64,
    quaternions: &mut Vec<(f64, Vector4<f64>)>,
    backend: &dyn NumericalBackend,
) -> Option<()> {
    let mut action = SMatrix::<f64, 16, 16>::zeros();
    upnp2::setup_action_gj(matrix, linear, gamma, &mut action)?;
    let (_, right_vectors) =
        backend.eigenpairs(&DMatrix::from_column_slice(16, 16, action.as_slice()))?;
    if right_vectors.shape() != (16, 16) {
        return None;
    }
    let double_sol_threshold = 0.000_000_01;
    for i in 0..16 {
        let mut quaternion = Vector4::zeros();
        let mut norm = 0.0;
        for j in 0..4 {
            quaternion[j] = right_vectors[(11 + j, i)].re;
            norm += quaternion[j].powf(2.0);
        }
        norm = norm.sqrt();
        if quaternion[0] < 0.0 {
            norm *= -1.0;
        }
        for j in 0..4 {
            quaternion[j] /= norm;
        }
        let mut already_there = false;
        for (_, old) in quaternions.iter() {
            let diff = quaternion - old;
            if diff.norm() < double_sol_threshold {
                already_there = true;
                break;
            }
        }
        if !already_there {
            let state = upnp_fill_s(&quaternion);
            let value_m = state.transpose() * matrix * state + linear * state * 2.0;
            let value = value_m[0] + gamma;
            let mut idx = 0;
            while idx < quaternions.len() && quaternions[idx].0 < value {
                idx += 1;
            }
            quaternions.insert(idx, (value, quaternion));
        }
    }
    Some(())
}
pub fn upnp_main_sym(
    matrix: &SMatrix<f64, 10, 10>,
    linear: &SMatrix<f64, 1, 10>,
    gamma: f64,
    quaternions: &mut Vec<(f64, Vector4<f64>)>,
    backend: &dyn NumericalBackend,
) -> Option<()> {
    let mut action = SMatrix::<f64, 8, 8>::zeros();
    upnp4::setup_action_sym_gj(matrix, linear, gamma, &mut action)?;
    let (_, right_vectors) =
        backend.eigenpairs(&DMatrix::from_column_slice(8, 8, action.as_slice()))?;
    if right_vectors.shape() != (8, 8) {
        return None;
    }
    for i in 0..8 {
        let mut quaternion = Vector4::zeros();
        quaternion[3] = right_vectors[(7, i)].re;
        quaternion[2] = right_vectors[(6, i)].re;
        quaternion[1] = right_vectors[(5, i)].re;
        quaternion[0] = right_vectors[(4, i)].re;
        let mut norm = 0.0;
        for j in 0..4 {
            norm += quaternion[j].powf(2.0);
        }
        norm = norm.sqrt();
        for j in 0..4 {
            quaternion[j] /= norm;
        }
        let state = upnp_fill_s(&quaternion);
        let value_m = state.transpose() * matrix * state + 2.0 * linear * state;
        let value = value_m[0] + gamma;
        let mut idx = 0;
        while idx < quaternions.len() && quaternions[idx].0 < value {
            idx += 1;
        }
        quaternions.insert(idx, (value, quaternion));
    }
    Some(())
}

fn kneip_quartic(
    f_1: f64,
    f_2: f64,
    p_1: f64,
    p_2: f64,
    d_12: f64,
    quadratic: f64,
) -> SMatrix<f64, 5, 1> {
    let f_1_pw2 = (f_1).powf(2.0);
    let f_2_pw2 = (f_2).powf(2.0);
    let p_1_pw2 = (p_1).powf(2.0);
    let p_1_pw3 = p_1_pw2 * p_1;
    let p_1_pw4 = p_1_pw3 * p_1;
    let p_2_pw2 = (p_2).powf(2.0);
    let p_2_pw3 = p_2_pw2 * p_2;
    let p_2_pw4 = p_2_pw3 * p_2;
    let d_12_pw2 = (d_12).powf(2.0);
    let b_pw2 = (quadratic).powf(2.0);

    let mut factors = SMatrix::<f64, 5, 1>::zeros();

    factors[(0, 0)] = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;

    factors[(1, 0)] = 2.0 * p_2_pw3 * d_12 * quadratic + 2.0 * f_2_pw2 * p_2_pw3 * d_12 * quadratic
        - 2.0 * f_2 * p_2_pw3 * f_1 * d_12;

    factors[(2, 0)] = -f_2_pw2 * p_2_pw2 * p_1_pw2
        - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2
        - f_2_pw2 * p_2_pw2 * d_12_pw2
        + f_2_pw2 * p_2_pw4
        + p_2_pw4 * f_1_pw2
        + 2.0 * p_1 * p_2_pw2 * d_12
        + 2.0 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * quadratic
        - p_2_pw2 * p_1_pw2 * f_1_pw2
        + 2.0 * p_1 * p_2_pw2 * f_2_pw2 * d_12
        - p_2_pw2 * d_12_pw2 * b_pw2
        - 2.0 * p_1_pw2 * p_2_pw2;

    factors[(3, 0)] = 2.0 * p_1_pw2 * p_2 * d_12 * quadratic + 2.0 * f_2 * p_2_pw3 * f_1 * d_12
        - 2.0 * f_2_pw2 * p_2_pw3 * d_12 * quadratic
        - 2.0 * p_1 * p_2 * d_12_pw2 * quadratic;

    factors[(4, 0)] = -2.0 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * quadratic
        + f_2_pw2 * p_2_pw2 * d_12_pw2
        + 2.0 * p_1_pw3 * d_12
        - p_1_pw2 * d_12_pw2
        + f_2_pw2 * p_2_pw2 * p_1_pw2
        - p_1_pw4
        - 2.0 * f_2_pw2 * p_2_pw2 * p_1 * d_12
        + p_2_pw2 * f_1_pw2 * p_1_pw2
        + f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

    factors
}

struct GaoTerms {
    leading: f64,
    quadratic: f64,
    point: f64,
    quaternion: f64,
    rotation: f64,
    a_sq: f64,
    b_sq: f64,
    p_sq: f64,
    q_sq: f64,
    r_sq: f64,
}
fn gao_back_substitution_polynomial(
    terms: &GaoTerms,
    x: &SMatrix<f64, 4, 1>,
    temp_xp2: &SMatrix<f64, 4, 1>,
    temp_xp3: &SMatrix<f64, 4, 1>,
    ones: &SMatrix<f64, 4, 1>,
    r_cb: f64,
) -> SMatrix<f64, 4, 1> {
    (terms.a_sq * r_cb + 2.0 * terms.quadratic * r_cb * terms.leading
        - terms.quadratic * terms.r_sq * r_cb * terms.leading
        - 2.0 * terms.leading * r_cb
        + r_cb
        + terms.b_sq * r_cb
        - 2.0 * r_cb * terms.quadratic)
        * temp_xp3
        + (terms.point * terms.r_sq + terms.point * terms.a_sq * terms.r_sq
            - 2.0 * terms.quadratic * r_cb * terms.quaternion * terms.leading
            + 2.0 * r_cb * terms.quadratic * terms.quaternion
            - 2.0 * r_cb * terms.quaternion
            - 2.0 * terms.point * (terms.leading + terms.quadratic) * terms.r_sq
            + terms.r_sq * terms.r_sq * terms.point * terms.quadratic
            + 4.0 * terms.leading * r_cb * terms.quaternion
            + terms.quadratic * terms.quaternion * terms.leading * r_cb * terms.r_sq
            - 2.0 * r_cb * terms.a_sq * terms.quaternion
            + 2.0 * terms.r_sq * terms.point * terms.quadratic * terms.leading
            + terms.b_sq * terms.r_sq * terms.point
            - terms.r_sq * terms.r_sq * terms.point * terms.b_sq)
            * temp_xp2
        + (r_cb * terms.q_sq
            + terms.r_sq * r_cb * terms.b_sq
            + terms.rotation * terms.p_sq * terms.b_sq
            - 4.0 * terms.leading * r_cb
            - 2.0 * terms.leading * r_cb * terms.q_sq
            + r_cb * terms.q_sq * terms.a_sq
            + 2.0 * terms.a_sq * r_cb
            - 2.0 * terms.b_sq * r_cb
            - 2.0 * terms.p_sq * terms.quadratic * terms.rotation
            + 4.0 * terms.point * terms.leading * terms.r_sq * terms.quaternion
            + 2.0 * terms.leading * terms.p_sq * terms.rotation * terms.quadratic
            - 2.0 * terms.leading * terms.r_sq * terms.quaternion * terms.quadratic * terms.point
            - 2.0 * terms.p_sq * terms.leading * terms.rotation
            + terms.rotation * terms.p_sq
            - terms.quadratic * terms.r_sq * r_cb * terms.leading
            + 2.0 * terms.point * terms.r_sq * terms.quadratic * terms.quaternion
            + terms.rotation * terms.p_sq * terms.a_sq
            - 2.0 * terms.point * terms.quaternion * terms.r_sq
            + 2.0 * r_cb
            - 2.0 * terms.r_sq * terms.point * terms.a_sq * terms.quaternion
            - terms.r_sq * terms.r_sq * terms.quaternion * terms.quadratic * terms.point)
            * x
        + (4.0 * terms.leading * r_cb * terms.quaternion
            + terms.point * terms.r_sq * terms.q_sq
            + 2.0 * terms.p_sq * terms.point * terms.quadratic * terms.leading
            - 4.0 * terms.point * terms.leading * terms.r_sq
            - 2.0 * r_cb * terms.quadratic * terms.quaternion
            - 2.0 * terms.p_sq * terms.quaternion * terms.rotation
            - 2.0 * terms.b_sq * terms.r_sq * terms.point
            + terms.r_sq * terms.r_sq * terms.point * terms.quadratic
            + 2.0 * terms.point * terms.a_sq * terms.r_sq
            - 2.0 * r_cb * terms.a_sq * terms.quaternion
            - 2.0 * terms.p_sq * terms.point * terms.leading
            + terms.p_sq * terms.point * terms.a_sq
            + 2.0 * terms.point * terms.r_sq
            + terms.p_sq * terms.point
            + 2.0 * terms.quadratic * r_cb * terms.quaternion * terms.leading
            + 2.0 * terms.quaternion * terms.p_sq * terms.quadratic * terms.rotation
            + 4.0 * terms.quaternion * terms.leading * terms.rotation * terms.p_sq
            - 2.0 * terms.point * terms.leading * terms.r_sq * terms.q_sq
            - 2.0 * terms.p_sq * terms.a_sq * terms.rotation * terms.quaternion
            + terms.point * terms.a_sq * terms.r_sq * terms.q_sq
            - 2.0 * r_cb * terms.quaternion
            - 2.0 * terms.p_sq * terms.point * terms.quadratic
            + terms.p_sq * terms.point * terms.b_sq
            - 2.0
                * terms.p_sq
                * terms.quadratic
                * terms.rotation
                * terms.quaternion
                * terms.leading)
            * ones
}
fn gao_back_substitution(
    terms: &GaoTerms,
    x: &SMatrix<f64, 4, 1>,
) -> (SMatrix<f64, 4, 1>, SMatrix<f64, 4, 1>) {
    let temp = terms.p_sq * (terms.leading - 1.0 + terms.quadratic)
        + terms.point * terms.quaternion * terms.rotation
        - terms.quaternion * terms.leading * terms.rotation * terms.point
        + (terms.leading - 1.0 - terms.quadratic) * terms.r_sq;
    let b0 = terms.quadratic * temp * temp;

    let r_cb = terms.r_sq * terms.rotation;

    let mut temp_xp2 = SMatrix::<f64, 4, 1>::zeros();
    temp_xp2[0] = x[0] * x[0];
    temp_xp2[1] = x[1] * x[1];
    temp_xp2[2] = x[2] * x[2];
    temp_xp2[3] = x[3] * x[3];
    let mut temp_xp3 = SMatrix::<f64, 4, 1>::zeros();
    temp_xp3[0] = temp_xp2[0] * x[0];
    temp_xp3[1] = temp_xp2[1] * x[1];
    temp_xp3[2] = temp_xp2[2] * x[2];
    temp_xp3[3] = temp_xp2[3] * x[3];

    let mut ones = SMatrix::<f64, 4, 1>::zeros();
    for i in 0..4 {
        ones[i] = 1.0;
    }

    let b1_part1: SMatrix<f64, 4, 1> = (1.0 - terms.leading - terms.quadratic) * temp_xp2
        + (terms.quaternion * terms.leading - terms.quaternion) * x
        + (1.0 - terms.leading + terms.quadratic) * ones;

    let b1_part2 = gao_back_substitution_polynomial(terms, x, &temp_xp2, &temp_xp3, &ones, r_cb);
    let mut b1 = SMatrix::<f64, 4, 1>::zeros();
    b1[0] = b1_part1[0] * b1_part2[0];
    b1[1] = b1_part1[1] * b1_part2[1];
    b1[2] = b1_part1[2] * b1_part2[2];
    b1[3] = b1_part1[3] * b1_part2[3];

    (b1 / b0, temp_xp2)
}

fn refine_gpnp_candidate(
    a: &SVector<f64, 12>,
    v: &SMatrix<f64, 12, 12>,
    c: &[Vector3<f64>],
    factors: &mut [f64],
    backend: &dyn NumericalBackend,
) -> Option<(f64, SMatrix<f64, 3, 4>)> {
    gpnp_optimize(a, v, c, factors, backend)?;
    let mut solution = *a;
    for (i, factor) in factors.iter().enumerate() {
        solution += *factor * v.column(12 - factors.len() + i);
    }
    gpnp_evaluate(&solution, c)
}

fn gao_quartic(terms: &GaoTerms) -> Option<SMatrix<f64, 5, 1>> {
    let mut factors = SMatrix::<f64, 5, 1>::zeros();

    factors[0] = -2.0 * terms.quadratic + terms.b_sq + terms.a_sq + 1.0
        - terms.quadratic * terms.r_sq * terms.leading
        + 2.0 * terms.quadratic * terms.leading
        - 2.0 * terms.leading;

    if factors[0] == 0.0 {
        return None;
    }

    factors[1] = -2.0 * terms.quadratic * terms.quaternion * terms.leading
        - 2.0 * terms.a_sq * terms.quaternion
        + terms.quadratic * terms.r_sq * terms.quaternion * terms.leading
        - 2.0 * terms.quaternion
        + 2.0 * terms.quadratic * terms.quaternion
        + 4.0 * terms.leading * terms.quaternion
        + terms.point * terms.quadratic * terms.rotation
        + terms.quadratic * terms.rotation * terms.point * terms.leading
        - terms.b_sq * terms.rotation * terms.point;
    factors[2] = terms.q_sq + terms.b_sq * terms.r_sq
        - terms.quadratic * terms.p_sq
        - terms.quaternion * terms.point * terms.quadratic * terms.rotation
        + terms.b_sq * terms.p_sq
        - terms.quadratic * terms.r_sq * terms.leading
        + 2.0
        - 2.0 * terms.b_sq
        - terms.leading * terms.quadratic * terms.rotation * terms.point * terms.quaternion
        + 2.0 * terms.a_sq
        - 4.0 * terms.leading
        - 2.0 * terms.q_sq * terms.leading
        + terms.q_sq * terms.a_sq;
    factors[3] = -terms.b_sq * terms.rotation * terms.point
        + terms.quadratic * terms.rotation * terms.point * terms.leading
        - 2.0 * terms.a_sq * terms.quaternion
        + terms.quaternion * terms.p_sq * terms.quadratic
        + 2.0 * terms.quadratic * terms.quaternion * terms.leading
        + 4.0 * terms.leading * terms.quaternion
        + terms.point * terms.quadratic * terms.rotation
        - 2.0 * terms.quadratic * terms.quaternion
        - 2.0 * terms.quaternion;
    factors[4] = 1.0 - 2.0 * terms.leading + 2.0 * terms.quadratic + terms.b_sq
        - terms.quadratic * terms.p_sq
        + terms.a_sq
        - 2.0 * terms.quadratic * terms.leading;

    Some(factors)
}
fn seed_gpnp_five_factors(
    a: &SVector<f64, 12>,
    basis: [&SVector<f64, 12>; 5],
    controls: [&Vector3<f64>; 4],
    factors: &mut Vec<f64>,
) {
    let [vec_5, vec_4, vec_3, vec_2, vec_1] = basis;
    let [c0, c1, c2, c3] = controls;
    let mut groebner_matrix5 = SMatrix::<f64, 44, 80>::zeros();
    gpnp5::init::init(
        &mut groebner_matrix5,
        [a, vec_5, vec_4, vec_3, vec_2, vec_1],
        [c0, c1, c2, c3],
    );
    gpnp5::code::compute(&mut groebner_matrix5);
    factors[0] = -groebner_matrix5[(42, 79)] / groebner_matrix5[(42, 78)];
    factors[1] = -(groebner_matrix5[(41, 78)] * factors[0] + groebner_matrix5[(41, 79)])
        / groebner_matrix5[(41, 77)];
    factors[2] = -(groebner_matrix5[(40, 77)] * factors[1]
        + groebner_matrix5[(40, 78)] * factors[0]
        + groebner_matrix5[(40, 79)])
        / groebner_matrix5[(40, 76)];
    factors[3] = -(groebner_matrix5[(39, 76)] * factors[2]
        + groebner_matrix5[(39, 77)] * factors[1]
        + groebner_matrix5[(39, 78)] * factors[0]
        + groebner_matrix5[(39, 79)])
        / groebner_matrix5[(39, 75)];
    factors.push(
        -(groebner_matrix5[(38, 75)] * factors[3]
            + groebner_matrix5[(38, 76)] * factors[1]
            + groebner_matrix5[(38, 77)] * factors[1]
            + groebner_matrix5[(38, 78)] * factors[0]
            + groebner_matrix5[(38, 79)])
            / groebner_matrix5[(38, 74)],
    );
}
