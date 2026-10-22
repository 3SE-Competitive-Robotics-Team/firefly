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

const NEGATIVE_UNIT: f64 = -1.0;

use crate::math::cayley::cayley2rot_reduced;
use nalgebra::{Matrix3, Matrix4, SMatrix, Vector3, Vector4};

/// Precomputed generalized bearing and offset moments.
pub struct GeneralizedMoments<'a> {
    pub xx_f: &'a Matrix3<f64>,
    pub yy_f: &'a Matrix3<f64>,
    pub zz_f: &'a Matrix3<f64>,
    pub xy_f: &'a Matrix3<f64>,
    pub yz_f: &'a Matrix3<f64>,
    pub zx_f: &'a Matrix3<f64>,
    pub x1_p: &'a SMatrix<f64, 3, 9>,
    pub y1_p: &'a SMatrix<f64, 3, 9>,
    pub z1_p: &'a SMatrix<f64, 3, 9>,
    pub x2_p: &'a SMatrix<f64, 3, 9>,
    pub y2_p: &'a SMatrix<f64, 3, 9>,
    pub z2_p: &'a SMatrix<f64, 3, 9>,
    pub m11_p: &'a SMatrix<f64, 9, 9>,
    pub m12_p: &'a SMatrix<f64, 9, 9>,
    pub m22_p: &'a SMatrix<f64, 9, 9>,
}
fn pow(x: f64, y: f64) -> f64 {
    x.powf(y)
}
fn sqrt(x: f64) -> f64 {
    x.sqrt()
}
fn acos(x: f64) -> f64 {
    x.acos()
}
fn cos(x: f64) -> f64 {
    x.cos()
}
/// Writes eigenvalue entries 2 and 3, preserving the other output entries.
pub fn get_ev(moments: &GeneralizedMoments<'_>, cayley: &Vector3<f64>, roots: &mut Vector4<f64>) {
    let gram: Matrix4<f64> = compose_g(moments, cayley);

    let g01_2: f64 = gram[(0, 1)] * gram[(0, 1)];
    let g02_2: f64 = gram[(0, 2)] * gram[(0, 2)];
    let g03_2: f64 = gram[(0, 3)] * gram[(0, 3)];

    let g12_2: f64 = gram[(1, 2)] * gram[(1, 2)];
    let g13_2: f64 = gram[(1, 3)] * gram[(1, 3)];

    let g23_2: f64 = gram[(2, 3)] * gram[(2, 3)];

    let quadratic: f64 = -gram[(3, 3)] - gram[(2, 2)] - gram[(1, 1)] - gram[(0, 0)];
    let linear: f64 = -g23_2 + gram[(2, 2)] * gram[(3, 3)] - g13_2 - g12_2
        + gram[(1, 1)] * gram[(3, 3)]
        + gram[(1, 1)] * gram[(2, 2)]
        - g03_2
        - g02_2
        - g01_2
        + gram[(0, 0)] * gram[(3, 3)]
        + gram[(0, 0)] * gram[(2, 2)]
        + gram[(0, 0)] * gram[(1, 1)];
    let constant: f64 = g13_2 * gram[(2, 2)] - 2.0 * gram[(1, 2)] * gram[(1, 3)] * gram[(2, 3)]
        + g12_2 * gram[(3, 3)]
        + gram[(1, 1)] * g23_2
        - gram[(1, 1)] * gram[(2, 2)] * gram[(3, 3)]
        + g03_2 * gram[(2, 2)]
        + g03_2 * gram[(1, 1)]
        - 2.0 * gram[(0, 2)] * gram[(0, 3)] * gram[(2, 3)]
        + g02_2 * gram[(3, 3)]
        + g02_2 * gram[(1, 1)]
        - 2.0 * gram[(0, 1)] * gram[(0, 3)] * gram[(1, 3)]
        - 2.0 * gram[(0, 1)] * gram[(0, 2)] * gram[(1, 2)]
        + g01_2 * gram[(3, 3)]
        + g01_2 * gram[(2, 2)]
        + gram[(0, 0)] * g23_2
        - gram[(0, 0)] * gram[(2, 2)] * gram[(3, 3)]
        + gram[(0, 0)] * g13_2
        + gram[(0, 0)] * g12_2
        - gram[(0, 0)] * gram[(1, 1)] * gram[(3, 3)]
        - gram[(0, 0)] * gram[(1, 1)] * gram[(2, 2)];
    let quartic_offset: f64 = g03_2 * g12_2
        - g03_2 * gram[(1, 1)] * gram[(2, 2)]
        - 2.0 * gram[(0, 2)] * gram[(0, 3)] * gram[(1, 2)] * gram[(1, 3)]
        + 2.0 * gram[(0, 2)] * gram[(0, 3)] * gram[(1, 1)] * gram[(2, 3)]
        + g02_2 * g13_2
        - g02_2 * gram[(1, 1)] * gram[(3, 3)]
        + 2.0 * gram[(0, 1)] * gram[(0, 3)] * gram[(1, 3)] * gram[(2, 2)]
        - 2.0 * gram[(0, 1)] * gram[(0, 3)] * gram[(1, 2)] * gram[(2, 3)]
        - 2.0 * gram[(0, 1)] * gram[(0, 2)] * gram[(1, 3)] * gram[(2, 3)]
        + 2.0 * gram[(0, 1)] * gram[(0, 2)] * gram[(1, 2)] * gram[(3, 3)]
        + g01_2 * g23_2
        - g01_2 * gram[(2, 2)] * gram[(3, 3)]
        - gram[(0, 0)] * g13_2 * gram[(2, 2)]
        + 2.0 * gram[(0, 0)] * gram[(1, 2)] * gram[(1, 3)] * gram[(2, 3)]
        - gram[(0, 0)] * g12_2 * gram[(3, 3)]
        - gram[(0, 0)] * gram[(1, 1)] * g23_2
        + gram[(0, 0)] * gram[(1, 1)] * gram[(2, 2)] * gram[(3, 3)];

    let b_pw2: f64 = quadratic * quadratic;
    let b_pw3: f64 = b_pw2 * quadratic;
    let b_pw4: f64 = b_pw3 * quadratic;
    let alpha: f64 = -0.375 * b_pw2 + linear;
    let beta: f64 = b_pw3 / 8.0 - quadratic * linear / 2.0 + constant;
    let gamma: f64 =
        -0.011_718_75 * b_pw4 + b_pw2 * linear / 16.0 - quadratic * constant / 4.0 + quartic_offset;
    let alpha_pw2: f64 = alpha * alpha;
    let alpha_pw3: f64 = alpha_pw2 * alpha;
    let point: f64 = -alpha_pw2 / 12.0 - gamma;
    let quaternion: f64 = -alpha_pw3 / 108.0 + alpha * gamma / 3.0 - pow(beta, 2.0) / 8.0;
    let helper1: f64 = -pow(point, 3.0) / 27.0;
    let theta2: f64 = pow(helper1, 1.0 / 3.0);
    let theta1: f64 = sqrt(theta2) * cos((1.0 / 3.0) * acos((-quaternion / 2.0) / sqrt(helper1)));
    let y: f64 = -(5.0 / 6.0) * alpha - ((1.0 / 3.0) * point * theta1 - theta1 * theta2) / theta2;
    let root_weight: f64 = sqrt(alpha + 2.0 * y);

    let temp1: f64 = -quadratic / 4.0 - 0.5 * root_weight;
    let temp2: f64 = 0.5 * sqrt(-3.0 * alpha - 2.0 * y + 2.0 * beta / root_weight);

    roots[2] = temp1 + temp2;
    roots[3] = temp1 - temp2;
}
#[must_use]
pub fn get_cost(moments: &GeneralizedMoments<'_>, cayley: &Vector3<f64>, step: i32) -> f64 {
    let mut roots: Vector4<f64> = Vector4::<f64>::zeros();
    get_ev(moments, cayley, &mut roots);

    let mut cost: f64 = 0.0;

    if step == 0 {
        cost = roots[2];
    }
    if step == 1 {
        cost = roots[3];
    }

    cost
}
pub fn get_cost_with_jacobian(
    moments: &GeneralizedMoments<'_>,
    cayley: &Vector3<f64>,
    jacobian: &mut SMatrix<f64, 1, 3>,
    step: i32,
) -> f64 {
    let eps: f64 = 0.000_000_01;
    let cost: f64 = get_cost(moments, cayley, step);

    for j in 0..3 {
        let mut cayley_j: Vector3<f64> = *cayley;
        cayley_j[j] += eps;
        let cost_j: f64 = get_cost(moments, &cayley_j, step);

        cayley_j = *cayley;
        cayley_j[j] -= eps;
        let cost_jm: f64 = get_cost(moments, &cayley_j, step);
        jacobian[(0, j)] = cost_j - cost_jm;
    }

    cost
}
pub fn get_quick_jacobian(
    moments: &GeneralizedMoments<'_>,
    cayley: &Vector3<f64>,
    current_value: f64,
    jacobian: &mut SMatrix<f64, 1, 3>,
    step: i32,
) {
    let eps: f64 = 0.000_000_01;

    for j in 0..3 {
        let mut cayley_j: Vector3<f64> = *cayley;
        cayley_j[j] += eps;
        let cost_j: f64 = get_cost(moments, &cayley_j, step);
        jacobian[(0, j)] = cost_j - current_value;
    }
}
#[must_use]
pub fn compose_g(moments: &GeneralizedMoments<'_>, cayley: &Vector3<f64>) -> Matrix4<f64> {
    let mut g: Matrix4<f64> = Matrix4::<f64>::zeros();
    let r: Matrix3<f64> = cayley2rot_reduced(cayley);

    let projected = [
        moments.xx_f * r.row(1).transpose(),
        moments.yy_f * r.row(0).transpose(),
        moments.zz_f * r.row(0).transpose(),
        moments.yz_f * r.row(0).transpose(),
        moments.xy_f * r.row(1).transpose(),
        moments.xy_f * r.row(2).transpose(),
        moments.zx_f * r.row(1).transpose(),
        moments.zx_f * r.row(2).transpose(),
    ];
    let mut temp: f64;
    temp = (r.row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    g[(0, 0)] = temp;
    temp = (-2.0 * r.row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    g[(0, 0)] += temp;
    temp = (r.row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    g[(0, 0)] += temp;

    temp = (r.row(2) * projected[3])[(0, 0)];
    g[(0, 1)] = temp;
    temp = (NEGATIVE_UNIT * r.row(2) * projected[5])[(0, 0)];
    g[(0, 1)] += temp;
    temp = (NEGATIVE_UNIT * r.row(1) * projected[2])[(0, 0)];
    g[(0, 1)] += temp;
    temp = (r.row(1) * projected[7])[(0, 0)];
    g[(0, 1)] += temp;

    temp = (r.row(2) * projected[4])[(0, 0)];
    g[(0, 2)] = temp;
    temp = (NEGATIVE_UNIT * r.row(2) * projected[1])[(0, 0)];
    g[(0, 2)] += temp;
    temp = (NEGATIVE_UNIT * r.row(1) * projected[6])[(0, 0)];
    g[(0, 2)] += temp;
    temp = (r.row(1) * projected[3])[(0, 0)];
    g[(0, 2)] += temp;

    temp = (r.row(0) * projected[2])[(0, 0)];
    g[(1, 1)] = temp;
    temp = (-2.0 * r.row(0) * projected[7])[(0, 0)];
    g[(1, 1)] += temp;
    temp = (r.row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    g[(1, 1)] += temp;

    temp = (r.row(0) * projected[6])[(0, 0)];
    g[(1, 2)] = temp;
    temp = (NEGATIVE_UNIT * r.row(0) * projected[3])[(0, 0)];
    g[(1, 2)] += temp;
    temp = (NEGATIVE_UNIT * r.row(2) * projected[0])[(0, 0)];
    g[(1, 2)] += temp;
    temp = (r.row(0) * projected[5])[(0, 0)];
    g[(1, 2)] += temp;

    temp = (r.row(1) * projected[0])[(0, 0)];
    g[(2, 2)] = temp;
    temp = (-2.0 * r.row(0) * projected[4])[(0, 0)];
    g[(2, 2)] += temp;
    temp = (r.row(0) * projected[1])[(0, 0)];
    g[(2, 2)] += temp;

    g[(1, 0)] = g[(0, 1)];
    g[(2, 0)] = g[(0, 2)];
    g[(2, 1)] = g[(1, 2)];

    compose_g_translation(moments, &r, &mut g);

    g
}
pub fn compose_g_with_jacobians(
    moments: &GeneralizedMoments<'_>,
    cayley: &Vector3<f64>,
    g_jac1: &mut Matrix4<f64>,
    g_jac2: &mut Matrix4<f64>,
    g_jac3: &mut Matrix4<f64>,
) -> Matrix4<f64> {
    let mut layout = RotationLayout::default();

    let r = cayley2rot_reduced(cayley);
    let rotation_differentials = reduced_rotation_differentials(cayley);
    let mut g = Matrix4::<f64>::zeros();
    let mut derivatives = [*g_jac1, *g_jac2, *g_jac3];
    compose_g_derivatives_section_0(
        &mut g,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    compose_g_derivatives_section_1(
        &mut g,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    compose_g_derivatives_section_2(
        &mut g,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    layout.rows = SMatrix::<f64, 1, 9>::zeros();
    compose_g_derivatives_stage_121(&r, &mut layout);
    layout.cols = SMatrix::<f64, 9, 1>::zeros();
    compose_g_derivatives_stage_123(&r, &mut layout);
    layout.rows_jac1 = SMatrix::<f64, 1, 9>::zeros();
    compose_g_derivatives_stage_125(&rotation_differentials, &mut layout);
    layout.rows_jac2 = SMatrix::<f64, 1, 9>::zeros();
    compose_g_derivatives_stage_127(&rotation_differentials, &mut layout);
    layout.rows_jac3 = SMatrix::<f64, 1, 9>::zeros();
    compose_g_derivatives_stage_129(&rotation_differentials, &mut layout);
    layout.cols_jac1 = SMatrix::<f64, 9, 1>::zeros();
    compose_g_derivatives_stage_131(&rotation_differentials, &mut layout);
    layout.cols_jac2 = SMatrix::<f64, 9, 1>::zeros();
    compose_g_derivatives_stage_133(&rotation_differentials, &mut layout);
    layout.cols_jac3 = SMatrix::<f64, 9, 1>::zeros();
    compose_g_derivatives_section_10(
        &mut g,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
        &mut layout,
    );
    compose_g_derivatives_section_11(
        &mut g,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
        &mut layout,
    );
    compose_g_derivatives_section_12(
        &mut g,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
        &mut layout,
    );
    *g_jac1 = derivatives[0];
    *g_jac2 = derivatives[1];
    *g_jac3 = derivatives[2];
    g
}

fn reduced_rotation_differentials(cayley: &Vector3<f64>) -> [Matrix3<f64>; 3] {
    let mut r_jac1: Matrix3<f64> = Matrix3::<f64>::zeros();
    let mut r_jac2: Matrix3<f64> = Matrix3::<f64>::zeros();
    let mut r_jac3: Matrix3<f64> = Matrix3::<f64>::zeros();
    r_jac1[(0, 0)] = 2.0 * cayley[0];
    r_jac1[(0, 1)] = 2.0 * cayley[1];
    r_jac1[(0, 2)] = 2.0 * cayley[2];
    r_jac1[(1, 0)] = 2.0 * cayley[1];
    r_jac1[(1, 1)] = -2.0 * cayley[0];
    r_jac1[(1, 2)] = -2.0;
    r_jac1[(2, 0)] = 2.0 * cayley[2];
    r_jac1[(2, 1)] = 2.0;
    r_jac1[(2, 2)] = -2.0 * cayley[0];
    r_jac2[(0, 0)] = -2.0 * cayley[1];
    r_jac2[(0, 1)] = 2.0 * cayley[0];
    r_jac2[(0, 2)] = 2.0;
    r_jac2[(1, 0)] = 2.0 * cayley[0];
    r_jac2[(1, 1)] = 2.0 * cayley[1];
    r_jac2[(1, 2)] = 2.0 * cayley[2];
    r_jac2[(2, 0)] = -2.0;
    r_jac2[(2, 1)] = 2.0 * cayley[2];
    r_jac2[(2, 2)] = -2.0 * cayley[1];
    r_jac3[(0, 0)] = -2.0 * cayley[2];
    r_jac3[(0, 1)] = -2.0;
    r_jac3[(0, 2)] = 2.0 * cayley[0];
    r_jac3[(1, 0)] = 2.0;
    r_jac3[(1, 1)] = -2.0 * cayley[2];
    r_jac3[(1, 2)] = 2.0 * cayley[1];
    r_jac3[(2, 0)] = 2.0 * cayley[0];
    r_jac3[(2, 1)] = 2.0 * cayley[1];
    r_jac3[(2, 2)] = 2.0 * cayley[2];
    [r_jac1, r_jac2, r_jac3]
}

#[inline]
fn compose_g_derivatives_stage_0(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    g[(0, 0)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_1(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (-2.0 * r.row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    g[(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_2(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    g[(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_3(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[0].row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(0, 0)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_4(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[0].row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_5(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(2) * moments.yz_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_6(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[0].row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_7(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[1].row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(0, 0)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_8(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[1].row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_9(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(2) * moments.yz_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_10(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[1].row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_11(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[2].row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(0, 0)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_12(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[2].row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_13(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(2) * moments.yz_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_14(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[2].row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 0)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_15(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    g[(0, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_16(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    g[(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_17(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    g[(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_18(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    g[(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_19(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[0].row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_20(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.yz_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_21(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[0].row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_22(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[0].row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_23(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(1) * moments.zz_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_24(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[0].row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_25(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(1) * moments.zx_f * rotation_differentials[0].row(2).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_26(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[1].row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_27(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.yz_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_28(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[1].row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_29(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[1].row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_30(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(1) * moments.zz_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_31(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[1].row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_32(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(1) * moments.zx_f * rotation_differentials[1].row(2).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_33(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[2].row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_34(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.yz_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_35(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[2].row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_36(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[2].row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_37(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(1) * moments.zz_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_38(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[2].row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_39(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(1) * moments.zx_f * rotation_differentials[2].row(2).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_40(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    g[(0, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_41(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    g[(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_42(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    g[(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_43(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    g[(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_44(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[0].row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_45(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_46(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[0].row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_47(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(2) * moments.yy_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_48(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[0].row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_49(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[0].row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_50(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(1) * moments.yz_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_51(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[1].row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_52(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_53(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[1].row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_54(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(2) * moments.yy_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_55(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[1].row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_56(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[1].row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_57(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(1) * moments.yz_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_58(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[2].row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_59(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_60(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[2].row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_61(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(2) * moments.yy_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_62(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[2].row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_63(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[2].row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_64(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(1) * moments.yz_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_65(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    g[(1, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_66(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (-2.0 * r.row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    g[(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_67(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    g[(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_68(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[0].row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(1, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_69(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[0].row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_70(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(0) * moments.zx_f * rotation_differentials[0].row(2).transpose())[(0, 0)];
    derivatives[0][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_71(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[0].row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_72(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[1].row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(1, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_73(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[1].row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_74(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(0) * moments.zx_f * rotation_differentials[1].row(2).transpose())[(0, 0)];
    derivatives[1][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_75(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[1].row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_76(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[2].row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(1, 1)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_77(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[2].row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_78(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(0) * moments.zx_f * rotation_differentials[2].row(2).transpose())[(0, 0)];
    derivatives[2][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_79(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[2].row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(1, 1)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_80(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    g[(1, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_81(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    g[(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_82(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    g[(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_83(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    g[(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_84(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[0].row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_85(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(0) * moments.zx_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_86(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[0].row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_87(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[0].row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_88(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(2) * moments.xx_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_89(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[0].row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_90(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_91(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[1].row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_92(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(0) * moments.zx_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_93(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[1].row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_94(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[1].row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_95(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(2) * moments.xx_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_96(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[1].row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_97(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_98(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[2].row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_99(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(0) * moments.zx_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_100(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[2].row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_101(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-rotation_differentials[2].row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_102(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (-r.row(2) * moments.xx_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_103(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (rotation_differentials[2].row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_104(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_105(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    g[(2, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_106(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (-2.0 * r.row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    g[(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_107(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
) {
    let temp = (r.row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    g[(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_108(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[0].row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(2, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_109(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[0].row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_110(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(0) * moments.xy_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_111(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[0].row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_112(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[1].row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(2, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_113(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[1].row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_114(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(0) * moments.xy_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_115(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[1].row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_116(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[2].row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(2, 2)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_117(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[2].row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_118(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (-2.0 * r.row(0) * moments.xy_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(2, 2)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_119(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[2].row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(2, 2)] += temp;
    g[(1, 0)] = g[(0, 1)];
    g[(2, 0)] = g[(0, 2)];
    g[(2, 1)] = g[(1, 2)];
    derivatives[0][(1, 0)] = derivatives[0][(0, 1)];
    derivatives[0][(2, 0)] = derivatives[0][(0, 2)];
    derivatives[0][(2, 1)] = derivatives[0][(1, 2)];
    derivatives[1][(1, 0)] = derivatives[1][(0, 1)];
    derivatives[1][(2, 0)] = derivatives[1][(0, 2)];
    derivatives[1][(2, 1)] = derivatives[1][(1, 2)];
    derivatives[2][(1, 0)] = derivatives[2][(0, 1)];
    derivatives[2][(2, 0)] = derivatives[2][(0, 2)];
    derivatives[2][(2, 1)] = derivatives[2][(1, 2)];
}
#[inline]
fn compose_g_derivatives_stage_121(r: &Matrix3<f64>, layout: &mut RotationLayout) {
    layout
        .rows
        .fixed_view_mut::<1, 3>(0, 0)
        .copy_from(&(r.row(0)));
    layout
        .rows
        .fixed_view_mut::<1, 3>(0, 3)
        .copy_from(&(r.row(1)));
    layout
        .rows
        .fixed_view_mut::<1, 3>(0, 6)
        .copy_from(&(r.row(2)));
}
#[inline]
fn compose_g_derivatives_stage_123(r: &Matrix3<f64>, layout: &mut RotationLayout) {
    layout
        .cols
        .fixed_view_mut::<3, 1>(0, 0)
        .copy_from(&(r.column(0)));
    layout
        .cols
        .fixed_view_mut::<3, 1>(3, 0)
        .copy_from(&(r.column(1)));
    layout
        .cols
        .fixed_view_mut::<3, 1>(6, 0)
        .copy_from(&(r.column(2)));
}
#[inline]
fn compose_g_derivatives_stage_125(
    rotation_differentials: &[Matrix3<f64>; 3],
    layout: &mut RotationLayout,
) {
    layout
        .rows_jac1
        .fixed_view_mut::<1, 3>(0, 0)
        .copy_from(&(rotation_differentials[0].row(0)));
    layout
        .rows_jac1
        .fixed_view_mut::<1, 3>(0, 3)
        .copy_from(&(rotation_differentials[0].row(1)));
    layout
        .rows_jac1
        .fixed_view_mut::<1, 3>(0, 6)
        .copy_from(&(rotation_differentials[0].row(2)));
}
#[inline]
fn compose_g_derivatives_stage_127(
    rotation_differentials: &[Matrix3<f64>; 3],
    layout: &mut RotationLayout,
) {
    layout
        .rows_jac2
        .fixed_view_mut::<1, 3>(0, 0)
        .copy_from(&(rotation_differentials[1].row(0)));
    layout
        .rows_jac2
        .fixed_view_mut::<1, 3>(0, 3)
        .copy_from(&(rotation_differentials[1].row(1)));
    layout
        .rows_jac2
        .fixed_view_mut::<1, 3>(0, 6)
        .copy_from(&(rotation_differentials[1].row(2)));
}
#[inline]
fn compose_g_derivatives_stage_129(
    rotation_differentials: &[Matrix3<f64>; 3],
    layout: &mut RotationLayout,
) {
    layout
        .rows_jac3
        .fixed_view_mut::<1, 3>(0, 0)
        .copy_from(&(rotation_differentials[2].row(0)));
    layout
        .rows_jac3
        .fixed_view_mut::<1, 3>(0, 3)
        .copy_from(&(rotation_differentials[2].row(1)));
    layout
        .rows_jac3
        .fixed_view_mut::<1, 3>(0, 6)
        .copy_from(&(rotation_differentials[2].row(2)));
}
#[inline]
fn compose_g_derivatives_stage_131(
    rotation_differentials: &[Matrix3<f64>; 3],
    layout: &mut RotationLayout,
) {
    layout
        .cols_jac1
        .fixed_view_mut::<3, 1>(0, 0)
        .copy_from(&(rotation_differentials[0].column(0)));
    layout
        .cols_jac1
        .fixed_view_mut::<3, 1>(3, 0)
        .copy_from(&(rotation_differentials[0].column(1)));
    layout
        .cols_jac1
        .fixed_view_mut::<3, 1>(6, 0)
        .copy_from(&(rotation_differentials[0].column(2)));
}
#[inline]
fn compose_g_derivatives_stage_133(
    rotation_differentials: &[Matrix3<f64>; 3],
    layout: &mut RotationLayout,
) {
    layout
        .cols_jac2
        .fixed_view_mut::<3, 1>(0, 0)
        .copy_from(&(rotation_differentials[1].column(0)));
    layout
        .cols_jac2
        .fixed_view_mut::<3, 1>(3, 0)
        .copy_from(&(rotation_differentials[1].column(1)));
    layout
        .cols_jac2
        .fixed_view_mut::<3, 1>(6, 0)
        .copy_from(&(rotation_differentials[1].column(2)));
}
#[inline]
fn compose_g_derivatives_stage_135(
    rotation_differentials: &[Matrix3<f64>; 3],
    layout: &mut RotationLayout,
) {
    layout
        .cols_jac3
        .fixed_view_mut::<3, 1>(0, 0)
        .copy_from(&(rotation_differentials[2].column(0)));
    layout
        .cols_jac3
        .fixed_view_mut::<3, 1>(3, 0)
        .copy_from(&(rotation_differentials[2].column(1)));
    layout
        .cols_jac3
        .fixed_view_mut::<3, 1>(6, 0)
        .copy_from(&(rotation_differentials[2].column(2)));
}
#[inline]
fn compose_g_derivatives_stage_136(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y1_p * layout.cols)[(0, 0)];
    g[(0, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_137(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y2_p * layout.rows.transpose())[(0, 0)];
    g[(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_138(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z1_p * layout.cols)[(0, 0)];
    g[(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_139(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z2_p * layout.rows.transpose())[(0, 0)];
    g[(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_140(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[0].row(2) * moments.y1_p * layout.cols)[(0, 0)];
    derivatives[0][(0, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_141(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y1_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_142(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[0].row(2) * moments.y2_p * layout.rows.transpose())[(0, 0)];
    derivatives[0][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_143(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y2_p * layout.rows_jac1.transpose())[(0, 0)];
    derivatives[0][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_144(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[0].row(1) * moments.z1_p * layout.cols)[(0, 0)];
    derivatives[0][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_145(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z1_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_146(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[0].row(1) * moments.z2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[0][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_147(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z2_p * layout.rows_jac1.transpose())[(0, 0)];
    derivatives[0][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_148(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[1].row(2) * moments.y1_p * layout.cols)[(0, 0)];
    derivatives[1][(0, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_149(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y1_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_150(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[1].row(2) * moments.y2_p * layout.rows.transpose())[(0, 0)];
    derivatives[1][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_151(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y2_p * layout.rows_jac2.transpose())[(0, 0)];
    derivatives[1][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_152(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[1].row(1) * moments.z1_p * layout.cols)[(0, 0)];
    derivatives[1][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_153(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z1_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_154(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[1].row(1) * moments.z2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[1][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_155(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z2_p * layout.rows_jac2.transpose())[(0, 0)];
    derivatives[1][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_156(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[2].row(2) * moments.y1_p * layout.cols)[(0, 0)];
    derivatives[2][(0, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_157(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y1_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_158(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[2].row(2) * moments.y2_p * layout.rows.transpose())[(0, 0)];
    derivatives[2][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_159(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(2) * moments.y2_p * layout.rows_jac3.transpose())[(0, 0)];
    derivatives[2][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_160(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[2].row(1) * moments.z1_p * layout.cols)[(0, 0)];
    derivatives[2][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_161(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z1_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_162(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[2].row(1) * moments.z2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[2][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_163(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.z2_p * layout.rows_jac3.transpose())[(0, 0)];
    derivatives[2][(0, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_164(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z1_p * layout.cols)[(0, 0)];
    g[(1, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_165(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z2_p * layout.rows.transpose())[(0, 0)];
    g[(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_166(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x1_p * layout.cols)[(0, 0)];
    g[(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_167(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x2_p * layout.rows.transpose())[(0, 0)];
    g[(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_168(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[0].row(0) * moments.z1_p * layout.cols)[(0, 0)];
    derivatives[0][(1, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_169(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z1_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_170(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[0].row(0) * moments.z2_p * layout.rows.transpose())[(0, 0)];
    derivatives[0][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_171(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z2_p * layout.rows_jac1.transpose())[(0, 0)];
    derivatives[0][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_172(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[0].row(2) * moments.x1_p * layout.cols)[(0, 0)];
    derivatives[0][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_173(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x1_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_174(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[0].row(2) * moments.x2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[0][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_175(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x2_p * layout.rows_jac1.transpose())[(0, 0)];
    derivatives[0][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_176(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[1].row(0) * moments.z1_p * layout.cols)[(0, 0)];
    derivatives[1][(1, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_177(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z1_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_178(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[1].row(0) * moments.z2_p * layout.rows.transpose())[(0, 0)];
    derivatives[1][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_179(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z2_p * layout.rows_jac2.transpose())[(0, 0)];
    derivatives[1][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_180(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[1].row(2) * moments.x1_p * layout.cols)[(0, 0)];
    derivatives[1][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_181(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x1_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_182(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[1].row(2) * moments.x2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[1][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_183(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x2_p * layout.rows_jac2.transpose())[(0, 0)];
    derivatives[1][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_184(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[2].row(0) * moments.z1_p * layout.cols)[(0, 0)];
    derivatives[2][(1, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_185(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z1_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_186(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[2].row(0) * moments.z2_p * layout.rows.transpose())[(0, 0)];
    derivatives[2][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_187(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(0) * moments.z2_p * layout.rows_jac3.transpose())[(0, 0)];
    derivatives[2][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_188(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[2].row(2) * moments.x1_p * layout.cols)[(0, 0)];
    derivatives[2][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_189(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x1_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_190(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[2].row(2) * moments.x2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[2][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_191(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.x2_p * layout.rows_jac3.transpose())[(0, 0)];
    derivatives[2][(1, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_192(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x1_p * layout.cols)[(0, 0)];
    g[(2, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_193(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x2_p * layout.rows.transpose())[(0, 0)];
    g[(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_194(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y1_p * layout.cols)[(0, 0)];
    g[(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_195(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y2_p * layout.rows.transpose())[(0, 0)];
    g[(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_196(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[0].row(1) * moments.x1_p * layout.cols)[(0, 0)];
    derivatives[0][(2, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_197(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x1_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_198(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[0].row(1) * moments.x2_p * layout.rows.transpose())[(0, 0)];
    derivatives[0][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_199(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x2_p * layout.rows_jac1.transpose())[(0, 0)];
    derivatives[0][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_200(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[0].row(0) * moments.y1_p * layout.cols)[(0, 0)];
    derivatives[0][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_201(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y1_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_202(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[0].row(0) * moments.y2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[0][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_203(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y2_p * layout.rows_jac1.transpose())[(0, 0)];
    derivatives[0][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_204(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[1].row(1) * moments.x1_p * layout.cols)[(0, 0)];
    derivatives[1][(2, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_205(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x1_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_206(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[1].row(1) * moments.x2_p * layout.rows.transpose())[(0, 0)];
    derivatives[1][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_207(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x2_p * layout.rows_jac2.transpose())[(0, 0)];
    derivatives[1][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_208(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[1].row(0) * moments.y1_p * layout.cols)[(0, 0)];
    derivatives[1][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_209(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y1_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_210(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[1].row(0) * moments.y2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[1][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_211(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y2_p * layout.rows_jac2.transpose())[(0, 0)];
    derivatives[1][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_212(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[2].row(1) * moments.x1_p * layout.cols)[(0, 0)];
    derivatives[2][(2, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_213(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x1_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_214(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (rotation_differentials[2].row(1) * moments.x2_p * layout.rows.transpose())[(0, 0)];
    derivatives[2][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_215(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (r.row(1) * moments.x2_p * layout.rows_jac3.transpose())[(0, 0)];
    derivatives[2][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_216(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[2].row(0) * moments.y1_p * layout.cols)[(0, 0)];
    derivatives[2][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_217(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y1_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_218(
    moments: &GeneralizedMoments<'_>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp =
        (NEGATIVE_UNIT * rotation_differentials[2].row(0) * moments.y2_p * layout.rows.transpose())
            [(0, 0)];
    derivatives[2][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_219(
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.y2_p * layout.rows_jac3.transpose())[(0, 0)];
    derivatives[2][(2, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_220(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * layout.cols.transpose() * moments.m11_p * layout.cols)[(0, 0)];
    g[(3, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_221(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    layout: &mut RotationLayout,
) {
    let temp = (NEGATIVE_UNIT * layout.rows * moments.m22_p * layout.rows.transpose())[(0, 0)];
    g[(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_222(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows * moments.m12_p * layout.cols)[(0, 0)];
    g[(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_223(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.cols.transpose() * moments.m11_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(3, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_224(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows_jac1 * moments.m22_p * layout.rows.transpose())[(0, 0)];
    derivatives[0][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_225(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows_jac1 * moments.m12_p * layout.cols)[(0, 0)];
    derivatives[0][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_226(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows * moments.m12_p * layout.cols_jac1)[(0, 0)];
    derivatives[0][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_227(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.cols.transpose() * moments.m11_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(3, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_228(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows_jac2 * moments.m22_p * layout.rows.transpose())[(0, 0)];
    derivatives[1][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_229(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows_jac2 * moments.m12_p * layout.cols)[(0, 0)];
    derivatives[1][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_230(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows * moments.m12_p * layout.cols_jac2)[(0, 0)];
    derivatives[1][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_231(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.cols.transpose() * moments.m11_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(3, 3)] = temp;
}
#[inline]
fn compose_g_derivatives_stage_232(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows_jac3 * moments.m22_p * layout.rows.transpose())[(0, 0)];
    derivatives[2][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_233(
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows_jac3 * moments.m12_p * layout.cols)[(0, 0)];
    derivatives[2][(3, 3)] += temp;
}
#[inline]
fn compose_g_derivatives_stage_234(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    let temp = (-2.0 * layout.rows * moments.m12_p * layout.cols_jac3)[(0, 0)];
    derivatives[2][(3, 3)] += temp;
    g[(3, 0)] = g[(0, 3)];
    g[(3, 1)] = g[(1, 3)];
    g[(3, 2)] = g[(2, 3)];
    derivatives[0][(3, 0)] = derivatives[0][(0, 3)];
    derivatives[0][(3, 1)] = derivatives[0][(1, 3)];
    derivatives[0][(3, 2)] = derivatives[0][(2, 3)];
    derivatives[1][(3, 0)] = derivatives[1][(0, 3)];
    derivatives[1][(3, 1)] = derivatives[1][(1, 3)];
    derivatives[1][(3, 2)] = derivatives[1][(2, 3)];
    derivatives[2][(3, 0)] = derivatives[2][(0, 3)];
    derivatives[2][(3, 1)] = derivatives[2][(1, 3)];
    derivatives[2][(3, 2)] = derivatives[2][(2, 3)];
}
#[derive(Default)]
struct RotationLayout {
    rows: SMatrix<f64, 1, 9>,
    cols: SMatrix<f64, 9, 1>,
    rows_jac1: SMatrix<f64, 1, 9>,
    rows_jac2: SMatrix<f64, 1, 9>,
    rows_jac3: SMatrix<f64, 1, 9>,
    cols_jac1: SMatrix<f64, 9, 1>,
    cols_jac2: SMatrix<f64, 9, 1>,
    cols_jac3: SMatrix<f64, 9, 1>,
}

fn compose_g_derivatives_section_0(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    compose_g_derivatives_stage_0(g, moments, r);
    compose_g_derivatives_stage_1(g, moments, r);
    compose_g_derivatives_stage_2(g, moments, r);
    compose_g_derivatives_stage_3(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_4(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_5(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_6(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_7(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_8(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_9(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_10(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_11(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_12(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_13(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_14(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_15(g, moments, r);
    compose_g_derivatives_stage_16(g, moments, r);
    compose_g_derivatives_stage_17(g, moments, r);
    compose_g_derivatives_stage_18(g, moments, r);
    compose_g_derivatives_stage_19(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_20(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_21(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_22(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_23(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_24(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_25(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_26(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_27(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_28(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_29(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_30(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_31(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_32(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_33(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_34(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_35(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_36(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_37(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_38(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_39(moments, r, rotation_differentials, derivatives);
}
fn compose_g_derivatives_section_1(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    compose_g_derivatives_stage_40(g, moments, r);
    compose_g_derivatives_stage_41(g, moments, r);
    compose_g_derivatives_stage_42(g, moments, r);
    compose_g_derivatives_stage_43(g, moments, r);
    compose_g_derivatives_stage_44(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_45(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_46(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_47(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_48(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_49(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_50(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_51(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_52(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_53(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_54(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_55(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_56(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_57(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_58(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_59(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_60(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_61(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_62(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_63(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_64(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_65(g, moments, r);
    compose_g_derivatives_stage_66(g, moments, r);
    compose_g_derivatives_stage_67(g, moments, r);
    compose_g_derivatives_stage_68(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_69(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_70(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_71(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_72(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_73(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_74(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_75(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_76(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_77(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_78(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_79(moments, r, rotation_differentials, derivatives);
}
fn compose_g_derivatives_section_2(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
) {
    compose_g_derivatives_stage_80(g, moments, r);
    compose_g_derivatives_stage_81(g, moments, r);
    compose_g_derivatives_stage_82(g, moments, r);
    compose_g_derivatives_stage_83(g, moments, r);
    compose_g_derivatives_stage_84(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_85(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_86(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_87(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_88(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_89(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_90(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_91(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_92(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_93(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_94(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_95(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_96(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_97(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_98(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_99(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_100(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_101(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_102(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_103(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_104(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_105(g, moments, r);
    compose_g_derivatives_stage_106(g, moments, r);
    compose_g_derivatives_stage_107(g, moments, r);
    compose_g_derivatives_stage_108(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_109(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_110(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_111(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_112(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_113(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_114(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_115(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_116(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_117(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_118(moments, r, rotation_differentials, derivatives);
    compose_g_derivatives_stage_119(g, moments, r, rotation_differentials, derivatives);
}
fn compose_g_derivatives_section_10(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    compose_g_derivatives_stage_135(rotation_differentials, layout);
    compose_g_derivatives_stage_136(g, moments, r, layout);
    compose_g_derivatives_stage_137(g, moments, r, layout);
    compose_g_derivatives_stage_138(g, moments, r, layout);
    compose_g_derivatives_stage_139(g, moments, r, layout);
    compose_g_derivatives_stage_140(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_141(moments, r, derivatives, layout);
    compose_g_derivatives_stage_142(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_143(moments, r, derivatives, layout);
    compose_g_derivatives_stage_144(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_145(moments, r, derivatives, layout);
    compose_g_derivatives_stage_146(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_147(moments, r, derivatives, layout);
    compose_g_derivatives_stage_148(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_149(moments, r, derivatives, layout);
    compose_g_derivatives_stage_150(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_151(moments, r, derivatives, layout);
    compose_g_derivatives_stage_152(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_153(moments, r, derivatives, layout);
    compose_g_derivatives_stage_154(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_155(moments, r, derivatives, layout);
    compose_g_derivatives_stage_156(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_157(moments, r, derivatives, layout);
    compose_g_derivatives_stage_158(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_159(moments, r, derivatives, layout);
    compose_g_derivatives_stage_160(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_161(moments, r, derivatives, layout);
    compose_g_derivatives_stage_162(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_163(moments, r, derivatives, layout);
    compose_g_derivatives_stage_164(g, moments, r, layout);
    compose_g_derivatives_stage_165(g, moments, r, layout);
    compose_g_derivatives_stage_166(g, moments, r, layout);
    compose_g_derivatives_stage_167(g, moments, r, layout);
    compose_g_derivatives_stage_168(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_169(moments, r, derivatives, layout);
    compose_g_derivatives_stage_170(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_171(moments, r, derivatives, layout);
    compose_g_derivatives_stage_172(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_173(moments, r, derivatives, layout);
    compose_g_derivatives_stage_174(moments, rotation_differentials, derivatives, layout);
}
fn compose_g_derivatives_section_11(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    compose_g_derivatives_stage_175(moments, r, derivatives, layout);
    compose_g_derivatives_stage_176(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_177(moments, r, derivatives, layout);
    compose_g_derivatives_stage_178(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_179(moments, r, derivatives, layout);
    compose_g_derivatives_stage_180(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_181(moments, r, derivatives, layout);
    compose_g_derivatives_stage_182(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_183(moments, r, derivatives, layout);
    compose_g_derivatives_stage_184(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_185(moments, r, derivatives, layout);
    compose_g_derivatives_stage_186(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_187(moments, r, derivatives, layout);
    compose_g_derivatives_stage_188(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_189(moments, r, derivatives, layout);
    compose_g_derivatives_stage_190(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_191(moments, r, derivatives, layout);
    compose_g_derivatives_stage_192(g, moments, r, layout);
    compose_g_derivatives_stage_193(g, moments, r, layout);
    compose_g_derivatives_stage_194(g, moments, r, layout);
    compose_g_derivatives_stage_195(g, moments, r, layout);
    compose_g_derivatives_stage_196(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_197(moments, r, derivatives, layout);
    compose_g_derivatives_stage_198(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_199(moments, r, derivatives, layout);
    compose_g_derivatives_stage_200(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_201(moments, r, derivatives, layout);
    compose_g_derivatives_stage_202(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_203(moments, r, derivatives, layout);
    compose_g_derivatives_stage_204(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_205(moments, r, derivatives, layout);
    compose_g_derivatives_stage_206(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_207(moments, r, derivatives, layout);
    compose_g_derivatives_stage_208(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_209(moments, r, derivatives, layout);
    compose_g_derivatives_stage_210(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_211(moments, r, derivatives, layout);
    compose_g_derivatives_stage_212(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_213(moments, r, derivatives, layout);
    compose_g_derivatives_stage_214(moments, rotation_differentials, derivatives, layout);
}
fn compose_g_derivatives_section_12(
    g: &mut Matrix4<f64>,
    moments: &GeneralizedMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix4<f64>; 3],
    layout: &mut RotationLayout,
) {
    compose_g_derivatives_stage_215(moments, r, derivatives, layout);
    compose_g_derivatives_stage_216(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_217(moments, r, derivatives, layout);
    compose_g_derivatives_stage_218(moments, rotation_differentials, derivatives, layout);
    compose_g_derivatives_stage_219(moments, r, derivatives, layout);
    compose_g_derivatives_stage_220(g, moments, layout);
    compose_g_derivatives_stage_221(g, moments, layout);
    compose_g_derivatives_stage_222(g, moments, layout);
    compose_g_derivatives_stage_223(moments, derivatives, layout);
    compose_g_derivatives_stage_224(moments, derivatives, layout);
    compose_g_derivatives_stage_225(moments, derivatives, layout);
    compose_g_derivatives_stage_226(moments, derivatives, layout);
    compose_g_derivatives_stage_227(moments, derivatives, layout);
    compose_g_derivatives_stage_228(moments, derivatives, layout);
    compose_g_derivatives_stage_229(moments, derivatives, layout);
    compose_g_derivatives_stage_230(moments, derivatives, layout);
    compose_g_derivatives_stage_231(moments, derivatives, layout);
    compose_g_derivatives_stage_232(moments, derivatives, layout);
    compose_g_derivatives_stage_233(moments, derivatives, layout);
    compose_g_derivatives_stage_234(g, moments, derivatives, layout);
}

fn compose_g_translation(moments: &GeneralizedMoments<'_>, r: &Matrix3<f64>, g: &mut Matrix4<f64>) {
    let mut rows: SMatrix<f64, 1, 9> = SMatrix::<f64, 1, 9>::zeros();
    rows.fixed_view_mut::<1, 3>(0, 0).copy_from(&(r.row(0)));
    rows.fixed_view_mut::<1, 3>(0, 3).copy_from(&(r.row(1)));
    rows.fixed_view_mut::<1, 3>(0, 6).copy_from(&(r.row(2)));
    let rowst: SMatrix<f64, 9, 1> = rows.transpose();

    let mut cols: SMatrix<f64, 9, 1> = SMatrix::<f64, 9, 1>::zeros();
    cols.fixed_view_mut::<3, 1>(0, 0).copy_from(&(r.column(0)));
    cols.fixed_view_mut::<3, 1>(3, 0).copy_from(&(r.column(1)));
    cols.fixed_view_mut::<3, 1>(6, 0).copy_from(&(r.column(2)));

    let projected_offsets = [
        moments.x1_p * cols,
        moments.y1_p * cols,
        moments.z1_p * cols,
        moments.x2_p * rowst,
        moments.y2_p * rowst,
        moments.z2_p * rowst,
    ];
    let temp = (r.row(2) * projected_offsets[1])[(0, 0)];
    g[(0, 3)] = temp;
    let temp = (r.row(2) * projected_offsets[4])[(0, 0)];
    g[(0, 3)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(1) * projected_offsets[2])[(0, 0)];
    g[(0, 3)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(1) * projected_offsets[5])[(0, 0)];
    g[(0, 3)] += temp;

    let temp = (r.row(0) * projected_offsets[2])[(0, 0)];
    g[(1, 3)] = temp;
    let temp = (r.row(0) * projected_offsets[5])[(0, 0)];
    g[(1, 3)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(2) * projected_offsets[0])[(0, 0)];
    g[(1, 3)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(2) * projected_offsets[3])[(0, 0)];
    g[(1, 3)] += temp;

    let temp = (r.row(1) * projected_offsets[0])[(0, 0)];
    g[(2, 3)] = temp;
    let temp = (r.row(1) * projected_offsets[3])[(0, 0)];
    g[(2, 3)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(0) * projected_offsets[1])[(0, 0)];
    g[(2, 3)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(0) * projected_offsets[4])[(0, 0)];
    g[(2, 3)] += temp;

    let temp = (NEGATIVE_UNIT * cols.transpose() * moments.m11_p * cols)[(0, 0)];
    g[(3, 3)] = temp;
    let temp = (NEGATIVE_UNIT * rows * moments.m22_p * rowst)[(0, 0)];
    g[(3, 3)] += temp;
    let temp = (-2.0 * rows * moments.m12_p * cols)[(0, 0)];
    g[(3, 3)] += temp;

    g[(3, 0)] = g[(0, 3)];
    g[(3, 1)] = g[(1, 3)];
    g[(3, 2)] = g[(2, 3)];
}
