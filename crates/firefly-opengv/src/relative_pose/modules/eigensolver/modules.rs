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
use nalgebra::{Matrix3, SMatrix, Vector3};

/// Precomputed bearing-product moments.
pub struct BearingMoments<'a> {
    pub xx_f: &'a Matrix3<f64>,
    pub yy_f: &'a Matrix3<f64>,
    pub zz_f: &'a Matrix3<f64>,
    pub xy_f: &'a Matrix3<f64>,
    pub yz_f: &'a Matrix3<f64>,
    pub zx_f: &'a Matrix3<f64>,
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
fn sin(x: f64) -> f64 {
    x.sin()
}
pub fn get_smallest_ev(
    moments: &BearingMoments<'_>,
    cayley: &Vector3<f64>,
    matrix: &mut Matrix3<f64>,
) -> f64 {
    *matrix = compose_m(moments, cayley);

    let quadratic: f64 = -matrix[(0, 0)] - matrix[(1, 1)] - matrix[(2, 2)];
    let linear: f64 =
        -pow(matrix[(0, 2)], 2.0) - pow(matrix[(1, 2)], 2.0) - pow(matrix[(0, 1)], 2.0)
            + matrix[(0, 0)] * matrix[(1, 1)]
            + matrix[(0, 0)] * matrix[(2, 2)]
            + matrix[(1, 1)] * matrix[(2, 2)];
    let constant: f64 = matrix[(1, 1)] * pow(matrix[(0, 2)], 2.0)
        + matrix[(0, 0)] * pow(matrix[(1, 2)], 2.0)
        + matrix[(2, 2)] * pow(matrix[(0, 1)], 2.0)
        - matrix[(0, 0)] * matrix[(1, 1)] * matrix[(2, 2)]
        - 2.0 * matrix[(0, 1)] * matrix[(1, 2)] * matrix[(0, 2)];

    let state: f64 = 2.0 * pow(quadratic, 3.0) - 9.0 * quadratic * linear + 27.0 * constant;
    let translation: f64 = 4.0 * pow(pow(quadratic, 2.0) - 3.0 * linear, 3.0);

    let alpha: f64 = acos(state / sqrt(translation));
    let beta: f64 = alpha / 3.0;
    let y: f64 = cos(beta);

    let rotation: f64 = 0.5 * sqrt(translation);
    let root_weight: f64 = pow(rotation, 1.0 / 3.0);

    let root_product: f64 = root_weight * y;
    let smallest_ev: f64 = (-quadratic - 2.0 * root_product) / 3.0;
    smallest_ev
}
pub fn get_smallest_ev_with_jacobian(
    moments: &BearingMoments<'_>,
    cayley: &Vector3<f64>,
    jacobian: &mut SMatrix<f64, 1, 3>,
) -> f64 {
    let mut m_jac1: Matrix3<f64> = Matrix3::zeros();
    let mut m_jac2: Matrix3<f64> = Matrix3::zeros();
    let mut m_jac3: Matrix3<f64> = Matrix3::zeros();

    let matrix: Matrix3<f64> =
        compose_m_with_jacobians(moments, cayley, &mut m_jac1, &mut m_jac2, &mut m_jac3);

    let quadratic: f64 = -matrix[(0, 0)] - matrix[(1, 1)] - matrix[(2, 2)];
    let b_jac1: f64 = -m_jac1[(0, 0)] - m_jac1[(1, 1)] - m_jac1[(2, 2)];
    let b_jac2: f64 = -m_jac2[(0, 0)] - m_jac2[(1, 1)] - m_jac2[(2, 2)];
    let b_jac3: f64 = -m_jac3[(0, 0)] - m_jac3[(1, 1)] - m_jac3[(2, 2)];
    let linear: f64 =
        -pow(matrix[(0, 2)], 2.0) - pow(matrix[(1, 2)], 2.0) - pow(matrix[(0, 1)], 2.0)
            + matrix[(0, 0)] * matrix[(1, 1)]
            + matrix[(0, 0)] * matrix[(2, 2)]
            + matrix[(1, 1)] * matrix[(2, 2)];
    let c_jac1: f64 = characteristic_linear_derivative(&matrix, &m_jac1);
    let c_jac2: f64 = characteristic_linear_derivative(&matrix, &m_jac2);
    let c_jac3: f64 = characteristic_linear_derivative(&matrix, &m_jac3);
    let constant: f64 = matrix[(1, 1)] * pow(matrix[(0, 2)], 2.0)
        + matrix[(0, 0)] * pow(matrix[(1, 2)], 2.0)
        + matrix[(2, 2)] * pow(matrix[(0, 1)], 2.0)
        - matrix[(0, 0)] * matrix[(1, 1)] * matrix[(2, 2)]
        - 2.0 * matrix[(0, 1)] * matrix[(1, 2)] * matrix[(0, 2)];
    let d_jac1: f64 = characteristic_constant_derivative(&matrix, &m_jac1);
    let d_jac2: f64 = characteristic_constant_derivative(&matrix, &m_jac2);
    let d_jac3: f64 = characteristic_constant_derivative(&matrix, &m_jac3);

    let state: f64 = 2.0 * pow(quadratic, 3.0) - 9.0 * quadratic * linear + 27.0 * constant;
    let translation: f64 = 4.0 * pow(pow(quadratic, 2.0) - 3.0 * linear, 3.0);
    let s_jac1: f64 =
        2.0 * 3.0 * pow(quadratic, 2.0) * b_jac1 - 9.0 * b_jac1 * linear - 9.0 * quadratic * c_jac1
            + 27.0 * d_jac1;
    let s_jac2: f64 =
        2.0 * 3.0 * pow(quadratic, 2.0) * b_jac2 - 9.0 * b_jac2 * linear - 9.0 * quadratic * c_jac2
            + 27.0 * d_jac2;
    let s_jac3: f64 =
        2.0 * 3.0 * pow(quadratic, 2.0) * b_jac3 - 9.0 * b_jac3 * linear - 9.0 * quadratic * c_jac3
            + 27.0 * d_jac3;
    let t_jac1: f64 = 4.0
        * 3.0
        * pow(pow(quadratic, 2.0) - 3.0 * linear, 2.0)
        * (2.0 * quadratic * b_jac1 - 3.0 * c_jac1);
    let t_jac2: f64 = 4.0
        * 3.0
        * pow(pow(quadratic, 2.0) - 3.0 * linear, 2.0)
        * (2.0 * quadratic * b_jac2 - 3.0 * c_jac2);
    let t_jac3: f64 = 4.0
        * 3.0
        * pow(pow(quadratic, 2.0) - 3.0 * linear, 2.0)
        * (2.0 * quadratic * b_jac3 - 3.0 * c_jac3);

    let alpha: f64 = acos(state / sqrt(translation));
    let alpha_jac1: f64 = -1.0 / sqrt(1.0 - (pow(state, 2.0) / translation))
        * (s_jac1 * sqrt(translation) - state * 0.5 * pow(translation, -0.5) * t_jac1)
        / translation;
    let alpha_jac2: f64 = -1.0 / sqrt(1.0 - (pow(state, 2.0) / translation))
        * (s_jac2 * sqrt(translation) - state * 0.5 * pow(translation, -0.5) * t_jac2)
        / translation;
    let alpha_jac3: f64 = -1.0 / sqrt(1.0 - (pow(state, 2.0) / translation))
        * (s_jac3 * sqrt(translation) - state * 0.5 * pow(translation, -0.5) * t_jac3)
        / translation;
    let beta: f64 = alpha / 3.0;
    let beta_jac1: f64 = alpha_jac1 / 3.0;
    let beta_jac2: f64 = alpha_jac2 / 3.0;
    let beta_jac3: f64 = alpha_jac3 / 3.0;
    let y: f64 = cos(beta);
    let y_jac1: f64 = -sin(beta) * beta_jac1;
    let y_jac2: f64 = -sin(beta) * beta_jac2;
    let y_jac3: f64 = -sin(beta) * beta_jac3;

    let rotation: f64 = 0.5 * sqrt(translation);
    let r_jac1: f64 = 0.25 * pow(translation, -0.5) * t_jac1;
    let r_jac2: f64 = 0.25 * pow(translation, -0.5) * t_jac2;
    let r_jac3: f64 = 0.25 * pow(translation, -0.5) * t_jac3;
    let root_weight: f64 = pow(rotation, 1.0 / 3.0);
    let w_jac1: f64 = (1.0 / 3.0) * pow(rotation, -2.0 / 3.0) * r_jac1;
    let w_jac2: f64 = (1.0 / 3.0) * pow(rotation, -2.0 / 3.0) * r_jac2;
    let w_jac3: f64 = (1.0 / 3.0) * pow(rotation, -2.0 / 3.0) * r_jac3;

    let root_product: f64 = root_weight * y;
    let k_jac1: f64 = w_jac1 * y + root_weight * y_jac1;
    let k_jac2: f64 = w_jac2 * y + root_weight * y_jac2;
    let k_jac3: f64 = w_jac3 * y + root_weight * y_jac3;
    let smallest_ev: f64 = (-quadratic - 2.0 * root_product) / 3.0;
    let smallest_ev_jac1: f64 = (-b_jac1 - 2.0 * k_jac1) / 3.0;
    let smallest_ev_jac2: f64 = (-b_jac2 - 2.0 * k_jac2) / 3.0;
    let smallest_ev_jac3: f64 = (-b_jac3 - 2.0 * k_jac3) / 3.0;

    jacobian[(0, 0)] = smallest_ev_jac1;
    jacobian[(0, 1)] = smallest_ev_jac2;
    jacobian[(0, 2)] = smallest_ev_jac3;
    smallest_ev
}
#[must_use]
pub fn compose_m(moments: &BearingMoments<'_>, cayley: &Vector3<f64>) -> Matrix3<f64> {
    let mut m: Matrix3<f64> = Matrix3::<f64>::zeros();
    let r: Matrix3<f64> = cayley2rot_reduced(cayley);

    let mut temp: f64;
    temp = (r.row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    m[(0, 0)] = temp;
    temp = (-2.0 * r.row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    m[(0, 0)] += temp;
    temp = (r.row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    m[(0, 0)] += temp;

    temp = (r.row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    m[(0, 1)] = temp;
    temp = (NEGATIVE_UNIT * r.row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    m[(0, 1)] += temp;
    temp = (NEGATIVE_UNIT * r.row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    m[(0, 1)] += temp;
    temp = (r.row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    m[(0, 1)] += temp;

    temp = (r.row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    m[(0, 2)] = temp;
    temp = (NEGATIVE_UNIT * r.row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    m[(0, 2)] += temp;
    temp = (NEGATIVE_UNIT * r.row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    m[(0, 2)] += temp;
    temp = (r.row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    m[(0, 2)] += temp;

    temp = (r.row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    m[(1, 1)] = temp;
    temp = (-2.0 * r.row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    m[(1, 1)] += temp;
    temp = (r.row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    m[(1, 1)] += temp;

    temp = (r.row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    m[(1, 2)] = temp;
    temp = (NEGATIVE_UNIT * r.row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    m[(1, 2)] += temp;
    temp = (NEGATIVE_UNIT * r.row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    m[(1, 2)] += temp;
    temp = (r.row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    m[(1, 2)] += temp;

    temp = (r.row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    m[(2, 2)] = temp;
    temp = (-2.0 * r.row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    m[(2, 2)] += temp;
    temp = (r.row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    m[(2, 2)] += temp;

    m[(1, 0)] = m[(0, 1)];
    m[(2, 0)] = m[(0, 2)];
    m[(2, 1)] = m[(1, 2)];

    m
}
pub fn compose_m_with_jacobians(
    moments: &BearingMoments<'_>,
    cayley: &Vector3<f64>,
    m_jac1: &mut Matrix3<f64>,
    m_jac2: &mut Matrix3<f64>,
    m_jac3: &mut Matrix3<f64>,
) -> Matrix3<f64> {
    let r = cayley2rot_reduced(cayley);
    let rotation_differentials = reduced_rotation_differentials(cayley);
    let mut m = Matrix3::<f64>::zeros();
    let mut derivatives = [*m_jac1, *m_jac2, *m_jac3];
    compose_m_derivatives_stage_0(
        &mut m,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    compose_m_derivatives_stage_1(
        &mut m,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    compose_m_derivatives_stage_2(
        &mut m,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    compose_m_derivatives_stage_3(
        &mut m,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    compose_m_derivatives_stage_4(
        &mut m,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    compose_m_derivatives_stage_5(
        &mut m,
        moments,
        &r,
        &rotation_differentials,
        &mut derivatives,
    );
    *m_jac1 = derivatives[0];
    *m_jac2 = derivatives[1];
    *m_jac3 = derivatives[2];
    m
}

fn characteristic_linear_derivative(matrix: &Matrix3<f64>, derivative: &Matrix3<f64>) -> f64 {
    -2.0 * matrix[(0, 2)] * derivative[(0, 2)]
        - 2.0 * matrix[(1, 2)] * derivative[(1, 2)]
        - 2.0 * matrix[(0, 1)] * derivative[(0, 1)]
        + derivative[(0, 0)] * matrix[(1, 1)]
        + matrix[(0, 0)] * derivative[(1, 1)]
        + derivative[(0, 0)] * matrix[(2, 2)]
        + matrix[(0, 0)] * derivative[(2, 2)]
        + derivative[(1, 1)] * matrix[(2, 2)]
        + matrix[(1, 1)] * derivative[(2, 2)]
}
fn characteristic_constant_derivative(matrix: &Matrix3<f64>, derivative: &Matrix3<f64>) -> f64 {
    derivative[(1, 1)] * pow(matrix[(0, 2)], 2.0)
        + matrix[(1, 1)] * 2.0 * matrix[(0, 2)] * derivative[(0, 2)]
        + derivative[(0, 0)] * pow(matrix[(1, 2)], 2.0)
        + matrix[(0, 0)] * 2.0 * matrix[(1, 2)] * derivative[(1, 2)]
        + derivative[(2, 2)] * pow(matrix[(0, 1)], 2.0)
        + matrix[(2, 2)] * 2.0 * matrix[(0, 1)] * derivative[(0, 1)]
        - derivative[(0, 0)] * matrix[(1, 1)] * matrix[(2, 2)]
        - matrix[(0, 0)] * derivative[(1, 1)] * matrix[(2, 2)]
        - matrix[(0, 0)] * matrix[(1, 1)] * derivative[(2, 2)]
        - 2.0
            * (derivative[(0, 1)] * matrix[(1, 2)] * matrix[(0, 2)]
                + matrix[(0, 1)] * derivative[(1, 2)] * matrix[(0, 2)]
                + matrix[(0, 1)] * matrix[(1, 2)] * derivative[(0, 2)])
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
fn compose_m_derivatives_stage_0(
    m: &mut Matrix3<f64>,
    moments: &BearingMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix3<f64>; 3],
) {
    let temp = (r.row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    m[(0, 0)] = temp;
    let temp = (-2.0 * r.row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    m[(0, 0)] += temp;
    let temp = (r.row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    m[(0, 0)] += temp;
    let temp =
        (2.0 * rotation_differentials[0].row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(0, 0)] = temp;
    let temp =
        (-2.0 * rotation_differentials[0].row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 0)] += temp;
    let temp =
        (-2.0 * r.row(2) * moments.yz_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(0, 0)] += temp;
    let temp =
        (2.0 * rotation_differentials[0].row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 0)] += temp;
    let temp =
        (2.0 * rotation_differentials[1].row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(0, 0)] = temp;
    let temp =
        (-2.0 * rotation_differentials[1].row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 0)] += temp;
    let temp =
        (-2.0 * r.row(2) * moments.yz_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(0, 0)] += temp;
    let temp =
        (2.0 * rotation_differentials[1].row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 0)] += temp;
    let temp =
        (2.0 * rotation_differentials[2].row(2) * moments.yy_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(0, 0)] = temp;
    let temp =
        (-2.0 * rotation_differentials[2].row(2) * moments.yz_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 0)] += temp;
    let temp =
        (-2.0 * r.row(2) * moments.yz_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(0, 0)] += temp;
    let temp =
        (2.0 * rotation_differentials[2].row(1) * moments.zz_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 0)] += temp;
    let temp = (r.row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    m[(0, 1)] = temp;
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    m[(0, 1)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    m[(0, 1)] += temp;
    let temp = (r.row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    m[(0, 1)] += temp;
    let temp = (rotation_differentials[0].row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] = temp;
    let temp = (r.row(2) * moments.yz_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
    let temp =
        (-2.0 * rotation_differentials[0].row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
    let temp = (-rotation_differentials[0].row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
}
#[inline]
fn compose_m_derivatives_stage_1(
    m: &mut Matrix3<f64>,
    moments: &BearingMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix3<f64>; 3],
) {
    let temp = (-r.row(1) * moments.zz_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
    let temp = (rotation_differentials[0].row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
    let temp = (r.row(1) * moments.zx_f * rotation_differentials[0].row(2).transpose())[(0, 0)];
    derivatives[0][(0, 1)] += temp;
    let temp = (rotation_differentials[1].row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] = temp;
    let temp = (r.row(2) * moments.yz_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
    let temp =
        (-2.0 * rotation_differentials[1].row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
    let temp = (-rotation_differentials[1].row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
    let temp = (-r.row(1) * moments.zz_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
    let temp = (rotation_differentials[1].row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
    let temp = (r.row(1) * moments.zx_f * rotation_differentials[1].row(2).transpose())[(0, 0)];
    derivatives[1][(0, 1)] += temp;
    let temp = (rotation_differentials[2].row(2) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] = temp;
    let temp = (r.row(2) * moments.yz_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
    let temp =
        (-2.0 * rotation_differentials[2].row(2) * moments.xy_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
    let temp = (-rotation_differentials[2].row(1) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
    let temp = (-r.row(1) * moments.zz_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
    let temp = (rotation_differentials[2].row(1) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
    let temp = (r.row(1) * moments.zx_f * rotation_differentials[2].row(2).transpose())[(0, 0)];
    derivatives[2][(0, 1)] += temp;
    let temp = (r.row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    m[(0, 2)] = temp;
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    m[(0, 2)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    m[(0, 2)] += temp;
    let temp = (r.row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    m[(0, 2)] += temp;
    let temp = (rotation_differentials[0].row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 2)] = temp;
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
}
#[inline]
fn compose_m_derivatives_stage_2(
    m: &mut Matrix3<f64>,
    moments: &BearingMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix3<f64>; 3],
) {
    let temp = (-rotation_differentials[0].row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
    let temp = (-r.row(2) * moments.yy_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
    let temp =
        (-2.0 * rotation_differentials[0].row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
    let temp = (rotation_differentials[0].row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
    let temp = (r.row(1) * moments.yz_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(0, 2)] += temp;
    let temp = (rotation_differentials[1].row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 2)] = temp;
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
    let temp = (-rotation_differentials[1].row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
    let temp = (-r.row(2) * moments.yy_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
    let temp =
        (-2.0 * rotation_differentials[1].row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
    let temp = (rotation_differentials[1].row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
    let temp = (r.row(1) * moments.yz_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(0, 2)] += temp;
    let temp = (rotation_differentials[2].row(2) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 2)] = temp;
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
    let temp = (-rotation_differentials[2].row(2) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
    let temp = (-r.row(2) * moments.yy_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
    let temp =
        (-2.0 * rotation_differentials[2].row(1) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
    let temp = (rotation_differentials[2].row(1) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
    let temp = (r.row(1) * moments.yz_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(0, 2)] += temp;
    let temp = (r.row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    m[(1, 1)] = temp;
    let temp = (-2.0 * r.row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    m[(1, 1)] += temp;
    let temp = (r.row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    m[(1, 1)] += temp;
    let temp =
        (2.0 * rotation_differentials[0].row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(1, 1)] = temp;
}
#[inline]
fn compose_m_derivatives_stage_3(
    m: &mut Matrix3<f64>,
    moments: &BearingMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix3<f64>; 3],
) {
    let temp =
        (-2.0 * rotation_differentials[0].row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(1, 1)] += temp;
    let temp =
        (-2.0 * r.row(0) * moments.zx_f * rotation_differentials[0].row(2).transpose())[(0, 0)];
    derivatives[0][(1, 1)] += temp;
    let temp =
        (2.0 * rotation_differentials[0].row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    derivatives[0][(1, 1)] += temp;
    let temp =
        (2.0 * rotation_differentials[1].row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(1, 1)] = temp;
    let temp =
        (-2.0 * rotation_differentials[1].row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(1, 1)] += temp;
    let temp =
        (-2.0 * r.row(0) * moments.zx_f * rotation_differentials[1].row(2).transpose())[(0, 0)];
    derivatives[1][(1, 1)] += temp;
    let temp =
        (2.0 * rotation_differentials[1].row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    derivatives[1][(1, 1)] += temp;
    let temp =
        (2.0 * rotation_differentials[2].row(0) * moments.zz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(1, 1)] = temp;
    let temp =
        (-2.0 * rotation_differentials[2].row(0) * moments.zx_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(1, 1)] += temp;
    let temp =
        (-2.0 * r.row(0) * moments.zx_f * rotation_differentials[2].row(2).transpose())[(0, 0)];
    derivatives[2][(1, 1)] += temp;
    let temp =
        (2.0 * rotation_differentials[2].row(2) * moments.xx_f * r.row(2).transpose())[(0, 0)];
    derivatives[2][(1, 1)] += temp;
    let temp = (r.row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    m[(1, 2)] = temp;
    let temp = (NEGATIVE_UNIT * r.row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    m[(1, 2)] += temp;
    let temp = (NEGATIVE_UNIT * r.row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    m[(1, 2)] += temp;
    let temp = (r.row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    m[(1, 2)] += temp;
    let temp = (rotation_differentials[0].row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] = temp;
    let temp = (r.row(0) * moments.zx_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
    let temp =
        (-2.0 * rotation_differentials[0].row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
    let temp = (-rotation_differentials[0].row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
    let temp = (-r.row(2) * moments.xx_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
    let temp = (rotation_differentials[0].row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[0].row(0).transpose())[(0, 0)];
    derivatives[0][(1, 2)] += temp;
    let temp = (rotation_differentials[1].row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] = temp;
}
#[inline]
fn compose_m_derivatives_stage_4(
    m: &mut Matrix3<f64>,
    moments: &BearingMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix3<f64>; 3],
) {
    let temp = (r.row(0) * moments.zx_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
    let temp =
        (-2.0 * rotation_differentials[1].row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
    let temp = (-rotation_differentials[1].row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
    let temp = (-r.row(2) * moments.xx_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
    let temp = (rotation_differentials[1].row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[1].row(0).transpose())[(0, 0)];
    derivatives[1][(1, 2)] += temp;
    let temp = (rotation_differentials[2].row(0) * moments.zx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] = temp;
    let temp = (r.row(0) * moments.zx_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
    let temp =
        (-2.0 * rotation_differentials[2].row(0) * moments.yz_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
    let temp = (-rotation_differentials[2].row(2) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
    let temp = (-r.row(2) * moments.xx_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
    let temp = (rotation_differentials[2].row(2) * moments.xy_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
    let temp = (r.row(2) * moments.xy_f * rotation_differentials[2].row(0).transpose())[(0, 0)];
    derivatives[2][(1, 2)] += temp;
    let temp = (r.row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    m[(2, 2)] = temp;
    let temp = (-2.0 * r.row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    m[(2, 2)] += temp;
    let temp = (r.row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    m[(2, 2)] += temp;
    let temp =
        (2.0 * rotation_differentials[0].row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(2, 2)] = temp;
    let temp =
        (-2.0 * rotation_differentials[0].row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[0][(2, 2)] += temp;
    let temp =
        (-2.0 * r.row(0) * moments.xy_f * rotation_differentials[0].row(1).transpose())[(0, 0)];
    derivatives[0][(2, 2)] += temp;
    let temp =
        (2.0 * rotation_differentials[0].row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[0][(2, 2)] += temp;
    let temp =
        (2.0 * rotation_differentials[1].row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(2, 2)] = temp;
    let temp =
        (-2.0 * rotation_differentials[1].row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[1][(2, 2)] += temp;
    let temp =
        (-2.0 * r.row(0) * moments.xy_f * rotation_differentials[1].row(1).transpose())[(0, 0)];
    derivatives[1][(2, 2)] += temp;
}
#[inline]
fn compose_m_derivatives_stage_5(
    m: &mut Matrix3<f64>,
    moments: &BearingMoments<'_>,
    r: &Matrix3<f64>,
    rotation_differentials: &[Matrix3<f64>; 3],
    derivatives: &mut [Matrix3<f64>; 3],
) {
    let temp =
        (2.0 * rotation_differentials[1].row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[1][(2, 2)] += temp;
    let temp =
        (2.0 * rotation_differentials[2].row(1) * moments.xx_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(2, 2)] = temp;
    let temp =
        (-2.0 * rotation_differentials[2].row(0) * moments.xy_f * r.row(1).transpose())[(0, 0)];
    derivatives[2][(2, 2)] += temp;
    let temp =
        (-2.0 * r.row(0) * moments.xy_f * rotation_differentials[2].row(1).transpose())[(0, 0)];
    derivatives[2][(2, 2)] += temp;
    let temp =
        (2.0 * rotation_differentials[2].row(0) * moments.yy_f * r.row(0).transpose())[(0, 0)];
    derivatives[2][(2, 2)] += temp;
    m[(1, 0)] = m[(0, 1)];
    m[(2, 0)] = m[(0, 2)];
    m[(2, 1)] = m[(1, 2)];
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
