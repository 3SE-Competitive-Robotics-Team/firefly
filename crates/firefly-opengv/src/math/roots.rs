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

//! Closed form polynomial roots; the reference returns every complex root's real part.
use nalgebra::{Complex, DMatrix};
use std::f64::consts::PI;
#[must_use]
pub fn o3_roots(point: &[f64]) -> Vec<f64> {
    let (leading, quadratic, linear, constant) = (point[0], point[1], point[2], point[3]);
    let theta0 = quadratic * quadratic - 3.0 * leading * linear;
    let theta1 = 2.0 * quadratic * quadratic * quadratic - 9.0 * leading * quadratic * linear
        + 27.0 * leading * leading * constant;
    let term = theta1 * theta1 - 4.0 * theta0 * theta0 * theta0;
    let linear = if term >= 0.0 {
        let discriminant_sum = theta1 + term.sqrt();
        let c3 = 0.5 * discriminant_sum;
        Complex::new(
            if c3 < 0.0 {
                -(-c3).powf(1.0 / 3.0)
            } else {
                c3.powf(1.0 / 3.0)
            },
            0.0,
        )
    } else {
        let c3 = Complex::new(0.5 * theta1, 0.5 * (-term).sqrt());
        let r3 = (c3.re.powi(2) + c3.im.powi(2)).sqrt();
        let mut a3 = (c3.im / c3.re).atan();
        if c3.re < 0.0 {
            a3 += PI;
        }
        let rotation = r3.powf(1.0 / 3.0);
        Complex::new(rotation * (a3 / 3.0).cos(), rotation * (a3 / 3.0).sin())
    };
    [
        Complex::new(1.0, 0.0),
        Complex::new(-0.5, 0.5 * 3.0_f64.sqrt()),
        Complex::new(-0.5, -0.5 * 3.0_f64.sqrt()),
    ]
    .iter()
    .map(|left_vectors| {
        (-(quadratic + left_vectors * linear + theta0 / (left_vectors * linear)) / (3.0 * leading))
            .re
    })
    .collect()
}
#[must_use]
pub fn o4_roots_matrix(p: &DMatrix<f64>) -> Vec<f64> {
    o4_roots(&[p[(0, 0)], p[(1, 0)], p[(2, 0)], p[(3, 0)], p[(4, 0)]])
}
#[must_use]
pub fn o4_roots(point: &[f64]) -> Vec<f64> {
    let (leading, quadratic, linear, constant, quartic_offset) =
        (point[0], point[1], point[2], point[3], point[4]);
    let (a2, b2) = (leading * leading, quadratic * quadratic);
    let (a3, b3) = (a2 * leading, b2 * quadratic);
    let (a4, b4) = (a3 * leading, b3 * quadratic);
    let alpha = -3.0 * b2 / (8.0 * a2) + linear / leading;
    let beta = b3 / (8.0 * a3) - quadratic * linear / (2.0 * a2) + constant / leading;
    let gamma = -3.0 * b4 / (256.0 * a4) + b2 * linear / (16.0 * a3)
        - quadratic * constant / (4.0 * a2)
        + quartic_offset / leading;
    let alpha2 = alpha * alpha;
    let alpha3 = alpha2 * alpha;
    let point = Complex::new(-alpha2 / 12.0 - gamma, 0.0);
    let quaternion = Complex::new(
        -alpha3 / 108.0 + alpha * gamma / 3.0 - beta.powi(2) / 8.0,
        0.0,
    );
    let rotation = -quaternion / 2.0 + (quaternion.powf(2.0) / 4.0 + point.powf(3.0) / 27.0).sqrt();
    let left_vectors = rotation.powf(1.0 / 3.0);
    let y = if left_vectors.re == 0.0 {
        -5.0 * alpha / 6.0 - quaternion.powf(1.0 / 3.0)
    } else {
        -5.0 * alpha / 6.0 - point / (3.0 * left_vectors) + left_vectors
    };
    let root_weight = (alpha + 2.0 * y).sqrt();
    let plus = (-(3.0 * alpha + 2.0 * y + 2.0 * beta / root_weight)).sqrt();
    let minus = (-(3.0 * alpha + 2.0 * y - 2.0 * beta / root_weight)).sqrt();
    [
        -quadratic / (4.0 * leading) + 0.5 * (root_weight + plus),
        -quadratic / (4.0 * leading) + 0.5 * (root_weight - plus),
        -quadratic / (4.0 * leading) + 0.5 * (-root_weight + minus),
        -quadratic / (4.0 * leading) + 0.5 * (-root_weight - minus),
    ]
    .iter()
    .map(|x| x.re)
    .collect()
}
