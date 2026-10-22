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

use nalgebra::{SMatrix, Vector3};

pub fn init(
    groebner_matrix: &mut SMatrix<f64, 5, 3>,
    a: &SMatrix<f64, 12, 1>,
    n: &SMatrix<f64, 12, 1>,
    c0: &Vector3<f64>,
    c1: &Vector3<f64>,
    c2: &Vector3<f64>,
    c3: &Vector3<f64>,
) {
    let mut temp = c0 - c1;
    let c01w = temp.norm() * temp.norm();
    temp = c0 - c2;
    let c02w = temp.norm() * temp.norm();
    temp = c0 - c3;
    let c03w = temp.norm() * temp.norm();

    groebner_matrix[(0, 0)] =
        -2.0 * n[(0, 0)] * n[(3, 0)] - 2.0 * n[(1, 0)] * n[(4, 0)] - 2.0 * n[(2, 0)] * n[(5, 0)]
            + n[(0, 0)].powf(2.0)
            + n[(3, 0)].powf(2.0)
            + n[(1, 0)].powf(2.0)
            + n[(4, 0)].powf(2.0)
            + n[(2, 0)].powf(2.0)
            + n[(5, 0)].powf(2.0);
    groebner_matrix[(0, 1)] =
        2.0 * a[(0, 0)] * n[(0, 0)] - 2.0 * a[(0, 0)] * n[(3, 0)] - 2.0 * n[(0, 0)] * a[(3, 0)]
            + 2.0 * a[(3, 0)] * n[(3, 0)]
            + 2.0 * a[(1, 0)] * n[(1, 0)]
            - 2.0 * a[(1, 0)] * n[(4, 0)]
            - 2.0 * n[(1, 0)] * a[(4, 0)]
            + 2.0 * a[(4, 0)] * n[(4, 0)]
            + 2.0 * a[(2, 0)] * n[(2, 0)]
            - 2.0 * a[(2, 0)] * n[(5, 0)]
            - 2.0 * n[(2, 0)] * a[(5, 0)]
            + 2.0 * a[(5, 0)] * n[(5, 0)];
    groebner_matrix[(0, 2)] = -c01w + a[(0, 0)].powf(2.0) - 2.0 * a[(0, 0)] * a[(3, 0)]
        + a[(3, 0)].powf(2.0)
        + a[(1, 0)].powf(2.0)
        - 2.0 * a[(1, 0)] * a[(4, 0)]
        + a[(4, 0)].powf(2.0)
        + a[(2, 0)].powf(2.0)
        - 2.0 * a[(2, 0)] * a[(5, 0)]
        + a[(5, 0)].powf(2.0);
    groebner_matrix[(1, 0)] =
        -2.0 * n[(0, 0)] * n[(6, 0)] - 2.0 * n[(1, 0)] * n[(7, 0)] - 2.0 * n[(2, 0)] * n[(8, 0)]
            + n[(0, 0)].powf(2.0)
            + n[(1, 0)].powf(2.0)
            + n[(2, 0)].powf(2.0)
            + n[(6, 0)].powf(2.0)
            + n[(7, 0)].powf(2.0)
            + n[(8, 0)].powf(2.0);
    groebner_matrix[(1, 1)] =
        2.0 * a[(0, 0)] * n[(0, 0)] + 2.0 * a[(1, 0)] * n[(1, 0)] + 2.0 * a[(2, 0)] * n[(2, 0)]
            - 2.0 * a[(0, 0)] * n[(6, 0)]
            - 2.0 * n[(0, 0)] * a[(6, 0)]
            + 2.0 * a[(6, 0)] * n[(6, 0)]
            - 2.0 * a[(1, 0)] * n[(7, 0)]
            - 2.0 * n[(1, 0)] * a[(7, 0)]
            + 2.0 * a[(7, 0)] * n[(7, 0)]
            - 2.0 * a[(2, 0)] * n[(8, 0)]
            - 2.0 * n[(2, 0)] * a[(8, 0)]
            + 2.0 * a[(8, 0)] * n[(8, 0)];
    groebner_matrix[(1, 2)] =
        -c02w + a[(0, 0)].powf(2.0) + a[(1, 0)].powf(2.0) + a[(2, 0)].powf(2.0)
            - 2.0 * a[(0, 0)] * a[(6, 0)]
            + a[(6, 0)].powf(2.0)
            - 2.0 * a[(1, 0)] * a[(7, 0)]
            + a[(7, 0)].powf(2.0)
            - 2.0 * a[(2, 0)] * a[(8, 0)]
            + a[(8, 0)].powf(2.0);
    groebner_matrix[(2, 0)] =
        -2.0 * n[(0, 0)] * n[(9, 0)] - 2.0 * n[(1, 0)] * n[(10, 0)] - 2.0 * n[(2, 0)] * n[(11, 0)]
            + n[(0, 0)].powf(2.0)
            + n[(1, 0)].powf(2.0)
            + n[(2, 0)].powf(2.0)
            + n[(9, 0)].powf(2.0)
            + n[(10, 0)].powf(2.0)
            + n[(11, 0)].powf(2.0);
    groebner_matrix[(2, 1)] =
        2.0 * a[(0, 0)] * n[(0, 0)] + 2.0 * a[(1, 0)] * n[(1, 0)] + 2.0 * a[(2, 0)] * n[(2, 0)]
            - 2.0 * a[(0, 0)] * n[(9, 0)]
            - 2.0 * n[(0, 0)] * a[(9, 0)]
            + 2.0 * a[(9, 0)] * n[(9, 0)]
            - 2.0 * a[(1, 0)] * n[(10, 0)]
            - 2.0 * n[(1, 0)] * a[(10, 0)]
            + 2.0 * a[(10, 0)] * n[(10, 0)]
            - 2.0 * a[(2, 0)] * n[(11, 0)]
            - 2.0 * n[(2, 0)] * a[(11, 0)]
            + 2.0 * a[(11, 0)] * n[(11, 0)];
    groebner_matrix[(2, 2)] =
        -c03w + a[(0, 0)].powf(2.0) + a[(1, 0)].powf(2.0) + a[(2, 0)].powf(2.0)
            - 2.0 * a[(0, 0)] * a[(9, 0)]
            + a[(9, 0)].powf(2.0)
            - 2.0 * a[(1, 0)] * a[(10, 0)]
            + a[(10, 0)].powf(2.0)
            - 2.0 * a[(2, 0)] * a[(11, 0)]
            + a[(11, 0)].powf(2.0);
}
