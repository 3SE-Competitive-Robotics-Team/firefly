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

use nalgebra::SMatrix;

pub fn init(
    groebner_matrix: &mut SMatrix<f64, 48, 85>,
    f: &SMatrix<f64, 3, 3>,
    v: &SMatrix<f64, 3, 3>,
    p: &SMatrix<f64, 3, 3>,
) {
    init_stage_0(groebner_matrix, f, v, p);
    init_stage_1(groebner_matrix, f, v, p);
    init_stage_2(groebner_matrix, f, v, p);
}
#[inline]
fn init_stage_0(
    groebner_matrix: &mut SMatrix<f64, 48, 85>,
    f: &SMatrix<f64, 3, 3>,
    v: &SMatrix<f64, 3, 3>,
    p: &SMatrix<f64, 3, 3>,
) {
    groebner_matrix[(0, 20)] = NEGATIVE_UNIT * f[(0, 1)];
    groebner_matrix[(0, 21)] = f[(0, 0)];
    groebner_matrix[(0, 32)] = NEGATIVE_UNIT * f[(0, 1)];
    groebner_matrix[(0, 33)] = f[(0, 0)];
    groebner_matrix[(0, 52)] = NEGATIVE_UNIT * f[(0, 1)];
    groebner_matrix[(0, 53)] = f[(0, 0)];
    groebner_matrix[(0, 66)] = ((v[(0, 0)] - v[(0, 1)]) + p[(0, 0)]) - p[(0, 1)];
    groebner_matrix[(0, 71)] = ((v[(0, 0)] - v[(0, 1)]) + p[(0, 0)]) - p[(0, 1)];
    groebner_matrix[(0, 75)] = -2.0 * p[(2, 0)] + 2.0 * p[(2, 1)];
    groebner_matrix[(0, 76)] = -2.0 * p[(1, 0)] + 2.0 * p[(1, 1)];
    groebner_matrix[(0, 77)] = ((v[(0, 0)] - v[(0, 1)]) - p[(0, 0)]) + p[(0, 1)];
    groebner_matrix[(0, 79)] = NEGATIVE_UNIT * f[(0, 1)];
    groebner_matrix[(0, 80)] = f[(0, 0)];
    groebner_matrix[(0, 81)] = 2.0 * p[(1, 0)] - 2.0 * p[(1, 1)];
    groebner_matrix[(0, 82)] = -2.0 * p[(2, 0)] + 2.0 * p[(2, 1)];
    groebner_matrix[(0, 84)] = ((-p[(0, 0)] + p[(0, 1)]) + v[(0, 0)]) - v[(0, 1)];
    groebner_matrix[(1, 20)] = NEGATIVE_UNIT * f[(1, 1)];
    groebner_matrix[(1, 21)] = f[(1, 0)];
    groebner_matrix[(1, 32)] = NEGATIVE_UNIT * f[(1, 1)];
    groebner_matrix[(1, 33)] = f[(1, 0)];
    groebner_matrix[(1, 52)] = NEGATIVE_UNIT * f[(1, 1)];
    groebner_matrix[(1, 53)] = f[(1, 0)];
    groebner_matrix[(1, 66)] = ((v[(1, 0)] - v[(1, 1)]) + p[(1, 0)]) - p[(1, 1)];
    groebner_matrix[(1, 70)] = -2.0 * p[(2, 0)] + 2.0 * p[(2, 1)];
    groebner_matrix[(1, 71)] = ((v[(1, 0)] - v[(1, 1)]) - p[(1, 0)]) + p[(1, 1)];
    groebner_matrix[(1, 76)] = 2.0 * p[(0, 1)] - 2.0 * p[(0, 0)];
    groebner_matrix[(1, 77)] = ((v[(1, 0)] - v[(1, 1)]) + p[(1, 0)]) - p[(1, 1)];
    groebner_matrix[(1, 79)] = NEGATIVE_UNIT * f[(1, 1)];
    groebner_matrix[(1, 80)] = f[(1, 0)];
    groebner_matrix[(1, 81)] = -2.0 * p[(0, 0)] + 2.0 * p[(0, 1)];
    groebner_matrix[(1, 83)] = 2.0 * p[(2, 0)] - 2.0 * p[(2, 1)];
    groebner_matrix[(1, 84)] = ((-p[(1, 0)] + p[(1, 1)]) + v[(1, 0)]) - v[(1, 1)];
    groebner_matrix[(2, 20)] = NEGATIVE_UNIT * f[(2, 1)];
    groebner_matrix[(2, 21)] = f[(2, 0)];
    groebner_matrix[(2, 32)] = NEGATIVE_UNIT * f[(2, 1)];
    groebner_matrix[(2, 33)] = f[(2, 0)];
    groebner_matrix[(2, 52)] = NEGATIVE_UNIT * f[(2, 1)];
    groebner_matrix[(2, 53)] = f[(2, 0)];
    groebner_matrix[(2, 66)] = ((v[(2, 0)] - v[(2, 1)]) - p[(2, 0)]) + p[(2, 1)];
    groebner_matrix[(2, 70)] = -2.0 * p[(1, 0)] + 2.0 * p[(1, 1)];
    groebner_matrix[(2, 71)] = ((v[(2, 0)] - v[(2, 1)]) + p[(2, 0)]) - p[(2, 1)];
    groebner_matrix[(2, 75)] = -2.0 * p[(0, 0)] + 2.0 * p[(0, 1)];
    groebner_matrix[(2, 77)] = ((v[(2, 0)] - v[(2, 1)]) + p[(2, 0)]) - p[(2, 1)];
    groebner_matrix[(2, 79)] = NEGATIVE_UNIT * f[(2, 1)];
    groebner_matrix[(2, 80)] = f[(2, 0)];
    groebner_matrix[(2, 82)] = 2.0 * p[(0, 0)] - 2.0 * p[(0, 1)];
    groebner_matrix[(2, 83)] = -2.0 * p[(1, 0)] + 2.0 * p[(1, 1)];
    groebner_matrix[(2, 84)] = ((-p[(2, 0)] + p[(2, 1)]) + v[(2, 0)]) - v[(2, 1)];
    groebner_matrix[(3, 19)] = NEGATIVE_UNIT * f[(0, 2)];
    groebner_matrix[(3, 20)] = f[(0, 1)];
    groebner_matrix[(3, 31)] = NEGATIVE_UNIT * f[(0, 2)];
    groebner_matrix[(3, 32)] = f[(0, 1)];
    groebner_matrix[(3, 51)] = NEGATIVE_UNIT * f[(0, 2)];
    groebner_matrix[(3, 52)] = f[(0, 1)];
    groebner_matrix[(3, 66)] = ((v[(0, 1)] + p[(0, 1)]) - v[(0, 2)]) - p[(0, 2)];
    groebner_matrix[(3, 71)] = ((v[(0, 1)] + p[(0, 1)]) - v[(0, 2)]) - p[(0, 2)];
    groebner_matrix[(3, 75)] = -2.0 * p[(2, 1)] + 2.0 * p[(2, 2)];
    groebner_matrix[(3, 76)] = -2.0 * p[(1, 1)] + 2.0 * p[(1, 2)];
    groebner_matrix[(3, 77)] = ((v[(0, 1)] - p[(0, 1)]) - v[(0, 2)]) + p[(0, 2)];
    groebner_matrix[(3, 78)] = NEGATIVE_UNIT * f[(0, 2)];
    groebner_matrix[(3, 79)] = f[(0, 1)];
    groebner_matrix[(3, 81)] = 2.0 * p[(1, 1)] - 2.0 * p[(1, 2)];
}

#[inline]
fn init_stage_1(
    groebner_matrix: &mut SMatrix<f64, 48, 85>,
    f: &SMatrix<f64, 3, 3>,
    v: &SMatrix<f64, 3, 3>,
    p: &SMatrix<f64, 3, 3>,
) {
    groebner_matrix[(3, 82)] = -2.0 * p[(2, 1)] + 2.0 * p[(2, 2)];
    groebner_matrix[(3, 84)] = ((-p[(0, 1)] + p[(0, 2)]) - v[(0, 2)]) + v[(0, 1)];
    groebner_matrix[(4, 19)] = NEGATIVE_UNIT * f[(1, 2)];
    groebner_matrix[(4, 20)] = f[(1, 1)];
    groebner_matrix[(4, 31)] = NEGATIVE_UNIT * f[(1, 2)];
    groebner_matrix[(4, 32)] = f[(1, 1)];
    groebner_matrix[(4, 51)] = NEGATIVE_UNIT * f[(1, 2)];
    groebner_matrix[(4, 52)] = f[(1, 1)];
    groebner_matrix[(4, 66)] = ((v[(1, 1)] + p[(1, 1)]) - v[(1, 2)]) - p[(1, 2)];
    groebner_matrix[(4, 70)] = 2.0 * p[(2, 2)] - 2.0 * p[(2, 1)];
    groebner_matrix[(4, 71)] = ((v[(1, 1)] - p[(1, 1)]) - v[(1, 2)]) + p[(1, 2)];
    groebner_matrix[(4, 76)] = -2.0 * p[(0, 1)] + 2.0 * p[(0, 2)];
    groebner_matrix[(4, 77)] = ((v[(1, 1)] + p[(1, 1)]) - v[(1, 2)]) - p[(1, 2)];
    groebner_matrix[(4, 78)] = NEGATIVE_UNIT * f[(1, 2)];
    groebner_matrix[(4, 79)] = f[(1, 1)];
    groebner_matrix[(4, 81)] = -2.0 * p[(0, 1)] + 2.0 * p[(0, 2)];
    groebner_matrix[(4, 83)] = 2.0 * p[(2, 1)] - 2.0 * p[(2, 2)];
    groebner_matrix[(4, 84)] = ((-p[(1, 1)] + p[(1, 2)]) - v[(1, 2)]) + v[(1, 1)];
    groebner_matrix[(5, 19)] = NEGATIVE_UNIT * f[(2, 2)];
    groebner_matrix[(5, 20)] = f[(2, 1)];
    groebner_matrix[(5, 31)] = NEGATIVE_UNIT * f[(2, 2)];
    groebner_matrix[(5, 32)] = f[(2, 1)];
    groebner_matrix[(5, 51)] = NEGATIVE_UNIT * f[(2, 2)];
    groebner_matrix[(5, 52)] = f[(2, 1)];
    groebner_matrix[(5, 66)] = ((v[(2, 1)] - p[(2, 1)]) - v[(2, 2)]) + p[(2, 2)];
    groebner_matrix[(5, 70)] = -2.0 * p[(1, 1)] + 2.0 * p[(1, 2)];
    groebner_matrix[(5, 71)] = ((v[(2, 1)] + p[(2, 1)]) - v[(2, 2)]) - p[(2, 2)];
    groebner_matrix[(5, 75)] = -2.0 * p[(0, 1)] + 2.0 * p[(0, 2)];
    groebner_matrix[(5, 77)] = ((v[(2, 1)] + p[(2, 1)]) - v[(2, 2)]) - p[(2, 2)];
    groebner_matrix[(5, 78)] = NEGATIVE_UNIT * f[(2, 2)];
    groebner_matrix[(5, 79)] = f[(2, 1)];
    groebner_matrix[(5, 82)] = 2.0 * p[(0, 1)] - 2.0 * p[(0, 2)];
    groebner_matrix[(5, 83)] = -2.0 * p[(1, 1)] + 2.0 * p[(1, 2)];
    groebner_matrix[(5, 84)] = ((-p[(2, 1)] + p[(2, 2)]) - v[(2, 2)]) + v[(2, 1)];
    groebner_matrix[(6, 19)] = f[(0, 2)];
    groebner_matrix[(6, 21)] = NEGATIVE_UNIT * f[(0, 0)];
    groebner_matrix[(6, 31)] = f[(0, 2)];
    groebner_matrix[(6, 33)] = NEGATIVE_UNIT * f[(0, 0)];
    groebner_matrix[(6, 51)] = f[(0, 2)];
    groebner_matrix[(6, 53)] = NEGATIVE_UNIT * f[(0, 0)];
    groebner_matrix[(6, 66)] = ((-v[(0, 0)] - p[(0, 0)]) + v[(0, 2)]) + p[(0, 2)];
    groebner_matrix[(6, 71)] = ((-v[(0, 0)] - p[(0, 0)]) + v[(0, 2)]) + p[(0, 2)];
    groebner_matrix[(6, 75)] = 2.0 * p[(2, 0)] - 2.0 * p[(2, 2)];
    groebner_matrix[(6, 76)] = 2.0 * p[(1, 0)] - 2.0 * p[(1, 2)];
    groebner_matrix[(6, 77)] = ((-v[(0, 0)] + p[(0, 0)]) + v[(0, 2)]) - p[(0, 2)];
    groebner_matrix[(6, 78)] = f[(0, 2)];
    groebner_matrix[(6, 80)] = NEGATIVE_UNIT * f[(0, 0)];
    groebner_matrix[(6, 81)] = -2.0 * p[(1, 0)] + 2.0 * p[(1, 2)];
    groebner_matrix[(6, 82)] = 2.0 * p[(2, 0)] - 2.0 * p[(2, 2)];
    groebner_matrix[(6, 84)] = ((p[(0, 0)] - p[(0, 2)]) - v[(0, 0)]) + v[(0, 2)];
    groebner_matrix[(7, 19)] = f[(1, 2)];
    groebner_matrix[(7, 21)] = NEGATIVE_UNIT * f[(1, 0)];
    groebner_matrix[(7, 31)] = f[(1, 2)];
    groebner_matrix[(7, 33)] = NEGATIVE_UNIT * f[(1, 0)];
    groebner_matrix[(7, 51)] = f[(1, 2)];
    groebner_matrix[(7, 53)] = NEGATIVE_UNIT * f[(1, 0)];
    groebner_matrix[(7, 66)] = ((-v[(1, 0)] - p[(1, 0)]) + v[(1, 2)]) + p[(1, 2)];
    groebner_matrix[(7, 70)] = -2.0 * p[(2, 2)] + 2.0 * p[(2, 0)];
    groebner_matrix[(7, 71)] = ((-v[(1, 0)] + p[(1, 0)]) + v[(1, 2)]) - p[(1, 2)];
    groebner_matrix[(7, 76)] = 2.0 * p[(0, 0)] - 2.0 * p[(0, 2)];
    groebner_matrix[(7, 77)] = ((-v[(1, 0)] - p[(1, 0)]) + v[(1, 2)]) + p[(1, 2)];
    groebner_matrix[(7, 78)] = f[(1, 2)];
}

#[inline]
fn init_stage_2(
    groebner_matrix: &mut SMatrix<f64, 48, 85>,
    f: &SMatrix<f64, 3, 3>,
    v: &SMatrix<f64, 3, 3>,
    p: &SMatrix<f64, 3, 3>,
) {
    groebner_matrix[(7, 80)] = NEGATIVE_UNIT * f[(1, 0)];
    groebner_matrix[(7, 81)] = 2.0 * p[(0, 0)] - 2.0 * p[(0, 2)];
    groebner_matrix[(7, 83)] = -2.0 * p[(2, 0)] + 2.0 * p[(2, 2)];
    groebner_matrix[(7, 84)] = ((p[(1, 0)] - p[(1, 2)]) - v[(1, 0)]) + v[(1, 2)];
    groebner_matrix[(8, 19)] = f[(2, 2)];
    groebner_matrix[(8, 21)] = NEGATIVE_UNIT * f[(2, 0)];
    groebner_matrix[(8, 31)] = f[(2, 2)];
    groebner_matrix[(8, 33)] = NEGATIVE_UNIT * f[(2, 0)];
    groebner_matrix[(8, 51)] = f[(2, 2)];
    groebner_matrix[(8, 53)] = NEGATIVE_UNIT * f[(2, 0)];
    groebner_matrix[(8, 66)] = ((-v[(2, 0)] + p[(2, 0)]) + v[(2, 2)]) - p[(2, 2)];
    groebner_matrix[(8, 70)] = 2.0 * p[(1, 0)] - 2.0 * p[(1, 2)];
    groebner_matrix[(8, 71)] = ((-v[(2, 0)] - p[(2, 0)]) + v[(2, 2)]) + p[(2, 2)];
    groebner_matrix[(8, 75)] = 2.0 * p[(0, 0)] - 2.0 * p[(0, 2)];
    groebner_matrix[(8, 77)] = ((-v[(2, 0)] - p[(2, 0)]) + v[(2, 2)]) + p[(2, 2)];
    groebner_matrix[(8, 78)] = f[(2, 2)];
    groebner_matrix[(8, 80)] = NEGATIVE_UNIT * f[(2, 0)];
    groebner_matrix[(8, 82)] = -2.0 * p[(0, 0)] + 2.0 * p[(0, 2)];
    groebner_matrix[(8, 83)] = 2.0 * p[(1, 0)] - 2.0 * p[(1, 2)];
    groebner_matrix[(8, 84)] = ((p[(2, 0)] - p[(2, 2)]) - v[(2, 0)]) + v[(2, 2)];
}
