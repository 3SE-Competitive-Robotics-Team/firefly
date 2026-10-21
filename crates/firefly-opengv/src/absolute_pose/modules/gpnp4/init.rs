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
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
    control_points: [&Vector3<f64>; 4],
) {
    let mut distances: [f64; 5] = [0.0; 5];
    let mut temp = control_points[0] - control_points[1];
    distances[0] = temp.norm() * temp.norm();
    temp = control_points[0] - control_points[2];
    distances[1] = temp.norm() * temp.norm();
    temp = control_points[0] - control_points[3];
    distances[2] = temp.norm() * temp.norm();
    temp = control_points[1] - control_points[2];
    distances[3] = temp.norm() * temp.norm();
    temp = control_points[1] - control_points[3];
    distances[4] = temp.norm() * temp.norm();
    init_stage_0(groebner_matrix, basis);
    init_stage_1(groebner_matrix, basis);
    init_stage_2(groebner_matrix, basis);
    init_stage_3(groebner_matrix, basis, &distances);
    init_stage_4(groebner_matrix, basis, &distances);
    init_stage_5(groebner_matrix, basis, &distances);
}
#[inline]
fn init_stage_0(groebner_matrix: &mut SMatrix<f64, 25, 37>, basis: [&SMatrix<f64, 12, 1>; 5]) {
    groebner_matrix[(0, 22)] = (((((((-2.0 * basis[4][(0, 0)] * basis[4][(3, 0)]
        - 2.0 * basis[4][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[4][(5, 0)])
        + basis[4][(0, 0)].powf(2.0))
        + basis[4][(3, 0)].powf(2.0))
        + basis[4][(1, 0)].powf(2.0))
        + basis[4][(4, 0)].powf(2.0))
        + basis[4][(2, 0)].powf(2.0))
        + basis[4][(5, 0)].powf(2.0);
    groebner_matrix[(0, 23)] = ((((((((((2.0 * basis[3][(0, 0)] * basis[4][(0, 0)]
        - 2.0 * basis[3][(0, 0)] * basis[4][(3, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[3][(3, 0)] * basis[4][(3, 0)])
        + 2.0 * basis[3][(1, 0)] * basis[4][(1, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[3][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[3][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[4][(5, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[3][(5, 0)] * basis[4][(5, 0)];
    groebner_matrix[(0, 24)] = (((((((-2.0 * basis[3][(0, 0)] * basis[3][(3, 0)]
        - 2.0 * basis[3][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(5, 0)])
        + basis[3][(0, 0)].powf(2.0))
        + basis[3][(3, 0)].powf(2.0))
        + basis[3][(1, 0)].powf(2.0))
        + basis[3][(4, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(5, 0)].powf(2.0);
    groebner_matrix[(0, 25)] = ((((((((((2.0 * basis[2][(2, 0)] * basis[4][(2, 0)]
        + 2.0 * basis[2][(0, 0)] * basis[4][(0, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[4][(3, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[4][(3, 0)])
        + 2.0 * basis[2][(1, 0)] * basis[4][(1, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[2][(4, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[4][(5, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[4][(5, 0)];
}

#[inline]
fn init_stage_1(groebner_matrix: &mut SMatrix<f64, 25, 37>, basis: [&SMatrix<f64, 12, 1>; 5]) {
    groebner_matrix[(0, 26)] = ((((((((((2.0 * basis[2][(2, 0)] * basis[3][(2, 0)]
        + 2.0 * basis[2][(0, 0)] * basis[3][(0, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[3][(3, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[2][(1, 0)] * basis[3][(1, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[2][(4, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(5, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)];
    groebner_matrix[(0, 27)] = (((((((-2.0 * basis[2][(0, 0)] * basis[2][(3, 0)]
        - 2.0 * basis[2][(1, 0)] * basis[2][(4, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(5, 0)])
        + basis[2][(0, 0)].powf(2.0))
        + basis[2][(3, 0)].powf(2.0))
        + basis[2][(1, 0)].powf(2.0))
        + basis[2][(4, 0)].powf(2.0))
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(5, 0)].powf(2.0);
    groebner_matrix[(0, 28)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[4][(0, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[4][(3, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[4][(3, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[4][(1, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[1][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[4][(5, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[4][(5, 0)];
    groebner_matrix[(0, 29)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[3][(0, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[3][(3, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[3][(1, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[1][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[3][(5, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[3][(5, 0)];
    groebner_matrix[(0, 30)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[2][(0, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[2][(3, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[2][(1, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[2][(4, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[1][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[2][(5, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[2][(5, 0)];
}

#[inline]
fn init_stage_2(groebner_matrix: &mut SMatrix<f64, 25, 37>, basis: [&SMatrix<f64, 12, 1>; 5]) {
    groebner_matrix[(0, 31)] = (((((((-2.0 * basis[1][(0, 0)] * basis[1][(3, 0)]
        - 2.0 * basis[1][(1, 0)] * basis[1][(4, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(5, 0)])
        + basis[1][(0, 0)].powf(2.0))
        + basis[1][(3, 0)].powf(2.0))
        + basis[1][(1, 0)].powf(2.0))
        + basis[1][(4, 0)].powf(2.0))
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(5, 0)].powf(2.0);
    groebner_matrix[(0, 32)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[4][(0, 0)]
        - 2.0 * basis[0][(0, 0)] * basis[4][(3, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[0][(3, 0)])
        + 2.0 * basis[0][(3, 0)] * basis[4][(3, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[4][(1, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[0][(4, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[4][(5, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[0][(5, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[4][(5, 0)];
    groebner_matrix[(0, 33)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[3][(0, 0)]
        - 2.0 * basis[0][(0, 0)] * basis[3][(3, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[0][(3, 0)])
        + 2.0 * basis[0][(3, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[3][(1, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[0][(4, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[3][(5, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[0][(5, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[3][(5, 0)];
    groebner_matrix[(0, 34)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
        - 2.0 * basis[0][(0, 0)] * basis[2][(3, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[0][(3, 0)])
        + 2.0 * basis[0][(3, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[2][(1, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[2][(4, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[0][(4, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[2][(5, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[0][(5, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[2][(5, 0)];
    groebner_matrix[(0, 35)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[1][(0, 0)]
        - 2.0 * basis[0][(0, 0)] * basis[1][(3, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[0][(3, 0)])
        + 2.0 * basis[0][(3, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[1][(1, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[1][(4, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[0][(4, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[1][(2, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[1][(5, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[0][(5, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[1][(5, 0)];
}

#[inline]
fn init_stage_3(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
    distances: &[f64; 5],
) {
    groebner_matrix[(0, 36)] = ((((((((-distances[0] + basis[0][(0, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(3, 0)])
        + basis[0][(3, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(4, 0)])
        + basis[0][(4, 0)].powf(2.0))
        + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(5, 0)])
        + basis[0][(5, 0)].powf(2.0);
    groebner_matrix[(1, 22)] = (((((((-2.0 * basis[4][(0, 0)] * basis[4][(6, 0)]
        - 2.0 * basis[4][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[4][(8, 0)])
        + basis[4][(0, 0)].powf(2.0))
        + basis[4][(1, 0)].powf(2.0))
        + basis[4][(2, 0)].powf(2.0))
        + basis[4][(6, 0)].powf(2.0))
        + basis[4][(7, 0)].powf(2.0))
        + basis[4][(8, 0)].powf(2.0);
    groebner_matrix[(1, 23)] = ((((((((((2.0 * basis[3][(0, 0)] * basis[4][(0, 0)]
        + 2.0 * basis[3][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[3][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[3][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[3][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[4][(8, 0)];
    groebner_matrix[(1, 24)] = (((((((-2.0 * basis[3][(0, 0)] * basis[3][(6, 0)]
        - 2.0 * basis[3][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(8, 0)])
        + basis[3][(0, 0)].powf(2.0))
        + basis[3][(1, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(6, 0)].powf(2.0))
        + basis[3][(7, 0)].powf(2.0))
        + basis[3][(8, 0)].powf(2.0);
    groebner_matrix[(1, 25)] = ((((((((((2.0 * basis[2][(2, 0)] * basis[4][(2, 0)]
        + 2.0 * basis[2][(0, 0)] * basis[4][(0, 0)])
        + 2.0 * basis[2][(1, 0)] * basis[4][(1, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[4][(8, 0)];
}

#[inline]
fn init_stage_4(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
    distances: &[f64; 5],
) {
    init_stage_4_section_0(groebner_matrix, basis);
    init_stage_4_section_1(groebner_matrix, basis);
    init_stage_4_section_2(groebner_matrix, basis, distances);
    init_stage_4_section_3(groebner_matrix, basis);
    init_stage_4_section_4(groebner_matrix, basis);
    init_stage_4_section_5(groebner_matrix, basis, distances);
    init_stage_4_section_6(groebner_matrix, basis);
    init_stage_4_section_7(groebner_matrix, basis);
    init_stage_4_section_8(groebner_matrix, basis, distances);
    init_stage_4_section_9(groebner_matrix, basis);
    init_stage_4_section_10(groebner_matrix, basis);
}
#[inline]
fn init_stage_4_section_0(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(1, 26)] = ((((((((((2.0 * basis[2][(2, 0)] * basis[3][(2, 0)]
        + 2.0 * basis[2][(0, 0)] * basis[3][(0, 0)])
        + 2.0 * basis[2][(1, 0)] * basis[3][(1, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[3][(8, 0)];
    groebner_matrix[(1, 27)] = (((((((-2.0 * basis[2][(0, 0)] * basis[2][(6, 0)]
        - 2.0 * basis[2][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(8, 0)])
        + basis[2][(0, 0)].powf(2.0))
        + basis[2][(1, 0)].powf(2.0))
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(6, 0)].powf(2.0))
        + basis[2][(7, 0)].powf(2.0))
        + basis[2][(8, 0)].powf(2.0);
    groebner_matrix[(1, 28)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[4][(0, 0)]
        + 2.0 * basis[1][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[4][(8, 0)];
    groebner_matrix[(1, 29)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[3][(0, 0)]
        + 2.0 * basis[1][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[3][(8, 0)];
    groebner_matrix[(1, 30)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[2][(0, 0)]
        + 2.0 * basis[1][(1, 0)] * basis[2][(1, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[2][(8, 0)];
}

#[inline]
fn init_stage_4_section_1(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(1, 31)] = (((((((-2.0 * basis[1][(0, 0)] * basis[1][(6, 0)]
        - 2.0 * basis[1][(1, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(8, 0)])
        + basis[1][(0, 0)].powf(2.0))
        + basis[1][(1, 0)].powf(2.0))
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(6, 0)].powf(2.0))
        + basis[1][(7, 0)].powf(2.0))
        + basis[1][(8, 0)].powf(2.0);
    groebner_matrix[(1, 32)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[4][(6, 0)]
        + 2.0 * basis[0][(0, 0)] * basis[4][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[4][(8, 0)];
    groebner_matrix[(1, 33)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[3][(6, 0)]
        + 2.0 * basis[0][(0, 0)] * basis[3][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[3][(8, 0)];
    groebner_matrix[(1, 34)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[2][(6, 0)]
        - 2.0 * basis[2][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(0, 0)] * basis[2][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[2][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[2][(2, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[2][(8, 0)];
    groebner_matrix[(1, 35)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[1][(6, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(0, 0)] * basis[1][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[1][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[1][(2, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[1][(8, 0)];
}

#[inline]
fn init_stage_4_section_2(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
    distances: &[f64; 5],
) {
    groebner_matrix[(1, 36)] = ((((((((-distances[1] + basis[0][(0, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0))
        + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(6, 0)])
        + basis[0][(6, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(7, 0)])
        + basis[0][(7, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(8, 0)])
        + basis[0][(8, 0)].powf(2.0);
    groebner_matrix[(2, 22)] = (((((((basis[4][(0, 0)].powf(2.0)
        + basis[4][(1, 0)].powf(2.0))
        + basis[4][(2, 0)].powf(2.0))
        - 2.0 * basis[4][(0, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[4][(11, 0)])
        + basis[4][(9, 0)].powf(2.0))
        + basis[4][(10, 0)].powf(2.0))
        + basis[4][(11, 0)].powf(2.0);
    groebner_matrix[(2, 23)] = ((((((((((2.0 * basis[3][(0, 0)] * basis[4][(0, 0)]
        + 2.0 * basis[3][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[3][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[3][(11, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[4][(11, 0)];
    groebner_matrix[(2, 24)] = (((((((basis[3][(0, 0)].powf(2.0)
        + basis[3][(1, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        - 2.0 * basis[3][(0, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(11, 0)])
        + basis[3][(9, 0)].powf(2.0))
        + basis[3][(10, 0)].powf(2.0))
        + basis[3][(11, 0)].powf(2.0);
    groebner_matrix[(2, 25)] = ((((((((((2.0 * basis[2][(2, 0)] * basis[4][(2, 0)]
        + 2.0 * basis[2][(0, 0)] * basis[4][(0, 0)])
        + 2.0 * basis[2][(1, 0)] * basis[4][(1, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[4][(11, 0)];
}

#[inline]
fn init_stage_4_section_3(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(2, 26)] = ((((((((((2.0 * basis[2][(2, 0)] * basis[3][(2, 0)]
        + 2.0 * basis[2][(0, 0)] * basis[3][(0, 0)])
        + 2.0 * basis[2][(1, 0)] * basis[3][(1, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[3][(11, 0)];
    groebner_matrix[(2, 27)] = (((((((basis[2][(0, 0)].powf(2.0)
        + basis[2][(1, 0)].powf(2.0))
        + basis[2][(2, 0)].powf(2.0))
        - 2.0 * basis[2][(0, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(11, 0)])
        + basis[2][(9, 0)].powf(2.0))
        + basis[2][(10, 0)].powf(2.0))
        + basis[2][(11, 0)].powf(2.0);
    groebner_matrix[(2, 28)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[4][(0, 0)]
        + 2.0 * basis[1][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[4][(11, 0)];
    groebner_matrix[(2, 29)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[3][(0, 0)]
        + 2.0 * basis[1][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[3][(11, 0)];
    groebner_matrix[(2, 30)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[2][(0, 0)]
        + 2.0 * basis[1][(1, 0)] * basis[2][(1, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[2][(11, 0)];
}

#[inline]
fn init_stage_4_section_4(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(2, 31)] = (((((((basis[1][(0, 0)].powf(2.0)
        + basis[1][(1, 0)].powf(2.0))
        + basis[1][(2, 0)].powf(2.0))
        - 2.0 * basis[1][(0, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(11, 0)])
        + basis[1][(9, 0)].powf(2.0))
        + basis[1][(10, 0)].powf(2.0))
        + basis[1][(11, 0)].powf(2.0);
    groebner_matrix[(2, 32)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[4][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[0][(9, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[4][(11, 0)];
    groebner_matrix[(2, 33)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[3][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[0][(9, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[3][(11, 0)];
    groebner_matrix[(2, 34)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[2][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[0][(9, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[2][(11, 0)];
    groebner_matrix[(2, 35)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[1][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[1][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[1][(2, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[0][(9, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[1][(11, 0)];
}

#[inline]
fn init_stage_4_section_5(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
    distances: &[f64; 5],
) {
    groebner_matrix[(2, 36)] = ((((((((-distances[2] + basis[0][(0, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0))
        + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(9, 0)])
        + basis[0][(9, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(10, 0)])
        + basis[0][(10, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(11, 0)])
        + basis[0][(11, 0)].powf(2.0);
    groebner_matrix[(3, 22)] = (((((((basis[4][(3, 0)].powf(2.0)
        + basis[4][(4, 0)].powf(2.0))
        + basis[4][(5, 0)].powf(2.0))
        + basis[4][(6, 0)].powf(2.0))
        + basis[4][(7, 0)].powf(2.0))
        + basis[4][(8, 0)].powf(2.0))
        - 2.0 * basis[4][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[4][(8, 0)];
    groebner_matrix[(3, 23)] = ((((((((((2.0 * basis[3][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[3][(4, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[3][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[3][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[3][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[3][(8, 0)];
    groebner_matrix[(3, 24)] = (((((((basis[3][(3, 0)].powf(2.0)
        + basis[3][(4, 0)].powf(2.0))
        + basis[3][(5, 0)].powf(2.0))
        + basis[3][(6, 0)].powf(2.0))
        + basis[3][(7, 0)].powf(2.0))
        + basis[3][(8, 0)].powf(2.0))
        - 2.0 * basis[3][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[3][(8, 0)];
    groebner_matrix[(3, 25)] = ((((((((((2.0 * basis[2][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[2][(4, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[2][(8, 0)];
}

#[inline]
fn init_stage_4_section_6(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(3, 26)] = ((((((((((2.0 * basis[2][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[2][(4, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[2][(8, 0)];
    groebner_matrix[(3, 27)] = (((((((basis[2][(3, 0)].powf(2.0)
        + basis[2][(4, 0)].powf(2.0))
        + basis[2][(5, 0)].powf(2.0))
        + basis[2][(6, 0)].powf(2.0))
        + basis[2][(7, 0)].powf(2.0))
        + basis[2][(8, 0)].powf(2.0))
        - 2.0 * basis[2][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[2][(8, 0)];
    groebner_matrix[(3, 28)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[1][(8, 0)];
    groebner_matrix[(3, 29)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[1][(8, 0)];
    groebner_matrix[(3, 30)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[2][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[2][(4, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[1][(8, 0)];
}

#[inline]
fn init_stage_4_section_7(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(3, 31)] = (((((((-2.0 * basis[1][(3, 0)] * basis[1][(6, 0)]
        + basis[1][(3, 0)].powf(2.0))
        + basis[1][(4, 0)].powf(2.0))
        + basis[1][(5, 0)].powf(2.0))
        + basis[1][(6, 0)].powf(2.0))
        + basis[1][(7, 0)].powf(2.0))
        + basis[1][(8, 0)].powf(2.0))
        - 2.0 * basis[1][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[1][(8, 0)];
    groebner_matrix[(3, 32)] = ((((((((((-2.0 * basis[0][(3, 0)] * basis[4][(6, 0)]
        + 2.0 * basis[0][(3, 0)] * basis[4][(3, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(3, 33)] = ((((((((((-2.0 * basis[0][(3, 0)] * basis[3][(6, 0)]
        + 2.0 * basis[0][(3, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(3, 34)] = ((((((((((-2.0 * basis[0][(3, 0)] * basis[2][(6, 0)]
        + 2.0 * basis[0][(3, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(3, 35)] = ((((((((((-2.0 * basis[1][(3, 0)] * basis[0][(6, 0)]
        - 2.0 * basis[0][(3, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[0][(3, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[0][(8, 0)];
}

#[inline]
fn init_stage_4_section_8(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
    distances: &[f64; 5],
) {
    groebner_matrix[(3, 36)] = ((((((((-distances[3] + basis[0][(3, 0)].powf(2.0))
        + basis[0][(4, 0)].powf(2.0))
        + basis[0][(5, 0)].powf(2.0))
        + basis[0][(6, 0)].powf(2.0))
        + basis[0][(7, 0)].powf(2.0))
        + basis[0][(8, 0)].powf(2.0))
        - 2.0 * basis[0][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(4, 22)] = (((((((basis[4][(3, 0)].powf(2.0)
        + basis[4][(4, 0)].powf(2.0))
        + basis[4][(5, 0)].powf(2.0))
        - 2.0 * basis[4][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[4][(11, 0)])
        + basis[4][(9, 0)].powf(2.0))
        + basis[4][(10, 0)].powf(2.0))
        + basis[4][(11, 0)].powf(2.0);
    groebner_matrix[(4, 23)] = ((((((((((2.0 * basis[3][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[3][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[3][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[3][(11, 0)];
    groebner_matrix[(4, 24)] = (((((((basis[3][(3, 0)].powf(2.0)
        + basis[3][(4, 0)].powf(2.0))
        + basis[3][(5, 0)].powf(2.0))
        - 2.0 * basis[3][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[3][(11, 0)])
        + basis[3][(9, 0)].powf(2.0))
        + basis[3][(10, 0)].powf(2.0))
        + basis[3][(11, 0)].powf(2.0);
    groebner_matrix[(4, 25)] = ((((((((((2.0 * basis[2][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[2][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[2][(11, 0)];
}

#[inline]
fn init_stage_4_section_9(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(4, 26)] = ((((((((((2.0 * basis[2][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[2][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[2][(11, 0)];
    groebner_matrix[(4, 27)] = (((((((basis[2][(3, 0)].powf(2.0)
        + basis[2][(4, 0)].powf(2.0))
        + basis[2][(5, 0)].powf(2.0))
        - 2.0 * basis[2][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[2][(11, 0)])
        + basis[2][(9, 0)].powf(2.0))
        + basis[2][(10, 0)].powf(2.0))
        + basis[2][(11, 0)].powf(2.0);
    groebner_matrix[(4, 28)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[1][(11, 0)];
    groebner_matrix[(4, 29)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[1][(11, 0)];
    groebner_matrix[(4, 30)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[2][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[1][(11, 0)];
}

#[inline]
fn init_stage_4_section_10(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
) {
    groebner_matrix[(4, 31)] = (((((((basis[1][(3, 0)].powf(2.0)
        + basis[1][(4, 0)].powf(2.0))
        + basis[1][(5, 0)].powf(2.0))
        - 2.0 * basis[1][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[1][(11, 0)])
        + basis[1][(9, 0)].powf(2.0))
        + basis[1][(10, 0)].powf(2.0))
        + basis[1][(11, 0)].powf(2.0);
}

#[inline]
fn init_stage_5(
    groebner_matrix: &mut SMatrix<f64, 25, 37>,
    basis: [&SMatrix<f64, 12, 1>; 5],
    distances: &[f64; 5],
) {
    groebner_matrix[(4, 32)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[0][(11, 0)];
    groebner_matrix[(4, 33)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[0][(11, 0)];
    groebner_matrix[(4, 34)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[2][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[0][(11, 0)];
    groebner_matrix[(4, 35)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[1][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[0][(11, 0)];
    groebner_matrix[(4, 36)] = ((((((((-distances[4] + basis[0][(3, 0)].powf(2.0))
        + basis[0][(4, 0)].powf(2.0))
        + basis[0][(5, 0)].powf(2.0))
        + basis[0][(9, 0)].powf(2.0))
        + basis[0][(10, 0)].powf(2.0))
        + basis[0][(11, 0)].powf(2.0))
        - 2.0 * basis[0][(3, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[0][(11, 0)];
}
