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
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    control_points: [&Vector3<f64>; 4],
) {
    let mut distances: [f64; 6] = [0.0; 6];
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
    temp = control_points[2] - control_points[3];
    distances[5] = temp.norm() * temp.norm();
    init_stage_0(groebner_matrix, basis);
    init_stage_1(groebner_matrix, basis);
    init_stage_2(groebner_matrix, basis);
    init_stage_3(groebner_matrix, basis, &distances);
    init_stage_4(groebner_matrix, basis, &distances);
    init_stage_5(groebner_matrix, basis, &distances);
}
#[inline]
fn init_stage_0(groebner_matrix: &mut SMatrix<f64, 44, 80>, basis: [&SMatrix<f64, 12, 1>; 6]) {
    groebner_matrix[(0, 59)] = (((((((-2.0 * basis[5][(0, 0)] * basis[5][(3, 0)]
        + basis[5][(1, 0)].powf(2.0))
        + basis[5][(4, 0)].powf(2.0))
        + basis[5][(2, 0)].powf(2.0))
        + basis[5][(5, 0)].powf(2.0))
        + basis[5][(0, 0)].powf(2.0))
        + basis[5][(3, 0)].powf(2.0))
        - 2.0 * basis[5][(1, 0)] * basis[5][(4, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[5][(5, 0)];
    groebner_matrix[(0, 60)] = ((((((((((2.0 * basis[4][(1, 0)] * basis[5][(1, 0)]
        - 2.0 * basis[4][(1, 0)] * basis[5][(4, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[4][(4, 0)] * basis[5][(4, 0)])
        + 2.0 * basis[4][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[5][(5, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[4][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[4][(0, 0)] * basis[5][(0, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[5][(3, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[4][(3, 0)])
        + 2.0 * basis[4][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(0, 61)] = (((((((-2.0 * basis[4][(0, 0)] * basis[4][(3, 0)]
        + basis[4][(1, 0)].powf(2.0))
        + basis[4][(4, 0)].powf(2.0))
        + basis[4][(2, 0)].powf(2.0))
        + basis[4][(5, 0)].powf(2.0))
        + basis[4][(0, 0)].powf(2.0))
        + basis[4][(3, 0)].powf(2.0))
        - 2.0 * basis[4][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[4][(5, 0)];
    groebner_matrix[(0, 62)] = ((((((((((2.0 * basis[3][(1, 0)] * basis[5][(1, 0)]
        - 2.0 * basis[3][(1, 0)] * basis[5][(4, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[3][(4, 0)] * basis[5][(4, 0)])
        + 2.0 * basis[3][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[5][(5, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[3][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[3][(0, 0)] * basis[5][(0, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[5][(3, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[3][(3, 0)] * basis[5][(3, 0)];
}

#[inline]
fn init_stage_1(groebner_matrix: &mut SMatrix<f64, 44, 80>, basis: [&SMatrix<f64, 12, 1>; 6]) {
    groebner_matrix[(0, 63)] = ((((((((((2.0 * basis[3][(1, 0)] * basis[4][(1, 0)]
        - 2.0 * basis[3][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[3][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[3][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[4][(5, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[3][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[3][(0, 0)] * basis[4][(0, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[4][(3, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[3][(3, 0)] * basis[4][(3, 0)];
    groebner_matrix[(0, 64)] = (((((((-2.0 * basis[3][(0, 0)] * basis[3][(3, 0)]
        + basis[3][(1, 0)].powf(2.0))
        + basis[3][(4, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(5, 0)].powf(2.0))
        + basis[3][(0, 0)].powf(2.0))
        + basis[3][(3, 0)].powf(2.0))
        - 2.0 * basis[3][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(5, 0)];
    groebner_matrix[(0, 65)] = ((((((((((2.0 * basis[2][(1, 0)] * basis[5][(1, 0)]
        - 2.0 * basis[2][(1, 0)] * basis[5][(4, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[2][(4, 0)] * basis[5][(4, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[5][(5, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[5][(0, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[5][(3, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(0, 66)] = ((((((((((2.0 * basis[2][(1, 0)] * basis[4][(1, 0)]
        - 2.0 * basis[2][(1, 0)] * basis[4][(4, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[2][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[4][(5, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[4][(0, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[4][(3, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[4][(3, 0)];
    groebner_matrix[(0, 67)] = ((((((((((2.0 * basis[2][(1, 0)] * basis[3][(1, 0)]
        - 2.0 * basis[2][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[2][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(5, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[3][(0, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[3][(3, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[3][(3, 0)];
}

#[inline]
fn init_stage_2(groebner_matrix: &mut SMatrix<f64, 44, 80>, basis: [&SMatrix<f64, 12, 1>; 6]) {
    groebner_matrix[(0, 68)] = (((((((-2.0 * basis[2][(0, 0)] * basis[2][(3, 0)]
        + basis[2][(1, 0)].powf(2.0))
        + basis[2][(4, 0)].powf(2.0))
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(5, 0)].powf(2.0))
        + basis[2][(0, 0)].powf(2.0))
        + basis[2][(3, 0)].powf(2.0))
        - 2.0 * basis[2][(1, 0)] * basis[2][(4, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(5, 0)];
    groebner_matrix[(0, 69)] = ((((((((((-2.0 * basis[5][(1, 0)] * basis[1][(4, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[5][(4, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[5][(5, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[5][(0, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[5][(3, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[5][(3, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[5][(1, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[5][(4, 0)];
    groebner_matrix[(0, 70)] = ((((((((((-2.0 * basis[4][(1, 0)] * basis[1][(4, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[4][(5, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[4][(0, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[4][(3, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[4][(3, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[4][(1, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[4][(4, 0)];
    groebner_matrix[(0, 71)] = ((((((((((-2.0 * basis[3][(1, 0)] * basis[1][(4, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[3][(5, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[3][(0, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[3][(3, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[3][(1, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[3][(4, 0)];
    groebner_matrix[(0, 72)] = ((((((((((-2.0 * basis[2][(1, 0)] * basis[1][(4, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[2][(5, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[2][(0, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[2][(3, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[1][(3, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[2][(1, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[2][(4, 0)];
}

#[inline]
fn init_stage_3(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    init_stage_3_section_0(groebner_matrix, basis);
    init_stage_3_section_1(groebner_matrix, basis, distances);
    init_stage_3_section_2(groebner_matrix, basis);
}
#[inline]
fn init_stage_3_section_0(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(0, 73)] = (((((((-2.0 * basis[1][(0, 0)] * basis[1][(3, 0)]
        + basis[1][(1, 0)].powf(2.0))
        + basis[1][(4, 0)].powf(2.0))
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(5, 0)].powf(2.0))
        + basis[1][(0, 0)].powf(2.0))
        + basis[1][(3, 0)].powf(2.0))
        - 2.0 * basis[1][(1, 0)] * basis[1][(4, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(5, 0)];
    groebner_matrix[(0, 74)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[5][(0, 0)]
        - 2.0 * basis[0][(0, 0)] * basis[5][(3, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[0][(3, 0)])
        + 2.0 * basis[0][(3, 0)] * basis[5][(3, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[5][(1, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[5][(4, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[0][(4, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[5][(4, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[5][(5, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[0][(5, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[5][(5, 0)];
    groebner_matrix[(0, 75)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[4][(0, 0)]
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
    groebner_matrix[(0, 76)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[3][(0, 0)]
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
    groebner_matrix[(0, 77)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
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
}

#[inline]
fn init_stage_3_section_1(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    groebner_matrix[(0, 78)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[1][(0, 0)]
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
    groebner_matrix[(0, 79)] = ((((((((-distances[0] + basis[0][(4, 0)].powf(2.0))
        + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(5, 0)])
        + basis[0][(5, 0)].powf(2.0))
        + basis[0][(0, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(3, 0)])
        + basis[0][(3, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(4, 0)];
    groebner_matrix[(1, 59)] = (((((((basis[5][(1, 0)].powf(2.0)
        + basis[5][(2, 0)].powf(2.0))
        + basis[5][(6, 0)].powf(2.0))
        + basis[5][(7, 0)].powf(2.0))
        + basis[5][(8, 0)].powf(2.0))
        + basis[5][(0, 0)].powf(2.0))
        - 2.0 * basis[5][(0, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[5][(8, 0)];
    groebner_matrix[(1, 60)] = ((((((((((2.0 * basis[4][(1, 0)] * basis[5][(1, 0)]
        + 2.0 * basis[4][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[4][(6, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[4][(7, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[4][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[4][(0, 0)] * basis[5][(0, 0)];
    groebner_matrix[(1, 61)] = (((((((basis[4][(1, 0)].powf(2.0)
        + basis[4][(2, 0)].powf(2.0))
        + basis[4][(6, 0)].powf(2.0))
        + basis[4][(7, 0)].powf(2.0))
        + basis[4][(8, 0)].powf(2.0))
        + basis[4][(0, 0)].powf(2.0))
        - 2.0 * basis[4][(0, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[4][(8, 0)];
}

#[inline]
fn init_stage_3_section_2(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(1, 62)] = ((((((((((2.0 * basis[3][(1, 0)] * basis[5][(1, 0)]
        + 2.0 * basis[3][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[3][(6, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[3][(7, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[3][(0, 0)] * basis[5][(0, 0)];
}

#[inline]
fn init_stage_4(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    init_stage_4_section_0(groebner_matrix, basis);
    init_stage_4_section_1(groebner_matrix, basis);
    init_stage_4_section_2(groebner_matrix, basis);
    init_stage_4_section_3(groebner_matrix, basis, distances);
    init_stage_4_section_4(groebner_matrix, basis);
    init_stage_4_section_5(groebner_matrix, basis);
    init_stage_4_section_6(groebner_matrix, basis);
    init_stage_4_section_7(groebner_matrix, basis, distances);
    init_stage_4_section_8(groebner_matrix, basis);
    init_stage_4_section_9(groebner_matrix, basis);
    init_stage_4_section_10(groebner_matrix, basis);
    init_stage_4_section_11(groebner_matrix, basis, distances);
    init_stage_4_section_12(groebner_matrix, basis);
}
#[inline]
fn init_stage_4_section_0(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(1, 63)] = ((((((((((2.0 * basis[3][(1, 0)] * basis[4][(1, 0)]
        + 2.0 * basis[3][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[3][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[3][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[3][(0, 0)] * basis[4][(0, 0)];
    groebner_matrix[(1, 64)] = (((((((basis[3][(1, 0)].powf(2.0)
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(6, 0)].powf(2.0))
        + basis[3][(7, 0)].powf(2.0))
        + basis[3][(8, 0)].powf(2.0))
        + basis[3][(0, 0)].powf(2.0))
        - 2.0 * basis[3][(0, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(8, 0)];
    groebner_matrix[(1, 65)] = ((((((((((2.0 * basis[2][(1, 0)] * basis[5][(1, 0)]
        + 2.0 * basis[2][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[5][(0, 0)];
    groebner_matrix[(1, 66)] = ((((((((((2.0 * basis[2][(1, 0)] * basis[4][(1, 0)]
        + 2.0 * basis[2][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[4][(0, 0)];
    groebner_matrix[(1, 67)] = ((((((((((2.0 * basis[2][(1, 0)] * basis[3][(1, 0)]
        + 2.0 * basis[2][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[3][(0, 0)];
}

#[inline]
fn init_stage_4_section_1(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(1, 68)] = (((((((basis[2][(1, 0)].powf(2.0)
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(6, 0)].powf(2.0))
        + basis[2][(7, 0)].powf(2.0))
        + basis[2][(8, 0)].powf(2.0))
        + basis[2][(0, 0)].powf(2.0))
        - 2.0 * basis[2][(0, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(8, 0)];
    groebner_matrix[(1, 69)] = ((((((((((2.0 * basis[1][(2, 0)] * basis[5][(2, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[5][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[5][(1, 0)];
    groebner_matrix[(1, 70)] = ((((((((((2.0 * basis[1][(2, 0)] * basis[4][(2, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[4][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[4][(1, 0)];
    groebner_matrix[(1, 71)] = ((((((((((2.0 * basis[1][(2, 0)] * basis[3][(2, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[3][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[3][(1, 0)];
    groebner_matrix[(1, 72)] = ((((((((((2.0 * basis[1][(2, 0)] * basis[2][(2, 0)]
        - 2.0 * basis[1][(0, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[2][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[2][(1, 0)];
}

#[inline]
fn init_stage_4_section_2(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(1, 73)] = (((((((basis[1][(1, 0)].powf(2.0)
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(6, 0)].powf(2.0))
        + basis[1][(7, 0)].powf(2.0))
        + basis[1][(8, 0)].powf(2.0))
        + basis[1][(0, 0)].powf(2.0))
        - 2.0 * basis[1][(0, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(8, 0)];
    groebner_matrix[(1, 74)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[5][(6, 0)]
        + 2.0 * basis[0][(0, 0)] * basis[5][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[5][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[5][(8, 0)];
    groebner_matrix[(1, 75)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[4][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[4][(6, 0)];
    groebner_matrix[(1, 76)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[3][(6, 0)]
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
    groebner_matrix[(1, 77)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[2][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[2][(6, 0)];
}

#[inline]
fn init_stage_4_section_3(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    groebner_matrix[(1, 78)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[1][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[1][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[1][(2, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[1][(6, 0)];
    groebner_matrix[(1, 79)] = ((((((((-distances[1] + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(6, 0)])
        + basis[0][(6, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(7, 0)])
        + basis[0][(7, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(8, 0)])
        + basis[0][(8, 0)].powf(2.0))
        + basis[0][(0, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0);
    groebner_matrix[(2, 59)] = (((((((basis[5][(1, 0)].powf(2.0)
        + basis[5][(2, 0)].powf(2.0))
        + basis[5][(9, 0)].powf(2.0))
        + basis[5][(10, 0)].powf(2.0))
        + basis[5][(11, 0)].powf(2.0))
        + basis[5][(0, 0)].powf(2.0))
        - 2.0 * basis[5][(0, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[5][(11, 0)];
    groebner_matrix[(2, 60)] = ((((((((((2.0 * basis[4][(1, 0)] * basis[5][(1, 0)]
        + 2.0 * basis[4][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[4][(9, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[4][(10, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[4][(11, 0)])
        + 2.0 * basis[4][(11, 0)] * basis[5][(11, 0)])
        + 2.0 * basis[4][(0, 0)] * basis[5][(0, 0)];
    groebner_matrix[(2, 61)] = (((((((basis[4][(1, 0)].powf(2.0)
        + basis[4][(2, 0)].powf(2.0))
        + basis[4][(9, 0)].powf(2.0))
        + basis[4][(10, 0)].powf(2.0))
        + basis[4][(11, 0)].powf(2.0))
        + basis[4][(0, 0)].powf(2.0))
        - 2.0 * basis[4][(0, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[4][(11, 0)];
}

#[inline]
fn init_stage_4_section_4(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(2, 62)] = ((((((((((2.0 * basis[3][(1, 0)] * basis[5][(1, 0)]
        + 2.0 * basis[3][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[3][(11, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[5][(11, 0)])
        + 2.0 * basis[3][(0, 0)] * basis[5][(0, 0)];
    groebner_matrix[(2, 63)] = ((((((((((2.0 * basis[3][(1, 0)] * basis[4][(1, 0)]
        + 2.0 * basis[3][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[3][(11, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[4][(11, 0)])
        + 2.0 * basis[3][(0, 0)] * basis[4][(0, 0)];
    groebner_matrix[(2, 64)] = (((((((-2.0 * basis[3][(0, 0)] * basis[3][(9, 0)]
        + basis[3][(1, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(9, 0)].powf(2.0))
        + basis[3][(10, 0)].powf(2.0))
        + basis[3][(11, 0)].powf(2.0))
        + basis[3][(0, 0)].powf(2.0))
        - 2.0 * basis[3][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(11, 0)];
    groebner_matrix[(2, 65)] = ((((((((((-2.0 * basis[2][(0, 0)] * basis[5][(9, 0)]
        + 2.0 * basis[2][(1, 0)] * basis[5][(1, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[5][(11, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[5][(0, 0)];
    groebner_matrix[(2, 66)] = ((((((((((-2.0 * basis[2][(0, 0)] * basis[4][(9, 0)]
        + 2.0 * basis[2][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[4][(11, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[4][(0, 0)];
}

#[inline]
fn init_stage_4_section_5(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(2, 67)] = ((((((((((-2.0 * basis[2][(0, 0)] * basis[3][(9, 0)]
        + 2.0 * basis[2][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[3][(11, 0)])
        + 2.0 * basis[2][(0, 0)] * basis[3][(0, 0)];
    groebner_matrix[(2, 68)] = (((((((basis[2][(1, 0)].powf(2.0)
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(9, 0)].powf(2.0))
        + basis[2][(10, 0)].powf(2.0))
        + basis[2][(11, 0)].powf(2.0))
        + basis[2][(0, 0)].powf(2.0))
        - 2.0 * basis[2][(1, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[2][(9, 0)];
    groebner_matrix[(2, 69)] = ((((((((((-2.0 * basis[1][(0, 0)] * basis[5][(9, 0)]
        + 2.0 * basis[1][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[5][(11, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[5][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[5][(1, 0)];
    groebner_matrix[(2, 70)] = ((((((((((-2.0 * basis[1][(0, 0)] * basis[4][(9, 0)]
        + 2.0 * basis[1][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[4][(11, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[4][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[4][(1, 0)];
    groebner_matrix[(2, 71)] = ((((((((((-2.0 * basis[1][(0, 0)] * basis[3][(9, 0)]
        + 2.0 * basis[1][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[3][(11, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[3][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[3][(1, 0)];
}

#[inline]
fn init_stage_4_section_6(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(2, 72)] = ((((((((((-2.0 * basis[1][(0, 0)] * basis[2][(9, 0)]
        - 2.0 * basis[2][(0, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[1][(2, 0)] * basis[2][(2, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[1][(0, 0)] * basis[2][(0, 0)])
        + 2.0 * basis[1][(1, 0)] * basis[2][(1, 0)];
    groebner_matrix[(2, 73)] = (((((((basis[1][(1, 0)].powf(2.0)
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(9, 0)].powf(2.0))
        + basis[1][(10, 0)].powf(2.0))
        + basis[1][(11, 0)].powf(2.0))
        + basis[1][(0, 0)].powf(2.0))
        - 2.0 * basis[1][(1, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[1][(9, 0)];
    groebner_matrix[(2, 74)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[5][(9, 0)]
        + 2.0 * basis[0][(0, 0)] * basis[5][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[5][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[5][(2, 0)])
        - 2.0 * basis[5][(0, 0)] * basis[0][(9, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[5][(11, 0)];
    groebner_matrix[(2, 75)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[4][(9, 0)]
        + 2.0 * basis[0][(0, 0)] * basis[4][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[4][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[4][(2, 0)])
        - 2.0 * basis[4][(0, 0)] * basis[0][(9, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[4][(11, 0)];
    groebner_matrix[(2, 76)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[3][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[3][(2, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[3][(9, 0)];
}

#[inline]
fn init_stage_4_section_7(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    groebner_matrix[(2, 77)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[2][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[2][(2, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[2][(9, 0)];
    groebner_matrix[(2, 78)] = ((((((((((-2.0 * basis[0][(0, 0)] * basis[1][(9, 0)]
        + 2.0 * basis[0][(0, 0)] * basis[1][(0, 0)])
        + 2.0 * basis[0][(1, 0)] * basis[1][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[1][(2, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[0][(10, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[0][(11, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[0][(9, 0)];
    groebner_matrix[(2, 79)] = ((((((((-distances[2] + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(9, 0)])
        + basis[0][(9, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(10, 0)])
        + basis[0][(10, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(11, 0)])
        + basis[0][(11, 0)].powf(2.0))
        + basis[0][(0, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0);
    groebner_matrix[(3, 59)] = (((((((basis[5][(4, 0)].powf(2.0)
        + basis[5][(5, 0)].powf(2.0))
        + basis[5][(6, 0)].powf(2.0))
        + basis[5][(7, 0)].powf(2.0))
        + basis[5][(8, 0)].powf(2.0))
        + basis[5][(3, 0)].powf(2.0))
        - 2.0 * basis[5][(3, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[5][(8, 0)];
    groebner_matrix[(3, 60)] = ((((((((((2.0 * basis[4][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[4][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[4][(6, 0)] * basis[5][(6, 0)])
        + 2.0 * basis[4][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[4][(8, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[4][(3, 0)] * basis[5][(3, 0)];
}

#[inline]
fn init_stage_4_section_8(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(3, 61)] = (((((((basis[4][(4, 0)].powf(2.0)
        + basis[4][(5, 0)].powf(2.0))
        + basis[4][(6, 0)].powf(2.0))
        + basis[4][(7, 0)].powf(2.0))
        + basis[4][(8, 0)].powf(2.0))
        + basis[4][(3, 0)].powf(2.0))
        - 2.0 * basis[4][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[4][(8, 0)];
    groebner_matrix[(3, 62)] = ((((((((((2.0 * basis[3][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[3][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[3][(6, 0)] * basis[5][(6, 0)])
        + 2.0 * basis[3][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[3][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(3, 63)] = ((((((((((2.0 * basis[3][(4, 0)] * basis[4][(4, 0)]
        + 2.0 * basis[3][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[3][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[3][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[3][(3, 0)] * basis[4][(3, 0)];
    groebner_matrix[(3, 64)] = (((((((basis[3][(4, 0)].powf(2.0)
        + basis[3][(5, 0)].powf(2.0))
        + basis[3][(6, 0)].powf(2.0))
        + basis[3][(7, 0)].powf(2.0))
        + basis[3][(8, 0)].powf(2.0))
        + basis[3][(3, 0)].powf(2.0))
        - 2.0 * basis[3][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[3][(7, 0)];
    groebner_matrix[(3, 65)] = ((((((((((2.0 * basis[2][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[2][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[5][(6, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[5][(3, 0)];
}

#[inline]
fn init_stage_4_section_9(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(3, 66)] = ((((((((((2.0 * basis[2][(4, 0)] * basis[4][(4, 0)]
        + 2.0 * basis[2][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[4][(3, 0)];
    groebner_matrix[(3, 67)] = ((((((((((2.0 * basis[2][(4, 0)] * basis[3][(4, 0)]
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[3][(3, 0)];
    groebner_matrix[(3, 68)] = (((((((basis[2][(4, 0)].powf(2.0)
        + basis[2][(5, 0)].powf(2.0))
        + basis[2][(6, 0)].powf(2.0))
        + basis[2][(7, 0)].powf(2.0))
        + basis[2][(8, 0)].powf(2.0))
        + basis[2][(3, 0)].powf(2.0))
        - 2.0 * basis[2][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[2][(8, 0)];
    groebner_matrix[(3, 69)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[5][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[5][(6, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(3, 70)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[4][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[4][(6, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[4][(3, 0)];
}

#[inline]
fn init_stage_4_section_10(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(3, 71)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[3][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[3][(3, 0)];
    groebner_matrix[(3, 72)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[2][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[2][(3, 0)];
    groebner_matrix[(3, 73)] = (((((((basis[1][(4, 0)].powf(2.0)
        + basis[1][(5, 0)].powf(2.0))
        + basis[1][(6, 0)].powf(2.0))
        + basis[1][(7, 0)].powf(2.0))
        + basis[1][(8, 0)].powf(2.0))
        + basis[1][(3, 0)].powf(2.0))
        - 2.0 * basis[1][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[1][(8, 0)];
    groebner_matrix[(3, 74)] = ((((((((((-2.0 * basis[0][(3, 0)] * basis[5][(6, 0)]
        + 2.0 * basis[0][(3, 0)] * basis[5][(3, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[5][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[5][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[5][(7, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[5][(8, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(3, 75)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[4][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[4][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[4][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[4][(7, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[4][(8, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[0][(8, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[4][(6, 0)];
}

#[inline]
fn init_stage_4_section_11(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    groebner_matrix[(3, 76)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[3][(3, 0)]
        - 2.0 * basis[3][(5, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[3][(6, 0)];
    groebner_matrix[(3, 77)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[2][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[0][(8, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[2][(6, 0)];
    groebner_matrix[(3, 78)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[1][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[0][(8, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[1][(6, 0)];
    groebner_matrix[(3, 79)] = ((((((((-distances[3] + basis[0][(4, 0)].powf(2.0))
        + basis[0][(5, 0)].powf(2.0))
        + basis[0][(6, 0)].powf(2.0))
        + basis[0][(7, 0)].powf(2.0))
        + basis[0][(8, 0)].powf(2.0))
        - 2.0 * basis[0][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[0][(7, 0)])
        + basis[0][(3, 0)].powf(2.0))
        - 2.0 * basis[0][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(4, 59)] = (((((((basis[5][(4, 0)].powf(2.0)
        + basis[5][(5, 0)].powf(2.0))
        + basis[5][(9, 0)].powf(2.0))
        + basis[5][(10, 0)].powf(2.0))
        + basis[5][(11, 0)].powf(2.0))
        + basis[5][(3, 0)].powf(2.0))
        - 2.0 * basis[5][(3, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[5][(11, 0)];
}

#[inline]
fn init_stage_4_section_12(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(4, 60)] = ((((((((((2.0 * basis[4][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[4][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[4][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[4][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[4][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[4][(11, 0)])
        + 2.0 * basis[4][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(4, 61)] = (((((((basis[4][(4, 0)].powf(2.0)
        + basis[4][(5, 0)].powf(2.0))
        + basis[4][(9, 0)].powf(2.0))
        + basis[4][(10, 0)].powf(2.0))
        + basis[4][(11, 0)].powf(2.0))
        + basis[4][(3, 0)].powf(2.0))
        - 2.0 * basis[4][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[4][(11, 0)];
}

#[inline]
fn init_stage_5(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    init_stage_5_section_0(groebner_matrix, basis);
    init_stage_5_section_1(groebner_matrix, basis);
    init_stage_5_section_2(groebner_matrix, basis);
    init_stage_5_section_3(groebner_matrix, basis, distances);
    init_stage_5_section_4(groebner_matrix, basis);
    init_stage_5_section_5(groebner_matrix, basis);
    init_stage_5_section_6(groebner_matrix, basis);
    init_stage_5_section_7(groebner_matrix, basis, distances);
}
#[inline]
fn init_stage_5_section_0(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(4, 62)] = ((((((((((2.0 * basis[3][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[3][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[3][(11, 0)])
        + 2.0 * basis[3][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(4, 63)] = ((((((((((2.0 * basis[3][(4, 0)] * basis[4][(4, 0)]
        + 2.0 * basis[3][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[3][(11, 0)])
        + 2.0 * basis[3][(3, 0)] * basis[4][(3, 0)];
    groebner_matrix[(4, 64)] = (((((((basis[3][(4, 0)].powf(2.0)
        + basis[3][(5, 0)].powf(2.0))
        + basis[3][(9, 0)].powf(2.0))
        + basis[3][(10, 0)].powf(2.0))
        + basis[3][(11, 0)].powf(2.0))
        + basis[3][(3, 0)].powf(2.0))
        - 2.0 * basis[3][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[3][(11, 0)];
    groebner_matrix[(4, 65)] = ((((((((((2.0 * basis[2][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[2][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(4, 66)] = ((((((((((2.0 * basis[2][(4, 0)] * basis[4][(4, 0)]
        + 2.0 * basis[2][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[4][(3, 0)];
}

#[inline]
fn init_stage_5_section_1(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(4, 67)] = ((((((((((2.0 * basis[2][(4, 0)] * basis[3][(4, 0)]
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[3][(3, 0)];
    groebner_matrix[(4, 68)] = (((((((basis[2][(4, 0)].powf(2.0)
        + basis[2][(5, 0)].powf(2.0))
        + basis[2][(9, 0)].powf(2.0))
        + basis[2][(10, 0)].powf(2.0))
        + basis[2][(11, 0)].powf(2.0))
        + basis[2][(3, 0)].powf(2.0))
        - 2.0 * basis[2][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[2][(11, 0)];
    groebner_matrix[(4, 69)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[5][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[5][(3, 0)];
    groebner_matrix[(4, 70)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[4][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[4][(5, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(5, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[4][(3, 0)];
    groebner_matrix[(4, 71)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[3][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[3][(3, 0)];
}

#[inline]
fn init_stage_5_section_2(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(4, 72)] = ((((((((((2.0 * basis[1][(4, 0)] * basis[2][(4, 0)]
        + 2.0 * basis[1][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[1][(11, 0)])
        + 2.0 * basis[1][(3, 0)] * basis[2][(3, 0)];
    groebner_matrix[(4, 73)] = (((((((basis[1][(4, 0)].powf(2.0)
        + basis[1][(5, 0)].powf(2.0))
        + basis[1][(9, 0)].powf(2.0))
        + basis[1][(10, 0)].powf(2.0))
        + basis[1][(11, 0)].powf(2.0))
        + basis[1][(3, 0)].powf(2.0))
        - 2.0 * basis[1][(3, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[1][(11, 0)];
    groebner_matrix[(4, 74)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[5][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[5][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[5][(5, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(3, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(4, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(5, 0)] * basis[0][(11, 0)];
    groebner_matrix[(4, 75)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[4][(3, 0)]
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
    groebner_matrix[(4, 76)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[3][(3, 0)]
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
}

#[inline]
fn init_stage_5_section_3(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    groebner_matrix[(4, 77)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[2][(3, 0)]
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
    groebner_matrix[(4, 78)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[1][(3, 0)]
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
    groebner_matrix[(4, 79)] = ((((((((-distances[4] + basis[0][(4, 0)].powf(2.0))
        + basis[0][(5, 0)].powf(2.0))
        + basis[0][(9, 0)].powf(2.0))
        + basis[0][(10, 0)].powf(2.0))
        + basis[0][(11, 0)].powf(2.0))
        + basis[0][(3, 0)].powf(2.0))
        - 2.0 * basis[0][(3, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[0][(11, 0)];
    groebner_matrix[(5, 59)] = (((((((basis[5][(6, 0)].powf(2.0)
        + basis[5][(7, 0)].powf(2.0))
        + basis[5][(8, 0)].powf(2.0))
        + basis[5][(9, 0)].powf(2.0))
        + basis[5][(10, 0)].powf(2.0))
        + basis[5][(11, 0)].powf(2.0))
        - 2.0 * basis[5][(8, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(6, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(7, 0)] * basis[5][(10, 0)];
    groebner_matrix[(5, 60)] = ((((((((((2.0 * basis[4][(6, 0)] * basis[5][(6, 0)]
        + 2.0 * basis[4][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[4][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[4][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[4][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[4][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[4][(6, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(6, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(7, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(7, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(8, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(8, 0)] * basis[4][(11, 0)];
}

#[inline]
fn init_stage_5_section_4(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(5, 61)] = (((((((basis[4][(6, 0)].powf(2.0)
        + basis[4][(7, 0)].powf(2.0))
        + basis[4][(8, 0)].powf(2.0))
        + basis[4][(9, 0)].powf(2.0))
        + basis[4][(10, 0)].powf(2.0))
        + basis[4][(11, 0)].powf(2.0))
        - 2.0 * basis[4][(8, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(6, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(7, 0)] * basis[4][(10, 0)];
    groebner_matrix[(5, 62)] = ((((((((((2.0 * basis[3][(6, 0)] * basis[5][(6, 0)]
        + 2.0 * basis[3][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[3][(7, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[3][(6, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(6, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[5][(7, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(8, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(8, 0)] * basis[3][(11, 0)];
    groebner_matrix[(5, 63)] = ((((((((((2.0 * basis[3][(6, 0)] * basis[4][(6, 0)]
        + 2.0 * basis[3][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[3][(8, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[3][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[3][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[3][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[3][(7, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(7, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(6, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(6, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(8, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(8, 0)] * basis[3][(11, 0)];
    groebner_matrix[(5, 64)] = (((((((basis[3][(6, 0)].powf(2.0)
        + basis[3][(7, 0)].powf(2.0))
        + basis[3][(8, 0)].powf(2.0))
        + basis[3][(9, 0)].powf(2.0))
        + basis[3][(10, 0)].powf(2.0))
        + basis[3][(11, 0)].powf(2.0))
        - 2.0 * basis[3][(8, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(6, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(7, 0)] * basis[3][(10, 0)];
    groebner_matrix[(5, 65)] = ((((((((((2.0 * basis[2][(6, 0)] * basis[5][(6, 0)]
        + 2.0 * basis[2][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[2][(6, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(6, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(7, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(7, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(8, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(8, 0)] * basis[2][(11, 0)];
}

#[inline]
fn init_stage_5_section_5(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(5, 66)] = ((((((((((2.0 * basis[2][(6, 0)] * basis[4][(6, 0)]
        + 2.0 * basis[2][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(7, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(6, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(6, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(7, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[2][(8, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(8, 0)] * basis[2][(11, 0)];
    groebner_matrix[(5, 67)] = ((((((((((2.0 * basis[2][(6, 0)] * basis[3][(6, 0)]
        + 2.0 * basis[2][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(7, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(6, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(6, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(7, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[2][(8, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(8, 0)] * basis[2][(11, 0)];
    groebner_matrix[(5, 68)] = (((((((basis[2][(6, 0)].powf(2.0)
        + basis[2][(7, 0)].powf(2.0))
        + basis[2][(8, 0)].powf(2.0))
        + basis[2][(9, 0)].powf(2.0))
        + basis[2][(10, 0)].powf(2.0))
        + basis[2][(11, 0)].powf(2.0))
        - 2.0 * basis[2][(8, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(6, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(7, 0)] * basis[2][(10, 0)];
    groebner_matrix[(5, 69)] = ((((((((((2.0 * basis[1][(6, 0)] * basis[5][(6, 0)]
        + 2.0 * basis[1][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[1][(6, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(6, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(7, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(7, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(8, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(8, 0)] * basis[1][(11, 0)];
    groebner_matrix[(5, 70)] = ((((((((((2.0 * basis[1][(6, 0)] * basis[4][(6, 0)]
        + 2.0 * basis[1][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(7, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(6, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(6, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(7, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[1][(8, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(8, 0)] * basis[1][(11, 0)];
}

#[inline]
fn init_stage_5_section_6(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
) {
    groebner_matrix[(5, 71)] = ((((((((((2.0 * basis[1][(6, 0)] * basis[3][(6, 0)]
        + 2.0 * basis[1][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(7, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(6, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(6, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(7, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[1][(8, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(8, 0)] * basis[1][(11, 0)];
    groebner_matrix[(5, 72)] = ((((((((((2.0 * basis[1][(6, 0)] * basis[2][(6, 0)]
        + 2.0 * basis[1][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[1][(9, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[1][(10, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[1][(11, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[1][(6, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(6, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(7, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(7, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(8, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(8, 0)] * basis[1][(11, 0)];
    groebner_matrix[(5, 73)] = (((((((basis[1][(6, 0)].powf(2.0)
        + basis[1][(7, 0)].powf(2.0))
        + basis[1][(8, 0)].powf(2.0))
        + basis[1][(9, 0)].powf(2.0))
        + basis[1][(10, 0)].powf(2.0))
        + basis[1][(11, 0)].powf(2.0))
        - 2.0 * basis[1][(8, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[1][(6, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(7, 0)] * basis[1][(10, 0)];
    groebner_matrix[(5, 74)] = ((((((((((2.0 * basis[0][(6, 0)] * basis[5][(6, 0)]
        + 2.0 * basis[0][(7, 0)] * basis[5][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[5][(8, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[5][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[5][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[0][(8, 0)] * basis[5][(11, 0)])
        - 2.0 * basis[5][(8, 0)] * basis[0][(11, 0)])
        - 2.0 * basis[0][(6, 0)] * basis[5][(9, 0)])
        - 2.0 * basis[5][(6, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(7, 0)] * basis[5][(10, 0)])
        - 2.0 * basis[5][(7, 0)] * basis[0][(10, 0)];
    groebner_matrix[(5, 75)] = ((((((((((2.0 * basis[0][(6, 0)] * basis[4][(6, 0)]
        + 2.0 * basis[0][(7, 0)] * basis[4][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[4][(8, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[4][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[4][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[0][(8, 0)] * basis[4][(11, 0)])
        - 2.0 * basis[4][(8, 0)] * basis[0][(11, 0)])
        - 2.0 * basis[0][(6, 0)] * basis[4][(9, 0)])
        - 2.0 * basis[4][(6, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(7, 0)] * basis[4][(10, 0)])
        - 2.0 * basis[4][(7, 0)] * basis[0][(10, 0)];
}

#[inline]
fn init_stage_5_section_7(
    groebner_matrix: &mut SMatrix<f64, 44, 80>,
    basis: [&SMatrix<f64, 12, 1>; 6],
    distances: &[f64; 6],
) {
    groebner_matrix[(5, 76)] = ((((((((((2.0 * basis[0][(6, 0)] * basis[3][(6, 0)]
        + 2.0 * basis[0][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[3][(8, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[3][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[3][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[0][(8, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(8, 0)] * basis[0][(11, 0)])
        - 2.0 * basis[0][(6, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(6, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(7, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(7, 0)] * basis[0][(10, 0)];
    groebner_matrix[(5, 77)] = ((((((((((2.0 * basis[0][(6, 0)] * basis[2][(6, 0)]
        + 2.0 * basis[0][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[0][(8, 0)] * basis[2][(11, 0)])
        - 2.0 * basis[2][(8, 0)] * basis[0][(11, 0)])
        - 2.0 * basis[0][(6, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(6, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(7, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(7, 0)] * basis[0][(10, 0)];
    groebner_matrix[(5, 78)] = ((((((((((2.0 * basis[0][(6, 0)] * basis[1][(6, 0)]
        + 2.0 * basis[0][(7, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[1][(8, 0)])
        + 2.0 * basis[0][(9, 0)] * basis[1][(9, 0)])
        + 2.0 * basis[0][(10, 0)] * basis[1][(10, 0)])
        + 2.0 * basis[0][(11, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[0][(8, 0)] * basis[1][(11, 0)])
        - 2.0 * basis[1][(8, 0)] * basis[0][(11, 0)])
        - 2.0 * basis[0][(6, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(6, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(7, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(7, 0)] * basis[0][(10, 0)];
    groebner_matrix[(5, 79)] = ((((((((-distances[5] + basis[0][(6, 0)].powf(2.0))
        + basis[0][(7, 0)].powf(2.0))
        + basis[0][(8, 0)].powf(2.0))
        + basis[0][(9, 0)].powf(2.0))
        + basis[0][(10, 0)].powf(2.0))
        + basis[0][(11, 0)].powf(2.0))
        - 2.0 * basis[0][(6, 0)] * basis[0][(9, 0)])
        - 2.0 * basis[0][(7, 0)] * basis[0][(10, 0)])
        - 2.0 * basis[0][(8, 0)] * basis[0][(11, 0)];
}
