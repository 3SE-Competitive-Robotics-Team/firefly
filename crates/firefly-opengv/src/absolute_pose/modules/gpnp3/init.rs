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
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
    control_points: [&Vector3<f64>; 4],
) {
    let mut distances: [f64; 4] = [0.0; 4];
    let mut temp = control_points[0] - control_points[1];
    distances[0] = temp.norm() * temp.norm();
    temp = control_points[0] - control_points[2];
    distances[1] = temp.norm() * temp.norm();
    temp = control_points[0] - control_points[3];
    distances[2] = temp.norm() * temp.norm();
    temp = control_points[1] - control_points[2];
    distances[3] = temp.norm() * temp.norm();
    init_stage_0(groebner_matrix, basis);
    init_stage_1(groebner_matrix, basis, &distances);
    init_stage_2(groebner_matrix, basis);
    init_stage_3(groebner_matrix, basis, &distances);
    init_stage_4(groebner_matrix, basis, &distances);
}
#[inline]
fn init_stage_0(groebner_matrix: &mut SMatrix<f64, 15, 18>, basis: [&SMatrix<f64, 12, 1>; 4]) {
    groebner_matrix[(0, 8)] = (((((((-2.0 * basis[3][(0, 0)] * basis[3][(3, 0)]
        - 2.0 * basis[3][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(5, 0)])
        + basis[3][(0, 0)].powf(2.0))
        + basis[3][(3, 0)].powf(2.0))
        + basis[3][(1, 0)].powf(2.0))
        + basis[3][(4, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(5, 0)].powf(2.0);
    groebner_matrix[(0, 9)] = ((((((((((2.0 * basis[2][(0, 0)] * basis[3][(0, 0)]
        - 2.0 * basis[2][(0, 0)] * basis[3][(3, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(3, 0)])
        + 2.0 * basis[2][(3, 0)] * basis[3][(3, 0)])
        + 2.0 * basis[2][(1, 0)] * basis[3][(1, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(4, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[2][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(5, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)];
    groebner_matrix[(0, 10)] = (((((((-2.0 * basis[2][(0, 0)] * basis[2][(3, 0)]
        - 2.0 * basis[2][(1, 0)] * basis[2][(4, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(5, 0)])
        + basis[2][(0, 0)].powf(2.0))
        + basis[2][(3, 0)].powf(2.0))
        + basis[2][(1, 0)].powf(2.0))
        + basis[2][(4, 0)].powf(2.0))
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(5, 0)].powf(2.0);
    groebner_matrix[(0, 11)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[3][(0, 0)]
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
    groebner_matrix[(0, 12)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[2][(0, 0)]
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
fn init_stage_1(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
    distances: &[f64; 4],
) {
    groebner_matrix[(0, 13)] = (((((((-2.0 * basis[1][(0, 0)] * basis[1][(3, 0)]
        - 2.0 * basis[1][(1, 0)] * basis[1][(4, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(5, 0)])
        + basis[1][(0, 0)].powf(2.0))
        + basis[1][(3, 0)].powf(2.0))
        + basis[1][(1, 0)].powf(2.0))
        + basis[1][(4, 0)].powf(2.0))
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(5, 0)].powf(2.0);
    groebner_matrix[(0, 14)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[3][(0, 0)]
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
    groebner_matrix[(0, 15)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
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
    groebner_matrix[(0, 16)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[1][(0, 0)]
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
    groebner_matrix[(0, 17)] = ((((((((-distances[0] + basis[0][(0, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(3, 0)])
        + basis[0][(3, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(4, 0)])
        + basis[0][(4, 0)].powf(2.0))
        + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(5, 0)])
        + basis[0][(5, 0)].powf(2.0);
}

#[inline]
fn init_stage_2(groebner_matrix: &mut SMatrix<f64, 15, 18>, basis: [&SMatrix<f64, 12, 1>; 4]) {
    groebner_matrix[(1, 8)] = (((((((-2.0 * basis[3][(0, 0)] * basis[3][(6, 0)]
        - 2.0 * basis[3][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(8, 0)])
        + basis[3][(0, 0)].powf(2.0))
        + basis[3][(1, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(6, 0)].powf(2.0))
        + basis[3][(7, 0)].powf(2.0))
        + basis[3][(8, 0)].powf(2.0);
    groebner_matrix[(1, 9)] = ((((((((((2.0 * basis[2][(0, 0)] * basis[3][(0, 0)]
        + 2.0 * basis[2][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(8, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[3][(8, 0)];
    groebner_matrix[(1, 10)] = (((((((-2.0 * basis[2][(0, 0)] * basis[2][(6, 0)]
        - 2.0 * basis[2][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(8, 0)])
        + basis[2][(0, 0)].powf(2.0))
        + basis[2][(1, 0)].powf(2.0))
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(6, 0)].powf(2.0))
        + basis[2][(7, 0)].powf(2.0))
        + basis[2][(8, 0)].powf(2.0);
    groebner_matrix[(1, 11)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[3][(0, 0)]
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
    groebner_matrix[(1, 12)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[2][(0, 0)]
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
fn init_stage_3(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
    distances: &[f64; 4],
) {
    init_stage_3_section_0(groebner_matrix, basis, distances);
    init_stage_3_section_1(groebner_matrix, basis);
    init_stage_3_section_2(groebner_matrix, basis);
}
#[inline]
fn init_stage_3_section_0(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
    distances: &[f64; 4],
) {
    groebner_matrix[(1, 13)] = (((((((-2.0 * basis[1][(0, 0)] * basis[1][(6, 0)]
        - 2.0 * basis[1][(1, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(8, 0)])
        + basis[1][(0, 0)].powf(2.0))
        + basis[1][(1, 0)].powf(2.0))
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(6, 0)].powf(2.0))
        + basis[1][(7, 0)].powf(2.0))
        + basis[1][(8, 0)].powf(2.0);
    groebner_matrix[(1, 14)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[3][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[3][(8, 0)];
    groebner_matrix[(1, 15)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[2][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[2][(2, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[2][(8, 0)];
    groebner_matrix[(1, 16)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[1][(0, 0)]
        + 2.0 * basis[0][(1, 0)] * basis[1][(1, 0)])
        + 2.0 * basis[0][(2, 0)] * basis[1][(2, 0)])
        - 2.0 * basis[0][(0, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(0, 0)] * basis[0][(6, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[0][(1, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[0][(7, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[0][(2, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[0][(8, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[1][(8, 0)];
    groebner_matrix[(1, 17)] = ((((((((-distances[1] + basis[0][(0, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0))
        + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(6, 0)])
        + basis[0][(6, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(7, 0)])
        + basis[0][(7, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(8, 0)])
        + basis[0][(8, 0)].powf(2.0);
    groebner_matrix[(2, 8)] = (((((((basis[3][(0, 0)].powf(2.0) + basis[3][(1, 0)].powf(2.0))
        + basis[3][(2, 0)].powf(2.0))
        + basis[3][(9, 0)].powf(2.0))
        + basis[3][(10, 0)].powf(2.0))
        + basis[3][(11, 0)].powf(2.0))
        - 2.0 * basis[3][(0, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[3][(11, 0)];
}

#[inline]
fn init_stage_3_section_1(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
) {
    groebner_matrix[(2, 9)] = ((((((((((2.0 * basis[2][(0, 0)] * basis[3][(0, 0)]
        + 2.0 * basis[2][(1, 0)] * basis[3][(1, 0)])
        + 2.0 * basis[2][(2, 0)] * basis[3][(2, 0)])
        - 2.0 * basis[2][(0, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[3][(0, 0)] * basis[2][(9, 0)])
        + 2.0 * basis[2][(9, 0)] * basis[3][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[3][(1, 0)] * basis[2][(10, 0)])
        + 2.0 * basis[2][(10, 0)] * basis[3][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[3][(11, 0)])
        - 2.0 * basis[3][(2, 0)] * basis[2][(11, 0)])
        + 2.0 * basis[2][(11, 0)] * basis[3][(11, 0)];
    groebner_matrix[(2, 10)] = (((((((basis[2][(0, 0)].powf(2.0)
        + basis[2][(1, 0)].powf(2.0))
        + basis[2][(2, 0)].powf(2.0))
        + basis[2][(9, 0)].powf(2.0))
        + basis[2][(10, 0)].powf(2.0))
        + basis[2][(11, 0)].powf(2.0))
        - 2.0 * basis[2][(0, 0)] * basis[2][(9, 0)])
        - 2.0 * basis[2][(1, 0)] * basis[2][(10, 0)])
        - 2.0 * basis[2][(2, 0)] * basis[2][(11, 0)];
    groebner_matrix[(2, 11)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[3][(0, 0)]
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
    groebner_matrix[(2, 12)] = ((((((((((2.0 * basis[1][(0, 0)] * basis[2][(0, 0)]
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
    groebner_matrix[(2, 13)] = (((((((basis[1][(0, 0)].powf(2.0)
        + basis[1][(1, 0)].powf(2.0))
        + basis[1][(2, 0)].powf(2.0))
        + basis[1][(9, 0)].powf(2.0))
        + basis[1][(10, 0)].powf(2.0))
        + basis[1][(11, 0)].powf(2.0))
        - 2.0 * basis[1][(0, 0)] * basis[1][(9, 0)])
        - 2.0 * basis[1][(1, 0)] * basis[1][(10, 0)])
        - 2.0 * basis[1][(2, 0)] * basis[1][(11, 0)];
}

#[inline]
fn init_stage_3_section_2(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
) {
    groebner_matrix[(2, 14)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[3][(0, 0)]
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
    groebner_matrix[(2, 15)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[2][(0, 0)]
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
    groebner_matrix[(2, 16)] = ((((((((((2.0 * basis[0][(0, 0)] * basis[1][(0, 0)]
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
fn init_stage_4(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
    distances: &[f64; 4],
) {
    init_stage_4_section_0(groebner_matrix, basis, distances);
    init_stage_4_section_1(groebner_matrix, basis, distances);
}
#[inline]
fn init_stage_4_section_0(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
    distances: &[f64; 4],
) {
    groebner_matrix[(2, 17)] = ((((((((-distances[2] + basis[0][(0, 0)].powf(2.0))
        + basis[0][(1, 0)].powf(2.0))
        + basis[0][(2, 0)].powf(2.0))
        - 2.0 * basis[0][(0, 0)] * basis[0][(9, 0)])
        + basis[0][(9, 0)].powf(2.0))
        - 2.0 * basis[0][(1, 0)] * basis[0][(10, 0)])
        + basis[0][(10, 0)].powf(2.0))
        - 2.0 * basis[0][(2, 0)] * basis[0][(11, 0)])
        + basis[0][(11, 0)].powf(2.0);
    groebner_matrix[(3, 8)] = (((((((basis[3][(3, 0)].powf(2.0) + basis[3][(4, 0)].powf(2.0))
        + basis[3][(5, 0)].powf(2.0))
        + basis[3][(6, 0)].powf(2.0))
        + basis[3][(7, 0)].powf(2.0))
        + basis[3][(8, 0)].powf(2.0))
        - 2.0 * basis[3][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[3][(8, 0)];
    groebner_matrix[(3, 9)] = ((((((((((2.0 * basis[2][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[2][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[2][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[2][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[2][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[2][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[2][(8, 0)];
    groebner_matrix[(3, 10)] = (((((((basis[2][(3, 0)].powf(2.0)
        + basis[2][(4, 0)].powf(2.0))
        + basis[2][(5, 0)].powf(2.0))
        + basis[2][(6, 0)].powf(2.0))
        + basis[2][(7, 0)].powf(2.0))
        + basis[2][(8, 0)].powf(2.0))
        - 2.0 * basis[2][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[2][(8, 0)];
    groebner_matrix[(3, 11)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[1][(8, 0)];
    groebner_matrix[(3, 12)] = ((((((((((2.0 * basis[1][(3, 0)] * basis[2][(3, 0)]
        + 2.0 * basis[1][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[1][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[1][(6, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[1][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[1][(8, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[1][(8, 0)];
}

#[inline]
fn init_stage_4_section_1(
    groebner_matrix: &mut SMatrix<f64, 15, 18>,
    basis: [&SMatrix<f64, 12, 1>; 4],
    distances: &[f64; 4],
) {
    groebner_matrix[(3, 13)] = (((((((basis[1][(3, 0)].powf(2.0)
        + basis[1][(4, 0)].powf(2.0))
        + basis[1][(5, 0)].powf(2.0))
        + basis[1][(6, 0)].powf(2.0))
        + basis[1][(7, 0)].powf(2.0))
        + basis[1][(8, 0)].powf(2.0))
        - 2.0 * basis[1][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[1][(8, 0)];
    groebner_matrix[(3, 14)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[3][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[3][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[3][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[3][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[3][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[3][(6, 0)])
        - 2.0 * basis[3][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[3][(7, 0)])
        - 2.0 * basis[3][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[3][(8, 0)])
        - 2.0 * basis[3][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(3, 15)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[2][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[2][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[2][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[2][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[2][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[2][(6, 0)])
        - 2.0 * basis[2][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[2][(7, 0)])
        - 2.0 * basis[2][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[2][(8, 0)])
        - 2.0 * basis[2][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(3, 16)] = ((((((((((2.0 * basis[0][(3, 0)] * basis[1][(3, 0)]
        + 2.0 * basis[0][(4, 0)] * basis[1][(4, 0)])
        + 2.0 * basis[0][(5, 0)] * basis[1][(5, 0)])
        + 2.0 * basis[0][(6, 0)] * basis[1][(6, 0)])
        + 2.0 * basis[0][(7, 0)] * basis[1][(7, 0)])
        + 2.0 * basis[0][(8, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[0][(3, 0)] * basis[1][(6, 0)])
        - 2.0 * basis[1][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[1][(7, 0)])
        - 2.0 * basis[1][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[1][(8, 0)])
        - 2.0 * basis[1][(5, 0)] * basis[0][(8, 0)];
    groebner_matrix[(3, 17)] = ((((((((-distances[3] + basis[0][(3, 0)].powf(2.0))
        + basis[0][(4, 0)].powf(2.0))
        + basis[0][(5, 0)].powf(2.0))
        + basis[0][(6, 0)].powf(2.0))
        + basis[0][(7, 0)].powf(2.0))
        + basis[0][(8, 0)].powf(2.0))
        - 2.0 * basis[0][(3, 0)] * basis[0][(6, 0)])
        - 2.0 * basis[0][(4, 0)] * basis[0][(7, 0)])
        - 2.0 * basis[0][(5, 0)] * basis[0][(8, 0)];
}
