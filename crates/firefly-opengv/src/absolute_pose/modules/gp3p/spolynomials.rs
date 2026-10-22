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

use nalgebra::SMatrix;

pub fn s_polynomial9(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(9, 21)] = groebner_matrix[(0, 21)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 21)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 33)] = groebner_matrix[(0, 33)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 33)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 53)] = groebner_matrix[(0, 53)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 53)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 66)] = groebner_matrix[(0, 66)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 66)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 70)] = -groebner_matrix[(1, 70)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 71)] = groebner_matrix[(0, 71)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 71)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 75)] = groebner_matrix[(0, 75)] / (groebner_matrix[(0, 20)]);
    groebner_matrix[(9, 76)] = groebner_matrix[(0, 76)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 76)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 77)] = groebner_matrix[(0, 77)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 77)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 80)] = groebner_matrix[(0, 80)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 80)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 81)] = groebner_matrix[(0, 81)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 81)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 82)] = groebner_matrix[(0, 82)] / (groebner_matrix[(0, 20)]);
    groebner_matrix[(9, 83)] = -groebner_matrix[(1, 83)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(9, 84)] = groebner_matrix[(0, 84)] / (groebner_matrix[(0, 20)])
        - groebner_matrix[(1, 84)] / (groebner_matrix[(1, 20)]);
}

pub fn s_polynomial10(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(10, 21)] = groebner_matrix[(1, 21)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 21)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 33)] = groebner_matrix[(1, 33)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 33)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 53)] = groebner_matrix[(1, 53)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 53)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 66)] = groebner_matrix[(1, 66)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 66)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 70)] = groebner_matrix[(1, 70)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 70)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 71)] = groebner_matrix[(1, 71)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 71)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 75)] = -groebner_matrix[(2, 75)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 76)] = groebner_matrix[(1, 76)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(10, 77)] = groebner_matrix[(1, 77)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 77)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 80)] = groebner_matrix[(1, 80)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 80)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 81)] = groebner_matrix[(1, 81)] / (groebner_matrix[(1, 20)]);
    groebner_matrix[(10, 82)] = -groebner_matrix[(2, 82)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 83)] = groebner_matrix[(1, 83)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 83)] / (groebner_matrix[(2, 20)]);
    groebner_matrix[(10, 84)] = groebner_matrix[(1, 84)] / (groebner_matrix[(1, 20)])
        - groebner_matrix[(2, 84)] / (groebner_matrix[(2, 20)]);
}

pub fn s_polynomial11(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(11, 20)] = groebner_matrix[(3, 20)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 20)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 32)] = groebner_matrix[(3, 32)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 32)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 52)] = groebner_matrix[(3, 52)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 52)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 66)] = groebner_matrix[(3, 66)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 66)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 70)] = -groebner_matrix[(4, 70)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 71)] = groebner_matrix[(3, 71)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 71)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 75)] = groebner_matrix[(3, 75)] / (groebner_matrix[(3, 19)]);
    groebner_matrix[(11, 76)] = groebner_matrix[(3, 76)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 76)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 77)] = groebner_matrix[(3, 77)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 77)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 79)] = groebner_matrix[(3, 79)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 79)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 81)] = groebner_matrix[(3, 81)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 81)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 82)] = groebner_matrix[(3, 82)] / (groebner_matrix[(3, 19)]);
    groebner_matrix[(11, 83)] = -groebner_matrix[(4, 83)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(11, 84)] = groebner_matrix[(3, 84)] / (groebner_matrix[(3, 19)])
        - groebner_matrix[(4, 84)] / (groebner_matrix[(4, 19)]);
}

pub fn s_polynomial12(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(12, 20)] = groebner_matrix[(4, 20)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 20)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 32)] = groebner_matrix[(4, 32)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 32)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 52)] = groebner_matrix[(4, 52)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 52)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 66)] = groebner_matrix[(4, 66)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 66)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 70)] = groebner_matrix[(4, 70)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 70)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 71)] = groebner_matrix[(4, 71)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 71)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 75)] = -groebner_matrix[(5, 75)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 76)] = groebner_matrix[(4, 76)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(12, 77)] = groebner_matrix[(4, 77)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 77)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 79)] = groebner_matrix[(4, 79)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 79)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 81)] = groebner_matrix[(4, 81)] / (groebner_matrix[(4, 19)]);
    groebner_matrix[(12, 82)] = -groebner_matrix[(5, 82)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 83)] = groebner_matrix[(4, 83)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 83)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(12, 84)] = groebner_matrix[(4, 84)] / (groebner_matrix[(4, 19)])
        - groebner_matrix[(5, 84)] / (groebner_matrix[(5, 19)]);
}

pub fn s_polynomial13(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(13, 20)] = groebner_matrix[(5, 20)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(13, 21)] = -groebner_matrix[(6, 21)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 32)] = groebner_matrix[(5, 32)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(13, 33)] = -groebner_matrix[(6, 33)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 52)] = groebner_matrix[(5, 52)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(13, 53)] = -groebner_matrix[(6, 53)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 66)] = groebner_matrix[(5, 66)] / (groebner_matrix[(5, 19)])
        - groebner_matrix[(6, 66)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 70)] = groebner_matrix[(5, 70)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(13, 71)] = groebner_matrix[(5, 71)] / (groebner_matrix[(5, 19)])
        - groebner_matrix[(6, 71)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 75)] = groebner_matrix[(5, 75)] / (groebner_matrix[(5, 19)])
        - groebner_matrix[(6, 75)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 76)] = -groebner_matrix[(6, 76)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 77)] = groebner_matrix[(5, 77)] / (groebner_matrix[(5, 19)])
        - groebner_matrix[(6, 77)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 79)] = groebner_matrix[(5, 79)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(13, 80)] = -groebner_matrix[(6, 80)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 81)] = -groebner_matrix[(6, 81)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 82)] = groebner_matrix[(5, 82)] / (groebner_matrix[(5, 19)])
        - groebner_matrix[(6, 82)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(13, 83)] = groebner_matrix[(5, 83)] / (groebner_matrix[(5, 19)]);
    groebner_matrix[(13, 84)] = groebner_matrix[(5, 84)] / (groebner_matrix[(5, 19)])
        - groebner_matrix[(6, 84)] / (groebner_matrix[(6, 19)]);
}

pub fn s_polynomial14(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(14, 21)] = groebner_matrix[(6, 21)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 21)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 33)] = groebner_matrix[(6, 33)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 33)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 53)] = groebner_matrix[(6, 53)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 53)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 66)] = groebner_matrix[(6, 66)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 66)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 70)] = -groebner_matrix[(7, 70)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 71)] = groebner_matrix[(6, 71)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 71)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 75)] = groebner_matrix[(6, 75)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(14, 76)] = groebner_matrix[(6, 76)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 76)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 77)] = groebner_matrix[(6, 77)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 77)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 80)] = groebner_matrix[(6, 80)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 80)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 81)] = groebner_matrix[(6, 81)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 81)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 82)] = groebner_matrix[(6, 82)] / (groebner_matrix[(6, 19)]);
    groebner_matrix[(14, 83)] = -groebner_matrix[(7, 83)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(14, 84)] = groebner_matrix[(6, 84)] / (groebner_matrix[(6, 19)])
        - groebner_matrix[(7, 84)] / (groebner_matrix[(7, 19)]);
}

pub fn s_polynomial15(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(15, 21)] = groebner_matrix[(7, 21)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 21)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 33)] = groebner_matrix[(7, 33)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 33)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 53)] = groebner_matrix[(7, 53)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 53)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 66)] = groebner_matrix[(7, 66)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 66)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 70)] = groebner_matrix[(7, 70)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 70)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 71)] = groebner_matrix[(7, 71)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 71)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 75)] = -groebner_matrix[(8, 75)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 76)] = groebner_matrix[(7, 76)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(15, 77)] = groebner_matrix[(7, 77)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 77)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 80)] = groebner_matrix[(7, 80)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 80)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 81)] = groebner_matrix[(7, 81)] / (groebner_matrix[(7, 19)]);
    groebner_matrix[(15, 82)] = -groebner_matrix[(8, 82)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 83)] = groebner_matrix[(7, 83)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 83)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(15, 84)] = groebner_matrix[(7, 84)] / (groebner_matrix[(7, 19)])
        - groebner_matrix[(8, 84)] / (groebner_matrix[(8, 19)]);
}

pub fn s_polynomial16(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(16, 35)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(16, 45)] = -groebner_matrix[(15, 75)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(16, 49)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(15, 76)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(16, 50)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(16, 54)] = -groebner_matrix[(15, 77)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(16, 55)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(16, 66)] = -groebner_matrix[(15, 81)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(16, 70)] = groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(15, 82)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(16, 71)] = groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(16, 75)] = -groebner_matrix[(15, 83)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(16, 76)] = groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(16, 81)] = -groebner_matrix[(15, 84)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(16, 82)] = groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
}

pub fn s_polynomial17(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(17, 44)] = groebner_matrix[(14, 44)] / (groebner_matrix[(14, 33)])
        - groebner_matrix[(15, 75)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(17, 48)] = groebner_matrix[(14, 48)] / (groebner_matrix[(14, 33)])
        - groebner_matrix[(15, 76)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(17, 53)] = groebner_matrix[(14, 53)] / (groebner_matrix[(14, 33)])
        - groebner_matrix[(15, 77)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(17, 65)] = groebner_matrix[(14, 65)] / (groebner_matrix[(14, 33)])
        - groebner_matrix[(15, 81)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(17, 69)] = groebner_matrix[(14, 69)] / (groebner_matrix[(14, 33)])
        - groebner_matrix[(15, 82)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(17, 71)] = groebner_matrix[(14, 71)] / (groebner_matrix[(14, 33)]);
    groebner_matrix[(17, 74)] = groebner_matrix[(14, 74)] / (groebner_matrix[(14, 33)])
        - groebner_matrix[(15, 83)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(17, 75)] = groebner_matrix[(14, 75)] / (groebner_matrix[(14, 33)]);
    groebner_matrix[(17, 76)] = groebner_matrix[(14, 76)] / (groebner_matrix[(14, 33)]);
    groebner_matrix[(17, 77)] = groebner_matrix[(14, 77)] / (groebner_matrix[(14, 33)]);
    groebner_matrix[(17, 80)] = groebner_matrix[(14, 80)] / (groebner_matrix[(14, 33)])
        - groebner_matrix[(15, 84)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(17, 81)] = groebner_matrix[(14, 81)] / (groebner_matrix[(14, 33)]);
    groebner_matrix[(17, 82)] = groebner_matrix[(14, 82)] / (groebner_matrix[(14, 33)]);
    groebner_matrix[(17, 83)] = groebner_matrix[(14, 83)] / (groebner_matrix[(14, 33)]);
    groebner_matrix[(17, 84)] = groebner_matrix[(14, 84)] / (groebner_matrix[(14, 33)]);
}

pub fn s_polynomial18(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(18, 33)] = groebner_matrix[(13, 33)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 43)] = groebner_matrix[(13, 43)] / (groebner_matrix[(13, 32)])
        - groebner_matrix[(15, 75)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(18, 44)] = groebner_matrix[(13, 44)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 47)] = groebner_matrix[(13, 47)] / (groebner_matrix[(13, 32)])
        - groebner_matrix[(15, 76)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(18, 48)] = groebner_matrix[(13, 48)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 52)] = groebner_matrix[(13, 52)] / (groebner_matrix[(13, 32)])
        - groebner_matrix[(15, 77)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(18, 53)] = groebner_matrix[(13, 53)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 64)] = groebner_matrix[(13, 64)] / (groebner_matrix[(13, 32)])
        - groebner_matrix[(15, 81)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(18, 65)] = groebner_matrix[(13, 65)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 68)] = groebner_matrix[(13, 68)] / (groebner_matrix[(13, 32)])
        - groebner_matrix[(15, 82)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(18, 69)] = groebner_matrix[(13, 69)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 71)] = groebner_matrix[(13, 71)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 73)] = groebner_matrix[(13, 73)] / (groebner_matrix[(13, 32)])
        - groebner_matrix[(15, 83)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(18, 74)] = groebner_matrix[(13, 74)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 75)] = groebner_matrix[(13, 75)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 76)] = groebner_matrix[(13, 76)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 77)] = groebner_matrix[(13, 77)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 79)] = groebner_matrix[(13, 79)] / (groebner_matrix[(13, 32)])
        - groebner_matrix[(15, 84)] / (groebner_matrix[(15, 71)]);
    groebner_matrix[(18, 80)] = groebner_matrix[(13, 80)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 81)] = groebner_matrix[(13, 81)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 82)] = groebner_matrix[(13, 82)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 83)] = groebner_matrix[(13, 83)] / (groebner_matrix[(13, 32)]);
    groebner_matrix[(18, 84)] = groebner_matrix[(13, 84)] / (groebner_matrix[(13, 32)]);
}

pub fn s_polynomial19(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(19, 34)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 35)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(19, 45)] = -groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 49)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 50)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(19, 54)] = -groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 55)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(19, 66)] = -groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 70)] = groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 71)] = groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(19, 75)] = -groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 76)] = groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(19, 81)] = -groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(19, 82)] = groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
}

pub fn s_polynomial20(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(20, 21)] = groebner_matrix[(8, 21)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 28)] = -groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 31)] = groebner_matrix[(8, 31)] / (groebner_matrix[(8, 19)])
        - groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 33)] = groebner_matrix[(8, 33)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 42)] = -groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 46)] = -groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 51)] = groebner_matrix[(8, 51)] / (groebner_matrix[(8, 19)])
        - groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 53)] = groebner_matrix[(8, 53)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 63)] = -groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 66)] = groebner_matrix[(8, 66)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 67)] = -groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 70)] = groebner_matrix[(8, 70)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 71)] = groebner_matrix[(8, 71)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 72)] = -groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 75)] = groebner_matrix[(8, 75)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 77)] = groebner_matrix[(8, 77)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 78)] = groebner_matrix[(8, 78)] / (groebner_matrix[(8, 19)])
        - groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(20, 80)] = groebner_matrix[(8, 80)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 82)] = groebner_matrix[(8, 82)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 83)] = groebner_matrix[(8, 83)] / (groebner_matrix[(8, 19)]);
    groebner_matrix[(20, 84)] = groebner_matrix[(8, 84)] / (groebner_matrix[(8, 19)]);
}

pub fn s_polynomial21(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(21, 11)] = groebner_matrix[(16, 55)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(17, 48)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 14)] = groebner_matrix[(16, 56)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(17, 53)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 44)] = groebner_matrix[(16, 75)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(17, 65)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 48)] = groebner_matrix[(16, 76)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(17, 69)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 53)] = groebner_matrix[(16, 77)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(17, 74)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 54)] = -groebner_matrix[(17, 75)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 55)] = -groebner_matrix[(17, 76)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 56)] = -groebner_matrix[(17, 77)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 65)] = groebner_matrix[(16, 81)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(21, 69)] = groebner_matrix[(16, 82)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(21, 74)] = groebner_matrix[(16, 83)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(17, 80)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 75)] = -groebner_matrix[(17, 81)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 76)] = -groebner_matrix[(17, 82)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 77)] = -groebner_matrix[(17, 83)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(21, 80)] = groebner_matrix[(16, 84)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(21, 83)] = -groebner_matrix[(17, 84)] / (groebner_matrix[(17, 44)]);
}

pub fn s_polynomial22(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(22, 10)] = groebner_matrix[(16, 55)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(18, 47)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 13)] = groebner_matrix[(16, 56)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(18, 52)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 43)] = groebner_matrix[(16, 75)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(18, 64)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 47)] = groebner_matrix[(16, 76)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(18, 68)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 52)] = groebner_matrix[(16, 77)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(18, 73)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 54)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 55)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 56)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 64)] = groebner_matrix[(16, 81)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(22, 68)] = groebner_matrix[(16, 82)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(22, 73)] = groebner_matrix[(16, 83)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(18, 79)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 75)] = -groebner_matrix[(18, 81)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 76)] = -groebner_matrix[(18, 82)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 77)] = -groebner_matrix[(18, 83)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(22, 79)] = groebner_matrix[(16, 84)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(22, 83)] = -groebner_matrix[(18, 84)] / (groebner_matrix[(18, 43)]);
}

pub fn s_polynomial23(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(23, 9)] = groebner_matrix[(16, 55)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(20, 46)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 12)] = groebner_matrix[(16, 56)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(20, 51)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 42)] = groebner_matrix[(16, 75)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(20, 63)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 46)] = groebner_matrix[(16, 76)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(20, 67)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 51)] = groebner_matrix[(16, 77)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(20, 72)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 54)] = -groebner_matrix[(20, 75)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 55)] = -groebner_matrix[(20, 76)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 56)] = -groebner_matrix[(20, 77)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 63)] = groebner_matrix[(16, 81)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(23, 67)] = groebner_matrix[(16, 82)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(23, 72)] = groebner_matrix[(16, 83)] / (groebner_matrix[(16, 54)])
        - groebner_matrix[(20, 78)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 75)] = -groebner_matrix[(20, 81)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 76)] = -groebner_matrix[(20, 82)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 77)] = -groebner_matrix[(20, 83)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(23, 78)] = groebner_matrix[(16, 84)] / (groebner_matrix[(16, 54)]);
    groebner_matrix[(23, 83)] = -groebner_matrix[(20, 84)] / (groebner_matrix[(20, 42)]);
}

pub fn s_polynomial24(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(24, 5)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(17, 48)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 8)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(24, 11)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(17, 53)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 14)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(24, 30)] = -groebner_matrix[(17, 65)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 33)] = -groebner_matrix[(17, 69)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 44)] = groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(24, 48)] = groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(17, 74)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 49)] = -groebner_matrix[(17, 75)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 50)] = -groebner_matrix[(17, 76)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 53)] = groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(24, 55)] = -groebner_matrix[(17, 77)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 69)] = -groebner_matrix[(17, 80)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 70)] = -groebner_matrix[(17, 81)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 71)] = -groebner_matrix[(17, 82)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 74)] = groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(24, 76)] = -groebner_matrix[(17, 83)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(24, 82)] = -groebner_matrix[(17, 84)] / (groebner_matrix[(17, 44)]);
}

pub fn s_polynomial25(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(25, 4)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(18, 47)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 7)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(25, 10)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(18, 52)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 13)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(25, 29)] = -groebner_matrix[(18, 64)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 32)] = -groebner_matrix[(18, 68)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 43)] = groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(25, 47)] = groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(18, 73)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 49)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 50)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 52)] = groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(25, 55)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 68)] = -groebner_matrix[(18, 79)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 70)] = -groebner_matrix[(18, 81)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 71)] = -groebner_matrix[(18, 82)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 73)] = groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(25, 76)] = -groebner_matrix[(18, 83)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(25, 82)] = -groebner_matrix[(18, 84)] / (groebner_matrix[(18, 43)]);
}

pub fn s_polynomial26(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(26, 3)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(20, 46)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 6)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(26, 9)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(20, 51)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 12)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(26, 28)] = -groebner_matrix[(20, 63)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 31)] = -groebner_matrix[(20, 67)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 42)] = groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(26, 46)] = groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(20, 72)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 49)] = -groebner_matrix[(20, 75)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 50)] = -groebner_matrix[(20, 76)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 51)] = groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(26, 55)] = -groebner_matrix[(20, 77)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 67)] = -groebner_matrix[(20, 78)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 70)] = -groebner_matrix[(20, 81)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 71)] = -groebner_matrix[(20, 82)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 72)] = groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(26, 76)] = -groebner_matrix[(20, 83)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(26, 82)] = -groebner_matrix[(20, 84)] / (groebner_matrix[(20, 42)]);
}

pub fn s_polynomial27(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(27, 2)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(17, 48)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 5)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(27, 8)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(17, 53)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 11)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(27, 14)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(27, 21)] = -groebner_matrix[(17, 65)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 30)] = -groebner_matrix[(17, 69)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 44)] = groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(17, 74)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 45)] = -groebner_matrix[(17, 75)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 48)] = groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(27, 49)] = -groebner_matrix[(17, 76)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 53)] = groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(27, 54)] = -groebner_matrix[(17, 77)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 65)] = -groebner_matrix[(17, 80)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 66)] = -groebner_matrix[(17, 81)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 70)] = -groebner_matrix[(17, 82)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 74)] = groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(27, 75)] = -groebner_matrix[(17, 83)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(27, 81)] = -groebner_matrix[(17, 84)] / (groebner_matrix[(17, 44)]);
}

pub fn s_polynomial28(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(28, 1)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(18, 47)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 4)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(28, 7)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(18, 52)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 10)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(28, 13)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(28, 20)] = -groebner_matrix[(18, 64)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 29)] = -groebner_matrix[(18, 68)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 43)] = groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(18, 73)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 45)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 47)] = groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(28, 49)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 52)] = groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(28, 54)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 64)] = -groebner_matrix[(18, 79)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 66)] = -groebner_matrix[(18, 81)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 70)] = -groebner_matrix[(18, 82)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 73)] = groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(28, 75)] = -groebner_matrix[(18, 83)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(28, 81)] = -groebner_matrix[(18, 84)] / (groebner_matrix[(18, 43)]);
}

pub fn s_polynomial29(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(29, 0)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(20, 46)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 3)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(29, 6)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(20, 51)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 9)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(29, 12)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(29, 19)] = -groebner_matrix[(20, 63)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 28)] = -groebner_matrix[(20, 67)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 42)] = groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(20, 72)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 45)] = -groebner_matrix[(20, 75)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 46)] = groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(29, 49)] = -groebner_matrix[(20, 76)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 51)] = groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(29, 54)] = -groebner_matrix[(20, 77)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 63)] = -groebner_matrix[(20, 78)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 66)] = -groebner_matrix[(20, 81)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 70)] = -groebner_matrix[(20, 82)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 72)] = groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(29, 75)] = -groebner_matrix[(20, 83)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(29, 81)] = -groebner_matrix[(20, 84)] / (groebner_matrix[(20, 42)]);
}

pub fn s_polynomial30(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(30, 43)] = groebner_matrix[(17, 75)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(30, 44)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(30, 47)] = groebner_matrix[(17, 76)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(30, 48)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(30, 52)] = groebner_matrix[(17, 77)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(30, 53)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(30, 64)] = groebner_matrix[(17, 81)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(30, 65)] = -groebner_matrix[(18, 81)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(30, 68)] = groebner_matrix[(17, 82)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(30, 69)] = -groebner_matrix[(18, 82)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(30, 73)] = groebner_matrix[(17, 83)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(30, 74)] = -groebner_matrix[(18, 83)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(30, 79)] = groebner_matrix[(17, 84)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(30, 80)] = -groebner_matrix[(18, 84)] / (groebner_matrix[(18, 43)]);
}

pub fn s_polynomial31(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(31, 42)] = groebner_matrix[(17, 75)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(31, 44)] = -groebner_matrix[(20, 75)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(31, 46)] = groebner_matrix[(17, 76)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(31, 48)] = -groebner_matrix[(20, 76)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(31, 51)] = groebner_matrix[(17, 77)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(31, 53)] = -groebner_matrix[(20, 77)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(31, 63)] = groebner_matrix[(17, 81)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(31, 65)] = -groebner_matrix[(20, 81)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(31, 67)] = groebner_matrix[(17, 82)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(31, 69)] = -groebner_matrix[(20, 82)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(31, 72)] = groebner_matrix[(17, 83)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(31, 74)] = -groebner_matrix[(20, 83)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(31, 78)] = groebner_matrix[(17, 84)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(31, 80)] = -groebner_matrix[(20, 84)] / (groebner_matrix[(20, 42)]);
}

pub fn s_polynomial32(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(32, 42)] = groebner_matrix[(18, 75)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(32, 43)] = -groebner_matrix[(20, 75)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(32, 46)] = groebner_matrix[(18, 76)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(32, 47)] = -groebner_matrix[(20, 76)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(32, 51)] = groebner_matrix[(18, 77)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(32, 52)] = -groebner_matrix[(20, 77)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(32, 63)] = groebner_matrix[(18, 81)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(32, 64)] = -groebner_matrix[(20, 81)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(32, 67)] = groebner_matrix[(18, 82)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(32, 68)] = -groebner_matrix[(20, 82)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(32, 72)] = groebner_matrix[(18, 83)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(32, 73)] = -groebner_matrix[(20, 83)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(32, 78)] = groebner_matrix[(18, 84)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(32, 79)] = -groebner_matrix[(20, 84)] / (groebner_matrix[(20, 42)]);
}

pub fn s_polynomial33(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(33, 46)] = -groebner_matrix[(32, 67)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 47)] = -groebner_matrix[(32, 68)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 48)] = groebner_matrix[(17, 48)] / (groebner_matrix[(17, 44)])
        - groebner_matrix[(32, 69)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 51)] = -groebner_matrix[(32, 72)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 52)] = -groebner_matrix[(32, 73)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 53)] = groebner_matrix[(17, 53)] / (groebner_matrix[(17, 44)])
        - groebner_matrix[(32, 74)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 54)] = -groebner_matrix[(32, 75)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 55)] = -groebner_matrix[(32, 76)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 56)] = -groebner_matrix[(32, 77)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 65)] = groebner_matrix[(17, 65)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(33, 69)] = groebner_matrix[(17, 69)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(33, 72)] = -groebner_matrix[(32, 78)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 73)] = -groebner_matrix[(32, 79)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 74)] = groebner_matrix[(17, 74)] / (groebner_matrix[(17, 44)])
        - groebner_matrix[(32, 80)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 75)] = groebner_matrix[(17, 75)] / (groebner_matrix[(17, 44)])
        - groebner_matrix[(32, 81)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 76)] = groebner_matrix[(17, 76)] / (groebner_matrix[(17, 44)])
        - groebner_matrix[(32, 82)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 77)] = groebner_matrix[(17, 77)] / (groebner_matrix[(17, 44)])
        - groebner_matrix[(32, 83)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 80)] = groebner_matrix[(17, 80)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(33, 81)] = groebner_matrix[(17, 81)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(33, 82)] = groebner_matrix[(17, 82)] / (groebner_matrix[(17, 44)]);
    groebner_matrix[(33, 83)] = groebner_matrix[(17, 83)] / (groebner_matrix[(17, 44)])
        - groebner_matrix[(32, 84)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(33, 84)] = groebner_matrix[(17, 84)] / (groebner_matrix[(17, 44)]);
}

pub fn s_polynomial34(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(34, 44)] = -groebner_matrix[(30, 65)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 47)] = groebner_matrix[(18, 47)] / (groebner_matrix[(18, 43)])
        - groebner_matrix[(30, 68)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 48)] = -groebner_matrix[(30, 69)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 52)] = groebner_matrix[(18, 52)] / (groebner_matrix[(18, 43)])
        - groebner_matrix[(30, 73)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 53)] = -groebner_matrix[(30, 74)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 54)] = -groebner_matrix[(30, 75)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 55)] = -groebner_matrix[(30, 76)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 56)] = -groebner_matrix[(30, 77)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 64)] = groebner_matrix[(18, 64)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(34, 68)] = groebner_matrix[(18, 68)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(34, 73)] = groebner_matrix[(18, 73)] / (groebner_matrix[(18, 43)])
        - groebner_matrix[(30, 79)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 74)] = -groebner_matrix[(30, 80)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 75)] = groebner_matrix[(18, 75)] / (groebner_matrix[(18, 43)])
        - groebner_matrix[(30, 81)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 76)] = groebner_matrix[(18, 76)] / (groebner_matrix[(18, 43)])
        - groebner_matrix[(30, 82)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 77)] = groebner_matrix[(18, 77)] / (groebner_matrix[(18, 43)])
        - groebner_matrix[(30, 83)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 79)] = groebner_matrix[(18, 79)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(34, 81)] = groebner_matrix[(18, 81)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(34, 82)] = groebner_matrix[(18, 82)] / (groebner_matrix[(18, 43)]);
    groebner_matrix[(34, 83)] = groebner_matrix[(18, 83)] / (groebner_matrix[(18, 43)])
        - groebner_matrix[(30, 84)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(34, 84)] = groebner_matrix[(18, 84)] / (groebner_matrix[(18, 43)]);
}

pub fn s_polynomial35(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(35, 44)] = -groebner_matrix[(31, 65)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 46)] = groebner_matrix[(20, 46)] / (groebner_matrix[(20, 42)])
        - groebner_matrix[(31, 67)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 48)] = -groebner_matrix[(31, 69)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 51)] = groebner_matrix[(20, 51)] / (groebner_matrix[(20, 42)])
        - groebner_matrix[(31, 72)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 53)] = -groebner_matrix[(31, 74)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 54)] = -groebner_matrix[(31, 75)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 55)] = -groebner_matrix[(31, 76)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 56)] = -groebner_matrix[(31, 77)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 63)] = groebner_matrix[(20, 63)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(35, 67)] = groebner_matrix[(20, 67)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(35, 72)] = groebner_matrix[(20, 72)] / (groebner_matrix[(20, 42)])
        - groebner_matrix[(31, 78)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 74)] = -groebner_matrix[(31, 80)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 75)] = groebner_matrix[(20, 75)] / (groebner_matrix[(20, 42)])
        - groebner_matrix[(31, 81)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 76)] = groebner_matrix[(20, 76)] / (groebner_matrix[(20, 42)])
        - groebner_matrix[(31, 82)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 77)] = groebner_matrix[(20, 77)] / (groebner_matrix[(20, 42)])
        - groebner_matrix[(31, 83)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 78)] = groebner_matrix[(20, 78)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(35, 81)] = groebner_matrix[(20, 81)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(35, 82)] = groebner_matrix[(20, 82)] / (groebner_matrix[(20, 42)]);
    groebner_matrix[(35, 83)] = groebner_matrix[(20, 83)] / (groebner_matrix[(20, 42)])
        - groebner_matrix[(31, 84)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(35, 84)] = groebner_matrix[(20, 84)] / (groebner_matrix[(20, 42)]);
}

pub fn s_polynomial36(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(36, 31)] = -groebner_matrix[(32, 67)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 32)] = -groebner_matrix[(32, 68)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 33)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(32, 69)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 44)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(36, 46)] = -groebner_matrix[(32, 72)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 47)] = -groebner_matrix[(32, 73)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 48)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(32, 74)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 49)] = -groebner_matrix[(32, 75)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 50)] = -groebner_matrix[(32, 76)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 53)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(36, 55)] = -groebner_matrix[(32, 77)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 65)] = groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(36, 67)] = -groebner_matrix[(32, 78)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 68)] = -groebner_matrix[(32, 79)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 69)] = groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(32, 80)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 70)] = -groebner_matrix[(32, 81)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 71)] = -groebner_matrix[(32, 82)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 74)] = groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(36, 76)] = -groebner_matrix[(32, 83)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(36, 80)] = groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(36, 82)] = -groebner_matrix[(32, 84)] / (groebner_matrix[(32, 65)]);
}

pub fn s_polynomial37(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(37, 30)] = -groebner_matrix[(30, 65)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 32)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(30, 68)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 33)] = -groebner_matrix[(30, 69)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 43)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(37, 47)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(30, 73)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 48)] = -groebner_matrix[(30, 74)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 49)] = -groebner_matrix[(30, 75)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 50)] = -groebner_matrix[(30, 76)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 52)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(37, 55)] = -groebner_matrix[(30, 77)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 64)] = groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(37, 68)] = groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(30, 79)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 69)] = -groebner_matrix[(30, 80)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 70)] = -groebner_matrix[(30, 81)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 71)] = -groebner_matrix[(30, 82)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 73)] = groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(37, 76)] = -groebner_matrix[(30, 83)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(37, 79)] = groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(37, 82)] = -groebner_matrix[(30, 84)] / (groebner_matrix[(30, 64)]);
}

pub fn s_polynomial38(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(38, 30)] = -groebner_matrix[(31, 65)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 31)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(31, 67)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 33)] = -groebner_matrix[(31, 69)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 42)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(38, 46)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(31, 72)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 48)] = -groebner_matrix[(31, 74)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 49)] = -groebner_matrix[(31, 75)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 50)] = -groebner_matrix[(31, 76)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 51)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(38, 55)] = -groebner_matrix[(31, 77)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 63)] = groebner_matrix[(12, 81)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(38, 67)] = groebner_matrix[(12, 82)] / (groebner_matrix[(12, 70)])
        - groebner_matrix[(31, 78)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 69)] = -groebner_matrix[(31, 80)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 70)] = -groebner_matrix[(31, 81)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 71)] = -groebner_matrix[(31, 82)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 72)] = groebner_matrix[(12, 83)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(38, 76)] = -groebner_matrix[(31, 83)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(38, 78)] = groebner_matrix[(12, 84)] / (groebner_matrix[(12, 70)]);
    groebner_matrix[(38, 82)] = -groebner_matrix[(31, 84)] / (groebner_matrix[(31, 63)]);
}

pub fn s_polynomial39(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(39, 28)] = -groebner_matrix[(32, 67)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 29)] = -groebner_matrix[(32, 68)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 30)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(32, 69)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 33)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(39, 42)] = -groebner_matrix[(32, 72)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 43)] = -groebner_matrix[(32, 73)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 44)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(32, 74)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 45)] = -groebner_matrix[(32, 75)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 48)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(39, 49)] = -groebner_matrix[(32, 76)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 53)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(39, 54)] = -groebner_matrix[(32, 77)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 63)] = -groebner_matrix[(32, 78)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 64)] = -groebner_matrix[(32, 79)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 65)] = groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(32, 80)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 66)] = -groebner_matrix[(32, 81)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 69)] = groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(39, 70)] = -groebner_matrix[(32, 82)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 74)] = groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(39, 75)] = -groebner_matrix[(32, 83)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(39, 80)] = groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(39, 81)] = -groebner_matrix[(32, 84)] / (groebner_matrix[(32, 65)]);
}

pub fn s_polynomial40(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(40, 21)] = -groebner_matrix[(30, 65)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 29)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(30, 68)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 30)] = -groebner_matrix[(30, 69)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 32)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(40, 43)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(30, 73)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 44)] = -groebner_matrix[(30, 74)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 45)] = -groebner_matrix[(30, 75)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 47)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(40, 49)] = -groebner_matrix[(30, 76)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 52)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(40, 54)] = -groebner_matrix[(30, 77)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 64)] = groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(30, 79)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 65)] = -groebner_matrix[(30, 80)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 66)] = -groebner_matrix[(30, 81)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 68)] = groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(40, 70)] = -groebner_matrix[(30, 82)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 73)] = groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(40, 75)] = -groebner_matrix[(30, 83)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(40, 79)] = groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(40, 81)] = -groebner_matrix[(30, 84)] / (groebner_matrix[(30, 64)]);
}

pub fn s_polynomial41(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(41, 21)] = -groebner_matrix[(31, 65)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 28)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(31, 67)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 30)] = -groebner_matrix[(31, 69)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 31)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(41, 42)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(31, 72)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 44)] = -groebner_matrix[(31, 74)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 45)] = -groebner_matrix[(31, 75)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 46)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(41, 49)] = -groebner_matrix[(31, 76)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 51)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(41, 54)] = -groebner_matrix[(31, 77)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 63)] = groebner_matrix[(10, 81)] / (groebner_matrix[(10, 66)])
        - groebner_matrix[(31, 78)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 65)] = -groebner_matrix[(31, 80)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 66)] = -groebner_matrix[(31, 81)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 67)] = groebner_matrix[(10, 82)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(41, 70)] = -groebner_matrix[(31, 82)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 72)] = groebner_matrix[(10, 83)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(41, 75)] = -groebner_matrix[(31, 83)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(41, 78)] = groebner_matrix[(10, 84)] / (groebner_matrix[(10, 66)]);
    groebner_matrix[(41, 81)] = -groebner_matrix[(31, 84)] / (groebner_matrix[(31, 63)]);
}

pub fn s_polynomial42(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(42, 18)] = groebner_matrix[(30, 65)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 23)] = -groebner_matrix[(32, 67)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 24)] = -groebner_matrix[(32, 68)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 26)] = groebner_matrix[(30, 68)] / (groebner_matrix[(30, 64)])
        - groebner_matrix[(32, 69)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 27)] = groebner_matrix[(30, 69)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 37)] = -groebner_matrix[(32, 72)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 38)] = -groebner_matrix[(32, 73)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 40)] = groebner_matrix[(30, 73)] / (groebner_matrix[(30, 64)])
        - groebner_matrix[(32, 74)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 41)] = groebner_matrix[(30, 74)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 43)] = -groebner_matrix[(32, 75)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 44)] = groebner_matrix[(30, 75)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 47)] = -groebner_matrix[(32, 76)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 48)] = groebner_matrix[(30, 76)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 52)] = -groebner_matrix[(32, 77)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 53)] = groebner_matrix[(30, 77)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 58)] = -groebner_matrix[(32, 78)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 59)] = -groebner_matrix[(32, 79)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 61)] = groebner_matrix[(30, 79)] / (groebner_matrix[(30, 64)])
        - groebner_matrix[(32, 80)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 62)] = groebner_matrix[(30, 80)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 64)] = -groebner_matrix[(32, 81)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 65)] = groebner_matrix[(30, 81)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 68)] = -groebner_matrix[(32, 82)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 69)] = groebner_matrix[(30, 82)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 73)] = -groebner_matrix[(32, 83)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 74)] = groebner_matrix[(30, 83)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(42, 79)] = -groebner_matrix[(32, 84)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(42, 80)] = groebner_matrix[(30, 84)] / (groebner_matrix[(30, 64)]);
}

pub fn s_polynomial43(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(43, 18)] = groebner_matrix[(31, 65)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 22)] = -groebner_matrix[(32, 67)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 23)] = -groebner_matrix[(32, 68)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 25)] = groebner_matrix[(31, 67)] / (groebner_matrix[(31, 63)])
        - groebner_matrix[(32, 69)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 27)] = groebner_matrix[(31, 69)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 36)] = -groebner_matrix[(32, 72)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 37)] = -groebner_matrix[(32, 73)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 39)] = groebner_matrix[(31, 72)] / (groebner_matrix[(31, 63)])
        - groebner_matrix[(32, 74)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 41)] = groebner_matrix[(31, 74)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 42)] = -groebner_matrix[(32, 75)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 44)] = groebner_matrix[(31, 75)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 46)] = -groebner_matrix[(32, 76)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 48)] = groebner_matrix[(31, 76)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 51)] = -groebner_matrix[(32, 77)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 53)] = groebner_matrix[(31, 77)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 57)] = -groebner_matrix[(32, 78)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 58)] = -groebner_matrix[(32, 79)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 60)] = groebner_matrix[(31, 78)] / (groebner_matrix[(31, 63)])
        - groebner_matrix[(32, 80)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 62)] = groebner_matrix[(31, 80)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 63)] = -groebner_matrix[(32, 81)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 65)] = groebner_matrix[(31, 81)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 67)] = -groebner_matrix[(32, 82)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 69)] = groebner_matrix[(31, 82)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 72)] = -groebner_matrix[(32, 83)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 74)] = groebner_matrix[(31, 83)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(43, 78)] = -groebner_matrix[(32, 84)] / (groebner_matrix[(32, 65)]);
    groebner_matrix[(43, 80)] = groebner_matrix[(31, 84)] / (groebner_matrix[(31, 63)]);
}

pub fn s_polynomial44(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(44, 16)] = groebner_matrix[(30, 65)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 17)] = -groebner_matrix[(31, 65)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 25)] = groebner_matrix[(30, 69)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 26)] = -groebner_matrix[(31, 69)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 39)] = groebner_matrix[(30, 74)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 40)] = -groebner_matrix[(31, 74)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 42)] = groebner_matrix[(30, 75)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 43)] = -groebner_matrix[(31, 75)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 46)] = groebner_matrix[(30, 76)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 47)] = -groebner_matrix[(31, 76)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 51)] = groebner_matrix[(30, 77)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 52)] = -groebner_matrix[(31, 77)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 60)] = groebner_matrix[(30, 80)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 61)] = -groebner_matrix[(31, 80)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 63)] = groebner_matrix[(30, 81)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 64)] = -groebner_matrix[(31, 81)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 67)] = groebner_matrix[(30, 82)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 68)] = -groebner_matrix[(31, 82)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 72)] = groebner_matrix[(30, 83)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 73)] = -groebner_matrix[(31, 83)] / (groebner_matrix[(31, 63)]);
    groebner_matrix[(44, 78)] = groebner_matrix[(30, 84)] / (groebner_matrix[(30, 64)]);
    groebner_matrix[(44, 79)] = -groebner_matrix[(31, 84)] / (groebner_matrix[(31, 63)]);
}

pub fn s_polynomial45(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(45, 14)] = groebner_matrix[(19, 56)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(45, 15)] = -groebner_matrix[(27, 56)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 30)] = -groebner_matrix[(27, 65)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 33)] = -groebner_matrix[(27, 69)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 44)] = groebner_matrix[(19, 75)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(45, 48)] = groebner_matrix[(19, 76)] / (groebner_matrix[(19, 55)])
        - groebner_matrix[(27, 74)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 49)] = -groebner_matrix[(27, 75)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 50)] = -groebner_matrix[(27, 76)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 53)] = groebner_matrix[(19, 77)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(45, 55)] = -groebner_matrix[(27, 77)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 65)] = groebner_matrix[(19, 81)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(45, 69)] = groebner_matrix[(19, 82)] / (groebner_matrix[(19, 55)])
        - groebner_matrix[(27, 80)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 70)] = -groebner_matrix[(27, 81)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 71)] = -groebner_matrix[(27, 82)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 74)] = groebner_matrix[(19, 83)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(45, 76)] = -groebner_matrix[(27, 83)] / (groebner_matrix[(27, 53)]);
    groebner_matrix[(45, 80)] = groebner_matrix[(19, 84)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(45, 82)] = -groebner_matrix[(27, 84)] / (groebner_matrix[(27, 53)]);
}

pub fn s_polynomial46(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(46, 13)] = groebner_matrix[(19, 56)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(46, 15)] = -groebner_matrix[(28, 56)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 29)] = -groebner_matrix[(28, 64)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 32)] = -groebner_matrix[(28, 68)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 43)] = groebner_matrix[(19, 75)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(46, 47)] = groebner_matrix[(19, 76)] / (groebner_matrix[(19, 55)])
        - groebner_matrix[(28, 73)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 49)] = -groebner_matrix[(28, 75)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 50)] = -groebner_matrix[(28, 76)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 52)] = groebner_matrix[(19, 77)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(46, 55)] = -groebner_matrix[(28, 77)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 64)] = groebner_matrix[(19, 81)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(46, 68)] = groebner_matrix[(19, 82)] / (groebner_matrix[(19, 55)])
        - groebner_matrix[(28, 79)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 70)] = -groebner_matrix[(28, 81)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 71)] = -groebner_matrix[(28, 82)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 73)] = groebner_matrix[(19, 83)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(46, 76)] = -groebner_matrix[(28, 83)] / (groebner_matrix[(28, 52)]);
    groebner_matrix[(46, 79)] = groebner_matrix[(19, 84)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(46, 82)] = -groebner_matrix[(28, 84)] / (groebner_matrix[(28, 52)]);
}

pub fn s_polynomial47(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_matrix[(47, 12)] = groebner_matrix[(19, 56)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(47, 15)] = -groebner_matrix[(29, 56)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 28)] = -groebner_matrix[(29, 63)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 31)] = -groebner_matrix[(29, 67)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 42)] = groebner_matrix[(19, 75)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(47, 46)] = groebner_matrix[(19, 76)] / (groebner_matrix[(19, 55)])
        - groebner_matrix[(29, 72)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 49)] = -groebner_matrix[(29, 75)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 50)] = -groebner_matrix[(29, 76)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 51)] = groebner_matrix[(19, 77)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(47, 55)] = -groebner_matrix[(29, 77)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 63)] = groebner_matrix[(19, 81)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(47, 67)] = groebner_matrix[(19, 82)] / (groebner_matrix[(19, 55)])
        - groebner_matrix[(29, 78)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 70)] = -groebner_matrix[(29, 81)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 71)] = -groebner_matrix[(29, 82)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 72)] = groebner_matrix[(19, 83)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(47, 76)] = -groebner_matrix[(29, 83)] / (groebner_matrix[(29, 51)]);
    groebner_matrix[(47, 78)] = groebner_matrix[(19, 84)] / (groebner_matrix[(19, 55)]);
    groebner_matrix[(47, 82)] = -groebner_matrix[(29, 84)] / (groebner_matrix[(29, 51)]);
}
