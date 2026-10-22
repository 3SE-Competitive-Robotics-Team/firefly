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

pub fn s_polynomial6(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(6, 60)] = groebner_matrix[(0, 60)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 60)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 61)] = groebner_matrix[(0, 61)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 61)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 62)] = groebner_matrix[(0, 62)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 62)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 63)] = groebner_matrix[(0, 63)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 63)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 64)] = groebner_matrix[(0, 64)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 64)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 65)] = groebner_matrix[(0, 65)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 65)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 66)] = groebner_matrix[(0, 66)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 66)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 67)] = groebner_matrix[(0, 67)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 67)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 68)] = groebner_matrix[(0, 68)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 68)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 69)] = groebner_matrix[(0, 69)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 69)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 70)] = groebner_matrix[(0, 70)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 70)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 71)] = groebner_matrix[(0, 71)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 71)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 72)] = groebner_matrix[(0, 72)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 72)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 73)] = groebner_matrix[(0, 73)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 73)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 74)] = groebner_matrix[(0, 74)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 74)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 75)] = groebner_matrix[(0, 75)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 75)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 76)] = groebner_matrix[(0, 76)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 76)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 77)] = groebner_matrix[(0, 77)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 77)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 78)] = groebner_matrix[(0, 78)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 78)] / (groebner_matrix[(1, 59)]);
    groebner_matrix[(6, 79)] = groebner_matrix[(0, 79)] / (groebner_matrix[(0, 59)])
        - groebner_matrix[(1, 79)] / (groebner_matrix[(1, 59)]);
}

pub fn s_polynomial7(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(7, 60)] = groebner_matrix[(1, 60)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 60)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 61)] = groebner_matrix[(1, 61)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 61)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 62)] = groebner_matrix[(1, 62)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 62)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 63)] = groebner_matrix[(1, 63)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 63)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 64)] = groebner_matrix[(1, 64)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 64)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 65)] = groebner_matrix[(1, 65)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 65)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 66)] = groebner_matrix[(1, 66)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 66)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 67)] = groebner_matrix[(1, 67)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 67)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 68)] = groebner_matrix[(1, 68)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 68)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 69)] = groebner_matrix[(1, 69)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 69)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 70)] = groebner_matrix[(1, 70)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 70)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 71)] = groebner_matrix[(1, 71)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 71)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 72)] = groebner_matrix[(1, 72)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 72)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 73)] = groebner_matrix[(1, 73)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 73)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 74)] = groebner_matrix[(1, 74)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 74)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 75)] = groebner_matrix[(1, 75)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 75)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 76)] = groebner_matrix[(1, 76)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 76)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 77)] = groebner_matrix[(1, 77)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 77)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 78)] = groebner_matrix[(1, 78)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 78)] / (groebner_matrix[(2, 59)]);
    groebner_matrix[(7, 79)] = groebner_matrix[(1, 79)] / (groebner_matrix[(1, 59)])
        - groebner_matrix[(2, 79)] / (groebner_matrix[(2, 59)]);
}

pub fn s_polynomial8(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(8, 60)] = groebner_matrix[(2, 60)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 60)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 61)] = groebner_matrix[(2, 61)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 61)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 62)] = groebner_matrix[(2, 62)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 62)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 63)] = groebner_matrix[(2, 63)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 63)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 64)] = groebner_matrix[(2, 64)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 64)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 65)] = groebner_matrix[(2, 65)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 65)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 66)] = groebner_matrix[(2, 66)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 66)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 67)] = groebner_matrix[(2, 67)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 67)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 68)] = groebner_matrix[(2, 68)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 68)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 69)] = groebner_matrix[(2, 69)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 69)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 70)] = groebner_matrix[(2, 70)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 70)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 71)] = groebner_matrix[(2, 71)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 71)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 72)] = groebner_matrix[(2, 72)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 72)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 73)] = groebner_matrix[(2, 73)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 73)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 74)] = groebner_matrix[(2, 74)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 74)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 75)] = groebner_matrix[(2, 75)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 75)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 76)] = groebner_matrix[(2, 76)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 76)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 77)] = groebner_matrix[(2, 77)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 77)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 78)] = groebner_matrix[(2, 78)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 78)] / (groebner_matrix[(3, 59)]);
    groebner_matrix[(8, 79)] = groebner_matrix[(2, 79)] / (groebner_matrix[(2, 59)])
        - groebner_matrix[(3, 79)] / (groebner_matrix[(3, 59)]);
}

pub fn s_polynomial9(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(9, 60)] = groebner_matrix[(3, 60)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 60)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 61)] = groebner_matrix[(3, 61)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 61)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 62)] = groebner_matrix[(3, 62)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 62)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 63)] = groebner_matrix[(3, 63)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 63)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 64)] = groebner_matrix[(3, 64)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 64)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 65)] = groebner_matrix[(3, 65)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 65)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 66)] = groebner_matrix[(3, 66)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 66)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 67)] = groebner_matrix[(3, 67)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 67)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 68)] = groebner_matrix[(3, 68)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 68)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 69)] = groebner_matrix[(3, 69)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 69)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 70)] = groebner_matrix[(3, 70)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 70)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 71)] = groebner_matrix[(3, 71)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 71)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 72)] = groebner_matrix[(3, 72)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 72)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 73)] = groebner_matrix[(3, 73)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 73)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 74)] = groebner_matrix[(3, 74)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 74)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 75)] = groebner_matrix[(3, 75)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 75)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 76)] = groebner_matrix[(3, 76)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 76)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 77)] = groebner_matrix[(3, 77)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 77)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 78)] = groebner_matrix[(3, 78)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 78)] / (groebner_matrix[(4, 59)]);
    groebner_matrix[(9, 79)] = groebner_matrix[(3, 79)] / (groebner_matrix[(3, 59)])
        - groebner_matrix[(4, 79)] / (groebner_matrix[(4, 59)]);
}

pub fn s_polynomial10(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(10, 60)] = groebner_matrix[(4, 60)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 60)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 61)] = groebner_matrix[(4, 61)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 61)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 62)] = groebner_matrix[(4, 62)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 62)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 63)] = groebner_matrix[(4, 63)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 63)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 64)] = groebner_matrix[(4, 64)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 64)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 65)] = groebner_matrix[(4, 65)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 65)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 66)] = groebner_matrix[(4, 66)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 66)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 67)] = groebner_matrix[(4, 67)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 67)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 68)] = groebner_matrix[(4, 68)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 68)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 69)] = groebner_matrix[(4, 69)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 69)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 70)] = groebner_matrix[(4, 70)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 70)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 71)] = groebner_matrix[(4, 71)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 71)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 72)] = groebner_matrix[(4, 72)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 72)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 73)] = groebner_matrix[(4, 73)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 73)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 74)] = groebner_matrix[(4, 74)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 74)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 75)] = groebner_matrix[(4, 75)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 75)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 76)] = groebner_matrix[(4, 76)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 76)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 77)] = groebner_matrix[(4, 77)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 77)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 78)] = groebner_matrix[(4, 78)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 78)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(10, 79)] = groebner_matrix[(4, 79)] / (groebner_matrix[(4, 59)])
        - groebner_matrix[(5, 79)] / (groebner_matrix[(5, 59)]);
}

pub fn s_polynomial11(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(11, 33)] = groebner_matrix[(9, 64)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 35)] = -groebner_matrix[(10, 65)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 36)] = -groebner_matrix[(10, 66)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 37)] = groebner_matrix[(9, 65)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 38)] = groebner_matrix[(9, 66)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(10, 67)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 39)] = groebner_matrix[(9, 67)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 41)] = -groebner_matrix[(10, 68)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 42)] = groebner_matrix[(9, 68)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 45)] = -groebner_matrix[(10, 69)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 46)] = -groebner_matrix[(10, 70)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 47)] = groebner_matrix[(9, 69)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 48)] = groebner_matrix[(9, 70)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(10, 71)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 49)] = groebner_matrix[(9, 71)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 51)] = -groebner_matrix[(10, 72)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 52)] = groebner_matrix[(9, 72)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 55)] = -groebner_matrix[(10, 73)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 56)] = groebner_matrix[(9, 73)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 60)] = -groebner_matrix[(10, 74)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 61)] = -groebner_matrix[(10, 75)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 62)] = groebner_matrix[(9, 74)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 63)] = groebner_matrix[(9, 75)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(10, 76)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 64)] = groebner_matrix[(9, 76)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 66)] = -groebner_matrix[(10, 77)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 67)] = groebner_matrix[(9, 77)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 70)] = -groebner_matrix[(10, 78)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 71)] = groebner_matrix[(9, 78)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(11, 75)] = -groebner_matrix[(10, 79)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(11, 76)] = groebner_matrix[(9, 79)] / (groebner_matrix[(9, 63)]);
}

pub fn s_polynomial12(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(12, 32)] = groebner_matrix[(8, 63)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 33)] = groebner_matrix[(8, 64)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 34)] = -groebner_matrix[(10, 65)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 35)] = -groebner_matrix[(10, 66)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 37)] = groebner_matrix[(8, 65)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(10, 67)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 38)] = groebner_matrix[(8, 66)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 39)] = groebner_matrix[(8, 67)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 40)] = -groebner_matrix[(10, 68)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 42)] = groebner_matrix[(8, 68)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 44)] = -groebner_matrix[(10, 69)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 45)] = -groebner_matrix[(10, 70)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 47)] = groebner_matrix[(8, 69)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(10, 71)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 48)] = groebner_matrix[(8, 70)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 49)] = groebner_matrix[(8, 71)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 50)] = -groebner_matrix[(10, 72)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 52)] = groebner_matrix[(8, 72)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 54)] = -groebner_matrix[(10, 73)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 56)] = groebner_matrix[(8, 73)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 59)] = -groebner_matrix[(10, 74)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 60)] = -groebner_matrix[(10, 75)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 62)] = groebner_matrix[(8, 74)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(10, 76)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 63)] = groebner_matrix[(8, 75)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 64)] = groebner_matrix[(8, 76)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 65)] = -groebner_matrix[(10, 77)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 67)] = groebner_matrix[(8, 77)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 69)] = -groebner_matrix[(10, 78)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 71)] = groebner_matrix[(8, 78)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(12, 74)] = -groebner_matrix[(10, 79)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(12, 76)] = groebner_matrix[(8, 79)] / (groebner_matrix[(8, 62)]);
}

pub fn s_polynomial13(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(13, 31)] = groebner_matrix[(7, 62)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 32)] = groebner_matrix[(7, 63)] / (groebner_matrix[(7, 61)])
        - groebner_matrix[(9, 64)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 33)] = groebner_matrix[(7, 64)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 35)] = -groebner_matrix[(9, 65)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 36)] = -groebner_matrix[(9, 66)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 37)] = groebner_matrix[(7, 65)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 38)] = groebner_matrix[(7, 66)] / (groebner_matrix[(7, 61)])
        - groebner_matrix[(9, 67)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 39)] = groebner_matrix[(7, 67)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 41)] = -groebner_matrix[(9, 68)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 42)] = groebner_matrix[(7, 68)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 45)] = -groebner_matrix[(9, 69)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 46)] = -groebner_matrix[(9, 70)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 47)] = groebner_matrix[(7, 69)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 48)] = groebner_matrix[(7, 70)] / (groebner_matrix[(7, 61)])
        - groebner_matrix[(9, 71)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 49)] = groebner_matrix[(7, 71)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 51)] = -groebner_matrix[(9, 72)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 52)] = groebner_matrix[(7, 72)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 55)] = -groebner_matrix[(9, 73)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 56)] = groebner_matrix[(7, 73)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 60)] = -groebner_matrix[(9, 74)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 61)] = -groebner_matrix[(9, 75)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 62)] = groebner_matrix[(7, 74)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 63)] = groebner_matrix[(7, 75)] / (groebner_matrix[(7, 61)])
        - groebner_matrix[(9, 76)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 64)] = groebner_matrix[(7, 76)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 66)] = -groebner_matrix[(9, 77)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 67)] = groebner_matrix[(7, 77)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 70)] = -groebner_matrix[(9, 78)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 71)] = groebner_matrix[(7, 78)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(13, 75)] = -groebner_matrix[(9, 79)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(13, 76)] = groebner_matrix[(7, 79)] / (groebner_matrix[(7, 61)]);
}

pub fn s_polynomial14(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(14, 30)] = groebner_matrix[(6, 61)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(8, 63)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 31)] = groebner_matrix[(6, 62)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 32)] = groebner_matrix[(6, 63)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(8, 64)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 33)] = groebner_matrix[(6, 64)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 35)] = -groebner_matrix[(8, 65)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 36)] = -groebner_matrix[(8, 66)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 37)] = groebner_matrix[(6, 65)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 38)] = groebner_matrix[(6, 66)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(8, 67)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 39)] = groebner_matrix[(6, 67)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 41)] = -groebner_matrix[(8, 68)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 42)] = groebner_matrix[(6, 68)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 45)] = -groebner_matrix[(8, 69)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 46)] = -groebner_matrix[(8, 70)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 47)] = groebner_matrix[(6, 69)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 48)] = groebner_matrix[(6, 70)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(8, 71)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 49)] = groebner_matrix[(6, 71)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 51)] = -groebner_matrix[(8, 72)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 52)] = groebner_matrix[(6, 72)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 55)] = -groebner_matrix[(8, 73)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 56)] = groebner_matrix[(6, 73)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 60)] = -groebner_matrix[(8, 74)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 61)] = -groebner_matrix[(8, 75)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 62)] = groebner_matrix[(6, 74)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 63)] = groebner_matrix[(6, 75)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(8, 76)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 64)] = groebner_matrix[(6, 76)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 66)] = -groebner_matrix[(8, 77)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 67)] = groebner_matrix[(6, 77)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 70)] = -groebner_matrix[(8, 78)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 71)] = groebner_matrix[(6, 78)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(14, 75)] = -groebner_matrix[(8, 79)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(14, 76)] = groebner_matrix[(6, 79)] / (groebner_matrix[(6, 60)]);
}

pub fn s_polynomial15(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(15, 30)] = groebner_matrix[(6, 61)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 31)] = groebner_matrix[(6, 62)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(9, 64)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 32)] = groebner_matrix[(6, 63)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 33)] = groebner_matrix[(6, 64)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 34)] = -groebner_matrix[(9, 65)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 35)] = -groebner_matrix[(9, 66)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 37)] = groebner_matrix[(6, 65)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(9, 67)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 38)] = groebner_matrix[(6, 66)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 39)] = groebner_matrix[(6, 67)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 40)] = -groebner_matrix[(9, 68)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 42)] = groebner_matrix[(6, 68)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 44)] = -groebner_matrix[(9, 69)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 45)] = -groebner_matrix[(9, 70)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 47)] = groebner_matrix[(6, 69)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(9, 71)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 48)] = groebner_matrix[(6, 70)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 49)] = groebner_matrix[(6, 71)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 50)] = -groebner_matrix[(9, 72)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 52)] = groebner_matrix[(6, 72)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 54)] = -groebner_matrix[(9, 73)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 56)] = groebner_matrix[(6, 73)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 59)] = -groebner_matrix[(9, 74)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 60)] = -groebner_matrix[(9, 75)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 62)] = groebner_matrix[(6, 74)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(9, 76)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 63)] = groebner_matrix[(6, 75)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 64)] = groebner_matrix[(6, 76)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 65)] = -groebner_matrix[(9, 77)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 67)] = groebner_matrix[(6, 77)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 69)] = -groebner_matrix[(9, 78)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 71)] = groebner_matrix[(6, 78)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(15, 74)] = -groebner_matrix[(9, 79)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(15, 76)] = groebner_matrix[(6, 79)] / (groebner_matrix[(6, 60)]);
}

pub fn s_polynomial16(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(16, 29)] = groebner_matrix[(5, 60)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(8, 63)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 30)] = groebner_matrix[(5, 61)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 31)] = groebner_matrix[(5, 62)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(8, 64)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 32)] = groebner_matrix[(5, 63)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 33)] = groebner_matrix[(5, 64)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 34)] = -groebner_matrix[(8, 65)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 35)] = -groebner_matrix[(8, 66)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 37)] = groebner_matrix[(5, 65)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(8, 67)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 38)] = groebner_matrix[(5, 66)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 39)] = groebner_matrix[(5, 67)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 40)] = -groebner_matrix[(8, 68)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 42)] = groebner_matrix[(5, 68)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 44)] = -groebner_matrix[(8, 69)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 45)] = -groebner_matrix[(8, 70)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 47)] = groebner_matrix[(5, 69)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(8, 71)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 48)] = groebner_matrix[(5, 70)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 49)] = groebner_matrix[(5, 71)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 50)] = -groebner_matrix[(8, 72)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 52)] = groebner_matrix[(5, 72)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 54)] = -groebner_matrix[(8, 73)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 56)] = groebner_matrix[(5, 73)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 59)] = -groebner_matrix[(8, 74)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 60)] = -groebner_matrix[(8, 75)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 62)] = groebner_matrix[(5, 74)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(8, 76)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 63)] = groebner_matrix[(5, 75)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 64)] = groebner_matrix[(5, 76)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 65)] = -groebner_matrix[(8, 77)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 67)] = groebner_matrix[(5, 77)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 69)] = -groebner_matrix[(8, 78)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 71)] = groebner_matrix[(5, 78)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(16, 74)] = -groebner_matrix[(8, 79)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(16, 76)] = groebner_matrix[(5, 79)] / (groebner_matrix[(5, 59)]);
}

pub fn s_polynomial17(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(17, 27)] = groebner_matrix[(6, 61)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 28)] = -groebner_matrix[(7, 62)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 29)] = groebner_matrix[(6, 62)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(7, 63)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 30)] = groebner_matrix[(6, 63)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 31)] = -groebner_matrix[(7, 64)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 32)] = groebner_matrix[(6, 64)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 34)] = -groebner_matrix[(7, 65)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 35)] = groebner_matrix[(6, 65)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(7, 66)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 36)] = groebner_matrix[(6, 66)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 37)] = -groebner_matrix[(7, 67)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 38)] = groebner_matrix[(6, 67)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 40)] = -groebner_matrix[(7, 68)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 41)] = groebner_matrix[(6, 68)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 44)] = -groebner_matrix[(7, 69)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 45)] = groebner_matrix[(6, 69)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(7, 70)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 46)] = groebner_matrix[(6, 70)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 47)] = -groebner_matrix[(7, 71)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 48)] = groebner_matrix[(6, 71)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 50)] = -groebner_matrix[(7, 72)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 51)] = groebner_matrix[(6, 72)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 54)] = -groebner_matrix[(7, 73)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 55)] = groebner_matrix[(6, 73)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 59)] = -groebner_matrix[(7, 74)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 60)] = groebner_matrix[(6, 74)] / (groebner_matrix[(6, 60)])
        - groebner_matrix[(7, 75)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 61)] = groebner_matrix[(6, 75)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 62)] = -groebner_matrix[(7, 76)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 63)] = groebner_matrix[(6, 76)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 65)] = -groebner_matrix[(7, 77)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 66)] = groebner_matrix[(6, 77)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 69)] = -groebner_matrix[(7, 78)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 70)] = groebner_matrix[(6, 78)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(17, 74)] = -groebner_matrix[(7, 79)] / (groebner_matrix[(7, 61)]);
    groebner_matrix[(17, 75)] = groebner_matrix[(6, 79)] / (groebner_matrix[(6, 60)]);
}

pub fn s_polynomial18(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(18, 26)] = groebner_matrix[(5, 60)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(6, 61)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 27)] = groebner_matrix[(5, 61)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 28)] = -groebner_matrix[(6, 62)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 29)] = groebner_matrix[(5, 62)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(6, 63)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 30)] = groebner_matrix[(5, 63)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 31)] = -groebner_matrix[(6, 64)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 32)] = groebner_matrix[(5, 64)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 34)] = -groebner_matrix[(6, 65)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 35)] = groebner_matrix[(5, 65)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(6, 66)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 36)] = groebner_matrix[(5, 66)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 37)] = -groebner_matrix[(6, 67)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 38)] = groebner_matrix[(5, 67)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 40)] = -groebner_matrix[(6, 68)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 41)] = groebner_matrix[(5, 68)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 44)] = -groebner_matrix[(6, 69)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 45)] = groebner_matrix[(5, 69)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(6, 70)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 46)] = groebner_matrix[(5, 70)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 47)] = -groebner_matrix[(6, 71)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 48)] = groebner_matrix[(5, 71)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 50)] = -groebner_matrix[(6, 72)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 51)] = groebner_matrix[(5, 72)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 54)] = -groebner_matrix[(6, 73)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 55)] = groebner_matrix[(5, 73)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 59)] = -groebner_matrix[(6, 74)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 60)] = groebner_matrix[(5, 74)] / (groebner_matrix[(5, 59)])
        - groebner_matrix[(6, 75)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 61)] = groebner_matrix[(5, 75)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 62)] = -groebner_matrix[(6, 76)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 63)] = groebner_matrix[(5, 76)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 65)] = -groebner_matrix[(6, 77)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 66)] = groebner_matrix[(5, 77)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 69)] = -groebner_matrix[(6, 78)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 70)] = groebner_matrix[(5, 78)] / (groebner_matrix[(5, 59)]);
    groebner_matrix[(18, 74)] = -groebner_matrix[(6, 79)] / (groebner_matrix[(6, 60)]);
    groebner_matrix[(18, 75)] = groebner_matrix[(5, 79)] / (groebner_matrix[(5, 59)]);
}

pub fn s_polynomial19(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(19, 17)] = groebner_matrix[(14, 50)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 54)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 18)] = groebner_matrix[(14, 51)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 55)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 19)] = groebner_matrix[(14, 52)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 56)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 20)] = groebner_matrix[(14, 53)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 57)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 21)] = groebner_matrix[(14, 54)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 22)] = groebner_matrix[(14, 55)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 23)] = groebner_matrix[(14, 56)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 24)] = groebner_matrix[(14, 57)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 58)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 25)] = groebner_matrix[(14, 58)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 40)] = -groebner_matrix[(18, 65)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 41)] = -groebner_matrix[(18, 66)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 42)] = -groebner_matrix[(18, 67)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 43)] = -groebner_matrix[(18, 68)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 50)] = groebner_matrix[(14, 65)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 69)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 51)] = groebner_matrix[(14, 66)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 70)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 52)] = groebner_matrix[(14, 67)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 71)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 53)] = groebner_matrix[(14, 68)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 72)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 54)] = groebner_matrix[(14, 69)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 55)] = groebner_matrix[(14, 70)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 56)] = groebner_matrix[(14, 71)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 57)] = groebner_matrix[(14, 72)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 73)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 58)] = groebner_matrix[(14, 73)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 65)] = -groebner_matrix[(18, 74)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 66)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 67)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 68)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 69)] = groebner_matrix[(14, 74)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 70)] = groebner_matrix[(14, 75)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 71)] = groebner_matrix[(14, 76)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 72)] = groebner_matrix[(14, 77)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(18, 78)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 73)] = groebner_matrix[(14, 78)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(19, 77)] = -groebner_matrix[(18, 79)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(19, 78)] = groebner_matrix[(14, 79)] / (groebner_matrix[(14, 43)]);
}

pub fn s_polynomial20(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(20, 10)] = groebner_matrix[(13, 43)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 53)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 17)] = groebner_matrix[(13, 50)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 54)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 18)] = groebner_matrix[(13, 51)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 55)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 19)] = groebner_matrix[(13, 52)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 56)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 20)] = groebner_matrix[(13, 53)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 57)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 21)] = groebner_matrix[(13, 54)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 22)] = groebner_matrix[(13, 55)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 23)] = groebner_matrix[(13, 56)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 24)] = groebner_matrix[(13, 57)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 58)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 25)] = groebner_matrix[(13, 58)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 40)] = -groebner_matrix[(17, 65)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 41)] = -groebner_matrix[(17, 66)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 42)] = -groebner_matrix[(17, 67)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 43)] = -groebner_matrix[(17, 68)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 50)] = groebner_matrix[(13, 65)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 69)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 51)] = groebner_matrix[(13, 66)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 70)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 52)] = groebner_matrix[(13, 67)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 71)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 53)] = groebner_matrix[(13, 68)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 72)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 54)] = groebner_matrix[(13, 69)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 55)] = groebner_matrix[(13, 70)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 56)] = groebner_matrix[(13, 71)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 57)] = groebner_matrix[(13, 72)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 73)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 58)] = groebner_matrix[(13, 73)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 65)] = -groebner_matrix[(17, 74)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 66)] = -groebner_matrix[(17, 75)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 67)] = -groebner_matrix[(17, 76)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 68)] = -groebner_matrix[(17, 77)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 69)] = groebner_matrix[(13, 74)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 70)] = groebner_matrix[(13, 75)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 71)] = groebner_matrix[(13, 76)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 72)] = groebner_matrix[(13, 77)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(17, 78)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 73)] = groebner_matrix[(13, 78)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(20, 77)] = -groebner_matrix[(17, 79)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(20, 78)] = groebner_matrix[(13, 79)] / (groebner_matrix[(13, 42)]);
}

pub fn s_polynomial21(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(21, 10)] = groebner_matrix[(13, 43)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 14)] = -groebner_matrix[(18, 54)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 15)] = -groebner_matrix[(18, 55)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 16)] = -groebner_matrix[(18, 56)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 17)] = groebner_matrix[(13, 50)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 18)] = groebner_matrix[(13, 51)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 19)] = groebner_matrix[(13, 52)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(18, 57)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 20)] = groebner_matrix[(13, 53)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 21)] = groebner_matrix[(13, 54)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 22)] = groebner_matrix[(13, 55)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 23)] = groebner_matrix[(13, 56)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(18, 58)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 24)] = groebner_matrix[(13, 57)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 25)] = groebner_matrix[(13, 58)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 37)] = -groebner_matrix[(18, 65)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 38)] = -groebner_matrix[(18, 66)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 39)] = -groebner_matrix[(18, 67)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 42)] = -groebner_matrix[(18, 68)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 47)] = -groebner_matrix[(18, 69)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 48)] = -groebner_matrix[(18, 70)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 49)] = -groebner_matrix[(18, 71)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 50)] = groebner_matrix[(13, 65)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 51)] = groebner_matrix[(13, 66)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 52)] = groebner_matrix[(13, 67)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(18, 72)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 53)] = groebner_matrix[(13, 68)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 54)] = groebner_matrix[(13, 69)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 55)] = groebner_matrix[(13, 70)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 56)] = groebner_matrix[(13, 71)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(18, 73)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 57)] = groebner_matrix[(13, 72)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 58)] = groebner_matrix[(13, 73)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 62)] = -groebner_matrix[(18, 74)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 63)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 64)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 67)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 69)] = groebner_matrix[(13, 74)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 70)] = groebner_matrix[(13, 75)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 71)] = groebner_matrix[(13, 76)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(18, 78)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 72)] = groebner_matrix[(13, 77)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 73)] = groebner_matrix[(13, 78)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(21, 76)] = -groebner_matrix[(18, 79)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(21, 78)] = groebner_matrix[(13, 79)] / (groebner_matrix[(13, 42)]);
}

pub fn s_polynomial22(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(22, 9)] = groebner_matrix[(12, 42)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 52)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 10)] = groebner_matrix[(12, 43)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 53)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 17)] = groebner_matrix[(12, 50)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 54)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 18)] = groebner_matrix[(12, 51)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 55)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 19)] = groebner_matrix[(12, 52)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 56)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 20)] = groebner_matrix[(12, 53)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 57)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 21)] = groebner_matrix[(12, 54)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 22)] = groebner_matrix[(12, 55)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 23)] = groebner_matrix[(12, 56)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 24)] = groebner_matrix[(12, 57)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 58)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 25)] = groebner_matrix[(12, 58)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 40)] = -groebner_matrix[(16, 65)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 41)] = -groebner_matrix[(16, 66)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 42)] = -groebner_matrix[(16, 67)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 43)] = -groebner_matrix[(16, 68)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 50)] = groebner_matrix[(12, 65)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 69)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 51)] = groebner_matrix[(12, 66)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 70)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 52)] = groebner_matrix[(12, 67)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 71)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 53)] = groebner_matrix[(12, 68)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 72)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 54)] = groebner_matrix[(12, 69)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 55)] = groebner_matrix[(12, 70)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 56)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 57)] = groebner_matrix[(12, 72)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 73)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 58)] = groebner_matrix[(12, 73)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 65)] = -groebner_matrix[(16, 74)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 66)] = -groebner_matrix[(16, 75)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 67)] = -groebner_matrix[(16, 76)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 68)] = -groebner_matrix[(16, 77)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 69)] = groebner_matrix[(12, 74)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 70)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 71)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 72)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(16, 78)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 73)] = groebner_matrix[(12, 78)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(22, 77)] = -groebner_matrix[(16, 79)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(22, 78)] = groebner_matrix[(12, 79)] / (groebner_matrix[(12, 41)]);
}

pub fn s_polynomial23(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(23, 9)] = groebner_matrix[(12, 42)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 10)] = groebner_matrix[(12, 43)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 12)] = -groebner_matrix[(18, 54)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 13)] = -groebner_matrix[(18, 55)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 15)] = -groebner_matrix[(18, 56)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 17)] = groebner_matrix[(12, 50)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 18)] = groebner_matrix[(12, 51)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(18, 57)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 19)] = groebner_matrix[(12, 52)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 20)] = groebner_matrix[(12, 53)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 21)] = groebner_matrix[(12, 54)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 22)] = groebner_matrix[(12, 55)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(18, 58)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 23)] = groebner_matrix[(12, 56)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 24)] = groebner_matrix[(12, 57)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 25)] = groebner_matrix[(12, 58)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 35)] = -groebner_matrix[(18, 65)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 36)] = -groebner_matrix[(18, 66)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 38)] = -groebner_matrix[(18, 67)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 41)] = -groebner_matrix[(18, 68)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 45)] = -groebner_matrix[(18, 69)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 46)] = -groebner_matrix[(18, 70)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 48)] = -groebner_matrix[(18, 71)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 50)] = groebner_matrix[(12, 65)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 51)] = groebner_matrix[(12, 66)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(18, 72)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 52)] = groebner_matrix[(12, 67)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 53)] = groebner_matrix[(12, 68)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 54)] = groebner_matrix[(12, 69)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 55)] = groebner_matrix[(12, 70)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(18, 73)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 56)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 57)] = groebner_matrix[(12, 72)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 58)] = groebner_matrix[(12, 73)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 60)] = -groebner_matrix[(18, 74)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 61)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 63)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 66)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 69)] = groebner_matrix[(12, 74)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 70)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(18, 78)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 71)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 72)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 73)] = groebner_matrix[(12, 78)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(23, 75)] = -groebner_matrix[(18, 79)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(23, 78)] = groebner_matrix[(12, 79)] / (groebner_matrix[(12, 41)]);
}

pub fn s_polynomial24(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(24, 8)] = groebner_matrix[(11, 41)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 51)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 9)] = groebner_matrix[(11, 42)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 52)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 10)] = groebner_matrix[(11, 43)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 53)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 17)] = groebner_matrix[(11, 50)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 54)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 18)] = groebner_matrix[(11, 51)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 55)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 19)] = groebner_matrix[(11, 52)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 56)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 20)] = groebner_matrix[(11, 53)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 57)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 21)] = groebner_matrix[(11, 54)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 22)] = groebner_matrix[(11, 55)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 23)] = groebner_matrix[(11, 56)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 24)] = groebner_matrix[(11, 57)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 58)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 25)] = groebner_matrix[(11, 58)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 40)] = -groebner_matrix[(15, 65)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 41)] = -groebner_matrix[(15, 66)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 42)] = -groebner_matrix[(15, 67)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 43)] = -groebner_matrix[(15, 68)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 50)] = groebner_matrix[(11, 65)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 69)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 51)] = groebner_matrix[(11, 66)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 70)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 52)] = groebner_matrix[(11, 67)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 71)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 53)] = groebner_matrix[(11, 68)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 72)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 54)] = groebner_matrix[(11, 69)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 55)] = groebner_matrix[(11, 70)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 56)] = groebner_matrix[(11, 71)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 57)] = groebner_matrix[(11, 72)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 73)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 58)] = groebner_matrix[(11, 73)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 65)] = -groebner_matrix[(15, 74)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 66)] = -groebner_matrix[(15, 75)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 67)] = -groebner_matrix[(15, 76)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 68)] = -groebner_matrix[(15, 77)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 69)] = groebner_matrix[(11, 74)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 70)] = groebner_matrix[(11, 75)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 71)] = groebner_matrix[(11, 76)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 72)] = groebner_matrix[(11, 77)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(15, 78)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 73)] = groebner_matrix[(11, 78)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(24, 77)] = -groebner_matrix[(15, 79)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(24, 78)] = groebner_matrix[(11, 79)] / (groebner_matrix[(11, 40)]);
}

pub fn s_polynomial25(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(25, 8)] = groebner_matrix[(11, 41)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 9)] = groebner_matrix[(11, 42)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 10)] = groebner_matrix[(11, 43)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 11)] = -groebner_matrix[(18, 54)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 12)] = -groebner_matrix[(18, 55)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 14)] = -groebner_matrix[(18, 56)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 17)] = groebner_matrix[(11, 50)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(18, 57)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 18)] = groebner_matrix[(11, 51)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 19)] = groebner_matrix[(11, 52)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 20)] = groebner_matrix[(11, 53)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 21)] = groebner_matrix[(11, 54)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(18, 58)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 22)] = groebner_matrix[(11, 55)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 23)] = groebner_matrix[(11, 56)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 24)] = groebner_matrix[(11, 57)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 25)] = groebner_matrix[(11, 58)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 34)] = -groebner_matrix[(18, 65)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 35)] = -groebner_matrix[(18, 66)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 37)] = -groebner_matrix[(18, 67)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 40)] = -groebner_matrix[(18, 68)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 44)] = -groebner_matrix[(18, 69)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 45)] = -groebner_matrix[(18, 70)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 47)] = -groebner_matrix[(18, 71)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 50)] = groebner_matrix[(11, 65)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(18, 72)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 51)] = groebner_matrix[(11, 66)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 52)] = groebner_matrix[(11, 67)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 53)] = groebner_matrix[(11, 68)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 54)] = groebner_matrix[(11, 69)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(18, 73)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 55)] = groebner_matrix[(11, 70)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 56)] = groebner_matrix[(11, 71)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 57)] = groebner_matrix[(11, 72)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 58)] = groebner_matrix[(11, 73)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 59)] = -groebner_matrix[(18, 74)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 60)] = -groebner_matrix[(18, 75)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 62)] = -groebner_matrix[(18, 76)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 65)] = -groebner_matrix[(18, 77)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 69)] = groebner_matrix[(11, 74)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(18, 78)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 70)] = groebner_matrix[(11, 75)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 71)] = groebner_matrix[(11, 76)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 72)] = groebner_matrix[(11, 77)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 73)] = groebner_matrix[(11, 78)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(25, 74)] = -groebner_matrix[(18, 79)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(25, 78)] = groebner_matrix[(11, 79)] / (groebner_matrix[(11, 40)]);
}

pub fn s_polynomial26(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(26, 7)] = groebner_matrix[(10, 65)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 8)] = groebner_matrix[(10, 66)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 9)] = groebner_matrix[(10, 67)] / (groebner_matrix[(10, 64)])
        - groebner_matrix[(17, 53)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 10)] = groebner_matrix[(10, 68)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 14)] = -groebner_matrix[(17, 54)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 15)] = -groebner_matrix[(17, 55)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 16)] = -groebner_matrix[(17, 56)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 17)] = groebner_matrix[(10, 69)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 18)] = groebner_matrix[(10, 70)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 19)] = groebner_matrix[(10, 71)] / (groebner_matrix[(10, 64)])
        - groebner_matrix[(17, 57)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 20)] = groebner_matrix[(10, 72)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 23)] = -groebner_matrix[(17, 58)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 24)] = groebner_matrix[(10, 73)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 37)] = -groebner_matrix[(17, 65)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 38)] = -groebner_matrix[(17, 66)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 39)] = -groebner_matrix[(17, 67)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 42)] = -groebner_matrix[(17, 68)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 47)] = -groebner_matrix[(17, 69)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 48)] = -groebner_matrix[(17, 70)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 49)] = -groebner_matrix[(17, 71)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 50)] = groebner_matrix[(10, 74)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 51)] = groebner_matrix[(10, 75)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 52)] = groebner_matrix[(10, 76)] / (groebner_matrix[(10, 64)])
        - groebner_matrix[(17, 72)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 53)] = groebner_matrix[(10, 77)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 56)] = -groebner_matrix[(17, 73)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 57)] = groebner_matrix[(10, 78)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 62)] = -groebner_matrix[(17, 74)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 63)] = -groebner_matrix[(17, 75)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 64)] = -groebner_matrix[(17, 76)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 67)] = -groebner_matrix[(17, 77)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 71)] = -groebner_matrix[(17, 78)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(26, 72)] = groebner_matrix[(10, 79)] / (groebner_matrix[(10, 64)]);
    groebner_matrix[(26, 76)] = -groebner_matrix[(17, 79)] / (groebner_matrix[(17, 52)]);
}

pub fn s_polynomial27(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(27, 6)] = groebner_matrix[(9, 64)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(16, 52)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 7)] = groebner_matrix[(9, 65)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 8)] = groebner_matrix[(9, 66)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 9)] = groebner_matrix[(9, 67)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(16, 53)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 10)] = groebner_matrix[(9, 68)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 14)] = -groebner_matrix[(16, 54)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 15)] = -groebner_matrix[(16, 55)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 16)] = -groebner_matrix[(16, 56)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 17)] = groebner_matrix[(9, 69)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 18)] = groebner_matrix[(9, 70)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 19)] = groebner_matrix[(9, 71)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(16, 57)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 20)] = groebner_matrix[(9, 72)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 23)] = -groebner_matrix[(16, 58)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 24)] = groebner_matrix[(9, 73)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 37)] = -groebner_matrix[(16, 65)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 38)] = -groebner_matrix[(16, 66)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 39)] = -groebner_matrix[(16, 67)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 42)] = -groebner_matrix[(16, 68)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 47)] = -groebner_matrix[(16, 69)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 48)] = -groebner_matrix[(16, 70)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 49)] = -groebner_matrix[(16, 71)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 50)] = groebner_matrix[(9, 74)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 51)] = groebner_matrix[(9, 75)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 52)] = groebner_matrix[(9, 76)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(16, 72)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 53)] = groebner_matrix[(9, 77)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 56)] = -groebner_matrix[(16, 73)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 57)] = groebner_matrix[(9, 78)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 62)] = -groebner_matrix[(16, 74)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 63)] = -groebner_matrix[(16, 75)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 64)] = -groebner_matrix[(16, 76)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 67)] = -groebner_matrix[(16, 77)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 71)] = -groebner_matrix[(16, 78)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(27, 72)] = groebner_matrix[(9, 79)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(27, 76)] = -groebner_matrix[(16, 79)] / (groebner_matrix[(16, 51)]);
}

pub fn s_polynomial28(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(28, 6)] = groebner_matrix[(9, 64)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 7)] = groebner_matrix[(9, 65)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 8)] = groebner_matrix[(9, 66)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(17, 53)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 9)] = groebner_matrix[(9, 67)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 10)] = groebner_matrix[(9, 68)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 12)] = -groebner_matrix[(17, 54)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 13)] = -groebner_matrix[(17, 55)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 15)] = -groebner_matrix[(17, 56)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 17)] = groebner_matrix[(9, 69)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 18)] = groebner_matrix[(9, 70)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(17, 57)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 19)] = groebner_matrix[(9, 71)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 20)] = groebner_matrix[(9, 72)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 22)] = -groebner_matrix[(17, 58)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 24)] = groebner_matrix[(9, 73)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 35)] = -groebner_matrix[(17, 65)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 36)] = -groebner_matrix[(17, 66)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 38)] = -groebner_matrix[(17, 67)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 41)] = -groebner_matrix[(17, 68)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 45)] = -groebner_matrix[(17, 69)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 46)] = -groebner_matrix[(17, 70)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 48)] = -groebner_matrix[(17, 71)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 50)] = groebner_matrix[(9, 74)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 51)] = groebner_matrix[(9, 75)] / (groebner_matrix[(9, 63)])
        - groebner_matrix[(17, 72)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 52)] = groebner_matrix[(9, 76)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 53)] = groebner_matrix[(9, 77)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 55)] = -groebner_matrix[(17, 73)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 57)] = groebner_matrix[(9, 78)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 60)] = -groebner_matrix[(17, 74)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 61)] = -groebner_matrix[(17, 75)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 63)] = -groebner_matrix[(17, 76)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 66)] = -groebner_matrix[(17, 77)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 70)] = -groebner_matrix[(17, 78)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(28, 72)] = groebner_matrix[(9, 79)] / (groebner_matrix[(9, 63)]);
    groebner_matrix[(28, 75)] = -groebner_matrix[(17, 79)] / (groebner_matrix[(17, 52)]);
}

pub fn s_polynomial29(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(29, 5)] = groebner_matrix[(8, 63)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(15, 51)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 6)] = groebner_matrix[(8, 64)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(15, 52)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 7)] = groebner_matrix[(8, 65)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 8)] = groebner_matrix[(8, 66)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 9)] = groebner_matrix[(8, 67)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(15, 53)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 10)] = groebner_matrix[(8, 68)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 14)] = -groebner_matrix[(15, 54)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 15)] = -groebner_matrix[(15, 55)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 16)] = -groebner_matrix[(15, 56)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 17)] = groebner_matrix[(8, 69)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 18)] = groebner_matrix[(8, 70)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 19)] = groebner_matrix[(8, 71)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(15, 57)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 20)] = groebner_matrix[(8, 72)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 23)] = -groebner_matrix[(15, 58)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 24)] = groebner_matrix[(8, 73)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 37)] = -groebner_matrix[(15, 65)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 38)] = -groebner_matrix[(15, 66)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 39)] = -groebner_matrix[(15, 67)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 42)] = -groebner_matrix[(15, 68)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 47)] = -groebner_matrix[(15, 69)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 48)] = -groebner_matrix[(15, 70)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 49)] = -groebner_matrix[(15, 71)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 50)] = groebner_matrix[(8, 74)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 51)] = groebner_matrix[(8, 75)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 52)] = groebner_matrix[(8, 76)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(15, 72)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 53)] = groebner_matrix[(8, 77)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 56)] = -groebner_matrix[(15, 73)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 57)] = groebner_matrix[(8, 78)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 62)] = -groebner_matrix[(15, 74)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 63)] = -groebner_matrix[(15, 75)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 64)] = -groebner_matrix[(15, 76)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 67)] = -groebner_matrix[(15, 77)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 71)] = -groebner_matrix[(15, 78)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(29, 72)] = groebner_matrix[(8, 79)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(29, 76)] = -groebner_matrix[(15, 79)] / (groebner_matrix[(15, 50)]);
}

pub fn s_polynomial30(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(30, 5)] = groebner_matrix[(8, 63)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 6)] = groebner_matrix[(8, 64)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 7)] = groebner_matrix[(8, 65)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(17, 53)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 8)] = groebner_matrix[(8, 66)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 9)] = groebner_matrix[(8, 67)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 10)] = groebner_matrix[(8, 68)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 11)] = -groebner_matrix[(17, 54)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 12)] = -groebner_matrix[(17, 55)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 14)] = -groebner_matrix[(17, 56)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 17)] = groebner_matrix[(8, 69)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(17, 57)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 18)] = groebner_matrix[(8, 70)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 19)] = groebner_matrix[(8, 71)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 20)] = groebner_matrix[(8, 72)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 21)] = -groebner_matrix[(17, 58)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 24)] = groebner_matrix[(8, 73)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 34)] = -groebner_matrix[(17, 65)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 35)] = -groebner_matrix[(17, 66)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 37)] = -groebner_matrix[(17, 67)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 40)] = -groebner_matrix[(17, 68)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 44)] = -groebner_matrix[(17, 69)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 45)] = -groebner_matrix[(17, 70)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 47)] = -groebner_matrix[(17, 71)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 50)] = groebner_matrix[(8, 74)] / (groebner_matrix[(8, 62)])
        - groebner_matrix[(17, 72)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 51)] = groebner_matrix[(8, 75)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 52)] = groebner_matrix[(8, 76)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 53)] = groebner_matrix[(8, 77)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 54)] = -groebner_matrix[(17, 73)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 57)] = groebner_matrix[(8, 78)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 59)] = -groebner_matrix[(17, 74)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 60)] = -groebner_matrix[(17, 75)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 62)] = -groebner_matrix[(17, 76)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 65)] = -groebner_matrix[(17, 77)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 69)] = -groebner_matrix[(17, 78)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(30, 72)] = groebner_matrix[(8, 79)] / (groebner_matrix[(8, 62)]);
    groebner_matrix[(30, 74)] = -groebner_matrix[(17, 79)] / (groebner_matrix[(17, 52)]);
}

pub fn s_polynomial31(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(31, 1)] = groebner_matrix[(13, 43)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 4)] = -groebner_matrix[(14, 50)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 5)] = -groebner_matrix[(14, 51)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 6)] = -groebner_matrix[(14, 52)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 7)] = groebner_matrix[(13, 50)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 8)] = groebner_matrix[(13, 51)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 9)] = groebner_matrix[(13, 52)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(14, 53)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 10)] = groebner_matrix[(13, 53)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 14)] = -groebner_matrix[(14, 54)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 15)] = -groebner_matrix[(14, 55)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 16)] = -groebner_matrix[(14, 56)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 17)] = groebner_matrix[(13, 54)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 18)] = groebner_matrix[(13, 55)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 19)] = groebner_matrix[(13, 56)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(14, 57)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 20)] = groebner_matrix[(13, 57)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 23)] = -groebner_matrix[(14, 58)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 24)] = groebner_matrix[(13, 58)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 37)] = -groebner_matrix[(14, 65)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 38)] = -groebner_matrix[(14, 66)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 39)] = -groebner_matrix[(14, 67)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 40)] = groebner_matrix[(13, 65)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 41)] = groebner_matrix[(13, 66)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 42)] = groebner_matrix[(13, 67)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(14, 68)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 43)] = groebner_matrix[(13, 68)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 47)] = -groebner_matrix[(14, 69)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 48)] = -groebner_matrix[(14, 70)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 49)] = -groebner_matrix[(14, 71)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 50)] = groebner_matrix[(13, 69)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 51)] = groebner_matrix[(13, 70)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 52)] = groebner_matrix[(13, 71)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(14, 72)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 53)] = groebner_matrix[(13, 72)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 56)] = -groebner_matrix[(14, 73)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 57)] = groebner_matrix[(13, 73)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 62)] = -groebner_matrix[(14, 74)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 63)] = -groebner_matrix[(14, 75)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 64)] = -groebner_matrix[(14, 76)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 65)] = groebner_matrix[(13, 74)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 66)] = groebner_matrix[(13, 75)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 67)] = groebner_matrix[(13, 76)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(14, 77)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 68)] = groebner_matrix[(13, 77)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 71)] = -groebner_matrix[(14, 78)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 72)] = groebner_matrix[(13, 78)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(31, 76)] = -groebner_matrix[(14, 79)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(31, 77)] = groebner_matrix[(13, 79)] / (groebner_matrix[(13, 42)]);
}

pub fn s_polynomial32(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(32, 0)] = groebner_matrix[(12, 42)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 1)] = groebner_matrix[(12, 43)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 2)] = -groebner_matrix[(14, 50)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 3)] = -groebner_matrix[(14, 51)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 5)] = -groebner_matrix[(14, 52)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 7)] = groebner_matrix[(12, 50)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 8)] = groebner_matrix[(12, 51)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(14, 53)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 9)] = groebner_matrix[(12, 52)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 10)] = groebner_matrix[(12, 53)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 12)] = -groebner_matrix[(14, 54)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 13)] = -groebner_matrix[(14, 55)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 15)] = -groebner_matrix[(14, 56)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 17)] = groebner_matrix[(12, 54)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 18)] = groebner_matrix[(12, 55)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(14, 57)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 19)] = groebner_matrix[(12, 56)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 20)] = groebner_matrix[(12, 57)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 22)] = -groebner_matrix[(14, 58)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 24)] = groebner_matrix[(12, 58)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 35)] = -groebner_matrix[(14, 65)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 36)] = -groebner_matrix[(14, 66)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 38)] = -groebner_matrix[(14, 67)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 40)] = groebner_matrix[(12, 65)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 41)] = groebner_matrix[(12, 66)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(14, 68)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 42)] = groebner_matrix[(12, 67)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 43)] = groebner_matrix[(12, 68)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 45)] = -groebner_matrix[(14, 69)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 46)] = -groebner_matrix[(14, 70)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 48)] = -groebner_matrix[(14, 71)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 50)] = groebner_matrix[(12, 69)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 51)] = groebner_matrix[(12, 70)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(14, 72)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 52)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 53)] = groebner_matrix[(12, 72)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 55)] = -groebner_matrix[(14, 73)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 57)] = groebner_matrix[(12, 73)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 60)] = -groebner_matrix[(14, 74)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 61)] = -groebner_matrix[(14, 75)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 63)] = -groebner_matrix[(14, 76)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 65)] = groebner_matrix[(12, 74)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 66)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(14, 77)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 67)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 68)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 70)] = -groebner_matrix[(14, 78)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 72)] = groebner_matrix[(12, 78)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(32, 75)] = -groebner_matrix[(14, 79)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(32, 77)] = groebner_matrix[(12, 79)] / (groebner_matrix[(12, 41)]);
}

pub fn s_polynomial33(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(33, 54)] = groebner_matrix[(18, 54)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 69)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 55)] = groebner_matrix[(18, 55)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 70)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 56)] = groebner_matrix[(18, 56)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 71)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 57)] = groebner_matrix[(18, 57)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 72)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 58)] = groebner_matrix[(18, 58)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 73)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 65)] = groebner_matrix[(18, 65)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 66)] = groebner_matrix[(18, 66)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 67)] = groebner_matrix[(18, 67)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 68)] = groebner_matrix[(18, 68)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 69)] = groebner_matrix[(18, 69)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 74)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 70)] = groebner_matrix[(18, 70)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 75)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 71)] = groebner_matrix[(18, 71)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 76)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 72)] = groebner_matrix[(18, 72)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 77)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 73)] = groebner_matrix[(18, 73)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 78)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 74)] = groebner_matrix[(18, 74)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 75)] = groebner_matrix[(18, 75)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 76)] = groebner_matrix[(18, 76)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 77)] = groebner_matrix[(18, 77)] / (groebner_matrix[(18, 53)]);
    groebner_matrix[(33, 78)] = groebner_matrix[(18, 78)] / (groebner_matrix[(18, 53)])
        - groebner_matrix[(32, 79)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(33, 79)] = groebner_matrix[(18, 79)] / (groebner_matrix[(18, 53)]);
}

pub fn s_polynomial34(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(34, 53)] = groebner_matrix[(17, 53)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 68)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 54)] = groebner_matrix[(17, 54)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 69)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 55)] = groebner_matrix[(17, 55)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 70)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 56)] = groebner_matrix[(17, 56)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 71)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 57)] = groebner_matrix[(17, 57)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 72)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 58)] = groebner_matrix[(17, 58)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 73)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 65)] = groebner_matrix[(17, 65)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 66)] = groebner_matrix[(17, 66)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 67)] = groebner_matrix[(17, 67)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 68)] = groebner_matrix[(17, 68)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 69)] = groebner_matrix[(17, 69)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 74)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 70)] = groebner_matrix[(17, 70)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 75)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 71)] = groebner_matrix[(17, 71)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 76)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 72)] = groebner_matrix[(17, 72)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 77)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 73)] = groebner_matrix[(17, 73)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 78)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 74)] = groebner_matrix[(17, 74)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 75)] = groebner_matrix[(17, 75)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 76)] = groebner_matrix[(17, 76)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 77)] = groebner_matrix[(17, 77)] / (groebner_matrix[(17, 52)]);
    groebner_matrix[(34, 78)] = groebner_matrix[(17, 78)] / (groebner_matrix[(17, 52)])
        - groebner_matrix[(31, 79)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(34, 79)] = groebner_matrix[(17, 79)] / (groebner_matrix[(17, 52)]);
}

pub fn s_polynomial35(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(35, 52)] = groebner_matrix[(16, 52)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 67)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 53)] = groebner_matrix[(16, 53)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 68)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 54)] = groebner_matrix[(16, 54)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 69)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 55)] = groebner_matrix[(16, 55)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 70)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 56)] = groebner_matrix[(16, 56)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 71)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 57)] = groebner_matrix[(16, 57)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 72)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 58)] = groebner_matrix[(16, 58)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 73)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 65)] = groebner_matrix[(16, 65)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 66)] = groebner_matrix[(16, 66)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 67)] = groebner_matrix[(16, 67)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 68)] = groebner_matrix[(16, 68)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 69)] = groebner_matrix[(16, 69)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 74)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 70)] = groebner_matrix[(16, 70)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 75)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 71)] = groebner_matrix[(16, 71)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 76)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 72)] = groebner_matrix[(16, 72)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 77)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 73)] = groebner_matrix[(16, 73)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 78)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 74)] = groebner_matrix[(16, 74)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 75)] = groebner_matrix[(16, 75)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 76)] = groebner_matrix[(16, 76)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 77)] = groebner_matrix[(16, 77)] / (groebner_matrix[(16, 51)]);
    groebner_matrix[(35, 78)] = groebner_matrix[(16, 78)] / (groebner_matrix[(16, 51)])
        - groebner_matrix[(30, 79)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(35, 79)] = groebner_matrix[(16, 79)] / (groebner_matrix[(16, 51)]);
}

pub fn s_polynomial36(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(36, 51)] = groebner_matrix[(15, 51)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 66)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 52)] = groebner_matrix[(15, 52)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 67)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 53)] = groebner_matrix[(15, 53)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 68)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 54)] = groebner_matrix[(15, 54)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 69)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 55)] = groebner_matrix[(15, 55)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 70)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 56)] = groebner_matrix[(15, 56)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 71)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 57)] = groebner_matrix[(15, 57)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 72)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 58)] = groebner_matrix[(15, 58)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 73)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 65)] = groebner_matrix[(15, 65)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 66)] = groebner_matrix[(15, 66)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 67)] = groebner_matrix[(15, 67)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 68)] = groebner_matrix[(15, 68)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 69)] = groebner_matrix[(15, 69)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 74)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 70)] = groebner_matrix[(15, 70)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 75)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 71)] = groebner_matrix[(15, 71)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 76)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 72)] = groebner_matrix[(15, 72)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 77)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 73)] = groebner_matrix[(15, 73)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 78)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 74)] = groebner_matrix[(15, 74)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 75)] = groebner_matrix[(15, 75)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 76)] = groebner_matrix[(15, 76)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 77)] = groebner_matrix[(15, 77)] / (groebner_matrix[(15, 50)]);
    groebner_matrix[(36, 78)] = groebner_matrix[(15, 78)] / (groebner_matrix[(15, 50)])
        - groebner_matrix[(29, 79)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(36, 79)] = groebner_matrix[(15, 79)] / (groebner_matrix[(15, 50)]);
}

pub fn s_polynomial37(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(37, 50)] = groebner_matrix[(14, 50)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 69)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 51)] = groebner_matrix[(14, 51)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 70)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 52)] = groebner_matrix[(14, 52)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 71)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 53)] = groebner_matrix[(14, 53)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 72)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 54)] = groebner_matrix[(14, 54)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 55)] = groebner_matrix[(14, 55)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 56)] = groebner_matrix[(14, 56)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 57)] = groebner_matrix[(14, 57)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 73)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 58)] = groebner_matrix[(14, 58)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 65)] = groebner_matrix[(14, 65)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 74)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 66)] = groebner_matrix[(14, 66)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 75)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 67)] = groebner_matrix[(14, 67)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 76)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 68)] = groebner_matrix[(14, 68)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 77)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 69)] = groebner_matrix[(14, 69)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 70)] = groebner_matrix[(14, 70)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 71)] = groebner_matrix[(14, 71)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 72)] = groebner_matrix[(14, 72)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 78)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 73)] = groebner_matrix[(14, 73)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 74)] = groebner_matrix[(14, 74)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 75)] = groebner_matrix[(14, 75)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 76)] = groebner_matrix[(14, 76)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 77)] = groebner_matrix[(14, 77)] / (groebner_matrix[(14, 43)])
        - groebner_matrix[(32, 79)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(37, 78)] = groebner_matrix[(14, 78)] / (groebner_matrix[(14, 43)]);
    groebner_matrix[(37, 79)] = groebner_matrix[(14, 79)] / (groebner_matrix[(14, 43)]);
}

pub fn s_polynomial38(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(38, 43)] = groebner_matrix[(13, 43)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 68)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 50)] = groebner_matrix[(13, 50)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 69)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 51)] = groebner_matrix[(13, 51)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 70)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 52)] = groebner_matrix[(13, 52)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 71)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 53)] = groebner_matrix[(13, 53)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 72)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 54)] = groebner_matrix[(13, 54)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 55)] = groebner_matrix[(13, 55)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 56)] = groebner_matrix[(13, 56)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 57)] = groebner_matrix[(13, 57)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 73)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 58)] = groebner_matrix[(13, 58)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 65)] = groebner_matrix[(13, 65)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 74)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 66)] = groebner_matrix[(13, 66)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 75)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 67)] = groebner_matrix[(13, 67)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 76)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 68)] = groebner_matrix[(13, 68)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 77)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 69)] = groebner_matrix[(13, 69)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 70)] = groebner_matrix[(13, 70)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 71)] = groebner_matrix[(13, 71)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 72)] = groebner_matrix[(13, 72)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 78)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 73)] = groebner_matrix[(13, 73)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 74)] = groebner_matrix[(13, 74)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 75)] = groebner_matrix[(13, 75)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 76)] = groebner_matrix[(13, 76)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 77)] = groebner_matrix[(13, 77)] / (groebner_matrix[(13, 42)])
        - groebner_matrix[(31, 79)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(38, 78)] = groebner_matrix[(13, 78)] / (groebner_matrix[(13, 42)]);
    groebner_matrix[(38, 79)] = groebner_matrix[(13, 79)] / (groebner_matrix[(13, 42)]);
}

pub fn s_polynomial39(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(39, 43)] = groebner_matrix[(31, 68)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 47)] = -groebner_matrix[(32, 69)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 48)] = -groebner_matrix[(32, 70)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 49)] = -groebner_matrix[(32, 71)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 50)] = groebner_matrix[(31, 69)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 51)] = groebner_matrix[(31, 70)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 52)] = groebner_matrix[(31, 71)] / (groebner_matrix[(31, 67)])
        - groebner_matrix[(32, 72)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 53)] = groebner_matrix[(31, 72)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 56)] = -groebner_matrix[(32, 73)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 57)] = groebner_matrix[(31, 73)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 62)] = -groebner_matrix[(32, 74)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 63)] = -groebner_matrix[(32, 75)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 64)] = -groebner_matrix[(32, 76)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 65)] = groebner_matrix[(31, 74)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 66)] = groebner_matrix[(31, 75)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 67)] = groebner_matrix[(31, 76)] / (groebner_matrix[(31, 67)])
        - groebner_matrix[(32, 77)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 68)] = groebner_matrix[(31, 77)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 71)] = -groebner_matrix[(32, 78)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 72)] = groebner_matrix[(31, 78)] / (groebner_matrix[(31, 67)]);
    groebner_matrix[(39, 76)] = -groebner_matrix[(32, 79)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(39, 77)] = groebner_matrix[(31, 79)] / (groebner_matrix[(31, 67)]);
}

pub fn s_polynomial40(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(40, 42)] = groebner_matrix[(12, 42)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 67)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 43)] = groebner_matrix[(12, 43)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 68)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 50)] = groebner_matrix[(12, 50)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 69)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 51)] = groebner_matrix[(12, 51)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 70)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 52)] = groebner_matrix[(12, 52)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 71)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 53)] = groebner_matrix[(12, 53)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 72)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 54)] = groebner_matrix[(12, 54)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 55)] = groebner_matrix[(12, 55)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 56)] = groebner_matrix[(12, 56)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 57)] = groebner_matrix[(12, 57)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 73)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 58)] = groebner_matrix[(12, 58)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 65)] = groebner_matrix[(12, 65)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 74)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 66)] = groebner_matrix[(12, 66)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 75)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 67)] = groebner_matrix[(12, 67)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 76)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 68)] = groebner_matrix[(12, 68)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 77)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 69)] = groebner_matrix[(12, 69)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 70)] = groebner_matrix[(12, 70)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 71)] = groebner_matrix[(12, 71)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 72)] = groebner_matrix[(12, 72)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 78)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 73)] = groebner_matrix[(12, 73)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 74)] = groebner_matrix[(12, 74)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 75)] = groebner_matrix[(12, 75)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 76)] = groebner_matrix[(12, 76)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 77)] = groebner_matrix[(12, 77)] / (groebner_matrix[(12, 41)])
        - groebner_matrix[(30, 79)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(40, 78)] = groebner_matrix[(12, 78)] / (groebner_matrix[(12, 41)]);
    groebner_matrix[(40, 79)] = groebner_matrix[(12, 79)] / (groebner_matrix[(12, 41)]);
}

pub fn s_polynomial41(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(41, 42)] = groebner_matrix[(30, 67)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 43)] = groebner_matrix[(30, 68)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 45)] = -groebner_matrix[(32, 69)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 46)] = -groebner_matrix[(32, 70)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 48)] = -groebner_matrix[(32, 71)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 50)] = groebner_matrix[(30, 69)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 51)] = groebner_matrix[(30, 70)] / (groebner_matrix[(30, 66)])
        - groebner_matrix[(32, 72)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 52)] = groebner_matrix[(30, 71)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 53)] = groebner_matrix[(30, 72)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 55)] = -groebner_matrix[(32, 73)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 57)] = groebner_matrix[(30, 73)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 60)] = -groebner_matrix[(32, 74)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 61)] = -groebner_matrix[(32, 75)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 63)] = -groebner_matrix[(32, 76)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 65)] = groebner_matrix[(30, 74)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 66)] = groebner_matrix[(30, 75)] / (groebner_matrix[(30, 66)])
        - groebner_matrix[(32, 77)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 67)] = groebner_matrix[(30, 76)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 68)] = groebner_matrix[(30, 77)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 70)] = -groebner_matrix[(32, 78)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 72)] = groebner_matrix[(30, 78)] / (groebner_matrix[(30, 66)]);
    groebner_matrix[(41, 75)] = -groebner_matrix[(32, 79)] / (groebner_matrix[(32, 68)]);
    groebner_matrix[(41, 77)] = groebner_matrix[(30, 79)] / (groebner_matrix[(30, 66)]);
}

pub fn s_polynomial42(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(42, 41)] = groebner_matrix[(11, 41)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 66)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 42)] = groebner_matrix[(11, 42)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 67)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 43)] = groebner_matrix[(11, 43)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 68)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 50)] = groebner_matrix[(11, 50)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 69)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 51)] = groebner_matrix[(11, 51)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 70)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 52)] = groebner_matrix[(11, 52)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 71)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 53)] = groebner_matrix[(11, 53)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 72)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 54)] = groebner_matrix[(11, 54)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 55)] = groebner_matrix[(11, 55)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 56)] = groebner_matrix[(11, 56)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 57)] = groebner_matrix[(11, 57)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 73)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 58)] = groebner_matrix[(11, 58)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 65)] = groebner_matrix[(11, 65)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 74)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 66)] = groebner_matrix[(11, 66)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 75)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 67)] = groebner_matrix[(11, 67)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 76)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 68)] = groebner_matrix[(11, 68)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 77)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 69)] = groebner_matrix[(11, 69)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 70)] = groebner_matrix[(11, 70)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 71)] = groebner_matrix[(11, 71)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 72)] = groebner_matrix[(11, 72)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 78)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 73)] = groebner_matrix[(11, 73)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 74)] = groebner_matrix[(11, 74)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 75)] = groebner_matrix[(11, 75)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 76)] = groebner_matrix[(11, 76)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 77)] = groebner_matrix[(11, 77)] / (groebner_matrix[(11, 40)])
        - groebner_matrix[(29, 79)] / (groebner_matrix[(29, 65)]);
    groebner_matrix[(42, 78)] = groebner_matrix[(11, 78)] / (groebner_matrix[(11, 40)]);
    groebner_matrix[(42, 79)] = groebner_matrix[(11, 79)] / (groebner_matrix[(11, 40)]);
}

pub fn s_polynomial43(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_matrix[(43, 50)] = -groebner_matrix[(28, 65)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 51)] = -groebner_matrix[(28, 66)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 52)] = -groebner_matrix[(28, 67)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 53)] = -groebner_matrix[(28, 68)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 54)] = groebner_matrix[(23, 54)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 69)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 55)] = groebner_matrix[(23, 55)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 70)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 56)] = groebner_matrix[(23, 56)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 71)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 57)] = groebner_matrix[(23, 57)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 72)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 58)] = groebner_matrix[(23, 58)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 73)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 65)] = groebner_matrix[(23, 65)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 66)] = groebner_matrix[(23, 66)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 67)] = groebner_matrix[(23, 67)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 68)] = groebner_matrix[(23, 68)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 69)] = groebner_matrix[(23, 69)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 74)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 70)] = groebner_matrix[(23, 70)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 75)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 71)] = groebner_matrix[(23, 71)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 76)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 72)] = groebner_matrix[(23, 72)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 77)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 73)] = groebner_matrix[(23, 73)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 78)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 74)] = groebner_matrix[(23, 74)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 75)] = groebner_matrix[(23, 75)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 76)] = groebner_matrix[(23, 76)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 77)] = groebner_matrix[(23, 77)] / (groebner_matrix[(23, 25)]);
    groebner_matrix[(43, 78)] = groebner_matrix[(23, 78)] / (groebner_matrix[(23, 25)])
        - groebner_matrix[(28, 79)] / (groebner_matrix[(28, 58)]);
    groebner_matrix[(43, 79)] = groebner_matrix[(23, 79)] / (groebner_matrix[(23, 25)]);
}
