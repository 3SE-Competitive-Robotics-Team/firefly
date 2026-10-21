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

pub fn groebner_row6_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 60)] / groebner_matrix[(6, 60)];
    groebner_matrix[(target_row, 60)] = 0.0;
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(6, 61)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(6, 62)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(6, 63)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(6, 64)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(6, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(6, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(6, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(6, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(6, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(6, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(6, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(6, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(6, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(6, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(6, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(6, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(6, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(6, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(6, 79)];
}

pub fn groebner_row7_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 61)] / groebner_matrix[(7, 61)];
    groebner_matrix[(target_row, 61)] = 0.0;
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(7, 62)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(7, 63)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(7, 64)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(7, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(7, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(7, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(7, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(7, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(7, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(7, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(7, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(7, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(7, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(7, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(7, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(7, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(7, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(7, 79)];
}

pub fn groebner_row8_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 62)] / groebner_matrix[(8, 62)];
    groebner_matrix[(target_row, 62)] = 0.0;
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(8, 63)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(8, 64)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(8, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(8, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(8, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(8, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(8, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(8, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(8, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(8, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(8, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(8, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(8, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(8, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(8, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(8, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(8, 79)];
}

pub fn groebner_row9_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 63)] / groebner_matrix[(9, 63)];
    groebner_matrix[(target_row, 63)] = 0.0;
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(9, 64)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(9, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(9, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(9, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(9, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(9, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(9, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(9, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(9, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(9, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(9, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(9, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(9, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(9, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(9, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(9, 79)];
}

pub fn groebner_row10_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 33)] / groebner_matrix[(10, 64)];
    groebner_matrix[(target_row, 33)] = 0.0;
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(10, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(10, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(10, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(10, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(10, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(10, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(10, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(10, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(10, 79)];
}

pub fn groebner_row6_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 35)] / groebner_matrix[(6, 60)];
    groebner_matrix[(target_row, 35)] = 0.0;
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(6, 61)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(6, 62)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(6, 63)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(6, 64)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(6, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(6, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(6, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(6, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(6, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(6, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(6, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(6, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(6, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(6, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(6, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(6, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(6, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(6, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(6, 79)];
}

pub fn groebner_row7_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 36)] / groebner_matrix[(7, 61)];
    groebner_matrix[(target_row, 36)] = 0.0;
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(7, 62)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(7, 63)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(7, 64)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(7, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(7, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(7, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(7, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(7, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(7, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(7, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(7, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(7, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(7, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(7, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(7, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(7, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(7, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(7, 79)];
}

pub fn groebner_row8_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 37)] / groebner_matrix[(8, 62)];
    groebner_matrix[(target_row, 37)] = 0.0;
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(8, 63)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(8, 64)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(8, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(8, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(8, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(8, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(8, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(8, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(8, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(8, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(8, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(8, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(8, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(8, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(8, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(8, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(8, 79)];
}

pub fn groebner_row9_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 38)] / groebner_matrix[(9, 63)];
    groebner_matrix[(target_row, 38)] = 0.0;
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(9, 64)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(9, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(9, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(9, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(9, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(9, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(9, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(9, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(9, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(9, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(9, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(9, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(9, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(9, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(9, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(9, 79)];
}

pub fn groebner_row10_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 39)] / groebner_matrix[(10, 64)];
    groebner_matrix[(target_row, 39)] = 0.0;
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(10, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(10, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(10, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(10, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(10, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(10, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(10, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(10, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(10, 79)];
}

pub fn groebner_row6_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 45)] / groebner_matrix[(6, 60)];
    groebner_matrix[(target_row, 45)] = 0.0;
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(6, 61)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(6, 62)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(6, 63)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(6, 64)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(6, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(6, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(6, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(6, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(6, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(6, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(6, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(6, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(6, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(6, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(6, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(6, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(6, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(6, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(6, 79)];
}

pub fn groebner_row7_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 46)] / groebner_matrix[(7, 61)];
    groebner_matrix[(target_row, 46)] = 0.0;
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(7, 62)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(7, 63)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(7, 64)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(7, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(7, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(7, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(7, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(7, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(7, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(7, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(7, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(7, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(7, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(7, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(7, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(7, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(7, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(7, 79)];
}

pub fn groebner_row8_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 47)] / groebner_matrix[(8, 62)];
    groebner_matrix[(target_row, 47)] = 0.0;
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(8, 63)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(8, 64)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(8, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(8, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(8, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(8, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(8, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(8, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(8, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(8, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(8, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(8, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(8, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(8, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(8, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(8, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(8, 79)];
}

pub fn groebner_row9_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 48)] / groebner_matrix[(9, 63)];
    groebner_matrix[(target_row, 48)] = 0.0;
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(9, 64)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(9, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(9, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(9, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(9, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(9, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(9, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(9, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(9, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(9, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(9, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(9, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(9, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(9, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(9, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(9, 79)];
}

pub fn groebner_row10_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 49)] / groebner_matrix[(10, 64)];
    groebner_matrix[(target_row, 49)] = 0.0;
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(10, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(10, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(10, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(10, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(10, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(10, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(10, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(10, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(10, 79)];
}

pub fn groebner_row10_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 64)] / groebner_matrix[(10, 64)];
    groebner_matrix[(target_row, 64)] = 0.0;
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(10, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(10, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(10, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(10, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(10, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(10, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(10, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(10, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(10, 79)];
}

pub fn groebner_row9_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 32)] / groebner_matrix[(9, 63)];
    groebner_matrix[(target_row, 32)] = 0.0;
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(9, 64)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(9, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(9, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(9, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(9, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(9, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(9, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(9, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(9, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(9, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(9, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(9, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(9, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(9, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(9, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(9, 79)];
}

pub fn groebner_row5_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 34)] / groebner_matrix[(5, 59)];
    groebner_matrix[(target_row, 34)] = 0.0;
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(5, 60)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(5, 61)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(5, 62)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(5, 63)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(5, 64)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(5, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(5, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(5, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(5, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(5, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(5, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(5, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(5, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(5, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(5, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(5, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(5, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(5, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(5, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(5, 79)];
}

pub fn groebner_row11_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 40)] / groebner_matrix[(11, 40)];
    groebner_matrix[(target_row, 40)] = 0.0;
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(11, 41)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(11, 42)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(11, 43)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(11, 50)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(11, 51)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(11, 52)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(11, 53)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(11, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(11, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(11, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(11, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(11, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(11, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(11, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(11, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(11, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(11, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(11, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(11, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(11, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(11, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(11, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(11, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(11, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(11, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(11, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(11, 79)];
}

pub fn groebner_row5_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 44)] / groebner_matrix[(5, 59)];
    groebner_matrix[(target_row, 44)] = 0.0;
    groebner_matrix[(target_row, 45)] -= factor * groebner_matrix[(5, 60)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(5, 61)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(5, 62)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(5, 63)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(5, 64)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(5, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(5, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(5, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(5, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(5, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(5, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(5, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(5, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(5, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(5, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(5, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(5, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(5, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(5, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(5, 79)];
}

pub fn groebner_row5_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 59)] / groebner_matrix[(5, 59)];
    groebner_matrix[(target_row, 59)] = 0.0;
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(5, 60)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(5, 61)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(5, 62)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(5, 63)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(5, 64)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(5, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(5, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(5, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(5, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(5, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(5, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(5, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(5, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(5, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(5, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(5, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(5, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(5, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(5, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(5, 79)];
}

pub fn groebner_row8_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 31)] / groebner_matrix[(8, 62)];
    groebner_matrix[(target_row, 31)] = 0.0;
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(8, 63)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(8, 64)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(8, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(8, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(8, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(8, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(8, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(8, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(8, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(8, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(8, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(8, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(8, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(8, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(8, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(8, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(8, 79)];
}

pub fn groebner_row12_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 41)] / groebner_matrix[(12, 41)];
    groebner_matrix[(target_row, 41)] = 0.0;
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(12, 42)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(12, 43)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(12, 50)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(12, 51)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(12, 52)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(12, 53)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(12, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(12, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(12, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(12, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(12, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(12, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(12, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(12, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(12, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(12, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(12, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(12, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(12, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(12, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(12, 79)];
}

pub fn groebner_row7_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 30)] / groebner_matrix[(7, 61)];
    groebner_matrix[(target_row, 30)] = 0.0;
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(7, 62)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(7, 63)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(7, 64)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(7, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(7, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(7, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(7, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(7, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(7, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(7, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(7, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(7, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(7, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(7, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(7, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(7, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(7, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(7, 79)];
}

pub fn groebner_row13_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 42)] / groebner_matrix[(13, 42)];
    groebner_matrix[(target_row, 42)] = 0.0;
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(13, 43)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(13, 50)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(13, 51)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(13, 52)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(13, 53)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(13, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(13, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(13, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(13, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(13, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(13, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(13, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(13, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(13, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(13, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(13, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(13, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(13, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(13, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(13, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(13, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(13, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(13, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(13, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(13, 79)];
}

pub fn groebner_row14_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 43)] / groebner_matrix[(14, 43)];
    groebner_matrix[(target_row, 43)] = 0.0;
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(14, 50)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(14, 51)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(14, 52)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(14, 53)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(14, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(14, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(14, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(14, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(14, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(14, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(14, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(14, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(14, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(14, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(14, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(14, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(14, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(14, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(14, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(14, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(14, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(14, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(14, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(14, 79)];
}

pub fn groebner_row6_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 29)] / groebner_matrix[(6, 60)];
    groebner_matrix[(target_row, 29)] = 0.0;
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(6, 61)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(6, 62)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(6, 63)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(6, 64)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(6, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(6, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(6, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(6, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(6, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(6, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(6, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(6, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(6, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(6, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(6, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(6, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(6, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(6, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(6, 79)];
}

pub fn groebner_row15_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 50)] / groebner_matrix[(15, 50)];
    groebner_matrix[(target_row, 50)] = 0.0;
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(15, 51)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(15, 52)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(15, 53)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(15, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(15, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(15, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(15, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(15, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(15, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(15, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(15, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(15, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(15, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(15, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(15, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(15, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(15, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(15, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(15, 79)];
}

pub fn groebner_row7_00010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 27)] / groebner_matrix[(7, 61)];
    groebner_matrix[(target_row, 27)] = 0.0;
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(7, 62)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(7, 63)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(7, 64)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(7, 65)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(7, 66)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(7, 67)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(7, 68)];
    groebner_matrix[(target_row, 45)] -= factor * groebner_matrix[(7, 69)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(7, 70)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(7, 71)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(7, 72)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(7, 73)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(7, 74)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(7, 75)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(7, 76)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(7, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(7, 78)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(7, 79)];
}

pub fn groebner_row5_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 28)] / groebner_matrix[(5, 59)];
    groebner_matrix[(target_row, 28)] = 0.0;
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(5, 60)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(5, 61)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(5, 62)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(5, 63)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(5, 64)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(5, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(5, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(5, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(5, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(5, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(5, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(5, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(5, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(5, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(5, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(5, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(5, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(5, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(5, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(5, 79)];
}

pub fn groebner_row16_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 51)] / groebner_matrix[(16, 51)];
    groebner_matrix[(target_row, 51)] = 0.0;
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(16, 52)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(16, 53)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(16, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(16, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(16, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(16, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(16, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(16, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(16, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(16, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(16, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(16, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(16, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(16, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(16, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(16, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(16, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(16, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(16, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(16, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(16, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(16, 79)];
}

pub fn groebner_row6_00010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 26)] / groebner_matrix[(6, 60)];
    groebner_matrix[(target_row, 26)] = 0.0;
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(6, 61)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(6, 62)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(6, 63)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(6, 64)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(6, 65)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(6, 66)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(6, 67)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(6, 68)];
    groebner_matrix[(target_row, 45)] -= factor * groebner_matrix[(6, 69)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(6, 70)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(6, 71)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(6, 72)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(6, 73)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(6, 74)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(6, 75)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(6, 76)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(6, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(6, 78)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(6, 79)];
}

pub fn groebner_row17_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 52)] / groebner_matrix[(17, 52)];
    groebner_matrix[(target_row, 52)] = 0.0;
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(17, 53)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(17, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(17, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(17, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(17, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(17, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(17, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(17, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(17, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(17, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(17, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(17, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(17, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(17, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(17, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(17, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(17, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(17, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(17, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(17, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(17, 79)];
}

pub fn groebner_row15_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 17)] / groebner_matrix[(15, 50)];
    groebner_matrix[(target_row, 17)] = 0.0;
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(15, 51)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(15, 52)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(15, 53)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(15, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(15, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(15, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(15, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(15, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(15, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(15, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(15, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(15, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(15, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(15, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(15, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(15, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(15, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(15, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(15, 79)];
}

pub fn groebner_row16_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 18)] / groebner_matrix[(16, 51)];
    groebner_matrix[(target_row, 18)] = 0.0;
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(16, 52)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(16, 53)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(16, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(16, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(16, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(16, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(16, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(16, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(16, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(16, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(16, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(16, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(16, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(16, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(16, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(16, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(16, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(16, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(16, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(16, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(16, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(16, 79)];
}

pub fn groebner_row17_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 19)] / groebner_matrix[(17, 52)];
    groebner_matrix[(target_row, 19)] = 0.0;
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(17, 53)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(17, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(17, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(17, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(17, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(17, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(17, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(17, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(17, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(17, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(17, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(17, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(17, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(17, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(17, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(17, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(17, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(17, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(17, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(17, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(17, 79)];
}

pub fn groebner_row18_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 20)] / groebner_matrix[(18, 53)];
    groebner_matrix[(target_row, 20)] = 0.0;
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(18, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(18, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(18, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(18, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(18, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(18, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(18, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(18, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(18, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(18, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(18, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(18, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(18, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(18, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(18, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(18, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(18, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(18, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(18, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(18, 79)];
}

pub fn groebner_row18_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 53)] / groebner_matrix[(18, 53)];
    groebner_matrix[(target_row, 53)] = 0.0;
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(18, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(18, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(18, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(18, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(18, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(18, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(18, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(18, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(18, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(18, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(18, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(18, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(18, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(18, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(18, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(18, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(18, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(18, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(18, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(18, 79)];
}

pub fn groebner_row14_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 10)] / groebner_matrix[(14, 43)];
    groebner_matrix[(target_row, 10)] = 0.0;
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(14, 50)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(14, 51)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(14, 52)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(14, 53)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(14, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(14, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(14, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(14, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(14, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(14, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(14, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(14, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(14, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(14, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(14, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(14, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(14, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(14, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(14, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(14, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(14, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(14, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(14, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(14, 79)];
}

pub fn groebner_row19_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 21)] / groebner_matrix[(19, 21)];
    groebner_matrix[(target_row, 21)] = 0.0;
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(19, 22)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(19, 23)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(19, 24)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(19, 25)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(19, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(19, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(19, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(19, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(19, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(19, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(19, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(19, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(19, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(19, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(19, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(19, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(19, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(19, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(19, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(19, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(19, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(19, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(19, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(19, 79)];
}

pub fn groebner_row8_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 14)] / groebner_matrix[(8, 62)];
    groebner_matrix[(target_row, 14)] = 0.0;
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(8, 63)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(8, 64)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(8, 65)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(8, 66)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(8, 67)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(8, 68)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(8, 69)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(8, 70)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(8, 71)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(8, 72)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(8, 73)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(8, 74)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(8, 75)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(8, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(8, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(8, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(8, 79)];
}

pub fn groebner_row9_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 15)] / groebner_matrix[(9, 63)];
    groebner_matrix[(target_row, 15)] = 0.0;
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(9, 64)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(9, 65)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(9, 66)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(9, 67)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(9, 68)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(9, 69)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(9, 70)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(9, 71)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(9, 72)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(9, 73)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(9, 74)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(9, 75)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(9, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(9, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(9, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(9, 79)];
}

pub fn groebner_row10_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 16)] / groebner_matrix[(10, 64)];
    groebner_matrix[(target_row, 16)] = 0.0;
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(10, 65)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(10, 67)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(10, 68)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(10, 69)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(10, 72)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(10, 73)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(10, 74)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(10, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(10, 79)];
}

pub fn groebner_row20_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 22)] / groebner_matrix[(20, 22)];
    groebner_matrix[(target_row, 22)] = 0.0;
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(20, 23)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(20, 24)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(20, 25)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(20, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(20, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(20, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(20, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(20, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(20, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(20, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(20, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(20, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(20, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(20, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(20, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(20, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(20, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(20, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(20, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(20, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(20, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(20, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(20, 79)];
}

pub fn groebner_row13_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 9)] / groebner_matrix[(13, 42)];
    groebner_matrix[(target_row, 9)] = 0.0;
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(13, 43)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(13, 50)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(13, 51)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(13, 52)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(13, 53)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(13, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(13, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(13, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(13, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(13, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(13, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(13, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(13, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(13, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(13, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(13, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(13, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(13, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(13, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(13, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(13, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(13, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(13, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(13, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(13, 79)];
}

pub fn groebner_row21_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 23)] / groebner_matrix[(21, 23)];
    groebner_matrix[(target_row, 23)] = 0.0;
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(21, 24)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(21, 25)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(21, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(21, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(21, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(21, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(21, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(21, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(21, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(21, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(21, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(21, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(21, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(21, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(21, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(21, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(21, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(21, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(21, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(21, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(21, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(21, 79)];
}

pub fn groebner_row6_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 12)] / groebner_matrix[(6, 60)];
    groebner_matrix[(target_row, 12)] = 0.0;
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(6, 61)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(6, 62)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(6, 63)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(6, 64)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(6, 65)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(6, 66)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(6, 67)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(6, 68)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(6, 69)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(6, 70)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(6, 71)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(6, 72)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(6, 73)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(6, 74)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(6, 75)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(6, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(6, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(6, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(6, 79)];
}

pub fn groebner_row7_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 13)] / groebner_matrix[(7, 61)];
    groebner_matrix[(target_row, 13)] = 0.0;
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(7, 62)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(7, 63)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(7, 64)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(7, 65)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(7, 66)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(7, 67)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(7, 68)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(7, 69)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(7, 70)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(7, 71)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(7, 72)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(7, 73)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(7, 74)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(7, 75)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(7, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(7, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(7, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(7, 79)];
}

pub fn groebner_row22_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 24)] / groebner_matrix[(22, 24)];
    groebner_matrix[(target_row, 24)] = 0.0;
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(22, 25)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(22, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(22, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(22, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(22, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(22, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(22, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(22, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(22, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(22, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(22, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(22, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(22, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(22, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(22, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(22, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(22, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(22, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(22, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(22, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(22, 79)];
}

pub fn groebner_row12_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 8)] / groebner_matrix[(12, 41)];
    groebner_matrix[(target_row, 8)] = 0.0;
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(12, 42)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(12, 43)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(12, 50)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(12, 51)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(12, 52)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(12, 53)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(12, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(12, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(12, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(12, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(12, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(12, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(12, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(12, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(12, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(12, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(12, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(12, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(12, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(12, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(12, 79)];
}

pub fn groebner_row23_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 25)] / groebner_matrix[(23, 25)];
    groebner_matrix[(target_row, 25)] = 0.0;
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(23, 54)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(23, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(23, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(23, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(23, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(23, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(23, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(23, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(23, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(23, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(23, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(23, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(23, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(23, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(23, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(23, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(23, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(23, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(23, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(23, 79)];
}

pub fn groebner_row5_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 11)] / groebner_matrix[(5, 59)];
    groebner_matrix[(target_row, 11)] = 0.0;
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(5, 60)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(5, 61)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(5, 62)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(5, 63)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(5, 64)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(5, 65)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(5, 66)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(5, 67)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(5, 68)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(5, 69)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(5, 70)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(5, 71)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(5, 72)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(5, 73)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(5, 74)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(5, 75)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(5, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(5, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(5, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(5, 79)];
}

pub fn groebner_row24_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 21)] / groebner_matrix[(24, 54)];
    groebner_matrix[(target_row, 21)] = 0.0;
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(24, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(24, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(24, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(24, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(24, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(24, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(24, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(24, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(24, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(24, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(24, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(24, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(24, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(24, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(24, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(24, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(24, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(24, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(24, 79)];
}

pub fn groebner_row24_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 54)] / groebner_matrix[(24, 54)];
    groebner_matrix[(target_row, 54)] = 0.0;
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(24, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(24, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(24, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(24, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(24, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(24, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(24, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(24, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(24, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(24, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(24, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(24, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(24, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(24, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(24, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(24, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(24, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(24, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(24, 79)];
}

pub fn groebner_row11_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 7)] / groebner_matrix[(11, 40)];
    groebner_matrix[(target_row, 7)] = 0.0;
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(11, 41)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(11, 42)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(11, 43)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(11, 50)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(11, 51)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(11, 52)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(11, 53)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(11, 54)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(11, 55)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(11, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(11, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(11, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(11, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(11, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(11, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(11, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(11, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(11, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(11, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(11, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(11, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(11, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(11, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(11, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(11, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(11, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(11, 79)];
}

pub fn groebner_row25_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 22)] / groebner_matrix[(25, 55)];
    groebner_matrix[(target_row, 22)] = 0.0;
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(25, 56)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(25, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(25, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(25, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(25, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(25, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(25, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(25, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(25, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(25, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(25, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(25, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(25, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(25, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(25, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(25, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(25, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(25, 79)];
}

pub fn groebner_row25_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 55)] / groebner_matrix[(25, 55)];
    groebner_matrix[(target_row, 55)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(25, 56)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(25, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(25, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(25, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(25, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(25, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(25, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(25, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(25, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(25, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(25, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(25, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(25, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(25, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(25, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(25, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(25, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(25, 79)];
}

pub fn groebner_row10_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 6)] / groebner_matrix[(10, 64)];
    groebner_matrix[(target_row, 6)] = 0.0;
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(10, 65)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(10, 67)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(10, 68)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(10, 69)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(10, 72)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(10, 73)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(10, 74)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(10, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(10, 79)];
}

pub fn groebner_row26_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 23)] / groebner_matrix[(26, 56)];
    groebner_matrix[(target_row, 23)] = 0.0;
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(26, 57)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(26, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(26, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(26, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(26, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(26, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(26, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(26, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(26, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(26, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(26, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(26, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(26, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(26, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(26, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(26, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(26, 79)];
}

pub fn groebner_row26_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 56)] / groebner_matrix[(26, 56)];
    groebner_matrix[(target_row, 56)] = 0.0;
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(26, 57)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(26, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(26, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(26, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(26, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(26, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(26, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(26, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(26, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(26, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(26, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(26, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(26, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(26, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(26, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(26, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(26, 79)];
}

pub fn groebner_row27_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 24)] / groebner_matrix[(27, 57)];
    groebner_matrix[(target_row, 24)] = 0.0;
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(27, 58)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(27, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(27, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(27, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(27, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(27, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(27, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(27, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(27, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(27, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(27, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(27, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(27, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(27, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(27, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(27, 79)];
}

pub fn groebner_row27_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 57)] / groebner_matrix[(27, 57)];
    groebner_matrix[(target_row, 57)] = 0.0;
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(27, 58)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(27, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(27, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(27, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(27, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(27, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(27, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(27, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(27, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(27, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(27, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(27, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(27, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(27, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(27, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(27, 79)];
}

pub fn groebner_row9_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 5)] / groebner_matrix[(9, 63)];
    groebner_matrix[(target_row, 5)] = 0.0;
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(9, 64)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(9, 65)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(9, 66)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(9, 67)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(9, 68)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(9, 69)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(9, 70)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(9, 71)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(9, 72)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(9, 73)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(9, 74)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(9, 75)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(9, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(9, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(9, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(9, 79)];
}

pub fn groebner_row28_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 25)] / groebner_matrix[(28, 58)];
    groebner_matrix[(target_row, 25)] = 0.0;
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(28, 65)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(28, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(28, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(28, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(28, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(28, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(28, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(28, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(28, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(28, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(28, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(28, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(28, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(28, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(28, 79)];
}

pub fn groebner_row28_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 58)] / groebner_matrix[(28, 58)];
    groebner_matrix[(target_row, 58)] = 0.0;
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(28, 65)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(28, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(28, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(28, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(28, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(28, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(28, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(28, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(28, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(28, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(28, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(28, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(28, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(28, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(28, 79)];
}

pub fn groebner_row18_00001_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 7)] / groebner_matrix[(18, 53)];
    groebner_matrix[(target_row, 7)] = 0.0;
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(18, 54)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(18, 55)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(18, 56)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(18, 57)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(18, 58)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(18, 65)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(18, 66)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(18, 67)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(18, 68)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(18, 69)];
    groebner_matrix[(target_row, 45)] -= factor * groebner_matrix[(18, 70)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(18, 71)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(18, 72)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(18, 73)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(18, 74)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(18, 75)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(18, 76)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(18, 77)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(18, 78)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(18, 79)];
}

pub fn groebner_row24_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 17)] / groebner_matrix[(24, 54)];
    groebner_matrix[(target_row, 17)] = 0.0;
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(24, 55)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(24, 56)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(24, 57)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(24, 58)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(24, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(24, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(24, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(24, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(24, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(24, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(24, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(24, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(24, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(24, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(24, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(24, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(24, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(24, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(24, 79)];
}

pub fn groebner_row29_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 40)] / groebner_matrix[(29, 65)];
    groebner_matrix[(target_row, 40)] = 0.0;
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(29, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(29, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(29, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(29, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(29, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(29, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(29, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(29, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(29, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(29, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(29, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(29, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(29, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(29, 79)];
}

pub fn groebner_row29_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 50)] / groebner_matrix[(29, 65)];
    groebner_matrix[(target_row, 50)] = 0.0;
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(29, 66)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(29, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(29, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(29, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(29, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(29, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(29, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(29, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(29, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(29, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(29, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(29, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(29, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(29, 79)];
}

pub fn groebner_row29_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 65)] / groebner_matrix[(29, 65)];
    groebner_matrix[(target_row, 65)] = 0.0;
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(29, 66)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(29, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(29, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(29, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(29, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(29, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(29, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(29, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(29, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(29, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(29, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(29, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(29, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(29, 79)];
}

pub fn groebner_row14_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(14, 43)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(14, 50)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(14, 51)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(14, 52)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(14, 53)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(14, 54)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(14, 55)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(14, 56)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(14, 57)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(14, 58)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(14, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(14, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(14, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(14, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(14, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(14, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(14, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(14, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(14, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(14, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(14, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(14, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(14, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(14, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(14, 79)];
}

pub fn groebner_row8_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 4)] / groebner_matrix[(8, 62)];
    groebner_matrix[(target_row, 4)] = 0.0;
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(8, 63)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(8, 64)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(8, 65)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(8, 66)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(8, 67)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(8, 68)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(8, 69)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(8, 70)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(8, 71)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(8, 72)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(8, 73)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(8, 74)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(8, 75)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(8, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(8, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(8, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(8, 79)];
}

pub fn groebner_row18_00010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 8)] / groebner_matrix[(18, 53)];
    groebner_matrix[(target_row, 8)] = 0.0;
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(18, 54)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(18, 55)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(18, 56)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(18, 57)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(18, 58)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(18, 65)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(18, 66)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(18, 67)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(18, 68)];
    groebner_matrix[(target_row, 45)] -= factor * groebner_matrix[(18, 69)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(18, 70)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(18, 71)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(18, 72)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(18, 73)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(18, 74)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(18, 75)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(18, 76)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(18, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(18, 78)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(18, 79)];
}

pub fn groebner_row25_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 18)] / groebner_matrix[(25, 55)];
    groebner_matrix[(target_row, 18)] = 0.0;
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(25, 56)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(25, 57)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(25, 58)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(25, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(25, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(25, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(25, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(25, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(25, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(25, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(25, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(25, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(25, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(25, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(25, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(25, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(25, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(25, 79)];
}

pub fn groebner_row30_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 41)] / groebner_matrix[(30, 66)];
    groebner_matrix[(target_row, 41)] = 0.0;
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(30, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(30, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(30, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(30, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(30, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(30, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(30, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(30, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(30, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(30, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(30, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(30, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(30, 79)];
}

pub fn groebner_row30_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 51)] / groebner_matrix[(30, 66)];
    groebner_matrix[(target_row, 51)] = 0.0;
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(30, 67)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(30, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(30, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(30, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(30, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(30, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(30, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(30, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(30, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(30, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(30, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(30, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(30, 79)];
}

pub fn groebner_row30_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 66)] / groebner_matrix[(30, 66)];
    groebner_matrix[(target_row, 66)] = 0.0;
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(30, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(30, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(30, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(30, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(30, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(30, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(30, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(30, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(30, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(30, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(30, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(30, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(30, 79)];
}

pub fn groebner_row14_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 0)] / groebner_matrix[(14, 43)];
    groebner_matrix[(target_row, 0)] = 0.0;
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(14, 50)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(14, 51)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(14, 52)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(14, 53)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(14, 54)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(14, 55)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(14, 56)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(14, 57)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(14, 58)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(14, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(14, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(14, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(14, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(14, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(14, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(14, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(14, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(14, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(14, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(14, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(14, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(14, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(14, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(14, 79)];
}

pub fn groebner_row6_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 2)] / groebner_matrix[(6, 60)];
    groebner_matrix[(target_row, 2)] = 0.0;
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(6, 61)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(6, 62)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(6, 63)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(6, 64)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(6, 65)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(6, 66)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(6, 67)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(6, 68)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(6, 69)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(6, 70)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(6, 71)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(6, 72)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(6, 73)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(6, 74)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(6, 75)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(6, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(6, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(6, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(6, 79)];
}

pub fn groebner_row7_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 3)] / groebner_matrix[(7, 61)];
    groebner_matrix[(target_row, 3)] = 0.0;
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(7, 62)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(7, 63)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(7, 64)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(7, 65)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(7, 66)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(7, 67)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(7, 68)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(7, 69)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(7, 70)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(7, 71)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(7, 72)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(7, 73)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(7, 74)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(7, 75)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(7, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(7, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(7, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(7, 79)];
}

pub fn groebner_row18_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 9)] / groebner_matrix[(18, 53)];
    groebner_matrix[(target_row, 9)] = 0.0;
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(18, 54)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(18, 55)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(18, 56)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(18, 57)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(18, 58)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(18, 65)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(18, 66)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(18, 67)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(18, 68)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(18, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(18, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(18, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(18, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(18, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(18, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(18, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(18, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(18, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(18, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(18, 79)];
}

pub fn groebner_row26_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 19)] / groebner_matrix[(26, 56)];
    groebner_matrix[(target_row, 19)] = 0.0;
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(26, 57)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(26, 58)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(26, 65)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(26, 66)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(26, 67)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(26, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(26, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(26, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(26, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(26, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(26, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(26, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(26, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(26, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(26, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(26, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(26, 79)];
}

pub fn groebner_row31_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 42)] / groebner_matrix[(31, 67)];
    groebner_matrix[(target_row, 42)] = 0.0;
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(31, 68)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(31, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(31, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(31, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(31, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(31, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(31, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(31, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(31, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(31, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(31, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(31, 79)];
}

pub fn groebner_row31_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 52)] / groebner_matrix[(31, 67)];
    groebner_matrix[(target_row, 52)] = 0.0;
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(31, 68)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(31, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(31, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(31, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(31, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(31, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(31, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(31, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(31, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(31, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(31, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(31, 79)];
}

pub fn groebner_row31_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 67)] / groebner_matrix[(31, 67)];
    groebner_matrix[(target_row, 67)] = 0.0;
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(31, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(31, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(31, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(31, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(31, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(31, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(31, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(31, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(31, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(31, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(31, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(31, 79)];
}

pub fn groebner_row32_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 68)] / groebner_matrix[(32, 68)];
    groebner_matrix[(target_row, 68)] = 0.0;
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(32, 69)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(32, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(32, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(32, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(32, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(32, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(32, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(32, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(32, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(32, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(32, 79)];
}

pub fn groebner_row32_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 53)] / groebner_matrix[(32, 68)];
    groebner_matrix[(target_row, 53)] = 0.0;
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(32, 69)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(32, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(32, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(32, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(32, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(32, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(32, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(32, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(32, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(32, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(32, 79)];
}

pub fn groebner_row33_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 54)] / groebner_matrix[(33, 69)];
    groebner_matrix[(target_row, 54)] = 0.0;
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(33, 70)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(33, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(33, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(33, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(33, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(33, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(33, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(33, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(33, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(33, 79)];
}

pub fn groebner_row33_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 69)] / groebner_matrix[(33, 69)];
    groebner_matrix[(target_row, 69)] = 0.0;
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(33, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(33, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(33, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(33, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(33, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(33, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(33, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(33, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(33, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(33, 79)];
}

pub fn groebner_row34_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 55)] / groebner_matrix[(34, 70)];
    groebner_matrix[(target_row, 55)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(34, 71)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(34, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(34, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(34, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(34, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(34, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(34, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(34, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(34, 79)];
}

pub fn groebner_row34_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 70)] / groebner_matrix[(34, 70)];
    groebner_matrix[(target_row, 70)] = 0.0;
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(34, 71)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(34, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(34, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(34, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(34, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(34, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(34, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(34, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(34, 79)];
}

pub fn groebner_row35_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 56)] / groebner_matrix[(35, 71)];
    groebner_matrix[(target_row, 56)] = 0.0;
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(35, 72)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(35, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(35, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(35, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(35, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(35, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(35, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(35, 79)];
}

pub fn groebner_row35_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 71)] / groebner_matrix[(35, 71)];
    groebner_matrix[(target_row, 71)] = 0.0;
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(35, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(35, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(35, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(35, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(35, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(35, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(35, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(35, 79)];
}

pub fn groebner_row36_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 57)] / groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 57)] = 0.0;
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(36, 79)];
}

pub fn groebner_row36_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 72)] / groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 72)] = 0.0;
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(36, 79)];
}

pub fn groebner_row32_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 43)] / groebner_matrix[(32, 68)];
    groebner_matrix[(target_row, 43)] = 0.0;
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(32, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(32, 70)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(32, 71)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(32, 72)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(32, 73)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(32, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(32, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(32, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(32, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(32, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(32, 79)];
}

pub fn groebner_row37_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 58)] / groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 58)] = 0.0;
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(37, 79)];
}

pub fn groebner_row37_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 73)] / groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 73)] = 0.0;
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(37, 79)];
}

pub fn groebner_row35_00001_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 47)] / groebner_matrix[(35, 71)];
    groebner_matrix[(target_row, 47)] = 0.0;
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(35, 72)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(35, 73)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(35, 74)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(35, 75)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(35, 76)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(35, 77)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(35, 78)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(35, 79)];
}

pub fn groebner_row36_00001_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 50)] / groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 50)] = 0.0;
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(36, 79)];
}

pub fn groebner_row37_00001_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 54)] / groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 54)] = 0.0;
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(37, 79)];
}

pub fn groebner_row38_00001_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 59)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 59)] = 0.0;
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row38_00010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 60)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 60)] = 0.0;
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row38_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 62)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 62)] = 0.0;
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row38_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 65)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 65)] = 0.0;
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row38_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 69)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 69)] = 0.0;
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row38_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 74)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 74)] = 0.0;
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row36_00010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 51)] / groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 51)] = 0.0;
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(36, 79)];
}

pub fn groebner_row37_00010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 55)] / groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 55)] = 0.0;
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(37, 79)];
}

pub fn groebner_row39_00010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 61)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 61)] = 0.0;
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row39_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 63)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 63)] = 0.0;
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row39_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 66)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 66)] = 0.0;
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row39_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 70)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 70)] = 0.0;
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row39_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 75)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 75)] = 0.0;
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row32_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 42)] / groebner_matrix[(32, 68)];
    groebner_matrix[(target_row, 42)] = 0.0;
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(32, 69)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(32, 70)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(32, 71)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(32, 72)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(32, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(32, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(32, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(32, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(32, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(32, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(32, 79)];
}

pub fn groebner_row38_10010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 45)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 45)] = 0.0;
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row39_10010_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 46)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 46)] = 0.0;
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row38_10100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 47)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 47)] = 0.0;
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row39_10100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 48)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 48)] = 0.0;
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row40_10100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 49)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 49)] = 0.0;
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row36_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 52)] / groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 52)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(36, 79)];
}

pub fn groebner_row37_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 56)] / groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 56)] = 0.0;
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(37, 79)];
}

pub fn groebner_row40_00100_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 64)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 64)] = 0.0;
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row40_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 67)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 67)] = 0.0;
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row40_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 71)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 71)] = 0.0;
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row40_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 76)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 76)] = 0.0;
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row39_02000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 41)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 41)] = 0.0;
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row40_02000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 42)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 42)] = 0.0;
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row41_02000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 43)] / groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 43)] = 0.0;
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(41, 79)];
}

pub fn groebner_row38_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 50)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 50)] = 0.0;
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row39_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 51)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 51)] = 0.0;
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row40_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 52)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 52)] = 0.0;
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row41_11000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 53)] / groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 53)] = 0.0;
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(41, 79)];
}

pub fn groebner_row37_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 57)] / groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 57)] = 0.0;
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(37, 79)];
}

pub fn groebner_row41_01000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 68)] / groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 68)] = 0.0;
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(41, 79)];
}

pub fn groebner_row41_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 72)] / groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 72)] = 0.0;
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(41, 79)];
}

pub fn groebner_row41_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 77)] / groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 77)] = 0.0;
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(41, 79)];
}

pub fn groebner_row38_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 54)] / groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 54)] = 0.0;
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(38, 79)];
}

pub fn groebner_row39_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 55)] / groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 55)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(39, 79)];
}

pub fn groebner_row40_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 56)] / groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 56)] = 0.0;
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(40, 79)];
}

pub fn groebner_row41_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 57)] / groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 57)] = 0.0;
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(41, 79)];
}

pub fn groebner_row42_20000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 58)] / groebner_matrix[(42, 78)];
    groebner_matrix[(target_row, 58)] = 0.0;
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(42, 79)];
}

pub fn groebner_row42_10000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 73)] / groebner_matrix[(42, 78)];
    groebner_matrix[(target_row, 73)] = 0.0;
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(42, 79)];
}

pub fn groebner_row42_00000_f(groebner_matrix: &mut SMatrix<f64, 44, 80>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 78)] / groebner_matrix[(42, 78)];
    groebner_matrix[(target_row, 78)] = 0.0;
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(42, 79)];
}
