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

pub fn groebner_row9_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 21)] / groebner_matrix[(9, 21)];
    groebner_matrix[(target_row, 21)] = 0.0;
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(9, 33)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(9, 53)];
    groebner_matrix[(target_row, 66)] -= factor * groebner_matrix[(9, 66)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(9, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(9, 71)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(9, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(9, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(9, 77)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(9, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(9, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(9, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(9, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(9, 84)];
}

pub fn groebner_row10_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 20)] / groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 20)] = 0.0;
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(10, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(10, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(10, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(10, 84)];
}

pub fn groebner_row10_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 66)] / groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 66)] = 0.0;
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(10, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(10, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(10, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(10, 84)];
}

pub fn groebner_row11_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 29)] / groebner_matrix[(11, 29)];
    groebner_matrix[(target_row, 29)] = 0.0;
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(11, 32)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(11, 43)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(11, 47)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(11, 52)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(11, 64)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(11, 68)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(11, 70)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(11, 71)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(11, 73)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(11, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(11, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(11, 77)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(11, 79)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(11, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(11, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(11, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(11, 84)];
}

pub fn groebner_row10_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 21)] / groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 21)] = 0.0;
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(10, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(10, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(10, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(10, 84)];
}

pub fn groebner_row12_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 29)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 29)] = 0.0;
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row12_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 30)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 30)] = 0.0;
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row12_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 70)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 70)] = 0.0;
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row14_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 33)] / groebner_matrix[(14, 33)];
    groebner_matrix[(target_row, 33)] = 0.0;
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(14, 44)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(14, 48)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(14, 53)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(14, 65)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(14, 69)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(14, 71)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(14, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(14, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(14, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(14, 77)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(14, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(14, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(14, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(14, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(14, 84)];
}

pub fn groebner_row15_010000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 35)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 35)] = 0.0;
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row10_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 45)] / groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 45)] = 0.0;
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(10, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(10, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(10, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(10, 84)];
}

pub fn groebner_row12_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 49)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 49)] = 0.0;
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row15_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 50)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 50)] = 0.0;
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row15_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 71)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 71)] = 0.0;
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row15_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 33)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 33)] = 0.0;
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row17_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 44)] / groebner_matrix[(17, 44)];
    groebner_matrix[(target_row, 44)] = 0.0;
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(17, 48)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(17, 53)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(17, 65)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(17, 69)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(17, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(17, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(17, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(17, 77)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(17, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(17, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(17, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(17, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(17, 84)];
}

pub fn groebner_row12_010000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 34)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 34)] = 0.0;
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row16_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 54)] / groebner_matrix[(16, 54)];
    groebner_matrix[(target_row, 54)] = 0.0;
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(16, 55)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(16, 56)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(16, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(16, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(16, 77)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(16, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(16, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(16, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(16, 84)];
}

pub fn groebner_row12_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 28)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 28)] = 0.0;
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row15_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 31)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 31)] = 0.0;
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row19_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 11)] / groebner_matrix[(19, 55)];
    groebner_matrix[(target_row, 11)] = 0.0;
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(19, 56)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(19, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(19, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(19, 77)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(19, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(19, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(19, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(19, 84)];
}

pub fn groebner_row19_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 55)] / groebner_matrix[(19, 55)];
    groebner_matrix[(target_row, 55)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(19, 56)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(19, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(19, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(19, 77)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(19, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(19, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(19, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(19, 84)];
}

pub fn groebner_row19_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 10)] / groebner_matrix[(19, 55)];
    groebner_matrix[(target_row, 10)] = 0.0;
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(19, 56)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(19, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(19, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(19, 77)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(19, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(19, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(19, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(19, 84)];
}

pub fn groebner_row18_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 43)] / groebner_matrix[(18, 43)];
    groebner_matrix[(target_row, 43)] = 0.0;
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(18, 47)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(18, 52)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(18, 64)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(18, 68)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(18, 73)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(18, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(18, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(18, 77)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(18, 79)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(18, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(18, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(18, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(18, 84)];
}

pub fn groebner_row19_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 9)] / groebner_matrix[(19, 55)];
    groebner_matrix[(target_row, 9)] = 0.0;
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(19, 56)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(19, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(19, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(19, 77)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(19, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(19, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(19, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(19, 84)];
}

pub fn groebner_row20_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 42)] / groebner_matrix[(20, 42)];
    groebner_matrix[(target_row, 42)] = 0.0;
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(20, 46)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(20, 51)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(20, 63)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(20, 67)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(20, 72)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(20, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(20, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(20, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(20, 78)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(20, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(20, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(20, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(20, 84)];
}

pub fn groebner_row15_100100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 5)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 5)] = 0.0;
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row16_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 8)] / groebner_matrix[(16, 54)];
    groebner_matrix[(target_row, 8)] = 0.0;
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(16, 55)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(16, 56)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(16, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(16, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(16, 77)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(16, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(16, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(16, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(16, 84)];
}

pub fn groebner_row21_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 14)] / groebner_matrix[(21, 14)];
    groebner_matrix[(target_row, 14)] = 0.0;
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(21, 48)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(21, 53)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(21, 56)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(21, 65)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(21, 69)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(21, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(21, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(21, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(21, 77)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(21, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(21, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(21, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(21, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(21, 84)];
}

pub fn groebner_row15_100010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 4)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 4)] = 0.0;
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row16_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 7)] / groebner_matrix[(16, 54)];
    groebner_matrix[(target_row, 7)] = 0.0;
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(16, 55)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(16, 56)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(16, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(16, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(16, 77)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(16, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(16, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(16, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(16, 84)];
}

pub fn groebner_row22_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 13)] / groebner_matrix[(22, 13)];
    groebner_matrix[(target_row, 13)] = 0.0;
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(22, 47)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(22, 52)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(22, 56)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(22, 64)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(22, 68)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(22, 73)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(22, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(22, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(22, 77)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(22, 79)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(22, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(22, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(22, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(22, 84)];
}

pub fn groebner_row15_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 32)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 32)] = 0.0;
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row15_100001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 3)] / groebner_matrix[(15, 71)];
    groebner_matrix[(target_row, 3)] = 0.0;
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(15, 75)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(15, 76)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(15, 77)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(15, 81)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(15, 82)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(15, 83)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(15, 84)];
}

pub fn groebner_row16_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 6)] / groebner_matrix[(16, 54)];
    groebner_matrix[(target_row, 6)] = 0.0;
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(16, 55)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(16, 56)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(16, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(16, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(16, 77)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(16, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(16, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(16, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(16, 84)];
}

pub fn groebner_row23_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 12)] / groebner_matrix[(23, 12)];
    groebner_matrix[(target_row, 12)] = 0.0;
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(23, 46)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(23, 51)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(23, 56)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(23, 63)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(23, 67)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(23, 72)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(23, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(23, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(23, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(23, 78)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(23, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(23, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(23, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(23, 84)];
}

pub fn groebner_row12_100100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 2)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 2)] = 0.0;
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row24_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 48)] / groebner_matrix[(24, 48)];
    groebner_matrix[(target_row, 48)] = 0.0;
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(24, 53)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(24, 56)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(24, 65)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(24, 69)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(24, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(24, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(24, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(24, 77)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(24, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(24, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(24, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(24, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(24, 84)];
}

pub fn groebner_row12_100010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row25_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 47)] / groebner_matrix[(25, 47)];
    groebner_matrix[(target_row, 47)] = 0.0;
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(25, 52)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(25, 56)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(25, 64)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(25, 68)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(25, 73)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(25, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(25, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(25, 77)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(25, 79)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(25, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(25, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(25, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(25, 84)];
}

pub fn groebner_row12_100001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 0)] / groebner_matrix[(12, 70)];
    groebner_matrix[(target_row, 0)] = 0.0;
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(12, 71)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(12, 75)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(12, 76)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(12, 77)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(12, 81)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(12, 82)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(12, 83)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(12, 84)];
}

pub fn groebner_row10_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 19)] / groebner_matrix[(10, 66)];
    groebner_matrix[(target_row, 19)] = 0.0;
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(10, 70)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(10, 71)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(10, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(10, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(10, 77)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(10, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(10, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(10, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(10, 84)];
}

pub fn groebner_row26_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 46)] / groebner_matrix[(26, 46)];
    groebner_matrix[(target_row, 46)] = 0.0;
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(26, 51)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(26, 56)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(26, 63)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(26, 67)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(26, 72)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(26, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(26, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(26, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(26, 78)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(26, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(26, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(26, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(26, 84)];
}

pub fn groebner_row28_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 52)] / groebner_matrix[(28, 52)];
    groebner_matrix[(target_row, 52)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(28, 56)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(28, 64)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(28, 68)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(28, 73)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(28, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(28, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(28, 77)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(28, 79)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(28, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(28, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(28, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(28, 84)];
}

pub fn groebner_row27_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 53)] / groebner_matrix[(27, 53)];
    groebner_matrix[(target_row, 53)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(27, 56)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(27, 65)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(27, 69)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(27, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(27, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(27, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(27, 77)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(27, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(27, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(27, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(27, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(27, 84)];
}

pub fn groebner_row29_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 51)] / groebner_matrix[(29, 51)];
    groebner_matrix[(target_row, 51)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(29, 56)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(29, 63)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(29, 67)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(29, 72)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(29, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(29, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(29, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(29, 78)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(29, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(29, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(29, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(29, 84)];
}

pub fn groebner_row31_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 42)] / groebner_matrix[(31, 63)];
    groebner_matrix[(target_row, 42)] = 0.0;
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(31, 65)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(31, 67)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(31, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(31, 72)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(31, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(31, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(31, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(31, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(31, 78)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(31, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(31, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(31, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(31, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(31, 84)];
}

pub fn groebner_row30_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 43)] / groebner_matrix[(30, 64)];
    groebner_matrix[(target_row, 43)] = 0.0;
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(30, 65)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(30, 68)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(30, 69)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(30, 73)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(30, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(30, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(30, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(30, 77)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(30, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(30, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(30, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(30, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(30, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(30, 84)];
}

pub fn groebner_row31_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 63)] / groebner_matrix[(31, 63)];
    groebner_matrix[(target_row, 63)] = 0.0;
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(31, 65)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(31, 67)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(31, 69)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(31, 72)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(31, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(31, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(31, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(31, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(31, 78)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(31, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(31, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(31, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(31, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(31, 84)];
}

pub fn groebner_row30_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 64)] / groebner_matrix[(30, 64)];
    groebner_matrix[(target_row, 64)] = 0.0;
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(30, 65)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(30, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(30, 69)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(30, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(30, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(30, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(30, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(30, 77)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(30, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(30, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(30, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(30, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(30, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(30, 84)];
}

pub fn groebner_row32_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 65)] / groebner_matrix[(32, 65)];
    groebner_matrix[(target_row, 65)] = 0.0;
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(32, 67)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(32, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(32, 69)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(32, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(32, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(32, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(32, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(32, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(32, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(32, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(32, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(32, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(32, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(32, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(32, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(32, 84)];
}

pub fn groebner_row32_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 44)] / groebner_matrix[(32, 65)];
    groebner_matrix[(target_row, 44)] = 0.0;
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(32, 67)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(32, 68)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(32, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(32, 72)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(32, 73)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(32, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(32, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(32, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(32, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(32, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(32, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(32, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(32, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(32, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(32, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(32, 84)];
}

pub fn groebner_row33_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 46)] / groebner_matrix[(33, 67)];
    groebner_matrix[(target_row, 46)] = 0.0;
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(33, 68)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(33, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(33, 72)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(33, 73)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(33, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(33, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(33, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(33, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(33, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(33, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(33, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(33, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(33, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(33, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(33, 84)];
}

pub fn groebner_row33_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 67)] / groebner_matrix[(33, 67)];
    groebner_matrix[(target_row, 67)] = 0.0;
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(33, 68)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(33, 69)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(33, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(33, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(33, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(33, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(33, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(33, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(33, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(33, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(33, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(33, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(33, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(33, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(33, 84)];
}

pub fn groebner_row34_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 47)] / groebner_matrix[(34, 68)];
    groebner_matrix[(target_row, 47)] = 0.0;
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(34, 69)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(34, 72)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(34, 73)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(34, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(34, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(34, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(34, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(34, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(34, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(34, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(34, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(34, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(34, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(34, 84)];
}

pub fn groebner_row34_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 68)] / groebner_matrix[(34, 68)];
    groebner_matrix[(target_row, 68)] = 0.0;
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(34, 69)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(34, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(34, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(34, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(34, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(34, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(34, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(34, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(34, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(34, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(34, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(34, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(34, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(34, 84)];
}

pub fn groebner_row35_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 48)] / groebner_matrix[(35, 69)];
    groebner_matrix[(target_row, 48)] = 0.0;
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(35, 72)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(35, 73)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(35, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(35, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(35, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(35, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(35, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(35, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(35, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(35, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(35, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(35, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(35, 84)];
}

pub fn groebner_row35_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 69)] / groebner_matrix[(35, 69)];
    groebner_matrix[(target_row, 69)] = 0.0;
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(35, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(35, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(35, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(35, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(35, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(35, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(35, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(35, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(35, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(35, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(35, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(35, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(35, 84)];
}

pub fn groebner_row37_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 51)] / groebner_matrix[(37, 72)];
    groebner_matrix[(target_row, 51)] = 0.0;
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(37, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(37, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(37, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(37, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(37, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(37, 84)];
}

pub fn groebner_row36_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 56)] / groebner_matrix[(36, 56)];
    groebner_matrix[(target_row, 56)] = 0.0;
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(36, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(36, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(36, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(36, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(36, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(36, 84)];
}

pub fn groebner_row37_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 72)] / groebner_matrix[(37, 72)];
    groebner_matrix[(target_row, 72)] = 0.0;
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(37, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(37, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(37, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(37, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(37, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(37, 84)];
}

pub fn groebner_row38_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 52)] / groebner_matrix[(38, 73)];
    groebner_matrix[(target_row, 52)] = 0.0;
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(38, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(38, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(38, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(38, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(38, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(38, 84)];
}

pub fn groebner_row38_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 73)] / groebner_matrix[(38, 73)];
    groebner_matrix[(target_row, 73)] = 0.0;
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(38, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(38, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(38, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(38, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(38, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(38, 84)];
}

pub fn groebner_row39_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 53)] / groebner_matrix[(39, 74)];
    groebner_matrix[(target_row, 53)] = 0.0;
    groebner_matrix[(target_row, 54)] -= factor * groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(39, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(39, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(39, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(39, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(39, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(39, 84)];
}

pub fn groebner_row39_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 74)] / groebner_matrix[(39, 74)];
    groebner_matrix[(target_row, 74)] = 0.0;
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(39, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(39, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(39, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(39, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(39, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(39, 84)];
}

pub fn groebner_row40_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 54)] / groebner_matrix[(40, 75)];
    groebner_matrix[(target_row, 54)] = 0.0;
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(40, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(40, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(40, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(40, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(40, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(40, 84)];
}

pub fn groebner_row40_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 75)] / groebner_matrix[(40, 75)];
    groebner_matrix[(target_row, 75)] = 0.0;
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(40, 76)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(40, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(40, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(40, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(40, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(40, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(40, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(40, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(40, 84)];
}

pub fn groebner_row32_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 18)] / groebner_matrix[(32, 65)];
    groebner_matrix[(target_row, 18)] = 0.0;
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(32, 67)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(32, 68)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(32, 69)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(32, 72)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(32, 73)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(32, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(32, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(32, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(32, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(32, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(32, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(32, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(32, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(32, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(32, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(32, 84)];
}

pub fn groebner_row33_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 23)] / groebner_matrix[(33, 67)];
    groebner_matrix[(target_row, 23)] = 0.0;
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(33, 68)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(33, 69)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(33, 72)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(33, 73)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(33, 74)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(33, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(33, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(33, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(33, 78)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(33, 79)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(33, 80)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(33, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(33, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(33, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(33, 84)];
}

pub fn groebner_row34_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 24)] / groebner_matrix[(34, 68)];
    groebner_matrix[(target_row, 24)] = 0.0;
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(34, 69)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(34, 72)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(34, 73)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(34, 74)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(34, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(34, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(34, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(34, 78)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(34, 79)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(34, 80)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(34, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(34, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(34, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(34, 84)];
}

pub fn groebner_row33_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 25)] / groebner_matrix[(33, 67)];
    groebner_matrix[(target_row, 25)] = 0.0;
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(33, 68)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(33, 69)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(33, 72)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(33, 73)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(33, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(33, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(33, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(33, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(33, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(33, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(33, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(33, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(33, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(33, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(33, 84)];
}

pub fn groebner_row34_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 26)] / groebner_matrix[(34, 68)];
    groebner_matrix[(target_row, 26)] = 0.0;
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(34, 69)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(34, 72)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(34, 73)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(34, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(34, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(34, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(34, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(34, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(34, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(34, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(34, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(34, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(34, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(34, 84)];
}

pub fn groebner_row35_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 27)] / groebner_matrix[(35, 69)];
    groebner_matrix[(target_row, 27)] = 0.0;
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(35, 72)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(35, 73)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(35, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(35, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(35, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(35, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(35, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(35, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(35, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(35, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(35, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(35, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(35, 84)];
}

pub fn groebner_row37_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 37)] / groebner_matrix[(37, 72)];
    groebner_matrix[(target_row, 37)] = 0.0;
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(37, 79)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(37, 80)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(37, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(37, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(37, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(37, 84)];
}

pub fn groebner_row38_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 38)] / groebner_matrix[(38, 73)];
    groebner_matrix[(target_row, 38)] = 0.0;
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(38, 79)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(38, 80)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(38, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(38, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(38, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(38, 84)];
}

pub fn groebner_row37_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 39)] / groebner_matrix[(37, 72)];
    groebner_matrix[(target_row, 39)] = 0.0;
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(37, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(37, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(37, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(37, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(37, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(37, 84)];
}

pub fn groebner_row38_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 40)] / groebner_matrix[(38, 73)];
    groebner_matrix[(target_row, 40)] = 0.0;
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(38, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(38, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(38, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(38, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(38, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(38, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(38, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(38, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(38, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(38, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(38, 84)];
}

pub fn groebner_row39_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 41)] / groebner_matrix[(39, 74)];
    groebner_matrix[(target_row, 41)] = 0.0;
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(39, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(39, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(39, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(39, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(39, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(39, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(39, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(39, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(39, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(39, 84)];
}

pub fn groebner_row41_100000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 55)] / groebner_matrix[(41, 76)];
    groebner_matrix[(target_row, 55)] = 0.0;
    groebner_matrix[(target_row, 56)] -= factor * groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(41, 79)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(41, 80)];
    groebner_matrix[(target_row, 75)] -= factor * groebner_matrix[(41, 81)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(41, 82)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(41, 83)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(41, 84)];
}

pub fn groebner_row41_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 76)] / groebner_matrix[(41, 76)];
    groebner_matrix[(target_row, 76)] = 0.0;
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(41, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(41, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(41, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(41, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(41, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(41, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(41, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(41, 84)];
}

pub fn groebner_row33_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 22)] / groebner_matrix[(33, 67)];
    groebner_matrix[(target_row, 22)] = 0.0;
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(33, 68)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(33, 69)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(33, 72)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(33, 73)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(33, 74)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(33, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(33, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(33, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(33, 78)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(33, 79)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(33, 80)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(33, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(33, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(33, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(33, 84)];
}

pub fn groebner_row37_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 36)] / groebner_matrix[(37, 72)];
    groebner_matrix[(target_row, 36)] = 0.0;
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(37, 73)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(37, 74)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(37, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(37, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(37, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(37, 78)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(37, 79)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(37, 80)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(37, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(37, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(37, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(37, 84)];
}

pub fn groebner_row42_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 58)] / groebner_matrix[(42, 58)];
    groebner_matrix[(target_row, 58)] = 0.0;
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(42, 59)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(42, 60)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(42, 61)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(42, 62)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(42, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(42, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(42, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(42, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(42, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(42, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(42, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(42, 84)];
}

pub fn groebner_row31_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 16)] / groebner_matrix[(31, 63)];
    groebner_matrix[(target_row, 16)] = 0.0;
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(31, 65)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(31, 67)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(31, 69)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(31, 72)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(31, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(31, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(31, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(31, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(31, 78)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(31, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(31, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(31, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(31, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(31, 84)];
}

pub fn groebner_row30_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 17)] / groebner_matrix[(30, 64)];
    groebner_matrix[(target_row, 17)] = 0.0;
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(30, 65)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(30, 68)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(30, 69)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(30, 73)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(30, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(30, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(30, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(30, 77)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(30, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(30, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(30, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(30, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(30, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(30, 84)];
}

pub fn groebner_row36_000100_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 14)] / groebner_matrix[(36, 56)];
    groebner_matrix[(target_row, 14)] = 0.0;
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 41)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 44)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 53)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(36, 79)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(36, 80)];
    groebner_matrix[(target_row, 65)] -= factor * groebner_matrix[(36, 81)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(36, 82)];
    groebner_matrix[(target_row, 74)] -= factor * groebner_matrix[(36, 83)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(36, 84)];
}

pub fn groebner_row36_010000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 15)] / groebner_matrix[(36, 56)];
    groebner_matrix[(target_row, 15)] = 0.0;
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 48)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 49)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 50)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 55)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(36, 79)];
    groebner_matrix[(target_row, 69)] -= factor * groebner_matrix[(36, 80)];
    groebner_matrix[(target_row, 70)] -= factor * groebner_matrix[(36, 81)];
    groebner_matrix[(target_row, 71)] -= factor * groebner_matrix[(36, 82)];
    groebner_matrix[(target_row, 76)] -= factor * groebner_matrix[(36, 83)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(36, 84)];
}

pub fn groebner_row44_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 60)] / groebner_matrix[(44, 60)];
    groebner_matrix[(target_row, 60)] = 0.0;
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(44, 61)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(44, 62)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(44, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(44, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(44, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(44, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(44, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(44, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(44, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(44, 84)];
}

pub fn groebner_row36_000010_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 13)] / groebner_matrix[(36, 56)];
    groebner_matrix[(target_row, 13)] = 0.0;
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 38)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 40)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 43)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 47)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 52)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(36, 79)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(36, 80)];
    groebner_matrix[(target_row, 64)] -= factor * groebner_matrix[(36, 81)];
    groebner_matrix[(target_row, 68)] -= factor * groebner_matrix[(36, 82)];
    groebner_matrix[(target_row, 73)] -= factor * groebner_matrix[(36, 83)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(36, 84)];
}

pub fn groebner_row45_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 61)] / groebner_matrix[(45, 61)];
    groebner_matrix[(target_row, 61)] = 0.0;
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(45, 62)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(45, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(45, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(45, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(45, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(45, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(45, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(45, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(45, 84)];
}

pub fn groebner_row36_000001_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 12)] / groebner_matrix[(36, 56)];
    groebner_matrix[(target_row, 12)] = 0.0;
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(36, 72)];
    groebner_matrix[(target_row, 37)] -= factor * groebner_matrix[(36, 73)];
    groebner_matrix[(target_row, 39)] -= factor * groebner_matrix[(36, 74)];
    groebner_matrix[(target_row, 42)] -= factor * groebner_matrix[(36, 75)];
    groebner_matrix[(target_row, 46)] -= factor * groebner_matrix[(36, 76)];
    groebner_matrix[(target_row, 51)] -= factor * groebner_matrix[(36, 77)];
    groebner_matrix[(target_row, 57)] -= factor * groebner_matrix[(36, 78)];
    groebner_matrix[(target_row, 58)] -= factor * groebner_matrix[(36, 79)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(36, 80)];
    groebner_matrix[(target_row, 63)] -= factor * groebner_matrix[(36, 81)];
    groebner_matrix[(target_row, 67)] -= factor * groebner_matrix[(36, 82)];
    groebner_matrix[(target_row, 72)] -= factor * groebner_matrix[(36, 83)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(36, 84)];
}

pub fn groebner_row43_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 57)] / groebner_matrix[(43, 57)];
    groebner_matrix[(target_row, 57)] = 0.0;
    groebner_matrix[(target_row, 59)] -= factor * groebner_matrix[(43, 59)];
    groebner_matrix[(target_row, 60)] -= factor * groebner_matrix[(43, 60)];
    groebner_matrix[(target_row, 61)] -= factor * groebner_matrix[(43, 61)];
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(43, 62)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(43, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(43, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(43, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(43, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(43, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(43, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(43, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(43, 84)];
}

pub fn groebner_row46_000000_f(groebner_matrix: &mut SMatrix<f64, 48, 85>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 59)] / groebner_matrix[(46, 59)];
    groebner_matrix[(target_row, 59)] = 0.0;
    groebner_matrix[(target_row, 62)] -= factor * groebner_matrix[(46, 62)];
    groebner_matrix[(target_row, 77)] -= factor * groebner_matrix[(46, 77)];
    groebner_matrix[(target_row, 78)] -= factor * groebner_matrix[(46, 78)];
    groebner_matrix[(target_row, 79)] -= factor * groebner_matrix[(46, 79)];
    groebner_matrix[(target_row, 80)] -= factor * groebner_matrix[(46, 80)];
    groebner_matrix[(target_row, 81)] -= factor * groebner_matrix[(46, 81)];
    groebner_matrix[(target_row, 82)] -= factor * groebner_matrix[(46, 82)];
    groebner_matrix[(target_row, 83)] -= factor * groebner_matrix[(46, 83)];
    groebner_matrix[(target_row, 84)] -= factor * groebner_matrix[(46, 84)];
}
