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

pub fn groebner_row4_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 9)] / groebner_matrix[(4, 9)];
    groebner_matrix[(target_row, 9)] = 0.0;
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(4, 10)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(4, 11)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(4, 12)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(4, 13)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(4, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(4, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(4, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(4, 17)];
}

pub fn groebner_row5_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 10)] / groebner_matrix[(5, 10)];
    groebner_matrix[(target_row, 10)] = 0.0;
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(5, 11)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(5, 12)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(5, 13)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(5, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(5, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(5, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(5, 17)];
}

pub fn groebner_row5_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 4)] / groebner_matrix[(5, 10)];
    groebner_matrix[(target_row, 4)] = 0.0;
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(5, 11)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(5, 12)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(5, 13)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(5, 14)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(5, 15)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(5, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(5, 17)];
}

pub fn groebner_row6_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 5)] / groebner_matrix[(6, 11)];
    groebner_matrix[(target_row, 5)] = 0.0;
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(6, 12)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(6, 13)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(6, 14)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(6, 15)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(6, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(6, 17)];
}

pub fn groebner_row6_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 11)] / groebner_matrix[(6, 11)];
    groebner_matrix[(target_row, 11)] = 0.0;
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(6, 12)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(6, 13)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(6, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(6, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(6, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(6, 17)];
}

pub fn groebner_row4_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 3)] / groebner_matrix[(4, 9)];
    groebner_matrix[(target_row, 3)] = 0.0;
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(4, 10)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(4, 11)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(4, 12)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(4, 13)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(4, 14)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(4, 15)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(4, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(4, 17)];
}

pub fn groebner_row7_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 6)] / groebner_matrix[(7, 6)];
    groebner_matrix[(target_row, 6)] = 0.0;
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(7, 7)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(7, 12)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(7, 13)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(7, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(7, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(7, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(7, 17)];
}

pub fn groebner_row3_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 8)] / groebner_matrix[(3, 8)];
    groebner_matrix[(target_row, 8)] = 0.0;
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(3, 9)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(3, 10)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(3, 11)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(3, 12)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(3, 13)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(3, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(3, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(3, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(3, 17)];
}

pub fn groebner_row5_010_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(5, 10)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(5, 11)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(5, 12)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(5, 13)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(5, 14)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(5, 15)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(5, 16)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(5, 17)];
}

pub fn groebner_row3_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 2)] / groebner_matrix[(3, 8)];
    groebner_matrix[(target_row, 2)] = 0.0;
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(3, 9)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(3, 10)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(3, 11)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(3, 12)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(3, 13)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(3, 14)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(3, 15)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(3, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(3, 17)];
}

pub fn groebner_row8_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 7)] / groebner_matrix[(8, 7)];
    groebner_matrix[(target_row, 7)] = 0.0;
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(8, 12)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(8, 13)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(8, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(8, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(8, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(8, 17)];
}

pub fn groebner_row4_010_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 0)] / groebner_matrix[(4, 9)];
    groebner_matrix[(target_row, 0)] = 0.0;
    groebner_matrix[(target_row, 1)] -= factor * groebner_matrix[(4, 10)];
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(4, 11)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(4, 12)];
    groebner_matrix[(target_row, 6)] -= factor * groebner_matrix[(4, 13)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(4, 14)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(4, 15)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(4, 16)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(4, 17)];
}

pub fn groebner_row9_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 6)] / groebner_matrix[(9, 12)];
    groebner_matrix[(target_row, 6)] = 0.0;
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(9, 13)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(9, 14)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(9, 15)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(9, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(9, 17)];
}

pub fn groebner_row9_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 12)] / groebner_matrix[(9, 12)];
    groebner_matrix[(target_row, 12)] = 0.0;
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(9, 13)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(9, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(9, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(9, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(9, 17)];
}

pub fn groebner_row10_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 13)] / groebner_matrix[(10, 13)];
    groebner_matrix[(target_row, 13)] = 0.0;
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(10, 14)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(10, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(10, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(10, 17)];
}

pub fn groebner_row10_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 7)] / groebner_matrix[(10, 13)];
    groebner_matrix[(target_row, 7)] = 0.0;
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(10, 14)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(10, 15)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(10, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(10, 17)];
}

pub fn groebner_row11_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 11)] / groebner_matrix[(11, 14)];
    groebner_matrix[(target_row, 11)] = 0.0;
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(11, 15)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(11, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(11, 17)];
}

pub fn groebner_row11_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 14)] / groebner_matrix[(11, 14)];
    groebner_matrix[(target_row, 14)] = 0.0;
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(11, 15)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(11, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(11, 17)];
}

pub fn groebner_row11_010_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 9)] / groebner_matrix[(11, 14)];
    groebner_matrix[(target_row, 9)] = 0.0;
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(11, 15)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(11, 16)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(11, 17)];
}

pub fn groebner_row12_010_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 10)] / groebner_matrix[(12, 15)];
    groebner_matrix[(target_row, 10)] = 0.0;
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(12, 16)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(12, 17)];
}

pub fn groebner_row12_100_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 12)] / groebner_matrix[(12, 15)];
    groebner_matrix[(target_row, 12)] = 0.0;
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(12, 16)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(12, 17)];
}

pub fn groebner_row12_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 15)] / groebner_matrix[(12, 15)];
    groebner_matrix[(target_row, 15)] = 0.0;
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(12, 16)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(12, 17)];
}

pub fn groebner_row13_000_f(groebner_matrix: &mut SMatrix<f64, 15, 18>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 16)] / groebner_matrix[(13, 16)];
    groebner_matrix[(target_row, 16)] = 0.0;
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(13, 17)];
}
