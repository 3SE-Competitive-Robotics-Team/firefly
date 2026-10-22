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

pub fn groebner_row5_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 23)] / groebner_matrix[(5, 23)];
    groebner_matrix[(target_row, 23)] = 0.0;
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(5, 24)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(5, 25)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(5, 26)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(5, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(5, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(5, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(5, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(5, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(5, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(5, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(5, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(5, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(5, 36)];
}

pub fn groebner_row6_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 24)] / groebner_matrix[(6, 24)];
    groebner_matrix[(target_row, 24)] = 0.0;
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(6, 25)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(6, 26)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(6, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(6, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(6, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(6, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(6, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(6, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(6, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(6, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(6, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(6, 36)];
}

pub fn groebner_row7_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 25)] / groebner_matrix[(7, 25)];
    groebner_matrix[(target_row, 25)] = 0.0;
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(7, 26)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(7, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(7, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(7, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(7, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(7, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(7, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(7, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(7, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(7, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(7, 36)];
}

pub fn groebner_row7_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 9)] / groebner_matrix[(7, 25)];
    groebner_matrix[(target_row, 9)] = 0.0;
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(7, 26)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(7, 27)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(7, 28)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(7, 29)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(7, 30)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(7, 31)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(7, 32)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(7, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(7, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(7, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(7, 36)];
}

pub fn groebner_row8_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 10)] / groebner_matrix[(8, 26)];
    groebner_matrix[(target_row, 10)] = 0.0;
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(8, 27)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(8, 28)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(8, 29)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(8, 30)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(8, 31)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(8, 32)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(8, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(8, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(8, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(8, 36)];
}

pub fn groebner_row5_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 13)] / groebner_matrix[(5, 23)];
    groebner_matrix[(target_row, 13)] = 0.0;
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(5, 24)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(5, 25)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(5, 26)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(5, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(5, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(5, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(5, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(5, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(5, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(5, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(5, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(5, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(5, 36)];
}

pub fn groebner_row6_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 14)] / groebner_matrix[(6, 24)];
    groebner_matrix[(target_row, 14)] = 0.0;
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(6, 25)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(6, 26)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(6, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(6, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(6, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(6, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(6, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(6, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(6, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(6, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(6, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(6, 36)];
}

pub fn groebner_row7_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 15)] / groebner_matrix[(7, 25)];
    groebner_matrix[(target_row, 15)] = 0.0;
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(7, 26)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(7, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(7, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(7, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(7, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(7, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(7, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(7, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(7, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(7, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(7, 36)];
}

pub fn groebner_row8_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 16)] / groebner_matrix[(8, 26)];
    groebner_matrix[(target_row, 16)] = 0.0;
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(8, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(8, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(8, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(8, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(8, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(8, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(8, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(8, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(8, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(8, 36)];
}

pub fn groebner_row8_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 26)] / groebner_matrix[(8, 26)];
    groebner_matrix[(target_row, 26)] = 0.0;
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(8, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(8, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(8, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(8, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(8, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(8, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(8, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(8, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(8, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(8, 36)];
}

pub fn groebner_row6_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 8)] / groebner_matrix[(6, 24)];
    groebner_matrix[(target_row, 8)] = 0.0;
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(6, 25)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(6, 26)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(6, 27)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(6, 28)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(6, 29)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(6, 30)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(6, 31)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(6, 32)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(6, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(6, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(6, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(6, 36)];
}

pub fn groebner_row9_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 11)] / groebner_matrix[(9, 11)];
    groebner_matrix[(target_row, 11)] = 0.0;
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(9, 17)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(9, 18)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(9, 19)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(9, 20)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(9, 21)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(9, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(9, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(9, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(9, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(9, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(9, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(9, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(9, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(9, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(9, 36)];
}

pub fn groebner_row4_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 12)] / groebner_matrix[(4, 22)];
    groebner_matrix[(target_row, 12)] = 0.0;
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(4, 23)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(4, 24)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(4, 25)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(4, 26)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(4, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(4, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(4, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(4, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(4, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(4, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(4, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(4, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(4, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(4, 36)];
}

pub fn groebner_row10_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 17)] / groebner_matrix[(10, 17)];
    groebner_matrix[(target_row, 17)] = 0.0;
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(10, 18)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(10, 19)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(10, 20)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(10, 21)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(10, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(10, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(10, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(10, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(10, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(10, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(10, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(10, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(10, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(10, 36)];
}

pub fn groebner_row4_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 22)] / groebner_matrix[(4, 22)];
    groebner_matrix[(target_row, 22)] = 0.0;
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(4, 23)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(4, 24)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(4, 25)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(4, 26)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(4, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(4, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(4, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(4, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(4, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(4, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(4, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(4, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(4, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(4, 36)];
}

pub fn groebner_row5_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 7)] / groebner_matrix[(5, 23)];
    groebner_matrix[(target_row, 7)] = 0.0;
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(5, 24)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(5, 25)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(5, 26)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(5, 27)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(5, 28)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(5, 29)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(5, 30)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(5, 31)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(5, 32)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(5, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(5, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(5, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(5, 36)];
}

pub fn groebner_row11_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 18)] / groebner_matrix[(11, 18)];
    groebner_matrix[(target_row, 18)] = 0.0;
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(11, 19)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(11, 20)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(11, 21)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(11, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(11, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(11, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(11, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(11, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(11, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(11, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(11, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(11, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(11, 36)];
}

pub fn groebner_row6_0010_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 5)] / groebner_matrix[(6, 24)];
    groebner_matrix[(target_row, 5)] = 0.0;
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(6, 25)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(6, 26)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(6, 27)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(6, 28)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(6, 29)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(6, 30)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(6, 31)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(6, 32)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(6, 33)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(6, 34)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(6, 35)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(6, 36)];
}

pub fn groebner_row4_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 6)] / groebner_matrix[(4, 22)];
    groebner_matrix[(target_row, 6)] = 0.0;
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(4, 23)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(4, 24)];
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(4, 25)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(4, 26)];
    groebner_matrix[(target_row, 11)] -= factor * groebner_matrix[(4, 27)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(4, 28)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(4, 29)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(4, 30)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(4, 31)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(4, 32)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(4, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(4, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(4, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(4, 36)];
}

pub fn groebner_row12_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 19)] / groebner_matrix[(12, 19)];
    groebner_matrix[(target_row, 19)] = 0.0;
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(12, 20)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(12, 21)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(12, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(12, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(12, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(12, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(12, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(12, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(12, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(12, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(12, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(12, 36)];
}

pub fn groebner_row5_0010_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 4)] / groebner_matrix[(5, 23)];
    groebner_matrix[(target_row, 4)] = 0.0;
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(5, 24)];
    groebner_matrix[(target_row, 7)] -= factor * groebner_matrix[(5, 25)];
    groebner_matrix[(target_row, 8)] -= factor * groebner_matrix[(5, 26)];
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(5, 27)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(5, 28)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(5, 29)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(5, 30)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(5, 31)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(5, 32)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(5, 33)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(5, 34)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(5, 35)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(5, 36)];
}

pub fn groebner_row13_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 20)] / groebner_matrix[(13, 20)];
    groebner_matrix[(target_row, 20)] = 0.0;
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(13, 21)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(13, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(13, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(13, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(13, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(13, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(13, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(13, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(13, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(13, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(13, 36)];
}

pub fn groebner_row14_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 3)] / groebner_matrix[(14, 21)];
    groebner_matrix[(target_row, 3)] = 0.0;
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(14, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(14, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(14, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(14, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(14, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(14, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(14, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(14, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(14, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(14, 36)];
}

pub fn groebner_row14_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 21)] / groebner_matrix[(14, 21)];
    groebner_matrix[(target_row, 21)] = 0.0;
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(14, 27)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(14, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(14, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(14, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(14, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(14, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(14, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(14, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(14, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(14, 36)];
}

pub fn groebner_row13_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 2)] / groebner_matrix[(13, 20)];
    groebner_matrix[(target_row, 2)] = 0.0;
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(13, 21)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(13, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(13, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(13, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(13, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(13, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(13, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(13, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(13, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(13, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(13, 36)];
}

pub fn groebner_row15_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 11)] / groebner_matrix[(15, 27)];
    groebner_matrix[(target_row, 11)] = 0.0;
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(15, 28)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(15, 29)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(15, 30)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(15, 31)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(15, 32)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(15, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(15, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(15, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(15, 36)];
}

pub fn groebner_row15_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 17)] / groebner_matrix[(15, 27)];
    groebner_matrix[(target_row, 17)] = 0.0;
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(15, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(15, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(15, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(15, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(15, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(15, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(15, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(15, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(15, 36)];
}

pub fn groebner_row15_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 27)] / groebner_matrix[(15, 27)];
    groebner_matrix[(target_row, 27)] = 0.0;
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(15, 28)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(15, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(15, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(15, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(15, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(15, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(15, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(15, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(15, 36)];
}

pub fn groebner_row12_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(12, 19)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 2)] -= factor * groebner_matrix[(12, 20)];
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(12, 21)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(12, 27)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(12, 28)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(12, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(12, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(12, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(12, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(12, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(12, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(12, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(12, 36)];
}

pub fn groebner_row16_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 18)] / groebner_matrix[(16, 28)];
    groebner_matrix[(target_row, 18)] = 0.0;
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(16, 29)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(16, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(16, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(16, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(16, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(16, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(16, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(16, 36)];
}

pub fn groebner_row16_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 28)] / groebner_matrix[(16, 28)];
    groebner_matrix[(target_row, 28)] = 0.0;
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(16, 29)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(16, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(16, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(16, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(16, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(16, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(16, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(16, 36)];
}

pub fn groebner_row14_0001_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 0)] / groebner_matrix[(14, 21)];
    groebner_matrix[(target_row, 0)] = 0.0;
    groebner_matrix[(target_row, 9)] -= factor * groebner_matrix[(14, 27)];
    groebner_matrix[(target_row, 12)] -= factor * groebner_matrix[(14, 28)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(14, 29)];
    groebner_matrix[(target_row, 15)] -= factor * groebner_matrix[(14, 30)];
    groebner_matrix[(target_row, 18)] -= factor * groebner_matrix[(14, 31)];
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(14, 32)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(14, 33)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(14, 34)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(14, 35)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(14, 36)];
}

pub fn groebner_row14_0010_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(14, 21)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 10)] -= factor * groebner_matrix[(14, 27)];
    groebner_matrix[(target_row, 13)] -= factor * groebner_matrix[(14, 28)];
    groebner_matrix[(target_row, 14)] -= factor * groebner_matrix[(14, 29)];
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(14, 30)];
    groebner_matrix[(target_row, 19)] -= factor * groebner_matrix[(14, 31)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(14, 32)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(14, 33)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(14, 34)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(14, 35)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(14, 36)];
}

pub fn groebner_row17_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 19)] / groebner_matrix[(17, 29)];
    groebner_matrix[(target_row, 19)] = 0.0;
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(17, 30)];
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(17, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(17, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(17, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(17, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(17, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(17, 36)];
}

pub fn groebner_row17_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 29)] / groebner_matrix[(17, 29)];
    groebner_matrix[(target_row, 29)] = 0.0;
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(17, 30)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(17, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(17, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(17, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(17, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(17, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(17, 36)];
}

pub fn groebner_row18_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 30)] / groebner_matrix[(18, 30)];
    groebner_matrix[(target_row, 30)] = 0.0;
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(18, 31)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(18, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(18, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(18, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(18, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(18, 36)];
}

pub fn groebner_row18_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 20)] / groebner_matrix[(18, 30)];
    groebner_matrix[(target_row, 20)] = 0.0;
    groebner_matrix[(target_row, 21)] -= factor * groebner_matrix[(18, 31)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(18, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(18, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(18, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(18, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(18, 36)];
}

pub fn groebner_row19_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 21)] / groebner_matrix[(19, 31)];
    groebner_matrix[(target_row, 21)] = 0.0;
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(19, 32)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(19, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(19, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(19, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(19, 36)];
}

pub fn groebner_row19_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 31)] / groebner_matrix[(19, 31)];
    groebner_matrix[(target_row, 31)] = 0.0;
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(19, 32)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(19, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(19, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(19, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(19, 36)];
}

pub fn groebner_row20_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 28)] / groebner_matrix[(20, 32)];
    groebner_matrix[(target_row, 28)] = 0.0;
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(20, 33)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(20, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(20, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(20, 36)];
}

pub fn groebner_row20_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 32)] / groebner_matrix[(20, 32)];
    groebner_matrix[(target_row, 32)] = 0.0;
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(20, 33)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(20, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(20, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(20, 36)];
}

pub fn groebner_row19_0001_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 18)] / groebner_matrix[(19, 31)];
    groebner_matrix[(target_row, 18)] = 0.0;
    groebner_matrix[(target_row, 22)] -= factor * groebner_matrix[(19, 32)];
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(19, 33)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(19, 34)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(19, 35)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(19, 36)];
}

pub fn groebner_row19_0010_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 19)] / groebner_matrix[(19, 31)];
    groebner_matrix[(target_row, 19)] = 0.0;
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(19, 32)];
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(19, 33)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(19, 34)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(19, 35)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(19, 36)];
}

pub fn groebner_row20_0001_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 22)] / groebner_matrix[(20, 32)];
    groebner_matrix[(target_row, 22)] = 0.0;
    groebner_matrix[(target_row, 23)] -= factor * groebner_matrix[(20, 33)];
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(20, 34)];
    groebner_matrix[(target_row, 28)] -= factor * groebner_matrix[(20, 35)];
    groebner_matrix[(target_row, 32)] -= factor * groebner_matrix[(20, 36)];
}

pub fn groebner_row20_0010_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 23)] / groebner_matrix[(20, 32)];
    groebner_matrix[(target_row, 23)] = 0.0;
    groebner_matrix[(target_row, 24)] -= factor * groebner_matrix[(20, 33)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(20, 34)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(20, 35)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(20, 36)];
}

pub fn groebner_row21_0010_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 24)] / groebner_matrix[(21, 33)];
    groebner_matrix[(target_row, 24)] = 0.0;
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(21, 34)];
    groebner_matrix[(target_row, 29)] -= factor * groebner_matrix[(21, 35)];
    groebner_matrix[(target_row, 33)] -= factor * groebner_matrix[(21, 36)];
}

pub fn groebner_row20_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 25)] / groebner_matrix[(20, 32)];
    groebner_matrix[(target_row, 25)] = 0.0;
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(20, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(20, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(20, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(20, 36)];
}

pub fn groebner_row21_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 26)] / groebner_matrix[(21, 33)];
    groebner_matrix[(target_row, 26)] = 0.0;
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(21, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(21, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(21, 36)];
}

pub fn groebner_row21_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 29)] / groebner_matrix[(21, 33)];
    groebner_matrix[(target_row, 29)] = 0.0;
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(21, 34)];
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(21, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(21, 36)];
}

pub fn groebner_row21_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 33)] / groebner_matrix[(21, 33)];
    groebner_matrix[(target_row, 33)] = 0.0;
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(21, 34)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(21, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(21, 36)];
}

pub fn groebner_row20_1100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 15)] / groebner_matrix[(20, 32)];
    groebner_matrix[(target_row, 15)] = 0.0;
    groebner_matrix[(target_row, 16)] -= factor * groebner_matrix[(20, 33)];
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(20, 34)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(20, 35)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(20, 36)];
}

pub fn groebner_row21_1100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 16)] / groebner_matrix[(21, 33)];
    groebner_matrix[(target_row, 16)] = 0.0;
    groebner_matrix[(target_row, 17)] -= factor * groebner_matrix[(21, 34)];
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(21, 35)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(21, 36)];
}

pub fn groebner_row22_1100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 17)] / groebner_matrix[(22, 34)];
    groebner_matrix[(target_row, 17)] = 0.0;
    groebner_matrix[(target_row, 20)] -= factor * groebner_matrix[(22, 35)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(22, 36)];
}

pub fn groebner_row19_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 20)] / groebner_matrix[(19, 31)];
    groebner_matrix[(target_row, 20)] = 0.0;
    groebner_matrix[(target_row, 25)] -= factor * groebner_matrix[(19, 32)];
    groebner_matrix[(target_row, 26)] -= factor * groebner_matrix[(19, 33)];
    groebner_matrix[(target_row, 27)] -= factor * groebner_matrix[(19, 34)];
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(19, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(19, 36)];
}

pub fn groebner_row22_0100_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 27)] / groebner_matrix[(22, 34)];
    groebner_matrix[(target_row, 27)] = 0.0;
    groebner_matrix[(target_row, 30)] -= factor * groebner_matrix[(22, 35)];
    groebner_matrix[(target_row, 34)] -= factor * groebner_matrix[(22, 36)];
}

pub fn groebner_row22_1000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 30)] / groebner_matrix[(22, 34)];
    groebner_matrix[(target_row, 30)] = 0.0;
    groebner_matrix[(target_row, 31)] -= factor * groebner_matrix[(22, 35)];
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(22, 36)];
}

pub fn groebner_row22_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 34)] / groebner_matrix[(22, 34)];
    groebner_matrix[(target_row, 34)] = 0.0;
    groebner_matrix[(target_row, 35)] -= factor * groebner_matrix[(22, 35)];
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(22, 36)];
}

pub fn groebner_row23_0000_f(groebner_matrix: &mut SMatrix<f64, 25, 37>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 35)] / groebner_matrix[(23, 35)];
    groebner_matrix[(target_row, 35)] = 0.0;
    groebner_matrix[(target_row, 36)] -= factor * groebner_matrix[(23, 36)];
}
