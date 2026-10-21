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

pub fn groebner_row5_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(5, 1)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 2)] -= factor * groebner_matrix[(5, 2)];
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(5, 3)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(5, 4)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(5, 5)];
}

pub fn groebner_row6_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 2)] / groebner_matrix[(6, 2)];
    groebner_matrix[(target_row, 2)] = 0.0;
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(6, 3)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(6, 4)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(6, 5)];
}

pub fn groebner_row7_10_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(7, 3)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 2)] -= factor * groebner_matrix[(7, 4)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(7, 5)];
}

pub fn groebner_row7_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 3)] / groebner_matrix[(7, 3)];
    groebner_matrix[(target_row, 3)] = 0.0;
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(7, 4)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(7, 5)];
}

pub fn groebner_row8_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 4)] / groebner_matrix[(8, 4)];
    groebner_matrix[(target_row, 4)] = 0.0;
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(8, 5)];
}
