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

pub fn s_polynomial4(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(4, 9)] = groebner_matrix[(0, 9)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 9)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 10)] = groebner_matrix[(0, 10)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 10)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 11)] = groebner_matrix[(0, 11)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 11)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 12)] = groebner_matrix[(0, 12)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 12)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 13)] = groebner_matrix[(0, 13)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 13)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 14)] = groebner_matrix[(0, 14)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 14)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 15)] = groebner_matrix[(0, 15)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 15)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 16)] = groebner_matrix[(0, 16)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 16)] / (groebner_matrix[(1, 8)]);
    groebner_matrix[(4, 17)] = groebner_matrix[(0, 17)] / (groebner_matrix[(0, 8)])
        - groebner_matrix[(1, 17)] / (groebner_matrix[(1, 8)]);
}

pub fn s_polynomial5(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(5, 9)] = groebner_matrix[(1, 9)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 9)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 10)] = groebner_matrix[(1, 10)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 10)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 11)] = groebner_matrix[(1, 11)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 11)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 12)] = groebner_matrix[(1, 12)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 12)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 13)] = groebner_matrix[(1, 13)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 13)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 14)] = groebner_matrix[(1, 14)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 14)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 15)] = groebner_matrix[(1, 15)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 15)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 16)] = groebner_matrix[(1, 16)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 16)] / (groebner_matrix[(2, 8)]);
    groebner_matrix[(5, 17)] = groebner_matrix[(1, 17)] / (groebner_matrix[(1, 8)])
        - groebner_matrix[(2, 17)] / (groebner_matrix[(2, 8)]);
}

pub fn s_polynomial6(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(6, 9)] = groebner_matrix[(2, 9)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 9)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 10)] = groebner_matrix[(2, 10)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 10)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 11)] = groebner_matrix[(2, 11)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 11)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 12)] = groebner_matrix[(2, 12)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 12)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 13)] = groebner_matrix[(2, 13)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 13)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 14)] = groebner_matrix[(2, 14)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 14)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 15)] = groebner_matrix[(2, 15)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 15)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 16)] = groebner_matrix[(2, 16)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 16)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(6, 17)] = groebner_matrix[(2, 17)] / (groebner_matrix[(2, 8)])
        - groebner_matrix[(3, 17)] / (groebner_matrix[(3, 8)]);
}

pub fn s_polynomial7(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(7, 4)] = groebner_matrix[(4, 10)] / (groebner_matrix[(4, 9)])
        - groebner_matrix[(6, 12)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(7, 5)] = groebner_matrix[(4, 11)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(7, 6)] = groebner_matrix[(4, 12)] / (groebner_matrix[(4, 9)])
        - groebner_matrix[(6, 13)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(7, 7)] = groebner_matrix[(4, 13)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(7, 9)] = -groebner_matrix[(6, 14)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(7, 10)] = -groebner_matrix[(6, 15)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(7, 11)] = groebner_matrix[(4, 14)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(7, 12)] = groebner_matrix[(4, 15)] / (groebner_matrix[(4, 9)])
        - groebner_matrix[(6, 16)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(7, 13)] = groebner_matrix[(4, 16)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(7, 15)] = -groebner_matrix[(6, 17)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(7, 16)] = groebner_matrix[(4, 17)] / (groebner_matrix[(4, 9)]);
}

pub fn s_polynomial8(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(8, 3)] = groebner_matrix[(3, 9)] / (groebner_matrix[(3, 8)])
        - groebner_matrix[(6, 12)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(8, 4)] = groebner_matrix[(3, 10)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(8, 5)] = groebner_matrix[(3, 11)] / (groebner_matrix[(3, 8)])
        - groebner_matrix[(6, 13)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(8, 6)] = groebner_matrix[(3, 12)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(8, 7)] = groebner_matrix[(3, 13)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(8, 8)] = -groebner_matrix[(6, 14)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(8, 9)] = -groebner_matrix[(6, 15)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(8, 11)] = groebner_matrix[(3, 14)] / (groebner_matrix[(3, 8)])
        - groebner_matrix[(6, 16)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(8, 12)] = groebner_matrix[(3, 15)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(8, 13)] = groebner_matrix[(3, 16)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(8, 14)] = -groebner_matrix[(6, 17)] / (groebner_matrix[(6, 11)]);
    groebner_matrix[(8, 16)] = groebner_matrix[(3, 17)] / (groebner_matrix[(3, 8)]);
}

pub fn s_polynomial9(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(9, 1)] = groebner_matrix[(4, 10)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(9, 2)] = -groebner_matrix[(5, 11)] / (groebner_matrix[(5, 10)]);
    groebner_matrix[(9, 3)] = groebner_matrix[(4, 11)] / (groebner_matrix[(4, 9)])
        - groebner_matrix[(5, 12)] / (groebner_matrix[(5, 10)]);
    groebner_matrix[(9, 4)] = groebner_matrix[(4, 12)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(9, 5)] = -groebner_matrix[(5, 13)] / (groebner_matrix[(5, 10)]);
    groebner_matrix[(9, 6)] = groebner_matrix[(4, 13)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(9, 8)] = -groebner_matrix[(5, 14)] / (groebner_matrix[(5, 10)]);
    groebner_matrix[(9, 9)] = groebner_matrix[(4, 14)] / (groebner_matrix[(4, 9)])
        - groebner_matrix[(5, 15)] / (groebner_matrix[(5, 10)]);
    groebner_matrix[(9, 10)] = groebner_matrix[(4, 15)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(9, 11)] = -groebner_matrix[(5, 16)] / (groebner_matrix[(5, 10)]);
    groebner_matrix[(9, 12)] = groebner_matrix[(4, 16)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(9, 14)] = -groebner_matrix[(5, 17)] / (groebner_matrix[(5, 10)]);
    groebner_matrix[(9, 15)] = groebner_matrix[(4, 17)] / (groebner_matrix[(4, 9)]);
}

pub fn s_polynomial10(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(10, 0)] = groebner_matrix[(3, 9)] / (groebner_matrix[(3, 8)])
        - groebner_matrix[(4, 10)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 1)] = groebner_matrix[(3, 10)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(10, 2)] = -groebner_matrix[(4, 11)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 3)] = groebner_matrix[(3, 11)] / (groebner_matrix[(3, 8)])
        - groebner_matrix[(4, 12)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 4)] = groebner_matrix[(3, 12)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(10, 5)] = -groebner_matrix[(4, 13)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 6)] = groebner_matrix[(3, 13)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(10, 8)] = -groebner_matrix[(4, 14)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 9)] = groebner_matrix[(3, 14)] / (groebner_matrix[(3, 8)])
        - groebner_matrix[(4, 15)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 10)] = groebner_matrix[(3, 15)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(10, 11)] = -groebner_matrix[(4, 16)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 12)] = groebner_matrix[(3, 16)] / (groebner_matrix[(3, 8)]);
    groebner_matrix[(10, 14)] = -groebner_matrix[(4, 17)] / (groebner_matrix[(4, 9)]);
    groebner_matrix[(10, 15)] = groebner_matrix[(3, 17)] / (groebner_matrix[(3, 8)]);
}

pub fn s_polynomial11(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(11, 11)] = -groebner_matrix[(10, 14)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(11, 12)] = groebner_matrix[(8, 12)] / (groebner_matrix[(8, 7)])
        - groebner_matrix[(10, 15)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(11, 13)] = groebner_matrix[(8, 13)] / (groebner_matrix[(8, 7)])
        - groebner_matrix[(10, 16)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(11, 14)] = groebner_matrix[(8, 14)] / (groebner_matrix[(8, 7)]);
    groebner_matrix[(11, 15)] = groebner_matrix[(8, 15)] / (groebner_matrix[(8, 7)]);
    groebner_matrix[(11, 16)] = groebner_matrix[(8, 16)] / (groebner_matrix[(8, 7)])
        - groebner_matrix[(10, 17)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(11, 17)] = groebner_matrix[(8, 17)] / (groebner_matrix[(8, 7)]);
}

pub fn s_polynomial12(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(12, 7)] = groebner_matrix[(7, 7)] / (groebner_matrix[(7, 6)])
        - groebner_matrix[(9, 13)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(12, 11)] = -groebner_matrix[(9, 14)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(12, 12)] = groebner_matrix[(7, 12)] / (groebner_matrix[(7, 6)])
        - groebner_matrix[(9, 15)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(12, 13)] = groebner_matrix[(7, 13)] / (groebner_matrix[(7, 6)])
        - groebner_matrix[(9, 16)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(12, 14)] = groebner_matrix[(7, 14)] / (groebner_matrix[(7, 6)]);
    groebner_matrix[(12, 15)] = groebner_matrix[(7, 15)] / (groebner_matrix[(7, 6)]);
    groebner_matrix[(12, 16)] = groebner_matrix[(7, 16)] / (groebner_matrix[(7, 6)])
        - groebner_matrix[(9, 17)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(12, 17)] = groebner_matrix[(7, 17)] / (groebner_matrix[(7, 6)]);
}

pub fn s_polynomial13(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(13, 7)] = groebner_matrix[(9, 13)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(13, 9)] = -groebner_matrix[(10, 14)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(13, 10)] = -groebner_matrix[(10, 15)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(13, 11)] = groebner_matrix[(9, 14)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(13, 12)] = groebner_matrix[(9, 15)] / (groebner_matrix[(9, 12)])
        - groebner_matrix[(10, 16)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(13, 13)] = groebner_matrix[(9, 16)] / (groebner_matrix[(9, 12)]);
    groebner_matrix[(13, 15)] = -groebner_matrix[(10, 17)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(13, 16)] = groebner_matrix[(9, 17)] / (groebner_matrix[(9, 12)]);
}

pub fn s_polynomial14(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    groebner_matrix[(14, 14)] = groebner_matrix[(10, 14)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(14, 15)] = groebner_matrix[(10, 15)] / (groebner_matrix[(10, 13)]);
    groebner_matrix[(14, 16)] = groebner_matrix[(10, 16)] / (groebner_matrix[(10, 13)])
        - groebner_matrix[(13, 17)] / (groebner_matrix[(13, 16)]);
    groebner_matrix[(14, 17)] = groebner_matrix[(10, 17)] / (groebner_matrix[(10, 13)]);
}
