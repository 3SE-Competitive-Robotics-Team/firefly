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

use super::{
    reductors::{
        groebner_row5_00000_f, groebner_row5_00100_f, groebner_row5_01000_f, groebner_row5_10000_f,
        groebner_row5_20000_f, groebner_row6_00000_f, groebner_row6_00010_f, groebner_row6_00100_f,
        groebner_row6_01000_f, groebner_row6_10000_f, groebner_row6_11000_f, groebner_row6_20000_f,
        groebner_row7_00000_f, groebner_row7_00010_f, groebner_row7_00100_f, groebner_row7_01000_f,
        groebner_row7_10000_f, groebner_row7_11000_f, groebner_row7_20000_f, groebner_row8_00000_f,
        groebner_row8_00100_f, groebner_row8_01000_f, groebner_row8_10000_f, groebner_row8_11000_f,
        groebner_row8_20000_f, groebner_row9_00000_f, groebner_row9_00100_f, groebner_row9_01000_f,
        groebner_row9_10000_f, groebner_row9_11000_f, groebner_row9_20000_f,
        groebner_row10_00000_f, groebner_row10_00100_f, groebner_row10_01000_f,
        groebner_row10_10000_f, groebner_row10_11000_f, groebner_row10_20000_f,
        groebner_row11_00000_f, groebner_row11_10000_f, groebner_row12_00000_f,
        groebner_row12_10000_f, groebner_row13_00000_f, groebner_row13_10000_f,
        groebner_row14_00000_f, groebner_row14_00100_f, groebner_row14_01000_f,
        groebner_row14_10000_f, groebner_row15_00000_f, groebner_row15_10000_f,
        groebner_row16_00000_f, groebner_row16_10000_f, groebner_row17_00000_f,
        groebner_row17_10000_f, groebner_row18_00000_f, groebner_row18_00001_f,
        groebner_row18_00010_f, groebner_row18_00100_f, groebner_row18_10000_f,
        groebner_row19_00000_f, groebner_row20_00000_f, groebner_row21_00000_f,
        groebner_row22_00000_f, groebner_row23_00000_f, groebner_row24_00000_f,
        groebner_row24_01000_f, groebner_row24_10000_f, groebner_row25_00000_f,
        groebner_row25_01000_f, groebner_row25_10000_f, groebner_row26_00000_f,
        groebner_row26_01000_f, groebner_row26_10000_f, groebner_row27_00000_f,
        groebner_row27_10000_f, groebner_row28_00000_f, groebner_row28_10000_f,
        groebner_row29_00000_f, groebner_row29_01000_f, groebner_row29_10000_f,
        groebner_row30_00000_f, groebner_row30_01000_f, groebner_row30_10000_f,
        groebner_row31_00000_f, groebner_row31_01000_f, groebner_row31_10000_f,
        groebner_row32_00000_f, groebner_row32_00100_f, groebner_row32_01000_f,
        groebner_row32_10000_f, groebner_row33_00000_f, groebner_row33_10000_f,
        groebner_row34_00000_f, groebner_row34_10000_f, groebner_row35_00000_f,
        groebner_row35_00001_f, groebner_row35_10000_f, groebner_row36_00000_f,
        groebner_row36_00001_f, groebner_row36_00010_f, groebner_row36_00100_f,
        groebner_row36_10000_f, groebner_row37_00000_f, groebner_row37_00001_f,
        groebner_row37_00010_f, groebner_row37_00100_f, groebner_row37_01000_f,
        groebner_row37_10000_f, groebner_row38_00000_f, groebner_row38_00001_f,
        groebner_row38_00010_f, groebner_row38_00100_f, groebner_row38_01000_f,
        groebner_row38_10000_f, groebner_row38_10010_f, groebner_row38_10100_f,
        groebner_row38_11000_f, groebner_row38_20000_f, groebner_row39_00000_f,
        groebner_row39_00010_f, groebner_row39_00100_f, groebner_row39_01000_f,
        groebner_row39_02000_f, groebner_row39_10000_f, groebner_row39_10010_f,
        groebner_row39_10100_f, groebner_row39_11000_f, groebner_row39_20000_f,
        groebner_row40_00000_f, groebner_row40_00100_f, groebner_row40_01000_f,
        groebner_row40_02000_f, groebner_row40_10000_f, groebner_row40_10100_f,
        groebner_row40_11000_f, groebner_row40_20000_f, groebner_row41_00000_f,
        groebner_row41_01000_f, groebner_row41_02000_f, groebner_row41_10000_f,
        groebner_row41_11000_f, groebner_row41_20000_f, groebner_row42_00000_f,
        groebner_row42_10000_f, groebner_row42_20000_f,
    },
    spolynomials::{
        s_polynomial6, s_polynomial7, s_polynomial8, s_polynomial9, s_polynomial10, s_polynomial11,
        s_polynomial12, s_polynomial13, s_polynomial14, s_polynomial15, s_polynomial16,
        s_polynomial17, s_polynomial18, s_polynomial19, s_polynomial20, s_polynomial21,
        s_polynomial22, s_polynomial23, s_polynomial24, s_polynomial25, s_polynomial26,
        s_polynomial27, s_polynomial28, s_polynomial29, s_polynomial30, s_polynomial31,
        s_polynomial32, s_polynomial33, s_polynomial34, s_polynomial35, s_polynomial36,
        s_polynomial37, s_polynomial38, s_polynomial39, s_polynomial40, s_polynomial41,
        s_polynomial42, s_polynomial43,
    },
};
use nalgebra::SMatrix;

pub fn compute(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    compute_stage_0(groebner_matrix);
    compute_stage_1(groebner_matrix);
    compute_stage_2(groebner_matrix);
    compute_stage_3(groebner_matrix);
    compute_stage_4(groebner_matrix);
    compute_stage_5(groebner_matrix);
    compute_stage_6(groebner_matrix);
    compute_stage_7(groebner_matrix);
    compute_stage_8(groebner_matrix);
    compute_stage_9(groebner_matrix);
    compute_stage_10(groebner_matrix);
    compute_stage_11(groebner_matrix);
    compute_stage_12(groebner_matrix);
    compute_stage_13(groebner_matrix);
    compute_stage_14(groebner_matrix);
    compute_stage_15(groebner_matrix);
}
#[inline]
fn compute_stage_0(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    s_polynomial6(groebner_matrix);
    s_polynomial7(groebner_matrix);
    groebner_row6_00000_f(groebner_matrix, 7);
    s_polynomial8(groebner_matrix);
    groebner_row6_00000_f(groebner_matrix, 8);
    groebner_row7_00000_f(groebner_matrix, 8);
    s_polynomial9(groebner_matrix);
    groebner_row6_00000_f(groebner_matrix, 9);
    groebner_row7_00000_f(groebner_matrix, 9);
    groebner_row8_00000_f(groebner_matrix, 9);
    s_polynomial10(groebner_matrix);
    groebner_row6_00000_f(groebner_matrix, 10);
    groebner_row7_00000_f(groebner_matrix, 10);
    groebner_row8_00000_f(groebner_matrix, 10);
    groebner_row9_00000_f(groebner_matrix, 10);
    s_polynomial11(groebner_matrix);
    groebner_row10_00100_f(groebner_matrix, 11);
    groebner_row6_01000_f(groebner_matrix, 11);
    groebner_row7_01000_f(groebner_matrix, 11);
    groebner_row8_01000_f(groebner_matrix, 11);
    groebner_row9_01000_f(groebner_matrix, 11);
    groebner_row10_01000_f(groebner_matrix, 11);
    groebner_row6_10000_f(groebner_matrix, 11);
    groebner_row7_10000_f(groebner_matrix, 11);
    groebner_row8_10000_f(groebner_matrix, 11);
    groebner_row9_10000_f(groebner_matrix, 11);
    groebner_row10_10000_f(groebner_matrix, 11);
    groebner_row6_00000_f(groebner_matrix, 11);
    groebner_row7_00000_f(groebner_matrix, 11);
    groebner_row8_00000_f(groebner_matrix, 11);
    groebner_row9_00000_f(groebner_matrix, 11);
    groebner_row10_00000_f(groebner_matrix, 11);
    s_polynomial12(groebner_matrix);
    groebner_row9_00100_f(groebner_matrix, 12);
    groebner_row10_00100_f(groebner_matrix, 12);
    groebner_row5_01000_f(groebner_matrix, 12);
    groebner_row6_01000_f(groebner_matrix, 12);
    groebner_row7_01000_f(groebner_matrix, 12);
    groebner_row8_01000_f(groebner_matrix, 12);
    groebner_row9_01000_f(groebner_matrix, 12);
    groebner_row10_01000_f(groebner_matrix, 12);
    groebner_row11_00000_f(groebner_matrix, 12);
    groebner_row5_10000_f(groebner_matrix, 12);
    groebner_row6_10000_f(groebner_matrix, 12);
    groebner_row7_10000_f(groebner_matrix, 12);
    groebner_row8_10000_f(groebner_matrix, 12);
    groebner_row9_10000_f(groebner_matrix, 12);
    groebner_row10_10000_f(groebner_matrix, 12);
    groebner_row5_00000_f(groebner_matrix, 12);
    groebner_row6_00000_f(groebner_matrix, 12);
    groebner_row7_00000_f(groebner_matrix, 12);
    groebner_row8_00000_f(groebner_matrix, 12);
    groebner_row9_00000_f(groebner_matrix, 12);
    groebner_row10_00000_f(groebner_matrix, 12);
    s_polynomial13(groebner_matrix);
    groebner_row8_00100_f(groebner_matrix, 13);
    groebner_row9_00100_f(groebner_matrix, 13);
    groebner_row10_00100_f(groebner_matrix, 13);
    groebner_row6_01000_f(groebner_matrix, 13);
    groebner_row7_01000_f(groebner_matrix, 13);
    groebner_row8_01000_f(groebner_matrix, 13);
    groebner_row9_01000_f(groebner_matrix, 13);
}

#[inline]
fn compute_stage_1(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row10_01000_f(groebner_matrix, 13);
    groebner_row11_00000_f(groebner_matrix, 13);
    groebner_row12_00000_f(groebner_matrix, 13);
    groebner_row6_10000_f(groebner_matrix, 13);
    groebner_row7_10000_f(groebner_matrix, 13);
    groebner_row8_10000_f(groebner_matrix, 13);
    groebner_row9_10000_f(groebner_matrix, 13);
    groebner_row10_10000_f(groebner_matrix, 13);
    groebner_row6_00000_f(groebner_matrix, 13);
    groebner_row7_00000_f(groebner_matrix, 13);
    groebner_row8_00000_f(groebner_matrix, 13);
    groebner_row9_00000_f(groebner_matrix, 13);
    groebner_row10_00000_f(groebner_matrix, 13);
    s_polynomial14(groebner_matrix);
    groebner_row7_00100_f(groebner_matrix, 14);
    groebner_row8_00100_f(groebner_matrix, 14);
    groebner_row9_00100_f(groebner_matrix, 14);
    groebner_row10_00100_f(groebner_matrix, 14);
    groebner_row6_01000_f(groebner_matrix, 14);
    groebner_row7_01000_f(groebner_matrix, 14);
    groebner_row8_01000_f(groebner_matrix, 14);
    groebner_row9_01000_f(groebner_matrix, 14);
    groebner_row10_01000_f(groebner_matrix, 14);
    groebner_row11_00000_f(groebner_matrix, 14);
    groebner_row12_00000_f(groebner_matrix, 14);
    groebner_row13_00000_f(groebner_matrix, 14);
    groebner_row6_10000_f(groebner_matrix, 14);
    groebner_row7_10000_f(groebner_matrix, 14);
    groebner_row8_10000_f(groebner_matrix, 14);
    groebner_row9_10000_f(groebner_matrix, 14);
    groebner_row10_10000_f(groebner_matrix, 14);
    groebner_row6_00000_f(groebner_matrix, 14);
    groebner_row7_00000_f(groebner_matrix, 14);
    groebner_row8_00000_f(groebner_matrix, 14);
    groebner_row9_00000_f(groebner_matrix, 14);
    groebner_row10_00000_f(groebner_matrix, 14);
    s_polynomial15(groebner_matrix);
    groebner_row7_00100_f(groebner_matrix, 15);
    groebner_row8_00100_f(groebner_matrix, 15);
    groebner_row9_00100_f(groebner_matrix, 15);
    groebner_row10_00100_f(groebner_matrix, 15);
    groebner_row5_01000_f(groebner_matrix, 15);
    groebner_row6_01000_f(groebner_matrix, 15);
    groebner_row7_01000_f(groebner_matrix, 15);
    groebner_row8_01000_f(groebner_matrix, 15);
    groebner_row9_01000_f(groebner_matrix, 15);
    groebner_row10_01000_f(groebner_matrix, 15);
    groebner_row11_00000_f(groebner_matrix, 15);
    groebner_row12_00000_f(groebner_matrix, 15);
    groebner_row13_00000_f(groebner_matrix, 15);
    groebner_row14_00000_f(groebner_matrix, 15);
    groebner_row5_10000_f(groebner_matrix, 15);
    groebner_row6_10000_f(groebner_matrix, 15);
    groebner_row7_10000_f(groebner_matrix, 15);
    groebner_row8_10000_f(groebner_matrix, 15);
    groebner_row9_10000_f(groebner_matrix, 15);
    groebner_row10_10000_f(groebner_matrix, 15);
    groebner_row5_00000_f(groebner_matrix, 15);
    groebner_row6_00000_f(groebner_matrix, 15);
    groebner_row7_00000_f(groebner_matrix, 15);
    groebner_row8_00000_f(groebner_matrix, 15);
    groebner_row9_00000_f(groebner_matrix, 15);
}

#[inline]
fn compute_stage_2(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row10_00000_f(groebner_matrix, 15);
    s_polynomial16(groebner_matrix);
    groebner_row6_00100_f(groebner_matrix, 16);
    groebner_row7_00100_f(groebner_matrix, 16);
    groebner_row8_00100_f(groebner_matrix, 16);
    groebner_row9_00100_f(groebner_matrix, 16);
    groebner_row10_00100_f(groebner_matrix, 16);
    groebner_row5_01000_f(groebner_matrix, 16);
    groebner_row6_01000_f(groebner_matrix, 16);
    groebner_row7_01000_f(groebner_matrix, 16);
    groebner_row8_01000_f(groebner_matrix, 16);
    groebner_row9_01000_f(groebner_matrix, 16);
    groebner_row10_01000_f(groebner_matrix, 16);
    groebner_row11_00000_f(groebner_matrix, 16);
    groebner_row12_00000_f(groebner_matrix, 16);
    groebner_row13_00000_f(groebner_matrix, 16);
    groebner_row14_00000_f(groebner_matrix, 16);
    groebner_row5_10000_f(groebner_matrix, 16);
    groebner_row6_10000_f(groebner_matrix, 16);
    groebner_row7_10000_f(groebner_matrix, 16);
    groebner_row8_10000_f(groebner_matrix, 16);
    groebner_row9_10000_f(groebner_matrix, 16);
    groebner_row10_10000_f(groebner_matrix, 16);
    groebner_row15_00000_f(groebner_matrix, 16);
    groebner_row5_00000_f(groebner_matrix, 16);
    groebner_row6_00000_f(groebner_matrix, 16);
    groebner_row7_00000_f(groebner_matrix, 16);
    groebner_row8_00000_f(groebner_matrix, 16);
    groebner_row9_00000_f(groebner_matrix, 16);
    groebner_row10_00000_f(groebner_matrix, 16);
    s_polynomial17(groebner_matrix);
    groebner_row7_00010_f(groebner_matrix, 17);
    groebner_row5_00100_f(groebner_matrix, 17);
    groebner_row6_00100_f(groebner_matrix, 17);
    groebner_row7_00100_f(groebner_matrix, 17);
    groebner_row8_00100_f(groebner_matrix, 17);
    groebner_row9_00100_f(groebner_matrix, 17);
    groebner_row10_00100_f(groebner_matrix, 17);
    groebner_row5_01000_f(groebner_matrix, 17);
    groebner_row6_01000_f(groebner_matrix, 17);
    groebner_row7_01000_f(groebner_matrix, 17);
    groebner_row8_01000_f(groebner_matrix, 17);
    groebner_row9_01000_f(groebner_matrix, 17);
    groebner_row10_01000_f(groebner_matrix, 17);
    groebner_row11_00000_f(groebner_matrix, 17);
    groebner_row12_00000_f(groebner_matrix, 17);
    groebner_row13_00000_f(groebner_matrix, 17);
    groebner_row14_00000_f(groebner_matrix, 17);
    groebner_row5_10000_f(groebner_matrix, 17);
    groebner_row6_10000_f(groebner_matrix, 17);
    groebner_row7_10000_f(groebner_matrix, 17);
    groebner_row8_10000_f(groebner_matrix, 17);
    groebner_row9_10000_f(groebner_matrix, 17);
    groebner_row10_10000_f(groebner_matrix, 17);
    groebner_row15_00000_f(groebner_matrix, 17);
    groebner_row16_00000_f(groebner_matrix, 17);
    groebner_row5_00000_f(groebner_matrix, 17);
    groebner_row6_00000_f(groebner_matrix, 17);
    groebner_row7_00000_f(groebner_matrix, 17);
    groebner_row8_00000_f(groebner_matrix, 17);
    groebner_row9_00000_f(groebner_matrix, 17);
    groebner_row10_00000_f(groebner_matrix, 17);
}

#[inline]
fn compute_stage_3(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    s_polynomial18(groebner_matrix);
    groebner_row6_00010_f(groebner_matrix, 18);
    groebner_row7_00010_f(groebner_matrix, 18);
    groebner_row5_00100_f(groebner_matrix, 18);
    groebner_row6_00100_f(groebner_matrix, 18);
    groebner_row7_00100_f(groebner_matrix, 18);
    groebner_row8_00100_f(groebner_matrix, 18);
    groebner_row9_00100_f(groebner_matrix, 18);
    groebner_row10_00100_f(groebner_matrix, 18);
    groebner_row5_01000_f(groebner_matrix, 18);
    groebner_row6_01000_f(groebner_matrix, 18);
    groebner_row7_01000_f(groebner_matrix, 18);
    groebner_row8_01000_f(groebner_matrix, 18);
    groebner_row9_01000_f(groebner_matrix, 18);
    groebner_row10_01000_f(groebner_matrix, 18);
    groebner_row11_00000_f(groebner_matrix, 18);
    groebner_row12_00000_f(groebner_matrix, 18);
    groebner_row13_00000_f(groebner_matrix, 18);
    groebner_row14_00000_f(groebner_matrix, 18);
    groebner_row5_10000_f(groebner_matrix, 18);
    groebner_row6_10000_f(groebner_matrix, 18);
    groebner_row7_10000_f(groebner_matrix, 18);
    groebner_row8_10000_f(groebner_matrix, 18);
    groebner_row9_10000_f(groebner_matrix, 18);
    groebner_row10_10000_f(groebner_matrix, 18);
    groebner_row15_00000_f(groebner_matrix, 18);
    groebner_row16_00000_f(groebner_matrix, 18);
    groebner_row17_00000_f(groebner_matrix, 18);
    groebner_row5_00000_f(groebner_matrix, 18);
    groebner_row6_00000_f(groebner_matrix, 18);
    groebner_row7_00000_f(groebner_matrix, 18);
    groebner_row8_00000_f(groebner_matrix, 18);
    groebner_row9_00000_f(groebner_matrix, 18);
    groebner_row10_00000_f(groebner_matrix, 18);
    s_polynomial19(groebner_matrix);
    groebner_row15_10000_f(groebner_matrix, 19);
    groebner_row16_10000_f(groebner_matrix, 19);
    groebner_row17_10000_f(groebner_matrix, 19);
    groebner_row18_10000_f(groebner_matrix, 19);
    groebner_row11_00000_f(groebner_matrix, 19);
    groebner_row12_00000_f(groebner_matrix, 19);
    groebner_row13_00000_f(groebner_matrix, 19);
    groebner_row14_00000_f(groebner_matrix, 19);
    groebner_row15_00000_f(groebner_matrix, 19);
    groebner_row16_00000_f(groebner_matrix, 19);
    groebner_row17_00000_f(groebner_matrix, 19);
    groebner_row18_00000_f(groebner_matrix, 19);
    s_polynomial20(groebner_matrix);
    groebner_row14_10000_f(groebner_matrix, 20);
    groebner_row15_10000_f(groebner_matrix, 20);
    groebner_row16_10000_f(groebner_matrix, 20);
    groebner_row17_10000_f(groebner_matrix, 20);
    groebner_row18_10000_f(groebner_matrix, 20);
    groebner_row19_00000_f(groebner_matrix, 20);
    groebner_row11_00000_f(groebner_matrix, 20);
    groebner_row12_00000_f(groebner_matrix, 20);
    groebner_row13_00000_f(groebner_matrix, 20);
    groebner_row14_00000_f(groebner_matrix, 20);
    groebner_row15_00000_f(groebner_matrix, 20);
    groebner_row16_00000_f(groebner_matrix, 20);
    groebner_row17_00000_f(groebner_matrix, 20);
    groebner_row18_00000_f(groebner_matrix, 20);
}

#[inline]
fn compute_stage_4(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    s_polynomial21(groebner_matrix);
    groebner_row14_10000_f(groebner_matrix, 21);
    groebner_row8_20000_f(groebner_matrix, 21);
    groebner_row9_20000_f(groebner_matrix, 21);
    groebner_row10_20000_f(groebner_matrix, 21);
    groebner_row15_10000_f(groebner_matrix, 21);
    groebner_row16_10000_f(groebner_matrix, 21);
    groebner_row17_10000_f(groebner_matrix, 21);
    groebner_row18_10000_f(groebner_matrix, 21);
    groebner_row19_00000_f(groebner_matrix, 21);
    groebner_row20_00000_f(groebner_matrix, 21);
    groebner_row8_01000_f(groebner_matrix, 21);
    groebner_row9_01000_f(groebner_matrix, 21);
    groebner_row10_01000_f(groebner_matrix, 21);
    groebner_row11_00000_f(groebner_matrix, 21);
    groebner_row12_00000_f(groebner_matrix, 21);
    groebner_row13_00000_f(groebner_matrix, 21);
    groebner_row14_00000_f(groebner_matrix, 21);
    groebner_row8_10000_f(groebner_matrix, 21);
    groebner_row9_10000_f(groebner_matrix, 21);
    groebner_row10_10000_f(groebner_matrix, 21);
    groebner_row15_00000_f(groebner_matrix, 21);
    groebner_row16_00000_f(groebner_matrix, 21);
    groebner_row17_00000_f(groebner_matrix, 21);
    groebner_row18_00000_f(groebner_matrix, 21);
    groebner_row8_00000_f(groebner_matrix, 21);
    groebner_row9_00000_f(groebner_matrix, 21);
    groebner_row10_00000_f(groebner_matrix, 21);
    s_polynomial22(groebner_matrix);
    groebner_row13_10000_f(groebner_matrix, 22);
    groebner_row14_10000_f(groebner_matrix, 22);
    groebner_row15_10000_f(groebner_matrix, 22);
    groebner_row16_10000_f(groebner_matrix, 22);
    groebner_row17_10000_f(groebner_matrix, 22);
    groebner_row18_10000_f(groebner_matrix, 22);
    groebner_row19_00000_f(groebner_matrix, 22);
    groebner_row20_00000_f(groebner_matrix, 22);
    groebner_row21_00000_f(groebner_matrix, 22);
    groebner_row11_00000_f(groebner_matrix, 22);
    groebner_row12_00000_f(groebner_matrix, 22);
    groebner_row13_00000_f(groebner_matrix, 22);
    groebner_row14_00000_f(groebner_matrix, 22);
    groebner_row15_00000_f(groebner_matrix, 22);
    groebner_row16_00000_f(groebner_matrix, 22);
    groebner_row17_00000_f(groebner_matrix, 22);
    groebner_row18_00000_f(groebner_matrix, 22);
    s_polynomial23(groebner_matrix);
    groebner_row13_10000_f(groebner_matrix, 23);
    groebner_row14_10000_f(groebner_matrix, 23);
    groebner_row6_20000_f(groebner_matrix, 23);
    groebner_row7_20000_f(groebner_matrix, 23);
    groebner_row8_20000_f(groebner_matrix, 23);
    groebner_row9_20000_f(groebner_matrix, 23);
    groebner_row10_20000_f(groebner_matrix, 23);
    groebner_row15_10000_f(groebner_matrix, 23);
    groebner_row16_10000_f(groebner_matrix, 23);
    groebner_row17_10000_f(groebner_matrix, 23);
    groebner_row18_10000_f(groebner_matrix, 23);
    groebner_row19_00000_f(groebner_matrix, 23);
    groebner_row20_00000_f(groebner_matrix, 23);
    groebner_row21_00000_f(groebner_matrix, 23);
    groebner_row22_00000_f(groebner_matrix, 23);
}

#[inline]
fn compute_stage_5(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row6_01000_f(groebner_matrix, 23);
    groebner_row7_01000_f(groebner_matrix, 23);
    groebner_row8_01000_f(groebner_matrix, 23);
    groebner_row9_01000_f(groebner_matrix, 23);
    groebner_row10_01000_f(groebner_matrix, 23);
    groebner_row11_00000_f(groebner_matrix, 23);
    groebner_row12_00000_f(groebner_matrix, 23);
    groebner_row13_00000_f(groebner_matrix, 23);
    groebner_row14_00000_f(groebner_matrix, 23);
    groebner_row6_10000_f(groebner_matrix, 23);
    groebner_row7_10000_f(groebner_matrix, 23);
    groebner_row8_10000_f(groebner_matrix, 23);
    groebner_row9_10000_f(groebner_matrix, 23);
    groebner_row10_10000_f(groebner_matrix, 23);
    groebner_row15_00000_f(groebner_matrix, 23);
    groebner_row16_00000_f(groebner_matrix, 23);
    groebner_row17_00000_f(groebner_matrix, 23);
    groebner_row18_00000_f(groebner_matrix, 23);
    groebner_row6_00000_f(groebner_matrix, 23);
    groebner_row7_00000_f(groebner_matrix, 23);
    groebner_row8_00000_f(groebner_matrix, 23);
    groebner_row9_00000_f(groebner_matrix, 23);
    groebner_row10_00000_f(groebner_matrix, 23);
    s_polynomial24(groebner_matrix);
    groebner_row12_10000_f(groebner_matrix, 24);
    groebner_row13_10000_f(groebner_matrix, 24);
    groebner_row14_10000_f(groebner_matrix, 24);
    groebner_row15_10000_f(groebner_matrix, 24);
    groebner_row16_10000_f(groebner_matrix, 24);
    groebner_row17_10000_f(groebner_matrix, 24);
    groebner_row18_10000_f(groebner_matrix, 24);
    groebner_row19_00000_f(groebner_matrix, 24);
    groebner_row20_00000_f(groebner_matrix, 24);
    groebner_row21_00000_f(groebner_matrix, 24);
    groebner_row22_00000_f(groebner_matrix, 24);
    groebner_row23_00000_f(groebner_matrix, 24);
    groebner_row11_00000_f(groebner_matrix, 24);
    groebner_row12_00000_f(groebner_matrix, 24);
    groebner_row13_00000_f(groebner_matrix, 24);
    groebner_row14_00000_f(groebner_matrix, 24);
    groebner_row15_00000_f(groebner_matrix, 24);
    groebner_row16_00000_f(groebner_matrix, 24);
    groebner_row17_00000_f(groebner_matrix, 24);
    groebner_row18_00000_f(groebner_matrix, 24);
    s_polynomial25(groebner_matrix);
    groebner_row12_10000_f(groebner_matrix, 25);
    groebner_row13_10000_f(groebner_matrix, 25);
    groebner_row14_10000_f(groebner_matrix, 25);
    groebner_row5_20000_f(groebner_matrix, 25);
    groebner_row6_20000_f(groebner_matrix, 25);
    groebner_row7_20000_f(groebner_matrix, 25);
    groebner_row8_20000_f(groebner_matrix, 25);
    groebner_row9_20000_f(groebner_matrix, 25);
    groebner_row10_20000_f(groebner_matrix, 25);
    groebner_row15_10000_f(groebner_matrix, 25);
    groebner_row16_10000_f(groebner_matrix, 25);
    groebner_row17_10000_f(groebner_matrix, 25);
    groebner_row18_10000_f(groebner_matrix, 25);
    groebner_row24_10000_f(groebner_matrix, 25);
    groebner_row20_00000_f(groebner_matrix, 25);
    groebner_row21_00000_f(groebner_matrix, 25);
    groebner_row22_00000_f(groebner_matrix, 25);
}

#[inline]
fn compute_stage_6(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row23_00000_f(groebner_matrix, 25);
    groebner_row5_01000_f(groebner_matrix, 25);
    groebner_row6_01000_f(groebner_matrix, 25);
    groebner_row7_01000_f(groebner_matrix, 25);
    groebner_row8_01000_f(groebner_matrix, 25);
    groebner_row9_01000_f(groebner_matrix, 25);
    groebner_row10_01000_f(groebner_matrix, 25);
    groebner_row11_00000_f(groebner_matrix, 25);
    groebner_row12_00000_f(groebner_matrix, 25);
    groebner_row13_00000_f(groebner_matrix, 25);
    groebner_row14_00000_f(groebner_matrix, 25);
    groebner_row5_10000_f(groebner_matrix, 25);
    groebner_row6_10000_f(groebner_matrix, 25);
    groebner_row7_10000_f(groebner_matrix, 25);
    groebner_row8_10000_f(groebner_matrix, 25);
    groebner_row9_10000_f(groebner_matrix, 25);
    groebner_row10_10000_f(groebner_matrix, 25);
    groebner_row15_00000_f(groebner_matrix, 25);
    groebner_row16_00000_f(groebner_matrix, 25);
    groebner_row17_00000_f(groebner_matrix, 25);
    groebner_row18_00000_f(groebner_matrix, 25);
    groebner_row24_00000_f(groebner_matrix, 25);
    groebner_row5_00000_f(groebner_matrix, 25);
    groebner_row6_00000_f(groebner_matrix, 25);
    groebner_row7_00000_f(groebner_matrix, 25);
    groebner_row8_00000_f(groebner_matrix, 25);
    groebner_row9_00000_f(groebner_matrix, 25);
    groebner_row10_00000_f(groebner_matrix, 25);
    s_polynomial26(groebner_matrix);
    groebner_row11_10000_f(groebner_matrix, 26);
    groebner_row12_10000_f(groebner_matrix, 26);
    groebner_row13_10000_f(groebner_matrix, 26);
    groebner_row14_10000_f(groebner_matrix, 26);
    groebner_row8_20000_f(groebner_matrix, 26);
    groebner_row9_20000_f(groebner_matrix, 26);
    groebner_row10_20000_f(groebner_matrix, 26);
    groebner_row15_10000_f(groebner_matrix, 26);
    groebner_row16_10000_f(groebner_matrix, 26);
    groebner_row17_10000_f(groebner_matrix, 26);
    groebner_row18_10000_f(groebner_matrix, 26);
    groebner_row24_10000_f(groebner_matrix, 26);
    groebner_row25_10000_f(groebner_matrix, 26);
    groebner_row21_00000_f(groebner_matrix, 26);
    groebner_row22_00000_f(groebner_matrix, 26);
    groebner_row23_00000_f(groebner_matrix, 26);
    groebner_row8_01000_f(groebner_matrix, 26);
    groebner_row9_01000_f(groebner_matrix, 26);
    groebner_row10_01000_f(groebner_matrix, 26);
    groebner_row11_00000_f(groebner_matrix, 26);
    groebner_row12_00000_f(groebner_matrix, 26);
    groebner_row13_00000_f(groebner_matrix, 26);
    groebner_row14_00000_f(groebner_matrix, 26);
    groebner_row8_10000_f(groebner_matrix, 26);
    groebner_row9_10000_f(groebner_matrix, 26);
    groebner_row10_10000_f(groebner_matrix, 26);
    groebner_row15_00000_f(groebner_matrix, 26);
    groebner_row16_00000_f(groebner_matrix, 26);
    groebner_row17_00000_f(groebner_matrix, 26);
    groebner_row18_00000_f(groebner_matrix, 26);
    groebner_row24_00000_f(groebner_matrix, 26);
    groebner_row25_00000_f(groebner_matrix, 26);
    groebner_row8_00000_f(groebner_matrix, 26);
}

#[inline]
fn compute_stage_7(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row9_00000_f(groebner_matrix, 26);
    groebner_row10_00000_f(groebner_matrix, 26);
    s_polynomial27(groebner_matrix);
    groebner_row10_11000_f(groebner_matrix, 27);
    groebner_row11_10000_f(groebner_matrix, 27);
    groebner_row12_10000_f(groebner_matrix, 27);
    groebner_row13_10000_f(groebner_matrix, 27);
    groebner_row14_10000_f(groebner_matrix, 27);
    groebner_row8_20000_f(groebner_matrix, 27);
    groebner_row9_20000_f(groebner_matrix, 27);
    groebner_row10_20000_f(groebner_matrix, 27);
    groebner_row15_10000_f(groebner_matrix, 27);
    groebner_row16_10000_f(groebner_matrix, 27);
    groebner_row17_10000_f(groebner_matrix, 27);
    groebner_row18_10000_f(groebner_matrix, 27);
    groebner_row24_10000_f(groebner_matrix, 27);
    groebner_row25_10000_f(groebner_matrix, 27);
    groebner_row26_10000_f(groebner_matrix, 27);
    groebner_row22_00000_f(groebner_matrix, 27);
    groebner_row23_00000_f(groebner_matrix, 27);
    groebner_row8_01000_f(groebner_matrix, 27);
    groebner_row9_01000_f(groebner_matrix, 27);
    groebner_row10_01000_f(groebner_matrix, 27);
    groebner_row11_00000_f(groebner_matrix, 27);
    groebner_row12_00000_f(groebner_matrix, 27);
    groebner_row13_00000_f(groebner_matrix, 27);
    groebner_row14_00000_f(groebner_matrix, 27);
    groebner_row8_10000_f(groebner_matrix, 27);
    groebner_row9_10000_f(groebner_matrix, 27);
    groebner_row10_10000_f(groebner_matrix, 27);
    groebner_row15_00000_f(groebner_matrix, 27);
    groebner_row16_00000_f(groebner_matrix, 27);
    groebner_row17_00000_f(groebner_matrix, 27);
    groebner_row18_00000_f(groebner_matrix, 27);
    groebner_row24_00000_f(groebner_matrix, 27);
    groebner_row25_00000_f(groebner_matrix, 27);
    groebner_row26_00000_f(groebner_matrix, 27);
    groebner_row8_00000_f(groebner_matrix, 27);
    groebner_row9_00000_f(groebner_matrix, 27);
    groebner_row10_00000_f(groebner_matrix, 27);
    s_polynomial28(groebner_matrix);
    groebner_row10_11000_f(groebner_matrix, 28);
    groebner_row11_10000_f(groebner_matrix, 28);
    groebner_row12_10000_f(groebner_matrix, 28);
    groebner_row13_10000_f(groebner_matrix, 28);
    groebner_row14_10000_f(groebner_matrix, 28);
    groebner_row6_20000_f(groebner_matrix, 28);
    groebner_row7_20000_f(groebner_matrix, 28);
    groebner_row8_20000_f(groebner_matrix, 28);
    groebner_row9_20000_f(groebner_matrix, 28);
    groebner_row10_20000_f(groebner_matrix, 28);
    groebner_row15_10000_f(groebner_matrix, 28);
    groebner_row16_10000_f(groebner_matrix, 28);
    groebner_row17_10000_f(groebner_matrix, 28);
    groebner_row18_10000_f(groebner_matrix, 28);
    groebner_row24_10000_f(groebner_matrix, 28);
    groebner_row25_10000_f(groebner_matrix, 28);
    groebner_row26_10000_f(groebner_matrix, 28);
    groebner_row27_10000_f(groebner_matrix, 28);
    groebner_row23_00000_f(groebner_matrix, 28);
    groebner_row6_01000_f(groebner_matrix, 28);
    groebner_row7_01000_f(groebner_matrix, 28);
}

#[inline]
fn compute_stage_8(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row8_01000_f(groebner_matrix, 28);
    groebner_row9_01000_f(groebner_matrix, 28);
    groebner_row10_01000_f(groebner_matrix, 28);
    groebner_row11_00000_f(groebner_matrix, 28);
    groebner_row12_00000_f(groebner_matrix, 28);
    groebner_row13_00000_f(groebner_matrix, 28);
    groebner_row14_00000_f(groebner_matrix, 28);
    groebner_row6_10000_f(groebner_matrix, 28);
    groebner_row7_10000_f(groebner_matrix, 28);
    groebner_row8_10000_f(groebner_matrix, 28);
    groebner_row9_10000_f(groebner_matrix, 28);
    groebner_row10_10000_f(groebner_matrix, 28);
    groebner_row15_00000_f(groebner_matrix, 28);
    groebner_row16_00000_f(groebner_matrix, 28);
    groebner_row17_00000_f(groebner_matrix, 28);
    groebner_row18_00000_f(groebner_matrix, 28);
    groebner_row24_00000_f(groebner_matrix, 28);
    groebner_row25_00000_f(groebner_matrix, 28);
    groebner_row26_00000_f(groebner_matrix, 28);
    groebner_row27_00000_f(groebner_matrix, 28);
    groebner_row6_00000_f(groebner_matrix, 28);
    groebner_row7_00000_f(groebner_matrix, 28);
    groebner_row8_00000_f(groebner_matrix, 28);
    groebner_row9_00000_f(groebner_matrix, 28);
    groebner_row10_00000_f(groebner_matrix, 28);
    s_polynomial29(groebner_matrix);
    groebner_row9_11000_f(groebner_matrix, 29);
    groebner_row10_11000_f(groebner_matrix, 29);
    groebner_row11_10000_f(groebner_matrix, 29);
    groebner_row12_10000_f(groebner_matrix, 29);
    groebner_row13_10000_f(groebner_matrix, 29);
    groebner_row14_10000_f(groebner_matrix, 29);
    groebner_row8_20000_f(groebner_matrix, 29);
    groebner_row9_20000_f(groebner_matrix, 29);
    groebner_row10_20000_f(groebner_matrix, 29);
    groebner_row15_10000_f(groebner_matrix, 29);
    groebner_row16_10000_f(groebner_matrix, 29);
    groebner_row17_10000_f(groebner_matrix, 29);
    groebner_row18_10000_f(groebner_matrix, 29);
    groebner_row24_10000_f(groebner_matrix, 29);
    groebner_row25_10000_f(groebner_matrix, 29);
    groebner_row26_10000_f(groebner_matrix, 29);
    groebner_row27_10000_f(groebner_matrix, 29);
    groebner_row28_10000_f(groebner_matrix, 29);
    groebner_row8_01000_f(groebner_matrix, 29);
    groebner_row9_01000_f(groebner_matrix, 29);
    groebner_row10_01000_f(groebner_matrix, 29);
    groebner_row11_00000_f(groebner_matrix, 29);
    groebner_row12_00000_f(groebner_matrix, 29);
    groebner_row13_00000_f(groebner_matrix, 29);
    groebner_row14_00000_f(groebner_matrix, 29);
    groebner_row8_10000_f(groebner_matrix, 29);
    groebner_row9_10000_f(groebner_matrix, 29);
    groebner_row10_10000_f(groebner_matrix, 29);
    groebner_row15_00000_f(groebner_matrix, 29);
    groebner_row16_00000_f(groebner_matrix, 29);
    groebner_row17_00000_f(groebner_matrix, 29);
    groebner_row18_00000_f(groebner_matrix, 29);
    groebner_row24_00000_f(groebner_matrix, 29);
    groebner_row25_00000_f(groebner_matrix, 29);
    groebner_row26_00000_f(groebner_matrix, 29);
    groebner_row27_00000_f(groebner_matrix, 29);
}

#[inline]
fn compute_stage_9(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row28_00000_f(groebner_matrix, 29);
    groebner_row8_00000_f(groebner_matrix, 29);
    groebner_row9_00000_f(groebner_matrix, 29);
    groebner_row10_00000_f(groebner_matrix, 29);
    s_polynomial30(groebner_matrix);
    groebner_row9_11000_f(groebner_matrix, 30);
    groebner_row10_11000_f(groebner_matrix, 30);
    groebner_row18_00001_f(groebner_matrix, 30);
    groebner_row12_10000_f(groebner_matrix, 30);
    groebner_row13_10000_f(groebner_matrix, 30);
    groebner_row14_10000_f(groebner_matrix, 30);
    groebner_row5_20000_f(groebner_matrix, 30);
    groebner_row6_20000_f(groebner_matrix, 30);
    groebner_row7_20000_f(groebner_matrix, 30);
    groebner_row8_20000_f(groebner_matrix, 30);
    groebner_row9_20000_f(groebner_matrix, 30);
    groebner_row10_20000_f(groebner_matrix, 30);
    groebner_row24_01000_f(groebner_matrix, 30);
    groebner_row16_10000_f(groebner_matrix, 30);
    groebner_row17_10000_f(groebner_matrix, 30);
    groebner_row18_10000_f(groebner_matrix, 30);
    groebner_row24_10000_f(groebner_matrix, 30);
    groebner_row25_10000_f(groebner_matrix, 30);
    groebner_row26_10000_f(groebner_matrix, 30);
    groebner_row27_10000_f(groebner_matrix, 30);
    groebner_row28_10000_f(groebner_matrix, 30);
    groebner_row5_01000_f(groebner_matrix, 30);
    groebner_row6_01000_f(groebner_matrix, 30);
    groebner_row7_01000_f(groebner_matrix, 30);
    groebner_row8_01000_f(groebner_matrix, 30);
    groebner_row9_01000_f(groebner_matrix, 30);
    groebner_row10_01000_f(groebner_matrix, 30);
    groebner_row29_01000_f(groebner_matrix, 30);
    groebner_row12_00000_f(groebner_matrix, 30);
    groebner_row13_00000_f(groebner_matrix, 30);
    groebner_row14_00000_f(groebner_matrix, 30);
    groebner_row5_10000_f(groebner_matrix, 30);
    groebner_row6_10000_f(groebner_matrix, 30);
    groebner_row7_10000_f(groebner_matrix, 30);
    groebner_row8_10000_f(groebner_matrix, 30);
    groebner_row9_10000_f(groebner_matrix, 30);
    groebner_row10_10000_f(groebner_matrix, 30);
    groebner_row29_10000_f(groebner_matrix, 30);
    groebner_row16_00000_f(groebner_matrix, 30);
    groebner_row17_00000_f(groebner_matrix, 30);
    groebner_row18_00000_f(groebner_matrix, 30);
    groebner_row24_00000_f(groebner_matrix, 30);
    groebner_row25_00000_f(groebner_matrix, 30);
    groebner_row26_00000_f(groebner_matrix, 30);
    groebner_row27_00000_f(groebner_matrix, 30);
    groebner_row28_00000_f(groebner_matrix, 30);
    groebner_row5_00000_f(groebner_matrix, 30);
    groebner_row6_00000_f(groebner_matrix, 30);
    groebner_row7_00000_f(groebner_matrix, 30);
    groebner_row8_00000_f(groebner_matrix, 30);
    groebner_row9_00000_f(groebner_matrix, 30);
    groebner_row10_00000_f(groebner_matrix, 30);
    groebner_row29_00000_f(groebner_matrix, 30);
    s_polynomial31(groebner_matrix);
    groebner_row14_01000_f(groebner_matrix, 31);
    groebner_row8_11000_f(groebner_matrix, 31);
    groebner_row9_11000_f(groebner_matrix, 31);
}

#[inline]
fn compute_stage_10(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row10_11000_f(groebner_matrix, 31);
    groebner_row18_00001_f(groebner_matrix, 31);
    groebner_row18_00010_f(groebner_matrix, 31);
    groebner_row13_10000_f(groebner_matrix, 31);
    groebner_row14_10000_f(groebner_matrix, 31);
    groebner_row5_20000_f(groebner_matrix, 31);
    groebner_row6_20000_f(groebner_matrix, 31);
    groebner_row7_20000_f(groebner_matrix, 31);
    groebner_row8_20000_f(groebner_matrix, 31);
    groebner_row9_20000_f(groebner_matrix, 31);
    groebner_row10_20000_f(groebner_matrix, 31);
    groebner_row24_01000_f(groebner_matrix, 31);
    groebner_row25_01000_f(groebner_matrix, 31);
    groebner_row17_10000_f(groebner_matrix, 31);
    groebner_row18_10000_f(groebner_matrix, 31);
    groebner_row24_10000_f(groebner_matrix, 31);
    groebner_row25_10000_f(groebner_matrix, 31);
    groebner_row26_10000_f(groebner_matrix, 31);
    groebner_row27_10000_f(groebner_matrix, 31);
    groebner_row28_10000_f(groebner_matrix, 31);
    groebner_row5_01000_f(groebner_matrix, 31);
    groebner_row6_01000_f(groebner_matrix, 31);
    groebner_row7_01000_f(groebner_matrix, 31);
    groebner_row8_01000_f(groebner_matrix, 31);
    groebner_row9_01000_f(groebner_matrix, 31);
    groebner_row10_01000_f(groebner_matrix, 31);
    groebner_row29_01000_f(groebner_matrix, 31);
    groebner_row30_01000_f(groebner_matrix, 31);
    groebner_row13_00000_f(groebner_matrix, 31);
    groebner_row14_00000_f(groebner_matrix, 31);
    groebner_row5_10000_f(groebner_matrix, 31);
    groebner_row6_10000_f(groebner_matrix, 31);
    groebner_row7_10000_f(groebner_matrix, 31);
    groebner_row8_10000_f(groebner_matrix, 31);
    groebner_row9_10000_f(groebner_matrix, 31);
    groebner_row10_10000_f(groebner_matrix, 31);
    groebner_row29_10000_f(groebner_matrix, 31);
    groebner_row30_10000_f(groebner_matrix, 31);
    groebner_row17_00000_f(groebner_matrix, 31);
    groebner_row18_00000_f(groebner_matrix, 31);
    groebner_row24_00000_f(groebner_matrix, 31);
    groebner_row25_00000_f(groebner_matrix, 31);
    groebner_row26_00000_f(groebner_matrix, 31);
    groebner_row27_00000_f(groebner_matrix, 31);
    groebner_row28_00000_f(groebner_matrix, 31);
    groebner_row5_00000_f(groebner_matrix, 31);
    groebner_row6_00000_f(groebner_matrix, 31);
    groebner_row7_00000_f(groebner_matrix, 31);
    groebner_row8_00000_f(groebner_matrix, 31);
    groebner_row9_00000_f(groebner_matrix, 31);
    groebner_row10_00000_f(groebner_matrix, 31);
    groebner_row29_00000_f(groebner_matrix, 31);
    groebner_row30_00000_f(groebner_matrix, 31);
    s_polynomial32(groebner_matrix);
    groebner_row14_00100_f(groebner_matrix, 32);
    groebner_row14_01000_f(groebner_matrix, 32);
    groebner_row6_11000_f(groebner_matrix, 32);
    groebner_row7_11000_f(groebner_matrix, 32);
    groebner_row8_11000_f(groebner_matrix, 32);
    groebner_row9_11000_f(groebner_matrix, 32);
    groebner_row10_11000_f(groebner_matrix, 32);
    groebner_row18_00001_f(groebner_matrix, 32);
}

#[inline]
fn compute_stage_11(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row18_00010_f(groebner_matrix, 32);
    groebner_row18_00100_f(groebner_matrix, 32);
    groebner_row14_10000_f(groebner_matrix, 32);
    groebner_row5_20000_f(groebner_matrix, 32);
    groebner_row6_20000_f(groebner_matrix, 32);
    groebner_row7_20000_f(groebner_matrix, 32);
    groebner_row8_20000_f(groebner_matrix, 32);
    groebner_row9_20000_f(groebner_matrix, 32);
    groebner_row10_20000_f(groebner_matrix, 32);
    groebner_row24_01000_f(groebner_matrix, 32);
    groebner_row25_01000_f(groebner_matrix, 32);
    groebner_row26_01000_f(groebner_matrix, 32);
    groebner_row18_10000_f(groebner_matrix, 32);
    groebner_row24_10000_f(groebner_matrix, 32);
    groebner_row25_10000_f(groebner_matrix, 32);
    groebner_row26_10000_f(groebner_matrix, 32);
    groebner_row27_10000_f(groebner_matrix, 32);
    groebner_row28_10000_f(groebner_matrix, 32);
    groebner_row5_01000_f(groebner_matrix, 32);
    groebner_row6_01000_f(groebner_matrix, 32);
    groebner_row7_01000_f(groebner_matrix, 32);
    groebner_row8_01000_f(groebner_matrix, 32);
    groebner_row9_01000_f(groebner_matrix, 32);
    groebner_row10_01000_f(groebner_matrix, 32);
    groebner_row29_01000_f(groebner_matrix, 32);
    groebner_row30_01000_f(groebner_matrix, 32);
    groebner_row31_01000_f(groebner_matrix, 32);
    groebner_row14_00000_f(groebner_matrix, 32);
    groebner_row5_10000_f(groebner_matrix, 32);
    groebner_row6_10000_f(groebner_matrix, 32);
    groebner_row7_10000_f(groebner_matrix, 32);
    groebner_row8_10000_f(groebner_matrix, 32);
    groebner_row9_10000_f(groebner_matrix, 32);
    groebner_row10_10000_f(groebner_matrix, 32);
    groebner_row29_10000_f(groebner_matrix, 32);
    groebner_row30_10000_f(groebner_matrix, 32);
    groebner_row31_10000_f(groebner_matrix, 32);
    groebner_row18_00000_f(groebner_matrix, 32);
    groebner_row24_00000_f(groebner_matrix, 32);
    groebner_row25_00000_f(groebner_matrix, 32);
    groebner_row26_00000_f(groebner_matrix, 32);
    groebner_row27_00000_f(groebner_matrix, 32);
    groebner_row28_00000_f(groebner_matrix, 32);
    groebner_row5_00000_f(groebner_matrix, 32);
    groebner_row6_00000_f(groebner_matrix, 32);
    groebner_row7_00000_f(groebner_matrix, 32);
    groebner_row8_00000_f(groebner_matrix, 32);
    groebner_row9_00000_f(groebner_matrix, 32);
    groebner_row10_00000_f(groebner_matrix, 32);
    groebner_row29_00000_f(groebner_matrix, 32);
    groebner_row30_00000_f(groebner_matrix, 32);
    groebner_row31_00000_f(groebner_matrix, 32);
    s_polynomial33(groebner_matrix);
    groebner_row24_00000_f(groebner_matrix, 33);
    groebner_row25_00000_f(groebner_matrix, 33);
    groebner_row26_00000_f(groebner_matrix, 33);
    groebner_row27_00000_f(groebner_matrix, 33);
    groebner_row28_00000_f(groebner_matrix, 33);
    groebner_row29_00000_f(groebner_matrix, 33);
    groebner_row30_00000_f(groebner_matrix, 33);
    groebner_row31_00000_f(groebner_matrix, 33);
    groebner_row32_00000_f(groebner_matrix, 33);
}

#[inline]
fn compute_stage_12(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    s_polynomial34(groebner_matrix);
    groebner_row32_10000_f(groebner_matrix, 34);
    groebner_row33_10000_f(groebner_matrix, 34);
    groebner_row25_00000_f(groebner_matrix, 34);
    groebner_row26_00000_f(groebner_matrix, 34);
    groebner_row27_00000_f(groebner_matrix, 34);
    groebner_row28_00000_f(groebner_matrix, 34);
    groebner_row29_00000_f(groebner_matrix, 34);
    groebner_row30_00000_f(groebner_matrix, 34);
    groebner_row31_00000_f(groebner_matrix, 34);
    groebner_row32_00000_f(groebner_matrix, 34);
    groebner_row33_00000_f(groebner_matrix, 34);
    s_polynomial35(groebner_matrix);
    groebner_row31_10000_f(groebner_matrix, 35);
    groebner_row32_10000_f(groebner_matrix, 35);
    groebner_row33_10000_f(groebner_matrix, 35);
    groebner_row34_10000_f(groebner_matrix, 35);
    groebner_row26_00000_f(groebner_matrix, 35);
    groebner_row27_00000_f(groebner_matrix, 35);
    groebner_row28_00000_f(groebner_matrix, 35);
    groebner_row29_00000_f(groebner_matrix, 35);
    groebner_row30_00000_f(groebner_matrix, 35);
    groebner_row31_00000_f(groebner_matrix, 35);
    groebner_row32_00000_f(groebner_matrix, 35);
    groebner_row33_00000_f(groebner_matrix, 35);
    groebner_row34_00000_f(groebner_matrix, 35);
    s_polynomial36(groebner_matrix);
    groebner_row30_10000_f(groebner_matrix, 36);
    groebner_row31_10000_f(groebner_matrix, 36);
    groebner_row32_10000_f(groebner_matrix, 36);
    groebner_row33_10000_f(groebner_matrix, 36);
    groebner_row34_10000_f(groebner_matrix, 36);
    groebner_row35_10000_f(groebner_matrix, 36);
    groebner_row27_00000_f(groebner_matrix, 36);
    groebner_row28_00000_f(groebner_matrix, 36);
    groebner_row29_00000_f(groebner_matrix, 36);
    groebner_row30_00000_f(groebner_matrix, 36);
    groebner_row31_00000_f(groebner_matrix, 36);
    groebner_row32_00000_f(groebner_matrix, 36);
    groebner_row33_00000_f(groebner_matrix, 36);
    groebner_row34_00000_f(groebner_matrix, 36);
    groebner_row35_00000_f(groebner_matrix, 36);
    s_polynomial37(groebner_matrix);
    groebner_row29_10000_f(groebner_matrix, 37);
    groebner_row30_10000_f(groebner_matrix, 37);
    groebner_row31_10000_f(groebner_matrix, 37);
    groebner_row32_10000_f(groebner_matrix, 37);
    groebner_row33_10000_f(groebner_matrix, 37);
    groebner_row34_10000_f(groebner_matrix, 37);
    groebner_row35_10000_f(groebner_matrix, 37);
    groebner_row36_10000_f(groebner_matrix, 37);
    groebner_row28_00000_f(groebner_matrix, 37);
    groebner_row29_00000_f(groebner_matrix, 37);
    groebner_row30_00000_f(groebner_matrix, 37);
    groebner_row31_00000_f(groebner_matrix, 37);
    groebner_row32_00000_f(groebner_matrix, 37);
    groebner_row33_00000_f(groebner_matrix, 37);
    groebner_row34_00000_f(groebner_matrix, 37);
    groebner_row35_00000_f(groebner_matrix, 37);
    groebner_row36_00000_f(groebner_matrix, 37);
    s_polynomial38(groebner_matrix);
    groebner_row32_01000_f(groebner_matrix, 38);
}

#[inline]
fn compute_stage_13(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row29_10000_f(groebner_matrix, 38);
    groebner_row30_10000_f(groebner_matrix, 38);
    groebner_row31_10000_f(groebner_matrix, 38);
    groebner_row32_10000_f(groebner_matrix, 38);
    groebner_row33_10000_f(groebner_matrix, 38);
    groebner_row34_10000_f(groebner_matrix, 38);
    groebner_row35_10000_f(groebner_matrix, 38);
    groebner_row36_10000_f(groebner_matrix, 38);
    groebner_row37_10000_f(groebner_matrix, 38);
    groebner_row29_00000_f(groebner_matrix, 38);
    groebner_row30_00000_f(groebner_matrix, 38);
    groebner_row31_00000_f(groebner_matrix, 38);
    groebner_row32_00000_f(groebner_matrix, 38);
    groebner_row33_00000_f(groebner_matrix, 38);
    groebner_row34_00000_f(groebner_matrix, 38);
    groebner_row35_00000_f(groebner_matrix, 38);
    groebner_row36_00000_f(groebner_matrix, 38);
    groebner_row37_00000_f(groebner_matrix, 38);
    s_polynomial39(groebner_matrix);
    groebner_row32_01000_f(groebner_matrix, 39);
    groebner_row35_00001_f(groebner_matrix, 39);
    groebner_row9_10000_f(groebner_matrix, 39);
    groebner_row10_10000_f(groebner_matrix, 39);
    groebner_row36_00001_f(groebner_matrix, 39);
    groebner_row30_10000_f(groebner_matrix, 39);
    groebner_row31_10000_f(groebner_matrix, 39);
    groebner_row32_10000_f(groebner_matrix, 39);
    groebner_row37_00001_f(groebner_matrix, 39);
    groebner_row34_10000_f(groebner_matrix, 39);
    groebner_row35_10000_f(groebner_matrix, 39);
    groebner_row36_10000_f(groebner_matrix, 39);
    groebner_row37_10000_f(groebner_matrix, 39);
    groebner_row38_00001_f(groebner_matrix, 39);
    groebner_row38_00010_f(groebner_matrix, 39);
    groebner_row7_00000_f(groebner_matrix, 39);
    groebner_row38_00100_f(groebner_matrix, 39);
    groebner_row9_00000_f(groebner_matrix, 39);
    groebner_row10_00000_f(groebner_matrix, 39);
    groebner_row38_01000_f(groebner_matrix, 39);
    groebner_row30_00000_f(groebner_matrix, 39);
    groebner_row31_00000_f(groebner_matrix, 39);
    groebner_row32_00000_f(groebner_matrix, 39);
    groebner_row38_10000_f(groebner_matrix, 39);
    groebner_row34_00000_f(groebner_matrix, 39);
    groebner_row35_00000_f(groebner_matrix, 39);
    groebner_row36_00000_f(groebner_matrix, 39);
    groebner_row37_00000_f(groebner_matrix, 39);
    groebner_row38_00000_f(groebner_matrix, 39);
    s_polynomial40(groebner_matrix);
    groebner_row31_01000_f(groebner_matrix, 40);
    groebner_row32_01000_f(groebner_matrix, 40);
    groebner_row36_00001_f(groebner_matrix, 40);
    groebner_row36_00010_f(groebner_matrix, 40);
    groebner_row31_10000_f(groebner_matrix, 40);
    groebner_row32_10000_f(groebner_matrix, 40);
    groebner_row37_00001_f(groebner_matrix, 40);
    groebner_row37_00010_f(groebner_matrix, 40);
    groebner_row35_10000_f(groebner_matrix, 40);
    groebner_row36_10000_f(groebner_matrix, 40);
    groebner_row37_10000_f(groebner_matrix, 40);
    groebner_row38_00001_f(groebner_matrix, 40);
    groebner_row38_00010_f(groebner_matrix, 40);
}

#[inline]
fn compute_stage_14(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row39_00010_f(groebner_matrix, 40);
    groebner_row38_00100_f(groebner_matrix, 40);
    groebner_row39_00100_f(groebner_matrix, 40);
    groebner_row10_00000_f(groebner_matrix, 40);
    groebner_row38_01000_f(groebner_matrix, 40);
    groebner_row39_01000_f(groebner_matrix, 40);
    groebner_row31_00000_f(groebner_matrix, 40);
    groebner_row32_00000_f(groebner_matrix, 40);
    groebner_row38_10000_f(groebner_matrix, 40);
    groebner_row39_10000_f(groebner_matrix, 40);
    groebner_row35_00000_f(groebner_matrix, 40);
    groebner_row36_00000_f(groebner_matrix, 40);
    groebner_row37_00000_f(groebner_matrix, 40);
    groebner_row38_00000_f(groebner_matrix, 40);
    groebner_row39_00000_f(groebner_matrix, 40);
    s_polynomial41(groebner_matrix);
    groebner_row32_00100_f(groebner_matrix, 41);
    groebner_row32_01000_f(groebner_matrix, 41);
    groebner_row38_10010_f(groebner_matrix, 41);
    groebner_row39_10010_f(groebner_matrix, 41);
    groebner_row38_10100_f(groebner_matrix, 41);
    groebner_row39_10100_f(groebner_matrix, 41);
    groebner_row40_10100_f(groebner_matrix, 41);
    groebner_row36_00001_f(groebner_matrix, 41);
    groebner_row36_00010_f(groebner_matrix, 41);
    groebner_row36_00100_f(groebner_matrix, 41);
    groebner_row32_10000_f(groebner_matrix, 41);
    groebner_row37_00001_f(groebner_matrix, 41);
    groebner_row37_00010_f(groebner_matrix, 41);
    groebner_row37_00100_f(groebner_matrix, 41);
    groebner_row36_10000_f(groebner_matrix, 41);
    groebner_row37_10000_f(groebner_matrix, 41);
    groebner_row38_00001_f(groebner_matrix, 41);
    groebner_row38_00010_f(groebner_matrix, 41);
    groebner_row39_00010_f(groebner_matrix, 41);
    groebner_row38_00100_f(groebner_matrix, 41);
    groebner_row39_00100_f(groebner_matrix, 41);
    groebner_row40_00100_f(groebner_matrix, 41);
    groebner_row38_01000_f(groebner_matrix, 41);
    groebner_row39_01000_f(groebner_matrix, 41);
    groebner_row40_01000_f(groebner_matrix, 41);
    groebner_row32_00000_f(groebner_matrix, 41);
    groebner_row38_10000_f(groebner_matrix, 41);
    groebner_row39_10000_f(groebner_matrix, 41);
    groebner_row40_10000_f(groebner_matrix, 41);
    groebner_row36_00000_f(groebner_matrix, 41);
    groebner_row37_00000_f(groebner_matrix, 41);
    groebner_row38_00000_f(groebner_matrix, 41);
    groebner_row39_00000_f(groebner_matrix, 41);
    groebner_row40_00000_f(groebner_matrix, 41);
    s_polynomial42(groebner_matrix);
    groebner_row39_02000_f(groebner_matrix, 42);
    groebner_row40_02000_f(groebner_matrix, 42);
    groebner_row41_02000_f(groebner_matrix, 42);
    groebner_row38_11000_f(groebner_matrix, 42);
    groebner_row39_11000_f(groebner_matrix, 42);
    groebner_row40_11000_f(groebner_matrix, 42);
    groebner_row41_11000_f(groebner_matrix, 42);
    groebner_row37_00001_f(groebner_matrix, 42);
    groebner_row37_00010_f(groebner_matrix, 42);
    groebner_row37_00100_f(groebner_matrix, 42);
    groebner_row37_01000_f(groebner_matrix, 42);
}

#[inline]
fn compute_stage_15(groebner_matrix: &mut SMatrix<f64, 44, 80>) {
    groebner_row37_10000_f(groebner_matrix, 42);
    groebner_row38_00001_f(groebner_matrix, 42);
    groebner_row38_00010_f(groebner_matrix, 42);
    groebner_row39_00010_f(groebner_matrix, 42);
    groebner_row38_00100_f(groebner_matrix, 42);
    groebner_row39_00100_f(groebner_matrix, 42);
    groebner_row40_00100_f(groebner_matrix, 42);
    groebner_row38_01000_f(groebner_matrix, 42);
    groebner_row39_01000_f(groebner_matrix, 42);
    groebner_row40_01000_f(groebner_matrix, 42);
    groebner_row41_01000_f(groebner_matrix, 42);
    groebner_row38_10000_f(groebner_matrix, 42);
    groebner_row39_10000_f(groebner_matrix, 42);
    groebner_row40_10000_f(groebner_matrix, 42);
    groebner_row41_10000_f(groebner_matrix, 42);
    groebner_row37_00000_f(groebner_matrix, 42);
    groebner_row38_00000_f(groebner_matrix, 42);
    groebner_row39_00000_f(groebner_matrix, 42);
    groebner_row40_00000_f(groebner_matrix, 42);
    groebner_row41_00000_f(groebner_matrix, 42);
    s_polynomial43(groebner_matrix);
    groebner_row38_11000_f(groebner_matrix, 43);
    groebner_row39_11000_f(groebner_matrix, 43);
    groebner_row40_11000_f(groebner_matrix, 43);
    groebner_row41_11000_f(groebner_matrix, 43);
    groebner_row38_20000_f(groebner_matrix, 43);
    groebner_row39_20000_f(groebner_matrix, 43);
    groebner_row40_20000_f(groebner_matrix, 43);
    groebner_row41_20000_f(groebner_matrix, 43);
    groebner_row42_20000_f(groebner_matrix, 43);
    groebner_row38_01000_f(groebner_matrix, 43);
    groebner_row39_01000_f(groebner_matrix, 43);
    groebner_row40_01000_f(groebner_matrix, 43);
    groebner_row41_01000_f(groebner_matrix, 43);
    groebner_row38_10000_f(groebner_matrix, 43);
    groebner_row39_10000_f(groebner_matrix, 43);
    groebner_row40_10000_f(groebner_matrix, 43);
    groebner_row41_10000_f(groebner_matrix, 43);
    groebner_row42_10000_f(groebner_matrix, 43);
    groebner_row38_00000_f(groebner_matrix, 43);
    groebner_row39_00000_f(groebner_matrix, 43);
    groebner_row40_00000_f(groebner_matrix, 43);
    groebner_row41_00000_f(groebner_matrix, 43);
    groebner_row42_00000_f(groebner_matrix, 43);
}
