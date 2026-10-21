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
        groebner_row9_000000_f, groebner_row10_000000_f, groebner_row10_000001_f,
        groebner_row10_000010_f, groebner_row10_000100_f, groebner_row10_100000_f,
        groebner_row11_000000_f, groebner_row12_000000_f, groebner_row12_000001_f,
        groebner_row12_000010_f, groebner_row12_000100_f, groebner_row12_010000_f,
        groebner_row12_100000_f, groebner_row12_100001_f, groebner_row12_100010_f,
        groebner_row12_100100_f, groebner_row14_000000_f, groebner_row15_000000_f,
        groebner_row15_000001_f, groebner_row15_000010_f, groebner_row15_000100_f,
        groebner_row15_010000_f, groebner_row15_100000_f, groebner_row15_100001_f,
        groebner_row15_100010_f, groebner_row15_100100_f, groebner_row16_000000_f,
        groebner_row16_000001_f, groebner_row16_000010_f, groebner_row16_000100_f,
        groebner_row17_000000_f, groebner_row18_000000_f, groebner_row19_000000_f,
        groebner_row19_000001_f, groebner_row19_000010_f, groebner_row19_000100_f,
        groebner_row20_000000_f, groebner_row21_000000_f, groebner_row22_000000_f,
        groebner_row23_000000_f, groebner_row24_000000_f, groebner_row25_000000_f,
        groebner_row26_000000_f, groebner_row27_000000_f, groebner_row28_000000_f,
        groebner_row29_000000_f, groebner_row30_000000_f, groebner_row30_000100_f,
        groebner_row30_100000_f, groebner_row31_000000_f, groebner_row31_000100_f,
        groebner_row31_100000_f, groebner_row32_000000_f, groebner_row32_000100_f,
        groebner_row32_100000_f, groebner_row33_000000_f, groebner_row33_000001_f,
        groebner_row33_000010_f, groebner_row33_000100_f, groebner_row33_100000_f,
        groebner_row34_000000_f, groebner_row34_000010_f, groebner_row34_000100_f,
        groebner_row34_100000_f, groebner_row35_000000_f, groebner_row35_000100_f,
        groebner_row35_100000_f, groebner_row36_000000_f, groebner_row36_000001_f,
        groebner_row36_000010_f, groebner_row36_000100_f, groebner_row36_010000_f,
        groebner_row37_000000_f, groebner_row37_000001_f, groebner_row37_000010_f,
        groebner_row37_000100_f, groebner_row37_100000_f, groebner_row38_000000_f,
        groebner_row38_000010_f, groebner_row38_000100_f, groebner_row38_100000_f,
        groebner_row39_000000_f, groebner_row39_000100_f, groebner_row39_100000_f,
        groebner_row40_000000_f, groebner_row40_100000_f, groebner_row41_000000_f,
        groebner_row41_100000_f, groebner_row42_000000_f, groebner_row43_000000_f,
        groebner_row44_000000_f, groebner_row45_000000_f, groebner_row46_000000_f,
    },
    spolynomials::{
        s_polynomial9, s_polynomial10, s_polynomial11, s_polynomial12, s_polynomial13,
        s_polynomial14, s_polynomial15, s_polynomial16, s_polynomial17, s_polynomial18,
        s_polynomial19, s_polynomial20, s_polynomial21, s_polynomial22, s_polynomial23,
        s_polynomial24, s_polynomial25, s_polynomial26, s_polynomial27, s_polynomial28,
        s_polynomial29, s_polynomial30, s_polynomial31, s_polynomial32, s_polynomial33,
        s_polynomial34, s_polynomial35, s_polynomial36, s_polynomial37, s_polynomial38,
        s_polynomial39, s_polynomial40, s_polynomial41, s_polynomial42, s_polynomial43,
        s_polynomial44, s_polynomial45, s_polynomial46, s_polynomial47,
    },
};
use nalgebra::SMatrix;

pub fn compute(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
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
}
#[inline]
fn compute_stage_0(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    s_polynomial9(groebner_matrix);
    s_polynomial10(groebner_matrix);
    groebner_row9_000000_f(groebner_matrix, 10);
    s_polynomial11(groebner_matrix);
    groebner_row10_000010_f(groebner_matrix, 11);
    groebner_row10_000000_f(groebner_matrix, 11);
    s_polynomial12(groebner_matrix);
    groebner_row10_000010_f(groebner_matrix, 12);
    groebner_row11_000000_f(groebner_matrix, 12);
    groebner_row10_000000_f(groebner_matrix, 12);
    s_polynomial13(groebner_matrix);
    groebner_row10_000010_f(groebner_matrix, 13);
    groebner_row10_000100_f(groebner_matrix, 13);
    groebner_row12_000010_f(groebner_matrix, 13);
    groebner_row12_000100_f(groebner_matrix, 13);
    groebner_row10_000000_f(groebner_matrix, 13);
    groebner_row12_000000_f(groebner_matrix, 13);
    s_polynomial14(groebner_matrix);
    groebner_row10_000100_f(groebner_matrix, 14);
    groebner_row12_000100_f(groebner_matrix, 14);
    groebner_row10_000000_f(groebner_matrix, 14);
    groebner_row12_000000_f(groebner_matrix, 14);
    s_polynomial15(groebner_matrix);
    groebner_row10_000100_f(groebner_matrix, 15);
    groebner_row12_000100_f(groebner_matrix, 15);
    groebner_row14_000000_f(groebner_matrix, 15);
    groebner_row10_000000_f(groebner_matrix, 15);
    groebner_row12_000000_f(groebner_matrix, 15);
    s_polynomial16(groebner_matrix);
    groebner_row15_010000_f(groebner_matrix, 16);
    groebner_row10_100000_f(groebner_matrix, 16);
    groebner_row12_100000_f(groebner_matrix, 16);
    groebner_row15_100000_f(groebner_matrix, 16);
    groebner_row10_000000_f(groebner_matrix, 16);
    groebner_row12_000000_f(groebner_matrix, 16);
    groebner_row15_000000_f(groebner_matrix, 16);
    s_polynomial17(groebner_matrix);
    groebner_row15_000000_f(groebner_matrix, 17);
    s_polynomial18(groebner_matrix);
    groebner_row15_000100_f(groebner_matrix, 18);
    groebner_row17_000000_f(groebner_matrix, 18);
    groebner_row15_000000_f(groebner_matrix, 18);
    s_polynomial19(groebner_matrix);
    groebner_row12_010000_f(groebner_matrix, 19);
    groebner_row15_010000_f(groebner_matrix, 19);
    groebner_row10_100000_f(groebner_matrix, 19);
    groebner_row12_100000_f(groebner_matrix, 19);
    groebner_row15_100000_f(groebner_matrix, 19);
    groebner_row16_000000_f(groebner_matrix, 19);
    groebner_row10_000000_f(groebner_matrix, 19);
    groebner_row12_000000_f(groebner_matrix, 19);
    groebner_row15_000000_f(groebner_matrix, 19);
    s_polynomial20(groebner_matrix);
    groebner_row10_000100_f(groebner_matrix, 20);
    groebner_row12_000001_f(groebner_matrix, 20);
    groebner_row12_000100_f(groebner_matrix, 20);
    groebner_row15_000001_f(groebner_matrix, 20);
    groebner_row15_000100_f(groebner_matrix, 20);
    groebner_row17_000000_f(groebner_matrix, 20);
    groebner_row10_000000_f(groebner_matrix, 20);
    groebner_row12_000000_f(groebner_matrix, 20);
    groebner_row15_000000_f(groebner_matrix, 20);
}

#[inline]
fn compute_stage_1(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    s_polynomial21(groebner_matrix);
    groebner_row19_000100_f(groebner_matrix, 21);
    groebner_row17_000000_f(groebner_matrix, 21);
    groebner_row16_000000_f(groebner_matrix, 21);
    groebner_row19_000000_f(groebner_matrix, 21);
    s_polynomial22(groebner_matrix);
    groebner_row19_000010_f(groebner_matrix, 22);
    groebner_row18_000000_f(groebner_matrix, 22);
    groebner_row16_000000_f(groebner_matrix, 22);
    groebner_row19_000000_f(groebner_matrix, 22);
    s_polynomial23(groebner_matrix);
    groebner_row19_000001_f(groebner_matrix, 23);
    groebner_row20_000000_f(groebner_matrix, 23);
    groebner_row16_000000_f(groebner_matrix, 23);
    groebner_row19_000000_f(groebner_matrix, 23);
    s_polynomial24(groebner_matrix);
    groebner_row15_100100_f(groebner_matrix, 24);
    groebner_row16_000100_f(groebner_matrix, 24);
    groebner_row19_000100_f(groebner_matrix, 24);
    groebner_row21_000000_f(groebner_matrix, 24);
    groebner_row12_000100_f(groebner_matrix, 24);
    groebner_row15_000100_f(groebner_matrix, 24);
    groebner_row17_000000_f(groebner_matrix, 24);
    groebner_row12_100000_f(groebner_matrix, 24);
    groebner_row15_100000_f(groebner_matrix, 24);
    groebner_row16_000000_f(groebner_matrix, 24);
    groebner_row19_000000_f(groebner_matrix, 24);
    groebner_row12_000000_f(groebner_matrix, 24);
    groebner_row15_000000_f(groebner_matrix, 24);
    s_polynomial25(groebner_matrix);
    groebner_row15_100010_f(groebner_matrix, 25);
    groebner_row16_000010_f(groebner_matrix, 25);
    groebner_row19_000010_f(groebner_matrix, 25);
    groebner_row22_000000_f(groebner_matrix, 25);
    groebner_row12_000010_f(groebner_matrix, 25);
    groebner_row15_000010_f(groebner_matrix, 25);
    groebner_row18_000000_f(groebner_matrix, 25);
    groebner_row12_100000_f(groebner_matrix, 25);
    groebner_row15_100000_f(groebner_matrix, 25);
    groebner_row16_000000_f(groebner_matrix, 25);
    groebner_row19_000000_f(groebner_matrix, 25);
    groebner_row12_000000_f(groebner_matrix, 25);
    groebner_row15_000000_f(groebner_matrix, 25);
    s_polynomial26(groebner_matrix);
    groebner_row15_100001_f(groebner_matrix, 26);
    groebner_row16_000001_f(groebner_matrix, 26);
    groebner_row19_000001_f(groebner_matrix, 26);
    groebner_row23_000000_f(groebner_matrix, 26);
    groebner_row12_000001_f(groebner_matrix, 26);
    groebner_row15_000001_f(groebner_matrix, 26);
    groebner_row20_000000_f(groebner_matrix, 26);
    groebner_row12_100000_f(groebner_matrix, 26);
    groebner_row15_100000_f(groebner_matrix, 26);
    groebner_row16_000000_f(groebner_matrix, 26);
    groebner_row19_000000_f(groebner_matrix, 26);
    groebner_row12_000000_f(groebner_matrix, 26);
    groebner_row15_000000_f(groebner_matrix, 26);
    s_polynomial27(groebner_matrix);
    groebner_row12_100100_f(groebner_matrix, 27);
    groebner_row15_100100_f(groebner_matrix, 27);
    groebner_row16_000100_f(groebner_matrix, 27);
    groebner_row19_000100_f(groebner_matrix, 27);
}

#[inline]
fn compute_stage_2(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row21_000000_f(groebner_matrix, 27);
    groebner_row10_000100_f(groebner_matrix, 27);
    groebner_row12_000100_f(groebner_matrix, 27);
    groebner_row15_000100_f(groebner_matrix, 27);
    groebner_row17_000000_f(groebner_matrix, 27);
    groebner_row10_100000_f(groebner_matrix, 27);
    groebner_row24_000000_f(groebner_matrix, 27);
    groebner_row12_100000_f(groebner_matrix, 27);
    groebner_row15_100000_f(groebner_matrix, 27);
    groebner_row16_000000_f(groebner_matrix, 27);
    groebner_row19_000000_f(groebner_matrix, 27);
    groebner_row10_000000_f(groebner_matrix, 27);
    groebner_row12_000000_f(groebner_matrix, 27);
    groebner_row15_000000_f(groebner_matrix, 27);
    s_polynomial28(groebner_matrix);
    groebner_row12_100010_f(groebner_matrix, 28);
    groebner_row15_100010_f(groebner_matrix, 28);
    groebner_row16_000010_f(groebner_matrix, 28);
    groebner_row19_000010_f(groebner_matrix, 28);
    groebner_row22_000000_f(groebner_matrix, 28);
    groebner_row10_000010_f(groebner_matrix, 28);
    groebner_row12_000010_f(groebner_matrix, 28);
    groebner_row15_000010_f(groebner_matrix, 28);
    groebner_row18_000000_f(groebner_matrix, 28);
    groebner_row10_100000_f(groebner_matrix, 28);
    groebner_row25_000000_f(groebner_matrix, 28);
    groebner_row12_100000_f(groebner_matrix, 28);
    groebner_row15_100000_f(groebner_matrix, 28);
    groebner_row16_000000_f(groebner_matrix, 28);
    groebner_row19_000000_f(groebner_matrix, 28);
    groebner_row10_000000_f(groebner_matrix, 28);
    groebner_row12_000000_f(groebner_matrix, 28);
    groebner_row15_000000_f(groebner_matrix, 28);
    s_polynomial29(groebner_matrix);
    groebner_row12_100001_f(groebner_matrix, 29);
    groebner_row15_100001_f(groebner_matrix, 29);
    groebner_row16_000001_f(groebner_matrix, 29);
    groebner_row19_000001_f(groebner_matrix, 29);
    groebner_row23_000000_f(groebner_matrix, 29);
    groebner_row10_000001_f(groebner_matrix, 29);
    groebner_row12_000001_f(groebner_matrix, 29);
    groebner_row15_000001_f(groebner_matrix, 29);
    groebner_row20_000000_f(groebner_matrix, 29);
    groebner_row10_100000_f(groebner_matrix, 29);
    groebner_row26_000000_f(groebner_matrix, 29);
    groebner_row12_100000_f(groebner_matrix, 29);
    groebner_row15_100000_f(groebner_matrix, 29);
    groebner_row16_000000_f(groebner_matrix, 29);
    groebner_row19_000000_f(groebner_matrix, 29);
    groebner_row10_000000_f(groebner_matrix, 29);
    groebner_row12_000000_f(groebner_matrix, 29);
    groebner_row15_000000_f(groebner_matrix, 29);
    s_polynomial30(groebner_matrix);
    groebner_row18_000000_f(groebner_matrix, 30);
    groebner_row17_000000_f(groebner_matrix, 30);
    groebner_row25_000000_f(groebner_matrix, 30);
    groebner_row24_000000_f(groebner_matrix, 30);
    groebner_row28_000000_f(groebner_matrix, 30);
    groebner_row27_000000_f(groebner_matrix, 30);
    s_polynomial31(groebner_matrix);
    groebner_row20_000000_f(groebner_matrix, 31);
    groebner_row17_000000_f(groebner_matrix, 31);
}

#[inline]
fn compute_stage_3(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row26_000000_f(groebner_matrix, 31);
    groebner_row24_000000_f(groebner_matrix, 31);
    groebner_row29_000000_f(groebner_matrix, 31);
    groebner_row27_000000_f(groebner_matrix, 31);
    s_polynomial32(groebner_matrix);
    groebner_row31_100000_f(groebner_matrix, 32);
    groebner_row30_100000_f(groebner_matrix, 32);
    groebner_row17_000000_f(groebner_matrix, 32);
    groebner_row26_000000_f(groebner_matrix, 32);
    groebner_row25_000000_f(groebner_matrix, 32);
    groebner_row24_000000_f(groebner_matrix, 32);
    groebner_row29_000000_f(groebner_matrix, 32);
    groebner_row28_000000_f(groebner_matrix, 32);
    groebner_row27_000000_f(groebner_matrix, 32);
    groebner_row16_000000_f(groebner_matrix, 32);
    groebner_row19_000000_f(groebner_matrix, 32);
    groebner_row31_000000_f(groebner_matrix, 32);
    groebner_row30_000000_f(groebner_matrix, 32);
    s_polynomial33(groebner_matrix);
    groebner_row26_000000_f(groebner_matrix, 33);
    groebner_row25_000000_f(groebner_matrix, 33);
    groebner_row24_000000_f(groebner_matrix, 33);
    groebner_row29_000000_f(groebner_matrix, 33);
    groebner_row28_000000_f(groebner_matrix, 33);
    groebner_row27_000000_f(groebner_matrix, 33);
    groebner_row16_000000_f(groebner_matrix, 33);
    groebner_row19_000000_f(groebner_matrix, 33);
    groebner_row31_000000_f(groebner_matrix, 33);
    groebner_row30_000000_f(groebner_matrix, 33);
    groebner_row32_000000_f(groebner_matrix, 33);
    s_polynomial34(groebner_matrix);
    groebner_row32_100000_f(groebner_matrix, 34);
    groebner_row33_100000_f(groebner_matrix, 34);
    groebner_row25_000000_f(groebner_matrix, 34);
    groebner_row24_000000_f(groebner_matrix, 34);
    groebner_row29_000000_f(groebner_matrix, 34);
    groebner_row28_000000_f(groebner_matrix, 34);
    groebner_row27_000000_f(groebner_matrix, 34);
    groebner_row16_000000_f(groebner_matrix, 34);
    groebner_row19_000000_f(groebner_matrix, 34);
    groebner_row31_000000_f(groebner_matrix, 34);
    groebner_row30_000000_f(groebner_matrix, 34);
    groebner_row32_000000_f(groebner_matrix, 34);
    groebner_row33_000000_f(groebner_matrix, 34);
    s_polynomial35(groebner_matrix);
    groebner_row32_100000_f(groebner_matrix, 35);
    groebner_row33_100000_f(groebner_matrix, 35);
    groebner_row34_100000_f(groebner_matrix, 35);
    groebner_row24_000000_f(groebner_matrix, 35);
    groebner_row29_000000_f(groebner_matrix, 35);
    groebner_row28_000000_f(groebner_matrix, 35);
    groebner_row27_000000_f(groebner_matrix, 35);
    groebner_row16_000000_f(groebner_matrix, 35);
    groebner_row19_000000_f(groebner_matrix, 35);
    groebner_row31_000000_f(groebner_matrix, 35);
    groebner_row30_000000_f(groebner_matrix, 35);
    groebner_row32_000000_f(groebner_matrix, 35);
    groebner_row33_000000_f(groebner_matrix, 35);
    groebner_row34_000000_f(groebner_matrix, 35);
    s_polynomial36(groebner_matrix);
    groebner_row15_000001_f(groebner_matrix, 36);
    groebner_row15_000010_f(groebner_matrix, 36);
}

#[inline]
fn compute_stage_4(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row15_000100_f(groebner_matrix, 36);
    groebner_row31_100000_f(groebner_matrix, 36);
    groebner_row30_100000_f(groebner_matrix, 36);
    groebner_row32_100000_f(groebner_matrix, 36);
    groebner_row33_100000_f(groebner_matrix, 36);
    groebner_row34_100000_f(groebner_matrix, 36);
    groebner_row35_100000_f(groebner_matrix, 36);
    groebner_row12_100000_f(groebner_matrix, 36);
    groebner_row15_100000_f(groebner_matrix, 36);
    groebner_row29_000000_f(groebner_matrix, 36);
    groebner_row28_000000_f(groebner_matrix, 36);
    groebner_row27_000000_f(groebner_matrix, 36);
    groebner_row16_000000_f(groebner_matrix, 36);
    groebner_row19_000000_f(groebner_matrix, 36);
    groebner_row31_000000_f(groebner_matrix, 36);
    groebner_row30_000000_f(groebner_matrix, 36);
    groebner_row32_000000_f(groebner_matrix, 36);
    groebner_row33_000000_f(groebner_matrix, 36);
    groebner_row34_000000_f(groebner_matrix, 36);
    groebner_row35_000000_f(groebner_matrix, 36);
    groebner_row12_000000_f(groebner_matrix, 36);
    groebner_row15_000000_f(groebner_matrix, 36);
    s_polynomial37(groebner_matrix);
    groebner_row12_000100_f(groebner_matrix, 37);
    groebner_row15_000010_f(groebner_matrix, 37);
    groebner_row15_000100_f(groebner_matrix, 37);
    groebner_row30_100000_f(groebner_matrix, 37);
    groebner_row32_100000_f(groebner_matrix, 37);
    groebner_row33_100000_f(groebner_matrix, 37);
    groebner_row34_100000_f(groebner_matrix, 37);
    groebner_row35_100000_f(groebner_matrix, 37);
    groebner_row12_100000_f(groebner_matrix, 37);
    groebner_row15_100000_f(groebner_matrix, 37);
    groebner_row29_000000_f(groebner_matrix, 37);
    groebner_row28_000000_f(groebner_matrix, 37);
    groebner_row27_000000_f(groebner_matrix, 37);
    groebner_row16_000000_f(groebner_matrix, 37);
    groebner_row19_000000_f(groebner_matrix, 37);
    groebner_row31_000000_f(groebner_matrix, 37);
    groebner_row30_000000_f(groebner_matrix, 37);
    groebner_row32_000000_f(groebner_matrix, 37);
    groebner_row33_000000_f(groebner_matrix, 37);
    groebner_row34_000000_f(groebner_matrix, 37);
    groebner_row35_000000_f(groebner_matrix, 37);
    groebner_row12_000000_f(groebner_matrix, 37);
    groebner_row15_000000_f(groebner_matrix, 37);
    s_polynomial38(groebner_matrix);
    groebner_row12_000100_f(groebner_matrix, 38);
    groebner_row15_000001_f(groebner_matrix, 38);
    groebner_row15_000100_f(groebner_matrix, 38);
    groebner_row31_100000_f(groebner_matrix, 38);
    groebner_row32_100000_f(groebner_matrix, 38);
    groebner_row33_100000_f(groebner_matrix, 38);
    groebner_row34_100000_f(groebner_matrix, 38);
    groebner_row35_100000_f(groebner_matrix, 38);
    groebner_row12_100000_f(groebner_matrix, 38);
    groebner_row15_100000_f(groebner_matrix, 38);
    groebner_row37_100000_f(groebner_matrix, 38);
    groebner_row28_000000_f(groebner_matrix, 38);
    groebner_row27_000000_f(groebner_matrix, 38);
    groebner_row16_000000_f(groebner_matrix, 38);
    groebner_row19_000000_f(groebner_matrix, 38);
}

#[inline]
fn compute_stage_5(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row36_000000_f(groebner_matrix, 38);
    groebner_row31_000000_f(groebner_matrix, 38);
    groebner_row30_000000_f(groebner_matrix, 38);
    groebner_row32_000000_f(groebner_matrix, 38);
    groebner_row33_000000_f(groebner_matrix, 38);
    groebner_row34_000000_f(groebner_matrix, 38);
    groebner_row35_000000_f(groebner_matrix, 38);
    groebner_row12_000000_f(groebner_matrix, 38);
    groebner_row15_000000_f(groebner_matrix, 38);
    groebner_row37_000000_f(groebner_matrix, 38);
    s_polynomial39(groebner_matrix);
    groebner_row12_000001_f(groebner_matrix, 39);
    groebner_row12_000010_f(groebner_matrix, 39);
    groebner_row12_000100_f(groebner_matrix, 39);
    groebner_row15_000001_f(groebner_matrix, 39);
    groebner_row15_000010_f(groebner_matrix, 39);
    groebner_row15_000100_f(groebner_matrix, 39);
    groebner_row31_100000_f(groebner_matrix, 39);
    groebner_row30_100000_f(groebner_matrix, 39);
    groebner_row32_100000_f(groebner_matrix, 39);
    groebner_row10_100000_f(groebner_matrix, 39);
    groebner_row33_100000_f(groebner_matrix, 39);
    groebner_row34_100000_f(groebner_matrix, 39);
    groebner_row35_100000_f(groebner_matrix, 39);
    groebner_row12_100000_f(groebner_matrix, 39);
    groebner_row15_100000_f(groebner_matrix, 39);
    groebner_row37_100000_f(groebner_matrix, 39);
    groebner_row38_100000_f(groebner_matrix, 39);
    groebner_row27_000000_f(groebner_matrix, 39);
    groebner_row16_000000_f(groebner_matrix, 39);
    groebner_row19_000000_f(groebner_matrix, 39);
    groebner_row36_000000_f(groebner_matrix, 39);
    groebner_row31_000000_f(groebner_matrix, 39);
    groebner_row30_000000_f(groebner_matrix, 39);
    groebner_row32_000000_f(groebner_matrix, 39);
    groebner_row10_000000_f(groebner_matrix, 39);
    groebner_row33_000000_f(groebner_matrix, 39);
    groebner_row34_000000_f(groebner_matrix, 39);
    groebner_row35_000000_f(groebner_matrix, 39);
    groebner_row12_000000_f(groebner_matrix, 39);
    groebner_row15_000000_f(groebner_matrix, 39);
    groebner_row37_000000_f(groebner_matrix, 39);
    groebner_row38_000000_f(groebner_matrix, 39);
    s_polynomial40(groebner_matrix);
    groebner_row10_000100_f(groebner_matrix, 40);
    groebner_row12_000010_f(groebner_matrix, 40);
    groebner_row12_000100_f(groebner_matrix, 40);
    groebner_row15_000010_f(groebner_matrix, 40);
    groebner_row15_000100_f(groebner_matrix, 40);
    groebner_row30_100000_f(groebner_matrix, 40);
    groebner_row32_100000_f(groebner_matrix, 40);
    groebner_row10_100000_f(groebner_matrix, 40);
    groebner_row33_100000_f(groebner_matrix, 40);
    groebner_row34_100000_f(groebner_matrix, 40);
    groebner_row35_100000_f(groebner_matrix, 40);
    groebner_row12_100000_f(groebner_matrix, 40);
    groebner_row15_100000_f(groebner_matrix, 40);
    groebner_row37_100000_f(groebner_matrix, 40);
    groebner_row38_100000_f(groebner_matrix, 40);
    groebner_row39_100000_f(groebner_matrix, 40);
    groebner_row16_000000_f(groebner_matrix, 40);
    groebner_row19_000000_f(groebner_matrix, 40);
}

#[inline]
fn compute_stage_6(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row36_000000_f(groebner_matrix, 40);
    groebner_row30_000000_f(groebner_matrix, 40);
    groebner_row32_000000_f(groebner_matrix, 40);
    groebner_row10_000000_f(groebner_matrix, 40);
    groebner_row33_000000_f(groebner_matrix, 40);
    groebner_row34_000000_f(groebner_matrix, 40);
    groebner_row35_000000_f(groebner_matrix, 40);
    groebner_row12_000000_f(groebner_matrix, 40);
    groebner_row15_000000_f(groebner_matrix, 40);
    groebner_row37_000000_f(groebner_matrix, 40);
    groebner_row38_000000_f(groebner_matrix, 40);
    groebner_row39_000000_f(groebner_matrix, 40);
    s_polynomial41(groebner_matrix);
    groebner_row10_000100_f(groebner_matrix, 41);
    groebner_row12_000001_f(groebner_matrix, 41);
    groebner_row12_000100_f(groebner_matrix, 41);
    groebner_row15_000001_f(groebner_matrix, 41);
    groebner_row15_000100_f(groebner_matrix, 41);
    groebner_row31_100000_f(groebner_matrix, 41);
    groebner_row32_100000_f(groebner_matrix, 41);
    groebner_row10_100000_f(groebner_matrix, 41);
    groebner_row33_100000_f(groebner_matrix, 41);
    groebner_row34_100000_f(groebner_matrix, 41);
    groebner_row35_100000_f(groebner_matrix, 41);
    groebner_row12_100000_f(groebner_matrix, 41);
    groebner_row15_100000_f(groebner_matrix, 41);
    groebner_row37_100000_f(groebner_matrix, 41);
    groebner_row38_100000_f(groebner_matrix, 41);
    groebner_row39_100000_f(groebner_matrix, 41);
    groebner_row40_100000_f(groebner_matrix, 41);
    groebner_row19_000000_f(groebner_matrix, 41);
    groebner_row36_000000_f(groebner_matrix, 41);
    groebner_row31_000000_f(groebner_matrix, 41);
    groebner_row32_000000_f(groebner_matrix, 41);
    groebner_row10_000000_f(groebner_matrix, 41);
    groebner_row33_000000_f(groebner_matrix, 41);
    groebner_row34_000000_f(groebner_matrix, 41);
    groebner_row35_000000_f(groebner_matrix, 41);
    groebner_row12_000000_f(groebner_matrix, 41);
    groebner_row15_000000_f(groebner_matrix, 41);
    groebner_row37_000000_f(groebner_matrix, 41);
    groebner_row38_000000_f(groebner_matrix, 41);
    groebner_row39_000000_f(groebner_matrix, 41);
    groebner_row40_000000_f(groebner_matrix, 41);
    s_polynomial42(groebner_matrix);
    groebner_row32_000100_f(groebner_matrix, 42);
    groebner_row33_000010_f(groebner_matrix, 42);
    groebner_row34_000010_f(groebner_matrix, 42);
    groebner_row33_000100_f(groebner_matrix, 42);
    groebner_row34_000100_f(groebner_matrix, 42);
    groebner_row35_000100_f(groebner_matrix, 42);
    groebner_row37_000010_f(groebner_matrix, 42);
    groebner_row38_000010_f(groebner_matrix, 42);
    groebner_row37_000100_f(groebner_matrix, 42);
    groebner_row38_000100_f(groebner_matrix, 42);
    groebner_row39_000100_f(groebner_matrix, 42);
    groebner_row30_100000_f(groebner_matrix, 42);
    groebner_row32_100000_f(groebner_matrix, 42);
    groebner_row33_100000_f(groebner_matrix, 42);
    groebner_row34_100000_f(groebner_matrix, 42);
    groebner_row35_100000_f(groebner_matrix, 42);
    groebner_row37_100000_f(groebner_matrix, 42);
}

#[inline]
fn compute_stage_7(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row38_100000_f(groebner_matrix, 42);
    groebner_row39_100000_f(groebner_matrix, 42);
    groebner_row40_100000_f(groebner_matrix, 42);
    groebner_row41_100000_f(groebner_matrix, 42);
    groebner_row36_000000_f(groebner_matrix, 42);
    groebner_row30_000000_f(groebner_matrix, 42);
    groebner_row32_000000_f(groebner_matrix, 42);
    groebner_row33_000000_f(groebner_matrix, 42);
    groebner_row34_000000_f(groebner_matrix, 42);
    groebner_row35_000000_f(groebner_matrix, 42);
    groebner_row37_000000_f(groebner_matrix, 42);
    groebner_row38_000000_f(groebner_matrix, 42);
    groebner_row39_000000_f(groebner_matrix, 42);
    groebner_row40_000000_f(groebner_matrix, 42);
    groebner_row41_000000_f(groebner_matrix, 42);
    s_polynomial43(groebner_matrix);
    groebner_row32_000100_f(groebner_matrix, 43);
    groebner_row33_000001_f(groebner_matrix, 43);
    groebner_row33_000010_f(groebner_matrix, 43);
    groebner_row34_000010_f(groebner_matrix, 43);
    groebner_row33_000100_f(groebner_matrix, 43);
    groebner_row34_000100_f(groebner_matrix, 43);
    groebner_row35_000100_f(groebner_matrix, 43);
    groebner_row37_000001_f(groebner_matrix, 43);
    groebner_row37_000010_f(groebner_matrix, 43);
    groebner_row38_000010_f(groebner_matrix, 43);
    groebner_row37_000100_f(groebner_matrix, 43);
    groebner_row38_000100_f(groebner_matrix, 43);
    groebner_row39_000100_f(groebner_matrix, 43);
    groebner_row31_100000_f(groebner_matrix, 43);
    groebner_row30_100000_f(groebner_matrix, 43);
    groebner_row32_100000_f(groebner_matrix, 43);
    groebner_row33_100000_f(groebner_matrix, 43);
    groebner_row34_100000_f(groebner_matrix, 43);
    groebner_row35_100000_f(groebner_matrix, 43);
    groebner_row37_100000_f(groebner_matrix, 43);
    groebner_row38_100000_f(groebner_matrix, 43);
    groebner_row39_100000_f(groebner_matrix, 43);
    groebner_row40_100000_f(groebner_matrix, 43);
    groebner_row41_100000_f(groebner_matrix, 43);
    groebner_row36_000000_f(groebner_matrix, 43);
    groebner_row42_000000_f(groebner_matrix, 43);
    groebner_row31_000000_f(groebner_matrix, 43);
    groebner_row30_000000_f(groebner_matrix, 43);
    groebner_row32_000000_f(groebner_matrix, 43);
    groebner_row33_000000_f(groebner_matrix, 43);
    groebner_row34_000000_f(groebner_matrix, 43);
    groebner_row35_000000_f(groebner_matrix, 43);
    groebner_row37_000000_f(groebner_matrix, 43);
    groebner_row38_000000_f(groebner_matrix, 43);
    groebner_row39_000000_f(groebner_matrix, 43);
    groebner_row40_000000_f(groebner_matrix, 43);
    groebner_row41_000000_f(groebner_matrix, 43);
    s_polynomial44(groebner_matrix);
    groebner_row31_000100_f(groebner_matrix, 44);
    groebner_row30_000100_f(groebner_matrix, 44);
    groebner_row33_000100_f(groebner_matrix, 44);
    groebner_row34_000100_f(groebner_matrix, 44);
    groebner_row35_000100_f(groebner_matrix, 44);
    groebner_row37_000100_f(groebner_matrix, 44);
    groebner_row38_000100_f(groebner_matrix, 44);
    groebner_row39_000100_f(groebner_matrix, 44);
}

#[inline]
fn compute_stage_8(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row31_100000_f(groebner_matrix, 44);
    groebner_row30_100000_f(groebner_matrix, 44);
    groebner_row32_100000_f(groebner_matrix, 44);
    groebner_row33_100000_f(groebner_matrix, 44);
    groebner_row34_100000_f(groebner_matrix, 44);
    groebner_row35_100000_f(groebner_matrix, 44);
    groebner_row37_100000_f(groebner_matrix, 44);
    groebner_row38_100000_f(groebner_matrix, 44);
    groebner_row39_100000_f(groebner_matrix, 44);
    groebner_row40_100000_f(groebner_matrix, 44);
    groebner_row41_100000_f(groebner_matrix, 44);
    groebner_row36_000000_f(groebner_matrix, 44);
    groebner_row31_000000_f(groebner_matrix, 44);
    groebner_row30_000000_f(groebner_matrix, 44);
    groebner_row32_000000_f(groebner_matrix, 44);
    groebner_row33_000000_f(groebner_matrix, 44);
    groebner_row34_000000_f(groebner_matrix, 44);
    groebner_row35_000000_f(groebner_matrix, 44);
    groebner_row37_000000_f(groebner_matrix, 44);
    groebner_row38_000000_f(groebner_matrix, 44);
    groebner_row39_000000_f(groebner_matrix, 44);
    groebner_row40_000000_f(groebner_matrix, 44);
    groebner_row41_000000_f(groebner_matrix, 44);
    s_polynomial45(groebner_matrix);
    groebner_row36_000100_f(groebner_matrix, 45);
    groebner_row36_010000_f(groebner_matrix, 45);
    groebner_row12_000100_f(groebner_matrix, 45);
    groebner_row15_000100_f(groebner_matrix, 45);
    groebner_row37_000100_f(groebner_matrix, 45);
    groebner_row38_000100_f(groebner_matrix, 45);
    groebner_row39_000100_f(groebner_matrix, 45);
    groebner_row32_100000_f(groebner_matrix, 45);
    groebner_row33_100000_f(groebner_matrix, 45);
    groebner_row34_100000_f(groebner_matrix, 45);
    groebner_row35_100000_f(groebner_matrix, 45);
    groebner_row12_100000_f(groebner_matrix, 45);
    groebner_row15_100000_f(groebner_matrix, 45);
    groebner_row37_100000_f(groebner_matrix, 45);
    groebner_row38_100000_f(groebner_matrix, 45);
    groebner_row39_100000_f(groebner_matrix, 45);
    groebner_row40_100000_f(groebner_matrix, 45);
    groebner_row41_100000_f(groebner_matrix, 45);
    groebner_row36_000000_f(groebner_matrix, 45);
    groebner_row44_000000_f(groebner_matrix, 45);
    groebner_row32_000000_f(groebner_matrix, 45);
    groebner_row33_000000_f(groebner_matrix, 45);
    groebner_row34_000000_f(groebner_matrix, 45);
    groebner_row35_000000_f(groebner_matrix, 45);
    groebner_row12_000000_f(groebner_matrix, 45);
    groebner_row15_000000_f(groebner_matrix, 45);
    groebner_row37_000000_f(groebner_matrix, 45);
    groebner_row38_000000_f(groebner_matrix, 45);
    groebner_row39_000000_f(groebner_matrix, 45);
    groebner_row40_000000_f(groebner_matrix, 45);
    groebner_row41_000000_f(groebner_matrix, 45);
    s_polynomial46(groebner_matrix);
    groebner_row36_000010_f(groebner_matrix, 46);
    groebner_row36_010000_f(groebner_matrix, 46);
    groebner_row12_000010_f(groebner_matrix, 46);
    groebner_row15_000010_f(groebner_matrix, 46);
    groebner_row37_000010_f(groebner_matrix, 46);
    groebner_row38_000010_f(groebner_matrix, 46);
}

#[inline]
fn compute_stage_9(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row38_000100_f(groebner_matrix, 46);
    groebner_row39_000100_f(groebner_matrix, 46);
    groebner_row30_100000_f(groebner_matrix, 46);
    groebner_row32_100000_f(groebner_matrix, 46);
    groebner_row33_100000_f(groebner_matrix, 46);
    groebner_row34_100000_f(groebner_matrix, 46);
    groebner_row35_100000_f(groebner_matrix, 46);
    groebner_row12_100000_f(groebner_matrix, 46);
    groebner_row15_100000_f(groebner_matrix, 46);
    groebner_row37_100000_f(groebner_matrix, 46);
    groebner_row38_100000_f(groebner_matrix, 46);
    groebner_row39_100000_f(groebner_matrix, 46);
    groebner_row40_100000_f(groebner_matrix, 46);
    groebner_row41_100000_f(groebner_matrix, 46);
    groebner_row36_000000_f(groebner_matrix, 46);
    groebner_row42_000000_f(groebner_matrix, 46);
    groebner_row44_000000_f(groebner_matrix, 46);
    groebner_row45_000000_f(groebner_matrix, 46);
    groebner_row30_000000_f(groebner_matrix, 46);
    groebner_row32_000000_f(groebner_matrix, 46);
    groebner_row33_000000_f(groebner_matrix, 46);
    groebner_row34_000000_f(groebner_matrix, 46);
    groebner_row35_000000_f(groebner_matrix, 46);
    groebner_row12_000000_f(groebner_matrix, 46);
    groebner_row15_000000_f(groebner_matrix, 46);
    groebner_row37_000000_f(groebner_matrix, 46);
    groebner_row38_000000_f(groebner_matrix, 46);
    groebner_row39_000000_f(groebner_matrix, 46);
    groebner_row40_000000_f(groebner_matrix, 46);
    groebner_row41_000000_f(groebner_matrix, 46);
    s_polynomial47(groebner_matrix);
    groebner_row36_000001_f(groebner_matrix, 47);
    groebner_row36_010000_f(groebner_matrix, 47);
    groebner_row12_000001_f(groebner_matrix, 47);
    groebner_row15_000001_f(groebner_matrix, 47);
    groebner_row37_000001_f(groebner_matrix, 47);
    groebner_row37_000010_f(groebner_matrix, 47);
    groebner_row38_000010_f(groebner_matrix, 47);
    groebner_row37_000100_f(groebner_matrix, 47);
    groebner_row38_000100_f(groebner_matrix, 47);
    groebner_row39_000100_f(groebner_matrix, 47);
    groebner_row31_100000_f(groebner_matrix, 47);
    groebner_row30_100000_f(groebner_matrix, 47);
    groebner_row32_100000_f(groebner_matrix, 47);
    groebner_row33_100000_f(groebner_matrix, 47);
    groebner_row34_100000_f(groebner_matrix, 47);
    groebner_row35_100000_f(groebner_matrix, 47);
    groebner_row12_100000_f(groebner_matrix, 47);
    groebner_row15_100000_f(groebner_matrix, 47);
    groebner_row37_100000_f(groebner_matrix, 47);
    groebner_row38_100000_f(groebner_matrix, 47);
    groebner_row39_100000_f(groebner_matrix, 47);
    groebner_row40_100000_f(groebner_matrix, 47);
    groebner_row41_100000_f(groebner_matrix, 47);
    groebner_row36_000000_f(groebner_matrix, 47);
    groebner_row43_000000_f(groebner_matrix, 47);
    groebner_row42_000000_f(groebner_matrix, 47);
    groebner_row46_000000_f(groebner_matrix, 47);
    groebner_row44_000000_f(groebner_matrix, 47);
    groebner_row45_000000_f(groebner_matrix, 47);
    groebner_row31_000000_f(groebner_matrix, 47);
    groebner_row30_000000_f(groebner_matrix, 47);
}

#[inline]
fn compute_stage_10(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row32_000000_f(groebner_matrix, 47);
    groebner_row33_000000_f(groebner_matrix, 47);
    groebner_row34_000000_f(groebner_matrix, 47);
    groebner_row35_000000_f(groebner_matrix, 47);
    groebner_row12_000000_f(groebner_matrix, 47);
    groebner_row15_000000_f(groebner_matrix, 47);
    groebner_row37_000000_f(groebner_matrix, 47);
    groebner_row38_000000_f(groebner_matrix, 47);
    groebner_row39_000000_f(groebner_matrix, 47);
    groebner_row40_000000_f(groebner_matrix, 47);
}

#[inline]
fn compute_stage_11(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    compute_stage_11_section_0(groebner_matrix);
    compute_stage_11_section_1(groebner_matrix);
    compute_stage_11_section_2(groebner_matrix);
    compute_stage_11_section_3(groebner_matrix);
    compute_stage_11_section_4(groebner_matrix);
    compute_stage_11_section_5(groebner_matrix);
    compute_stage_11_section_6(groebner_matrix);
    compute_stage_11_section_7(groebner_matrix);
}
#[inline]
fn compute_stage_11_section_0(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    groebner_row41_000000_f(groebner_matrix, 47);
    let factor = 1.0 / groebner_matrix[(10, 66)];
    groebner_matrix.row_mut(10).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(12, 70)];
    groebner_matrix.row_mut(12).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(15, 71)];
    groebner_matrix.row_mut(15).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(30, 64)];
    groebner_matrix.row_mut(30).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(31, 63)];
    groebner_matrix.row_mut(31).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(32, 65)];
    groebner_matrix.row_mut(32).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(33, 67)];
    groebner_matrix.row_mut(33).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(34, 68)];
    groebner_matrix.row_mut(34).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(35, 69)];
    groebner_matrix.row_mut(35).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(36, 56)];
    groebner_matrix.row_mut(36).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(37, 72)];
    groebner_matrix.row_mut(37).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(38, 73)];
    groebner_matrix.row_mut(38).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(39, 74)];
    groebner_matrix.row_mut(39).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(40, 75)];
    groebner_matrix.row_mut(40).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(41, 76)];
    groebner_matrix.row_mut(41).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(42, 58)];
    groebner_matrix.row_mut(42).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(43, 57)];
    groebner_matrix.row_mut(43).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(44, 60)];
    groebner_matrix.row_mut(44).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(45, 61)];
    groebner_matrix.row_mut(45).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(46, 59)];
    groebner_matrix.row_mut(46).scale_mut(factor);
    let factor = 1.0 / groebner_matrix[(47, 62)];
    groebner_matrix.row_mut(47).scale_mut(factor);
    let factor = groebner_matrix[(10, 70)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(12, col)];
        groebner_matrix[(10, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(10, 71)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(15, col)];
        groebner_matrix[(10, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(10, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(10, col)] -= reduction_term;
    }
}

#[inline]
fn compute_stage_11_section_1(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    let factor = groebner_matrix[(10, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(10, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(12, 71)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(15, col)];
        groebner_matrix[(12, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(12, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(12, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(12, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(12, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(15, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(15, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 65)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(32, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 67)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(33, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 68)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(34, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 69)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(35, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 72)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(37, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
}

#[inline]
fn compute_stage_11_section_2(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    let factor = groebner_matrix[(30, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(30, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 65)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(32, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 67)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(33, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 68)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(34, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 69)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(35, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 72)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(37, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(31, col)] -= reduction_term;
    }
}

#[inline]
fn compute_stage_11_section_3(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    let factor = groebner_matrix[(32, 67)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(33, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 68)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(34, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 69)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(35, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 72)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(37, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(32, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 68)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(34, col)];
        groebner_matrix[(33, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 69)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(35, col)];
        groebner_matrix[(33, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 72)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(37, col)];
        groebner_matrix[(33, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(33, col)] -= reduction_term;
    }
}

#[inline]
fn compute_stage_11_section_4(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    let factor = groebner_matrix[(33, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(33, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(33, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(33, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 69)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(35, col)];
        groebner_matrix[(34, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 72)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(37, col)];
        groebner_matrix[(34, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(34, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(34, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(34, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(34, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 72)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(37, col)];
        groebner_matrix[(35, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(35, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(35, col)] -= reduction_term;
    }
}

#[inline]
fn compute_stage_11_section_5(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    let factor = groebner_matrix[(35, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(35, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(35, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 72)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(37, col)];
        groebner_matrix[(36, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(36, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(36, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(36, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(36, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 73)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(38, col)];
        groebner_matrix[(37, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(37, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(37, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(37, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(38, 74)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(39, col)];
        groebner_matrix[(38, col)] -= reduction_term;
    }
}

#[inline]
fn compute_stage_11_section_6(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    let factor = groebner_matrix[(38, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(38, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(38, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(38, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(39, 75)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(40, col)];
        groebner_matrix[(39, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(39, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(39, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(40, 76)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(41, col)];
        groebner_matrix[(40, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(42, 59)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(46, col)];
        groebner_matrix[(42, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(42, 60)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(44, col)];
        groebner_matrix[(42, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(42, 61)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(45, col)];
        groebner_matrix[(42, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(42, 62)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(47, col)];
        groebner_matrix[(42, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(43, 59)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(46, col)];
        groebner_matrix[(43, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(43, 60)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(44, col)];
        groebner_matrix[(43, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(43, 61)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(45, col)];
        groebner_matrix[(43, col)] -= reduction_term;
    }
}

#[inline]
fn compute_stage_11_section_7(groebner_matrix: &mut SMatrix<f64, 48, 85>) {
    let factor = groebner_matrix[(43, 62)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(47, col)];
        groebner_matrix[(43, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(44, 61)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(45, col)];
        groebner_matrix[(44, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(44, 62)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(47, col)];
        groebner_matrix[(44, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 62)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(47, col)];
        groebner_matrix[(45, col)] -= reduction_term;
    }
    let factor = groebner_matrix[(46, 62)];
    for col in 0..groebner_matrix.ncols() {
        let reduction_term = factor * groebner_matrix[(47, col)];
        groebner_matrix[(46, col)] -= reduction_term;
    }
}
