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
        groebner_row4_0000_f, groebner_row4_0100_f, groebner_row4_1000_f, groebner_row5_0000_f,
        groebner_row5_0010_f, groebner_row5_0100_f, groebner_row5_1000_f, groebner_row6_0000_f,
        groebner_row6_0010_f, groebner_row6_0100_f, groebner_row6_1000_f, groebner_row7_0000_f,
        groebner_row7_0100_f, groebner_row7_1000_f, groebner_row8_0000_f, groebner_row8_0100_f,
        groebner_row8_1000_f, groebner_row9_0000_f, groebner_row10_0000_f, groebner_row11_0000_f,
        groebner_row12_0000_f, groebner_row12_1000_f, groebner_row13_0000_f, groebner_row13_1000_f,
        groebner_row14_0000_f, groebner_row14_0001_f, groebner_row14_0010_f, groebner_row14_1000_f,
        groebner_row15_0000_f, groebner_row15_0100_f, groebner_row15_1000_f, groebner_row16_0000_f,
        groebner_row16_1000_f, groebner_row17_0000_f, groebner_row17_1000_f, groebner_row18_0000_f,
        groebner_row18_1000_f, groebner_row19_0000_f, groebner_row19_0001_f, groebner_row19_0010_f,
        groebner_row19_0100_f, groebner_row19_1000_f, groebner_row20_0000_f, groebner_row20_0001_f,
        groebner_row20_0010_f, groebner_row20_0100_f, groebner_row20_1000_f, groebner_row20_1100_f,
        groebner_row21_0000_f, groebner_row21_0010_f, groebner_row21_0100_f, groebner_row21_1000_f,
        groebner_row21_1100_f, groebner_row22_0000_f, groebner_row22_0100_f, groebner_row22_1000_f,
        groebner_row22_1100_f, groebner_row23_0000_f,
    },
    spolynomials::{
        s_polynomial5, s_polynomial6, s_polynomial7, s_polynomial8, s_polynomial9, s_polynomial10,
        s_polynomial11, s_polynomial12, s_polynomial13, s_polynomial14, s_polynomial15,
        s_polynomial16, s_polynomial17, s_polynomial18, s_polynomial19, s_polynomial20,
        s_polynomial21, s_polynomial22, s_polynomial23, s_polynomial24,
    },
};
use nalgebra::SMatrix;

pub fn compute(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    compute_stage_0(groebner_matrix);
    compute_stage_1(groebner_matrix);
    compute_stage_2(groebner_matrix);
    compute_stage_3(groebner_matrix);
    compute_stage_4(groebner_matrix);
}
#[inline]
fn compute_stage_0(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    s_polynomial5(groebner_matrix);
    s_polynomial6(groebner_matrix);
    groebner_row5_0000_f(groebner_matrix, 6);
    s_polynomial7(groebner_matrix);
    groebner_row5_0000_f(groebner_matrix, 7);
    groebner_row6_0000_f(groebner_matrix, 7);
    s_polynomial8(groebner_matrix);
    groebner_row5_0000_f(groebner_matrix, 8);
    groebner_row6_0000_f(groebner_matrix, 8);
    groebner_row7_0000_f(groebner_matrix, 8);
    s_polynomial9(groebner_matrix);
    groebner_row7_0100_f(groebner_matrix, 9);
    groebner_row8_0100_f(groebner_matrix, 9);
    groebner_row5_1000_f(groebner_matrix, 9);
    groebner_row6_1000_f(groebner_matrix, 9);
    groebner_row7_1000_f(groebner_matrix, 9);
    groebner_row8_1000_f(groebner_matrix, 9);
    groebner_row5_0000_f(groebner_matrix, 9);
    groebner_row6_0000_f(groebner_matrix, 9);
    groebner_row7_0000_f(groebner_matrix, 9);
    groebner_row8_0000_f(groebner_matrix, 9);
    s_polynomial10(groebner_matrix);
    groebner_row6_0100_f(groebner_matrix, 10);
    groebner_row7_0100_f(groebner_matrix, 10);
    groebner_row8_0100_f(groebner_matrix, 10);
    groebner_row9_0000_f(groebner_matrix, 10);
    groebner_row5_1000_f(groebner_matrix, 10);
    groebner_row6_1000_f(groebner_matrix, 10);
    groebner_row7_1000_f(groebner_matrix, 10);
    groebner_row8_1000_f(groebner_matrix, 10);
    groebner_row5_0000_f(groebner_matrix, 10);
    groebner_row6_0000_f(groebner_matrix, 10);
    groebner_row7_0000_f(groebner_matrix, 10);
    groebner_row8_0000_f(groebner_matrix, 10);
    s_polynomial11(groebner_matrix);
    groebner_row6_0100_f(groebner_matrix, 11);
    groebner_row7_0100_f(groebner_matrix, 11);
    groebner_row8_0100_f(groebner_matrix, 11);
    groebner_row9_0000_f(groebner_matrix, 11);
    groebner_row4_1000_f(groebner_matrix, 11);
    groebner_row5_1000_f(groebner_matrix, 11);
    groebner_row6_1000_f(groebner_matrix, 11);
    groebner_row7_1000_f(groebner_matrix, 11);
    groebner_row8_1000_f(groebner_matrix, 11);
    groebner_row10_0000_f(groebner_matrix, 11);
    groebner_row4_0000_f(groebner_matrix, 11);
    groebner_row5_0000_f(groebner_matrix, 11);
    groebner_row6_0000_f(groebner_matrix, 11);
    groebner_row7_0000_f(groebner_matrix, 11);
    groebner_row8_0000_f(groebner_matrix, 11);
    s_polynomial12(groebner_matrix);
    groebner_row5_0100_f(groebner_matrix, 12);
    groebner_row6_0100_f(groebner_matrix, 12);
    groebner_row7_0100_f(groebner_matrix, 12);
    groebner_row8_0100_f(groebner_matrix, 12);
    groebner_row9_0000_f(groebner_matrix, 12);
    groebner_row4_1000_f(groebner_matrix, 12);
    groebner_row5_1000_f(groebner_matrix, 12);
    groebner_row6_1000_f(groebner_matrix, 12);
    groebner_row7_1000_f(groebner_matrix, 12);
    groebner_row8_1000_f(groebner_matrix, 12);
    groebner_row10_0000_f(groebner_matrix, 12);
}

#[inline]
fn compute_stage_1(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_row11_0000_f(groebner_matrix, 12);
    groebner_row4_0000_f(groebner_matrix, 12);
    groebner_row5_0000_f(groebner_matrix, 12);
    groebner_row6_0000_f(groebner_matrix, 12);
    groebner_row7_0000_f(groebner_matrix, 12);
    groebner_row8_0000_f(groebner_matrix, 12);
    s_polynomial13(groebner_matrix);
    groebner_row6_0010_f(groebner_matrix, 13);
    groebner_row4_0100_f(groebner_matrix, 13);
    groebner_row5_0100_f(groebner_matrix, 13);
    groebner_row6_0100_f(groebner_matrix, 13);
    groebner_row7_0100_f(groebner_matrix, 13);
    groebner_row8_0100_f(groebner_matrix, 13);
    groebner_row9_0000_f(groebner_matrix, 13);
    groebner_row4_1000_f(groebner_matrix, 13);
    groebner_row5_1000_f(groebner_matrix, 13);
    groebner_row6_1000_f(groebner_matrix, 13);
    groebner_row7_1000_f(groebner_matrix, 13);
    groebner_row8_1000_f(groebner_matrix, 13);
    groebner_row10_0000_f(groebner_matrix, 13);
    groebner_row11_0000_f(groebner_matrix, 13);
    groebner_row12_0000_f(groebner_matrix, 13);
    groebner_row4_0000_f(groebner_matrix, 13);
    groebner_row5_0000_f(groebner_matrix, 13);
    groebner_row6_0000_f(groebner_matrix, 13);
    groebner_row7_0000_f(groebner_matrix, 13);
    groebner_row8_0000_f(groebner_matrix, 13);
    s_polynomial14(groebner_matrix);
    groebner_row5_0010_f(groebner_matrix, 14);
    groebner_row6_0010_f(groebner_matrix, 14);
    groebner_row4_0100_f(groebner_matrix, 14);
    groebner_row5_0100_f(groebner_matrix, 14);
    groebner_row6_0100_f(groebner_matrix, 14);
    groebner_row7_0100_f(groebner_matrix, 14);
    groebner_row8_0100_f(groebner_matrix, 14);
    groebner_row9_0000_f(groebner_matrix, 14);
    groebner_row4_1000_f(groebner_matrix, 14);
    groebner_row5_1000_f(groebner_matrix, 14);
    groebner_row6_1000_f(groebner_matrix, 14);
    groebner_row7_1000_f(groebner_matrix, 14);
    groebner_row8_1000_f(groebner_matrix, 14);
    groebner_row10_0000_f(groebner_matrix, 14);
    groebner_row11_0000_f(groebner_matrix, 14);
    groebner_row12_0000_f(groebner_matrix, 14);
    groebner_row13_0000_f(groebner_matrix, 14);
    groebner_row4_0000_f(groebner_matrix, 14);
    groebner_row5_0000_f(groebner_matrix, 14);
    groebner_row6_0000_f(groebner_matrix, 14);
    groebner_row7_0000_f(groebner_matrix, 14);
    groebner_row8_0000_f(groebner_matrix, 14);
    s_polynomial15(groebner_matrix);
    groebner_row14_1000_f(groebner_matrix, 15);
    groebner_row9_0000_f(groebner_matrix, 15);
    groebner_row7_1000_f(groebner_matrix, 15);
    groebner_row8_1000_f(groebner_matrix, 15);
    groebner_row10_0000_f(groebner_matrix, 15);
    groebner_row11_0000_f(groebner_matrix, 15);
    groebner_row12_0000_f(groebner_matrix, 15);
    groebner_row13_0000_f(groebner_matrix, 15);
    groebner_row14_0000_f(groebner_matrix, 15);
    groebner_row7_0000_f(groebner_matrix, 15);
    groebner_row8_0000_f(groebner_matrix, 15);
}

#[inline]
fn compute_stage_2(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    s_polynomial16(groebner_matrix);
    groebner_row13_1000_f(groebner_matrix, 16);
    groebner_row14_1000_f(groebner_matrix, 16);
    groebner_row8_0100_f(groebner_matrix, 16);
    groebner_row15_0100_f(groebner_matrix, 16);
    groebner_row5_1000_f(groebner_matrix, 16);
    groebner_row6_1000_f(groebner_matrix, 16);
    groebner_row7_1000_f(groebner_matrix, 16);
    groebner_row8_1000_f(groebner_matrix, 16);
    groebner_row15_1000_f(groebner_matrix, 16);
    groebner_row11_0000_f(groebner_matrix, 16);
    groebner_row12_0000_f(groebner_matrix, 16);
    groebner_row13_0000_f(groebner_matrix, 16);
    groebner_row14_0000_f(groebner_matrix, 16);
    groebner_row5_0000_f(groebner_matrix, 16);
    groebner_row6_0000_f(groebner_matrix, 16);
    groebner_row7_0000_f(groebner_matrix, 16);
    groebner_row8_0000_f(groebner_matrix, 16);
    groebner_row15_0000_f(groebner_matrix, 16);
    s_polynomial17(groebner_matrix);
    groebner_row12_1000_f(groebner_matrix, 17);
    groebner_row13_1000_f(groebner_matrix, 17);
    groebner_row14_1000_f(groebner_matrix, 17);
    groebner_row7_0100_f(groebner_matrix, 17);
    groebner_row8_0100_f(groebner_matrix, 17);
    groebner_row15_0100_f(groebner_matrix, 17);
    groebner_row4_1000_f(groebner_matrix, 17);
    groebner_row5_1000_f(groebner_matrix, 17);
    groebner_row6_1000_f(groebner_matrix, 17);
    groebner_row7_1000_f(groebner_matrix, 17);
    groebner_row8_1000_f(groebner_matrix, 17);
    groebner_row15_1000_f(groebner_matrix, 17);
    groebner_row16_1000_f(groebner_matrix, 17);
    groebner_row12_0000_f(groebner_matrix, 17);
    groebner_row13_0000_f(groebner_matrix, 17);
    groebner_row14_0000_f(groebner_matrix, 17);
    groebner_row4_0000_f(groebner_matrix, 17);
    groebner_row5_0000_f(groebner_matrix, 17);
    groebner_row6_0000_f(groebner_matrix, 17);
    groebner_row7_0000_f(groebner_matrix, 17);
    groebner_row8_0000_f(groebner_matrix, 17);
    groebner_row15_0000_f(groebner_matrix, 17);
    groebner_row16_0000_f(groebner_matrix, 17);
    s_polynomial18(groebner_matrix);
    groebner_row14_0001_f(groebner_matrix, 18);
    groebner_row14_0010_f(groebner_matrix, 18);
    groebner_row13_1000_f(groebner_matrix, 18);
    groebner_row14_1000_f(groebner_matrix, 18);
    groebner_row7_0100_f(groebner_matrix, 18);
    groebner_row8_0100_f(groebner_matrix, 18);
    groebner_row15_0100_f(groebner_matrix, 18);
    groebner_row4_1000_f(groebner_matrix, 18);
    groebner_row5_1000_f(groebner_matrix, 18);
    groebner_row6_1000_f(groebner_matrix, 18);
    groebner_row7_1000_f(groebner_matrix, 18);
    groebner_row8_1000_f(groebner_matrix, 18);
    groebner_row15_1000_f(groebner_matrix, 18);
    groebner_row16_1000_f(groebner_matrix, 18);
    groebner_row17_1000_f(groebner_matrix, 18);
    groebner_row13_0000_f(groebner_matrix, 18);
    groebner_row14_0000_f(groebner_matrix, 18);
    groebner_row4_0000_f(groebner_matrix, 18);
}

#[inline]
fn compute_stage_3(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_row5_0000_f(groebner_matrix, 18);
    groebner_row6_0000_f(groebner_matrix, 18);
    groebner_row7_0000_f(groebner_matrix, 18);
    groebner_row8_0000_f(groebner_matrix, 18);
    groebner_row15_0000_f(groebner_matrix, 18);
    groebner_row16_0000_f(groebner_matrix, 18);
    groebner_row17_0000_f(groebner_matrix, 18);
    s_polynomial19(groebner_matrix);
    groebner_row14_0000_f(groebner_matrix, 19);
    groebner_row15_0000_f(groebner_matrix, 19);
    groebner_row16_0000_f(groebner_matrix, 19);
    groebner_row17_0000_f(groebner_matrix, 19);
    groebner_row18_0000_f(groebner_matrix, 19);
    s_polynomial20(groebner_matrix);
    groebner_row18_1000_f(groebner_matrix, 20);
    groebner_row19_1000_f(groebner_matrix, 20);
    groebner_row15_0000_f(groebner_matrix, 20);
    groebner_row16_0000_f(groebner_matrix, 20);
    groebner_row17_0000_f(groebner_matrix, 20);
    groebner_row18_0000_f(groebner_matrix, 20);
    groebner_row19_0000_f(groebner_matrix, 20);
    s_polynomial21(groebner_matrix);
    groebner_row17_1000_f(groebner_matrix, 21);
    groebner_row18_1000_f(groebner_matrix, 21);
    groebner_row19_1000_f(groebner_matrix, 21);
    groebner_row15_0000_f(groebner_matrix, 21);
    groebner_row20_1000_f(groebner_matrix, 21);
    groebner_row17_0000_f(groebner_matrix, 21);
    groebner_row18_0000_f(groebner_matrix, 21);
    groebner_row19_0000_f(groebner_matrix, 21);
    groebner_row20_0000_f(groebner_matrix, 21);
    s_polynomial22(groebner_matrix);
    groebner_row19_0001_f(groebner_matrix, 22);
    groebner_row19_0010_f(groebner_matrix, 22);
    groebner_row18_1000_f(groebner_matrix, 22);
    groebner_row19_1000_f(groebner_matrix, 22);
    groebner_row20_0001_f(groebner_matrix, 22);
    groebner_row20_0010_f(groebner_matrix, 22);
    groebner_row21_0010_f(groebner_matrix, 22);
    groebner_row20_0100_f(groebner_matrix, 22);
    groebner_row21_0100_f(groebner_matrix, 22);
    groebner_row15_0000_f(groebner_matrix, 22);
    groebner_row20_1000_f(groebner_matrix, 22);
    groebner_row21_1000_f(groebner_matrix, 22);
    groebner_row18_0000_f(groebner_matrix, 22);
    groebner_row19_0000_f(groebner_matrix, 22);
    groebner_row20_0000_f(groebner_matrix, 22);
    groebner_row21_0000_f(groebner_matrix, 22);
    s_polynomial23(groebner_matrix);
    groebner_row20_1100_f(groebner_matrix, 23);
    groebner_row21_1100_f(groebner_matrix, 23);
    groebner_row22_1100_f(groebner_matrix, 23);
    groebner_row19_0001_f(groebner_matrix, 23);
    groebner_row19_0010_f(groebner_matrix, 23);
    groebner_row19_0100_f(groebner_matrix, 23);
    groebner_row19_1000_f(groebner_matrix, 23);
    groebner_row20_0001_f(groebner_matrix, 23);
    groebner_row20_0010_f(groebner_matrix, 23);
    groebner_row21_0010_f(groebner_matrix, 23);
    groebner_row20_0100_f(groebner_matrix, 23);
    groebner_row21_0100_f(groebner_matrix, 23);
    groebner_row22_0100_f(groebner_matrix, 23);
}

#[inline]
fn compute_stage_4(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_row20_1000_f(groebner_matrix, 23);
    groebner_row21_1000_f(groebner_matrix, 23);
    groebner_row22_1000_f(groebner_matrix, 23);
    groebner_row19_0000_f(groebner_matrix, 23);
    groebner_row20_0000_f(groebner_matrix, 23);
    groebner_row21_0000_f(groebner_matrix, 23);
    groebner_row22_0000_f(groebner_matrix, 23);
    s_polynomial24(groebner_matrix);
    groebner_row20_0000_f(groebner_matrix, 24);
    groebner_row21_0000_f(groebner_matrix, 24);
    groebner_row22_0000_f(groebner_matrix, 24);
    groebner_row23_0000_f(groebner_matrix, 24);
}
