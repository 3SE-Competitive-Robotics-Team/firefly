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
        groebner_row3_000_f, groebner_row3_100_f, groebner_row4_000_f, groebner_row4_010_f,
        groebner_row4_100_f, groebner_row5_000_f, groebner_row5_010_f, groebner_row5_100_f,
        groebner_row6_000_f, groebner_row6_100_f, groebner_row7_000_f, groebner_row8_000_f,
        groebner_row9_000_f, groebner_row9_100_f, groebner_row10_000_f, groebner_row10_100_f,
        groebner_row11_000_f, groebner_row11_010_f, groebner_row11_100_f, groebner_row12_000_f,
        groebner_row12_010_f, groebner_row12_100_f, groebner_row13_000_f,
    },
    spolynomials::{
        s_polynomial4, s_polynomial5, s_polynomial6, s_polynomial7, s_polynomial8, s_polynomial9,
        s_polynomial10, s_polynomial11, s_polynomial12, s_polynomial13, s_polynomial14,
    },
};
use nalgebra::SMatrix;

pub fn compute(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    s_polynomial4(groebner_matrix);

    s_polynomial5(groebner_matrix);
    groebner_row4_000_f(groebner_matrix, 5);

    s_polynomial6(groebner_matrix);
    groebner_row4_000_f(groebner_matrix, 6);
    groebner_row5_000_f(groebner_matrix, 6);

    s_polynomial7(groebner_matrix);
    groebner_row5_100_f(groebner_matrix, 7);
    groebner_row6_100_f(groebner_matrix, 7);
    groebner_row4_000_f(groebner_matrix, 7);
    groebner_row5_000_f(groebner_matrix, 7);
    groebner_row6_000_f(groebner_matrix, 7);

    s_polynomial8(groebner_matrix);
    groebner_row4_100_f(groebner_matrix, 8);
    groebner_row5_100_f(groebner_matrix, 8);
    groebner_row6_100_f(groebner_matrix, 8);
    groebner_row7_000_f(groebner_matrix, 8);
    groebner_row3_000_f(groebner_matrix, 8);
    groebner_row4_000_f(groebner_matrix, 8);
    groebner_row5_000_f(groebner_matrix, 8);
    groebner_row6_000_f(groebner_matrix, 8);

    s_polynomial9(groebner_matrix);
    groebner_row5_010_f(groebner_matrix, 9);
    groebner_row3_100_f(groebner_matrix, 9);
    groebner_row4_100_f(groebner_matrix, 9);
    groebner_row5_100_f(groebner_matrix, 9);
    groebner_row6_100_f(groebner_matrix, 9);
    groebner_row7_000_f(groebner_matrix, 9);
    groebner_row8_000_f(groebner_matrix, 9);
    groebner_row3_000_f(groebner_matrix, 9);
    groebner_row4_000_f(groebner_matrix, 9);
    groebner_row5_000_f(groebner_matrix, 9);
    groebner_row6_000_f(groebner_matrix, 9);

    s_polynomial10(groebner_matrix);
    groebner_row4_010_f(groebner_matrix, 10);
    groebner_row5_010_f(groebner_matrix, 10);
    groebner_row3_100_f(groebner_matrix, 10);
    groebner_row4_100_f(groebner_matrix, 10);
    groebner_row5_100_f(groebner_matrix, 10);
    groebner_row6_100_f(groebner_matrix, 10);
    groebner_row9_100_f(groebner_matrix, 10);
    groebner_row8_000_f(groebner_matrix, 10);
    groebner_row3_000_f(groebner_matrix, 10);
    groebner_row4_000_f(groebner_matrix, 10);
    groebner_row5_000_f(groebner_matrix, 10);
    groebner_row6_000_f(groebner_matrix, 10);
    groebner_row9_000_f(groebner_matrix, 10);

    s_polynomial11(groebner_matrix);
    groebner_row6_000_f(groebner_matrix, 11);
    groebner_row9_000_f(groebner_matrix, 11);
    groebner_row10_000_f(groebner_matrix, 11);

    s_polynomial12(groebner_matrix);
    groebner_row10_100_f(groebner_matrix, 12);
    groebner_row11_100_f(groebner_matrix, 12);
    groebner_row9_000_f(groebner_matrix, 12);
    groebner_row10_000_f(groebner_matrix, 12);
    groebner_row11_000_f(groebner_matrix, 12);

    s_polynomial13(groebner_matrix);
    groebner_row10_100_f(groebner_matrix, 13);
    groebner_row11_010_f(groebner_matrix, 13);
    groebner_row12_010_f(groebner_matrix, 13);
    groebner_row11_100_f(groebner_matrix, 13);
    groebner_row12_100_f(groebner_matrix, 13);
    groebner_row10_000_f(groebner_matrix, 13);
    groebner_row11_000_f(groebner_matrix, 13);
    groebner_row12_000_f(groebner_matrix, 13);

    s_polynomial14(groebner_matrix);
    groebner_row11_000_f(groebner_matrix, 14);
    groebner_row12_000_f(groebner_matrix, 14);
    groebner_row13_000_f(groebner_matrix, 14);
}
