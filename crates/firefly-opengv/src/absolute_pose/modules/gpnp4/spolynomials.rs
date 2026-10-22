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

pub fn s_polynomial5(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(5, 23)] = groebner_matrix[(0, 23)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 23)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 24)] = groebner_matrix[(0, 24)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 24)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 25)] = groebner_matrix[(0, 25)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 25)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 26)] = groebner_matrix[(0, 26)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 26)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 27)] = groebner_matrix[(0, 27)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 27)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 28)] = groebner_matrix[(0, 28)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 28)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 29)] = groebner_matrix[(0, 29)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 29)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 30)] = groebner_matrix[(0, 30)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 30)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 31)] = groebner_matrix[(0, 31)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 31)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 32)] = groebner_matrix[(0, 32)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 32)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 33)] = groebner_matrix[(0, 33)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 33)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 34)] = groebner_matrix[(0, 34)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 34)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 35)] = groebner_matrix[(0, 35)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 35)] / (groebner_matrix[(1, 22)]);
    groebner_matrix[(5, 36)] = groebner_matrix[(0, 36)] / (groebner_matrix[(0, 22)])
        - groebner_matrix[(1, 36)] / (groebner_matrix[(1, 22)]);
}

pub fn s_polynomial6(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(6, 23)] = groebner_matrix[(1, 23)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 23)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 24)] = groebner_matrix[(1, 24)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 24)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 25)] = groebner_matrix[(1, 25)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 25)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 26)] = groebner_matrix[(1, 26)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 26)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 27)] = groebner_matrix[(1, 27)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 27)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 28)] = groebner_matrix[(1, 28)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 28)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 29)] = groebner_matrix[(1, 29)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 29)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 30)] = groebner_matrix[(1, 30)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 30)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 31)] = groebner_matrix[(1, 31)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 31)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 32)] = groebner_matrix[(1, 32)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 32)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 33)] = groebner_matrix[(1, 33)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 33)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 34)] = groebner_matrix[(1, 34)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 34)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 35)] = groebner_matrix[(1, 35)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 35)] / (groebner_matrix[(2, 22)]);
    groebner_matrix[(6, 36)] = groebner_matrix[(1, 36)] / (groebner_matrix[(1, 22)])
        - groebner_matrix[(2, 36)] / (groebner_matrix[(2, 22)]);
}

pub fn s_polynomial7(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(7, 23)] = groebner_matrix[(2, 23)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 23)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 24)] = groebner_matrix[(2, 24)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 24)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 25)] = groebner_matrix[(2, 25)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 25)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 26)] = groebner_matrix[(2, 26)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 26)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 27)] = groebner_matrix[(2, 27)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 27)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 28)] = groebner_matrix[(2, 28)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 28)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 29)] = groebner_matrix[(2, 29)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 29)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 30)] = groebner_matrix[(2, 30)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 30)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 31)] = groebner_matrix[(2, 31)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 31)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 32)] = groebner_matrix[(2, 32)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 32)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 33)] = groebner_matrix[(2, 33)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 33)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 34)] = groebner_matrix[(2, 34)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 34)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 35)] = groebner_matrix[(2, 35)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 35)] / (groebner_matrix[(3, 22)]);
    groebner_matrix[(7, 36)] = groebner_matrix[(2, 36)] / (groebner_matrix[(2, 22)])
        - groebner_matrix[(3, 36)] / (groebner_matrix[(3, 22)]);
}

pub fn s_polynomial8(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(8, 23)] = groebner_matrix[(3, 23)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 23)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 24)] = groebner_matrix[(3, 24)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 24)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 25)] = groebner_matrix[(3, 25)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 25)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 26)] = groebner_matrix[(3, 26)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 26)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 27)] = groebner_matrix[(3, 27)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 27)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 28)] = groebner_matrix[(3, 28)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 28)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 29)] = groebner_matrix[(3, 29)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 29)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 30)] = groebner_matrix[(3, 30)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 30)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 31)] = groebner_matrix[(3, 31)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 31)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 32)] = groebner_matrix[(3, 32)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 32)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 33)] = groebner_matrix[(3, 33)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 33)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 34)] = groebner_matrix[(3, 34)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 34)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 35)] = groebner_matrix[(3, 35)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 35)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(8, 36)] = groebner_matrix[(3, 36)] / (groebner_matrix[(3, 22)])
        - groebner_matrix[(4, 36)] / (groebner_matrix[(4, 22)]);
}

pub fn s_polynomial9(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(9, 9)] = groebner_matrix[(6, 25)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 10)] = groebner_matrix[(6, 26)] / (groebner_matrix[(6, 24)])
        - groebner_matrix[(8, 27)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 11)] = groebner_matrix[(6, 27)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 13)] = -groebner_matrix[(8, 28)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 14)] = -groebner_matrix[(8, 29)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 15)] = groebner_matrix[(6, 28)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 16)] = groebner_matrix[(6, 29)] / (groebner_matrix[(6, 24)])
        - groebner_matrix[(8, 30)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 17)] = groebner_matrix[(6, 30)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 19)] = -groebner_matrix[(8, 31)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 20)] = groebner_matrix[(6, 31)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 23)] = -groebner_matrix[(8, 32)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 24)] = -groebner_matrix[(8, 33)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 25)] = groebner_matrix[(6, 32)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 26)] = groebner_matrix[(6, 33)] / (groebner_matrix[(6, 24)])
        - groebner_matrix[(8, 34)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 27)] = groebner_matrix[(6, 34)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 29)] = -groebner_matrix[(8, 35)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 30)] = groebner_matrix[(6, 35)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(9, 33)] = -groebner_matrix[(8, 36)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(9, 34)] = groebner_matrix[(6, 36)] / (groebner_matrix[(6, 24)]);
}

pub fn s_polynomial10(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(10, 8)] = groebner_matrix[(5, 24)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(7, 26)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 9)] = groebner_matrix[(5, 25)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 10)] = groebner_matrix[(5, 26)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(7, 27)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 11)] = groebner_matrix[(5, 27)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 13)] = -groebner_matrix[(7, 28)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 14)] = -groebner_matrix[(7, 29)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 15)] = groebner_matrix[(5, 28)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 16)] = groebner_matrix[(5, 29)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(7, 30)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 17)] = groebner_matrix[(5, 30)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 19)] = -groebner_matrix[(7, 31)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 20)] = groebner_matrix[(5, 31)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 23)] = -groebner_matrix[(7, 32)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 24)] = -groebner_matrix[(7, 33)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 25)] = groebner_matrix[(5, 32)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 26)] = groebner_matrix[(5, 33)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(7, 34)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 27)] = groebner_matrix[(5, 34)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 29)] = -groebner_matrix[(7, 35)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 30)] = groebner_matrix[(5, 35)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(10, 33)] = -groebner_matrix[(7, 36)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(10, 34)] = groebner_matrix[(5, 36)] / (groebner_matrix[(5, 23)]);
}

pub fn s_polynomial11(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(11, 8)] = groebner_matrix[(5, 24)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 9)] = groebner_matrix[(5, 25)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(8, 27)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 10)] = groebner_matrix[(5, 26)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 11)] = groebner_matrix[(5, 27)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 12)] = -groebner_matrix[(8, 28)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 13)] = -groebner_matrix[(8, 29)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 15)] = groebner_matrix[(5, 28)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(8, 30)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 16)] = groebner_matrix[(5, 29)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 17)] = groebner_matrix[(5, 30)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 18)] = -groebner_matrix[(8, 31)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 20)] = groebner_matrix[(5, 31)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 22)] = -groebner_matrix[(8, 32)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 23)] = -groebner_matrix[(8, 33)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 25)] = groebner_matrix[(5, 32)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(8, 34)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 26)] = groebner_matrix[(5, 33)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 27)] = groebner_matrix[(5, 34)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 28)] = -groebner_matrix[(8, 35)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 30)] = groebner_matrix[(5, 35)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(11, 32)] = -groebner_matrix[(8, 36)] / (groebner_matrix[(8, 26)]);
    groebner_matrix[(11, 34)] = groebner_matrix[(5, 36)] / (groebner_matrix[(5, 23)]);
}

pub fn s_polynomial12(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(12, 7)] = groebner_matrix[(4, 23)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(7, 26)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 8)] = groebner_matrix[(4, 24)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 9)] = groebner_matrix[(4, 25)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(7, 27)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 10)] = groebner_matrix[(4, 26)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 11)] = groebner_matrix[(4, 27)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 12)] = -groebner_matrix[(7, 28)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 13)] = -groebner_matrix[(7, 29)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 15)] = groebner_matrix[(4, 28)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(7, 30)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 16)] = groebner_matrix[(4, 29)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 17)] = groebner_matrix[(4, 30)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 18)] = -groebner_matrix[(7, 31)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 20)] = groebner_matrix[(4, 31)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 22)] = -groebner_matrix[(7, 32)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 23)] = -groebner_matrix[(7, 33)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 25)] = groebner_matrix[(4, 32)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(7, 34)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 26)] = groebner_matrix[(4, 33)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 27)] = groebner_matrix[(4, 34)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 28)] = -groebner_matrix[(7, 35)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 30)] = groebner_matrix[(4, 35)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(12, 32)] = -groebner_matrix[(7, 36)] / (groebner_matrix[(7, 25)]);
    groebner_matrix[(12, 34)] = groebner_matrix[(4, 36)] / (groebner_matrix[(4, 22)]);
}

pub fn s_polynomial13(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(13, 5)] = groebner_matrix[(5, 24)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 6)] = -groebner_matrix[(6, 25)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 7)] = groebner_matrix[(5, 25)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(6, 26)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 8)] = groebner_matrix[(5, 26)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 9)] = -groebner_matrix[(6, 27)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 10)] = groebner_matrix[(5, 27)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 12)] = -groebner_matrix[(6, 28)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 13)] = groebner_matrix[(5, 28)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(6, 29)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 14)] = groebner_matrix[(5, 29)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 15)] = -groebner_matrix[(6, 30)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 16)] = groebner_matrix[(5, 30)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 18)] = -groebner_matrix[(6, 31)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 19)] = groebner_matrix[(5, 31)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 22)] = -groebner_matrix[(6, 32)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 23)] = groebner_matrix[(5, 32)] / (groebner_matrix[(5, 23)])
        - groebner_matrix[(6, 33)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 24)] = groebner_matrix[(5, 33)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 25)] = -groebner_matrix[(6, 34)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 26)] = groebner_matrix[(5, 34)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 28)] = -groebner_matrix[(6, 35)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 29)] = groebner_matrix[(5, 35)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(13, 32)] = -groebner_matrix[(6, 36)] / (groebner_matrix[(6, 24)]);
    groebner_matrix[(13, 33)] = groebner_matrix[(5, 36)] / (groebner_matrix[(5, 23)]);
}

pub fn s_polynomial14(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(14, 4)] = groebner_matrix[(4, 23)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(5, 24)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 5)] = groebner_matrix[(4, 24)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 6)] = -groebner_matrix[(5, 25)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 7)] = groebner_matrix[(4, 25)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(5, 26)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 8)] = groebner_matrix[(4, 26)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 9)] = -groebner_matrix[(5, 27)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 10)] = groebner_matrix[(4, 27)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 12)] = -groebner_matrix[(5, 28)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 13)] = groebner_matrix[(4, 28)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(5, 29)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 14)] = groebner_matrix[(4, 29)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 15)] = -groebner_matrix[(5, 30)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 16)] = groebner_matrix[(4, 30)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 18)] = -groebner_matrix[(5, 31)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 19)] = groebner_matrix[(4, 31)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 22)] = -groebner_matrix[(5, 32)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 23)] = groebner_matrix[(4, 32)] / (groebner_matrix[(4, 22)])
        - groebner_matrix[(5, 33)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 24)] = groebner_matrix[(4, 33)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 25)] = -groebner_matrix[(5, 34)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 26)] = groebner_matrix[(4, 34)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 28)] = -groebner_matrix[(5, 35)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 29)] = groebner_matrix[(4, 35)] / (groebner_matrix[(4, 22)]);
    groebner_matrix[(14, 32)] = -groebner_matrix[(5, 36)] / (groebner_matrix[(5, 23)]);
    groebner_matrix[(14, 33)] = groebner_matrix[(4, 36)] / (groebner_matrix[(4, 22)]);
}

pub fn s_polynomial15(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(15, 3)] = groebner_matrix[(13, 21)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(15, 11)] = -groebner_matrix[(14, 27)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 15)] = -groebner_matrix[(14, 28)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 16)] = -groebner_matrix[(14, 29)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 17)] = groebner_matrix[(13, 27)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(14, 30)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 18)] = groebner_matrix[(13, 28)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(15, 19)] = groebner_matrix[(13, 29)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(15, 20)] = groebner_matrix[(13, 30)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(14, 31)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 21)] = groebner_matrix[(13, 31)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(15, 25)] = -groebner_matrix[(14, 32)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 26)] = -groebner_matrix[(14, 33)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 27)] = -groebner_matrix[(14, 34)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 28)] = groebner_matrix[(13, 32)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(15, 29)] = groebner_matrix[(13, 33)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(15, 30)] = groebner_matrix[(13, 34)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(14, 35)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 31)] = groebner_matrix[(13, 35)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(15, 34)] = -groebner_matrix[(14, 36)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(15, 35)] = groebner_matrix[(13, 36)] / (groebner_matrix[(13, 20)]);
}

pub fn s_polynomial16(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(16, 2)] = groebner_matrix[(12, 20)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 3)] = groebner_matrix[(12, 21)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 10)] = -groebner_matrix[(14, 27)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 13)] = -groebner_matrix[(14, 28)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 14)] = -groebner_matrix[(14, 29)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 16)] = -groebner_matrix[(14, 30)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 17)] = groebner_matrix[(12, 27)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 18)] = groebner_matrix[(12, 28)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 19)] = groebner_matrix[(12, 29)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(14, 31)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 20)] = groebner_matrix[(12, 30)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 21)] = groebner_matrix[(12, 31)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 23)] = -groebner_matrix[(14, 32)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 24)] = -groebner_matrix[(14, 33)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 26)] = -groebner_matrix[(14, 34)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 28)] = groebner_matrix[(12, 32)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 29)] = groebner_matrix[(12, 33)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(14, 35)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 30)] = groebner_matrix[(12, 34)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 31)] = groebner_matrix[(12, 35)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(16, 33)] = -groebner_matrix[(14, 36)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(16, 35)] = groebner_matrix[(12, 36)] / (groebner_matrix[(12, 19)]);
}

pub fn s_polynomial17(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(17, 1)] = groebner_matrix[(11, 19)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 2)] = groebner_matrix[(11, 20)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 3)] = groebner_matrix[(11, 21)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 9)] = -groebner_matrix[(14, 27)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 12)] = -groebner_matrix[(14, 28)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 13)] = -groebner_matrix[(14, 29)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 15)] = -groebner_matrix[(14, 30)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 17)] = groebner_matrix[(11, 27)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 18)] = groebner_matrix[(11, 28)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(14, 31)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 19)] = groebner_matrix[(11, 29)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 20)] = groebner_matrix[(11, 30)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 21)] = groebner_matrix[(11, 31)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 22)] = -groebner_matrix[(14, 32)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 23)] = -groebner_matrix[(14, 33)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 25)] = -groebner_matrix[(14, 34)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 28)] = groebner_matrix[(11, 32)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(14, 35)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 29)] = groebner_matrix[(11, 33)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 30)] = groebner_matrix[(11, 34)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 31)] = groebner_matrix[(11, 35)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(17, 32)] = -groebner_matrix[(14, 36)] / (groebner_matrix[(14, 21)]);
    groebner_matrix[(17, 35)] = groebner_matrix[(11, 36)] / (groebner_matrix[(11, 18)]);
}

pub fn s_polynomial18(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(18, 0)] = groebner_matrix[(10, 18)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 1)] = groebner_matrix[(10, 19)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 2)] = groebner_matrix[(10, 20)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(13, 21)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 3)] = groebner_matrix[(10, 21)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 11)] = -groebner_matrix[(13, 27)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 15)] = -groebner_matrix[(13, 28)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 16)] = -groebner_matrix[(13, 29)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 17)] = groebner_matrix[(10, 27)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(13, 30)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 18)] = groebner_matrix[(10, 28)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 19)] = groebner_matrix[(10, 29)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 20)] = groebner_matrix[(10, 30)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(13, 31)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 21)] = groebner_matrix[(10, 31)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 25)] = -groebner_matrix[(13, 32)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 26)] = -groebner_matrix[(13, 33)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 27)] = -groebner_matrix[(13, 34)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 28)] = groebner_matrix[(10, 32)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 29)] = groebner_matrix[(10, 33)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 30)] = groebner_matrix[(10, 34)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(13, 35)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 31)] = groebner_matrix[(10, 35)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(18, 34)] = -groebner_matrix[(13, 36)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(18, 35)] = groebner_matrix[(10, 36)] / (groebner_matrix[(10, 17)]);
}

pub fn s_polynomial19(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(19, 21)] = groebner_matrix[(13, 21)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(18, 31)] / (groebner_matrix[(18, 30)]);
    groebner_matrix[(19, 27)] = groebner_matrix[(13, 27)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(19, 28)] = groebner_matrix[(13, 28)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(18, 32)] / (groebner_matrix[(18, 30)]);
    groebner_matrix[(19, 29)] = groebner_matrix[(13, 29)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(18, 33)] / (groebner_matrix[(18, 30)]);
    groebner_matrix[(19, 30)] = groebner_matrix[(13, 30)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(18, 34)] / (groebner_matrix[(18, 30)]);
    groebner_matrix[(19, 31)] = groebner_matrix[(13, 31)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(18, 35)] / (groebner_matrix[(18, 30)]);
    groebner_matrix[(19, 32)] = groebner_matrix[(13, 32)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(19, 33)] = groebner_matrix[(13, 33)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(19, 34)] = groebner_matrix[(13, 34)] / (groebner_matrix[(13, 20)]);
    groebner_matrix[(19, 35)] = groebner_matrix[(13, 35)] / (groebner_matrix[(13, 20)])
        - groebner_matrix[(18, 36)] / (groebner_matrix[(18, 30)]);
    groebner_matrix[(19, 36)] = groebner_matrix[(13, 36)] / (groebner_matrix[(13, 20)]);
}

pub fn s_polynomial20(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(20, 20)] = groebner_matrix[(12, 20)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(17, 30)] / (groebner_matrix[(17, 29)]);
    groebner_matrix[(20, 21)] = groebner_matrix[(12, 21)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(17, 31)] / (groebner_matrix[(17, 29)]);
    groebner_matrix[(20, 27)] = groebner_matrix[(12, 27)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(20, 28)] = groebner_matrix[(12, 28)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(17, 32)] / (groebner_matrix[(17, 29)]);
    groebner_matrix[(20, 29)] = groebner_matrix[(12, 29)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(17, 33)] / (groebner_matrix[(17, 29)]);
    groebner_matrix[(20, 30)] = groebner_matrix[(12, 30)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(17, 34)] / (groebner_matrix[(17, 29)]);
    groebner_matrix[(20, 31)] = groebner_matrix[(12, 31)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(17, 35)] / (groebner_matrix[(17, 29)]);
    groebner_matrix[(20, 32)] = groebner_matrix[(12, 32)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(20, 33)] = groebner_matrix[(12, 33)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(20, 34)] = groebner_matrix[(12, 34)] / (groebner_matrix[(12, 19)]);
    groebner_matrix[(20, 35)] = groebner_matrix[(12, 35)] / (groebner_matrix[(12, 19)])
        - groebner_matrix[(17, 36)] / (groebner_matrix[(17, 29)]);
    groebner_matrix[(20, 36)] = groebner_matrix[(12, 36)] / (groebner_matrix[(12, 19)]);
}

pub fn s_polynomial21(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(21, 19)] = groebner_matrix[(11, 19)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 29)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 20)] = groebner_matrix[(11, 20)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 30)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 21)] = groebner_matrix[(11, 21)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 31)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 27)] = groebner_matrix[(11, 27)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(21, 28)] = groebner_matrix[(11, 28)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 32)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 29)] = groebner_matrix[(11, 29)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 33)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 30)] = groebner_matrix[(11, 30)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 34)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 31)] = groebner_matrix[(11, 31)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 35)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 32)] = groebner_matrix[(11, 32)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(21, 33)] = groebner_matrix[(11, 33)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(21, 34)] = groebner_matrix[(11, 34)] / (groebner_matrix[(11, 18)]);
    groebner_matrix[(21, 35)] = groebner_matrix[(11, 35)] / (groebner_matrix[(11, 18)])
        - groebner_matrix[(16, 36)] / (groebner_matrix[(16, 28)]);
    groebner_matrix[(21, 36)] = groebner_matrix[(11, 36)] / (groebner_matrix[(11, 18)]);
}

pub fn s_polynomial22(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(22, 18)] = groebner_matrix[(10, 18)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 28)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 19)] = groebner_matrix[(10, 19)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 29)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 20)] = groebner_matrix[(10, 20)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 30)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 21)] = groebner_matrix[(10, 21)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 31)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 27)] = groebner_matrix[(10, 27)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(22, 28)] = groebner_matrix[(10, 28)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 32)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 29)] = groebner_matrix[(10, 29)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 33)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 30)] = groebner_matrix[(10, 30)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 34)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 31)] = groebner_matrix[(10, 31)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 35)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 32)] = groebner_matrix[(10, 32)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(22, 33)] = groebner_matrix[(10, 33)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(22, 34)] = groebner_matrix[(10, 34)] / (groebner_matrix[(10, 17)]);
    groebner_matrix[(22, 35)] = groebner_matrix[(10, 35)] / (groebner_matrix[(10, 17)])
        - groebner_matrix[(15, 36)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(22, 36)] = groebner_matrix[(10, 36)] / (groebner_matrix[(10, 17)]);
}

pub fn s_polynomial23(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(23, 15)] = -groebner_matrix[(15, 28)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 16)] = -groebner_matrix[(15, 29)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 17)] = groebner_matrix[(9, 17)] / (groebner_matrix[(9, 11)])
        - groebner_matrix[(15, 30)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 18)] = groebner_matrix[(9, 18)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 19)] = groebner_matrix[(9, 19)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 20)] = groebner_matrix[(9, 20)] / (groebner_matrix[(9, 11)])
        - groebner_matrix[(15, 31)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 21)] = groebner_matrix[(9, 21)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 25)] = -groebner_matrix[(15, 32)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 26)] = -groebner_matrix[(15, 33)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 27)] = groebner_matrix[(9, 27)] / (groebner_matrix[(9, 11)])
        - groebner_matrix[(15, 34)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 28)] = groebner_matrix[(9, 28)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 29)] = groebner_matrix[(9, 29)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 30)] = groebner_matrix[(9, 30)] / (groebner_matrix[(9, 11)])
        - groebner_matrix[(15, 35)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 31)] = groebner_matrix[(9, 31)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 32)] = groebner_matrix[(9, 32)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 33)] = groebner_matrix[(9, 33)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 34)] = groebner_matrix[(9, 34)] / (groebner_matrix[(9, 11)])
        - groebner_matrix[(15, 36)] / (groebner_matrix[(15, 27)]);
    groebner_matrix[(23, 35)] = groebner_matrix[(9, 35)] / (groebner_matrix[(9, 11)]);
    groebner_matrix[(23, 36)] = groebner_matrix[(9, 36)] / (groebner_matrix[(9, 11)]);
}

pub fn s_polynomial24(groebner_matrix: &mut SMatrix<f64, 25, 37>) {
    groebner_matrix[(24, 32)] = groebner_matrix[(19, 32)] / (groebner_matrix[(19, 31)]);
    groebner_matrix[(24, 33)] = groebner_matrix[(19, 33)] / (groebner_matrix[(19, 31)]);
    groebner_matrix[(24, 34)] = groebner_matrix[(19, 34)] / (groebner_matrix[(19, 31)]);
    groebner_matrix[(24, 35)] = groebner_matrix[(19, 35)] / (groebner_matrix[(19, 31)])
        - groebner_matrix[(23, 36)] / (groebner_matrix[(23, 35)]);
    groebner_matrix[(24, 36)] = groebner_matrix[(19, 36)] / (groebner_matrix[(19, 31)]);
}
