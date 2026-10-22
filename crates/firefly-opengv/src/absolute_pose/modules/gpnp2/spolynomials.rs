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

pub fn s_polynomial5(groebner_matrix: &mut SMatrix<f64, 10, 6>) {
    groebner_matrix[(5, 1)] = groebner_matrix[(0, 1)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 1)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(5, 2)] = groebner_matrix[(0, 2)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 2)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(5, 3)] = groebner_matrix[(0, 3)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 3)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(5, 4)] = groebner_matrix[(0, 4)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 4)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(5, 5)] = groebner_matrix[(0, 5)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 5)] / (groebner_matrix[(1, 0)]);
}

pub fn s_polynomial6(groebner_matrix: &mut SMatrix<f64, 10, 6>) {
    groebner_matrix[(6, 1)] = groebner_matrix[(1, 1)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 1)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(6, 2)] = groebner_matrix[(1, 2)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 2)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(6, 3)] = groebner_matrix[(1, 3)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 3)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(6, 4)] = groebner_matrix[(1, 4)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 4)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(6, 5)] = groebner_matrix[(1, 5)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 5)] / (groebner_matrix[(2, 0)]);
}

pub fn s_polynomial7(groebner_matrix: &mut SMatrix<f64, 10, 6>) {
    groebner_matrix[(7, 1)] = groebner_matrix[(2, 1)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 1)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(7, 2)] = groebner_matrix[(2, 2)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 2)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(7, 3)] = groebner_matrix[(2, 3)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 3)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(7, 4)] = groebner_matrix[(2, 4)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 4)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(7, 5)] = groebner_matrix[(2, 5)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 5)] / (groebner_matrix[(3, 0)]);
}

pub fn s_polynomial8(groebner_matrix: &mut SMatrix<f64, 10, 6>) {
    groebner_matrix[(8, 1)] = groebner_matrix[(3, 1)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 1)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(8, 2)] = groebner_matrix[(3, 2)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 2)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(8, 3)] = groebner_matrix[(3, 3)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 3)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(8, 4)] = groebner_matrix[(3, 4)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 4)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(8, 5)] = groebner_matrix[(3, 5)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 5)] / (groebner_matrix[(4, 0)]);
}

pub fn s_polynomial9(groebner_matrix: &mut SMatrix<f64, 10, 6>) {
    groebner_matrix[(9, 3)] = groebner_matrix[(6, 3)] / (groebner_matrix[(6, 2)]);
    groebner_matrix[(9, 4)] = groebner_matrix[(6, 4)] / (groebner_matrix[(6, 2)])
        - groebner_matrix[(8, 5)] / (groebner_matrix[(8, 4)]);
    groebner_matrix[(9, 5)] = groebner_matrix[(6, 5)] / (groebner_matrix[(6, 2)]);
}
