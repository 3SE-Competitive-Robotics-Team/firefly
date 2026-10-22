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
pub fn s_polynomial30(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial30_stage_0(groebner_matrix);
    s_polynomial30_stage_1(groebner_matrix);
    s_polynomial30_stage_2(groebner_matrix);
    s_polynomial30_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial30_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(30, 1)] = groebner_matrix[(0, 1)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 1)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 2)] = groebner_matrix[(0, 2)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 2)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 3)] = groebner_matrix[(0, 3)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 3)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 4)] = groebner_matrix[(0, 4)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 4)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 5)] = groebner_matrix[(0, 5)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 5)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 6)] = groebner_matrix[(0, 6)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 6)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 8)] = groebner_matrix[(0, 8)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 8)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 10)] = groebner_matrix[(0, 10)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 10)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 11)] = groebner_matrix[(0, 11)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 11)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 12)] = groebner_matrix[(0, 12)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 12)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 13)] = groebner_matrix[(0, 13)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 13)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 14)] = groebner_matrix[(0, 14)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 14)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 15)] = groebner_matrix[(0, 15)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 15)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 16)] = groebner_matrix[(0, 16)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 16)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 18)] = groebner_matrix[(0, 18)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 18)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 19)] = groebner_matrix[(0, 19)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 19)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 20)] = groebner_matrix[(0, 20)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 20)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 21)] = groebner_matrix[(0, 21)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 21)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 22)] = groebner_matrix[(0, 22)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 22)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 23)] = groebner_matrix[(0, 23)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 23)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 25)] = groebner_matrix[(0, 25)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 25)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 26)] = groebner_matrix[(0, 26)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 26)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 27)] = groebner_matrix[(0, 27)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 27)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 28)] = groebner_matrix[(0, 28)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 28)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 29)] = groebner_matrix[(0, 29)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 29)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 30)] = groebner_matrix[(0, 30)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 30)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 33)] = groebner_matrix[(0, 33)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 33)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 34)] = groebner_matrix[(0, 34)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 34)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 39)] = groebner_matrix[(0, 39)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 39)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 40)] = groebner_matrix[(0, 40)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 40)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 41)] = groebner_matrix[(0, 41)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 41)] / (groebner_matrix[(1, 0)]);
}

#[inline]
fn s_polynomial30_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(30, 42)] = groebner_matrix[(0, 42)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 42)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 43)] = groebner_matrix[(0, 43)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 43)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 44)] = groebner_matrix[(0, 44)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 44)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 45)] = groebner_matrix[(0, 45)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 45)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 46)] = groebner_matrix[(0, 46)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 46)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 47)] = groebner_matrix[(0, 47)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 47)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 48)] = groebner_matrix[(0, 48)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 48)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 49)] = groebner_matrix[(0, 49)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 49)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 50)] = groebner_matrix[(0, 50)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 50)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 51)] = groebner_matrix[(0, 51)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 51)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 52)] = groebner_matrix[(0, 52)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 52)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 53)] = groebner_matrix[(0, 53)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 53)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 54)] = groebner_matrix[(0, 54)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 54)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 55)] = groebner_matrix[(0, 55)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 55)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 56)] = groebner_matrix[(0, 56)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 56)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 57)] = groebner_matrix[(0, 57)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 57)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 58)] = groebner_matrix[(0, 58)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 58)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 59)] = groebner_matrix[(0, 59)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 59)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 60)] = groebner_matrix[(0, 60)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 60)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 61)] = groebner_matrix[(0, 61)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 61)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 62)] = groebner_matrix[(0, 62)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 62)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 64)] = groebner_matrix[(0, 64)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 64)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 65)] = groebner_matrix[(0, 65)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 65)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 66)] = groebner_matrix[(0, 66)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 66)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 67)] = groebner_matrix[(0, 67)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 67)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 68)] = groebner_matrix[(0, 68)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 68)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 69)] = groebner_matrix[(0, 69)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 69)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 70)] = groebner_matrix[(0, 70)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 70)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 71)] = groebner_matrix[(0, 71)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 71)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 73)] = groebner_matrix[(0, 73)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 73)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 74)] = groebner_matrix[(0, 74)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 74)] / (groebner_matrix[(1, 0)]);
}

#[inline]
fn s_polynomial30_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(30, 76)] = groebner_matrix[(0, 76)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 76)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 77)] = groebner_matrix[(0, 77)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 77)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 78)] = groebner_matrix[(0, 78)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 78)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 79)] = groebner_matrix[(0, 79)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 79)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 80)] = groebner_matrix[(0, 80)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 80)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 81)] = groebner_matrix[(0, 81)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 81)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 82)] = groebner_matrix[(0, 82)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 82)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 83)] = groebner_matrix[(0, 83)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 83)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 84)] = groebner_matrix[(0, 84)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 84)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 85)] = groebner_matrix[(0, 85)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 85)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 86)] = groebner_matrix[(0, 86)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 86)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 87)] = groebner_matrix[(0, 87)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 87)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 89)] = groebner_matrix[(0, 89)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 89)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 91)] = groebner_matrix[(0, 91)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 91)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 92)] = groebner_matrix[(0, 92)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 92)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 94)] = groebner_matrix[(0, 94)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 94)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 97)] = groebner_matrix[(0, 97)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 97)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 98)] = groebner_matrix[(0, 98)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 98)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 99)] = groebner_matrix[(0, 99)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 99)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 100)] = groebner_matrix[(0, 100)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 100)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 101)] = groebner_matrix[(0, 101)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 101)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 103)] = groebner_matrix[(0, 103)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 103)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 104)] = groebner_matrix[(0, 104)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 104)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 105)] = groebner_matrix[(0, 105)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 105)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 106)] = groebner_matrix[(0, 106)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 106)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 107)] = groebner_matrix[(0, 107)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 107)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 108)] = groebner_matrix[(0, 108)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 108)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 109)] = groebner_matrix[(0, 109)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 109)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 110)] = groebner_matrix[(0, 110)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 110)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 111)] = groebner_matrix[(0, 111)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 111)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 112)] = groebner_matrix[(0, 112)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 112)] / (groebner_matrix[(1, 0)]);
}

#[inline]
fn s_polynomial30_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(30, 113)] = groebner_matrix[(0, 113)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 113)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 115)] = groebner_matrix[(0, 115)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 115)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 116)] = groebner_matrix[(0, 116)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 116)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 118)] = groebner_matrix[(0, 118)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 118)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 119)] = groebner_matrix[(0, 119)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 119)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 120)] = groebner_matrix[(0, 120)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 120)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 121)] = groebner_matrix[(0, 121)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 121)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 122)] = groebner_matrix[(0, 122)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 122)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 123)] = groebner_matrix[(0, 123)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 123)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 125)] = groebner_matrix[(0, 125)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 125)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 126)] = groebner_matrix[(0, 126)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 126)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 127)] = groebner_matrix[(0, 127)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 127)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 128)] = groebner_matrix[(0, 128)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 128)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 129)] = groebner_matrix[(0, 129)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 129)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 130)] = groebner_matrix[(0, 130)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 130)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 133)] = groebner_matrix[(0, 133)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 133)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 134)] = groebner_matrix[(0, 134)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 134)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 136)] = groebner_matrix[(0, 136)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 136)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(30, 137)] = groebner_matrix[(0, 137)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 137)] / (groebner_matrix[(1, 0)]);
}

pub fn s_polynomial31(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial31_stage_0(groebner_matrix);
    s_polynomial31_stage_1(groebner_matrix);
    s_polynomial31_stage_2(groebner_matrix);
    s_polynomial31_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial31_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(31, 1)] = groebner_matrix[(1, 1)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 1)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 2)] = groebner_matrix[(1, 2)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 2)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 3)] = groebner_matrix[(1, 3)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 3)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 4)] = groebner_matrix[(1, 4)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 4)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 5)] = groebner_matrix[(1, 5)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 5)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 6)] = groebner_matrix[(1, 6)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 6)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 8)] = groebner_matrix[(1, 8)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 8)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 10)] = groebner_matrix[(1, 10)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 10)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 11)] = groebner_matrix[(1, 11)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 11)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 12)] = groebner_matrix[(1, 12)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 12)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 13)] = groebner_matrix[(1, 13)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 13)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 14)] = groebner_matrix[(1, 14)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 14)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 15)] = groebner_matrix[(1, 15)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 15)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 16)] = groebner_matrix[(1, 16)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 16)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 18)] = groebner_matrix[(1, 18)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 18)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 19)] = groebner_matrix[(1, 19)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 19)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 20)] = groebner_matrix[(1, 20)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 20)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 21)] = groebner_matrix[(1, 21)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 21)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 22)] = groebner_matrix[(1, 22)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 22)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 23)] = groebner_matrix[(1, 23)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 23)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 25)] = groebner_matrix[(1, 25)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 25)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 26)] = groebner_matrix[(1, 26)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 26)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 27)] = groebner_matrix[(1, 27)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 27)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 28)] = groebner_matrix[(1, 28)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 28)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 29)] = groebner_matrix[(1, 29)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 29)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 30)] = groebner_matrix[(1, 30)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 30)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 33)] = groebner_matrix[(1, 33)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 33)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 34)] = groebner_matrix[(1, 34)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 34)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 39)] = groebner_matrix[(1, 39)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 39)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 40)] = groebner_matrix[(1, 40)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 40)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 41)] = groebner_matrix[(1, 41)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 41)] / (groebner_matrix[(2, 0)]);
}

#[inline]
fn s_polynomial31_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(31, 42)] = groebner_matrix[(1, 42)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 42)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 43)] = groebner_matrix[(1, 43)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 43)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 44)] = groebner_matrix[(1, 44)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 44)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 45)] = groebner_matrix[(1, 45)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 45)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 46)] = groebner_matrix[(1, 46)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 46)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 47)] = groebner_matrix[(1, 47)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 47)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 48)] = groebner_matrix[(1, 48)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 48)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 49)] = groebner_matrix[(1, 49)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 49)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 50)] = groebner_matrix[(1, 50)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 50)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 51)] = groebner_matrix[(1, 51)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 51)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 52)] = groebner_matrix[(1, 52)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 52)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 53)] = groebner_matrix[(1, 53)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 53)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 54)] = groebner_matrix[(1, 54)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 54)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 55)] = groebner_matrix[(1, 55)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 55)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 56)] = groebner_matrix[(1, 56)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 56)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 57)] = groebner_matrix[(1, 57)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 57)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 58)] = groebner_matrix[(1, 58)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 58)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 59)] = groebner_matrix[(1, 59)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 59)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 60)] = groebner_matrix[(1, 60)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 60)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 61)] = groebner_matrix[(1, 61)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 61)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 62)] = groebner_matrix[(1, 62)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 62)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 64)] = groebner_matrix[(1, 64)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 64)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 65)] = groebner_matrix[(1, 65)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 65)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 66)] = groebner_matrix[(1, 66)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 66)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 67)] = groebner_matrix[(1, 67)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 67)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 68)] = groebner_matrix[(1, 68)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 68)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 69)] = groebner_matrix[(1, 69)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 69)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 70)] = groebner_matrix[(1, 70)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 70)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 71)] = groebner_matrix[(1, 71)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 71)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 73)] = groebner_matrix[(1, 73)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 73)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 74)] = groebner_matrix[(1, 74)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 74)] / (groebner_matrix[(2, 0)]);
}

#[inline]
fn s_polynomial31_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(31, 76)] = groebner_matrix[(1, 76)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 76)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 77)] = groebner_matrix[(1, 77)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 77)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 78)] = groebner_matrix[(1, 78)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 78)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 79)] = groebner_matrix[(1, 79)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 79)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 80)] = groebner_matrix[(1, 80)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 80)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 81)] = groebner_matrix[(1, 81)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 81)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 82)] = groebner_matrix[(1, 82)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 82)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 83)] = groebner_matrix[(1, 83)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 83)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 84)] = groebner_matrix[(1, 84)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 84)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 85)] = groebner_matrix[(1, 85)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 85)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 86)] = groebner_matrix[(1, 86)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 86)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 87)] = groebner_matrix[(1, 87)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 87)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 89)] = groebner_matrix[(1, 89)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 89)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 91)] = groebner_matrix[(1, 91)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 91)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 92)] = groebner_matrix[(1, 92)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 92)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 94)] = groebner_matrix[(1, 94)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 94)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 97)] = groebner_matrix[(1, 97)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 97)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 98)] = groebner_matrix[(1, 98)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 98)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 99)] = groebner_matrix[(1, 99)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 99)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 100)] = groebner_matrix[(1, 100)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 100)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 101)] = groebner_matrix[(1, 101)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 101)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 103)] = groebner_matrix[(1, 103)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 103)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 104)] = groebner_matrix[(1, 104)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 104)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 105)] = groebner_matrix[(1, 105)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 105)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 106)] = groebner_matrix[(1, 106)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 106)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 107)] = groebner_matrix[(1, 107)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 107)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 108)] = groebner_matrix[(1, 108)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 108)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 109)] = groebner_matrix[(1, 109)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 109)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 110)] = groebner_matrix[(1, 110)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 110)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 111)] = groebner_matrix[(1, 111)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 111)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 112)] = groebner_matrix[(1, 112)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 112)] / (groebner_matrix[(2, 0)]);
}

#[inline]
fn s_polynomial31_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(31, 113)] = groebner_matrix[(1, 113)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 113)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 115)] = groebner_matrix[(1, 115)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 115)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 116)] = groebner_matrix[(1, 116)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 116)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 118)] = groebner_matrix[(1, 118)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 118)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 119)] = groebner_matrix[(1, 119)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 119)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 120)] = groebner_matrix[(1, 120)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 120)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 121)] = groebner_matrix[(1, 121)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 121)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 122)] = groebner_matrix[(1, 122)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 122)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 123)] = groebner_matrix[(1, 123)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 123)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 125)] = groebner_matrix[(1, 125)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 125)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 126)] = groebner_matrix[(1, 126)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 126)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 127)] = groebner_matrix[(1, 127)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 127)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 128)] = groebner_matrix[(1, 128)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 128)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 129)] = groebner_matrix[(1, 129)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 129)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 130)] = groebner_matrix[(1, 130)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 130)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 133)] = groebner_matrix[(1, 133)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 133)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 134)] = groebner_matrix[(1, 134)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 134)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 136)] = groebner_matrix[(1, 136)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 136)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(31, 137)] = groebner_matrix[(1, 137)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 137)] / (groebner_matrix[(2, 0)]);
}

pub fn s_polynomial32(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial32_stage_0(groebner_matrix);
    s_polynomial32_stage_1(groebner_matrix);
    s_polynomial32_stage_2(groebner_matrix);
    s_polynomial32_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial32_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(32, 1)] = groebner_matrix[(2, 1)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 1)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 2)] = groebner_matrix[(2, 2)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 2)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 3)] = groebner_matrix[(2, 3)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 3)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 4)] = groebner_matrix[(2, 4)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 4)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 5)] = groebner_matrix[(2, 5)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 5)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 6)] = groebner_matrix[(2, 6)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 6)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 8)] = groebner_matrix[(2, 8)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 8)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 10)] = groebner_matrix[(2, 10)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 10)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 11)] = groebner_matrix[(2, 11)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 11)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 12)] = groebner_matrix[(2, 12)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 12)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 13)] = groebner_matrix[(2, 13)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 13)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 14)] = groebner_matrix[(2, 14)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 14)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 15)] = groebner_matrix[(2, 15)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 15)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 16)] = groebner_matrix[(2, 16)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 16)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 18)] = groebner_matrix[(2, 18)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 18)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 19)] = groebner_matrix[(2, 19)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 19)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 20)] = groebner_matrix[(2, 20)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 20)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 21)] = groebner_matrix[(2, 21)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 21)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 22)] = groebner_matrix[(2, 22)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 22)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 23)] = groebner_matrix[(2, 23)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 23)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 25)] = groebner_matrix[(2, 25)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 25)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 26)] = groebner_matrix[(2, 26)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 26)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 27)] = groebner_matrix[(2, 27)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 27)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 28)] = groebner_matrix[(2, 28)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 28)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 29)] = groebner_matrix[(2, 29)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 29)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 30)] = groebner_matrix[(2, 30)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 30)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 33)] = groebner_matrix[(2, 33)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 33)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 34)] = groebner_matrix[(2, 34)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 34)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 39)] = groebner_matrix[(2, 39)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 39)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 40)] = groebner_matrix[(2, 40)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 40)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 41)] = groebner_matrix[(2, 41)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 41)] / (groebner_matrix[(3, 0)]);
}

#[inline]
fn s_polynomial32_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(32, 42)] = groebner_matrix[(2, 42)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 42)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 43)] = groebner_matrix[(2, 43)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 43)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 44)] = groebner_matrix[(2, 44)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 44)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 45)] = groebner_matrix[(2, 45)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 45)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 46)] = groebner_matrix[(2, 46)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 46)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 47)] = groebner_matrix[(2, 47)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 47)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 48)] = groebner_matrix[(2, 48)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 48)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 49)] = groebner_matrix[(2, 49)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 49)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 50)] = groebner_matrix[(2, 50)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 50)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 51)] = groebner_matrix[(2, 51)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 51)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 52)] = groebner_matrix[(2, 52)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 52)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 53)] = groebner_matrix[(2, 53)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 53)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 54)] = groebner_matrix[(2, 54)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 54)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 55)] = groebner_matrix[(2, 55)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 55)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 56)] = groebner_matrix[(2, 56)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 56)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 57)] = groebner_matrix[(2, 57)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 57)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 58)] = groebner_matrix[(2, 58)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 58)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 59)] = groebner_matrix[(2, 59)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 59)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 60)] = groebner_matrix[(2, 60)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 60)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 61)] = groebner_matrix[(2, 61)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 61)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 62)] = groebner_matrix[(2, 62)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 62)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 64)] = groebner_matrix[(2, 64)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 64)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 65)] = groebner_matrix[(2, 65)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 65)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 66)] = groebner_matrix[(2, 66)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 66)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 67)] = groebner_matrix[(2, 67)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 67)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 68)] = groebner_matrix[(2, 68)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 68)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 69)] = groebner_matrix[(2, 69)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 69)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 70)] = groebner_matrix[(2, 70)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 70)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 71)] = groebner_matrix[(2, 71)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 71)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 73)] = groebner_matrix[(2, 73)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 73)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 74)] = groebner_matrix[(2, 74)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 74)] / (groebner_matrix[(3, 0)]);
}

#[inline]
fn s_polynomial32_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(32, 76)] = groebner_matrix[(2, 76)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 76)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 77)] = groebner_matrix[(2, 77)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 77)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 78)] = groebner_matrix[(2, 78)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 78)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 79)] = groebner_matrix[(2, 79)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 79)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 80)] = groebner_matrix[(2, 80)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 80)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 81)] = groebner_matrix[(2, 81)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 81)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 82)] = groebner_matrix[(2, 82)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 82)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 83)] = groebner_matrix[(2, 83)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 83)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 84)] = groebner_matrix[(2, 84)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 84)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 85)] = groebner_matrix[(2, 85)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 85)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 86)] = groebner_matrix[(2, 86)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 86)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 87)] = groebner_matrix[(2, 87)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 87)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 89)] = groebner_matrix[(2, 89)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 89)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 91)] = groebner_matrix[(2, 91)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 91)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 92)] = groebner_matrix[(2, 92)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 92)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 94)] = groebner_matrix[(2, 94)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 94)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 97)] = groebner_matrix[(2, 97)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 97)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 98)] = groebner_matrix[(2, 98)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 98)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 99)] = groebner_matrix[(2, 99)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 99)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 100)] = groebner_matrix[(2, 100)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 100)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 101)] = groebner_matrix[(2, 101)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 101)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 103)] = groebner_matrix[(2, 103)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 103)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 104)] = groebner_matrix[(2, 104)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 104)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 105)] = groebner_matrix[(2, 105)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 105)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 106)] = groebner_matrix[(2, 106)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 106)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 107)] = groebner_matrix[(2, 107)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 107)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 108)] = groebner_matrix[(2, 108)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 108)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 109)] = groebner_matrix[(2, 109)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 109)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 110)] = groebner_matrix[(2, 110)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 110)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 111)] = groebner_matrix[(2, 111)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 111)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 112)] = groebner_matrix[(2, 112)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 112)] / (groebner_matrix[(3, 0)]);
}

#[inline]
fn s_polynomial32_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(32, 113)] = groebner_matrix[(2, 113)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 113)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 115)] = groebner_matrix[(2, 115)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 115)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 116)] = groebner_matrix[(2, 116)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 116)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 118)] = groebner_matrix[(2, 118)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 118)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 119)] = groebner_matrix[(2, 119)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 119)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 120)] = groebner_matrix[(2, 120)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 120)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 121)] = groebner_matrix[(2, 121)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 121)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 122)] = groebner_matrix[(2, 122)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 122)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 123)] = groebner_matrix[(2, 123)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 123)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 125)] = groebner_matrix[(2, 125)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 125)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 126)] = groebner_matrix[(2, 126)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 126)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 127)] = groebner_matrix[(2, 127)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 127)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 128)] = groebner_matrix[(2, 128)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 128)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 129)] = groebner_matrix[(2, 129)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 129)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 130)] = groebner_matrix[(2, 130)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 130)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 133)] = groebner_matrix[(2, 133)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 133)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 134)] = groebner_matrix[(2, 134)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 134)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 136)] = groebner_matrix[(2, 136)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 136)] / (groebner_matrix[(3, 0)]);
    groebner_matrix[(32, 137)] = groebner_matrix[(2, 137)] / (groebner_matrix[(2, 0)])
        - groebner_matrix[(3, 137)] / (groebner_matrix[(3, 0)]);
}

pub fn s_polynomial33(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial33_stage_0(groebner_matrix);
    s_polynomial33_stage_1(groebner_matrix);
    s_polynomial33_stage_2(groebner_matrix);
    s_polynomial33_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial33_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(33, 1)] = groebner_matrix[(3, 1)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 1)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 2)] = groebner_matrix[(3, 2)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 2)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 3)] = groebner_matrix[(3, 3)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 3)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 4)] = groebner_matrix[(3, 4)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 4)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 5)] = groebner_matrix[(3, 5)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 5)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 6)] = groebner_matrix[(3, 6)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 6)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 8)] = groebner_matrix[(3, 8)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 8)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 10)] = groebner_matrix[(3, 10)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 10)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 11)] = groebner_matrix[(3, 11)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 11)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 12)] = groebner_matrix[(3, 12)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 12)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 13)] = groebner_matrix[(3, 13)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 13)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 14)] = groebner_matrix[(3, 14)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 14)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 15)] = groebner_matrix[(3, 15)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 15)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 16)] = groebner_matrix[(3, 16)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 16)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 18)] = groebner_matrix[(3, 18)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 18)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 19)] = groebner_matrix[(3, 19)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 19)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 20)] = groebner_matrix[(3, 20)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 20)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 21)] = groebner_matrix[(3, 21)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 21)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 22)] = groebner_matrix[(3, 22)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 22)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 23)] = groebner_matrix[(3, 23)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 23)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 25)] = groebner_matrix[(3, 25)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 25)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 26)] = groebner_matrix[(3, 26)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 26)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 27)] = groebner_matrix[(3, 27)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 27)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 28)] = groebner_matrix[(3, 28)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 28)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 29)] = groebner_matrix[(3, 29)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 29)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 30)] = groebner_matrix[(3, 30)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 30)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 33)] = groebner_matrix[(3, 33)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 33)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 34)] = groebner_matrix[(3, 34)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 34)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 39)] = groebner_matrix[(3, 39)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 39)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 40)] = groebner_matrix[(3, 40)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 40)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 41)] = groebner_matrix[(3, 41)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 41)] / (groebner_matrix[(4, 0)]);
}

#[inline]
fn s_polynomial33_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(33, 42)] = groebner_matrix[(3, 42)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 42)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 43)] = groebner_matrix[(3, 43)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 43)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 44)] = groebner_matrix[(3, 44)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 44)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 45)] = groebner_matrix[(3, 45)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 45)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 46)] = groebner_matrix[(3, 46)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 46)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 47)] = groebner_matrix[(3, 47)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 47)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 48)] = groebner_matrix[(3, 48)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 48)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 49)] = groebner_matrix[(3, 49)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 49)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 50)] = groebner_matrix[(3, 50)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 50)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 51)] = groebner_matrix[(3, 51)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 51)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 52)] = groebner_matrix[(3, 52)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 52)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 53)] = groebner_matrix[(3, 53)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 53)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 54)] = groebner_matrix[(3, 54)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 54)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 55)] = groebner_matrix[(3, 55)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 55)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 56)] = groebner_matrix[(3, 56)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 56)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 57)] = groebner_matrix[(3, 57)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 57)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 58)] = groebner_matrix[(3, 58)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 58)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 59)] = groebner_matrix[(3, 59)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 59)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 60)] = groebner_matrix[(3, 60)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 60)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 61)] = groebner_matrix[(3, 61)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 61)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 62)] = groebner_matrix[(3, 62)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 62)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 64)] = groebner_matrix[(3, 64)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 64)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 65)] = groebner_matrix[(3, 65)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 65)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 66)] = groebner_matrix[(3, 66)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 66)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 67)] = groebner_matrix[(3, 67)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 67)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 68)] = groebner_matrix[(3, 68)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 68)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 69)] = groebner_matrix[(3, 69)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 69)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 70)] = groebner_matrix[(3, 70)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 70)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 71)] = groebner_matrix[(3, 71)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 71)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 73)] = groebner_matrix[(3, 73)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 73)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 74)] = groebner_matrix[(3, 74)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 74)] / (groebner_matrix[(4, 0)]);
}

#[inline]
fn s_polynomial33_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(33, 76)] = groebner_matrix[(3, 76)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 76)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 77)] = groebner_matrix[(3, 77)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 77)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 78)] = groebner_matrix[(3, 78)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 78)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 79)] = groebner_matrix[(3, 79)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 79)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 80)] = groebner_matrix[(3, 80)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 80)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 81)] = groebner_matrix[(3, 81)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 81)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 82)] = groebner_matrix[(3, 82)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 82)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 83)] = groebner_matrix[(3, 83)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 83)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 84)] = groebner_matrix[(3, 84)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 84)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 85)] = groebner_matrix[(3, 85)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 85)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 86)] = groebner_matrix[(3, 86)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 86)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 87)] = groebner_matrix[(3, 87)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 87)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 89)] = groebner_matrix[(3, 89)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 89)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 91)] = groebner_matrix[(3, 91)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 91)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 92)] = groebner_matrix[(3, 92)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 92)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 94)] = groebner_matrix[(3, 94)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 94)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 97)] = groebner_matrix[(3, 97)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 97)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 98)] = groebner_matrix[(3, 98)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 98)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 99)] = groebner_matrix[(3, 99)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 99)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 100)] = groebner_matrix[(3, 100)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 100)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 101)] = groebner_matrix[(3, 101)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 101)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 103)] = groebner_matrix[(3, 103)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 103)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 104)] = groebner_matrix[(3, 104)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 104)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 105)] = groebner_matrix[(3, 105)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 105)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 106)] = groebner_matrix[(3, 106)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 106)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 107)] = groebner_matrix[(3, 107)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 107)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 108)] = groebner_matrix[(3, 108)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 108)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 109)] = groebner_matrix[(3, 109)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 109)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 110)] = groebner_matrix[(3, 110)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 110)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 111)] = groebner_matrix[(3, 111)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 111)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 112)] = groebner_matrix[(3, 112)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 112)] / (groebner_matrix[(4, 0)]);
}

#[inline]
fn s_polynomial33_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(33, 113)] = groebner_matrix[(3, 113)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 113)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 115)] = groebner_matrix[(3, 115)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 115)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 116)] = groebner_matrix[(3, 116)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 116)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 118)] = groebner_matrix[(3, 118)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 118)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 119)] = groebner_matrix[(3, 119)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 119)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 120)] = groebner_matrix[(3, 120)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 120)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 121)] = groebner_matrix[(3, 121)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 121)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 122)] = groebner_matrix[(3, 122)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 122)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 123)] = groebner_matrix[(3, 123)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 123)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 125)] = groebner_matrix[(3, 125)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 125)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 126)] = groebner_matrix[(3, 126)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 126)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 127)] = groebner_matrix[(3, 127)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 127)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 128)] = groebner_matrix[(3, 128)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 128)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 129)] = groebner_matrix[(3, 129)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 129)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 130)] = groebner_matrix[(3, 130)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 130)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 133)] = groebner_matrix[(3, 133)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 133)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 134)] = groebner_matrix[(3, 134)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 134)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 136)] = groebner_matrix[(3, 136)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 136)] / (groebner_matrix[(4, 0)]);
    groebner_matrix[(33, 137)] = groebner_matrix[(3, 137)] / (groebner_matrix[(3, 0)])
        - groebner_matrix[(4, 137)] / (groebner_matrix[(4, 0)]);
}

pub fn s_polynomial34(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial34_stage_0(groebner_matrix);
    s_polynomial34_stage_1(groebner_matrix);
    s_polynomial34_stage_2(groebner_matrix);
    s_polynomial34_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial34_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(34, 1)] = groebner_matrix[(4, 1)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 1)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 2)] = groebner_matrix[(4, 2)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 2)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 3)] = groebner_matrix[(4, 3)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 3)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 4)] = groebner_matrix[(4, 4)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 4)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 5)] = groebner_matrix[(4, 5)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 5)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 6)] = groebner_matrix[(4, 6)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 6)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 8)] = groebner_matrix[(4, 8)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 8)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 10)] = groebner_matrix[(4, 10)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 10)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 11)] = groebner_matrix[(4, 11)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 11)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 12)] = groebner_matrix[(4, 12)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 12)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 13)] = groebner_matrix[(4, 13)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 13)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 14)] = groebner_matrix[(4, 14)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 14)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 15)] = groebner_matrix[(4, 15)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 15)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 16)] = groebner_matrix[(4, 16)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 16)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 18)] = groebner_matrix[(4, 18)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 18)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 19)] = groebner_matrix[(4, 19)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 19)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 20)] = groebner_matrix[(4, 20)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 20)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 21)] = groebner_matrix[(4, 21)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 21)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 22)] = groebner_matrix[(4, 22)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 22)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 23)] = groebner_matrix[(4, 23)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 23)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 25)] = groebner_matrix[(4, 25)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 25)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 26)] = groebner_matrix[(4, 26)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 26)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 27)] = groebner_matrix[(4, 27)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 27)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 28)] = groebner_matrix[(4, 28)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 28)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 29)] = groebner_matrix[(4, 29)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 29)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 30)] = groebner_matrix[(4, 30)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 30)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 33)] = groebner_matrix[(4, 33)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 33)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 34)] = groebner_matrix[(4, 34)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 34)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 39)] = groebner_matrix[(4, 39)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 39)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 40)] = groebner_matrix[(4, 40)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 40)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 41)] = groebner_matrix[(4, 41)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 41)] / (groebner_matrix[(5, 0)]);
}

#[inline]
fn s_polynomial34_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(34, 42)] = groebner_matrix[(4, 42)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 42)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 43)] = groebner_matrix[(4, 43)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 43)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 44)] = groebner_matrix[(4, 44)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 44)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 45)] = groebner_matrix[(4, 45)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 45)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 46)] = groebner_matrix[(4, 46)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 46)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 47)] = groebner_matrix[(4, 47)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 47)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 48)] = groebner_matrix[(4, 48)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 48)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 49)] = groebner_matrix[(4, 49)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 49)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 50)] = groebner_matrix[(4, 50)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 50)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 51)] = groebner_matrix[(4, 51)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 51)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 52)] = groebner_matrix[(4, 52)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 52)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 53)] = groebner_matrix[(4, 53)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 53)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 54)] = groebner_matrix[(4, 54)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 54)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 55)] = groebner_matrix[(4, 55)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 55)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 56)] = groebner_matrix[(4, 56)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 56)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 57)] = groebner_matrix[(4, 57)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 57)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 58)] = groebner_matrix[(4, 58)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 58)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 59)] = groebner_matrix[(4, 59)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 59)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 60)] = groebner_matrix[(4, 60)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 60)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 61)] = groebner_matrix[(4, 61)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 61)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 62)] = groebner_matrix[(4, 62)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 62)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 64)] = groebner_matrix[(4, 64)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 64)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 65)] = groebner_matrix[(4, 65)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 65)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 66)] = groebner_matrix[(4, 66)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 66)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 67)] = groebner_matrix[(4, 67)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 67)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 68)] = groebner_matrix[(4, 68)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 68)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 69)] = groebner_matrix[(4, 69)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 69)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 70)] = groebner_matrix[(4, 70)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 70)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 71)] = groebner_matrix[(4, 71)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 71)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 73)] = groebner_matrix[(4, 73)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 73)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 74)] = groebner_matrix[(4, 74)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 74)] / (groebner_matrix[(5, 0)]);
}

#[inline]
fn s_polynomial34_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(34, 76)] = groebner_matrix[(4, 76)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 76)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 77)] = groebner_matrix[(4, 77)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 77)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 78)] = groebner_matrix[(4, 78)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 78)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 79)] = groebner_matrix[(4, 79)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 79)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 80)] = groebner_matrix[(4, 80)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 80)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 81)] = groebner_matrix[(4, 81)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 81)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 82)] = groebner_matrix[(4, 82)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 82)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 83)] = groebner_matrix[(4, 83)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 83)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 84)] = groebner_matrix[(4, 84)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 84)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 85)] = groebner_matrix[(4, 85)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 85)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 86)] = groebner_matrix[(4, 86)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 86)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 87)] = groebner_matrix[(4, 87)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 87)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 89)] = groebner_matrix[(4, 89)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 89)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 91)] = groebner_matrix[(4, 91)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 91)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 92)] = groebner_matrix[(4, 92)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 92)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 94)] = groebner_matrix[(4, 94)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 94)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 97)] = groebner_matrix[(4, 97)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 97)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 98)] = groebner_matrix[(4, 98)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 98)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 99)] = groebner_matrix[(4, 99)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 99)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 100)] = groebner_matrix[(4, 100)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 100)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 101)] = groebner_matrix[(4, 101)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 101)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 103)] = groebner_matrix[(4, 103)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 103)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 104)] = groebner_matrix[(4, 104)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 104)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 105)] = groebner_matrix[(4, 105)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 105)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 106)] = groebner_matrix[(4, 106)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 106)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 107)] = groebner_matrix[(4, 107)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 107)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 108)] = groebner_matrix[(4, 108)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 108)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 109)] = groebner_matrix[(4, 109)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 109)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 110)] = groebner_matrix[(4, 110)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 110)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 111)] = groebner_matrix[(4, 111)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 111)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 112)] = groebner_matrix[(4, 112)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 112)] / (groebner_matrix[(5, 0)]);
}

#[inline]
fn s_polynomial34_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(34, 113)] = groebner_matrix[(4, 113)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 113)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 115)] = groebner_matrix[(4, 115)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 115)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 116)] = groebner_matrix[(4, 116)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 116)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 118)] = groebner_matrix[(4, 118)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 118)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 119)] = groebner_matrix[(4, 119)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 119)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 120)] = groebner_matrix[(4, 120)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 120)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 121)] = groebner_matrix[(4, 121)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 121)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 122)] = groebner_matrix[(4, 122)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 122)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 123)] = groebner_matrix[(4, 123)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 123)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 125)] = groebner_matrix[(4, 125)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 125)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 126)] = groebner_matrix[(4, 126)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 126)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 127)] = groebner_matrix[(4, 127)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 127)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 128)] = groebner_matrix[(4, 128)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 128)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 129)] = groebner_matrix[(4, 129)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 129)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 130)] = groebner_matrix[(4, 130)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 130)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 133)] = groebner_matrix[(4, 133)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 133)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 134)] = groebner_matrix[(4, 134)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 134)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 136)] = groebner_matrix[(4, 136)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 136)] / (groebner_matrix[(5, 0)]);
    groebner_matrix[(34, 137)] = groebner_matrix[(4, 137)] / (groebner_matrix[(4, 0)])
        - groebner_matrix[(5, 137)] / (groebner_matrix[(5, 0)]);
}

pub fn s_polynomial35(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial35_stage_0(groebner_matrix);
    s_polynomial35_stage_1(groebner_matrix);
    s_polynomial35_stage_2(groebner_matrix);
    s_polynomial35_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial35_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(35, 1)] = groebner_matrix[(5, 1)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 1)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 2)] = groebner_matrix[(5, 2)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 2)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 3)] = groebner_matrix[(5, 3)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 3)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 4)] = groebner_matrix[(5, 4)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 4)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 5)] = groebner_matrix[(5, 5)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 5)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 6)] = groebner_matrix[(5, 6)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 6)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 8)] = groebner_matrix[(5, 8)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 8)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 10)] = groebner_matrix[(5, 10)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 10)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 11)] = groebner_matrix[(5, 11)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 11)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 12)] = groebner_matrix[(5, 12)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 12)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 13)] = groebner_matrix[(5, 13)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 13)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 14)] = groebner_matrix[(5, 14)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 14)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 15)] = groebner_matrix[(5, 15)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 15)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 16)] = groebner_matrix[(5, 16)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 16)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 18)] = groebner_matrix[(5, 18)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 18)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 19)] = groebner_matrix[(5, 19)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 19)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 20)] = groebner_matrix[(5, 20)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 20)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 21)] = groebner_matrix[(5, 21)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 21)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 22)] = groebner_matrix[(5, 22)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 22)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 23)] = groebner_matrix[(5, 23)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 23)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 25)] = groebner_matrix[(5, 25)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 25)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 26)] = groebner_matrix[(5, 26)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 26)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 27)] = groebner_matrix[(5, 27)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 27)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 28)] = groebner_matrix[(5, 28)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 28)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 29)] = groebner_matrix[(5, 29)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 29)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 30)] = groebner_matrix[(5, 30)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 30)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 33)] = groebner_matrix[(5, 33)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 33)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 34)] = groebner_matrix[(5, 34)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 34)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 39)] = groebner_matrix[(5, 39)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 39)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 40)] = groebner_matrix[(5, 40)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 40)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 41)] = groebner_matrix[(5, 41)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 41)] / (groebner_matrix[(6, 0)]);
}

#[inline]
fn s_polynomial35_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(35, 42)] = groebner_matrix[(5, 42)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 42)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 43)] = groebner_matrix[(5, 43)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 43)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 44)] = groebner_matrix[(5, 44)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 44)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 45)] = groebner_matrix[(5, 45)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 45)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 46)] = groebner_matrix[(5, 46)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 46)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 47)] = groebner_matrix[(5, 47)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 47)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 48)] = groebner_matrix[(5, 48)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 48)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 49)] = groebner_matrix[(5, 49)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 49)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 50)] = groebner_matrix[(5, 50)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 50)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 51)] = groebner_matrix[(5, 51)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 51)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 52)] = groebner_matrix[(5, 52)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 52)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 53)] = groebner_matrix[(5, 53)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 53)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 54)] = groebner_matrix[(5, 54)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 54)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 55)] = groebner_matrix[(5, 55)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 55)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 56)] = groebner_matrix[(5, 56)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 56)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 57)] = groebner_matrix[(5, 57)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 57)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 58)] = groebner_matrix[(5, 58)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 58)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 59)] = groebner_matrix[(5, 59)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 59)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 60)] = groebner_matrix[(5, 60)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 60)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 61)] = groebner_matrix[(5, 61)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 61)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 62)] = groebner_matrix[(5, 62)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 62)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 64)] = groebner_matrix[(5, 64)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 64)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 65)] = groebner_matrix[(5, 65)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 65)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 66)] = groebner_matrix[(5, 66)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 66)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 67)] = groebner_matrix[(5, 67)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 67)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 68)] = groebner_matrix[(5, 68)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 68)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 69)] = groebner_matrix[(5, 69)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 69)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 70)] = groebner_matrix[(5, 70)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 70)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 71)] = groebner_matrix[(5, 71)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 71)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 73)] = groebner_matrix[(5, 73)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 73)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 74)] = groebner_matrix[(5, 74)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 74)] / (groebner_matrix[(6, 0)]);
}

#[inline]
fn s_polynomial35_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(35, 76)] = groebner_matrix[(5, 76)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 76)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 77)] = groebner_matrix[(5, 77)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 77)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 78)] = groebner_matrix[(5, 78)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 78)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 79)] = groebner_matrix[(5, 79)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 79)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 80)] = groebner_matrix[(5, 80)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 80)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 81)] = groebner_matrix[(5, 81)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 81)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 82)] = groebner_matrix[(5, 82)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 82)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 83)] = groebner_matrix[(5, 83)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 83)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 84)] = groebner_matrix[(5, 84)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 84)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 85)] = groebner_matrix[(5, 85)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 85)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 86)] = groebner_matrix[(5, 86)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 86)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 87)] = groebner_matrix[(5, 87)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 87)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 89)] = groebner_matrix[(5, 89)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 89)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 91)] = groebner_matrix[(5, 91)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 91)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 92)] = groebner_matrix[(5, 92)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 92)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 94)] = groebner_matrix[(5, 94)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 94)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 97)] = groebner_matrix[(5, 97)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 97)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 98)] = groebner_matrix[(5, 98)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 98)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 99)] = groebner_matrix[(5, 99)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 99)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 100)] = groebner_matrix[(5, 100)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 100)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 101)] = groebner_matrix[(5, 101)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 101)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 103)] = groebner_matrix[(5, 103)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 103)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 104)] = groebner_matrix[(5, 104)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 104)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 105)] = groebner_matrix[(5, 105)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 105)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 106)] = groebner_matrix[(5, 106)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 106)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 107)] = groebner_matrix[(5, 107)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 107)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 108)] = groebner_matrix[(5, 108)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 108)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 109)] = groebner_matrix[(5, 109)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 109)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 110)] = groebner_matrix[(5, 110)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 110)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 111)] = groebner_matrix[(5, 111)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 111)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 112)] = groebner_matrix[(5, 112)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 112)] / (groebner_matrix[(6, 0)]);
}

#[inline]
fn s_polynomial35_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(35, 113)] = groebner_matrix[(5, 113)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 113)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 115)] = groebner_matrix[(5, 115)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 115)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 116)] = groebner_matrix[(5, 116)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 116)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 118)] = groebner_matrix[(5, 118)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 118)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 119)] = groebner_matrix[(5, 119)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 119)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 120)] = groebner_matrix[(5, 120)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 120)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 121)] = groebner_matrix[(5, 121)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 121)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 122)] = groebner_matrix[(5, 122)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 122)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 123)] = groebner_matrix[(5, 123)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 123)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 125)] = groebner_matrix[(5, 125)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 125)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 126)] = groebner_matrix[(5, 126)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 126)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 127)] = groebner_matrix[(5, 127)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 127)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 128)] = groebner_matrix[(5, 128)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 128)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 129)] = groebner_matrix[(5, 129)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 129)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 130)] = groebner_matrix[(5, 130)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 130)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 133)] = groebner_matrix[(5, 133)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 133)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 134)] = groebner_matrix[(5, 134)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 134)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 136)] = groebner_matrix[(5, 136)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 136)] / (groebner_matrix[(6, 0)]);
    groebner_matrix[(35, 137)] = groebner_matrix[(5, 137)] / (groebner_matrix[(5, 0)])
        - groebner_matrix[(6, 137)] / (groebner_matrix[(6, 0)]);
}

pub fn s_polynomial36(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial36_stage_0(groebner_matrix);
    s_polynomial36_stage_1(groebner_matrix);
    s_polynomial36_stage_2(groebner_matrix);
    s_polynomial36_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial36_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(36, 1)] = groebner_matrix[(6, 1)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 1)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 2)] = groebner_matrix[(6, 2)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 2)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 3)] = groebner_matrix[(6, 3)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 3)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 4)] = groebner_matrix[(6, 4)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 4)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 5)] = groebner_matrix[(6, 5)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 5)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 6)] = groebner_matrix[(6, 6)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 6)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 8)] = groebner_matrix[(6, 8)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 8)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 10)] = groebner_matrix[(6, 10)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 10)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 11)] = groebner_matrix[(6, 11)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 11)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 12)] = groebner_matrix[(6, 12)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 12)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 13)] = groebner_matrix[(6, 13)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 13)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 14)] = groebner_matrix[(6, 14)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 14)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 15)] = groebner_matrix[(6, 15)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 15)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 16)] = groebner_matrix[(6, 16)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 16)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 18)] = groebner_matrix[(6, 18)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 18)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 19)] = groebner_matrix[(6, 19)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 19)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 20)] = groebner_matrix[(6, 20)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 20)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 21)] = groebner_matrix[(6, 21)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 21)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 22)] = groebner_matrix[(6, 22)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 22)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 23)] = groebner_matrix[(6, 23)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 23)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 25)] = groebner_matrix[(6, 25)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 25)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 26)] = groebner_matrix[(6, 26)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 26)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 27)] = groebner_matrix[(6, 27)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 27)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 28)] = groebner_matrix[(6, 28)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 28)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 29)] = groebner_matrix[(6, 29)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 29)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 30)] = groebner_matrix[(6, 30)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 30)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 33)] = groebner_matrix[(6, 33)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 33)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 34)] = groebner_matrix[(6, 34)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 34)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 39)] = groebner_matrix[(6, 39)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 39)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 40)] = groebner_matrix[(6, 40)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 40)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 41)] = groebner_matrix[(6, 41)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 41)] / (groebner_matrix[(7, 0)]);
}

#[inline]
fn s_polynomial36_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(36, 42)] = groebner_matrix[(6, 42)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 42)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 43)] = groebner_matrix[(6, 43)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 43)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 44)] = groebner_matrix[(6, 44)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 44)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 45)] = groebner_matrix[(6, 45)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 45)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 46)] = groebner_matrix[(6, 46)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 46)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 47)] = groebner_matrix[(6, 47)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 47)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 48)] = groebner_matrix[(6, 48)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 48)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 49)] = groebner_matrix[(6, 49)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 49)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 50)] = groebner_matrix[(6, 50)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 50)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 51)] = groebner_matrix[(6, 51)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 51)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 52)] = groebner_matrix[(6, 52)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 52)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 53)] = groebner_matrix[(6, 53)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 53)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 54)] = groebner_matrix[(6, 54)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 54)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 55)] = groebner_matrix[(6, 55)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 55)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 56)] = groebner_matrix[(6, 56)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 56)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 57)] = groebner_matrix[(6, 57)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 57)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 58)] = groebner_matrix[(6, 58)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 58)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 59)] = groebner_matrix[(6, 59)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 59)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 60)] = groebner_matrix[(6, 60)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 60)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 61)] = groebner_matrix[(6, 61)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 61)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 62)] = groebner_matrix[(6, 62)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 62)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 64)] = groebner_matrix[(6, 64)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 64)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 65)] = groebner_matrix[(6, 65)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 65)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 66)] = groebner_matrix[(6, 66)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 66)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 67)] = groebner_matrix[(6, 67)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 67)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 68)] = groebner_matrix[(6, 68)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 68)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 69)] = groebner_matrix[(6, 69)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 69)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 70)] = groebner_matrix[(6, 70)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 70)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 71)] = groebner_matrix[(6, 71)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 71)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 73)] = groebner_matrix[(6, 73)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 73)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 74)] = groebner_matrix[(6, 74)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 74)] / (groebner_matrix[(7, 0)]);
}

#[inline]
fn s_polynomial36_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(36, 76)] = groebner_matrix[(6, 76)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 76)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 77)] = groebner_matrix[(6, 77)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 77)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 78)] = groebner_matrix[(6, 78)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 78)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 79)] = groebner_matrix[(6, 79)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 79)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 80)] = groebner_matrix[(6, 80)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 80)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 81)] = groebner_matrix[(6, 81)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 81)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 82)] = groebner_matrix[(6, 82)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 82)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 83)] = groebner_matrix[(6, 83)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 83)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 84)] = groebner_matrix[(6, 84)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 84)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 85)] = groebner_matrix[(6, 85)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 85)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 86)] = groebner_matrix[(6, 86)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 86)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 87)] = groebner_matrix[(6, 87)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 87)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 89)] = groebner_matrix[(6, 89)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 89)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 91)] = groebner_matrix[(6, 91)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 91)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 92)] = groebner_matrix[(6, 92)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 92)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 94)] = groebner_matrix[(6, 94)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 94)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 97)] = groebner_matrix[(6, 97)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 97)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 98)] = groebner_matrix[(6, 98)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 98)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 99)] = groebner_matrix[(6, 99)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 99)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 100)] = groebner_matrix[(6, 100)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 100)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 101)] = groebner_matrix[(6, 101)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 101)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 103)] = groebner_matrix[(6, 103)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 103)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 104)] = groebner_matrix[(6, 104)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 104)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 105)] = groebner_matrix[(6, 105)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 105)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 106)] = groebner_matrix[(6, 106)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 106)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 107)] = groebner_matrix[(6, 107)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 107)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 108)] = groebner_matrix[(6, 108)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 108)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 109)] = groebner_matrix[(6, 109)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 109)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 110)] = groebner_matrix[(6, 110)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 110)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 111)] = groebner_matrix[(6, 111)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 111)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 112)] = groebner_matrix[(6, 112)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 112)] / (groebner_matrix[(7, 0)]);
}

#[inline]
fn s_polynomial36_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(36, 113)] = groebner_matrix[(6, 113)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 113)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 115)] = groebner_matrix[(6, 115)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 115)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 116)] = groebner_matrix[(6, 116)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 116)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 118)] = groebner_matrix[(6, 118)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 118)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 119)] = groebner_matrix[(6, 119)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 119)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 120)] = groebner_matrix[(6, 120)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 120)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 121)] = groebner_matrix[(6, 121)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 121)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 122)] = groebner_matrix[(6, 122)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 122)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 123)] = groebner_matrix[(6, 123)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 123)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 125)] = groebner_matrix[(6, 125)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 125)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 126)] = groebner_matrix[(6, 126)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 126)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 127)] = groebner_matrix[(6, 127)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 127)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 128)] = groebner_matrix[(6, 128)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 128)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 129)] = groebner_matrix[(6, 129)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 129)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 130)] = groebner_matrix[(6, 130)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 130)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 133)] = groebner_matrix[(6, 133)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 133)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 134)] = groebner_matrix[(6, 134)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 134)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 136)] = groebner_matrix[(6, 136)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 136)] / (groebner_matrix[(7, 0)]);
    groebner_matrix[(36, 137)] = groebner_matrix[(6, 137)] / (groebner_matrix[(6, 0)])
        - groebner_matrix[(7, 137)] / (groebner_matrix[(7, 0)]);
}

pub fn s_polynomial37(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial37_stage_0(groebner_matrix);
    s_polynomial37_stage_1(groebner_matrix);
    s_polynomial37_stage_2(groebner_matrix);
    s_polynomial37_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial37_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(37, 1)] = groebner_matrix[(7, 1)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 1)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 2)] = groebner_matrix[(7, 2)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 2)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 3)] = groebner_matrix[(7, 3)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 3)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 4)] = groebner_matrix[(7, 4)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 4)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 5)] = groebner_matrix[(7, 5)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 5)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 6)] = groebner_matrix[(7, 6)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 6)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 8)] = groebner_matrix[(7, 8)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 8)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 10)] = groebner_matrix[(7, 10)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 10)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 11)] = groebner_matrix[(7, 11)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 11)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 12)] = groebner_matrix[(7, 12)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 12)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 13)] = groebner_matrix[(7, 13)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 13)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 14)] = groebner_matrix[(7, 14)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 14)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 15)] = groebner_matrix[(7, 15)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 15)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 16)] = groebner_matrix[(7, 16)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 16)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 18)] = groebner_matrix[(7, 18)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 18)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 19)] = groebner_matrix[(7, 19)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 19)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 20)] = groebner_matrix[(7, 20)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 20)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 21)] = groebner_matrix[(7, 21)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 21)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 22)] = groebner_matrix[(7, 22)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 22)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 23)] = groebner_matrix[(7, 23)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 23)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 25)] = groebner_matrix[(7, 25)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 25)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 26)] = groebner_matrix[(7, 26)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 26)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 27)] = groebner_matrix[(7, 27)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 27)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 28)] = groebner_matrix[(7, 28)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 28)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 29)] = groebner_matrix[(7, 29)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 29)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 30)] = groebner_matrix[(7, 30)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 30)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 33)] = groebner_matrix[(7, 33)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 33)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 34)] = groebner_matrix[(7, 34)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 34)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 39)] = groebner_matrix[(7, 39)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 39)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 40)] = groebner_matrix[(7, 40)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 40)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 41)] = groebner_matrix[(7, 41)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 41)] / (groebner_matrix[(8, 0)]);
}

#[inline]
fn s_polynomial37_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(37, 42)] = groebner_matrix[(7, 42)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 42)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 43)] = groebner_matrix[(7, 43)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 43)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 44)] = groebner_matrix[(7, 44)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 44)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 45)] = groebner_matrix[(7, 45)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 45)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 46)] = groebner_matrix[(7, 46)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 46)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 47)] = groebner_matrix[(7, 47)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 47)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 48)] = groebner_matrix[(7, 48)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 48)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 49)] = groebner_matrix[(7, 49)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 49)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 50)] = groebner_matrix[(7, 50)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 50)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 51)] = groebner_matrix[(7, 51)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 51)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 52)] = groebner_matrix[(7, 52)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 52)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 53)] = groebner_matrix[(7, 53)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 53)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 54)] = groebner_matrix[(7, 54)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 54)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 55)] = groebner_matrix[(7, 55)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 55)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 56)] = groebner_matrix[(7, 56)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 56)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 57)] = groebner_matrix[(7, 57)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 57)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 58)] = groebner_matrix[(7, 58)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 58)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 59)] = groebner_matrix[(7, 59)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 59)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 60)] = groebner_matrix[(7, 60)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 60)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 61)] = groebner_matrix[(7, 61)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 61)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 62)] = groebner_matrix[(7, 62)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 62)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 64)] = groebner_matrix[(7, 64)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 64)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 65)] = groebner_matrix[(7, 65)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 65)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 66)] = groebner_matrix[(7, 66)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 66)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 67)] = groebner_matrix[(7, 67)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 67)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 68)] = groebner_matrix[(7, 68)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 68)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 69)] = groebner_matrix[(7, 69)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 69)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 70)] = groebner_matrix[(7, 70)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 70)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 71)] = groebner_matrix[(7, 71)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 71)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 73)] = groebner_matrix[(7, 73)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 73)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 74)] = groebner_matrix[(7, 74)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 74)] / (groebner_matrix[(8, 0)]);
}

#[inline]
fn s_polynomial37_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(37, 76)] = groebner_matrix[(7, 76)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 76)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 77)] = groebner_matrix[(7, 77)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 77)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 78)] = groebner_matrix[(7, 78)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 78)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 79)] = groebner_matrix[(7, 79)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 79)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 80)] = groebner_matrix[(7, 80)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 80)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 81)] = groebner_matrix[(7, 81)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 81)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 82)] = groebner_matrix[(7, 82)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 82)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 83)] = groebner_matrix[(7, 83)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 83)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 84)] = groebner_matrix[(7, 84)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 84)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 85)] = groebner_matrix[(7, 85)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 85)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 86)] = groebner_matrix[(7, 86)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 86)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 87)] = groebner_matrix[(7, 87)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 87)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 89)] = groebner_matrix[(7, 89)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 89)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 91)] = groebner_matrix[(7, 91)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 91)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 92)] = groebner_matrix[(7, 92)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 92)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 94)] = groebner_matrix[(7, 94)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 94)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 97)] = groebner_matrix[(7, 97)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 97)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 98)] = groebner_matrix[(7, 98)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 98)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 99)] = groebner_matrix[(7, 99)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 99)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 100)] = groebner_matrix[(7, 100)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 100)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 101)] = groebner_matrix[(7, 101)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 101)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 103)] = groebner_matrix[(7, 103)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 103)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 104)] = groebner_matrix[(7, 104)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 104)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 105)] = groebner_matrix[(7, 105)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 105)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 106)] = groebner_matrix[(7, 106)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 106)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 107)] = groebner_matrix[(7, 107)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 107)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 108)] = groebner_matrix[(7, 108)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 108)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 109)] = groebner_matrix[(7, 109)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 109)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 110)] = groebner_matrix[(7, 110)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 110)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 111)] = groebner_matrix[(7, 111)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 111)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 112)] = groebner_matrix[(7, 112)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 112)] / (groebner_matrix[(8, 0)]);
}

#[inline]
fn s_polynomial37_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(37, 113)] = groebner_matrix[(7, 113)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 113)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 115)] = groebner_matrix[(7, 115)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 115)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 116)] = groebner_matrix[(7, 116)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 116)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 118)] = groebner_matrix[(7, 118)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 118)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 119)] = groebner_matrix[(7, 119)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 119)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 120)] = groebner_matrix[(7, 120)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 120)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 121)] = groebner_matrix[(7, 121)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 121)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 122)] = groebner_matrix[(7, 122)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 122)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 123)] = groebner_matrix[(7, 123)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 123)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 125)] = groebner_matrix[(7, 125)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 125)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 126)] = groebner_matrix[(7, 126)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 126)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 127)] = groebner_matrix[(7, 127)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 127)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 128)] = groebner_matrix[(7, 128)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 128)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 129)] = groebner_matrix[(7, 129)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 129)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 130)] = groebner_matrix[(7, 130)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 130)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 133)] = groebner_matrix[(7, 133)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 133)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 134)] = groebner_matrix[(7, 134)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 134)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 136)] = groebner_matrix[(7, 136)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 136)] / (groebner_matrix[(8, 0)]);
    groebner_matrix[(37, 137)] = groebner_matrix[(7, 137)] / (groebner_matrix[(7, 0)])
        - groebner_matrix[(8, 137)] / (groebner_matrix[(8, 0)]);
}

pub fn s_polynomial38(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial38_stage_0(groebner_matrix);
    s_polynomial38_stage_1(groebner_matrix);
    s_polynomial38_stage_2(groebner_matrix);
    s_polynomial38_stage_3(groebner_matrix);
}
#[inline]
fn s_polynomial38_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(38, 1)] = groebner_matrix[(8, 1)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 1)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 2)] = groebner_matrix[(8, 2)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 2)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 3)] = groebner_matrix[(8, 3)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 3)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 4)] = groebner_matrix[(8, 4)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 4)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 5)] = groebner_matrix[(8, 5)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 5)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 6)] = groebner_matrix[(8, 6)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 6)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 8)] = groebner_matrix[(8, 8)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 8)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 10)] = groebner_matrix[(8, 10)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 10)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 11)] = groebner_matrix[(8, 11)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 11)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 12)] = groebner_matrix[(8, 12)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 12)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 13)] = groebner_matrix[(8, 13)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 13)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 14)] = groebner_matrix[(8, 14)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 14)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 15)] = groebner_matrix[(8, 15)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 15)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 16)] = groebner_matrix[(8, 16)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 16)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 18)] = groebner_matrix[(8, 18)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 18)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 19)] = groebner_matrix[(8, 19)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 19)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 20)] = groebner_matrix[(8, 20)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 20)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 21)] = groebner_matrix[(8, 21)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 21)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 22)] = groebner_matrix[(8, 22)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 22)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 23)] = groebner_matrix[(8, 23)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 23)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 25)] = groebner_matrix[(8, 25)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 25)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 26)] = groebner_matrix[(8, 26)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 26)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 27)] = groebner_matrix[(8, 27)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 27)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 28)] = groebner_matrix[(8, 28)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 28)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 29)] = groebner_matrix[(8, 29)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 29)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 30)] = groebner_matrix[(8, 30)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 30)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 33)] = groebner_matrix[(8, 33)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 33)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 34)] = groebner_matrix[(8, 34)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 34)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 39)] = groebner_matrix[(8, 39)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 39)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 40)] = groebner_matrix[(8, 40)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 40)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 41)] = groebner_matrix[(8, 41)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 41)] / (groebner_matrix[(9, 0)]);
}

#[inline]
fn s_polynomial38_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(38, 42)] = groebner_matrix[(8, 42)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 42)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 43)] = groebner_matrix[(8, 43)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 43)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 44)] = groebner_matrix[(8, 44)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 44)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 45)] = groebner_matrix[(8, 45)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 45)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 46)] = groebner_matrix[(8, 46)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 46)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 47)] = groebner_matrix[(8, 47)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 47)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 48)] = groebner_matrix[(8, 48)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 48)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 49)] = groebner_matrix[(8, 49)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 49)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 50)] = groebner_matrix[(8, 50)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 50)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 51)] = groebner_matrix[(8, 51)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 51)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 52)] = groebner_matrix[(8, 52)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 52)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 53)] = groebner_matrix[(8, 53)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 53)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 54)] = groebner_matrix[(8, 54)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 54)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 55)] = groebner_matrix[(8, 55)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 55)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 56)] = groebner_matrix[(8, 56)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 56)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 57)] = groebner_matrix[(8, 57)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 57)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 58)] = groebner_matrix[(8, 58)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 58)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 59)] = groebner_matrix[(8, 59)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 59)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 60)] = groebner_matrix[(8, 60)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 60)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 61)] = groebner_matrix[(8, 61)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 61)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 62)] = groebner_matrix[(8, 62)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 62)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 64)] = groebner_matrix[(8, 64)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 64)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 65)] = groebner_matrix[(8, 65)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 65)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 66)] = groebner_matrix[(8, 66)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 66)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 67)] = groebner_matrix[(8, 67)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 67)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 68)] = groebner_matrix[(8, 68)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 68)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 69)] = groebner_matrix[(8, 69)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 69)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 70)] = groebner_matrix[(8, 70)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 70)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 71)] = groebner_matrix[(8, 71)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 71)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 73)] = groebner_matrix[(8, 73)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 73)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 74)] = groebner_matrix[(8, 74)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 74)] / (groebner_matrix[(9, 0)]);
}

#[inline]
fn s_polynomial38_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(38, 76)] = groebner_matrix[(8, 76)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 76)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 77)] = groebner_matrix[(8, 77)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 77)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 78)] = groebner_matrix[(8, 78)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 78)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 79)] = groebner_matrix[(8, 79)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 79)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 80)] = groebner_matrix[(8, 80)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 80)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 81)] = groebner_matrix[(8, 81)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 81)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 82)] = groebner_matrix[(8, 82)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 82)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 83)] = groebner_matrix[(8, 83)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 83)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 84)] = groebner_matrix[(8, 84)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 84)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 85)] = groebner_matrix[(8, 85)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 85)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 86)] = groebner_matrix[(8, 86)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 86)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 87)] = groebner_matrix[(8, 87)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 87)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 89)] = groebner_matrix[(8, 89)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 89)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 91)] = groebner_matrix[(8, 91)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 91)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 92)] = groebner_matrix[(8, 92)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 92)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 94)] = groebner_matrix[(8, 94)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 94)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 97)] = groebner_matrix[(8, 97)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 97)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 98)] = groebner_matrix[(8, 98)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 98)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 99)] = groebner_matrix[(8, 99)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 99)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 100)] = groebner_matrix[(8, 100)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 100)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 101)] = groebner_matrix[(8, 101)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 101)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 103)] = groebner_matrix[(8, 103)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 103)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 104)] = groebner_matrix[(8, 104)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 104)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 105)] = groebner_matrix[(8, 105)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 105)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 106)] = groebner_matrix[(8, 106)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 106)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 107)] = groebner_matrix[(8, 107)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 107)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 108)] = groebner_matrix[(8, 108)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 108)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 109)] = groebner_matrix[(8, 109)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 109)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 110)] = groebner_matrix[(8, 110)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 110)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 111)] = groebner_matrix[(8, 111)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 111)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 112)] = groebner_matrix[(8, 112)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 112)] / (groebner_matrix[(9, 0)]);
}

#[inline]
fn s_polynomial38_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(38, 113)] = groebner_matrix[(8, 113)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 113)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 115)] = groebner_matrix[(8, 115)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 115)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 116)] = groebner_matrix[(8, 116)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 116)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 118)] = groebner_matrix[(8, 118)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 118)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 119)] = groebner_matrix[(8, 119)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 119)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 120)] = groebner_matrix[(8, 120)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 120)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 121)] = groebner_matrix[(8, 121)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 121)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 122)] = groebner_matrix[(8, 122)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 122)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 123)] = groebner_matrix[(8, 123)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 123)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 125)] = groebner_matrix[(8, 125)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 125)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 126)] = groebner_matrix[(8, 126)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 126)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 127)] = groebner_matrix[(8, 127)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 127)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 128)] = groebner_matrix[(8, 128)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 128)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 129)] = groebner_matrix[(8, 129)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 129)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 130)] = groebner_matrix[(8, 130)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 130)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 133)] = groebner_matrix[(8, 133)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 133)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 134)] = groebner_matrix[(8, 134)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 134)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 136)] = groebner_matrix[(8, 136)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 136)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(38, 137)] = groebner_matrix[(8, 137)] / (groebner_matrix[(8, 0)])
        - groebner_matrix[(9, 137)] / (groebner_matrix[(9, 0)]);
}

pub fn s_polynomial39(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial39_stage_0(groebner_matrix);
    s_polynomial39_stage_1(groebner_matrix);
}
#[inline]
fn s_polynomial39_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(39, 1)] = groebner_matrix[(9, 1)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 2)] = groebner_matrix[(9, 2)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 3)] = groebner_matrix[(9, 3)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 4)] = groebner_matrix[(9, 4)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 5)] = groebner_matrix[(9, 5)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 6)] = groebner_matrix[(9, 6)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 7)] = groebner_matrix[(9, 7)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 8)] = groebner_matrix[(9, 8)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 9)] = -groebner_matrix[(14, 153)] / (groebner_matrix[(14, 148)]);
    groebner_matrix[(39, 10)] = groebner_matrix[(9, 10)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 11)] = groebner_matrix[(9, 11)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 12)] = groebner_matrix[(9, 12)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 13)] = groebner_matrix[(9, 13)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 14)] = groebner_matrix[(9, 14)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 15)] = groebner_matrix[(9, 15)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 16)] = groebner_matrix[(9, 16)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 18)] = groebner_matrix[(9, 18)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 19)] = groebner_matrix[(9, 19)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 20)] = groebner_matrix[(9, 20)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 21)] = groebner_matrix[(9, 21)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 22)] = groebner_matrix[(9, 22)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 23)] = groebner_matrix[(9, 23)] / (groebner_matrix[(9, 0)])
        - groebner_matrix[(14, 159)] / (groebner_matrix[(14, 148)]);
    groebner_matrix[(39, 25)] = groebner_matrix[(9, 25)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 26)] = groebner_matrix[(9, 26)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 27)] = groebner_matrix[(9, 27)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 28)] = groebner_matrix[(9, 28)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 29)] = groebner_matrix[(9, 29)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 30)] = groebner_matrix[(9, 30)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 33)] = groebner_matrix[(9, 33)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 34)] = groebner_matrix[(9, 34)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 39)] = groebner_matrix[(9, 39)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 40)] = groebner_matrix[(9, 40)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 41)] = groebner_matrix[(9, 41)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 42)] = groebner_matrix[(9, 42)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 43)] = groebner_matrix[(9, 43)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 44)] = groebner_matrix[(9, 44)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 45)] = groebner_matrix[(9, 45)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 46)] = groebner_matrix[(9, 46)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 47)] = groebner_matrix[(9, 47)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 48)] = groebner_matrix[(9, 48)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 49)] = groebner_matrix[(9, 49)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 50)] = groebner_matrix[(9, 50)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 51)] = groebner_matrix[(9, 51)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 52)] = groebner_matrix[(9, 52)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 53)] = groebner_matrix[(9, 53)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 54)] = groebner_matrix[(9, 54)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 55)] = groebner_matrix[(9, 55)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 56)] = groebner_matrix[(9, 56)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 57)] = groebner_matrix[(9, 57)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 58)] = groebner_matrix[(9, 58)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 59)] = groebner_matrix[(9, 59)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 60)] = groebner_matrix[(9, 60)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 61)] = groebner_matrix[(9, 61)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 62)] = groebner_matrix[(9, 62)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 64)] = groebner_matrix[(9, 64)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 65)] = groebner_matrix[(9, 65)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 66)] = groebner_matrix[(9, 66)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 67)] = groebner_matrix[(9, 67)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 68)] = groebner_matrix[(9, 68)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 69)] = groebner_matrix[(9, 69)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 70)] = groebner_matrix[(9, 70)] / (groebner_matrix[(9, 0)]);
}

#[inline]
fn s_polynomial39_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(39, 71)] = groebner_matrix[(9, 71)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 73)] = groebner_matrix[(9, 73)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 74)] = groebner_matrix[(9, 74)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 76)] = groebner_matrix[(9, 76)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 77)] = groebner_matrix[(9, 77)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 78)] = groebner_matrix[(9, 78)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 79)] = groebner_matrix[(9, 79)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 80)] = groebner_matrix[(9, 80)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 81)] = groebner_matrix[(9, 81)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 82)] = groebner_matrix[(9, 82)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 83)] = groebner_matrix[(9, 83)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 84)] = groebner_matrix[(9, 84)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 85)] = groebner_matrix[(9, 85)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 86)] = groebner_matrix[(9, 86)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 87)] = groebner_matrix[(9, 87)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 89)] = groebner_matrix[(9, 89)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 91)] = groebner_matrix[(9, 91)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 92)] = groebner_matrix[(9, 92)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 94)] = groebner_matrix[(9, 94)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 97)] = groebner_matrix[(9, 97)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 98)] = groebner_matrix[(9, 98)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 99)] = groebner_matrix[(9, 99)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 100)] = groebner_matrix[(9, 100)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 101)] = groebner_matrix[(9, 101)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 103)] = groebner_matrix[(9, 103)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 104)] = groebner_matrix[(9, 104)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 105)] = groebner_matrix[(9, 105)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 106)] = groebner_matrix[(9, 106)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 107)] = groebner_matrix[(9, 107)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 108)] = groebner_matrix[(9, 108)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 109)] = groebner_matrix[(9, 109)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 110)] = groebner_matrix[(9, 110)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 111)] = groebner_matrix[(9, 111)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 112)] = groebner_matrix[(9, 112)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 113)] = groebner_matrix[(9, 113)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 115)] = groebner_matrix[(9, 115)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 116)] = groebner_matrix[(9, 116)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 118)] = groebner_matrix[(9, 118)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 119)] = groebner_matrix[(9, 119)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 120)] = groebner_matrix[(9, 120)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 121)] = groebner_matrix[(9, 121)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 122)] = groebner_matrix[(9, 122)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 123)] = groebner_matrix[(9, 123)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 125)] = groebner_matrix[(9, 125)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 126)] = groebner_matrix[(9, 126)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 127)] = groebner_matrix[(9, 127)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 128)] = groebner_matrix[(9, 128)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 129)] = groebner_matrix[(9, 129)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 130)] = groebner_matrix[(9, 130)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 133)] = groebner_matrix[(9, 133)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 134)] = groebner_matrix[(9, 134)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 136)] = groebner_matrix[(9, 136)] / (groebner_matrix[(9, 0)]);
    groebner_matrix[(39, 137)] = groebner_matrix[(9, 137)] / (groebner_matrix[(9, 0)]);
}

pub fn s_polynomial40(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(40, 89)] = -groebner_matrix[(39, 170)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 90)] = -groebner_matrix[(39, 171)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 92)] = -groebner_matrix[(39, 173)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 93)] = -groebner_matrix[(39, 174)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 95)] = -groebner_matrix[(39, 176)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 96)] = -groebner_matrix[(39, 177)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 116)] = groebner_matrix[(18, 182)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(40, 125)] = -groebner_matrix[(39, 178)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 126)] = -groebner_matrix[(39, 179)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 127)] = -groebner_matrix[(39, 180)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 128)] = -groebner_matrix[(39, 181)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 129)] = -groebner_matrix[(39, 182)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 130)] = -groebner_matrix[(39, 183)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 131)] = -groebner_matrix[(39, 184)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 132)] = -groebner_matrix[(39, 185)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 140)] = -groebner_matrix[(39, 186)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 157)] = groebner_matrix[(18, 187)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(40, 170)] = -groebner_matrix[(39, 187)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 171)] = -groebner_matrix[(39, 188)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 172)] = -groebner_matrix[(39, 189)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 173)] = -groebner_matrix[(39, 190)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 174)] = -groebner_matrix[(39, 191)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 175)] = -groebner_matrix[(39, 192)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 176)] = -groebner_matrix[(39, 193)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 177)] = -groebner_matrix[(39, 194)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 185)] = -groebner_matrix[(39, 195)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(40, 194)] = -groebner_matrix[(39, 196)] / (groebner_matrix[(39, 162)]);
}
pub fn s_polynomial41(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(41, 81)] = -groebner_matrix[(38, 162)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 89)] = -groebner_matrix[(38, 170)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 90)] = -groebner_matrix[(38, 171)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 92)] = -groebner_matrix[(38, 173)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 93)] = -groebner_matrix[(38, 174)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 95)] = -groebner_matrix[(38, 176)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 96)] = -groebner_matrix[(38, 177)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 111)] = groebner_matrix[(18, 182)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(41, 125)] = -groebner_matrix[(38, 178)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 126)] = -groebner_matrix[(38, 179)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 127)] = -groebner_matrix[(38, 180)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 128)] = -groebner_matrix[(38, 181)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 129)] = -groebner_matrix[(38, 182)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 130)] = -groebner_matrix[(38, 183)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 131)] = -groebner_matrix[(38, 184)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 132)] = -groebner_matrix[(38, 185)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 140)] = -groebner_matrix[(38, 186)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 152)] = groebner_matrix[(18, 187)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(41, 170)] = -groebner_matrix[(38, 187)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 171)] = -groebner_matrix[(38, 188)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 172)] = -groebner_matrix[(38, 189)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 173)] = -groebner_matrix[(38, 190)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 174)] = -groebner_matrix[(38, 191)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 175)] = -groebner_matrix[(38, 192)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 176)] = -groebner_matrix[(38, 193)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 177)] = -groebner_matrix[(38, 194)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 185)] = -groebner_matrix[(38, 195)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(41, 194)] = -groebner_matrix[(38, 196)] / (groebner_matrix[(38, 161)]);
}
pub fn s_polynomial42(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(42, 80)] = -groebner_matrix[(37, 161)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 81)] = -groebner_matrix[(37, 162)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 89)] = -groebner_matrix[(37, 170)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 90)] = -groebner_matrix[(37, 171)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 92)] = -groebner_matrix[(37, 173)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 93)] = -groebner_matrix[(37, 174)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 95)] = -groebner_matrix[(37, 176)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 96)] = -groebner_matrix[(37, 177)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 110)] = groebner_matrix[(18, 182)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(42, 125)] = -groebner_matrix[(37, 178)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 126)] = -groebner_matrix[(37, 179)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 127)] = -groebner_matrix[(37, 180)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 128)] = -groebner_matrix[(37, 181)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 129)] = -groebner_matrix[(37, 182)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 130)] = -groebner_matrix[(37, 183)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 131)] = -groebner_matrix[(37, 184)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 132)] = -groebner_matrix[(37, 185)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 140)] = -groebner_matrix[(37, 186)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 148)] = groebner_matrix[(18, 187)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(42, 170)] = -groebner_matrix[(37, 187)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 171)] = -groebner_matrix[(37, 188)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 172)] = -groebner_matrix[(37, 189)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 173)] = -groebner_matrix[(37, 190)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 174)] = -groebner_matrix[(37, 191)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 175)] = -groebner_matrix[(37, 192)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 176)] = -groebner_matrix[(37, 193)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 177)] = -groebner_matrix[(37, 194)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 185)] = -groebner_matrix[(37, 195)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(42, 194)] = -groebner_matrix[(37, 196)] / (groebner_matrix[(37, 160)]);
}
pub fn s_polynomial43(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(43, 79)] = -groebner_matrix[(36, 160)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 80)] = -groebner_matrix[(36, 161)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 81)] = -groebner_matrix[(36, 162)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 89)] = -groebner_matrix[(36, 170)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 90)] = -groebner_matrix[(36, 171)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 92)] = -groebner_matrix[(36, 173)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 93)] = -groebner_matrix[(36, 174)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 95)] = -groebner_matrix[(36, 176)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 96)] = -groebner_matrix[(36, 177)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 113)] = groebner_matrix[(17, 179)] / (groebner_matrix[(17, 172)]);
    groebner_matrix[(43, 125)] = -groebner_matrix[(36, 178)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 126)] = -groebner_matrix[(36, 179)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 127)] = -groebner_matrix[(36, 180)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 128)] = -groebner_matrix[(36, 181)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 129)] = -groebner_matrix[(36, 182)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 130)] = -groebner_matrix[(36, 183)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 131)] = -groebner_matrix[(36, 184)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 132)] = -groebner_matrix[(36, 185)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 140)] = -groebner_matrix[(36, 186)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 160)] = groebner_matrix[(17, 190)] / (groebner_matrix[(17, 172)]);
    groebner_matrix[(43, 170)] = -groebner_matrix[(36, 187)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 171)] = -groebner_matrix[(36, 188)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 172)] = -groebner_matrix[(36, 189)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 173)] = -groebner_matrix[(36, 190)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 174)] = -groebner_matrix[(36, 191)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 175)] = -groebner_matrix[(36, 192)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 176)] = -groebner_matrix[(36, 193)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 177)] = -groebner_matrix[(36, 194)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 185)] = -groebner_matrix[(36, 195)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(43, 194)] = -groebner_matrix[(36, 196)] / (groebner_matrix[(36, 159)]);
}
pub fn s_polynomial44(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(44, 78)] = -groebner_matrix[(35, 159)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 79)] = -groebner_matrix[(35, 160)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 80)] = -groebner_matrix[(35, 161)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 81)] = -groebner_matrix[(35, 162)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 89)] = -groebner_matrix[(35, 170)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 90)] = -groebner_matrix[(35, 171)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 92)] = -groebner_matrix[(35, 173)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 93)] = -groebner_matrix[(35, 174)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 95)] = -groebner_matrix[(35, 176)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 96)] = -groebner_matrix[(35, 177)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 108)] = groebner_matrix[(18, 182)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(44, 125)] = -groebner_matrix[(35, 178)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 126)] = -groebner_matrix[(35, 179)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 127)] = -groebner_matrix[(35, 180)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 128)] = -groebner_matrix[(35, 181)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 129)] = -groebner_matrix[(35, 182)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 130)] = -groebner_matrix[(35, 183)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 131)] = -groebner_matrix[(35, 184)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 132)] = -groebner_matrix[(35, 185)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 140)] = -groebner_matrix[(35, 186)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 143)] = groebner_matrix[(18, 187)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(44, 170)] = -groebner_matrix[(35, 187)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 171)] = -groebner_matrix[(35, 188)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 172)] = -groebner_matrix[(35, 189)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 173)] = -groebner_matrix[(35, 190)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 174)] = -groebner_matrix[(35, 191)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 175)] = -groebner_matrix[(35, 192)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 176)] = -groebner_matrix[(35, 193)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 177)] = -groebner_matrix[(35, 194)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 185)] = -groebner_matrix[(35, 195)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(44, 194)] = -groebner_matrix[(35, 196)] / (groebner_matrix[(35, 158)]);
}
pub fn s_polynomial45(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(45, 77)] = -groebner_matrix[(34, 158)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 78)] = -groebner_matrix[(34, 159)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 79)] = -groebner_matrix[(34, 160)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 80)] = -groebner_matrix[(34, 161)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 81)] = -groebner_matrix[(34, 162)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 89)] = -groebner_matrix[(34, 170)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 90)] = -groebner_matrix[(34, 171)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 92)] = -groebner_matrix[(34, 173)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 93)] = -groebner_matrix[(34, 174)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 95)] = -groebner_matrix[(34, 176)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 96)] = -groebner_matrix[(34, 177)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 107)] = groebner_matrix[(18, 182)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(45, 125)] = -groebner_matrix[(34, 178)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 126)] = -groebner_matrix[(34, 179)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 127)] = -groebner_matrix[(34, 180)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 128)] = -groebner_matrix[(34, 181)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 129)] = -groebner_matrix[(34, 182)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 130)] = -groebner_matrix[(34, 183)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 131)] = -groebner_matrix[(34, 184)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 132)] = -groebner_matrix[(34, 185)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 140)] = -groebner_matrix[(34, 186)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 142)] = groebner_matrix[(18, 187)] / (groebner_matrix[(18, 175)]);
    groebner_matrix[(45, 170)] = -groebner_matrix[(34, 187)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 171)] = -groebner_matrix[(34, 188)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 172)] = -groebner_matrix[(34, 189)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 173)] = -groebner_matrix[(34, 190)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 174)] = -groebner_matrix[(34, 191)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 175)] = -groebner_matrix[(34, 192)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 176)] = -groebner_matrix[(34, 193)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 177)] = -groebner_matrix[(34, 194)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 185)] = -groebner_matrix[(34, 195)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(45, 194)] = -groebner_matrix[(34, 196)] / (groebner_matrix[(34, 157)]);
}
pub fn s_polynomial46(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(46, 82)] = -groebner_matrix[(39, 170)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 83)] = -groebner_matrix[(39, 171)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 85)] = -groebner_matrix[(39, 173)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 86)] = -groebner_matrix[(39, 174)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 88)] = -groebner_matrix[(39, 176)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 95)] = -groebner_matrix[(39, 177)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 115)] = groebner_matrix[(27, 181)] / (groebner_matrix[(27, 168)]);
    groebner_matrix[(46, 118)] = -groebner_matrix[(39, 178)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 119)] = -groebner_matrix[(39, 179)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 120)] = -groebner_matrix[(39, 180)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 121)] = -groebner_matrix[(39, 181)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 122)] = -groebner_matrix[(39, 182)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 123)] = -groebner_matrix[(39, 183)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 124)] = -groebner_matrix[(39, 184)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 131)] = -groebner_matrix[(39, 185)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 139)] = -groebner_matrix[(39, 186)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 158)] = groebner_matrix[(27, 188)] / (groebner_matrix[(27, 168)]);
    groebner_matrix[(46, 163)] = -groebner_matrix[(39, 187)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 164)] = -groebner_matrix[(39, 188)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 165)] = -groebner_matrix[(39, 189)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 166)] = -groebner_matrix[(39, 190)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 167)] = -groebner_matrix[(39, 191)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 168)] = -groebner_matrix[(39, 192)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 169)] = -groebner_matrix[(39, 193)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 176)] = -groebner_matrix[(39, 194)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 184)] = -groebner_matrix[(39, 195)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(46, 193)] = -groebner_matrix[(39, 196)] / (groebner_matrix[(39, 162)]);
}
pub fn s_polynomial47(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(47, 56)] = -groebner_matrix[(38, 162)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 79)] = groebner_matrix[(21, 173)] / (groebner_matrix[(21, 167)]);
    groebner_matrix[(47, 82)] = -groebner_matrix[(38, 170)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 83)] = -groebner_matrix[(38, 171)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 85)] = -groebner_matrix[(38, 173)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 86)] = -groebner_matrix[(38, 174)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 88)] = -groebner_matrix[(38, 176)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 95)] = -groebner_matrix[(38, 177)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 118)] = -groebner_matrix[(38, 178)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 119)] = -groebner_matrix[(38, 179)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 120)] = -groebner_matrix[(38, 180)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 121)] = -groebner_matrix[(38, 181)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 122)] = -groebner_matrix[(38, 182)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 123)] = -groebner_matrix[(38, 183)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 124)] = -groebner_matrix[(38, 184)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 131)] = -groebner_matrix[(38, 185)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 139)] = -groebner_matrix[(38, 186)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 159)] = groebner_matrix[(21, 189)] / (groebner_matrix[(21, 167)]);
    groebner_matrix[(47, 163)] = -groebner_matrix[(38, 187)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 164)] = -groebner_matrix[(38, 188)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 165)] = -groebner_matrix[(38, 189)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 166)] = -groebner_matrix[(38, 190)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 167)] = -groebner_matrix[(38, 191)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 168)] = -groebner_matrix[(38, 192)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 169)] = -groebner_matrix[(38, 193)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 176)] = -groebner_matrix[(38, 194)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 184)] = -groebner_matrix[(38, 195)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(47, 193)] = -groebner_matrix[(38, 196)] / (groebner_matrix[(38, 161)]);
}
pub fn s_polynomial48(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(48, 55)] = -groebner_matrix[(37, 161)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 56)] = -groebner_matrix[(37, 162)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 80)] = groebner_matrix[(11, 174)] / (groebner_matrix[(11, 166)]);
    groebner_matrix[(48, 82)] = -groebner_matrix[(37, 170)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 83)] = -groebner_matrix[(37, 171)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 85)] = -groebner_matrix[(37, 173)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 86)] = -groebner_matrix[(37, 174)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 88)] = -groebner_matrix[(37, 176)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 95)] = -groebner_matrix[(37, 177)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 117)] = groebner_matrix[(11, 183)] / (groebner_matrix[(11, 166)]);
    groebner_matrix[(48, 118)] = -groebner_matrix[(37, 178)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 119)] = -groebner_matrix[(37, 179)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 120)] = -groebner_matrix[(37, 180)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 121)] = -groebner_matrix[(37, 181)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 122)] = -groebner_matrix[(37, 182)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 123)] = -groebner_matrix[(37, 183)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 124)] = -groebner_matrix[(37, 184)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 131)] = -groebner_matrix[(37, 185)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 139)] = -groebner_matrix[(37, 186)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 163)] = -groebner_matrix[(37, 187)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 164)] = -groebner_matrix[(37, 188)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 165)] = -groebner_matrix[(37, 189)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 166)] = -groebner_matrix[(37, 190)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 167)] = -groebner_matrix[(37, 191)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 168)] = -groebner_matrix[(37, 192)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 169)] = -groebner_matrix[(37, 193)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 176)] = -groebner_matrix[(37, 194)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 184)] = -groebner_matrix[(37, 195)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(48, 193)] = -groebner_matrix[(37, 196)] / (groebner_matrix[(37, 160)]);
}
pub fn s_polynomial49(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(49, 54)] = -groebner_matrix[(36, 160)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 55)] = -groebner_matrix[(36, 161)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 56)] = -groebner_matrix[(36, 162)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 82)] = -groebner_matrix[(36, 170)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 83)] = -groebner_matrix[(36, 171)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 85)] = -groebner_matrix[(36, 173)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 86)] = -groebner_matrix[(36, 174)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 88)] = -groebner_matrix[(36, 176)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 95)] = -groebner_matrix[(36, 177)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 105)] = groebner_matrix[(27, 181)] / (groebner_matrix[(27, 168)]);
    groebner_matrix[(49, 118)] = -groebner_matrix[(36, 178)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 119)] = -groebner_matrix[(36, 179)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 120)] = -groebner_matrix[(36, 180)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 121)] = -groebner_matrix[(36, 181)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 122)] = -groebner_matrix[(36, 182)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 123)] = -groebner_matrix[(36, 183)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 124)] = -groebner_matrix[(36, 184)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 131)] = -groebner_matrix[(36, 185)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 139)] = -groebner_matrix[(36, 186)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 146)] = groebner_matrix[(27, 188)] / (groebner_matrix[(27, 168)]);
    groebner_matrix[(49, 163)] = -groebner_matrix[(36, 187)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 164)] = -groebner_matrix[(36, 188)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 165)] = -groebner_matrix[(36, 189)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 166)] = -groebner_matrix[(36, 190)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 167)] = -groebner_matrix[(36, 191)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 168)] = -groebner_matrix[(36, 192)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 169)] = -groebner_matrix[(36, 193)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 176)] = -groebner_matrix[(36, 194)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 184)] = -groebner_matrix[(36, 195)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(49, 193)] = -groebner_matrix[(36, 196)] / (groebner_matrix[(36, 159)]);
}
pub fn s_polynomial50(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(50, 53)] = -groebner_matrix[(35, 159)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 54)] = -groebner_matrix[(35, 160)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 55)] = -groebner_matrix[(35, 161)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 56)] = -groebner_matrix[(35, 162)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 76)] = groebner_matrix[(20, 170)] / (groebner_matrix[(20, 164)]);
    groebner_matrix[(50, 82)] = -groebner_matrix[(35, 170)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 83)] = -groebner_matrix[(35, 171)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 85)] = -groebner_matrix[(35, 173)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 86)] = -groebner_matrix[(35, 174)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 88)] = -groebner_matrix[(35, 176)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 95)] = -groebner_matrix[(35, 177)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 118)] = -groebner_matrix[(35, 178)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 119)] = -groebner_matrix[(35, 179)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 120)] = -groebner_matrix[(35, 180)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 121)] = -groebner_matrix[(35, 181)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 122)] = -groebner_matrix[(35, 182)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 123)] = -groebner_matrix[(35, 183)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 124)] = -groebner_matrix[(35, 184)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 131)] = -groebner_matrix[(35, 185)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 139)] = -groebner_matrix[(35, 186)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 162)] = groebner_matrix[(20, 192)] / (groebner_matrix[(20, 164)]);
    groebner_matrix[(50, 163)] = -groebner_matrix[(35, 187)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 164)] = -groebner_matrix[(35, 188)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 165)] = -groebner_matrix[(35, 189)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 166)] = -groebner_matrix[(35, 190)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 167)] = -groebner_matrix[(35, 191)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 168)] = -groebner_matrix[(35, 192)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 169)] = -groebner_matrix[(35, 193)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 176)] = -groebner_matrix[(35, 194)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 184)] = -groebner_matrix[(35, 195)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(50, 193)] = -groebner_matrix[(35, 196)] / (groebner_matrix[(35, 158)]);
}
pub fn s_polynomial51(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(51, 52)] = -groebner_matrix[(34, 158)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 53)] = -groebner_matrix[(34, 159)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 54)] = -groebner_matrix[(34, 160)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 55)] = -groebner_matrix[(34, 161)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 56)] = -groebner_matrix[(34, 162)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 77)] = groebner_matrix[(12, 171)] / (groebner_matrix[(12, 163)]);
    groebner_matrix[(51, 82)] = -groebner_matrix[(34, 170)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 83)] = -groebner_matrix[(34, 171)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 85)] = -groebner_matrix[(34, 173)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 86)] = -groebner_matrix[(34, 174)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 88)] = -groebner_matrix[(34, 176)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 95)] = -groebner_matrix[(34, 177)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 114)] = groebner_matrix[(12, 180)] / (groebner_matrix[(12, 163)]);
    groebner_matrix[(51, 118)] = -groebner_matrix[(34, 178)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 119)] = -groebner_matrix[(34, 179)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 120)] = -groebner_matrix[(34, 180)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 121)] = -groebner_matrix[(34, 181)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 122)] = -groebner_matrix[(34, 182)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 123)] = -groebner_matrix[(34, 183)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 124)] = -groebner_matrix[(34, 184)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 131)] = -groebner_matrix[(34, 185)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 139)] = -groebner_matrix[(34, 186)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 163)] = -groebner_matrix[(34, 187)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 164)] = -groebner_matrix[(34, 188)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 165)] = -groebner_matrix[(34, 189)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 166)] = -groebner_matrix[(34, 190)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 167)] = -groebner_matrix[(34, 191)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 168)] = -groebner_matrix[(34, 192)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 169)] = -groebner_matrix[(34, 193)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 176)] = -groebner_matrix[(34, 194)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 184)] = -groebner_matrix[(34, 195)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(51, 193)] = -groebner_matrix[(34, 196)] / (groebner_matrix[(34, 157)]);
}
pub fn s_polynomial52(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(52, 51)] = -groebner_matrix[(33, 157)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 52)] = -groebner_matrix[(33, 158)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 53)] = -groebner_matrix[(33, 159)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 54)] = -groebner_matrix[(33, 160)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 55)] = -groebner_matrix[(33, 161)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 56)] = -groebner_matrix[(33, 162)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 74)] = groebner_matrix[(21, 173)] / (groebner_matrix[(21, 167)]);
    groebner_matrix[(52, 82)] = -groebner_matrix[(33, 170)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 83)] = -groebner_matrix[(33, 171)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 85)] = -groebner_matrix[(33, 173)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 86)] = -groebner_matrix[(33, 174)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 88)] = -groebner_matrix[(33, 176)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 95)] = -groebner_matrix[(33, 177)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 118)] = -groebner_matrix[(33, 178)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 119)] = -groebner_matrix[(33, 179)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 120)] = -groebner_matrix[(33, 180)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 121)] = -groebner_matrix[(33, 181)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 122)] = -groebner_matrix[(33, 182)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 123)] = -groebner_matrix[(33, 183)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 124)] = -groebner_matrix[(33, 184)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 131)] = -groebner_matrix[(33, 185)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 139)] = -groebner_matrix[(33, 186)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 154)] = groebner_matrix[(21, 189)] / (groebner_matrix[(21, 167)]);
    groebner_matrix[(52, 163)] = -groebner_matrix[(33, 187)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 164)] = -groebner_matrix[(33, 188)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 165)] = -groebner_matrix[(33, 189)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 166)] = -groebner_matrix[(33, 190)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 167)] = -groebner_matrix[(33, 191)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 168)] = -groebner_matrix[(33, 192)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 169)] = -groebner_matrix[(33, 193)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 176)] = -groebner_matrix[(33, 194)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 184)] = -groebner_matrix[(33, 195)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(52, 193)] = -groebner_matrix[(33, 196)] / (groebner_matrix[(33, 156)]);
}
pub fn s_polynomial53(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(53, 50)] = -groebner_matrix[(32, 156)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 51)] = -groebner_matrix[(32, 157)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 52)] = -groebner_matrix[(32, 158)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 53)] = -groebner_matrix[(32, 159)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 54)] = -groebner_matrix[(32, 160)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 55)] = -groebner_matrix[(32, 161)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 56)] = -groebner_matrix[(32, 162)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 75)] = groebner_matrix[(11, 174)] / (groebner_matrix[(11, 166)]);
    groebner_matrix[(53, 82)] = -groebner_matrix[(32, 170)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 83)] = -groebner_matrix[(32, 171)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 85)] = -groebner_matrix[(32, 173)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 86)] = -groebner_matrix[(32, 174)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 88)] = -groebner_matrix[(32, 176)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 95)] = -groebner_matrix[(32, 177)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 116)] = groebner_matrix[(11, 183)] / (groebner_matrix[(11, 166)]);
    groebner_matrix[(53, 118)] = -groebner_matrix[(32, 178)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 119)] = -groebner_matrix[(32, 179)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 120)] = -groebner_matrix[(32, 180)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 121)] = -groebner_matrix[(32, 181)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 122)] = -groebner_matrix[(32, 182)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 123)] = -groebner_matrix[(32, 183)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 124)] = -groebner_matrix[(32, 184)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 131)] = -groebner_matrix[(32, 185)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 139)] = -groebner_matrix[(32, 186)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 163)] = -groebner_matrix[(32, 187)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 164)] = -groebner_matrix[(32, 188)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 165)] = -groebner_matrix[(32, 189)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 166)] = -groebner_matrix[(32, 190)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 167)] = -groebner_matrix[(32, 191)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 168)] = -groebner_matrix[(32, 192)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 169)] = -groebner_matrix[(32, 193)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 176)] = -groebner_matrix[(32, 194)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 184)] = -groebner_matrix[(32, 195)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(53, 193)] = -groebner_matrix[(32, 196)] / (groebner_matrix[(32, 155)]);
}
pub fn s_polynomial54(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(54, 49)] = -groebner_matrix[(31, 155)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 50)] = -groebner_matrix[(31, 156)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 51)] = -groebner_matrix[(31, 157)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 52)] = -groebner_matrix[(31, 158)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 53)] = -groebner_matrix[(31, 159)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 54)] = -groebner_matrix[(31, 160)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 55)] = -groebner_matrix[(31, 161)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 56)] = -groebner_matrix[(31, 162)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 71)] = groebner_matrix[(20, 170)] / (groebner_matrix[(20, 164)]);
    groebner_matrix[(54, 82)] = -groebner_matrix[(31, 170)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 83)] = -groebner_matrix[(31, 171)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 85)] = -groebner_matrix[(31, 173)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 86)] = -groebner_matrix[(31, 174)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 88)] = -groebner_matrix[(31, 176)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 95)] = -groebner_matrix[(31, 177)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 118)] = -groebner_matrix[(31, 178)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 119)] = -groebner_matrix[(31, 179)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 120)] = -groebner_matrix[(31, 180)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 121)] = -groebner_matrix[(31, 181)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 122)] = -groebner_matrix[(31, 182)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 123)] = -groebner_matrix[(31, 183)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 124)] = -groebner_matrix[(31, 184)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 131)] = -groebner_matrix[(31, 185)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 139)] = -groebner_matrix[(31, 186)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 161)] = groebner_matrix[(20, 192)] / (groebner_matrix[(20, 164)]);
    groebner_matrix[(54, 163)] = -groebner_matrix[(31, 187)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 164)] = -groebner_matrix[(31, 188)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 165)] = -groebner_matrix[(31, 189)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 166)] = -groebner_matrix[(31, 190)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 167)] = -groebner_matrix[(31, 191)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 168)] = -groebner_matrix[(31, 192)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 169)] = -groebner_matrix[(31, 193)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 176)] = -groebner_matrix[(31, 194)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 184)] = -groebner_matrix[(31, 195)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(54, 193)] = -groebner_matrix[(31, 196)] / (groebner_matrix[(31, 153)]);
}
pub fn s_polynomial55(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(55, 38)] = groebner_matrix[(38, 162)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 71)] = -groebner_matrix[(39, 170)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 72)] = -groebner_matrix[(39, 171)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 74)] = -groebner_matrix[(39, 173)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 75)] = -groebner_matrix[(39, 174)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 76)] = groebner_matrix[(38, 170)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 77)] = groebner_matrix[(38, 171)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 79)] = groebner_matrix[(38, 173)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 80)] = groebner_matrix[(38, 174)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 86)] = -groebner_matrix[(39, 176)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 87)] = groebner_matrix[(38, 176)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 93)] = -groebner_matrix[(39, 177)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 94)] = groebner_matrix[(38, 177)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 107)] = -groebner_matrix[(39, 178)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 108)] = -groebner_matrix[(39, 179)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 109)] = -groebner_matrix[(39, 180)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 110)] = -groebner_matrix[(39, 181)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 111)] = -groebner_matrix[(39, 182)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 112)] = groebner_matrix[(38, 178)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 113)] = groebner_matrix[(38, 179)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 114)] = groebner_matrix[(38, 180)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 115)] = groebner_matrix[(38, 181)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 116)] = groebner_matrix[(38, 182)] / (groebner_matrix[(38, 161)])
        - groebner_matrix[(39, 183)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 117)] = groebner_matrix[(38, 183)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 122)] = -groebner_matrix[(39, 184)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 123)] = groebner_matrix[(38, 184)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 129)] = -groebner_matrix[(39, 185)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 130)] = groebner_matrix[(38, 185)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 137)] = -groebner_matrix[(39, 186)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 138)] = groebner_matrix[(38, 186)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 152)] = -groebner_matrix[(39, 187)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 153)] = -groebner_matrix[(39, 188)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 154)] = -groebner_matrix[(39, 189)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 155)] = -groebner_matrix[(39, 190)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 156)] = -groebner_matrix[(39, 191)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 157)] = groebner_matrix[(38, 187)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 158)] = groebner_matrix[(38, 188)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 159)] = groebner_matrix[(38, 189)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 160)] = groebner_matrix[(38, 190)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 161)] = groebner_matrix[(38, 191)] / (groebner_matrix[(38, 161)])
        - groebner_matrix[(39, 192)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 162)] = groebner_matrix[(38, 192)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 167)] = -groebner_matrix[(39, 193)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 168)] = groebner_matrix[(38, 193)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 174)] = -groebner_matrix[(39, 194)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 175)] = groebner_matrix[(38, 194)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 182)] = -groebner_matrix[(39, 195)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 183)] = groebner_matrix[(38, 195)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(55, 191)] = -groebner_matrix[(39, 196)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(55, 192)] = groebner_matrix[(38, 196)] / (groebner_matrix[(38, 161)]);
}
pub fn s_polynomial56(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(56, 37)] = groebner_matrix[(37, 161)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 38)] = groebner_matrix[(37, 162)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 67)] = -groebner_matrix[(39, 170)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 68)] = -groebner_matrix[(39, 171)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 70)] = -groebner_matrix[(39, 173)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 74)] = -groebner_matrix[(39, 174)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 76)] = groebner_matrix[(37, 170)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 77)] = groebner_matrix[(37, 171)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 79)] = groebner_matrix[(37, 173)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 80)] = groebner_matrix[(37, 174)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 85)] = -groebner_matrix[(39, 176)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 87)] = groebner_matrix[(37, 176)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 92)] = -groebner_matrix[(39, 177)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 94)] = groebner_matrix[(37, 177)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 103)] = -groebner_matrix[(39, 178)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 104)] = -groebner_matrix[(39, 179)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 105)] = -groebner_matrix[(39, 180)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 106)] = -groebner_matrix[(39, 181)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 110)] = -groebner_matrix[(39, 182)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 112)] = groebner_matrix[(37, 178)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 113)] = groebner_matrix[(37, 179)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 114)] = groebner_matrix[(37, 180)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 115)] = groebner_matrix[(37, 181)] / (groebner_matrix[(37, 160)])
        - groebner_matrix[(39, 183)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 116)] = groebner_matrix[(37, 182)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 117)] = groebner_matrix[(37, 183)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 121)] = -groebner_matrix[(39, 184)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 123)] = groebner_matrix[(37, 184)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 128)] = -groebner_matrix[(39, 185)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 130)] = groebner_matrix[(37, 185)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 136)] = -groebner_matrix[(39, 186)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 138)] = groebner_matrix[(37, 186)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 148)] = -groebner_matrix[(39, 187)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 149)] = -groebner_matrix[(39, 188)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 150)] = -groebner_matrix[(39, 189)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 151)] = -groebner_matrix[(39, 190)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 155)] = -groebner_matrix[(39, 191)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 157)] = groebner_matrix[(37, 187)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 158)] = groebner_matrix[(37, 188)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 159)] = groebner_matrix[(37, 189)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 160)] = groebner_matrix[(37, 190)] / (groebner_matrix[(37, 160)])
        - groebner_matrix[(39, 192)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 161)] = groebner_matrix[(37, 191)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 162)] = groebner_matrix[(37, 192)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 166)] = -groebner_matrix[(39, 193)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 168)] = groebner_matrix[(37, 193)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 173)] = -groebner_matrix[(39, 194)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 175)] = groebner_matrix[(37, 194)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 181)] = -groebner_matrix[(39, 195)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 183)] = groebner_matrix[(37, 195)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(56, 190)] = -groebner_matrix[(39, 196)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(56, 192)] = groebner_matrix[(37, 196)] / (groebner_matrix[(37, 160)]);
}
pub fn s_polynomial57(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(57, 36)] = groebner_matrix[(36, 160)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 37)] = groebner_matrix[(36, 161)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 38)] = groebner_matrix[(36, 162)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 64)] = -groebner_matrix[(39, 170)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 65)] = -groebner_matrix[(39, 171)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 69)] = -groebner_matrix[(39, 173)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 73)] = -groebner_matrix[(39, 174)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 76)] = groebner_matrix[(36, 170)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 77)] = groebner_matrix[(36, 171)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 79)] = groebner_matrix[(36, 173)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 80)] = groebner_matrix[(36, 174)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 84)] = -groebner_matrix[(39, 176)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 87)] = groebner_matrix[(36, 176)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 91)] = -groebner_matrix[(39, 177)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 94)] = groebner_matrix[(36, 177)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 100)] = -groebner_matrix[(39, 178)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 101)] = -groebner_matrix[(39, 179)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 102)] = -groebner_matrix[(39, 180)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 105)] = -groebner_matrix[(39, 181)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 109)] = -groebner_matrix[(39, 182)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 112)] = groebner_matrix[(36, 178)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 113)] = groebner_matrix[(36, 179)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 114)] = groebner_matrix[(36, 180)] / (groebner_matrix[(36, 159)])
        - groebner_matrix[(39, 183)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 115)] = groebner_matrix[(36, 181)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 116)] = groebner_matrix[(36, 182)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 117)] = groebner_matrix[(36, 183)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 120)] = -groebner_matrix[(39, 184)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 123)] = groebner_matrix[(36, 184)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 127)] = -groebner_matrix[(39, 185)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 130)] = groebner_matrix[(36, 185)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 135)] = -groebner_matrix[(39, 186)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 138)] = groebner_matrix[(36, 186)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 145)] = -groebner_matrix[(39, 187)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 146)] = -groebner_matrix[(39, 188)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 147)] = -groebner_matrix[(39, 189)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 150)] = -groebner_matrix[(39, 190)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 154)] = -groebner_matrix[(39, 191)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 157)] = groebner_matrix[(36, 187)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 158)] = groebner_matrix[(36, 188)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 159)] = groebner_matrix[(36, 189)] / (groebner_matrix[(36, 159)])
        - groebner_matrix[(39, 192)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 160)] = groebner_matrix[(36, 190)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 161)] = groebner_matrix[(36, 191)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 162)] = groebner_matrix[(36, 192)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 165)] = -groebner_matrix[(39, 193)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 168)] = groebner_matrix[(36, 193)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 172)] = -groebner_matrix[(39, 194)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 175)] = groebner_matrix[(36, 194)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 180)] = -groebner_matrix[(39, 195)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 183)] = groebner_matrix[(36, 195)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(57, 189)] = -groebner_matrix[(39, 196)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(57, 192)] = groebner_matrix[(36, 196)] / (groebner_matrix[(36, 159)]);
}
pub fn s_polynomial58(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(58, 35)] = groebner_matrix[(35, 159)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 36)] = groebner_matrix[(35, 160)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 37)] = groebner_matrix[(35, 161)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 38)] = groebner_matrix[(35, 162)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 62)] = -groebner_matrix[(39, 170)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 63)] = -groebner_matrix[(39, 171)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 68)] = -groebner_matrix[(39, 173)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 72)] = -groebner_matrix[(39, 174)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 76)] = groebner_matrix[(35, 170)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 77)] = groebner_matrix[(35, 171)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 79)] = groebner_matrix[(35, 173)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 80)] = groebner_matrix[(35, 174)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 83)] = -groebner_matrix[(39, 176)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 87)] = groebner_matrix[(35, 176)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 90)] = -groebner_matrix[(39, 177)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 94)] = groebner_matrix[(35, 177)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 98)] = -groebner_matrix[(39, 178)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 99)] = -groebner_matrix[(39, 179)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 101)] = -groebner_matrix[(39, 180)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 104)] = -groebner_matrix[(39, 181)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 108)] = -groebner_matrix[(39, 182)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 112)] = groebner_matrix[(35, 178)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 113)] = groebner_matrix[(35, 179)] / (groebner_matrix[(35, 158)])
        - groebner_matrix[(39, 183)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 114)] = groebner_matrix[(35, 180)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 115)] = groebner_matrix[(35, 181)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 116)] = groebner_matrix[(35, 182)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 117)] = groebner_matrix[(35, 183)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 119)] = -groebner_matrix[(39, 184)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 123)] = groebner_matrix[(35, 184)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 126)] = -groebner_matrix[(39, 185)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 130)] = groebner_matrix[(35, 185)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 134)] = -groebner_matrix[(39, 186)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 138)] = groebner_matrix[(35, 186)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 143)] = -groebner_matrix[(39, 187)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 144)] = -groebner_matrix[(39, 188)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 146)] = -groebner_matrix[(39, 189)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 149)] = -groebner_matrix[(39, 190)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 153)] = -groebner_matrix[(39, 191)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 157)] = groebner_matrix[(35, 187)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 158)] = groebner_matrix[(35, 188)] / (groebner_matrix[(35, 158)])
        - groebner_matrix[(39, 192)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 159)] = groebner_matrix[(35, 189)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 160)] = groebner_matrix[(35, 190)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 161)] = groebner_matrix[(35, 191)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 162)] = groebner_matrix[(35, 192)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 164)] = -groebner_matrix[(39, 193)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 168)] = groebner_matrix[(35, 193)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 171)] = -groebner_matrix[(39, 194)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 175)] = groebner_matrix[(35, 194)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 179)] = -groebner_matrix[(39, 195)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 183)] = groebner_matrix[(35, 195)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(58, 188)] = -groebner_matrix[(39, 196)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(58, 192)] = groebner_matrix[(35, 196)] / (groebner_matrix[(35, 158)]);
}
pub fn s_polynomial59(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(59, 34)] = groebner_matrix[(34, 158)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 35)] = groebner_matrix[(34, 159)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 36)] = groebner_matrix[(34, 160)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 37)] = groebner_matrix[(34, 161)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 38)] = groebner_matrix[(34, 162)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 61)] = -groebner_matrix[(39, 170)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 62)] = -groebner_matrix[(39, 171)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 67)] = -groebner_matrix[(39, 173)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 71)] = -groebner_matrix[(39, 174)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 76)] = groebner_matrix[(34, 170)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 77)] = groebner_matrix[(34, 171)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 79)] = groebner_matrix[(34, 173)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 80)] = groebner_matrix[(34, 174)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 82)] = -groebner_matrix[(39, 176)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 87)] = groebner_matrix[(34, 176)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 89)] = -groebner_matrix[(39, 177)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 94)] = groebner_matrix[(34, 177)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 97)] = -groebner_matrix[(39, 178)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 98)] = -groebner_matrix[(39, 179)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 100)] = -groebner_matrix[(39, 180)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 103)] = -groebner_matrix[(39, 181)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 107)] = -groebner_matrix[(39, 182)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 112)] = groebner_matrix[(34, 178)] / (groebner_matrix[(34, 157)])
        - groebner_matrix[(39, 183)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 113)] = groebner_matrix[(34, 179)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 114)] = groebner_matrix[(34, 180)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 115)] = groebner_matrix[(34, 181)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 116)] = groebner_matrix[(34, 182)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 117)] = groebner_matrix[(34, 183)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 118)] = -groebner_matrix[(39, 184)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 123)] = groebner_matrix[(34, 184)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 125)] = -groebner_matrix[(39, 185)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 130)] = groebner_matrix[(34, 185)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 133)] = -groebner_matrix[(39, 186)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 138)] = groebner_matrix[(34, 186)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 142)] = -groebner_matrix[(39, 187)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 143)] = -groebner_matrix[(39, 188)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 145)] = -groebner_matrix[(39, 189)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 148)] = -groebner_matrix[(39, 190)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 152)] = -groebner_matrix[(39, 191)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 157)] = groebner_matrix[(34, 187)] / (groebner_matrix[(34, 157)])
        - groebner_matrix[(39, 192)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 158)] = groebner_matrix[(34, 188)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 159)] = groebner_matrix[(34, 189)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 160)] = groebner_matrix[(34, 190)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 161)] = groebner_matrix[(34, 191)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 162)] = groebner_matrix[(34, 192)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 163)] = -groebner_matrix[(39, 193)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 168)] = groebner_matrix[(34, 193)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 170)] = -groebner_matrix[(39, 194)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 175)] = groebner_matrix[(34, 194)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 178)] = -groebner_matrix[(39, 195)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 183)] = groebner_matrix[(34, 195)] / (groebner_matrix[(34, 157)]);
    groebner_matrix[(59, 187)] = -groebner_matrix[(39, 196)] / (groebner_matrix[(39, 162)]);
    groebner_matrix[(59, 192)] = groebner_matrix[(34, 196)] / (groebner_matrix[(34, 157)]);
}
pub fn s_polynomial60(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(60, 33)] = groebner_matrix[(33, 157)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 34)] = groebner_matrix[(33, 158)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 35)] = groebner_matrix[(33, 159)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 36)] = groebner_matrix[(33, 160)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 37)] = groebner_matrix[(33, 161)] / (groebner_matrix[(33, 156)])
        - groebner_matrix[(38, 162)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 38)] = groebner_matrix[(33, 162)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 71)] = -groebner_matrix[(38, 170)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 72)] = -groebner_matrix[(38, 171)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 74)] = -groebner_matrix[(38, 173)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 75)] = -groebner_matrix[(38, 174)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 76)] = groebner_matrix[(33, 170)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 77)] = groebner_matrix[(33, 171)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 79)] = groebner_matrix[(33, 173)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 80)] = groebner_matrix[(33, 174)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 86)] = -groebner_matrix[(38, 176)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 87)] = groebner_matrix[(33, 176)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 93)] = -groebner_matrix[(38, 177)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 94)] = groebner_matrix[(33, 177)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 107)] = -groebner_matrix[(38, 178)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 108)] = -groebner_matrix[(38, 179)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 109)] = -groebner_matrix[(38, 180)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 110)] = -groebner_matrix[(38, 181)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 111)] = -groebner_matrix[(38, 182)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 112)] = groebner_matrix[(33, 178)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 113)] = groebner_matrix[(33, 179)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 114)] = groebner_matrix[(33, 180)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 115)] = groebner_matrix[(33, 181)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 116)] = groebner_matrix[(33, 182)] / (groebner_matrix[(33, 156)])
        - groebner_matrix[(38, 183)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 117)] = groebner_matrix[(33, 183)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 122)] = -groebner_matrix[(38, 184)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 123)] = groebner_matrix[(33, 184)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 129)] = -groebner_matrix[(38, 185)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 130)] = groebner_matrix[(33, 185)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 137)] = -groebner_matrix[(38, 186)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 138)] = groebner_matrix[(33, 186)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 152)] = -groebner_matrix[(38, 187)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 153)] = -groebner_matrix[(38, 188)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 154)] = -groebner_matrix[(38, 189)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 155)] = -groebner_matrix[(38, 190)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 156)] = -groebner_matrix[(38, 191)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 157)] = groebner_matrix[(33, 187)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 158)] = groebner_matrix[(33, 188)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 159)] = groebner_matrix[(33, 189)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 160)] = groebner_matrix[(33, 190)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 161)] = groebner_matrix[(33, 191)] / (groebner_matrix[(33, 156)])
        - groebner_matrix[(38, 192)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 162)] = groebner_matrix[(33, 192)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 167)] = -groebner_matrix[(38, 193)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 168)] = groebner_matrix[(33, 193)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 174)] = -groebner_matrix[(38, 194)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 175)] = groebner_matrix[(33, 194)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 182)] = -groebner_matrix[(38, 195)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 183)] = groebner_matrix[(33, 195)] / (groebner_matrix[(33, 156)]);
    groebner_matrix[(60, 191)] = -groebner_matrix[(38, 196)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(60, 192)] = groebner_matrix[(33, 196)] / (groebner_matrix[(33, 156)]);
}
pub fn s_polynomial61(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(61, 32)] = groebner_matrix[(32, 156)] / (groebner_matrix[(32, 155)])
        - groebner_matrix[(37, 161)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 33)] = groebner_matrix[(32, 157)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 34)] = groebner_matrix[(32, 158)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 35)] = groebner_matrix[(32, 159)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 36)] = groebner_matrix[(32, 160)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 37)] = groebner_matrix[(32, 161)] / (groebner_matrix[(32, 155)])
        - groebner_matrix[(37, 162)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 38)] = groebner_matrix[(32, 162)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 71)] = -groebner_matrix[(37, 170)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 72)] = -groebner_matrix[(37, 171)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 74)] = -groebner_matrix[(37, 173)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 75)] = -groebner_matrix[(37, 174)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 76)] = groebner_matrix[(32, 170)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 77)] = groebner_matrix[(32, 171)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 79)] = groebner_matrix[(32, 173)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 80)] = groebner_matrix[(32, 174)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 86)] = -groebner_matrix[(37, 176)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 87)] = groebner_matrix[(32, 176)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 93)] = -groebner_matrix[(37, 177)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 94)] = groebner_matrix[(32, 177)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 107)] = -groebner_matrix[(37, 178)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 108)] = -groebner_matrix[(37, 179)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 109)] = -groebner_matrix[(37, 180)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 110)] = -groebner_matrix[(37, 181)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 111)] = -groebner_matrix[(37, 182)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 112)] = groebner_matrix[(32, 178)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 113)] = groebner_matrix[(32, 179)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 114)] = groebner_matrix[(32, 180)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 115)] = groebner_matrix[(32, 181)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 116)] = groebner_matrix[(32, 182)] / (groebner_matrix[(32, 155)])
        - groebner_matrix[(37, 183)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 117)] = groebner_matrix[(32, 183)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 122)] = -groebner_matrix[(37, 184)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 123)] = groebner_matrix[(32, 184)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 129)] = -groebner_matrix[(37, 185)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 130)] = groebner_matrix[(32, 185)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 137)] = -groebner_matrix[(37, 186)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 138)] = groebner_matrix[(32, 186)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 152)] = -groebner_matrix[(37, 187)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 153)] = -groebner_matrix[(37, 188)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 154)] = -groebner_matrix[(37, 189)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 155)] = -groebner_matrix[(37, 190)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 156)] = -groebner_matrix[(37, 191)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 157)] = groebner_matrix[(32, 187)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 158)] = groebner_matrix[(32, 188)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 159)] = groebner_matrix[(32, 189)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 160)] = groebner_matrix[(32, 190)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 161)] = groebner_matrix[(32, 191)] / (groebner_matrix[(32, 155)])
        - groebner_matrix[(37, 192)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 162)] = groebner_matrix[(32, 192)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 167)] = -groebner_matrix[(37, 193)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 168)] = groebner_matrix[(32, 193)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 174)] = -groebner_matrix[(37, 194)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 175)] = groebner_matrix[(32, 194)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 182)] = -groebner_matrix[(37, 195)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 183)] = groebner_matrix[(32, 195)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(61, 191)] = -groebner_matrix[(37, 196)] / (groebner_matrix[(37, 160)]);
    groebner_matrix[(61, 192)] = groebner_matrix[(32, 196)] / (groebner_matrix[(32, 155)]);
}
pub fn s_polynomial62(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(62, 32)] = groebner_matrix[(32, 156)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 33)] = groebner_matrix[(32, 157)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 34)] = groebner_matrix[(32, 158)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 35)] = groebner_matrix[(32, 159)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 36)] = groebner_matrix[(32, 160)] / (groebner_matrix[(32, 155)])
        - groebner_matrix[(38, 162)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 37)] = groebner_matrix[(32, 161)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 38)] = groebner_matrix[(32, 162)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 67)] = -groebner_matrix[(38, 170)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 68)] = -groebner_matrix[(38, 171)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 70)] = -groebner_matrix[(38, 173)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 74)] = -groebner_matrix[(38, 174)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 76)] = groebner_matrix[(32, 170)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 77)] = groebner_matrix[(32, 171)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 79)] = groebner_matrix[(32, 173)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 80)] = groebner_matrix[(32, 174)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 85)] = -groebner_matrix[(38, 176)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 87)] = groebner_matrix[(32, 176)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 92)] = -groebner_matrix[(38, 177)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 94)] = groebner_matrix[(32, 177)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 103)] = -groebner_matrix[(38, 178)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 104)] = -groebner_matrix[(38, 179)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 105)] = -groebner_matrix[(38, 180)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 106)] = -groebner_matrix[(38, 181)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 110)] = -groebner_matrix[(38, 182)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 112)] = groebner_matrix[(32, 178)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 113)] = groebner_matrix[(32, 179)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 114)] = groebner_matrix[(32, 180)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 115)] = groebner_matrix[(32, 181)] / (groebner_matrix[(32, 155)])
        - groebner_matrix[(38, 183)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 116)] = groebner_matrix[(32, 182)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 117)] = groebner_matrix[(32, 183)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 121)] = -groebner_matrix[(38, 184)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 123)] = groebner_matrix[(32, 184)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 128)] = -groebner_matrix[(38, 185)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 130)] = groebner_matrix[(32, 185)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 136)] = -groebner_matrix[(38, 186)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 138)] = groebner_matrix[(32, 186)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 148)] = -groebner_matrix[(38, 187)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 149)] = -groebner_matrix[(38, 188)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 150)] = -groebner_matrix[(38, 189)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 151)] = -groebner_matrix[(38, 190)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 155)] = -groebner_matrix[(38, 191)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 157)] = groebner_matrix[(32, 187)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 158)] = groebner_matrix[(32, 188)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 159)] = groebner_matrix[(32, 189)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 160)] = groebner_matrix[(32, 190)] / (groebner_matrix[(32, 155)])
        - groebner_matrix[(38, 192)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 161)] = groebner_matrix[(32, 191)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 162)] = groebner_matrix[(32, 192)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 166)] = -groebner_matrix[(38, 193)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 168)] = groebner_matrix[(32, 193)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 173)] = -groebner_matrix[(38, 194)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 175)] = groebner_matrix[(32, 194)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 181)] = -groebner_matrix[(38, 195)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 183)] = groebner_matrix[(32, 195)] / (groebner_matrix[(32, 155)]);
    groebner_matrix[(62, 190)] = -groebner_matrix[(38, 196)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(62, 192)] = groebner_matrix[(32, 196)] / (groebner_matrix[(32, 155)]);
}
pub fn s_polynomial63(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(63, 31)] = -groebner_matrix[(36, 160)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 32)] = -groebner_matrix[(36, 161)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 34)] = groebner_matrix[(16, 158)] / (groebner_matrix[(16, 154)]);
    groebner_matrix[(63, 37)] = -groebner_matrix[(36, 162)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 71)] = -groebner_matrix[(36, 170)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 72)] = -groebner_matrix[(36, 171)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 74)] = -groebner_matrix[(36, 173)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 75)] = -groebner_matrix[(36, 174)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 86)] = -groebner_matrix[(36, 176)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 93)] = -groebner_matrix[(36, 177)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 107)] = -groebner_matrix[(36, 178)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 108)] = -groebner_matrix[(36, 179)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 109)] = -groebner_matrix[(36, 180)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 110)] = -groebner_matrix[(36, 181)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 111)] = -groebner_matrix[(36, 182)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 116)] = -groebner_matrix[(36, 183)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 122)] = -groebner_matrix[(36, 184)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 129)] = -groebner_matrix[(36, 185)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 137)] = -groebner_matrix[(36, 186)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 152)] = -groebner_matrix[(36, 187)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 153)] = -groebner_matrix[(36, 188)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 154)] = -groebner_matrix[(36, 189)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 155)] = -groebner_matrix[(36, 190)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 156)] = -groebner_matrix[(36, 191)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 161)] = -groebner_matrix[(36, 192)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 167)] = -groebner_matrix[(36, 193)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 168)] = groebner_matrix[(16, 193)] / (groebner_matrix[(16, 154)]);
    groebner_matrix[(63, 174)] = -groebner_matrix[(36, 194)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 182)] = -groebner_matrix[(36, 195)] / (groebner_matrix[(36, 159)]);
    groebner_matrix[(63, 191)] = -groebner_matrix[(36, 196)] / (groebner_matrix[(36, 159)]);
}
pub fn s_polynomial64(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(64, 34)] = groebner_matrix[(16, 158)] / (groebner_matrix[(16, 154)]);
    groebner_matrix[(64, 35)] = -groebner_matrix[(38, 162)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 64)] = -groebner_matrix[(38, 170)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 65)] = -groebner_matrix[(38, 171)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 69)] = -groebner_matrix[(38, 173)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 73)] = -groebner_matrix[(38, 174)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 84)] = -groebner_matrix[(38, 176)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 91)] = -groebner_matrix[(38, 177)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 100)] = -groebner_matrix[(38, 178)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 101)] = -groebner_matrix[(38, 179)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 102)] = -groebner_matrix[(38, 180)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 105)] = -groebner_matrix[(38, 181)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 109)] = -groebner_matrix[(38, 182)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 114)] = -groebner_matrix[(38, 183)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 120)] = -groebner_matrix[(38, 184)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 127)] = -groebner_matrix[(38, 185)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 135)] = -groebner_matrix[(38, 186)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 145)] = -groebner_matrix[(38, 187)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 146)] = -groebner_matrix[(38, 188)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 147)] = -groebner_matrix[(38, 189)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 150)] = -groebner_matrix[(38, 190)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 154)] = -groebner_matrix[(38, 191)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 159)] = -groebner_matrix[(38, 192)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 165)] = -groebner_matrix[(38, 193)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 168)] = groebner_matrix[(16, 193)] / (groebner_matrix[(16, 154)]);
    groebner_matrix[(64, 172)] = -groebner_matrix[(38, 194)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 180)] = -groebner_matrix[(38, 195)] / (groebner_matrix[(38, 161)]);
    groebner_matrix[(64, 189)] = -groebner_matrix[(38, 196)] / (groebner_matrix[(38, 161)]);
}
pub fn s_polynomial65(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_matrix[(65, 30)] = -groebner_matrix[(35, 159)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 31)] = groebner_matrix[(31, 155)] / (groebner_matrix[(31, 153)])
        - groebner_matrix[(35, 160)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 32)] = groebner_matrix[(31, 156)] / (groebner_matrix[(31, 153)])
        - groebner_matrix[(35, 161)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 33)] = groebner_matrix[(31, 157)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 34)] = groebner_matrix[(31, 158)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 35)] = groebner_matrix[(31, 159)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 36)] = groebner_matrix[(31, 160)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 37)] = groebner_matrix[(31, 161)] / (groebner_matrix[(31, 153)])
        - groebner_matrix[(35, 162)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 38)] = groebner_matrix[(31, 162)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 71)] = -groebner_matrix[(35, 170)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 72)] = -groebner_matrix[(35, 171)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 74)] = -groebner_matrix[(35, 173)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 75)] = -groebner_matrix[(35, 174)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 76)] = groebner_matrix[(31, 170)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 77)] = groebner_matrix[(31, 171)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 79)] = groebner_matrix[(31, 173)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 80)] = groebner_matrix[(31, 174)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 86)] = -groebner_matrix[(35, 176)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 87)] = groebner_matrix[(31, 176)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 93)] = -groebner_matrix[(35, 177)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 94)] = groebner_matrix[(31, 177)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 107)] = -groebner_matrix[(35, 178)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 108)] = -groebner_matrix[(35, 179)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 109)] = -groebner_matrix[(35, 180)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 110)] = -groebner_matrix[(35, 181)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 111)] = -groebner_matrix[(35, 182)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 112)] = groebner_matrix[(31, 178)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 113)] = groebner_matrix[(31, 179)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 114)] = groebner_matrix[(31, 180)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 115)] = groebner_matrix[(31, 181)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 116)] = groebner_matrix[(31, 182)] / (groebner_matrix[(31, 153)])
        - groebner_matrix[(35, 183)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 117)] = groebner_matrix[(31, 183)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 122)] = -groebner_matrix[(35, 184)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 123)] = groebner_matrix[(31, 184)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 129)] = -groebner_matrix[(35, 185)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 130)] = groebner_matrix[(31, 185)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 137)] = -groebner_matrix[(35, 186)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 138)] = groebner_matrix[(31, 186)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 152)] = -groebner_matrix[(35, 187)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 153)] = -groebner_matrix[(35, 188)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 154)] = -groebner_matrix[(35, 189)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 155)] = -groebner_matrix[(35, 190)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 156)] = -groebner_matrix[(35, 191)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 157)] = groebner_matrix[(31, 187)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 158)] = groebner_matrix[(31, 188)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 159)] = groebner_matrix[(31, 189)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 160)] = groebner_matrix[(31, 190)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 161)] = groebner_matrix[(31, 191)] / (groebner_matrix[(31, 153)])
        - groebner_matrix[(35, 192)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 162)] = groebner_matrix[(31, 192)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 167)] = -groebner_matrix[(35, 193)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 168)] = groebner_matrix[(31, 193)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 174)] = -groebner_matrix[(35, 194)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 175)] = groebner_matrix[(31, 194)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 182)] = -groebner_matrix[(35, 195)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 183)] = groebner_matrix[(31, 195)] / (groebner_matrix[(31, 153)]);
    groebner_matrix[(65, 191)] = -groebner_matrix[(35, 196)] / (groebner_matrix[(35, 158)]);
    groebner_matrix[(65, 192)] = groebner_matrix[(31, 196)] / (groebner_matrix[(31, 153)]);
}
