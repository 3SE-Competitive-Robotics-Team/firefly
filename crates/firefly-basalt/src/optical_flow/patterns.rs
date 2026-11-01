/*
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//! Translation of `include/basalt/optical_flow/patterns.h` at upstream
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

use nalgebra::{RealField, SMatrix};

/// A sampling pattern with one offset per matrix column.
pub trait Pattern<S: RealField + Copy, const N: usize> {
    /// Number of samples (including the historical 50/51 names with 52 samples).
    const PATTERN_SIZE: usize = N;

    /// Offsets in the exact upstream order.
    fn pattern2() -> SMatrix<S, 2, N>;
}

/// The original 24-point sampling pattern.
pub struct Pattern24;
/// The original 52-point sampling pattern.
pub struct Pattern52;
/// The 52-point pattern scaled by one half.
pub struct Pattern51;
/// The 52-point pattern scaled by three quarters.
pub struct Pattern50;

impl Pattern24 {
    /// Integer offsets before scalar conversion.
    pub const PATTERN_RAW: [[i16; 2]; 24] = [
        [-1, 5],
        [1, 5],
        [-3, 3],
        [-1, 3],
        [1, 3],
        [3, 3],
        [-5, 1],
        [-3, 1],
        [-1, 1],
        [1, 1],
        [3, 1],
        [5, 1],
        [-5, -1],
        [-3, -1],
        [-1, -1],
        [1, -1],
        [3, -1],
        [5, -1],
        [-3, -3],
        [-1, -3],
        [1, -3],
        [3, -3],
        [-1, -5],
        [1, -5],
    ];
}

impl Pattern52 {
    /// Integer offsets before scalar conversion.
    pub const PATTERN_RAW: [[i16; 2]; 52] = [
        [-3, 7],
        [-1, 7],
        [1, 7],
        [3, 7],
        [-5, 5],
        [-3, 5],
        [-1, 5],
        [1, 5],
        [3, 5],
        [5, 5],
        [-7, 3],
        [-5, 3],
        [-3, 3],
        [-1, 3],
        [1, 3],
        [3, 3],
        [5, 3],
        [7, 3],
        [-7, 1],
        [-5, 1],
        [-3, 1],
        [-1, 1],
        [1, 1],
        [3, 1],
        [5, 1],
        [7, 1],
        [-7, -1],
        [-5, -1],
        [-3, -1],
        [-1, -1],
        [1, -1],
        [3, -1],
        [5, -1],
        [7, -1],
        [-7, -3],
        [-5, -3],
        [-3, -3],
        [-1, -3],
        [1, -3],
        [3, -3],
        [5, -3],
        [7, -3],
        [-5, -5],
        [-3, -5],
        [-1, -5],
        [1, -5],
        [3, -5],
        [5, -5],
        [-3, -7],
        [-1, -7],
        [1, -7],
        [3, -7],
    ];
}

impl<S: RealField + Copy> Pattern<S, 24> for Pattern24 {
    fn pattern2() -> SMatrix<S, 2, 24> {
        SMatrix::from_fn(|row, col| S::from_i16(Self::PATTERN_RAW[col][row]).unwrap())
    }
}

impl<S: RealField + Copy> Pattern<S, 52> for Pattern52 {
    fn pattern2() -> SMatrix<S, 2, 52> {
        SMatrix::from_fn(|row, col| S::from_i16(Self::PATTERN_RAW[col][row]).unwrap())
    }
}

impl<S: RealField + Copy> Pattern<S, 52> for Pattern51 {
    fn pattern2() -> SMatrix<S, 2, 52> {
        <Pattern52 as Pattern<S, 52>>::pattern2() * S::from_f64(0.5).unwrap()
    }
}

impl<S: RealField + Copy> Pattern<S, 52> for Pattern50 {
    fn pattern2() -> SMatrix<S, 2, 52> {
        <Pattern52 as Pattern<S, 52>>::pattern2() * S::from_f64(0.75).unwrap()
    }
}
