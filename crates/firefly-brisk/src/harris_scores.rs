/*
Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
Stefan Leutenegger, Simon Lynen and Margarita Chli.

Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
Stefan Leutenegger and Simon Lynen.

Copyright (C) 2020  Smart Robotics Lab, Imperial College London
Stefan Leutenegger

Copyright (C) 2024  Smart Robotics Lab, Technical University of Munich
Stefan Leutenegger

All rights reserved.

This is the Author's implementation of BRISK: Binary Robust Invariant
Scalable Keypoints [1]. Various (partly unpublished) extensions are provided,
some of which are described in [2].

[1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary
    Robust Invariant Scalable Keypoints. In Proceedings of the IEEE
    International Conference on Computer Vision (ICCV), 2011.

[2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
    Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

This file is part of BRISK.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//! Integer Scharr product scores with the source fixed-point Harris weighting.
use crate::vectorized_filters::{check, invalid};
use firefly_error::Result;
use purecv::core::Matrix;
/// # Errors
/// Returns an error if source image storage or derivative/filter support is invalid.
pub fn harris_scores_sse(src: &Matrix<u8>) -> Result<Matrix<i32>> {
    check(src)?;
    if src.cols < 18 && src.rows > 2 {
        return Err(invalid("Harris gradient requires at least 18 columns"));
    }
    let mut dst = Matrix::new(src.rows, src.cols, 1);
    let n = src.data.len();
    let (mut xx, mut yy, mut xy) = (vec![0i16; n], vec![0i16; n], vec![0i16; n]);
    for y in 1..src.rows.saturating_sub(1) {
        for x in 1..src.cols.saturating_sub(1) {
            let at = |dy: isize, dx: isize| {
                i32::from(
                    src.data[(y as isize + dy) as usize * src.cols + (x as isize + dx) as usize],
                )
            };
            let dx = ((10 * (at(0, -1) - at(0, 1))
                + 3 * (at(-1, -1) - at(-1, 1))
                + 3 * (at(1, -1) - at(1, 1))) as i16)
                .wrapping_shl(3);
            let dy = ((10 * (at(-1, 0) - at(1, 0))
                + 3 * (at(-1, -1) - at(1, -1))
                + 3 * (at(-1, 1) - at(1, 1))) as i16)
                .wrapping_shl(3);
            let i = y * src.cols + x;
            xx[i] = ((i32::from(dx) * i32::from(dx)) >> 16) as i16;
            yy[i] = ((i32::from(dy) * i32::from(dy)) >> 16) as i16;
            xy[i] = ((i32::from(dx) * i32::from(dy)) >> 16) as i16;
        }
    }
    for y in 2..src.rows.saturating_sub(2) {
        for x in 2..src.cols.saturating_sub(2) {
            let smooth = |m: &[i16]| {
                let at = |dy: isize, dx: isize| {
                    i32::from(m[(y as isize + dy) as usize * src.cols + (x as isize + dx) as usize])
                };
                (4 * at(0, 0)
                    + 2 * (at(-1, 0) + at(1, 0) + at(0, -1) + at(0, 1))
                    + (at(-1, -1) + at(-1, 1) + at(1, -1) + at(1, 1)))
                    >> 4
            };
            let a = smooth(&xx);
            let b = smooth(&yy);
            let c = smooth(&xy);
            let trace = (a + b) >> 1;
            dst.data[y * src.cols + x] = a
                .wrapping_mul(b)
                .wrapping_sub(c.wrapping_mul(c))
                .wrapping_sub(trace.wrapping_mul(trace) / 5);
        }
    }
    Ok(dst)
}
