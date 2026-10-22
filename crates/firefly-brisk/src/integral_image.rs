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

//! Summed images with the source's integer lanes and u16 block/tail conversion.
use crate::vectorized_filters::{check, invalid};
use firefly_error::Result;
use purecv::core::Matrix;
/// # Errors
/// Returns an error if source image storage is malformed or the integral shape overflows.
pub fn integral_image_8(src: &Matrix<u8>) -> Result<Matrix<i32>> {
    check(src)?;
    let rows = src
        .rows
        .checked_add(1)
        .ok_or_else(|| invalid("integral rows overflow"))?;
    let cols = src
        .cols
        .checked_add(1)
        .ok_or_else(|| invalid("integral columns overflow"))?;
    rows.checked_mul(cols)
        .ok_or_else(|| invalid("integral image size overflow"))?;
    let mut dst = Matrix::<i32>::new(rows, cols, 1);
    for y in 0..src.rows {
        let mut s = 0i32;
        for x in 0..src.cols {
            s = s.wrapping_add(i32::from(src.data[y * src.cols + x]));
            dst.data[(y + 1) * cols + x + 1] = dst.data[y * cols + x + 1].wrapping_add(s);
        }
    }
    Ok(dst)
}
/// # Errors
/// Returns an error if source image storage is malformed or the integral shape overflows.
pub fn integral_image_16(src: &Matrix<u16>) -> Result<Matrix<f32>> {
    check(src)?;
    let rows = src
        .rows
        .checked_add(1)
        .ok_or_else(|| invalid("integral rows overflow"))?;
    let cols = src
        .cols
        .checked_add(1)
        .ok_or_else(|| invalid("integral columns overflow"))?;
    rows.checked_mul(cols)
        .ok_or_else(|| invalid("integral image size overflow"))?;
    let mut dst = Matrix::<f32>::new(rows, cols, 1);
    let block_end = src.cols / 4 * 4;
    for y in 0..src.rows {
        let mut s = 0f32;
        for x in 0..src.cols {
            let v = f32::from(src.data[y * src.cols + x]);
            s += if x < block_end {
                v * (1.0 / 65536.0)
            } else {
                v
            };
            dst.data[(y + 1) * cols + x + 1] = dst.data[y * cols + x + 1] + s;
        }
    }
    Ok(dst)
}
