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

//! Fixed-width filter arithmetic and block traversal corresponding to the SIMD kernels.
use firefly_error::{Error, ErrorKind, Result};
use purecv::core::Matrix;
pub(crate) fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("BRISK")
}
pub(crate) fn check<T>(m: &Matrix<T>) -> Result<()> {
    if m.channels != 1 || m.rows.checked_mul(m.cols) != Some(m.data.len()) {
        return Err(invalid("single-channel contiguous image required"));
    }
    Ok(())
}
pub(crate) fn starts(cols: usize, divisor: usize, step: usize, lanes: usize) -> Result<Vec<usize>> {
    if cols < 2 {
        return Ok(Vec::new());
    }
    let max = (cols - 2) / divisor * divisor;
    if max == 0 {
        return Ok(Vec::new());
    }
    if cols < lanes + 2 {
        return Err(invalid("SIMD block exceeds row width"));
    }
    let (mut j, mut end) = (0, false);
    let mut result = Vec::new();
    while j < max {
        result.push(j);
        j += step;
        if j >= max && !end {
            j = cols - 2 - lanes;
            end = true;
        }
    }
    Ok(result)
}
pub(crate) fn sample<T: Copy>(m: &Matrix<T>, y: usize, x: usize) -> Result<T> {
    if y >= m.rows || x >= m.cols {
        return Err(invalid("filter support exceeds image"));
    }
    Ok(m.data[y * m.cols + x])
}
/// # Errors
/// Returns an error for malformed storage, an unsupported scalar type,
/// or a source/kernel shape incompatible with the filter support.
pub fn filter_2d_16s<const X: usize, const Y: usize>(
    src: &Matrix<i16>,
    kernel: &Matrix<i16>,
) -> Result<Matrix<i16>> {
    check(src)?;
    check(kernel)?;
    if kernel.rows != Y || kernel.cols != X || X.is_multiple_of(2) || Y.is_multiple_of(2) {
        return Err(invalid("odd filter dimensions must match kernel"));
    }
    let mut dst = Matrix::new(src.rows, src.cols, 1);
    for i in 0..src.rows.saturating_sub(2) {
        for j in starts(src.cols, 8, 8, 8)? {
            for lane in 0..8 {
                let mut sum = 0i16;
                for x in 0..X {
                    for y in 0..Y {
                        let m = i16::from(kernel.data[y * X + x] as i8);
                        if m != 0 {
                            sum =
                                sum.wrapping_add(sample(src, i + y, j + x + lane)?.wrapping_mul(m));
                        }
                    }
                }
                let dy = i + Y / 2;
                let dx = j + X / 2 + lane;
                if dy >= src.rows || dx >= src.cols {
                    return Err(invalid("filter destination exceeds image"));
                }
                dst.data[dy * src.cols + dx] = sum;
            }
        }
    }
    Ok(dst)
}
/// # Errors
/// Returns an error for malformed storage, an unsupported scalar type,
/// or a source/kernel shape incompatible with the filter support.
pub fn filter_2d_8u<const X: usize, const Y: usize>(
    src: &Matrix<u8>,
    kernel: &Matrix<i8>,
) -> Result<Matrix<i16>> {
    check(src)?;
    check(kernel)?;
    if kernel.rows != Y || kernel.cols != X || X.is_multiple_of(2) || Y.is_multiple_of(2) {
        return Err(invalid("odd filter dimensions must match kernel"));
    }
    let mut dst = Matrix::new(src.rows, src.cols, 1);
    for i in 0..src.rows.saturating_sub(2) {
        for j in starts(src.cols, 16, 16, 16)? {
            let (mut hi, mut lo) = ([0i16; 8], [0i16; 8]);
            for x in 0..X {
                for y in 0..Y {
                    let m = i16::from(kernel.data[y * X + x]);
                    if m == 0 {
                        continue;
                    }
                    for k in 0..8 {
                        let a = (u16::from(sample(src, i + y, j + x + 2 * k + 1)?) << 8) as i16;
                        let b = if k < 7 {
                            (u16::from(sample(src, i + y, j + x + 2 * k + 2)?) << 8) as i16
                        } else {
                            0
                        };
                        hi[k] = hi[k].wrapping_add(a.wrapping_mul(m));
                        lo[k] = lo[k].wrapping_add(b.wrapping_mul(m));
                    }
                }
            }
            for k in 0..8 {
                let dy = i + Y / 2;
                let dx = j + X / 2 + 2 * k;
                if dy >= src.rows || dx + 1 >= src.cols {
                    return Err(invalid("filter destination exceeds image"));
                }
                dst.data[dy * src.cols + dx] = hi[k];
                dst.data[dy * src.cols + dx + 1] = lo[k];
            }
        }
    }
    Ok(dst)
}
/// # Errors
/// Returns an error for malformed storage, an unsupported scalar type,
/// or a source/kernel shape incompatible with the filter support.
pub fn filter_gauss_3_by_3_16s(src: &Matrix<i16>) -> Result<Matrix<i16>> {
    filter_weighted_16s(src, true)
}
/// # Errors
/// Returns an error for malformed storage, an unsupported scalar type,
/// or a source/kernel shape incompatible with the filter support.
pub fn filter_box_3_by_3_16s(src: &Matrix<i16>) -> Result<Matrix<i16>> {
    filter_weighted_16s(src, false)
}
fn filter_weighted_16s(src: &Matrix<i16>, gauss: bool) -> Result<Matrix<i16>> {
    check(src)?;
    let mut dst = Matrix::new(src.rows, src.cols, 1);
    for i in 0..src.rows.saturating_sub(2) {
        for j in starts(src.cols, 8, 8, 8)? {
            for k in 0..8 {
                let x = j + k;
                let s = |y, dx| src.data[(i + y) * src.cols + x + dx];
                let mut sum = if gauss {
                    s(1, 1).wrapping_shl(2)
                } else {
                    s(1, 1)
                };
                for (y, dx) in [
                    (0, 0),
                    (2, 0),
                    (0, 2),
                    (2, 2),
                    (1, 0),
                    (0, 1),
                    (1, 2),
                    (2, 1),
                ] {
                    let v = s(y, dx);
                    sum = sum.wrapping_add(if gauss && (y == 1 || dx == 1) {
                        v.wrapping_shl(1)
                    } else {
                        v
                    });
                }
                dst.data[(i + 1) * src.cols + x + 1] = sum;
            }
        }
    }
    Ok(dst)
}
/// # Errors
/// Returns an error for malformed storage, an unsupported scalar type,
/// or a source/kernel shape incompatible with the filter support.
pub fn filter_gauss_3_by_3_32f(src: &Matrix<f32>) -> Result<Matrix<f32>> {
    check(src)?;
    let mut dst = Matrix::new(src.rows, src.cols, 1);
    for i in 0..src.rows.saturating_sub(2) {
        for j in starts(src.cols, 8, 4, 4)? {
            for k in 0..4 {
                let x = j + k;
                let mut sum = src.data[i * src.cols + x + 1] + src.data[(i + 1) * src.cols + x];
                sum += src.data[(i + 1) * src.cols + x + 2];
                sum += src.data[(i + 2) * src.cols + x + 1];
                dst.data[(i + 1) * src.cols + x + 1] = sum * 0.0625;
            }
        }
    }
    Ok(dst)
}

/// Dispatch according to the source and kernel depths used by the reference filters.
/// # Errors
/// Returns an error for malformed storage, an unsupported scalar type,
/// or a source/kernel shape incompatible with the filter support.
pub fn filter_2d<const X: usize, const Y: usize>(
    src: &purecv::core::DynamicMatrix,
    kernel: &purecv::core::DynamicMatrix,
) -> Result<Matrix<i16>> {
    use purecv::core::DynamicData;
    match (&src.data, &kernel.data) {
        (DynamicData::U8(src), DynamicData::I8(kernel)) => filter_2d_8u::<X, Y>(src, kernel),
        (DynamicData::I16(src), DynamicData::I16(kernel)) => filter_2d_16s::<X, Y>(src, kernel),
        _ => Err(invalid("filter image or kernel depth is unsupported")),
    }
}
