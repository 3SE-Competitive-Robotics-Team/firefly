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

//! Half and two-thirds scale image reductions, retaining block/tail rounding.
use crate::vectorized_filters::{check, invalid};
use firefly_error::Result;
use purecv::core::Matrix;
fn target<S, T>(
    src: &Matrix<S>,
    dst: &Matrix<T>,
    numerator: usize,
    denominator: usize,
) -> Result<()> {
    check(src)?;
    check(dst)?;
    if src.rows / denominator * numerator != dst.rows
        || src.cols / denominator * numerator != dst.cols
    {
        return Err(invalid("downsample destination dimensions differ"));
    }
    Ok(())
}
fn average(a: u32, b: u32) -> u32 {
    (a + b).div_ceil(2)
}
/// # Errors
/// Returns an error if either image is malformed or destination dimensions do not match
/// the required reduction ratio.
pub fn halfsample_8(src: &Matrix<u8>, dst: &mut Matrix<u8>) -> Result<()> {
    target(src, dst, 1, 2)?;
    let blocks = src.cols / 16;
    let full_end = blocks / 2 * 32;
    let odd_block = blocks % 2 == 1;
    for y in 0..dst.rows {
        for x in 0..dst.cols {
            let sx = 2 * x;
            let a = u32::from(src.data[2 * y * src.cols + sx]);
            let b = u32::from(src.data[2 * y * src.cols + sx + 1]);
            let c = u32::from(src.data[(2 * y + 1) * src.cols + sx]);
            let d = u32::from(src.data[(2 * y + 1) * src.cols + sx + 1]);
            dst.data[y * dst.cols + x] = if sx < full_end {
                average(average(a, c), average(b, d)) as u8
            } else if odd_block && sx < blocks * 16 {
                (average(a, c).midpoint(average(b, d))) as u8
            } else {
                ((a + b + c + d + 2) / 4) as u8
            };
        }
    }
    Ok(())
}
/// # Errors
/// Returns an error for malformed images or mismatched output dimensions.
/// The 16-bit half-sampler also returns an error on NEON targets.
pub fn halfsample_16(src: &Matrix<u16>, dst: &mut Matrix<u16>) -> Result<()> {
    target(src, dst, 1, 2)?;
    #[cfg(any(target_arch = "aarch64", target_feature = "neon"))]
    {
        return Err(invalid("Halfsample16 is not implemented for NEON"));
    }
    #[cfg(not(any(target_arch = "aarch64", target_feature = "neon")))]
    {
        if src.cols / 2 * 2 < 16 {
            return Ok(());
        }
        for y in 0..dst.rows {
            for x in 0..dst.cols {
                let sx = 2 * x;
                let a = u32::from(src.data[2 * y * src.cols + sx]);
                let b = u32::from(src.data[2 * y * src.cols + sx + 1]);
                let c = u32::from(
                    src.data[(2 * y + 1) * src.cols + sx]
                        .saturating_add(1)
                        .saturating_add(1),
                );
                let d = u32::from(src.data[(2 * y + 1) * src.cols + sx + 1]);
                dst.data[y * dst.cols + x] = average(average(a, b), average(c, d)) as u16;
            }
        }
        Ok(())
    }
}
/// # Errors
/// Returns an error if either image is malformed or destination dimensions do not match
/// the required reduction ratio.
pub fn twothirdsample_8(src: &Matrix<u8>, dst: &mut Matrix<u8>) -> Result<()> {
    target(src, dst, 2, 3)?;
    let blocks = src.cols / 15;
    for by in 0..src.rows / 3 {
        for bx in 0..src.cols / 3 {
            for oy in 0..2 {
                for ox in 0..2 {
                    let y = by * 3 + oy * 2;
                    let x = bx * 3 + ox * 2;
                    let leading = u32::from(src.data[y * src.cols + x]);
                    let quadratic = u32::from(src.data[y * src.cols + bx * 3 + 1]);
                    let linear = u32::from(src.data[(by * 3 + 1) * src.cols + x]);
                    let constant = u32::from(src.data[(by * 3 + 1) * src.cols + bx * 3 + 1]);
                    let value = if bx * 3 < blocks * 15 {
                        let outer = average(average(leading, linear), leading);
                        let middle = average(average(quadratic, constant), quadratic);
                        average(average(outer, middle), outer)
                    } else {
                        (4 * leading + 2 * (quadratic + linear + 1) + constant + 1) / 9
                    };
                    let at = (by * 2 + oy) * dst.cols + bx * 2 + ox;
                    #[cfg(any(target_arch = "aarch64", target_feature = "neon"))]
                    {
                        if bx * 3 < blocks * 15 && bx / 5 * 10 + 16 > dst.cols {
                            dst.data[at] |= value as u8;
                        } else {
                            dst.data[at] = value as u8;
                        }
                    }
                    #[cfg(not(any(target_arch = "aarch64", target_feature = "neon")))]
                    {
                        dst.data[at] = value as u8;
                    }
                }
            }
        }
    }
    Ok(())
}
/// # Errors
/// Returns an error if either image is malformed or destination dimensions do not match
/// the required reduction ratio.
pub fn twothirdsample_16(src: &Matrix<u16>, dst: &mut Matrix<u16>) -> Result<()> {
    target(src, dst, 2, 3)?;
    #[cfg(any(target_arch = "aarch64", target_feature = "neon"))]
    {
        return Err(invalid("Twothirdsample16 is not implemented for NEON"));
    }
    #[cfg(not(any(target_arch = "aarch64", target_feature = "neon")))]
    {
        if src.cols / 3 * 3 < 12 {
            return Ok(());
        }
        for by in 0..src.rows / 3 {
            for bx in 0..src.cols / 3 {
                for oy in 0..2 {
                    for ox in 0..2 {
                        let y = by * 3 + oy * 2;
                        let x = bx * 3 + ox * 2;
                        let leading = u32::from(src.data[y * src.cols + x]);
                        let quadratic = u32::from(src.data[y * src.cols + bx * 3 + 1]);
                        let linear = u32::from(src.data[(by * 3 + 1) * src.cols + x]);
                        let constant = u32::from(src.data[(by * 3 + 1) * src.cols + bx * 3 + 1]);
                        dst.data[(by * 2 + oy) * dst.cols + bx * 2 + ox] =
                            ((4 * leading + 2 * (quadratic + linear) + constant) / 9)
                                .min(i16::MAX as u32) as u16;
                    }
                }
            }
        }
        Ok(())
    }
}
