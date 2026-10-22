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

//! Descriptor Hamming accumulation, retaining each architecture's lane arithmetic.
use crate::vectorized_filters::invalid;
use firefly_error::Result;
#[derive(Clone, Copy, Debug, Default)]
pub struct Hamming;
pub type ValueType = u8;
pub type ResultType = i32;
impl Hamming {
    /// The byte count truncates to whole 128-bit words before accumulation.
    /// # Errors
    /// Returns an error if the word count is invalid or either descriptor is too short.
    pub fn distance(&self, a: &[u8], b: &[u8], size: i32) -> Result<ResultType> {
        Self::popcnt_of_xored(a, b, size / 16).map(|v| v as ResultType)
    }
    /// # Errors
    /// Returns an error if the word count is invalid or either descriptor is too short.
    pub fn popcnt_of_xored(a: &[u8], b: &[u8], words: i32) -> Result<u32> {
        #[cfg(any(target_arch = "aarch64", target_feature = "neon"))]
        {
            Self::neon_popcnt_of_xored(a, b, words)
        }
        #[cfg(not(any(target_arch = "aarch64", target_feature = "neon")))]
        {
            Self::ssse3_popcnt_of_xored(a, b, words)
        }
    }
    /// # Errors
    /// Returns an error if the word count is invalid or either descriptor is too short.
    pub fn neon_popcnt_of_xored(a: &[u8], b: &[u8], words: i32) -> Result<u32> {
        let n = words.max(0) as usize * 16;
        if a.len() < n || b.len() < n {
            return Err(invalid(
                "descriptor storage is shorter than requested words",
            ));
        }
        Ok(a[..n]
            .iter()
            .zip(&b[..n])
            .fold(0u32, |sum, (&a, &b)| sum.wrapping_add((a ^ b).count_ones())))
    }
    /// SSSE3 adds byte-lane popcounts before horizontally summing the lanes.
    /// The do-while consumes one block even for a nonpositive word count.
    /// # Errors
    /// Returns an error if the word count is invalid or either descriptor is too short.
    pub fn ssse3_popcnt_of_xored(a: &[u8], b: &[u8], words: i32) -> Result<u32> {
        let n = words.max(1) as usize * 16;
        if a.len() < n || b.len() < n {
            return Err(invalid(
                "descriptor storage is shorter than requested words",
            ));
        }
        let mut lanes = [0u8; 16];
        for i in 0..n {
            lanes[i % 16] = lanes[i % 16].wrapping_add((a[i] ^ b[i]).count_ones() as u8);
        }
        Ok(lanes.into_iter().map(u32::from).sum())
    }
}
