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

//! AGAST-based BRISK detector with optional masking and scale recomputation.
use crate::{
    brisk_scale_space::BriskScaleSpace,
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use purecv::{core::Matrix, features2d::KeyPoint};
pub struct BriskFeatureDetector {
    pub threshold: i32,
    pub octaves: i32,
    pub suppress_scale_nonmaxima: bool,
}
impl BriskFeatureDetector {
    #[must_use]
    pub fn new(threshold: i32, octaves: i32, suppress_scale_nonmaxima: bool) -> Self {
        Self {
            threshold,
            octaves,
            suppress_scale_nonmaxima,
        }
    }
    /// # Errors
    /// Returns an error for malformed image/mask storage or invalid pyramid sampling support.
    pub fn detect(
        &self,
        image: &Matrix<u8>,
        points: &mut Vec<KeyPoint>,
        mask: Option<&Matrix<u8>>,
    ) -> Result<()> {
        check(image)?;
        points.clear();
        let mut space = BriskScaleSpace::new(self.octaves as u8, self.suppress_scale_nonmaxima);
        space.construct_pyramid(image, self.threshold as u8, 10)?;
        space.get_keypoints(points)?;
        if let Some(mask) = mask {
            check(mask)?;
            if !mask.data.is_empty() {
                let mut filtered = Vec::with_capacity(points.len());
                for p in points.iter() {
                    let (x, y) = ((p.pt.x + 0.5f32) as i32, (p.pt.y + 0.5f32) as i32);
                    if x < 0 || y < 0 || x >= mask.cols as i32 || y >= mask.rows as i32 {
                        return Err(invalid("keypoint outside match mask"));
                    }
                    if mask.data[y as usize * mask.cols + x as usize] != 0 {
                        filtered.push(p.clone());
                    }
                }
                *points = filtered;
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for malformed image/mask storage or invalid pyramid sampling support.
    pub fn compute_scale(&self, image: &Matrix<u8>, points: &mut Vec<KeyPoint>) -> Result<()> {
        let mut space = BriskScaleSpace::new(self.octaves as u8, self.suppress_scale_nonmaxima);
        space.construct_pyramid(image, self.threshold as u8, 0)?;
        space.get_keypoints(points)
    }
}
