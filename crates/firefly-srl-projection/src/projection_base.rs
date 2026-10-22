/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jan 28, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

//! Pixel bounds and the SRL camera mask convention used by projection.
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector2;
use purecv::core::Matrix;
use std::sync::Arc;
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProjectionStatus {
    Successful,
    OutsideImage,
    Masked,
    Behind,
    Invalid,
}
#[derive(Clone, Debug, Default)]
pub struct ProjectionBase {
    pub width: usize,
    pub height: usize,
    pub mask: Option<Arc<Matrix<u8>>>,
}
impl ProjectionBase {
    #[must_use]
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            mask: None,
        }
    }
    pub fn set_mask(&mut self, mask: Arc<Matrix<u8>>) -> bool {
        if mask.channels != 1
            || mask.rows.checked_mul(mask.cols) != Some(mask.data.len())
            || mask.rows != self.height
            || mask.cols != self.width
        {
            return false;
        }
        self.mask = Some(mask);
        true
    }
    pub fn remove_mask(&mut self) -> bool {
        self.mask = None;
        true
    }
    #[must_use]
    pub fn has_mask(&self) -> bool {
        self.mask.as_ref().is_some_and(|m| !m.data.is_empty())
    }
    #[must_use]
    pub fn is_in_image(&self, point: &Vector2<f32>) -> bool {
        !(point.x < -0.5f32
            || point.y < -0.5f32
            || point.x >= self.width as f32 - 0.5f32
            || point.y >= self.height as f32 - 0.5f32)
    }
    /// A nonzero mask byte is masked, matching the SRL implementation.
    ///
    /// # Errors
    /// Returns an error if a mask lookup receives a non-finite pixel or its
    /// truncated coordinates fall outside the mask image.
    pub fn is_masked(&self, point: &Vector2<f32>) -> Result<bool> {
        if !self.is_in_image(point) {
            return Ok(true);
        }
        let Some(mask) = self.mask.as_ref().filter(|m| !m.data.is_empty()) else {
            return Ok(false);
        };
        if !point.x.is_finite() || !point.y.is_finite() {
            return Err(invalid("non-finite mask pixel"));
        }
        let x = point.x as i32;
        let y = point.y as i32;
        if x < 0 || y < 0 || x as usize >= mask.cols || y as usize >= mask.rows {
            return Err(invalid("mask pixel outside image"));
        }
        Ok(mask.data[y as usize * mask.cols + x as usize] != 0)
    }
}

fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("srl_projection")
}
