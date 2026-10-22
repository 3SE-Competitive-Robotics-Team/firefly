//
//    AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 8 pixel mask
//
//    Copyright (C) 2010  Elmar Mair
//    All rights reserved.
//
//    Redistribution and use in source and binary forms, with or without
//    modification, are permitted provided that the following conditions are met:
//        * Redistributions of source code must retain the above copyright
//          notice, this list of conditions and the following disclaimer.
//        * Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the
//          documentation and/or other materials provided with the distribution.
//        * Neither the name of the <organization> nor the
//          names of its contributors may be used to endorse or promote products
//          derived from this software without specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//    DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! Shared AGAST state, scoring and adaptive detector interface.
use crate::vectorized_filters::invalid;
use firefly_error::Result;
use purecv::core::Matrix;
use purecv::features2d::KeyPoint;
#[derive(Clone, Copy, Debug)]
pub(crate) enum AdaptiveState {
    Homogeneous,
    Structured,
}
#[derive(Clone, Debug)]
pub struct AstDetectorState {
    pub width: usize,
    pub height: usize,
    pub threshold: i32,
    pub upper_threshold: Option<i32>,
    pub lower_threshold: Option<i32>,
    pub comparison_threshold: Option<i32>,
    pub scores: Vec<i32>,
    pub nms_flags: Vec<i32>,
}
impl Default for AstDetectorState {
    fn default() -> Self {
        Self::new(0, 0, -1)
    }
}
impl AstDetectorState {
    #[must_use]
    pub fn new(width: usize, height: usize, threshold: i32) -> Self {
        Self {
            width,
            height,
            threshold,
            upper_threshold: None,
            lower_threshold: None,
            comparison_threshold: None,
            scores: Vec::new(),
            nms_flags: Vec::new(),
        }
    }
    pub fn set_threshold(&mut self, threshold: i32, upper: i32, lower: i32) {
        self.threshold = threshold;
        self.upper_threshold = Some(upper);
        self.lower_threshold = Some(lower);
        self.comparison_threshold = Some(threshold.wrapping_mul(lower) / 100);
    }
    pub(crate) fn threshold_bounds(&self) -> Result<(i32, i32, i32)> {
        match (
            self.upper_threshold,
            self.lower_threshold,
            self.comparison_threshold,
        ) {
            (Some(u), Some(l), Some(c)) => Ok((u, l, c)),
            _ => Err(invalid("threshold-map bounds have not been set")),
        }
    }
}
pub(crate) fn checked_image(state: &AstDetectorState, image: &[u8]) -> Result<()> {
    if state.width > i32::MAX as usize
        || state.height > i32::MAX as usize
        || state.width.checked_mul(state.height) != Some(image.len())
    {
        return Err(invalid("AGAST image shape does not match storage"));
    }
    Ok(())
}
pub(crate) fn keypoint(x: usize, y: usize) -> KeyPoint {
    let mut p = KeyPoint::default();
    p.pt.x = x as f32;
    p.pt.y = y as f32;
    p
}
pub(crate) fn neighborhood<const N: usize>(
    image: &[u8],
    center: usize,
    width: usize,
    pattern: &[(isize, isize); N],
) -> Result<[i32; N]> {
    if center >= image.len() {
        return Err(invalid("AGAST center outside image"));
    }
    let mut values = [0; N];
    for (i, &(x, y)) in pattern.iter().enumerate() {
        let at = center as i128 + x as i128 + y as i128 * width as i128;
        if at < 0 || at >= image.len() as i128 {
            return Err(invalid("AGAST pattern outside image"));
        }
        values[i] = i32::from(image[at as usize]);
    }
    Ok(values)
}
pub trait AstDetector {
    fn state(&self) -> &AstDetectorState;
    fn state_mut(&mut self) -> &mut AstDetectorState;
    /// # Errors
    /// Returns an error if image dimensions/storage or threshold-map geometry is invalid.
    fn detect(
        &mut self,
        image: &[u8],
        corners: &mut Vec<KeyPoint>,
        threshold_map: Option<&Matrix<u8>>,
    ) -> Result<()>;
    fn border_width(&self) -> usize;
    /// # Errors
    /// Returns an error for invalid image storage, a corner/support outside the image,
    /// or a score threshold outside [-1, 255].
    fn corner_score(&self, image: &[u8], index: usize) -> Result<i32>;
    /// # Errors
    /// Returns an error if image storage or any corner location/support is invalid.
    fn score(&mut self, image: &[u8], corners: &[KeyPoint]) -> Result<()> {
        checked_image(self.state(), image)?;
        let mut values = Vec::with_capacity(corners.len());
        for p in corners {
            if !p.pt.x.is_finite() || !p.pt.y.is_finite() || p.pt.x < 0.0 || p.pt.y < 0.0 {
                return Err(invalid("invalid AGAST keypoint"));
            }
            let x = p.pt.x as usize;
            let y = p.pt.y as usize;
            if x >= self.state().width || y >= self.state().height {
                return Err(invalid("AGAST keypoint outside image"));
            }
            values.push(self.corner_score(image, y * self.state().width + x)?);
        }
        self.state_mut().scores = values;
        Ok(())
    }
    /// # Errors
    /// Returns an error if corner scoring fails or score and corner data are incompatible.
    fn nms(
        &mut self,
        image: &[u8],
        corners: &[KeyPoint],
        output: &mut Vec<KeyPoint>,
    ) -> Result<()> {
        self.score(image, corners)?;
        let state = self.state_mut();
        crate::agast::non_maximum_suppression::non_maximum_suppression(
            &state.scores,
            corners,
            output,
            &mut state.nms_flags,
        )
    }
    /// # Errors
    /// Propagates errors from detection, scoring, or non-maximum suppression.
    fn process_image(&mut self, image: &[u8], output: &mut Vec<KeyPoint>) -> Result<()> {
        let mut points = Vec::new();
        self.detect(image, &mut points, None)?;
        self.nms(image, &points, output)
    }
}
