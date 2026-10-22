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
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

//! SRL pinhole projection specialized to `NoDistortion` and f32.
//! Supereight defines `SRL_FLOAT_T` as float in se/common/projection.hpp.
use crate::{
    no_distortion::NoDistortion,
    projection_base::{ProjectionBase, ProjectionStatus},
};
use nalgebra::{SMatrix, Vector2, Vector3};
#[derive(Clone, Debug)]
pub struct ProjectionPinholeCamera {
    pub base: ProjectionBase,
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub one_over_fx: f32,
    pub one_over_fy: f32,
    pub fx_over_fy: f32,
    pub distortion: NoDistortion,
}
impl std::ops::Deref for ProjectionPinholeCamera {
    type Target = ProjectionBase;
    fn deref(&self) -> &Self::Target {
        &self.base
    }
}
impl std::ops::DerefMut for ProjectionPinholeCamera {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.base
    }
}
impl ProjectionPinholeCamera {
    #[must_use]
    pub fn new(width: usize, height: usize, fx: f32, fy: f32, cx: f32, cy: f32) -> Self {
        Self {
            base: ProjectionBase::new(width, height),
            fx,
            fy,
            cx,
            cy,
            one_over_fx: (1.0f64 / f64::from(fx)) as f32,
            one_over_fy: (1.0f64 / f64::from(fy)) as f32,
            fx_over_fy: fx / fy,
            distortion: NoDistortion,
        }
    }
    /// Preserve the output pixel when the source singularity check returns Invalid.
    pub fn project_into(&self, point: &Vector3<f32>, pixel: &mut Vector2<f32>) -> ProjectionStatus {
        if f64::from(point.z.abs()) < 1e-12 {
            return ProjectionStatus::Invalid;
        }
        let rz = (1.0f64 / f64::from(point.z)) as f32;
        let normalized = Vector2::new(point.x * rz, point.y * rz);
        let p = self.distortion.distort(&normalized);
        pixel.x = self.fx * p.x + self.cx;
        pixel.y = self.fy * p.y + self.cy;
        if !self.base.is_in_image(pixel) {
            return ProjectionStatus::OutsideImage;
        }
        match self.base.is_masked(pixel) {
            Ok(true) => return ProjectionStatus::Masked,
            Err(_) => return ProjectionStatus::Invalid,
            Ok(false) => {}
        }
        if point.z > 0.0 {
            ProjectionStatus::Successful
        } else {
            ProjectionStatus::Behind
        }
    }
    /// The returned pixel is NaN where the source leaves the output unassigned.
    #[must_use]
    pub fn project(&self, point: &Vector3<f32>) -> (ProjectionStatus, Vector2<f32>) {
        let mut pixel = Vector2::repeat(f32::NAN);
        let status = self.project_into(point, &mut pixel);
        (status, pixel)
    }
    #[must_use]
    pub fn project_batch(&self, points: &[Vector3<f32>]) -> Vec<(ProjectionStatus, Vector2<f32>)> {
        points.iter().map(|p| self.project(p)).collect()
    }
    #[must_use]
    pub fn back_project(&self, pixel: &Vector2<f32>) -> Vector3<f32> {
        let p = self.distortion.undistort(&Vector2::new(
            (pixel.x - self.cx) * self.one_over_fx,
            (pixel.y - self.cy) * self.one_over_fy,
        ));
        Vector3::new(p.x, p.y, 1.0)
    }
    #[must_use]
    pub fn back_project_batch(&self, pixels: &[Vector2<f32>]) -> Vec<Vector3<f32>> {
        pixels.iter().map(|p| self.back_project(p)).collect()
    }
    #[must_use]
    pub fn back_project_with_jacobian(
        &self,
        pixel: &Vector2<f32>,
    ) -> (Vector3<f32>, SMatrix<f32, 3, 2>) {
        (
            self.back_project(pixel),
            SMatrix::<f32, 3, 2>::new(self.one_over_fx, 0.0, 0.0, self.one_over_fy, 0.0, 0.0),
        )
    }
}
