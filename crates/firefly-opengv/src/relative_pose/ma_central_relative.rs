/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

use crate::relative_pose::relative_adapter_base::RelativeAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed storage adapter with world/viewpoint priors.
/// Access indices must satisfy the source assertions; constructors validate storage bounds.
pub struct MaCentralRelative<'a> {
    pub bearing_vectors1: &'a [f64],
    pub bearing_vectors2: &'a [f64],
    pub number_bearing_vectors1: usize,
    pub number_bearing_vectors2: usize,
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
}
impl<'a> MaCentralRelative<'a> {
    /// # Errors
    /// Returns an error if correspondence counts, coordinate storage, camera assignments,
    /// or camera calibration arrays have incompatible bounds.
    pub fn new(
        bearing_vectors1: &'a [f64],
        bearing_vectors2: &'a [f64],
        number_bearing_vectors1: usize,
        number_bearing_vectors2: usize,
    ) -> Result<Self, String> {
        if bearing_vectors1.len() / 3 < number_bearing_vectors1
            || bearing_vectors2.len() / 3 < number_bearing_vectors2
        {
            return Err("adapter storage shape violates correspondence bounds".into());
        }
        Ok(Self {
            bearing_vectors1,
            bearing_vectors2,
            number_bearing_vectors1,
            number_bearing_vectors2,
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
        })
    }
}
impl RelativeAdapterBase for MaCentralRelative<'_> {
    fn bearing_vector1(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_bearing_vectors1);
        Vector3::from_column_slice(&self.bearing_vectors1[index * 3..index * 3 + 3])
    }
    fn bearing_vector2(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_bearing_vectors2);
        Vector3::from_column_slice(&self.bearing_vectors2[index * 3..index * 3 + 3])
    }
    fn cam_offset1(&self, _: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn cam_rotation1(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn cam_offset2(&self, _: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn cam_rotation2(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn weight(&self, _: usize) -> f64 {
        1.0
    }
    fn number_correspondences(&self) -> usize {
        self.number_bearing_vectors2
    }
    fn t12(&self) -> Vector3<f64> {
        self.t12
    }
    fn set_t12(&mut self, value: Vector3<f64>) {
        self.t12 = value;
    }
    fn r12(&self) -> Matrix3<f64> {
        self.r12
    }
    fn set_r12(&mut self, value: Matrix3<f64>) {
        self.r12 = value;
    }
}
