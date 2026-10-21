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

use crate::absolute_pose::absolute_adapter_base::AbsoluteAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed storage adapter with world/viewpoint priors.
/// Access indices must satisfy the source assertions; constructors validate storage bounds.
pub struct MaNoncentralAbsolute<'a> {
    pub points: &'a [f64],
    pub bearing_vectors: &'a [f64],
    pub number_points: usize,
    pub number_bearing_vectors: usize,
    pub t: Vector3<f64>,
    pub r: Matrix3<f64>,
}
impl<'a> MaNoncentralAbsolute<'a> {
    /// # Errors
    /// Returns an error if correspondence counts, coordinate storage, camera assignments,
    /// or camera calibration arrays have incompatible bounds.
    pub fn new(
        points: &'a [f64],
        bearing_vectors: &'a [f64],
        number_points: usize,
        number_bearing_vectors: usize,
    ) -> Result<Self, String> {
        if points.len() / 3 < number_points || bearing_vectors.len() / 6 < number_bearing_vectors {
            return Err("adapter storage shape violates correspondence bounds".into());
        }
        Ok(Self {
            points,
            bearing_vectors,
            number_points,
            number_bearing_vectors,
            t: Vector3::zeros(),
            r: Matrix3::identity(),
        })
    }
}
impl AbsoluteAdapterBase for MaNoncentralAbsolute<'_> {
    fn bearing_vector(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_bearing_vectors);
        Vector3::from_column_slice(&self.bearing_vectors[index * 6..index * 6 + 3])
    }
    fn point(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_points);
        Vector3::from_column_slice(&self.points[index * 3..index * 3 + 3])
    }
    fn cam_offset(&self, index: usize) -> Vector3<f64> {
        {
            assert!(index < self.number_bearing_vectors);
            Vector3::from_column_slice(&self.bearing_vectors[index * 6 + 3..index * 6 + 6])
        }
    }
    fn cam_rotation(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn weight(&self, _: usize) -> f64 {
        1.0
    }
    fn number_correspondences(&self) -> usize {
        self.number_bearing_vectors
    }
    fn t(&self) -> Vector3<f64> {
        self.t
    }
    fn set_t(&mut self, value: Vector3<f64>) {
        self.t = value;
    }
    fn r(&self) -> Matrix3<f64> {
        self.r
    }
    fn set_r(&mut self, value: Matrix3<f64>) {
        self.r = value;
    }
}
