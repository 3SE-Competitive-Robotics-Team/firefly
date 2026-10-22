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

use crate::absolute_pose::{
    absolute_adapter_base::AbsoluteAdapterBase,
    absolute_multi_adapter_base::AbsoluteMultiAdapterBase,
};
use nalgebra::{Matrix3, Vector3};

pub struct NoncentralAbsoluteMultiAdapter<'a> {
    pub bearing_vectors: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub points: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub cam_offsets: &'a [Vector3<f64>],
    pub cam_rotations: &'a [Matrix3<f64>],
    pub t: Vector3<f64>,
    pub r: Matrix3<f64>,
    offsets: Vec<usize>,
    groups: Vec<usize>,
    correspondences: Vec<usize>,
}
impl<'a> NoncentralAbsoluteMultiAdapter<'a> {
    /// # Errors
    /// Returns an error if correspondence counts, coordinate storage, camera assignments,
    /// or camera calibration arrays have incompatible bounds.
    pub fn new(
        bearing_vectors: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
        points: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
        cam_offsets: &'a [Vector3<f64>],
        cam_rotations: &'a [Matrix3<f64>],
    ) -> Result<Self, String> {
        if bearing_vectors.len() != points.len()
            || bearing_vectors.len() != cam_offsets.len()
            || cam_offsets.len() != cam_rotations.len()
            || bearing_vectors
                .iter()
                .zip(&points)
                .any(|(b, p)| b.len() != p.len())
        {
            return Err("adapter storage shape violates correspondence bounds".into());
        }

        let mut offsets = Vec::new();
        let mut groups = Vec::new();
        let mut correspondences = Vec::new();
        let mut offset = 0;
        for (g, n) in bearing_vectors.iter().map(|v| v.len()).enumerate() {
            offsets.push(offset);
            for i in 0..n {
                groups.push(g);
                correspondences.push(i);
            }
            offset += n;
        }
        Ok(Self {
            bearing_vectors,
            points,
            cam_offsets,
            cam_rotations,
            t: Vector3::zeros(),
            r: Matrix3::identity(),
            offsets,
            groups,
            correspondences,
        })
    }
}
impl AbsoluteMultiAdapterBase for NoncentralAbsoluteMultiAdapter<'_> {
    fn bearing_vector_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.bearing_vectors[g][i]
    }
    fn point_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.points[g][i]
    }
    fn weight_at(&self, _: usize, _: usize) -> f64 {
        1.0
    }
    fn multi_cam_offset(&self, g: usize) -> Vector3<f64> {
        self.cam_offsets[g]
    }
    fn multi_cam_rotation(&self, g: usize) -> Matrix3<f64> {
        self.cam_rotations[g]
    }
    fn number_correspondences_at(&self, g: usize) -> usize {
        self.bearing_vectors[g].len()
    }
    fn number_frames(&self) -> usize {
        self.cam_offsets.len()
    }
    fn convert_multiindex(&self, g: usize, i: usize) -> usize {
        self.offsets[g] + i
    }
    fn multi_frameindex(&self, i: usize) -> usize {
        self.groups[i]
    }
    fn multicorrespondence_index(&self, i: usize) -> usize {
        self.correspondences[i]
    }
}
impl AbsoluteAdapterBase for NoncentralAbsoluteMultiAdapter<'_> {
    fn bearing_vector(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector_at(self.multi_frameindex(i), self.multicorrespondence_index(i))
    }
    fn point(&self, i: usize) -> Vector3<f64> {
        self.point_at(self.multi_frameindex(i), self.multicorrespondence_index(i))
    }
    fn weight(&self, i: usize) -> f64 {
        self.weight_at(self.multi_frameindex(i), self.multicorrespondence_index(i))
    }
    fn cam_offset(&self, i: usize) -> Vector3<f64> {
        self.multi_cam_offset(self.multi_frameindex(i))
    }
    fn cam_rotation(&self, i: usize) -> Matrix3<f64> {
        self.multi_cam_rotation(self.multi_frameindex(i))
    }
    fn number_correspondences(&self) -> usize {
        (0..self.number_frames())
            .map(|g| self.number_correspondences_at(g))
            .sum()
    }
    fn t(&self) -> Vector3<f64> {
        self.t
    }
    fn set_t(&mut self, v: Vector3<f64>) {
        self.t = v;
    }
    fn r(&self) -> Matrix3<f64> {
        self.r
    }
    fn set_r(&mut self, v: Matrix3<f64>) {
        self.r = v;
    }
}
