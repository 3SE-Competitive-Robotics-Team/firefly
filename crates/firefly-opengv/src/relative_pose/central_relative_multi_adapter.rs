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

use crate::relative_pose::{
    relative_adapter_base::RelativeAdapterBase,
    relative_multi_adapter_base::RelativeMultiAdapterBase,
};
use nalgebra::{Matrix3, Vector3};

pub struct CentralRelativeMultiAdapter {
    pub bearing_vectors1: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub bearing_vectors2: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
    offsets: Vec<usize>,
    groups: Vec<usize>,
    correspondences: Vec<usize>,
}
impl CentralRelativeMultiAdapter {
    /// # Errors
    /// Returns an error if correspondence counts, coordinate storage, camera assignments,
    /// or camera calibration arrays have incompatible bounds.
    pub fn new(
        bearing_vectors1: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
        bearing_vectors2: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    ) -> Result<Self, String> {
        if bearing_vectors1.len() != bearing_vectors2.len()
            || bearing_vectors1
                .iter()
                .zip(&bearing_vectors2)
                .any(|(a, b)| a.len() != b.len())
        {
            return Err("adapter storage shape violates correspondence bounds".into());
        }

        let mut offsets = Vec::new();
        let mut groups = Vec::new();
        let mut correspondences = Vec::new();
        let mut offset = 0;
        for (g, n) in bearing_vectors2.iter().map(|v| v.len()).enumerate() {
            offsets.push(offset);
            for i in 0..n {
                groups.push(g);
                correspondences.push(i);
            }
            offset += n;
        }
        Ok(Self {
            bearing_vectors1,
            bearing_vectors2,
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
            offsets,
            groups,
            correspondences,
        })
    }
}
impl RelativeMultiAdapterBase for CentralRelativeMultiAdapter {
    fn bearing_vector1_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.bearing_vectors1[g][i]
    }
    fn bearing_vector2_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.bearing_vectors2[g][i]
    }
    fn weight_at(&self, _: usize, _: usize) -> f64 {
        1.0
    }
    fn cam_offset(&self, _: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn cam_rotation(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn number_correspondences_at(&self, g: usize) -> usize {
        self.bearing_vectors2[g].len()
    }
    fn number_pairs(&self) -> usize {
        self.bearing_vectors2.len()
    }
    fn convert_multiindex(&self, g: usize, i: usize) -> usize {
        self.offsets[g] + i
    }
    fn multi_pairindex(&self, i: usize) -> usize {
        self.groups[i]
    }
    fn multicorrespondence_index(&self, i: usize) -> usize {
        self.correspondences[i]
    }
}
impl RelativeAdapterBase for CentralRelativeMultiAdapter {
    fn bearing_vector1(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector1_at(self.multi_pairindex(i), self.multicorrespondence_index(i))
    }
    fn bearing_vector2(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector2_at(self.multi_pairindex(i), self.multicorrespondence_index(i))
    }
    fn weight(&self, i: usize) -> f64 {
        self.weight_at(self.multi_pairindex(i), self.multicorrespondence_index(i))
    }
    fn cam_offset1(&self, i: usize) -> Vector3<f64> {
        self.cam_offset(self.multi_pairindex(i))
    }
    fn cam_rotation1(&self, i: usize) -> Matrix3<f64> {
        self.cam_rotation(self.multi_pairindex(i))
    }
    fn cam_offset2(&self, i: usize) -> Vector3<f64> {
        self.cam_offset(self.multi_pairindex(i))
    }
    fn cam_rotation2(&self, i: usize) -> Matrix3<f64> {
        self.cam_rotation(self.multi_pairindex(i))
    }
    fn number_correspondences(&self) -> usize {
        (0..self.number_pairs())
            .map(|g| self.number_correspondences_at(g))
            .sum()
    }
    fn t12(&self) -> Vector3<f64> {
        self.t12
    }
    fn set_t12(&mut self, v: Vector3<f64>) {
        self.t12 = v;
    }
    fn r12(&self) -> Matrix3<f64> {
        self.r12
    }
    fn set_r12(&mut self, v: Matrix3<f64>) {
        self.r12 = v;
    }
}
