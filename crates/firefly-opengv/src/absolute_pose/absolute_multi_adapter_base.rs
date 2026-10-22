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

use super::absolute_adapter_base::AbsoluteAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Grouped correspondences with stable serialization in group-major order.
pub trait AbsoluteMultiAdapterBase: AbsoluteAdapterBase {
    fn bearing_vector_at(&self, group: usize, index: usize) -> Vector3<f64>;
    fn point_at(&self, group: usize, index: usize) -> Vector3<f64>;
    fn weight_at(&self, group: usize, index: usize) -> f64;
    fn multi_cam_offset(&self, group: usize) -> Vector3<f64>;
    fn multi_cam_rotation(&self, group: usize) -> Matrix3<f64>;
    fn number_correspondences_at(&self, group: usize) -> usize;
    fn number_frames(&self) -> usize;
    fn convert_multiindex(&self, group: usize, index: usize) -> usize;
    fn multi_frameindex(&self, index: usize) -> usize;
    fn multicorrespondence_index(&self, index: usize) -> usize;
    fn convert_multi_indices(&self, indices: &[Vec<usize>]) -> Vec<usize> {
        indices
            .iter()
            .enumerate()
            .flat_map(|(g, v)| v.iter().map(move |&i| self.convert_multiindex(g, i)))
            .collect()
    }
}
