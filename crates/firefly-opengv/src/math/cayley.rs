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

//! Cayley rotation coordinates. The inverse is undefined for a 180 degree rotation.
use nalgebra::{Matrix3, Vector3};
#[must_use]
pub fn cayley2rot(c: &Vector3<f64>) -> Matrix3<f64> {
    let scale = 1.0 + c[0].powi(2) + c[1].powi(2) + c[2].powi(2);
    (1.0 / scale) * cayley2rot_reduced(c)
}
#[must_use]
pub fn cayley2rot_reduced(c: &Vector3<f64>) -> Matrix3<f64> {
    let (x, y, z) = (c[0], c[1], c[2]);
    Matrix3::new(
        1.0 + x * x - y * y - z * z,
        2.0 * (x * y - z),
        2.0 * (x * z + y),
        2.0 * (x * y + z),
        1.0 - x * x + y * y - z * z,
        2.0 * (y * z - x),
        2.0 * (x * z - y),
        2.0 * (y * z + x),
        1.0 - x * x - y * y + z * z,
    )
}
#[must_use]
pub fn rot2cayley(r: &Matrix3<f64>) -> Vector3<f64> {
    let c1 = r - Matrix3::identity();
    let c2 = r + Matrix3::identity();
    // Explicit adjugate preserves Eigen's non-finite result at a singular chart.
    let det = c2.determinant();
    let minor_indices = [[1, 2], [0, 2], [0, 1]];
    let adj = Matrix3::from_fn(|i, j| {
        let rows = minor_indices[j];
        let cols = minor_indices[i];
        let minor = c2[(rows[0], cols[0])] * c2[(rows[1], cols[1])]
            - c2[(rows[0], cols[1])] * c2[(rows[1], cols[0])];
        if (i + j) % 2 == 0 {
            minor / det
        } else {
            -minor / det
        }
    });
    let c = c1 * adj;
    Vector3::new(-c[(1, 2)], c[(0, 2)], -c[(0, 1)])
}
