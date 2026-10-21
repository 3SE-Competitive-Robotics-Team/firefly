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

//! Arun rigid alignment, mapping the second point cloud into the first.
use nalgebra::{Matrix3, SMatrix, Vector3};
#[must_use]
/// # Panics
/// Panics if the SVD does not supply the requested full U and V factors.
pub fn arun(h_cross: &Matrix3<f64>) -> Matrix3<f64> {
    let svd = h_cross.svd(true, true);
    let u = svd.u.expect("full U requested");
    let mut v = svd.v_t.expect("full V requested").transpose();
    let mut r = v * u.transpose();
    if r.determinant() < 0.0 {
        v.set_column(2, &(-v.column(2)));
        r = v * u.transpose();
    }
    r
}
#[must_use]
/// # Panics
/// Panics if the point-cloud lengths differ or the requested SVD factors are absent.
pub fn arun_complete(p1: &[Vector3<f64>], p2: &[Vector3<f64>]) -> SMatrix<f64, 3, 4> {
    assert_eq!(p1.len(), p2.len());
    let c1 = p1.iter().fold(Vector3::zeros(), |a, p| a + p) / p1.len() as f64;
    let c2 = p2.iter().fold(Vector3::zeros(), |a, p| a + p) / p2.len() as f64;
    let h = p1.iter().zip(p2).fold(Matrix3::zeros(), |h, (p, q)| {
        h + (q - c2) * (p - c1).transpose()
    });
    let r = arun(&h);
    let mut result = SMatrix::<f64, 3, 4>::zeros();
    result.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    result.set_column(3, &(c1 - r * c2));
    result
}
