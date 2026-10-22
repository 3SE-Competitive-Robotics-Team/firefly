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

//! Linear DLT and midpoint triangulation in the first camera frame.
use nalgebra::{Matrix2, Matrix3, Matrix4, SMatrix, Vector2, Vector3};
/// Relative-adapter capabilities used by `OpenGV`'s central triangulation kernels.
pub trait RelativeAdapterBase {
    fn t12(&self) -> Vector3<f64>;
    fn r12(&self) -> Matrix3<f64>;
    fn bearing_vector1(&self, index: usize) -> Vector3<f64>;
    fn bearing_vector2(&self, index: usize) -> Vector3<f64>;
}
/// # Panics
/// Panics if the SVD does not supply the requested full V factor or the adapter index is invalid.
pub fn triangulate(adapter: &impl RelativeAdapterBase, index: usize) -> Vector3<f64> {
    let t = adapter.t12();
    let r = adapter.r12();
    let mut p1 = SMatrix::<f64, 3, 4>::zeros();
    p1.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&Matrix3::identity());
    let mut p2 = SMatrix::<f64, 3, 4>::zeros();
    p2.fixed_view_mut::<3, 3>(0, 0).copy_from(&r.transpose());
    p2.set_column(3, &(-r.transpose() * t));
    let f1 = adapter.bearing_vector1(index);
    let f2 = adapter.bearing_vector2(index);
    let mut a = Matrix4::zeros();
    a.set_row(0, &(f1[0] * p1.row(2) - f1[2] * p1.row(0)));
    a.set_row(1, &(f1[1] * p1.row(2) - f1[2] * p1.row(1)));
    a.set_row(2, &(f2[0] * p2.row(2) - f2[2] * p2.row(0)));
    a.set_row(3, &(f2[1] * p2.row(2) - f2[2] * p2.row(1)));
    let vt = a.svd(false, true).v_t.expect("full V requested");
    Vector3::new(vt[(3, 0)], vt[(3, 1)], vt[(3, 2)]) / vt[(3, 3)]
}
pub fn triangulate2(adapter: &impl RelativeAdapterBase, index: usize) -> Vector3<f64> {
    let t = adapter.t12();
    let r = adapter.r12();
    let f1 = adapter.bearing_vector1(index);
    let f2 = r * adapter.bearing_vector2(index);
    let b = Vector2::new(t.dot(&f1), t.dot(&f2));
    let a = Matrix2::new(f1.dot(&f1), -f1.dot(&f2), f1.dot(&f2), -f2.dot(&f2));
    let det = a.determinant();
    let inverse = Matrix2::new(a[(1, 1)], -a[(0, 1)], -a[(1, 0)], a[(0, 0)]) / det;
    let lambda = inverse * b;
    (lambda[0] * f1 + t + lambda[1] * f2) / 2.0
}
