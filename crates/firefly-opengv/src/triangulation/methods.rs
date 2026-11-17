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
