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
