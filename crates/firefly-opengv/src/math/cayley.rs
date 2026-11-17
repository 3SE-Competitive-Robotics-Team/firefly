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
