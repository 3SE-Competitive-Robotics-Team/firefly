use crate::{
    math::{
        arun::arun,
        cayley::{cayley2rot, rot2cayley},
    },
    optimization_functor::{LmParameters, NumericalBackend},
    point_cloud::point_cloud_adapter_base::PointCloudAdapterBase,
    types::Transformation,
};
use nalgebra::{DVector, Matrix3, Vector3};
/// # Panics
/// Panics if fewer than three indices are provided, or if an adapter index is invalid.
pub fn threept_arun(a: &impl PointCloudAdapterBase, idx: &[usize]) -> Transformation {
    assert!(idx.len() > 2);
    let (mut c1, mut c2) = (Vector3::zeros(), Vector3::zeros());
    for &i in idx {
        c1 += a.point1(i);
        c2 += a.point2(i);
    }
    c1 /= idx.len() as f64;
    c2 /= idx.len() as f64;
    let mut h = Matrix3::zeros();
    for &i in idx {
        h += (a.point2(i) - c2) * (a.point1(i) - c1).transpose();
    }
    let r = arun(&h);
    let t = c1 - r * c2;
    let mut out = Transformation::zeros();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    out.set_column(3, &t);
    out
}
pub fn optimize_nonlinear(
    leading: &impl PointCloudAdapterBase,
    idx: &[usize],
    backend: &dyn NumericalBackend,
) -> Option<Transformation> {
    let mut x = DVector::zeros(6);
    x.fixed_rows_mut::<3>(0).copy_from(&leading.t12());
    x.fixed_rows_mut::<3>(3)
        .copy_from(&rot2cayley(&leading.r12()));
    let mut residual = |x: &DVector<f64>, out: &mut DVector<f64>| {
        if x.len() != 6 || out.len() != idx.len() {
            return None;
        }
        let rotation = cayley2rot(&x.fixed_rows::<3>(3).into_owned());
        let translation = x.fixed_rows::<3>(0);
        for (i, &root_product) in idx.iter().enumerate() {
            let mut transformation = Transformation::zeros();
            transformation
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&rotation);
            transformation.column_mut(3).copy_from(&translation);
            let point = leading.point2(root_product);
            let p_hom = nalgebra::Vector4::new(point[0], point[1], point[2], 1.0);
            let point_value = transformation * p_hom;
            out[i] = (leading.point1(root_product) - point_value).norm();
        }
        Some(())
    };
    backend.levenberg_marquardt(
        &mut x,
        idx.len(),
        &mut residual,
        LmParameters {
            ftol: 1E10 * f64::EPSILON,
            xtol: 1E10 * f64::EPSILON,
            maxfev: 1000,
            ..Default::default()
        },
    )?;
    let mut out = Transformation::zeros();
    out.set_column(3, &x.fixed_rows::<3>(0));
    out.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&cayley2rot(&x.fixed_rows::<3>(3).into_owned()));
    Some(out)
}
