//! 先验播种的鲁棒重投影精化；左扰动作用于 OpenCV camera←map。
use crate::{CameraIntrinsics, VisualPose, axis_flip, passes_prior_gate};
use firefly_base::se3::se3_exp;
use nalgebra::{
    Matrix2x3, Matrix2x6, Matrix3, Matrix4, Matrix6, Vector2, Vector3, Vector4, Vector6,
};

/// 像素残差对 camera 左扰动 `[rot, trans]` 的雅可比。
fn projection(c: Vector3<f64>, k: CameraIntrinsics) -> (Vector2<f64>, Matrix2x6<f64>) {
    let z = c.z;
    let p = Vector2::new(k.focal * c.x / z + k.cx, k.focal * c.y / z + k.cy);
    let j = Matrix2x3::new(
        k.focal / z,
        0.,
        -k.focal * c.x / (z * z),
        0.,
        k.focal / z,
        -k.focal * c.y / (z * z),
    );
    let minus_skew = Matrix3::new(0., c.z, -c.y, -c.z, 0., c.x, c.y, -c.x, 0.);
    let mut jac = Matrix2x6::zeros();
    jac.fixed_columns_mut::<3>(0).copy_from(&(j * minus_skew));
    jac.fixed_columns_mut::<3>(3).copy_from(&j);
    (p, jac)
}

fn equations(
    t: &Matrix4<f64>,
    p2: &[[f32; 2]],
    p3: &[[f64; 3]],
    k: CameraIntrinsics,
) -> (f64, Matrix6<f64>, Vector6<f64>) {
    let mut cost = 0.;
    let mut h = Matrix6::zeros();
    let mut g = Vector6::zeros();
    for (uv, p) in p2.iter().zip(p3) {
        let c = t * Vector4::new(p[0], p[1], p[2], 1.);
        if !c.iter().all(|x| x.is_finite()) || c.z <= 1e-6 {
            cost += 1e6;
            continue;
        }
        let (pixel, j) = projection(c.xyz(), k);
        let r = pixel - Vector2::new(f64::from(uv[0]), f64::from(uv[1]));
        let n = r.norm();
        // Huber 2px：亚像素对应为二次代价，大误匹配限制影响。
        let weight = if n <= 2. { 1. } else { 2. / n };
        cost += if n <= 2. { 0.5 * n * n } else { 2. * n - 2. };
        h += weight * j.transpose() * j;
        g += weight * j.transpose() * r;
    }
    (cost, h, g)
}

/// 先验只作为优化初值；接受要求实际像素支持与几何门，不能直接输出先验。
pub(crate) fn from_prior(
    p2: &[[f32; 2]],
    p3: &[[f64; 3]],
    k: CameraIntrinsics,
    prior: &Matrix4<f64>,
) -> Option<VisualPose> {
    let mut t = axis_flip() * prior.try_inverse()?;
    for _ in 0..30 {
        let (cost, h, g) = equations(&t, p2, p3, k);
        let delta = -(h + Matrix6::identity() * 1e-6).cholesky()?.solve(&g);
        if !delta.iter().all(|v| v.is_finite()) {
            return None;
        }
        if delta.norm() < 1e-9 {
            break;
        }
        let mut accepted = false;
        for step in 0..12 {
            let candidate = se3_exp(&(delta * 0.5_f64.powi(step))) * t;
            if equations(&candidate, p2, p3, k).0 < cost {
                t = candidate;
                accepted = true;
                break;
            }
        }
        if !accepted {
            break;
        }
    }
    let pose = t.try_inverse()? * axis_flip();
    if !passes_prior_gate(&pose, prior) {
        return None;
    }
    let errors: Vec<_> = p2
        .iter()
        .zip(p3)
        .filter_map(|(&uv, &p)| crate::reproj_error(uv, p, k, &t))
        .filter(|e| e.is_finite() && *e <= 8.)
        .collect();
    if errors.len() < 12 || errors.len() * 3 < p2.len() {
        return None;
    }
    let (_, h, _) = equations(&t, p2, p3, k);
    let spectrum = h.symmetric_eigen().eigenvalues;
    if spectrum.min() <= 1e-10 * spectrum.max() {
        return None;
    }
    Some(VisualPose {
        t_global: pose,
        num_inliers: errors.len(),
        total: p2.len(),
        mean_reproj_px: errors.iter().sum::<f64>() / errors.len() as f64,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn projection_jacobian_matches_independent_pinhole_and_differences() {
        let k = CameraIntrinsics {
            focal: 200.,
            cx: 160.,
            cy: 120.,
        };
        let c = Vector3::new(1., 2., 5.);
        let (uv, j) = projection(c, k);
        assert!((uv - Vector2::new(200., 200.)).norm() < 1e-12);
        for axis in 0..6 {
            let mut d = Vector6::zeros();
            d[axis] = 1e-6;
            let a = se3_exp(&d) * c.push(1.);
            let b = se3_exp(&-d) * c.push(1.);
            let numeric = (projection(a.xyz(), k).0 - projection(b.xyz(), k).0) / 2e-6;
            assert!((numeric - j.column(axis)).norm() < 1e-6);
        }
    }
    #[test]
    fn planar_map_offset_is_recovered_and_false_matches_rejected() {
        let k = CameraIntrinsics {
            focal: 200.,
            cx: 160.,
            cy: 120.,
        };
        let mut p2 = Vec::new();
        let mut p3 = Vec::new();
        for y in -3..=3 {
            for x in -4..=4 {
                p3.push([f64::from(x) * 0.2 - 13., f64::from(y) * 0.2 - 7., 0.]);
                p2.push([
                    (160. + f64::from(x) * 10.) as f32,
                    (120. + f64::from(y) * 10.) as f32,
                ]);
            }
        }
        let mut prior = axis_flip();
        prior[(0, 3)] = -12.9;
        prior[(1, 3)] = -7.;
        prior[(2, 3)] = -4.;
        let out = from_prior(&p2, &p3, k, &prior).unwrap();
        assert!((out.t_global[(0, 3)] + 13.).abs() < 1e-5);
        assert!((out.t_global[(1, 3)] + 7.).abs() < 1e-5);
        assert!((out.t_global[(2, 3)] + 4.).abs() < 1e-5);
        assert_eq!(out.num_inliers, p2.len());
        assert!(out.mean_reproj_px < 1e-5);
        p2.fill([20., 20.]);
        assert!(from_prior(&p2, &p3, k, &prior).is_none());
    }
}
