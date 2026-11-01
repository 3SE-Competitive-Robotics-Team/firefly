// Copyright (C) 2000, Intel Corporation, all rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
// Full redistribution conditions and disclaimer: ../../utils/apriltag_subpixel.rs.
//! Four-point square specialization of `OpenCV` 4.10.0 `calibration.cpp`
//! (`cvFindExtrinsicCameraParams2`, `cvRodrigues2`) and `compat_ptsetreg.cpp`
//! (`CvLevMarq::update/step`). The square lies in z=0 with zero mean, so the
//! original planar coordinate transform is identity.
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Matrix3, Matrix4, SMatrix, SVector, Vector3};
use purecv::{
    calib3d::homography::{HomographyMethod, find_homography},
    core::types::Point2f,
};

fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0., -v.z, v.y, v.z, 0., -v.x, -v.y, v.x, 0.)
}
fn rodrigues(vector: Vector3<f64>) -> (Matrix3<f64>, [Matrix3<f64>; 3]) {
    let theta = vector.norm();
    let basis = [Vector3::x(), Vector3::y(), Vector3::z()];
    if theta < f64::EPSILON {
        return (Matrix3::identity(), basis.map(skew));
    }
    let r = vector / theta;
    let c = theta.cos();
    let s = theta.sin();
    let complement = 1. - c;
    let outer = r * r.transpose();
    let cross = skew(r);
    let rotation = c * Matrix3::identity() + complement * outer + s * cross;
    let jacobian = std::array::from_fn(|i| {
        let douter = basis[i] * r.transpose() + r * basis[i].transpose();
        (-s * r[i]) * Matrix3::identity()
            + ((s - 2. * complement / theta) * r[i]) * outer
            + (complement / theta) * douter
            + ((c - s / theta) * r[i]) * cross
            + (s / theta) * skew(basis[i])
    });
    (rotation, jacobian)
}
#[expect(
    clippy::manual_midpoint,
    reason = "Preserve original bounded rotation-entry arithmetic"
)]
fn rotation_vector(matrix: Matrix3<f64>) -> Vector3<f64> {
    if matrix
        .iter()
        .any(|x| !x.is_finite() || *x < -100. || *x >= 100.)
    {
        return Vector3::zeros();
    }
    let decomposition = matrix.svd(true, true);
    let rotation = decomposition.u.unwrap() * decomposition.v_t.unwrap();
    let mut r = Vector3::new(
        rotation[(2, 1)] - rotation[(1, 2)],
        rotation[(0, 2)] - rotation[(2, 0)],
        rotation[(1, 0)] - rotation[(0, 1)],
    );
    let sine = (r.norm_squared() * 0.25).sqrt();
    let cosine = ((rotation.trace() - 1.) * 0.5).clamp(-1., 1.);
    let theta = cosine.acos();
    if sine < 1e-5 {
        if cosine > 0. {
            return Vector3::zeros();
        }
        r.x = ((rotation[(0, 0)] + 1.) * 0.5).max(0.).sqrt();
        r.y = ((rotation[(1, 1)] + 1.) * 0.5).max(0.).sqrt()
            * if rotation[(0, 1)] < 0. { -1. } else { 1. };
        r.z = ((rotation[(2, 2)] + 1.) * 0.5).max(0.).sqrt()
            * if rotation[(0, 2)] < 0. { -1. } else { 1. };
        if r.x.abs() < r.y.abs()
            && r.x.abs() < r.z.abs()
            && (rotation[(1, 2)] > 0.) != (r.y * r.z > 0.)
        {
            r.z = -r.z;
        }
        r *= theta / r.norm();
    } else {
        r *= theta / (2. * sine);
    }
    r
}

fn residual(
    param: &SVector<f64, 6>,
    object: &[Vector3<f64>; 4],
    image: &[(f32, f32); 4],
    intrinsics: [f64; 4],
) -> (SVector<f64, 8>, SMatrix<f64, 8, 6>) {
    let (rotation, derivatives) = rodrigues(param.fixed_rows::<3>(0).into_owned());
    let translation = param.fixed_rows::<3>(3);
    let [fx, fy, cx, cy] = intrinsics;
    let mut error = SVector::zeros();
    let mut jacobian = SMatrix::zeros();
    for i in 0..4 {
        let p = rotation * object[i] + translation;
        let inverse_z = if p.z == 0. { 1. } else { 1. / p.z };
        let x = p.x * inverse_z;
        let y = p.y * inverse_z;
        error[2 * i] = fx * x + cx - f64::from(image[i].0);
        error[2 * i + 1] = fy * y + cy - f64::from(image[i].1);
        for j in 0..3 {
            let derivative = derivatives[j] * object[i];
            jacobian[(2 * i, j)] = fx * inverse_z * (derivative.x - x * derivative.z);
            jacobian[(2 * i + 1, j)] = fy * inverse_z * (derivative.y - y * derivative.z);
        }
        jacobian[(2 * i, 3)] = fx * inverse_z;
        jacobian[(2 * i, 5)] = -fx * x * inverse_z;
        jacobian[(2 * i + 1, 4)] = fy * inverse_z;
        jacobian[(2 * i + 1, 5)] = -fy * y * inverse_z;
    }
    (error, jacobian)
}

pub(super) fn solve(
    points: &[(f32, f32); 4],
    tag_size: f64,
    intrinsics: [f64; 4],
) -> Result<Matrix4<f64>> {
    let [fx, fy, cx, cy] = intrinsics.map(|x| f64::from(x as f32));
    let half = f64::from((tag_size / 2.) as f32);
    let object = [
        Vector3::new(-half, -half, 0.),
        Vector3::new(half, -half, 0.),
        Vector3::new(half, half, 0.),
        Vector3::new(-half, half, 0.),
    ];
    let source: Vec<_> = object
        .iter()
        .map(|p| Point2f::new(p.x as f32, p.y as f32))
        .collect();
    let target: Vec<_> = points
        .iter()
        .map(|p| {
            Point2f::new(
                ((f64::from(p.0) - cx) / fx) as f32,
                ((f64::from(p.1) - cy) / fy) as f32,
            )
        })
        .collect();
    let homography = find_homography(&source, &target, HomographyMethod::None, 3., None)
        .map_err(|e| Error::new(ErrorKind::InvalidArgument, e.to_string()))?;
    let mut h = Matrix3::from_fn(|r, c| *homography.get(r, c, 0).expect("3x3 homography"));
    let norm1 = h.column(0).norm();
    let norm2 = h.column(1).norm();
    let translation = h.column(2) * (2. / (norm1 + norm2).max(f64::EPSILON));
    h.column_mut(0).scale_mut(1. / norm1.max(f64::EPSILON));
    h.column_mut(1).scale_mut(1. / norm2.max(f64::EPSILON));
    let cross = h.column(0).cross(&h.column(1));
    h.set_column(2, &cross);
    let mut param = SVector::<f64, 6>::zeros();
    param.fixed_rows_mut::<3>(0).copy_from(&rotation_vector(h));
    param.fixed_rows_mut::<3>(3).copy_from(&translation);
    let intrinsics = [fx, fy, cx, cy];
    let mut lambda_log = -3_i32;
    for _ in 0..20 {
        let previous = param;
        let (error, jacobian) = residual(&param, &object, points, intrinsics);
        let normal = jacobian.transpose() * jacobian;
        let gradient = jacobian.transpose() * error;
        loop {
            let mut damped = normal;
            let lambda = (f64::from(lambda_log) * 10_f64.ln()).exp();
            for i in 0..6 {
                damped[(i, i)] *= 1. + lambda;
            }
            let svd = damped.svd(true, true);
            let threshold = 2. * f64::EPSILON * svd.singular_values.sum();
            let step = svd
                .solve(&gradient, threshold)
                .map_err(|e| Error::new(ErrorKind::InvalidArgument, e))?;
            param = previous - step;
            let candidate = residual(&param, &object, points, intrinsics).0.norm();
            if candidate > error.norm() {
                lambda_log += 1;
                if lambda_log <= 16 {
                    continue;
                }
            }
            break;
        }
        lambda_log = (lambda_log - 1).max(-16);
        if (param - previous).norm() / (previous.norm() + f64::EPSILON) < f64::from(f32::EPSILON) {
            break;
        }
    }
    let mut output = Matrix4::identity();
    output
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&rodrigues(param.fixed_rows::<3>(0).into_owned()).0);
    output
        .fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&param.fixed_rows::<3>(3));
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rodrigues_and_projection_jacobians() {
        let object = [
            Vector3::new(-0.08, -0.08, 0.),
            Vector3::new(0.08, -0.08, 0.),
            Vector3::new(0.08, 0.08, 0.),
            Vector3::new(-0.08, 0.08, 0.),
        ];
        for rotation in [
            Vector3::zeros(),
            Vector3::new(0.2, -0.3, 0.1),
            Vector3::new(3.1, 0.2, 0.1),
        ] {
            let mut param = SVector::<f64, 6>::zeros();
            param.fixed_rows_mut::<3>(0).copy_from(&rotation);
            param
                .fixed_rows_mut::<3>(3)
                .copy_from(&Vector3::new(0.1, -0.02, 1.2));
            let (_, jacobian) = residual(&param, &object, &[(0., 0.); 4], [350., 370., 120., 120.]);
            for column in 0..6 {
                let mut plus = param;
                let mut minus = param;
                plus[column] += 1e-6;
                minus[column] -= 1e-6;
                let numerical =
                    (residual(&plus, &object, &[(0., 0.); 4], [350., 370., 120., 120.]).0
                        - residual(&minus, &object, &[(0., 0.); 4], [350., 370., 120., 120.]).0)
                        / (2e-6);
                assert!((numerical - jacobian.column(column)).amax() < 2e-7);
            }
        }
    }
}
