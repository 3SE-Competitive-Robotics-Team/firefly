//! 已匹配 RGB-D 路标的米制刚体配准；没有最近邻搜索，尺度固定为一。
#![allow(clippy::many_single_char_names)]
use firefly_base::{FrameId, RigidTransform};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

fn fit(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    indices: &[usize],
) -> Option<(Matrix3<f64>, Vector3<f64>)> {
    if indices.len() < 3 {
        return None;
    }
    let n = indices.len() as f64;
    let a = indices.iter().map(|&i| source[i]).sum::<Vector3<f64>>() / n;
    let b = indices.iter().map(|&i| target[i]).sum::<Vector3<f64>>() / n;
    let h = indices
        .iter()
        .map(|&i| (source[i] - a) * (target[i] - b).transpose())
        .sum::<Matrix3<f64>>();
    if !h.iter().all(|x| x.is_finite()) {
        return None;
    }
    let svd = h.svd(true, true);
    if svd.singular_values[1] < 1e-6 * svd.singular_values[0].max(1e-12) {
        return None;
    }
    let u = svd.u?;
    let vt = svd.v_t?;
    let mut sign = Matrix3::identity();
    sign[(2, 2)] = (vt.transpose() * u.transpose()).determinant().signum();
    let r = vt.transpose() * sign * u.transpose();
    Some((r, b - r * a))
}
/// 三点 RANSAC + 共识 SVD；阈值为三维欧氏距离（米）。
/// 共线、低共识、非有限输入不产生约束；旋转的行列式必须为正。
#[must_use]
pub fn rigid_consensus(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    target_frame: FrameId,
    source_frame: FrameId,
    threshold: f64,
) -> Option<RigidTransform> {
    let n = source.len();
    if n < 30
        || n != target.len()
        || !threshold.is_finite()
        || threshold <= 0.
        || source
            .iter()
            .chain(target)
            .any(|v| !v.iter().all(|x| x.is_finite()))
    {
        return None;
    }
    let mut best = Vec::new();
    let mut state = 0x1234_5678_u64;
    for _ in 0..256 {
        let indices: [usize; 3] = std::array::from_fn(|_| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            (state >> 32) as usize % n
        });
        if indices[0] == indices[1] || indices[0] == indices[2] || indices[1] == indices[2] {
            continue;
        }
        let Some((r, t)) = fit(source, target, &indices) else {
            continue;
        };
        let inliers: Vec<_> = (0..n)
            .filter(|&i| (r * source[i] + t - target[i]).norm() <= threshold)
            .collect();
        if inliers.len() > best.len() {
            best = inliers;
        }
        if best.len() == n {
            break;
        }
    }
    if best.len() < 30 || (best.len() as f64) < 0.4 * n as f64 {
        return None;
    }
    let (r, t) = fit(source, target, &best)?;
    let q = UnitQuaternion::from_matrix(&r);
    let q = q.quaternion();
    RigidTransform::from_parts(target_frame, source_frame, t.into(), [q.i, q.j, q.k, q.w]).ok()
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn known_rigid_motion_outliers_and_translation_derivative() {
        let source: Vec<_> = (0..60)
            .map(|i| {
                Vector3::new(
                    f64::from(i % 10) * 0.1,
                    f64::from(i / 10) * 0.1,
                    f64::from(i % 3) * 0.2,
                )
            })
            .collect();
        let r = UnitQuaternion::from_euler_angles(0.1, -0.05, 0.2);
        let t = Vector3::new(0.3, -0.2, 0.1);
        let mut target: Vec<_> = source.iter().map(|p| r * p + t).collect();
        for p in target.iter_mut().take(15) {
            *p += Vector3::new(4., -3., 1.);
        }
        let estimate =
            rigid_consensus(&source, &target, FrameId(1024), FrameId::BODY, 0.02).unwrap();
        assert!((estimate.isometry().translation.vector - t).norm() < 1e-10);
        assert!(estimate.isometry().rotation.angle_to(&r) < 1e-10);
        let perturb = |step| {
            let shifted: Vec<_> = target
                .iter()
                .map(|p| p + Vector3::new(step, 0., 0.))
                .collect();
            rigid_consensus(&source, &shifted, FrameId(1024), FrameId::BODY, 0.02)
                .unwrap()
                .isometry()
                .translation
                .x
        };
        assert!(((perturb(1e-6) - perturb(-1e-6)) / 2e-6 - 1.).abs() < 1e-8);
        let line: Vec<_> = (0..30)
            .map(|i| Vector3::new(f64::from(i), 0., 0.))
            .collect();
        assert!(rigid_consensus(&line, &line, FrameId(1024), FrameId::BODY, 0.02).is_none());
    }
}
