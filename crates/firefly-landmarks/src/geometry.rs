//! 固定尺度路标几何；相机使用 `OpenCV` 右、下、前轴。
#![allow(clippy::many_single_char_names)]
use firefly_base::{FrameId, RigidTransform};
use nalgebra::{Matrix3, SMatrix, Vector2, Vector3};

pub const CAMERA: FrameId = FrameId(2048);
pub const HISTORIC_BODY: FrameId = FrameId(2049);

#[derive(Clone, Debug)]
pub struct Camera {
    pub camera_from_body: RigidTransform,
    pub intrinsics: [f64; 4],
}
impl Camera {
    #[must_use]
    pub fn project(&self, body: Vector3<f64>) -> Option<Vector2<f64>> {
        let p = self.camera_from_body.point(body.into()).coords;
        (p.iter().all(|v| v.is_finite()) && p.z > 0.1).then(|| Vector2::new(p.x / p.z, p.y / p.z))
    }
    #[must_use]
    pub fn residual(&self, body: Vector3<f64>, observed: [f64; 2]) -> Option<Vector2<f64>> {
        let p = self.project(body)?;
        Some(Vector2::new(
            (p.x - observed[0]) * self.intrinsics[0],
            (p.y - observed[1]) * self.intrinsics[1],
        ))
    }
    #[must_use]
    pub fn jacobian(&self, body: Vector3<f64>) -> Option<SMatrix<f64, 2, 3>> {
        let p = self.camera_from_body.point(body.into()).coords;
        if p.z <= 0.1 {
            return None;
        }
        let [fx, fy, _, _] = self.intrinsics;
        Some(
            SMatrix::<f64, 2, 3>::new(
                fx / p.z,
                0.,
                -fx * p.x / (p.z * p.z),
                0.,
                fy / p.z,
                -fy * p.y / (p.z * p.z),
            ) * self
                .camera_from_body
                .isometry()
                .rotation
                .to_rotation_matrix()
                .matrix(),
        )
    }
}

/// 两视图以上的射线交会；射线正交距离的最小二乘解经重投影精化与条件数检查。
#[must_use]
pub fn triangulate(observations: &[(&RigidTransform, &Camera, [f64; 2])]) -> Option<Vector3<f64>> {
    let mut h = Matrix3::zeros();
    let mut g = Vector3::zeros();
    for (pose, camera, uv) in observations {
        let camera_pose = pose.compose(&camera.camera_from_body.inverse()).ok()?;
        let ray = camera_pose.vector(Vector3::new(uv[0], uv[1], 1.).normalize());
        let normal = Matrix3::identity() - ray * ray.transpose();
        h += normal;
        g += normal * camera_pose.isometry().translation.vector;
    }
    let values = h.symmetric_eigen().eigenvalues;
    if observations.len() < 2 || values.min() <= 0. || values.max() / values.min() > 100_000. {
        return None;
    }
    refine_point(h.cholesky()?.solve(&g), observations)
}

/// 固定位姿下的多视图路标精化；每条二维观测只计一次，不重复加入 VIO 点先验。
#[must_use]
pub fn refine_point(
    seed: Vector3<f64>,
    observations: &[(&RigidTransform, &Camera, [f64; 2])],
) -> Option<Vector3<f64>> {
    if observations.len() < 2 {
        return None;
    }
    let cost = |p: Vector3<f64>| -> Option<f64> {
        observations
            .iter()
            .map(|(pose, cam, uv)| {
                cam.residual(pose.inverse().point(p.into()).coords, *uv)
                    .map(|r| {
                        let n = r.norm();
                        if n <= 3. { n * n } else { 6. * n - 9. }
                    })
            })
            .sum()
    };
    let mut point = seed;
    for _ in 0..5 {
        let mut h = Matrix3::zeros();
        let mut g = Vector3::zeros();
        for (pose, cam, uv) in observations {
            let body = pose.inverse().point(point.into()).coords;
            let r = cam.residual(body, *uv)?;
            let j = cam.jacobian(body)?
                * pose
                    .isometry()
                    .rotation
                    .inverse()
                    .to_rotation_matrix()
                    .matrix();
            let w = if r.norm() > 3. { 3. / r.norm() } else { 1. };
            h += w * j.transpose() * j;
            g += w * j.transpose() * r;
        }
        let eigen = h.symmetric_eigen().eigenvalues;
        if eigen.min() <= 0. || eigen.max() / eigen.min() > 100_000. {
            return None;
        }
        let step = h.cholesky()?.solve(&g);
        let before = cost(point)?;
        let mut accepted = false;
        for scale in [1., 0.5, 0.25, 0.125] {
            let trial = point - scale * step;
            if cost(trial).is_some_and(|v| v <= before) {
                point = trial;
                accepted = true;
                break;
            }
        }
        if !accepted || step.norm() < 1e-6 {
            break;
        }
    }
    let good = observations
        .iter()
        .filter(|(pose, cam, uv)| {
            cam.residual(pose.inverse().point(point.into()).coords, *uv)
                .is_some_and(|r| r.norm() <= 3.)
        })
        .count();
    (good >= 2 && good * 5 >= observations.len() * 4).then_some(point)
}

/// 右扰动机体位姿的像素雅可比，列顺序为平移、旋转。
#[must_use]
pub fn pose_jacobian(camera: &Camera, body: Vector3<f64>) -> Option<SMatrix<f64, 2, 6>> {
    let skew = Matrix3::new(
        0., -body.z, body.y, body.z, 0., -body.x, -body.y, body.x, 0.,
    );
    let mut motion = SMatrix::<f64, 3, 6>::zeros();
    motion
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&-Matrix3::identity());
    motion.fixed_view_mut::<3, 3>(0, 3).copy_from(&skew);
    Some(camera.jacobian(body)? * motion)
}

/// 回环位姿精化；路标固定在参考系，线搜索要求重投影代价下降。
#[must_use]
pub fn refine_pose(
    seed: RigidTransform,
    camera: &Camera,
    observations: &[(Vector3<f64>, [f64; 2])],
) -> Option<RigidTransform> {
    use nalgebra::{Isometry3, SVector, Translation3, UnitQuaternion};
    if observations.len() < 30 {
        return None;
    }
    let cost = |pose: &RigidTransform| -> Option<f64> {
        observations
            .iter()
            .map(|(point, uv)| {
                camera
                    .residual(pose.inverse().point((*point).into()).coords, *uv)
                    .map(|r| {
                        let n = r.norm();
                        if n <= 3. { n * n } else { 6. * n - 9. }
                    })
            })
            .sum()
    };
    let mut pose = seed;
    for _ in 0..5 {
        let mut h = SMatrix::<f64, 6, 6>::zeros();
        let mut g = SVector::<f64, 6>::zeros();
        for (point, uv) in observations {
            let body = pose.inverse().point((*point).into()).coords;
            let r = camera.residual(body, *uv)?;
            let j = pose_jacobian(camera, body)?;
            let w = if r.norm() > 3. { 3. / r.norm() } else { 1. };
            h += w * j.transpose() * j;
            g += w * j.transpose() * r;
        }
        let eigen = h.symmetric_eigen().eigenvalues;
        if eigen.min() <= 0. || eigen.max() / eigen.min() > 1e8 {
            return None;
        }
        let step = h.cholesky()?.solve(&g);
        let before = cost(&pose)?;
        let mut accepted = false;
        for scale in [1., 0.5, 0.25, 0.125] {
            let step = step * (-scale);
            let delta = Isometry3::from_parts(
                Translation3::new(step[0], step[1], step[2]),
                UnitQuaternion::from_scaled_axis(Vector3::new(step[3], step[4], step[5])),
            );
            let trial = RigidTransform::from_matrix(
                pose.target(),
                pose.source(),
                &(pose.isometry() * delta).to_homogeneous(),
            )
            .ok()?;
            if cost(&trial).is_some_and(|v| v <= before) {
                pose = trial;
                accepted = true;
                break;
            }
        }
        if !accepted || step.norm() < 1e-7 {
            break;
        }
    }
    Some(pose)
}
/// 双侧持久路标与实际观测；历史观测允许来自候选的共视关键帧。
pub struct Correspondence<'a> {
    pub source: Vector3<f64>,
    pub target: Vector3<f64>,
    pub source_camera: &'a Camera,
    pub target_camera: &'a Camera,
    pub target_from_anchor: nalgebra::Isometry3<f64>,
    pub source_uv: [f64; 2],
    pub target_uv: [f64; 2],
}
impl Correspondence<'_> {
    #[must_use]
    pub fn agrees(&self, transform: &RigidTransform) -> bool {
        let forward = transform.inverse().point(self.target.into()).coords;
        let backward = self
            .target_from_anchor
            .transform_point(&transform.point(self.source.into()))
            .coords;
        self.source_camera
            .residual(forward, self.source_uv)
            .is_some_and(|r| r.norm() <= 3.)
            && self
                .target_camera
                .residual(backward, self.target_uv)
                .is_some_and(|r| r.norm() <= 3.)
    }
}
/// ORB-SLAM3 `Sim3Solver` 的固定尺度分支：三点米制初解，双侧像素误差判断共识。
/// 初解需要至少 20 对应、15 共识；只能引导投影扩匹配，不能直接作为接受约束。
/// 三维点的轴向不确定性随距离变化，不能用固定米制球代替观测误差模型。
#[must_use]
pub fn loop_consensus(pairs: &[Correspondence<'_>]) -> Option<RigidTransform> {
    if pairs.len() < 20 {
        return None;
    }
    let source: Vec<_> = pairs.iter().map(|p| p.source).collect();
    let target: Vec<_> = pairs.iter().map(|p| p.target).collect();
    let mut state = 0x1234_5678_u64;
    let mut best = Vec::new();
    let mut solution = None;
    for _ in 0..300 {
        let indices: [usize; 3] = std::array::from_fn(|_| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            (state >> 32) as usize % pairs.len()
        });
        if indices[0] == indices[1] || indices[0] == indices[2] || indices[1] == indices[2] {
            continue;
        }
        let Some(estimate) = firefly_vision_match::rgbd::fit_rigid(
            &source,
            &target,
            &indices,
            HISTORIC_BODY,
            FrameId::BODY,
        ) else {
            continue;
        };
        let inliers: Vec<_> = pairs
            .iter()
            .enumerate()
            .filter(|(_, p)| p.agrees(&estimate))
            .map(|(i, _)| i)
            .collect();
        if inliers.len() > best.len() {
            best = inliers;
            solution = Some(estimate);
        }
        if best.len() == pairs.len() {
            break;
        }
    }
    log::info!("loop seed support={}/{}", best.len(), pairs.len());
    if best.len() < 15 {
        return None;
    }
    let refined = firefly_vision_match::rgbd::fit_rigid(
        &source,
        &target,
        &best,
        HISTORIC_BODY,
        FrameId::BODY,
    )?;
    if pairs.iter().filter(|p| p.agrees(&refined)).count() >= best.len() {
        Some(refined)
    } else {
        solution
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn camera() -> Camera {
        Camera {
            camera_from_body: RigidTransform::from_parts(
                CAMERA,
                FrameId::BODY,
                [0.; 3],
                [0., 0., 0., 1.],
            )
            .unwrap(),
            intrinsics: [100., 100., 160., 120.],
        }
    }
    #[test]
    fn bilateral_loop_recovers_known_motion_and_rejects_false_observations() {
        let camera = camera();
        let rotation = nalgebra::UnitQuaternion::from_euler_angles(0.03, -0.02, 0.1);
        let truth = RigidTransform::from_parts(
            HISTORIC_BODY,
            FrameId::BODY,
            [0.3, -0.2, 0.1],
            [rotation.i, rotation.j, rotation.k, rotation.w],
        )
        .unwrap();
        let mut pairs: Vec<_> = (0..60)
            .map(|i| {
                let source = Vector3::new(
                    f64::from(i % 10) * 0.3 - 1.5,
                    f64::from(i / 10) * 0.3 - 0.9,
                    4. + f64::from(i % 3) * 0.2,
                );
                let target = truth.point(source.into()).coords;
                Correspondence {
                    source,
                    target,
                    source_camera: &camera,
                    target_camera: &camera,
                    target_from_anchor: nalgebra::Isometry3::identity(),
                    source_uv: camera.project(source).unwrap().into(),
                    target_uv: camera.project(target).unwrap().into(),
                }
            })
            .collect();
        for pair in pairs.iter_mut().take(12) {
            pair.target += Vector3::new(3., -2., 1.);
            pair.target_uv = camera.project(pair.target).unwrap().into();
        }
        let estimate = loop_consensus(&pairs).unwrap();
        assert!((estimate.matrix() - truth.matrix()).norm() < 1e-10);
        for pair in &mut pairs {
            pair.target_uv[0] += 1.;
        }
        assert!(loop_consensus(&pairs).is_none());
    }

    #[test]
    fn pose_block_jacobian_and_known_translation() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion};
        let camera = camera();
        let body = Vector3::new(1., 2., 5.);
        let jac = pose_jacobian(&camera, body).unwrap();
        let identity =
            RigidTransform::from_parts(FrameId::MAP, FrameId::BODY, [0.; 3], [0., 0., 0., 1.])
                .unwrap();
        for axis in 0..6 {
            let residual = |step: f64| {
                let mut translation = Vector3::zeros();
                let mut rotation = Vector3::zeros();
                if axis < 3 {
                    translation[axis] = step;
                } else {
                    rotation[axis - 3] = step;
                }
                let delta = Isometry3::from_parts(
                    Translation3::from(translation),
                    UnitQuaternion::from_scaled_axis(rotation),
                );
                camera
                    .residual(delta.inverse_transform_point(&body.into()).coords, [0.; 2])
                    .unwrap()
            };
            let numeric = (residual(1e-6) - residual(-1e-6)) / 2e-6;
            assert!((numeric - jac.column(axis)).norm() < 1e-7);
        }
        // 解析投影：相机中心在 (0.2,-0.1,0.3)，朝向不变。
        let observations: Vec<_> = (0..48)
            .map(|i| {
                let point = Vector3::new(
                    f64::from(i % 8) * 0.3 - 1.,
                    f64::from(i / 8) * 0.3 - 0.8,
                    4. + f64::from(i % 3) * 0.2,
                );
                (
                    point,
                    [
                        (point.x - 0.2) / (point.z - 0.3),
                        (point.y + 0.1) / (point.z - 0.3),
                    ],
                )
            })
            .collect();
        let pose = refine_pose(identity, &camera, &observations).unwrap();
        assert!((pose.isometry().translation.vector - Vector3::new(0.2, -0.1, 0.3)).norm() < 1e-5);
        assert!(pose.isometry().rotation.angle() < 1e-5);
    }
    #[test]
    fn projection_jacobian_and_multiview_solution_have_independent_oracles() {
        let camera = camera();
        let point = Vector3::new(1., 2., 5.);
        assert_eq!(camera.project(point).unwrap(), Vector2::new(0.2, 0.4));
        let expected = SMatrix::<f64, 2, 3>::new(20., 0., -4., 0., 20., -8.);
        assert!((camera.jacobian(point).unwrap() - expected).norm() < 1e-12);
        for axis in 0..3 {
            let mut d = Vector3::zeros();
            d[axis] = 1e-6;
            let numerical = (camera.residual(point + d, [0.; 2]).unwrap()
                - camera.residual(point - d, [0.; 2]).unwrap())
                / 2e-6;
            assert!((numerical - expected.column(axis)).norm() < 1e-7);
        }
        let a = RigidTransform::from_parts(FrameId::MAP, FrameId::BODY, [0.; 3], [0., 0., 0., 1.])
            .unwrap();
        let b =
            RigidTransform::from_parts(FrameId::MAP, FrameId::BODY, [1., 0., 0.], [0., 0., 0., 1.])
                .unwrap();
        let output = refine_point(
            Vector3::new(1.1, 1.9, 5.2),
            &[(&a, &camera, [0.2, 0.4]), (&b, &camera, [0., 0.4])],
        )
        .unwrap();
        assert!((output - point).norm() < 1e-5);
        assert!(
            refine_point(
                point,
                &[(&a, &camera, [0.2, 0.4]), (&a, &camera, [0.2, 0.4])]
            )
            .is_none()
        );
    }
}
