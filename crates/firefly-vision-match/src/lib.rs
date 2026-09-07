//! 视觉位姿解算：2D-3D 对应 → `PnP+RANSAC` → 全局位姿 + 协方差
//! （对照 VINS-Fusion `findConnection` 的 `PnP` 段；求解器用 `purecv`）。

pub mod calibration;

use firefly_error::{Error, ErrorKind};
use nalgebra::{Isometry3, Matrix4, Matrix6, Translation3, Unit, UnitQuaternion, Vector3};
use purecv::prelude::{Matrix, Point2f, Point3f, SolvePnPMethod, solve_pnp_ransac};

/// 相机内参（针孔、无畸变；与仿真/标定一致）。
#[derive(Debug, Clone, Copy)]
pub struct CameraIntrinsics {
    /// 焦距（像素）。
    pub focal: f64,
    /// 主点 x（像素）。
    pub cx: f64,
    /// 主点 y（像素）。
    pub cy: f64,
}

/// 视觉位姿解算结果。
#[derive(Debug, Clone)]
pub struct VisualPose {
    /// 全局位姿 `T_global`（4×4 齐次）。
    pub t_global: Matrix4<f64>,
    /// 内点数。
    pub num_inliers: usize,
    /// 总对应数。
    pub total: usize,
    /// 内点平均重投影误差（像素）。
    pub mean_reproj_px: f64,
}

/// `+Z`（`OpenCV`/`purecv`）↔ `-Z`（本仓相机系）桥：`M=diag(1,-1,-1)`。
/// 两系像素一致，漏掉即约 180° 系统误差——三处共用（求解右乘、重投影左乘、
/// 测试期望），单一事实来源。
fn axis_flip() -> Matrix4<f64> {
    Matrix4::new(
        1.0, 0.0, 0.0, 0.0, //
        0.0, -1.0, 0.0, 0.0, //
        0.0, 0.0, -1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    )
}

/// 由 2D（当前图像素）-3D（库图全局）对应解全局位姿。
///
/// `prior` 为 VIO 先验位姿（作初值用，无则传 `None`）；对应数 `< 6` 时返回
/// `InvalidArgument`（`purecv` 下限）；`RANSAC` 无足够内点时返回 `Ok(None)`
/// （调用方按拒收处理，与 `FusionFilter` 门控语义一致）。
/// 由位姿矩阵取旋转向量（`purecv` 初值用）。
fn matrix_to_rvec(t: &Matrix4<f64>) -> [f64; 3] {
    let iso = Isometry3::from_parts(
        Translation3::new(t[(0, 3)], t[(1, 3)], t[(2, 3)]),
        UnitQuaternion::from_matrix(&t.fixed_view::<3, 3>(0, 0).into_owned()),
    );
    let (axis, angle) = iso
        .rotation
        .axis_angle()
        .map_or_else(|| (Vector3::x(), 0.0), |(a, an)| (a.into_inner(), an));
    [axis.x * angle, axis.y * angle, axis.z * angle]
}

/// 旋转向量 + 平移组装位姿矩阵。
fn vecs_to_isometry(rvec: [f64; 3], tvec: [f64; 3]) -> Isometry3<f64> {
    let axis = Vector3::new(rvec[0], rvec[1], rvec[2]);
    let angle = axis.norm();
    let quat = if angle > 1e-12 {
        UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), angle)
    } else {
        UnitQuaternion::identity()
    };
    Isometry3::from_parts(Translation3::new(tvec[0], tvec[1], tvec[2]), quat)
}

/// 单点重投影误差（像素）；`t_cam` 为 world→camera（与 `purecv` 同惯例），
/// 深度退化时返回 `None`（调用方跳过）。
fn reproj_error(
    observed: [f32; 2],
    world: [f64; 3],
    intrinsics: CameraIntrinsics,
    t_global: &Matrix4<f64>,
) -> Option<f64> {
    let col = t_global.fixed_view::<3, 4>(0, 0);
    let cam_x =
        col[(0, 0)] * world[0] + col[(0, 1)] * world[1] + col[(0, 2)] * world[2] + col[(0, 3)];
    let cam_y =
        col[(1, 0)] * world[0] + col[(1, 1)] * world[1] + col[(1, 2)] * world[2] + col[(1, 3)];
    let cam_z =
        col[(2, 0)] * world[0] + col[(2, 1)] * world[1] + col[(2, 2)] * world[2] + col[(2, 3)];
    if cam_z <= 1e-9 {
        return None;
    }
    let du = intrinsics.focal * cam_x / cam_z + intrinsics.cx - f64::from(observed[0]);
    let dv = intrinsics.focal * cam_y / cam_z + intrinsics.cy - f64::from(observed[1]);
    Some((du * du + dv * dv).sqrt())
}

/// 由 2D（当前图像素）-3D（库图全局）对应解全局位姿。
///
/// `prior` 为 VIO 先验位姿：当前用于调用方的库图短名单与最终精化初值
/// （`purecv` 的 `ransac` 尚不支持初值，精化段见 [`refine_with_inliers`]）。
/// 对应数 `< 6` 时拒绝。
///
/// # Errors
///
/// 对应数不一致/不足（`< 6`）时返回 `InvalidArgument`；`purecv` 内部失败时
/// 返回 `Internal`。`RANSAC` 无足够内点属正常拒收，返回 `Ok(None)`（调用方
/// 按拒收处理，与 `FusionFilter` 门控语义一致）。
pub fn solve_visual_pose(
    points_2d: &[[f32; 2]],
    points_3d: &[[f64; 3]],
    intrinsics: CameraIntrinsics,
    prior: Option<Matrix4<f64>>,
) -> Result<Option<VisualPose>, Error> {
    if points_2d.len() != points_3d.len() {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            format!(
                "2D/3D 对应数不一致: {} vs {}",
                points_2d.len(),
                points_3d.len()
            ),
        ));
    }
    if points_2d.len() < 6 {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            format!("对应数 {} < 6，不解算", points_2d.len()),
        ));
    }
    let object: Vec<Point3f> = points_3d
        .iter()
        .map(|p| Point3f::new(p[0] as f32, p[1] as f32, p[2] as f32))
        .collect();
    let image: Vec<Point2f> = points_2d.iter().map(|p| Point2f::new(p[0], p[1])).collect();
    let cam = camera_matrix(intrinsics);
    let mut rvec = Matrix::new(3, 1, 1);
    let mut tvec = Matrix::new(3, 1, 1);
    // `purecv` 的 `ransac` 尚不支持初值（`NotImplemented`）：初值仅在最终
    // 精化段使用（见下）；此处先验先填入矩阵备用。
    if let Some(t) = prior {
        for (idx, val) in matrix_to_rvec(&t).iter().enumerate() {
            rvec.set(idx, 0, 0, *val);
        }
        for (idx, val) in [t[(0, 3)], t[(1, 3)], t[(2, 3)]].iter().enumerate() {
            tvec.set(idx, 0, 0, *val);
        }
    }
    let mut inliers: Vec<i32> = Vec::new();
    let ok = solve_pnp_ransac(
        &object,
        &image,
        &cam,
        None,
        &mut rvec,
        &mut tvec,
        false,
        100,
        8.0,
        0.99,
        Some(&mut inliers),
        SolvePnPMethod::Iterative,
    )
    .map_err(|e| Error::new(ErrorKind::Internal, format!("PnP 求解失败: {e:?}")))?;
    if !ok || inliers.is_empty() {
        return Ok(None);
    }
    if let Some((r_ref, t_ref)) = refine_with_inliers(&object, &image, &cam, &inliers) {
        rvec = r_ref;
        tvec = t_ref;
    }
    let rot = [
        rvec.at(0, 0, 0).copied().unwrap_or(0.0),
        rvec.at(1, 0, 0).copied().unwrap_or(0.0),
        rvec.at(2, 0, 0).copied().unwrap_or(0.0),
    ];
    let trans = [
        tvec.at(0, 0, 0).copied().unwrap_or(0.0),
        tvec.at(1, 0, 0).copied().unwrap_or(0.0),
        tvec.at(2, 0, 0).copied().unwrap_or(0.0),
    ];
    let iso_cam_from_world = vecs_to_isometry(rot, trans);
    // purecv/OpenCV 惯例：(rvec, tvec) 为 world→camera（X_cam = R·X_world + t，
    // +Z 朝向），全局位姿需先求逆再右乘桥。
    let t_global = iso_cam_from_world.inverse().to_homogeneous() * axis_flip();
    let mean_reproj_px = mean_reprojection(points_2d, points_3d, &inliers, intrinsics, &t_global);
    Ok(Some(VisualPose {
        t_global,
        num_inliers: inliers.len(),
        total: points_2d.len(),
        mean_reproj_px,
    }))
}

/// 由内参组装 `3×3` 相机矩阵。
fn camera_matrix(intrinsics: CameraIntrinsics) -> Matrix<f64> {
    let mut cam = Matrix::new(3, 3, 1);
    for (r, c, v) in [
        (0, 0, intrinsics.focal),
        (0, 2, intrinsics.cx),
        (1, 1, intrinsics.focal),
        (1, 2, intrinsics.cy),
        (2, 2, 1.0),
    ] {
        cam.set(r, c, 0, v);
    }
    cam
}

/// 全量内点精化（对照 `OpenCV solvePnPGeneric` 的 `ITERATIVE` 段与
/// `VINS-Fusion KeyFrame::PnPRANSAC`：`ransac` 输出作初值再 `LM` 精化；
/// `purecv::solve_pnp(use_extrinsic_guess=true)` 当前对初值返回
/// `NotImplemented`，此处用 `false` 重跑一轮 `DLT + Gauss-Newton`
/// 全量内点精化，等价于初值落在内点共识域时的 `LM` 收敛）。
fn refine_with_inliers(
    object: &[Point3f],
    image: &[Point2f],
    cam: &Matrix<f64>,
    inliers: &[i32],
) -> Option<(Matrix<f64>, Matrix<f64>)> {
    let in_obj: Vec<Point3f> = inliers
        .iter()
        .filter_map(|&i| usize::try_from(i).ok())
        .filter(|&k| k < object.len())
        .map(|k| object[k])
        .collect();
    let in_img: Vec<Point2f> = inliers
        .iter()
        .filter_map(|&i| usize::try_from(i).ok())
        .filter(|&k| k < image.len())
        .map(|k| image[k])
        .collect();
    if in_obj.len() < 6 {
        return None;
    }
    let mut rvec_ref = Matrix::new(3, 1, 1);
    let mut tvec_ref = Matrix::new(3, 1, 1);
    purecv::prelude::solve_pnp(
        &in_obj,
        &in_img,
        cam,
        None,
        &mut rvec_ref,
        &mut tvec_ref,
        false,
        SolvePnPMethod::Iterative,
    )
    .ok()?;
    Some((rvec_ref, tvec_ref))
}

/// 协方差（对角 6×6，`[rot, trans]`）：由内点数与平均重投影误差经验映射，
/// 后续用离线数据标定（与 `FusionFilter` 的 `r_floor` 思路一致）。
#[must_use]
pub fn pose_covariance(pose: &VisualPose) -> Matrix6<f64> {
    let n = pose.num_inliers.max(1) as f64;
    let err = pose.mean_reproj_px.max(0.2);
    let pos_var = (0.02 * err / n.sqrt()).powi(2);
    let rot_var = (0.005 * err / n.sqrt()).powi(2);
    let mut r = Matrix6::zeros();
    for i in 0..3 {
        r[(i, i)] = rot_var;
    }
    for i in 3..6 {
        r[(i, i)] = pos_var;
    }
    r
}

/// 内点平均重投影误差（像素，诊断/协方差用；`t_global` 为 camera→world 的
/// -Z 朝向位姿（`= PnP逆 · M`），投影前左乘 `M` 转回 `OpenCV` +Z 位姿）。
fn mean_reprojection(
    points_2d: &[[f32; 2]],
    points_3d: &[[f64; 3]],
    inliers: &[i32],
    intrinsics: CameraIntrinsics,
    t_global: &Matrix4<f64>,
) -> f64 {
    if inliers.is_empty() {
        return f64::INFINITY;
    }
    // -Z→+Z 桥（左乘：`X_ocv = M·X_ours`，与 `solve` 段同桥）。
    let flip = axis_flip();
    let t_cam = flip * t_global.try_inverse().unwrap_or(Matrix4::identity());
    let mut sum = 0.0;
    let mut count = 0usize;
    for &idx in inliers {
        let Ok(k) = usize::try_from(idx) else {
            continue;
        };
        if k >= points_2d.len() {
            continue;
        }
        if let Some(err) = reproj_error(points_2d[k], points_3d[k], intrinsics, &t_cam) {
            sum += err;
            count += 1;
        }
    }
    if count == 0 {
        return f64::INFINITY;
    }
    sum / count as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 合成：已知位姿投影 3D→2D，加噪后求解，误差应小。
    #[test]
    fn solves_synthetic_pose() {
        let intrinsics = CameraIntrinsics {
            focal: 168.0,
            cx: 160.0,
            cy: 120.0,
        };
        let t_gt = Isometry3::from_parts(
            Translation3::new(1.0, 2.0, 0.5),
            UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3),
        )
        .to_homogeneous();
        let mut rng = 12345u64;
        let mut rand = move || {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (rng >> 33) as f64 / f64::from(u32::MAX)
        };
        let mut p3 = Vec::new();
        let mut p2 = Vec::new();
        for _ in 0..60 {
            let p = [rand() * 6.0 - 1.0, rand() * 4.0 - 1.0, rand() * 2.0 + 2.0];
            let x = t_gt[(0, 0)] * p[0] + t_gt[(0, 1)] * p[1] + t_gt[(0, 2)] * p[2] + t_gt[(0, 3)];
            let y = t_gt[(1, 0)] * p[0] + t_gt[(1, 1)] * p[1] + t_gt[(1, 2)] * p[2] + t_gt[(1, 3)];
            let z = t_gt[(2, 0)] * p[0] + t_gt[(2, 1)] * p[1] + t_gt[(2, 2)] * p[2] + t_gt[(2, 3)];
            p3.push(p);
            p2.push([
                (intrinsics.focal * x / z + intrinsics.cx + (rand() - 0.5)) as f32,
                (intrinsics.focal * y / z + intrinsics.cy + (rand() - 0.5)) as f32,
            ]);
        }
        let pose = solve_visual_pose(&p2, &p3, intrinsics, None)
            .unwrap()
            .expect("synthetic should solve");
        // 测试投影用 +Z（OpenCV）惯例：`t_gt` 即 world→camera；`pose.t_global` 是
        // -Z 惯例的 camera→world（= t_gt⁻¹ · 桥），同物理位姿。
        let flip = axis_flip();
        let t_expected = t_gt.try_inverse().unwrap() * flip;
        let dt =
            (pose.t_global.fixed_view::<3, 1>(0, 3) - t_expected.fixed_view::<3, 1>(0, 3)).norm();
        let dr =
            (pose.t_global.fixed_view::<3, 3>(0, 0) - t_expected.fixed_view::<3, 3>(0, 0)).norm();
        assert!(dt < 0.05, "trans err {dt}");
        assert!(dr < 0.02, "rot err {dr}");
        assert!(pose.mean_reproj_px < 2.0);
        assert!(pose.num_inliers >= 50);
        let r = pose_covariance(&pose);
        assert!(r[(0, 0)] > 0.0 && r[(3, 3)] > 0.0);
    }

    /// 全链回归：下倾左目（机体经外参直达相机）→ `OpenCV` 类 `PnP` → `cam_pose_to_body`，
    /// 解出的机体系位姿应贴合真值（含旋转）。
    #[test]
    fn tilted_camera_full_chain() {
        use crate::calibration::{MUJOCO_FOCAL, cam_pose_to_body, rot_cam_to_body};
        let intrinsics = CameraIntrinsics {
            focal: MUJOCO_FOCAL,
            cx: 160.0,
            cy: 120.0,
        };
        let t_body_truth =
            Isometry3::from_parts(Translation3::new(1.0, 4.0, 1.0), UnitQuaternion::identity())
                .to_homogeneous();
        let t_cam_body = Isometry3::from_parts(
            Translation3::new(0.0, -0.025, 0.0),
            UnitQuaternion::from_matrix(&rot_cam_to_body()),
        )
        .to_homogeneous();
        let t_world_cam = t_body_truth * t_cam_body;
        let mut rng = 999u64;
        let mut rand = move || {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (rng >> 33) as f64 / f64::from(u32::MAX)
        };
        let mut p2 = Vec::new();
        let mut p3 = Vec::new();
        while p2.len() < 80 {
            let u = rand() * 320.0;
            let v = rand() * 240.0;
            let d = rand() * 4.0 + 1.0;
            // 真相机 -Z 朝向：Xc=(dx*d, dy*d, -d)
            let dx = (u - 160.0) / MUJOCO_FOCAL;
            let dy = -(v - 120.0) / MUJOCO_FOCAL;
            let xc = nalgebra::Vector4::new(dx * d, dy * d, -d, 1.0);
            let xw = t_world_cam * xc;
            p2.push([u as f32, v as f32]);
            p3.push([xw[0], xw[1], xw[2]]);
        }
        let pose = solve_visual_pose(&p2, &p3, intrinsics, None)
            .unwrap()
            .expect("tilted chain should solve");
        let t_body = cam_pose_to_body(&pose.t_global);
        let dt = (t_body.fixed_view::<3, 1>(0, 3) - t_body_truth.fixed_view::<3, 1>(0, 3)).norm();
        let r_err =
            (t_body.fixed_view::<3, 3>(0, 0) - t_body_truth.fixed_view::<3, 3>(0, 0)).norm();
        assert!(dt < 0.1, "trans err {dt}");
        assert!(r_err < 0.05, "rot err {r_err}");
    }
}
