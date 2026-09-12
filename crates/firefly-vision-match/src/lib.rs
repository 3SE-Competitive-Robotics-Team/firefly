//! 视觉位姿解算：2D-3D 对应 → `PnP+RANSAC` → 全局位姿 + 协方差
//! （对照 VINS-Fusion `findConnection` 的 `PnP` 段；求解器用 `purecv`）。

pub mod calibration;

use firefly_error::{Error, ErrorKind};
use nalgebra::{Isometry3, Matrix3, Matrix4, Matrix6, Translation3, Unit, UnitQuaternion, Vector3};
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

/// 先验几何门：PnP 解与先验差 `PRIOR_MAX_TRANS` / `PRIOR_MAX_ROT` 以上即拒收。
/// 平移阈值取库图查询半径（解落在检索球外即自相矛盾）；旋转阈值远大于
/// `VIO` 短时漂移、远小于 `PnP` 翻转二义性的 `~90°+`。
pub const PRIOR_MAX_TRANS: f64 = 3.0;
/// 先验几何门旋转阈值（弧度，15°）。
pub const PRIOR_MAX_ROT: f64 = 15.0 * std::f64::consts::PI / 180.0;

/// 由 2D（当前图像素）-3D（库图全局）对应解全局位姿。
///
/// `prior` 为 VIO 先验位姿（相机系，与返回的 `t_global` 同系）：RANSAC 解算后
/// 与先验做几何一致性校验（对照 VINS-Fusion `KeyFrame::findConnection` 尾段
/// `abs(relative_yaw) < 30° && relative_t.norm() < 20m` 才接受 loop 边——本链
/// 是连续跟踪，先验可信度高于检环，故阈值更紧）。无先验（`None`）时不校验。
/// 对应数 `< 6` 时拒绝。
///
/// # Errors
///
/// 对应数不一致/不足（`< 6`）时返回 `InvalidArgument`；`purecv` 内部失败时
/// 返回 `Internal`。
///
/// `RANSAC` 无足够内点、先验几何门未通过属正常拒收，返回 `Ok(None)`（调用方
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
    // 主路：全集 RANSAC＋精化。无先验不校验、有先验过门即发布（原行为）。
    if let Some((t_global, inliers)) = ransac_pose(&object, &image, &cam)? {
        let prior_ok = prior
            .as_ref()
            .is_none_or(|t_prior| passes_prior_gate(&t_global, t_prior));
        if prior_ok {
            let mean_reproj_px =
                mean_reprojection(points_2d, points_3d, &inliers, intrinsics, &t_global);
            return Ok(Some(VisualPose {
                t_global,
                num_inliers: inliers.len(),
                total: points_2d.len(),
                mean_reproj_px,
            }));
        }
    }
    // 恢复路：主路未发布（门拒收或 RANSAC 无共识）且先验存在 → 先验子集重解：
    // 先验投影误差宽松筛选（`PRIOR_SUBSET_PX`）后子集上再跑 RANSAC＋精化。
    // 走廊混叠下全集共识常锁翻转解；子集提高真值占比后 RANSAC 有机会命中
    // 真值 basin。失败即回落 `None`（fail-closed，原行为）。严格增量：主路
    // 能过则绝不走到这里，已发布行为零变化。
    let Some(t_prior) = prior else {
        return Ok(None);
    };
    // 先验（camera→world，`-Z`）→ OCV（world→camera，`+Z`）（与
    // `mean_reprojection` 内桥同式：`ocv = flip · prior⁻¹`，注意不可交换
    // 左右乘顺序）。
    let flip = axis_flip();
    let Some(prior_inv) = t_prior.try_inverse() else {
        return Ok(None);
    };
    let ocv_prior = flip * prior_inv;
    let r_p = ocv_prior.fixed_view::<3, 3>(0, 0).into_owned();
    let t_p = ocv_prior.fixed_view::<3, 1>(0, 3).into_owned();
    // 子集数组与内点索引同系（`mean_reprojection` 按索引取点，错系即错数）。
    let mut sub_3d: Vec<[f64; 3]> = Vec::new();
    let mut sub_2d: Vec<[f32; 2]> = Vec::new();
    for (p3, p2) in points_3d.iter().zip(points_2d.iter()) {
        let Some((u, v)) = project_ocv(&r_p, &t_p, p3, intrinsics) else {
            continue;
        };
        let du = u - f64::from(p2[0]);
        let dv = v - f64::from(p2[1]);
        if (du * du + dv * dv).sqrt() <= PRIOR_SUBSET_PX {
            sub_3d.push(*p3);
            sub_2d.push(*p2);
        }
    }
    log::debug!("先验子集重解（子集 {}/{})", sub_3d.len(), points_2d.len());
    // 子集与全集相同 → 确定性 RANSAC 重跑结果相同，跳过。
    if sub_3d.len() == points_2d.len() || sub_3d.len() < SUBSET_MIN_PAIRS {
        return Ok(None);
    }
    let sub_obj: Vec<Point3f> = sub_3d
        .iter()
        .map(|p| Point3f::new(p[0] as f32, p[1] as f32, p[2] as f32))
        .collect();
    let sub_img: Vec<Point2f> = sub_2d.iter().map(|p| Point2f::new(p[0], p[1])).collect();
    let Some((t_global_b, inliers_b)) = ransac_pose(&sub_obj, &sub_img, &cam)? else {
        return Ok(None);
    };
    if !passes_prior_gate(&t_global_b, &t_prior) {
        return Ok(None);
    }
    let mean_reproj_px = mean_reprojection(&sub_2d, &sub_3d, &inliers_b, intrinsics, &t_global_b);
    log::debug!(
        "先验子集恢复发布（内点 {}/{})",
        inliers_b.len(),
        points_2d.len()
    );
    Ok(Some(VisualPose {
        t_global: t_global_b,
        num_inliers: inliers_b.len(),
        total: points_2d.len(),
        mean_reproj_px,
    }))
}

/// 先验几何门：解与先验的平移差与旋转差是否在限内（两系同为 camera→world）。
fn passes_prior_gate(t_global: &Matrix4<f64>, t_prior: &Matrix4<f64>) -> bool {
    let dt = (t_global.fixed_view::<3, 1>(0, 3) - t_prior.fixed_view::<3, 1>(0, 3)).norm();
    let r_rel = t_global.fixed_view::<3, 3>(0, 0).into_owned().transpose()
        * t_prior.fixed_view::<3, 3>(0, 0).into_owned();
    let cos_a = ((r_rel.trace() - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cos_a.acos();
    let pass = dt <= PRIOR_MAX_TRANS && angle <= PRIOR_MAX_ROT;
    log::debug!(
        "先验几何门 dt={dt:.2}m rot={:.1}°（限 3.0m/15°）{}",
        angle.to_degrees(),
        if pass { "通过" } else { "拒收" }
    );
    pass
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

/// 先验播种精化调参（与 `RANSAC` 同口径，见 `solve_pnp_ransac` 调用处）。
/// `Huber` 拐点：真值内点亚像素、混叠外点数十像素以上，`2px` 分得开。
/// 先验子集筛选半径（像素）：覆盖米级先验误差在 5m+ 深度的投影（`~35px`）
/// 并留裕量；混叠外点在此半径外，被子集排除后 `RANSAC` 有机会命中真值。
/// 子集与全集相同时跳过重试（`RANSAC` 确定性种子，重跑结果相同）。
const PRIOR_SUBSET_PX: f64 = 64.0;
/// 子集最小对应数（低于此数 `RANSAC` 无共识 odds，直接回落）。
const SUBSET_MIN_PAIRS: usize = 12;

/// OCV 针孔投影（`+Z` 朝向，无畸变，与 `purecv` 同假设）：返回像素与相机系坐标。
fn project_ocv(
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
    p: &[f64; 3],
    intr: CameraIntrinsics,
) -> Option<(f64, f64)> {
    let c = r * Vector3::new(p[0], p[1], p[2]) + t;
    // 非有限与近零深度都不可投影（`NaN <= x` 为假，须显式判非有限）。
    if !c.z.is_finite() || c.z <= 1e-9 {
        return None;
    }
    Some((
        intr.focal * c.x / c.z + intr.cx,
        intr.focal * c.y / c.z + intr.cy,
    ))
}

/// RANSAC＋精化＋成位姿（无门）：`RANSAC` 共识 → 全量内点精化 →
/// `t_global`（camera→world，`-Z`）。无共识返回空；调用方做门与发布。
/// （`purecv` 的 `RANSAC` 定种子，同输入同输出，子集与全集相同时禁止重试。）
/// （返回类型复杂是领域常态，本文件 `match_frame` 处同惯例。）
#[allow(clippy::type_complexity)]
fn ransac_pose(
    object: &[Point3f],
    image: &[Point2f],
    cam: &Matrix<f64>,
) -> Result<Option<(Matrix4<f64>, Vec<i32>)>, Error> {
    let mut rvec = Matrix::new(3, 1, 1);
    let mut tvec = Matrix::new(3, 1, 1);
    let mut inliers: Vec<i32> = Vec::new();
    let ok = solve_pnp_ransac(
        object,
        image,
        cam,
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
        log::debug!(
            "PnP 无共识（ok={ok} 内点 {}/{} 对应）",
            inliers.len(),
            object.len()
        );
        return Ok(None);
    }
    if let Some((r_ref, t_ref)) = refine_with_inliers(object, image, cam, &inliers) {
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
    Ok(Some((
        iso_cam_from_world.inverse().to_homogeneous() * axis_flip(),
        inliers,
    )))
}

/// 协方差（对角 6×6，`[rot, trans]`）：由内点数与平均重投影误差经验映射，
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

    /// 先验几何门：近先验解通过，公里级野值拒收（无先验时不校验）。
    ///
    /// 直接构造门输入（`t_global`/`t_prior` 同为 camera→world）：几何门是纯矩阵
    /// 比较，与 `PnP` 求解器无关——合成投影的符号/桥约定由上两个测试覆盖。
    #[test]
    fn prior_gate_rejects_wild_pose() {
        let near = Isometry3::from_parts(
            Translation3::new(20.1, 0.05, 1.02),
            UnitQuaternion::from_euler_angles(0.01, 0.02, 0.03),
        )
        .to_homogeneous();
        let prior = Isometry3::from_parts(
            Translation3::new(20.0, 0.0, 1.0),
            UnitQuaternion::identity(),
        )
        .to_homogeneous();
        assert!(passes_prior_gate(&near, &prior));
        let far = Isometry3::from_parts(
            Translation3::new(40000.0, 0.0, 1.0),
            UnitQuaternion::identity(),
        )
        .to_homogeneous();
        assert!(!passes_prior_gate(&far, &prior));
        let flipped = Isometry3::from_parts(
            Translation3::new(20.0, 0.0, 1.0),
            UnitQuaternion::from_euler_angles(0.0, std::f64::consts::PI, 0.0),
        )
        .to_homogeneous();
        assert!(!passes_prior_gate(&flipped, &prior));
        assert!(passes_prior_gate(&near, &near));
    }

    /// 合成位姿真值（OCV 系）与投影：`project_ocv` 自洽性由收敛测试覆盖。
    fn synth_scene(
        n: usize,
        planar: bool,
        r_gt: Matrix3<f64>,
        t_gt: Vector3<f64>,
    ) -> (Vec<[f64; 3]>, Vec<[f32; 2]>) {
        let intr = CameraIntrinsics {
            focal: 168.0,
            cx: 160.0,
            cy: 120.0,
        };
        let mut rng = 777u64;
        let mut rand = move || {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (rng >> 33) as f64 / f64::from(u32::MAX)
        };
        let mut obj = Vec::new();
        let mut img = Vec::new();
        while obj.len() < n {
            let p = if planar {
                [rand() * 12.0 - 6.0, rand() * 4.0 - 2.0, 8.0]
            } else {
                [rand() * 6.0 - 3.0, rand() * 4.0 - 2.0, rand() * 5.0 + 4.0]
            };
            let Some((u, v)) = project_ocv(&r_gt, &t_gt, &p, intr) else {
                continue;
            };
            if !(0.0..320.0).contains(&u) || !(0.0..240.0).contains(&v) {
                continue;
            }
            obj.push(p);
            img.push([u as f32, v as f32]);
        }
        (obj, img)
    }

    /// 端到端翻转共识恢复：20 真值对混入 60 翻转一致对（`RANSAC` 全集共识
    /// 锁翻转解）＋真值附近先验 → 先验子集（真值占比反转）重解发布真值。
    /// 无论全集直解命中哪边，最终必为真值 basin（翻转 `~180°` 过不了先验门）。
    #[test]
    fn solve_recovers_flip_consensus() {
        let intr = CameraIntrinsics {
            focal: 168.0,
            cx: 160.0,
            cy: 120.0,
        };
        let r_gt = Matrix3::identity();
        let t_gt = Vector3::zeros();
        let (mut obj, mut img) = synth_scene(20, true, r_gt, t_gt);
        // 翻转一致对：绕光轴 180° 的位姿下自洽（与真值差 180°，先验门必拒）。
        let r_flip = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI)
            .to_rotation_matrix()
            .into_inner();
        let mut rng = 4242u64;
        let mut rand = move || {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (rng >> 33) as f64 / f64::from(u32::MAX)
        };
        while obj.len() < 80 {
            let p = [rand() * 12.0 - 6.0, rand() * 4.0 - 2.0, 8.0];
            let Some((u, v)) = project_ocv(&r_flip, &t_gt, &p, intr) else {
                continue;
            };
            if !(0.0..320.0).contains(&u) || !(0.0..240.0).contains(&v) {
                continue;
            }
            obj.push(p);
            img.push([u as f32, v as f32]);
        }
        let dq = UnitQuaternion::from_euler_angles(0.0, 0.14, 0.0);
        let r_pert = dq.to_rotation_matrix().into_inner();
        let t_pert = Vector3::new(0.5, 0.0, 0.0);
        let mut ocv = Matrix4::identity();
        ocv.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_pert);
        ocv.fixed_view_mut::<3, 1>(0, 3).copy_from(&t_pert);
        let t_prior = ocv.try_inverse().expect("synthetic ocv invertible") * axis_flip();
        let pose = solve_visual_pose(
            &img.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>(),
            &obj,
            intr,
            Some(t_prior),
        )
        .unwrap()
        .expect("subset recovery should publish truth");
        let dt = pose.t_global.fixed_view::<3, 1>(0, 3).norm();
        assert!(dt < 0.5, "trans err {dt}");
        let m = axis_flip().fixed_view::<3, 3>(0, 0).into_owned();
        let cos_a = ((pose
            .t_global
            .fixed_view::<3, 3>(0, 0)
            .into_owned()
            .transpose()
            * m)
            .trace()
            - 1.0)
            / 2.0;
        assert!(cos_a.clamp(-1.0, 1.0).acos() < 0.17, "rot off basin");
    }

    /// 端到端翻转恢复：平面点集（走廊墙面形态）＋真值附近先验 → 发布真值
    /// basin（无论 `RANSAC` 直解命中与否；翻转解 `~180°` 过不了先验门）。
    #[test]
    fn solve_recovers_prior_basin_on_plane() {
        let intr = CameraIntrinsics {
            focal: 168.0,
            cx: 160.0,
            cy: 120.0,
        };
        let r_gt = Matrix3::identity();
        let t_gt = Vector3::zeros();
        let (obj, img_f32) = synth_scene(80, true, r_gt, t_gt);
        // 先验（camera→world，`-Z`）：真值扰动 0.5m/8° 后过桥。
        let dq = UnitQuaternion::from_euler_angles(0.0, 0.14, 0.0);
        let r_pert = dq.to_rotation_matrix().into_inner();
        let t_pert = Vector3::new(0.5, 0.0, 0.0);
        let mut ocv = Matrix4::identity();
        ocv.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_pert);
        ocv.fixed_view_mut::<3, 1>(0, 3).copy_from(&t_pert);
        let t_prior = ocv.try_inverse().expect("synthetic ocv invertible") * axis_flip();
        let p2: Vec<[f32; 2]> = img_f32;
        let p3: Vec<[f64; 3]> = obj;
        let pose = solve_visual_pose(&p2, &p3, intr, Some(t_prior))
            .unwrap()
            .expect("planar scene with near prior should publish");
        // 期望真值 basin（`ocv⁻¹·flip` 的平移为零、旋转为桥本身）。
        let dt = pose.t_global.fixed_view::<3, 1>(0, 3).norm();
        assert!(dt < 0.5, "trans err {dt}");
        let m = axis_flip().fixed_view::<3, 3>(0, 0).into_owned();
        let cos_a = ((pose
            .t_global
            .fixed_view::<3, 3>(0, 0)
            .into_owned()
            .transpose()
            * m)
            .trace()
            - 1.0)
            / 2.0;
        assert!(cos_a.clamp(-1.0, 1.0).acos() < 0.17, "rot off basin");
    }
}
