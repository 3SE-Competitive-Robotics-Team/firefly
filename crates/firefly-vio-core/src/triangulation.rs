//! 特征三角化（对照 `OpenVINS` `ov_core/src/feat/FeatureInitializer.cpp` 的
//! `single_triangulation`；`single_triangulation_1d`/`single_gaussnewton`
//! 属 SLAM 特征初始化，标注 TODO 待 SLAM 移植时补充）。
//!
//! 多视图线性三角化（DLT）：以测量最多的相机为锚点，把每条视线投影到与
//! 锚点系垂直的平面（`Bperp = skew(b_i)`），累加正规方程 `A p = b` 求解，
//! 最后做条件数与深度范围检查（对照 `FeatureInitializer::single_triangulation`）。

use std::collections::HashMap;

use nalgebra::{Matrix3, Vector2, Vector3};

use crate::feat::Feature;
use firefly_vio_types::quat_ops::skew_x;

/// 某克隆时刻的位姿（对照 `ClonePose`：`Rot()`/`pos()`）。
#[derive(Debug, Clone)]
pub struct ClonePose {
    /// 全局到相机系旋转 `R_GtoCi`。
    pub rot: Matrix3<f64>,
    /// 相机系原点在全局系位置 `p_CiinG`。
    pub pos: Vector3<f64>,
}

/// 三角化检查参数（对照 `FeatureInitializerOptions` 默认值）。
#[derive(Debug, Clone, Copy)]
pub struct TriangulationOptions {
    /// 条件数上限（`max_cond_number = 10000`）。
    pub max_cond_number: f64,
    /// 最小深度（`min_dist = 0.10` m）。
    pub min_dist: f64,
    /// 最大深度（`max_dist = 60` m）。
    pub max_dist: f64,
    /// 是否在三角化后做高斯牛顿精化（`refine_features = true`）。
    pub refine_features: bool,
    /// 精化最大迭代次数（`max_runs = 5`）。
    pub max_runs: usize,
    /// LM 阻尼初值（`init_lamda = 1e-3`）。
    pub init_lamda: f64,
    /// LM 阻尼上限（`max_lamda = 1e10`）。
    pub max_lamda: f64,
    /// 步长收敛阈值（`min_dx = 1e-6`）。
    pub min_dx: f64,
    /// 相对代价收敛阈值（`min_dcost = 1e-6`）。
    pub min_dcost: f64,
    /// 阻尼倍增系数（`lam_mult = 10`）。
    pub lam_mult: f64,
    /// 深度/基线比上限（`max_baseline = 40`）。
    pub max_baseline: f64,
}

impl Default for TriangulationOptions {
    fn default() -> Self {
        Self {
            max_cond_number: 10_000.0,
            min_dist: 0.10,
            max_dist: 60.0,
            refine_features: true,
            max_runs: 5,
            init_lamda: 1e-3,
            max_lamda: 1e10,
            min_dx: 1e-6,
            min_dcost: 1e-6,
            lam_mult: 10.0,
            max_baseline: 40.0,
        }
    }
}

/// 相机在某时刻的位姿表（时间升序；f64 无 `Hash`/`Ord`，用 `Vec` 线性查找，
/// 每相机克隆数 ≤ 滑动窗口大小，代价可忽略）。
pub type CloneMap = Vec<(f64, ClonePose)>;

/// 多视图三角化一个特征（对照 `FeatureInitializer::single_triangulation`）。
///
/// 设置 `feat.anchor_cam_id`/`anchor_clone_timestamp`（测量最多的相机 +
/// 该相机最后一个测量时刻），解算 `p_FinA`（锚点系）与 `p_FinG`（全局系）。
///
/// `clones_cam`：相机 id → `CloneMap`（由调用方从滑动窗口组装）。
///
/// # Returns
/// `true` 表示三角化成功；失败原因：条件数过大、深度越界或数值异常
/// （与 C++ 的返回语义一致，失败时 `p_FinA/p_FinG` 保持原值）。
///
/// # Panics
/// 特征无测量；锚点克隆缺失；或测量相机的克隆表中缺少对应时刻的位姿
/// （与 C++ 的 `.at()` 抛异常语义一致，属调用方组装错误）。
#[must_use]
#[allow(clippy::implicit_hasher)] // 键为固定 usize 相机 id，默认 hasher 足够
pub fn single_triangulation(
    feat: &mut Feature,
    clones_cam: &HashMap<usize, CloneMap>,
    options: &TriangulationOptions,
) -> bool {
    // 统计测量数并选出锚点相机（测量最多者；对照 C++ 的 most_meas 循环）
    let mut total_meas = 0usize;
    let mut anchor_cam_id = 0usize;
    let mut most_meas = 0usize;
    for (cam_id, times) in &feat.timestamps {
        total_meas += times.len();
        if times.len() > most_meas {
            anchor_cam_id = *cam_id;
            most_meas = times.len();
        }
    }
    if total_meas == 0 {
        return false;
    }
    feat.anchor_cam_id = anchor_cam_id as i32;
    feat.anchor_clone_timestamp = *feat
        .timestamps
        .get(&anchor_cam_id)
        .and_then(|t| t.last())
        .expect("锚点相机必须有测量");

    // 线性系统（对照 C++ 的 A/b 累加）
    let mut a = Matrix3::<f64>::zeros();
    let mut b = Vector3::<f64>::zeros();

    let anchor_clone = clones_cam
        .get(&anchor_cam_id)
        .and_then(|m| {
            m.iter()
                .find(|(t, _)| t.total_cmp(&feat.anchor_clone_timestamp).is_eq())
        })
        .map(|(_, c)| c)
        .expect("锚点克隆必须存在于 clonesCAM");
    let r_gto_a = &anchor_clone.rot;
    let p_ain_g = &anchor_clone.pos;

    // 逐相机逐测量累加
    for (cam_id, times) in &feat.timestamps {
        let cam_clones = clones_cam
            .get(cam_id)
            .expect("特征测量相机必须存在于 clonesCAM");
        let uvs_norm = feat
            .uvs_norm
            .get(cam_id)
            .expect("uvs_norm 与 timestamps 桶对齐");
        for (m, &t) in times.iter().enumerate() {
            let clone = cam_clones
                .iter()
                .find(|(ct, _)| ct.total_cmp(&t).is_eq())
                .map(|(_, c)| c)
                .expect("特征测量时刻必须存在于 clonesCAM");
            let r_gto_ci = &clone.rot;
            let p_ciin_g = &clone.pos;

            // 相对锚点的位姿（对照 C++：R_AtoCi、p_CiinA）
            let r_a_to_ci = r_gto_ci * r_gto_a.transpose();
            let p_ciin_a = r_gto_a * (p_ciin_g - p_ain_g);

            // 视线方向（归一化坐标，旋转到锚点系并归一化）
            let uv = uvs_norm[m];
            let b_i = r_a_to_ci.transpose() * Vector3::new(f64::from(uv.x), f64::from(uv.y), 1.0);
            let b_i = b_i / b_i.norm();
            let b_perp = skew_x(&b_i);

            // 正规方程累加（对照 C++：Ai = BperpᵀBperp）
            let ai = b_perp.transpose() * b_perp;
            a += ai;
            b += ai * p_ciin_a;
        }
    }

    // 求解 A p = b（对照 C++ 的 colPivHouseholderQr）
    let p_f = a
        .lu()
        .solve(&b)
        .expect("三角化正规方程 A 应可解（秩不足时条件数检查会拦截）");

    // 条件数与深度检查（对照 C++ 的 condA 检查）
    let singular_values = a.svd(true, false).singular_values;
    let cond_a = singular_values[0] / singular_values[singular_values.len() - 1];
    let depth = p_f.z;
    if cond_a.abs() > options.max_cond_number
        || depth < options.min_dist
        || depth > options.max_dist
        || !p_f.norm().is_finite()
    {
        log::debug!(
            "triang 失败: cond={cond_a:.0}(>{}) depth={depth:.3}([{},{}]) 测量{}",
            options.max_cond_number,
            options.min_dist,
            options.max_dist,
            feat.timestamps.values().map(Vec::len).sum::<usize>()
        );
        return false;
    }

    feat.p_FinA = p_f;
    feat.p_FinG = r_gto_a.transpose() * p_f + p_ain_g;
    true
}

/// 高斯牛顿精化特征位置（对照 `FeatureInitializer::single_gaussnewton`）。
///
/// 在锚点系逆深度参数化 `(alpha, beta, rho) = (x/z, y/z, 1/z)` 上做
/// Levenberg–Marquardt 迭代：残差为归一化坐标测量与预测之差，解析雅可比
/// 由锚点→相机旋转/平移给出。收敛后按 `max_baseline`（深度/基线比）与
/// 深度范围检查，失败返回 `false`（`p_FinA/p_FinG` 保持精化前值）。
///
/// # Panics
/// 特征无测量或锚点克隆缺失（调用方组装错误，与 [`single_triangulation`] 同）。
// 与 C++ 1:1 移植的 LM 迭代长函数，拆分会破坏对照可审计性。
#[allow(clippy::too_many_lines, clippy::implicit_hasher)]
#[must_use]
pub fn single_gaussnewton(
    feat: &mut Feature,
    clones_cam: &HashMap<usize, CloneMap>,
    options: &TriangulationOptions,
) -> bool {
    // 进入逆深度（对照 C++ 开头）
    let mut alpha = feat.p_FinA.x / feat.p_FinA.z;
    let mut beta = feat.p_FinA.y / feat.p_FinA.z;
    let mut rho = 1.0 / feat.p_FinA.z;

    // 优化参数（对照 C++ 的 lam/eps/runs）
    let mut lam = options.init_lamda;
    let mut eps = 10_000.0f64;
    let mut runs = 0usize;
    let mut recompute = true;
    let mut hess = Matrix3::<f64>::zeros();
    let mut grad = Vector3::<f64>::zeros();
    let mut cost_old = compute_error(feat, clones_cam, alpha, beta, rho);

    // 锚点位姿（对照 C++ 的 R_GtoA/p_AinG）
    let (r_gto_a, p_ain_g) = {
        let anchor = clones_cam
            .get(&(feat.anchor_cam_id as usize))
            .and_then(|m| {
                m.iter()
                    .find(|(t, _)| t.total_cmp(&feat.anchor_clone_timestamp).is_eq())
            })
            .map(|(_, c)| c)
            .expect("锚点克隆必须存在于 clonesCAM");
        (anchor.rot, anchor.pos)
    };

    while runs < options.max_runs && lam < options.max_lamda && eps > options.min_dx {
        if recompute {
            hess.fill(0.0);
            grad.fill(0.0);
            for (cam_id, times) in &feat.timestamps {
                for (m, t) in times.iter().enumerate() {
                    // 该克隆在全局系位姿（对照 C++ R_GtoCi/p_CiinG）
                    let clone = clones_cam
                        .get(cam_id)
                        .and_then(|cm| cm.iter().find(|(ct, _)| ct.total_cmp(t).is_eq()))
                        .map(|(_, c)| c)
                        .expect("特征测量时刻必须存在于 clonesCAM");
                    let r_gto_ci = clone.rot;
                    let p_ciin_g = clone.pos;
                    // 相对锚点（对照 C++ R_AtoCi/p_CiinA/p_AinCi）
                    let r_a_to_ci = r_gto_ci * r_gto_a.transpose();
                    let p_ciin_a = r_gto_a * (p_ciin_g - p_ain_g);
                    let p_ain_ci = -r_a_to_ci * p_ciin_a;

                    // 中间变量（对照 C++ hi1/hi2/hi3）
                    let hi1 = r_a_to_ci[(0, 0)] * alpha
                        + r_a_to_ci[(0, 1)] * beta
                        + r_a_to_ci[(0, 2)]
                        + rho * p_ain_ci.x;
                    let hi2 = r_a_to_ci[(1, 0)] * alpha
                        + r_a_to_ci[(1, 1)] * beta
                        + r_a_to_ci[(1, 2)]
                        + rho * p_ain_ci.y;
                    let hi3 = r_a_to_ci[(2, 0)] * alpha
                        + r_a_to_ci[(2, 1)] * beta
                        + r_a_to_ci[(2, 2)]
                        + rho * p_ain_ci.z;

                    // 雅可比（2×3，对照 C++ 的 d_z*_d_*）
                    let h = nalgebra::Matrix2x3::<f64>::new(
                        (r_a_to_ci[(0, 0)] * hi3 - hi1 * r_a_to_ci[(2, 0)]) / (hi3 * hi3),
                        (r_a_to_ci[(0, 1)] * hi3 - hi1 * r_a_to_ci[(2, 1)]) / (hi3 * hi3),
                        (p_ain_ci.x * hi3 - hi1 * p_ain_ci.z) / (hi3 * hi3),
                        (r_a_to_ci[(1, 0)] * hi3 - hi2 * r_a_to_ci[(2, 0)]) / (hi3 * hi3),
                        (r_a_to_ci[(1, 1)] * hi3 - hi2 * r_a_to_ci[(2, 1)]) / (hi3 * hi3),
                        (p_ain_ci.y * hi3 - hi2 * p_ain_ci.z) / (hi3 * hi3),
                    );

                    // 残差（对照 C++ z/res；2 维）
                    let z = Vector3::new(hi1 / hi3, hi2 / hi3, 1.0);
                    let uv = feat.uvs_norm[cam_id][m];
                    let res = Vector2::new(f64::from(uv.x) - z.x, f64::from(uv.y) - z.y);

                    grad += h.transpose() * res;
                    hess += h.transpose() * h;
                }
            }
        }

        // LM 迭代（对照 C++：Hess_l = Hess，对角 ×(1+λ)，colPivHouseholderQr 解）
        let mut hess_l = hess;
        for r in 0..3 {
            hess_l[(r, r)] *= 1.0 + lam;
        }
        let dx = hess_l.lu().solve(&grad).expect("LM 正规方程应可解");

        // 代价是否下降（对照 C++ 的 compute_error 检查）
        let cost = compute_error(feat, clones_cam, alpha + dx.x, beta + dx.y, rho + dx.z);

        // 收敛（对照 C++：cost <= cost_old 且相对下降 < min_dcost）
        if cost <= cost_old && (cost_old - cost) / cost_old < options.min_dcost {
            alpha += dx.x;
            beta += dx.y;
            rho += dx.z;
            break;
        }

        if cost <= cost_old {
            recompute = true;
            cost_old = cost;
            alpha += dx.x;
            beta += dx.y;
            rho += dx.z;
            runs += 1;
            lam /= options.lam_mult;
            eps = dx.norm();
        } else {
            recompute = false;
            lam *= options.lam_mult;
        }
    }

    // 还原为标准表示（对照 C++ 末尾）
    let p_fin_a = Vector3::new(alpha / rho, beta / rho, 1.0 / rho);

    // 基线检查（对照 C++ 的 max_baseline 段）
    let mut base_line_max = 0.0f64;
    for (cam_id, times) in &feat.timestamps {
        for t in times {
            let clone = clones_cam
                .get(cam_id)
                .and_then(|cm| cm.iter().find(|(ct, _)| ct.total_cmp(t).is_eq()))
                .map(|(_, c)| c)
                .expect("特征测量时刻必须存在于 clonesCAM");
            let p_ciin_a = r_gto_a * (clone.pos - p_ain_g);
            base_line_max = base_line_max.max(p_ciin_a.xy().norm());
        }
    }

    // 检查（对照 C++：深度/基线比/NaN）
    if p_fin_a.z < options.min_dist
        || p_fin_a.z > options.max_dist
        || (p_fin_a.norm() / base_line_max) > options.max_baseline
        || !p_fin_a.norm().is_finite()
    {
        return false;
    }

    // 写回（对照 C++ 末尾的 p_FinA/p_FinG）
    feat.p_FinA = p_fin_a;
    feat.p_FinG = r_gto_a.transpose() * p_fin_a + p_ain_g;
    true
}

/// 逆深度参数下的重投影代价（对照 `FeatureInitializer::compute_error`）。
fn compute_error(
    feat: &Feature,
    clones_cam: &HashMap<usize, CloneMap>,
    alpha: f64,
    beta: f64,
    rho: f64,
) -> f64 {
    let mut err = 0.0f64;
    let (r_gto_a, p_ain_g) = {
        let anchor = clones_cam
            .get(&(feat.anchor_cam_id as usize))
            .and_then(|m| {
                m.iter()
                    .find(|(t, _)| t.total_cmp(&feat.anchor_clone_timestamp).is_eq())
            })
            .map(|(_, c)| c)
            .expect("锚点克隆必须存在于 clonesCAM");
        (anchor.rot, anchor.pos)
    };
    for (cam_id, times) in &feat.timestamps {
        for (m, t) in times.iter().enumerate() {
            let clone = clones_cam
                .get(cam_id)
                .and_then(|cm| cm.iter().find(|(ct, _)| ct.total_cmp(t).is_eq()))
                .map(|(_, c)| c)
                .expect("特征测量时刻必须存在于 clonesCAM");
            let r_a_to_ci = clone.rot * r_gto_a.transpose();
            let p_ciin_a = r_gto_a * (clone.pos - p_ain_g);
            let p_ain_ci = -r_a_to_ci * p_ciin_a;
            let hi1 = r_a_to_ci[(0, 0)] * alpha
                + r_a_to_ci[(0, 1)] * beta
                + r_a_to_ci[(0, 2)]
                + rho * p_ain_ci.x;
            let hi2 = r_a_to_ci[(1, 0)] * alpha
                + r_a_to_ci[(1, 1)] * beta
                + r_a_to_ci[(1, 2)]
                + rho * p_ain_ci.y;
            let hi3 = r_a_to_ci[(2, 0)] * alpha
                + r_a_to_ci[(2, 1)] * beta
                + r_a_to_ci[(2, 2)]
                + rho * p_ain_ci.z;
            let uv = feat.uvs_norm[cam_id][m];
            let res = Vector2::new(f64::from(uv.x) - hi1 / hi3, f64::from(uv.y) - hi2 / hi3);
            err += res.norm_squared();
        }
    }
    err
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feat::Feature;
    use nalgebra::Vector2;

    fn clone_pose(t: &Vector3<f64>, r: &Matrix3<f64>) -> ClonePose {
        ClonePose { rot: *r, pos: *t }
    }

    /// 合成两视图场景：特征在锚点系 (0.5, 0.2, 3.0)，相机在 +x 方向平移 0.5m。
    fn synthetic_scene() -> (Feature, HashMap<usize, CloneMap>) {
        let p_anchor = Vector3::new(0.5, 0.2, 3.0);
        // 相机 A（锚点）：位姿 = 单位旋转 + 原点
        let r_a = Matrix3::identity();
        let t_a = Vector3::zeros();
        // 相机 B：绕 z 转 0.1 rad + 平移 (0.5, 0, 0)
        let r_b = firefly_vio_types::quat_ops::rot_z(0.1);
        let t_b = Vector3::new(0.5, 0.0, 0.0);

        // 投影：特征全局位置 p_G = R_Aᵀ·p_FinA + t_A；
        // 相机 c 观测 p_c = R_c·(p_G − t_c)（pinhole 归一化）
        let p_g = r_a.transpose() * p_anchor + t_a;
        let project = |r: &Matrix3<f64>, t: &Vector3<f64>| -> Vector2<f32> {
            let p_c = r * (p_g - t);
            Vector2::new((p_c.x / p_c.z) as f32, (p_c.y / p_c.z) as f32)
        };
        let uv_a = project(&r_a, &t_a);
        let uv_b = project(&r_b, &t_b);

        let mut feat = Feature {
            featid: 1,
            to_delete: false,
            timestamps: HashMap::new(),
            uvs: HashMap::new(),
            uvs_norm: HashMap::new(),
            anchor_cam_id: -1,
            anchor_clone_timestamp: 0.0,
            p_FinA: Vector3::zeros(),
            p_FinG: Vector3::zeros(),
        };
        feat.timestamps.insert(0, vec![1.0, 2.0]);
        feat.timestamps.insert(1, vec![1.5]);
        feat.uvs_norm.insert(0, vec![uv_a, uv_a]);
        feat.uvs_norm.insert(1, vec![uv_b]);

        let mut clones: HashMap<usize, CloneMap> = HashMap::new();
        clones.insert(
            0,
            vec![(1.0, clone_pose(&t_a, &r_a)), (2.0, clone_pose(&t_a, &r_a))],
        );
        clones.insert(1, vec![(1.5, clone_pose(&t_b, &r_b))]);
        (feat, clones)
    }

    #[test]
    fn triangulates_synthetic_scene() {
        let (mut feat, clones) = synthetic_scene();
        let ok = single_triangulation(&mut feat, &clones, &TriangulationOptions::default());
        assert!(ok, "三角化应成功");
        // 锚点系深度 ≈ 3.0
        assert!((feat.p_FinA.z - 3.0).abs() < 1e-6, "z = {}", feat.p_FinA.z);
        assert!((feat.p_FinA.x - 0.5).abs() < 1e-6);
        assert!((feat.p_FinA.y - 0.2).abs() < 1e-6);
        // anchor 设置：测量最多的相机 0 + 最后时刻 2.0
        assert_eq!(feat.anchor_cam_id, 0);
        assert!((feat.anchor_clone_timestamp - 2.0).abs() < 1e-12);
    }

    #[test]
    fn fails_on_degenerate_zero_depth() {
        // 所有测量在同一位置（零视差）→ 条件数极大 → 失败
        let (mut feat, mut clones) = synthetic_scene();
        // 让所有克隆位姿相同（视差为零）
        for cam in clones.values_mut() {
            for (_, cp) in cam.iter_mut() {
                cp.rot = Matrix3::identity();
                cp.pos = Vector3::zeros();
            }
        }
        // 重新投影：全部用锚点观测
        feat.uvs_norm
            .insert(0, vec![Vector2::new(0.5 / 3.0, 0.2 / 3.0); 2]);
        feat.uvs_norm
            .insert(1, vec![Vector2::new(0.5 / 3.0, 0.2 / 3.0)]);
        let ok = single_triangulation(&mut feat, &clones, &TriangulationOptions::default());
        assert!(!ok, "零视差应失败");
    }

    #[test]
    fn rejects_out_of_range_depth() {
        let (mut feat, clones) = synthetic_scene();
        let opts = TriangulationOptions {
            min_dist: 10.0,
            ..TriangulationOptions::default()
        };
        let ok = single_triangulation(&mut feat, &clones, &opts);
        assert!(!ok, "深度 3.0 < min_dist 10.0 应失败");
    }

    #[test]
    fn gaussnewton_refines_noisy_initial_solution() {
        let (mut feat, clones) = synthetic_scene();
        // 先线性三角化得到初值
        let ok = single_triangulation(&mut feat, &clones, &TriangulationOptions::default());
        assert!(ok);
        let p_lin = feat.p_FinA;
        // 精化前后代价都应很小（合成场景无噪声），且精化不劣于初值
        let cost_before = compute_error(
            &feat,
            &clones,
            p_lin.x / p_lin.z,
            p_lin.y / p_lin.z,
            1.0 / p_lin.z,
        );
        let ok = single_gaussnewton(&mut feat, &clones, &TriangulationOptions::default());
        assert!(ok);
        let p_ref = feat.p_FinA;
        let cost_after = compute_error(
            &feat,
            &clones,
            p_ref.x / p_ref.z,
            p_ref.y / p_ref.z,
            1.0 / p_ref.z,
        );
        // 收敛后代价不增
        assert!(
            cost_after <= cost_before + 1e-12,
            "cost {cost_after} > {cost_before}"
        );
        // 精化结果仍接近真值 (0.5, 0.2, 3.0)
        assert!((p_ref - Vector3::new(0.5, 0.2, 3.0)).norm() < 1e-6);
        // p_FinG 同步更新
        assert!((feat.p_FinG - feat.p_FinA).norm() < 1e-9);
    }

    #[test]
    fn gaussnewton_rejects_bad_baseline() {
        let (mut feat, clones) = synthetic_scene();
        // 构造退化基线（两相机同位置）：先三角化设置 anchor（会失败），
        // 精化也应失败（baseline=0 被 max_baseline 拒绝）
        let mut clones_degen = clones.clone();
        for cam in clones_degen.values_mut() {
            for (_, cp) in cam.iter_mut() {
                cp.rot = Matrix3::identity();
                cp.pos = Vector3::zeros();
            }
        }
        feat.uvs_norm
            .insert(0, vec![Vector2::new(0.5 / 3.0, 0.2 / 3.0); 2]);
        feat.uvs_norm
            .insert(1, vec![Vector2::new(0.5 / 3.0, 0.2 / 3.0)]);
        // 先三角化设置 anchor 字段（退化 → 返回 false 但 anchor 已写）
        let _ = single_triangulation(&mut feat, &clones_degen, &TriangulationOptions::default());
        let ok = single_gaussnewton(&mut feat, &clones_degen, &TriangulationOptions::default());
        assert!(!ok, "零基线应被 max_baseline 拒绝");
    }
}
