//! 特征三角化（对照 `OpenVINS` `ov_core/src/feat/FeatureInitializer.cpp` 的
//! `single_triangulation` 与 `single_gaussnewton`）。
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
    /// 条件数上限（`max_cond_number = 100000`）。低视差单目下 DLT 条件数偏大；
    /// 维持 1e5：放宽至 1e6 后拒绝主因转为深度/精化检查，三角化存活无改善。
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
    /// 深度/基线比上限（OpenVINS 默认 40；本场景 D430 基线 0.05m + 特征
    /// 2~6m（z=1.5 悬停时最近可见地面 ~2.6m、柱 ~3.6m），40 只允许 ≤2m——
    /// 目的地悬停全部特征被拒 → 视觉更新死亡 → IMU 漂移。放宽到 120
    /// （≤6m）后近柱/地面可用，视觉能拉回机动后的姿态误差。
    pub max_baseline: f64,
}

impl Default for TriangulationOptions {
    fn default() -> Self {
        Self {
            max_cond_number: 100_000.0,
            min_dist: 0.10,
            max_dist: 60.0,
            refine_features: true,
            max_runs: 5,
            init_lamda: 1e-3,
            max_lamda: 1e10,
            min_dx: 1e-6,
            min_dcost: 1e-6,
            lam_mult: 10.0,
            max_baseline: 120.0,
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

    // 求解 A p = b（对照 C++ 的 colPivHouseholderQr：C++ 对奇异阵返回最小二乘
    // 解再由条件数/深度检查拒绝，不会失败；Rust `lu().solve()` 遇奇异返回
    // None——按三角化失败处理，交由调用方删除该特征）
    let Some(p_f) = a.lu().solve(&b) else {
        log::debug!("triang 失败: 正规方程奇异不可解");
        return false;
    };

    // 条件数与深度检查（对照 C++ 的 condA 检查）
    let singular_values = a.svd(true, false).singular_values;
    let cond_a = singular_values[0] / singular_values[singular_values.len() - 1];
    let depth = p_f.z;
    if cond_a.abs() > options.max_cond_number
        || depth < options.min_dist
        || depth > options.max_dist
        || !p_f.norm().is_finite()
    {
        let total_meas: usize = feat.timestamps.values().map(Vec::len).sum();
        let per_cam: Vec<String> = feat
            .timestamps
            .iter()
            .map(|(c, t)| format!("{c}:{}", t.len()))
            .collect();
        log::debug!(
            "triang 失败: cond={cond_a:.0}(>{}) depth={depth:.3} meas={} 相机[{}]",
            options.max_cond_number,
            total_meas,
            per_cam.join(",")
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
    // C++：HouseholderQR(p_FinA).householderQ() 列 1..3 张成与视线方向
    // （单位向量 ê=p_FinA/|p_FinA|）垂直的平面，基线取克隆位姿在该平面上
    // 的投影长度——衡量对逆深度可观的侧向视差。此处用闭式等价：
    //   |v⊥|² = |v|² − (vᵀê)²
    // 与取正交基列 1..3 投影完全一致（避免对 3×1 输入构造紧凑 QR）。
    // 必须取 ⊥ê 分量而非 xy().norm()：前飞场景克隆位移沿视线方向时
    // xy 分量 ≈0，会把基线误判为 0 而全部拒绝。
    let e_hat = p_fin_a / p_fin_a.norm();
    let mut base_line_max = 0.0f64;
    for (cam_id, times) in &feat.timestamps {
        for t in times {
            let clone = clones_cam
                .get(cam_id)
                .and_then(|cm| cm.iter().find(|(ct, _)| ct.total_cmp(t).is_eq()))
                .map(|(_, c)| c)
                .expect("特征测量时刻必须存在于 clonesCAM");
            let p_ciin_a = r_gto_a * (clone.pos - p_ain_g);
            let perp_sq = p_ciin_a.norm_squared() - p_ciin_a.dot(&e_hat).powi(2);
            base_line_max = base_line_max.max(perp_sq.max(0.0).sqrt());
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

/// 复刻 e2e 走廊几何的解析输入三角化验证：
/// 12 个克隆沿 +x 匀速（1.2s），双目杆臂 ±0.025，特征为已知走廊点——
/// 输入 `uvs_norm` 由真值投影精确生成（无跟踪噪声），
/// 若失败则 `single_triangulation` 数学存在约定缺陷。
#[test]
fn single_triangulation_recovers_corridor_dot_exact_inputs() {
    // 与 e2e 一致的外参与杆臂
    let r_ito_c = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
    let levers = [
        (0usize, Vector3::new(-0.025, 0.0, 0.0)), // 左目（p_IinC = R·(0,+0.025)）
        (1usize, Vector3::new(0.025, 0.0, 0.0)),
    ];

    // 12 克隆：t=0.2..1.3，位置 (t,0,1)，单位姿态
    let mut clones_cam: HashMap<usize, CloneMap> = HashMap::new();
    for (cam_id, lever_ic) in levers {
        let mut cm: CloneMap = Vec::new();
        for i in 0..12 {
            let t = 0.2 + 0.1 * f64::from(i as u32);
            // p_CiinG = p_IinG − R_GtoCiᵀ·p_IinC；R_GtoCi=R_ItoC（单位姿态）
            let lever_body = r_ito_c.transpose() * lever_ic;
            let pos = Vector3::new(t, 0.0, 1.0) - lever_body;
            cm.push((t, ClonePose { rot: r_ito_c, pos }));
        }
        clones_cam.insert(cam_id, cm);
    }

    // 真值点：走廊内典型位置
    let truth = Vector3::new(6.5, -2.5, 0.9);

    // 构造测量：uvs_norm = 真值投影（y-down 相机系）
    let mut feat = Feature {
        featid: 999,
        ..Feature::default()
    };
    for (cam_id, lever_ic) in levers {
        let lever_body = r_ito_c.transpose() * lever_ic;
        for i in 0..12 {
            let t = 0.2 + 0.1 * f64::from(i as u32);
            let v = truth - (Vector3::new(t, 0.0, 1.0) - lever_body);
            let pc = r_ito_c * v;
            assert!(pc.z > 0.5, "测试几何应保证正深度");
            feat.timestamps.entry(cam_id).or_default().push(t);
            feat.uvs_norm
                .entry(cam_id)
                .or_default()
                .push(nalgebra::Vector2::new(
                    (pc.x / pc.z) as f32,
                    (pc.y / pc.z) as f32,
                ));
        }
    }

    let opts = TriangulationOptions {
        max_cond_number: 100_000.0,
        min_dist: 0.10,
        max_dist: 60.0,
        refine_features: true,
        max_runs: 5,
        init_lamda: 1e-3,
        lam_mult: 10.0,
        min_dcost: 1e-8,
        max_baseline: 40.0,
        max_lamda: 1e10,
        min_dx: 1e-6,
    };

    let ok_triang = single_triangulation(&mut feat, &clones_cam, &opts);
    println!(
        "single_triangulation: ok={ok_triang} depth={:.3}",
        feat.p_FinA.z
    );
    assert!(ok_triang, "解析输入下三角化不应失败");

    let ok_gn = single_gaussnewton(&mut feat, &clones_cam, &opts);
    assert!(ok_gn, "高斯牛顿精化不应失败");

    let err = (feat.p_FinG - truth).norm();
    println!(
        "恢复位置=({:.4},{:.4},{:.4}) 真值={truth} 误差={err:.4}m",
        feat.p_FinG.x, feat.p_FinG.y, feat.p_FinG.z
    );
    assert!(err < 0.05, "解析输入下应恢复真值位置: {err:.4}m");
}

#[cfg(test)]
mod pure_translation_tests {
    use super::*;
    use std::collections::HashMap;

    fn clone_pose2(t: &Vector3<f64>, r: &Matrix3<f64>) -> ClonePose {
        ClonePose { rot: *r, pos: *t }
    }

    /// 相机沿 +x 纯平移 12 帧（模拟前向飞行），特征在前方 (10, 2, 0)。
    /// 对照 e2e：走廊点云在前方、前向运动——验证纯平移视差下三角化。
    #[test]
    fn triangulates_pure_translation_forward() {
        let p_g = Vector3::new(10.0, 2.0, 3.0);
        let mut timestamps = Vec::new();
        let mut uvs_norm = Vec::new();
        let mut cam_clones = Vec::new();
        let r = Matrix3::identity();
        for i in 0..12 {
            let t = Vector3::new(f64::from(i) * 0.1, 0.0, 1.0);
            let p_c = r * (p_g - t);
            let uv = Vector2::new((p_c.x / p_c.z) as f32, (p_c.y / p_c.z) as f32);
            timestamps.push(f64::from(i) * 0.1);
            uvs_norm.push(uv);
            cam_clones.push((f64::from(i) * 0.1, clone_pose2(&t, &r)));
        }
        let mut feat = Feature {
            featid: 1,
            to_delete: false,
            timestamps: HashMap::from([(0usize, timestamps)]),
            uvs: HashMap::new(),
            uvs_norm: HashMap::from([(0usize, uvs_norm)]),
            anchor_cam_id: -1,
            anchor_clone_timestamp: 0.0,
            p_FinA: Vector3::zeros(),
            p_FinG: Vector3::zeros(),
        };
        let mut clones = HashMap::new();
        clones.insert(0usize, cam_clones);
        let ok = single_triangulation(&mut feat, &clones, &TriangulationOptions::default());
        println!("纯平移前向: ok={ok} p_FinA={:?}", feat.p_FinA);
        assert!(ok, "纯平移前向三角化失败");
        // 锚点 = 相机 0 最后时刻（t=1.1s、位姿 (1.1,0,1)）→ p_FinA = 特征相对锚点
        // 特征 (10,2,3) − 锚点 (1.1,0,1) = (8.9, 2, 2)
        assert!((feat.p_FinA.x - 8.9).abs() < 1e-3, "x = {}", feat.p_FinA.x);
        assert!((feat.p_FinA.y - 2.0).abs() < 1e-3, "y = {}", feat.p_FinA.y);
        assert!((feat.p_FinA.z - 2.0).abs() < 1e-3, "z = {}", feat.p_FinA.z);
    }
}

#[cfg(test)]
mod e2e_style_tests {
    use super::*;
    use std::collections::HashMap;
    /// e2e 风格：完整几何复刻（`r_ito_c` 外参 + 前向平移 12 帧 + 双目基线）。
    /// 特征 G 系 (26, 2.5, 0.5)，相机 body 位置 (10+i*0.1, 0, 1)，姿态单位阵；
    /// 相机系 `p_c = r_ito_c·(p_b − t_cam_body)`，克隆位姿按 `build_clones_cam` 公式。
    #[test]
    fn triangulates_e2e_style_far_feature() {
        let r_ito_c = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        let p_g = Vector3::new(26.0, 2.5, 0.5);
        let mut clones = HashMap::new();
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
        for cam in 0..2 {
            let mut cam_clones = Vec::new();
            let mut ts = Vec::new();
            let mut uvns = Vec::new();
            for i in 0..12 {
                let t_body = Vector3::new(10.0 + f64::from(i) * 0.1, 0.0, 1.0);
                // 双目偏移：p_IinC = r_ito_c·(0, ±0.025, 0)
                let p_iin_c =
                    r_ito_c * Vector3::new(0.0, if cam == 0 { 0.025 } else { -0.025 }, 0.0);
                let r_gto_ci = r_ito_c;
                let p_ciin_g = t_body - r_gto_ci.transpose() * p_iin_c;
                cam_clones.push((
                    f64::from(i) * 0.1,
                    ClonePose {
                        rot: r_gto_ci,
                        pos: p_ciin_g,
                    },
                ));
                // 投影（相机系，z 前向）
                let p_c = r_ito_c * (p_g - t_body);
                let uv = Vector2::new((p_c.x / p_c.z) as f32, (p_c.y / p_c.z) as f32);
                ts.push(f64::from(i) * 0.1);
                uvns.push(uv);
            }
            clones.insert(cam, cam_clones);
            feat.timestamps.insert(cam, ts);
            feat.uvs_norm.insert(cam, uvns);
        }
        let ok = single_triangulation(&mut feat, &clones, &TriangulationOptions::default());
        println!("e2e 风格远端: ok={ok} p_FinA={:?}", feat.p_FinA);
        assert!(ok, "e2e 风格远端三角化失败");
    }
}
