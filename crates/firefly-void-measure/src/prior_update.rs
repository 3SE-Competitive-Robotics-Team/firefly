//! 静态先验平面点-面测量模型（P11.2，LIOP 式紧耦合先验批次）。
//!
//! 与 [`crate::plane_update::DepthMeasurement`] **数学同构**（对照
//! `voxel_map.cpp:414-458` H 组装与 `:447-449` 噪声）：逐点把深度点
//! 变换到世界系，查**静态先验平面容器**（而非在线 VoxelMap），产出
//! 点面残差；H 行 `[⌊p_b×⌋Rᵀn, n]`、零信息行（R=1e12）、卡方门控
//! 全部复用 [`crate::planar::match_plane`] 与 [`MeasurementModel`]。
//!
//! 先验语义对照 `~/Projects/liop_prior/Lidar_IMU_Localization/
//! src/loc/map_location.cpp:1701-1766`：当前帧 surf 点变换到全局系后
//! 对**只读先验 kdtree** 做最近邻 → 平面 → 点面残差，每帧持续执行
//! （`Estimate()` :1043），先验面是"世界系固定参照"——不随估计漂移，
//! 直接给自举在线图一个不可漂移的锚（`docs/void-motion-drift.md` 根因
//! 链第 4 环：地图跟漂 → 残差恒 ≈0 → 无纠正信号；先验面残差与漂移
//! `Δx` 线性相关 → K 拉回）。
//!
//! 平面候选：`PriorPlaneMap::candidates_at`（大平面全局 + 局部根体素
//! 哈希）。匹配判据（径向/噪声/卡方/择优）与在线测量共用
//! [`crate::planar::match_plane`]。

use std::cell::Cell;

use firefly_void_esikf::update::MeasurementModel;
use firefly_void_map::prior_map::PriorPlaneMap;
use firefly_void_map::voxel::transform_point;
use firefly_void_types::state::{DIM_STATE, State};
use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, Vector3};

use crate::options::PriorOptions;
use crate::planar::{PlaneQuery, PlaneResidual, match_plane};

/// 先验测量逐帧诊断（探针，不参与算法决策）。
#[derive(Debug, Clone, Copy, Default)]
pub struct PriorDiag {
    /// 点云总点数。
    pub total: usize,
    /// 无对应平面（无候选 / 径向判据不过）。
    pub no_plane: usize,
    /// 卡方门控拒绝。
    pub chi2_rejected: usize,
    /// 最终有效点（卡方过滤后）。
    pub kept: usize,
    /// 有效点的残差绝对均值（m；`kept` 外的诊断指标——看残差才有物理
    /// 意义，`depth_ok`/`converged` 是误导性指标，见
    /// `docs/void-motion-drift.md`）。
    pub residual_mean: f64,
    /// 有效点噪声方差 `σ²` 均值（m²；`kept` 有效行 R 对角的算术平均——
    /// 门控/`var_scale` 调参的实据：σ² 远大于残差量级说明平面噪声给大，
    /// 更新增益被压得过低）。
    pub sigma_mean: f64,
}

/// 先验平面点-面测量模型。
///
/// 每次构造对应一帧深度点云（与 [`crate::plane_update::DepthMeasurement`]
/// 同构），`residual` 在当前估计位姿下把点变换到全局系、查静态先验
/// 平面集并计算残差。先验面世界系固定，不随估计漂移。
pub struct PriorPlaneMeasurement<'a> {
    /// 静态先验平面容器引用（只读；查询语义见 [`PriorPlaneMap`]）。
    prior_map: &'a PriorPlaneMap,
    /// 深度相机系点云（虚拟针孔系，与
    /// [`DepthMeasurement`](crate::plane_update::DepthMeasurement) 输入同构）。
    points_l: Vec<Vector3<f64>>,
    /// 各点相机系协方差 `Σ_pj`（由 [`crate::noise::DepthNoise`] 预计算）。
    covs: Vec<Matrix3<f64>>,
    /// 深度相机→IMU 外参（虚拟系下为纯旋转，单位阵）。
    ext: Isometry3<f64>,
    opts: PriorOptions,
    /// 最近一次 `residual` 的逐点拒绝统计 + 残差均值（探针）。
    last_diag: Cell<PriorDiag>,
}

impl Clone for PriorPlaneMeasurement<'_> {
    fn clone(&self) -> Self {
        Self {
            prior_map: self.prior_map,
            points_l: self.points_l.clone(),
            covs: self.covs.clone(),
            ext: self.ext,
            opts: self.opts,
            last_diag: Cell::new(self.last_diag.get()),
        }
    }
}

impl<'a> PriorPlaneMeasurement<'a> {
    /// 构造：`points_l` 为深度相机系点云，`covs` 为各点相机系协方差。
    ///
    /// # Panics
    /// `points_l.len() != covs.len()` 时 panic。
    #[must_use]
    pub fn new(
        prior_map: &'a PriorPlaneMap,
        points_l: Vec<Vector3<f64>>,
        covs: Vec<Matrix3<f64>>,
        ext: Isometry3<f64>,
        opts: PriorOptions,
    ) -> Self {
        assert_eq!(points_l.len(), covs.len(), "点与协方差数量必须一致");
        Self {
            prior_map,
            points_l,
            covs,
            ext,
            opts,
            last_diag: Cell::new(PriorDiag::default()),
        }
    }

    /// 最近一次 `residual` 的逐点拒绝统计 + 残差均值（探针）。
    #[must_use]
    pub fn last_diag(&self) -> PriorDiag {
        self.last_diag.get()
    }

    /// 有效点数（上次 `residual` 计算后）。与
    /// [`crate::plane_update::DepthMeasurement::effective_count`] 同构。
    #[must_use]
    pub fn effective_count(&self, x: &State) -> usize {
        let (z, _, r) = self.residual(x);
        z.iter()
            .zip(r.diagonal().iter())
            .filter(|&(_, sig)| *sig < 1e6)
            .count()
    }

    /// 计算残差、H 与 R（对照 `voxel_map.cpp:414-458` 的 H 组装）。
    ///
    /// 固定维度契约：返回行数恒为 `points_l.len()`（与 [`dim`] 一致）。
    /// 无效点（无先验平面/外点）填零信息行（`z=0, H=0, R=1e12`），
    /// 对 KF 不贡献信息。
    #[allow(clippy::many_single_char_names)] // 单字符 `n`/`h`/`r` 为论文 (18)(19) 式记号
    fn build(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        let rot = x.rot.matrix();
        let mut residuals: Vec<PlaneResidual> = Vec::new();
        let mut diag = PriorDiag {
            total: self.points_l.len(),
            ..PriorDiag::default()
        };

        for (p_l, cov) in self.points_l.iter().zip(&self.covs) {
            let p_b = transform_point(&self.ext, p_l);
            let p_w = rot * p_b + x.pos;
            let cov_w = rot * cov * rot.transpose();
            // 平面候选来自静态先验容器（大平面全局 + 局部根体素哈希）；
            // 判据本体 = 在线/先验共用 match_plane（先验 Σ_nq 经
            // var_scale 放大——诚实给大 σ，见 PriorOptions）。
            let candidates = self.prior_map.candidates_at(&p_w);
            match match_plane(
                &candidates,
                &p_w,
                &cov_w,
                self.opts.radius_k,
                self.opts.sigma_num,
                self.opts.var_scale,
            ) {
                PlaneQuery::Matched(n, dis, sig) => residuals.push(Some((p_b, n, dis, sig))),
                PlaneQuery::NoPlane => {
                    diag.no_plane += 1;
                    residuals.push(None);
                }
                PlaneQuery::Chi2Rejected => {
                    diag.chi2_rejected += 1;
                    residuals.push(None);
                }
            }
        }

        diag.kept = residuals.iter().filter(|r| r.is_some()).count();
        // 残差均值与有效行 σ² 均值（kept=0 时为 0；σ² 直接取自 R 对角
        // 值——有效行噪声方差，门控/var_scale 调参实据）
        let kept: Vec<&(Vector3<f64>, Vector3<f64>, f64, f64)> =
            residuals.iter().filter_map(|r| r.as_ref()).collect();
        diag.residual_mean =
            kept.iter().map(|(_, _, dis, _)| dis.abs()).sum::<f64>() / diag.kept.max(1) as f64;
        diag.sigma_mean =
            kept.iter().map(|(_, _, _, sig)| sig).sum::<f64>() / diag.kept.max(1) as f64;
        self.last_diag.set(diag);

        let mut zs = Vec::with_capacity(self.points_l.len());
        let mut rs = Vec::with_capacity(self.points_l.len());
        let zero_info = 1e12;
        for res in &residuals {
            if let Some((_p_b, _n, dis, sig)) = res {
                zs.push(*dis);
                rs.push(*sig);
            } else {
                zs.push(0.0);
                rs.push(zero_info);
            }
        }

        let n_rows = self.points_l.len();
        let z_vec = DVector::from_iterator(n_rows, zs);
        let mut h_mat = DMatrix::zeros(n_rows, DIM_STATE);
        let r_mat = DMatrix::from_diagonal(&DVector::from_iterator(n_rows, rs));
        // `DMatrix::zeros` 每行恒为零向量（零信息行模板，供下方有效行拷贝
        // 后写非零块）。保持取行 0 作模板——该行在有效行循环里总是被
        // 覆盖；勿改成动态取有效行，行模板必须与 H 行结构一致。
        let zero_info_row = h_mat.row_mut(0).clone_owned(); // 零行
        for (i, res) in residuals.iter().enumerate() {
            let Some((p_b, n, _, _)) = res else { continue };
            let p_cross = firefly_void_types::so3::skew(p_b);
            let a_vec = p_cross * rot.transpose() * n;
            let mut row = zero_info_row.clone_owned();
            for k in 0..3 {
                row[k] = a_vec[k];
                row[3 + k] = n[k];
            }
            h_mat.set_row(i, &row);
        }
        (z_vec, h_mat, r_mat)
    }
}

impl MeasurementModel for PriorPlaneMeasurement<'_> {
    fn residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        self.build(x)
    }

    fn dim(&self) -> usize {
        self.points_l.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_void_map::plane::VoxelPlane;
    use firefly_void_types::state::ErrorState;
    use nalgebra::{Rotation3, Translation3, UnitQuaternion};

    fn identity_pose() -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity())
    }

    fn plane(center: Vector3<f64>, normal: Vector3<f64>, radius: f64) -> VoxelPlane {
        VoxelPlane {
            center,
            normal,
            d: -normal.dot(&center),
            plane_var: nalgebra::Matrix6::identity() * 1e-8,
            covariance: Matrix3::identity() * radius,
            radius,
            eigen_min: 1e-6,
            eigen_mid: radius * radius,
            eigen_max: radius * radius,
            points_count: 50,
            is_plane: true,
            is_mature: true,
        }
    }

    fn covs_for(n: usize) -> Vec<Matrix3<f64>> {
        vec![Matrix3::identity() * 1e-8; n]
    }

    /// 均匀网格点（贴在给定平面 center/normal 上，由两切向参数化）。
    fn points_on_plane(n: usize, center: Vector3<f64>, normal: Vector3<f64>) -> Vec<Vector3<f64>> {
        // 两个切向量
        let t1 = if normal[0].abs() < 0.9 {
            normal.cross(&Vector3::new(1.0, 0.0, 0.0))
        } else {
            normal.cross(&Vector3::new(0.0, 1.0, 0.0))
        }
        .normalize();
        let t2 = normal.cross(&t1).normalize();
        let side = (n as f64).sqrt().ceil() as usize;
        (0..n)
            .map(|i| {
                let a = -0.2 + (i % side) as f64 * 0.4 / side as f64;
                let b = -0.2 + (i / side) as f64 * 0.4 / side as f64;
                center + t1 * a + t2 * b
            })
            .collect()
    }

    /// 数值雅可比（中心差分）：对 19 维误差状态逐分量扰动。
    fn numeric_h<F: Fn(&State) -> DVector<f64>>(x: &State, f: F, eps: f64) -> DMatrix<f64> {
        let z0 = f(x);
        let m = z0.len();
        let mut h = DMatrix::zeros(m, DIM_STATE);
        for j in 0..DIM_STATE {
            let mut dp = ErrorState::zeros();
            dp[j] = eps;
            let zp = f(&x.boxplus(&dp));
            dp[j] = -eps;
            let zm = f(&x.boxplus(&dp));
            h.set_column(j, &((&zp - &zm) / (2.0 * eps)));
        }
        h
    }

    fn model_for(
        prior: &PriorPlaneMap,
        pts: Vec<Vector3<f64>>,
        covs: Vec<Matrix3<f64>>,
    ) -> PriorPlaneMeasurement<'_> {
        PriorPlaneMeasurement::new(prior, pts, covs, identity_pose(), PriorOptions::default())
    }

    fn residual_valid_z(model: &PriorPlaneMeasurement<'_>, x: &State) -> (Vec<f64>, f64) {
        let (z, _, r) = model.residual(x);
        let kept: Vec<f64> = z
            .iter()
            .zip(r.diagonal().iter())
            .filter(|&(_, sig)| *sig < 1e6)
            .map(|(v, _)| *v)
            .collect();
        let mean = kept.iter().sum::<f64>() / kept.len().max(1) as f64;
        (kept, mean)
    }

    fn drifted_state(pos: Vector3<f64>) -> State {
        use firefly_void_types::state::StateCovariance;
        // 位置协方差 σ=1cm（≈漂移量级：滤波器被地图跟漂拖走时置信度仍
        // 高但已偏——测量须有足够增益拉回）
        let mut cov = StateCovariance::identity() * 1e-4;
        cov[(6, 6)] = 1e-5;
        for i in 10..19 {
            cov[(i, i)] = 1e-5;
        }
        State {
            pos,
            cov,
            ..State::default()
        }
    }

    #[test]
    fn residual_zero_at_truth_pose() {
        let prior = PriorPlaneMap::from_planes(vec![plane(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0),
            0.4,
        )]);
        let pts = points_on_plane(64, Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0));
        let model = model_for(&prior, pts, covs_for(64));
        let (z_vec, h_mat, r_mat) = model.residual(&State::default());
        assert_eq!(z_vec.len(), 64);
        let valid: Vec<f64> = z_vec
            .iter()
            .zip(r_mat.diagonal().iter())
            .filter(|&(_, sig)| *sig < 1e6)
            .map(|(v, _)| *v)
            .collect();
        assert!(!valid.is_empty());
        assert!(valid.iter().all(|v| v.abs() < 1e-6), "残差应≈0: {valid:?}");
        assert_eq!(h_mat.nrows(), 64);
        assert_eq!(h_mat.ncols(), DIM_STATE);
    }

    #[test]
    fn jacobian_matches_finite_difference() {
        let prior = PriorPlaneMap::from_planes(vec![plane(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0),
            0.4,
        )]);
        let pts = points_on_plane(64, Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0));
        let model = model_for(&prior, pts, covs_for(64));
        // 位姿扰动：x 方向偏 5cm、绕 y 转 3°（点变换后脱离平面，残差/H 非平凡）
        let x = State {
            pos: Vector3::new(0.05, -0.02, 0.01),
            rot: Rotation3::from_axis_angle(&Vector3::y_axis(), 0.03),
            ..State::default()
        };
        let (_, h_mat, _) = model.residual(&x);
        let f = |s: &State| -> DVector<f64> {
            let (z, _, _) = model.residual(s);
            z
        };
        let h_num = numeric_h(&x, f, 1e-6);
        let err = (&h_mat - &h_num).norm() / (h_mat.norm() + 1e-12);
        assert!(err < 1e-6, "解析 H 与数值 H 相对误差 {err}");
    }

    /// 关键合成测试：估计沿**竖直墙法向**漂移 Δx 后，深度点对固定
    /// 先验墙面的残差 ≠ 0（在线自举图会跟漂使残差≈0），ESIKF 顺序更新
    /// 把状态拉回——直接验证"固定先验面根治跟漂"的机制。
    #[test]
    fn esikf_pulls_drift_back_to_prior_wall() {
        use firefly_void_esikf::update::EskfUpdater;

        // 竖直墙 x=1（法向 +x）：真实场景侧向约束，主漂移方向 x/y
        let wall_center = Vector3::new(1.0, 0.0, 0.5);
        let wall_normal = Vector3::new(1.0, 0.0, 0.0);
        let prior = PriorPlaneMap::from_planes(vec![plane(wall_center, wall_normal, 0.5)]);
        let pts = points_on_plane(100, wall_center, wall_normal);
        let model = model_for(&prior, pts, covs_for(100));

        // 估计位置向 −x 漂移 1cm（点离开墙面 1cm，门控内：
        // σ = √(0.001+…)≈3cm，3σ 门 ≈ 9cm）
        let mut drifted = drifted_state(Vector3::new(-0.01, 0.0, 0.0));

        // 漂移状态下残差 ≈ −1cm ≠ 0（先验墙固定 → 有真实信号）
        let (kept, mean_drift) = residual_valid_z(&model, &drifted);
        assert!(!kept.is_empty(), "漂移状态下应有有效先验匹配");
        assert!(
            (mean_drift + 0.01).abs() < 0.005,
            "墙面固定：漂移 Δx=−1cm 应给残差≈−1cm，实测 {mean_drift}"
        );

        // ESIKF 顺序更新拉回 x→0（updater 消费 model 的 clone，原 model
        // 保留做拉回后残差校验）。断言 5mm：漂移 1cm 拉回 >50% 即机制
        // 验证（KF 增益取决于先验/测量协方差比，非全拉）
        let mut updater = EskfUpdater::new(model.clone(), 10, (1e-6, 1e-6));
        let (iters, _) = updater.update(&mut drifted).unwrap();
        assert!(iters >= 1);
        assert!(
            drifted.pos[0].abs() < 5e-3,
            "ESIKF 应把 x 大幅拉回 0：估计 {}",
            drifted.pos[0]
        );
        // 拉回后残差显著下降（≈0 到门控内）
        let (kept, mean_final) = residual_valid_z(&model, &drifted);
        assert!(!kept.is_empty());
        assert!(
            mean_final.abs() < 5e-3,
            "拉回后残差应显著下降：{mean_final}"
        );
    }

    /// 地面（法向 +z）漂移同样拉回（另一个自由度）。
    #[test]
    fn esikf_pulls_drift_back_to_prior_ground() {
        use firefly_void_esikf::update::EskfUpdater;

        let prior = PriorPlaneMap::from_planes(vec![plane(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0),
            0.5,
        )]);
        let pts = points_on_plane(
            100,
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0),
        );
        let model = model_for(&prior, pts, covs_for(100));

        // 估计位置 z 漂移 −1cm（点穿到面下方）
        let mut drifted = drifted_state(Vector3::new(0.0, 0.0, -0.01));

        let (kept, mean_drift) = residual_valid_z(&model, &drifted);
        assert!(!kept.is_empty());
        assert!(
            (mean_drift + 0.01).abs() < 0.005,
            "地面固定：漂移 Δz=−1cm 应给残差≈−1cm，实测 {mean_drift}"
        );

        let mut updater = EskfUpdater::new(model, 10, (1e-6, 1e-6));
        let _ = updater.update(&mut drifted).unwrap();
        assert!(
            drifted.pos[2].abs() < 5e-3,
            "ESIKF 应把 z 大幅拉回 0：估计 {}",
            drifted.pos[2]
        );
    }

    /// 门控行为：点离先验面太远（> `sigma_num`·√R）时被拒绝，不产生拉偏。
    #[test]
    fn chi2_rejects_distant_point() {
        let prior = PriorPlaneMap::from_planes(vec![plane(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0),
            0.4,
        )]);
        // 点在世界系 z=0.7（距面 0.3m > 3σ≈0.095m，但仍在面覆盖盒内）
        let pts = vec![Vector3::new(0.0, 0.0, 0.7)];
        let model = model_for(&prior, pts, covs_for(1));
        let (z, _, r) = model.residual(&State::default());
        assert!(r[(0, 0)] > 1e6, "远处点应零信息：r={}", r[(0, 0)]);
        assert!(z[0].abs() < 1e-12, "无效行残差应为 0：z[0]={}", z[0]);
        let d = model.last_diag();
        assert_eq!(d.kept, 0);
        assert_eq!(d.chi2_rejected, 1);
    }

    /// 真值位姿下全部匹配、残差≈0（诊断字段自洽）。
    #[test]
    fn prior_diag_reports_kept_and_residual() {
        let prior = PriorPlaneMap::from_planes(vec![plane(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0),
            0.4,
        )]);
        let pts = points_on_plane(64, Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0));
        let model = model_for(&prior, pts, covs_for(64));
        let _ = model.residual(&State::default());
        let d = model.last_diag();
        assert_eq!(d.kept, 64);
        assert_eq!(d.no_plane, 0);
        assert_eq!(d.chi2_rejected, 0);
        assert!(d.residual_mean < 1e-6);
        // σ² = J_nq·Σ_nq·J_nqᵀ + nᵀ·Σ_pj·n + 0.001 ≈ 0.001（点贴面、
        // 平面 Σ_nq 极小）；残差/σ 比值应远小于门控（1e-3 / 3e-2 量级）
        assert!(
            (d.sigma_mean - 0.001).abs() < 1e-3,
            "贴面点 σ² ≈ 0.001：{}",
            d.sigma_mean
        );
    }
}
