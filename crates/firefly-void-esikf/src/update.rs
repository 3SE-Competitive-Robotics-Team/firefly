//! 顺序更新框架（论文第 IV-D 节 Algorithm 1 主体结构）。
//!
//! 论文 (11) 式的迭代更新：
//! `K = (HᵀR⁻¹H + P⁻¹)⁻¹ HᵀR⁻¹`，
//! `x^{κ+1} = x^κ ⊞ (−Kz^κ − (I − KH)(x^κ ⊟ x̂))`，
//! 收敛判据 `‖x^{κ+1} ⊟ x^κ‖ < ε`（Algorithm 1 第 10 行）。
//!
//! 测量模型经 [`MeasurementModel`] trait 注入（依赖倒置）：esikf 不依赖
//! 测量 crate，具体残差/H 由 `firefly-void-measure` 在 P3 实现。
//! 官方对照：`src/voxel_map.cpp:461-500`（激光点-平面更新）与
//! `src/vio.cpp:1648-1688`（视觉更新，含误差下降判据）。
//!
//! 数学/几何转录代码中单字符标量（`x`,`z`,`h`,`r`,`k`）与论文记号
//! （`ε`,`κ`,`δx`）属于固有风格，予以模块级允许（对照
//! `firefly-vio-core/src/cam.rs` 的既有先例）。
#![allow(clippy::many_single_char_names)]
use firefly_error::{Error, ErrorKind};
use firefly_void_types::state::{DIM_STATE, State, StateCovariance};
use nalgebra::{DMatrix, DVector, SMatrix};

/// 19×19 信息矩阵（更新中间量）。
pub type InfoMatrix = SMatrix<f64, DIM_STATE, DIM_STATE>;

/// 测量模型 trait：提供残差、雅可比与测量噪声。
///
/// 输出约定：
/// - `residual` 返回 `(z, H, R)`：`z^κ = h(x̂^κ, 0) − y`（论文 (9) 式，
///   `m` 维测量残差），`H` 为对误差状态 `δx` 的雅可比（`m × 19`），
///   `R` 为测量噪声协方差（`m × m`）；
/// - `dim`：本测量批次维度 `m`（可为 0，表示无有效测量）。
pub trait MeasurementModel {
    /// 计算残差、雅可比与测量噪声（在 `x` 处线性化）。
    fn residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>);
    /// 测量维度 `m`。
    fn dim(&self) -> usize;

    /// 聚合信息量（对照官方视觉更新 `H_T_H = H_sub_T·H_sub`、
    /// `HTz = H_sub_T·z`，`vio.cpp:1660-1662`）。
    ///
    /// 返回 `(HᵀR⁻¹H, HᵀR⁻¹z, zᵀR⁻¹z)`：信息矩阵 `S ∈ 19×19`、信息向量
    /// `b ∈ 19`、加权残差标量 `e`。默认实现用 `residual` 的逐行累加——
    /// `m` 大时（视觉逐像素 67k 行）默认实现避免构造 `m×19` 矩阵，
    /// 直接把 `H_iᵀR_ii⁻¹H_i` 累加进 `S`。
    ///
    /// ESIKF 迭代（`vio.cpp:1666-1669`）只需 `S`、`b`、`e` 与先验协方差
    /// （`K·H = (S+P⁻¹)⁻¹·S`、`K·z = (S+P⁻¹)⁻¹·b`），无需显式 `H`。
    #[must_use]
    fn information(&self, x: &State) -> (InfoMatrix, DVector<f64>, f64) {
        let (z, h, r) = self.residual(x);
        let mut s = InfoMatrix::zeros();
        let mut b = DVector::zeros(DIM_STATE);
        let mut e = 0.0;
        for i in 0..z.len() {
            let r_inv = if r[(i, i)] > 0.0 && r[(i, i)].is_finite() {
                1.0 / r[(i, i)]
            } else {
                0.0 // 零信息行（R=1e12 占位）贡献为 0
            };
            if r_inv == 0.0 {
                continue;
            }
            let hi = h.row(i);
            // S += hiᵀ·r_inv·hi（19×19 外积累加）
            for a in 0..DIM_STATE {
                let hia = hi[a];
                for bb in 0..DIM_STATE {
                    s[(a, bb)] += hia * hi[bb] * r_inv;
                }
            }
            let zi = z[i];
            for a in 0..DIM_STATE {
                b[a] += hi[a] * zi * r_inv;
            }
            e += zi * zi * r_inv;
        }
        (s, b, e)
    }
}

/// 空测量模型：`dim()=0`，顺序更新直接早退（占位/禁用测量批次用）。
impl MeasurementModel for () {
    fn residual(&self, _x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        (
            DVector::zeros(0),
            DMatrix::zeros(0, 0),
            DMatrix::zeros(0, 0),
        )
    }

    fn dim(&self) -> usize {
        0
    }
}

/// 顺序更新器：对给定先验执行迭代更新（Algorithm 1 的单个 update 步骤）。
#[derive(Debug, Clone)]
pub struct EskfUpdater<M> {
    model: M,
    /// 最大迭代次数（对照 `max_iterations`，`voxel_map.cpp:372` 默认 5）。
    max_iterations: usize,
    /// 旋转增量收敛阈值（rad，官方 `voxel_map.cpp:477` 深度 / `vio.cpp:1675` 视觉）。
    rot_eps: f64,
    /// 平移增量收敛阈值（m，官方同上）。
    pos_eps: f64,
    /// 发散保护：残差范数相对先验残差范数的放大倍数上限。
    divergence_factor: f64,
    /// 误差下降判据（对照官方视觉更新 `vio.cpp:1643-1672`：残差不降则
    /// 回退到上一接受状态并终止；深度更新官方无此判据，保持关闭）。
    accept_on_error_descent: bool,
}

/// 官方深度更新收敛阈值（`voxel_map.cpp:477`）：
/// `rot_add.norm()*57.3<0.01 && t_add.norm()*100<0.015`。
#[must_use]
pub const fn depth_convergence() -> (f64, f64) {
    (0.01 / 57.3, 0.015 / 100.0) // 1.745e-4 rad, 1.5e-4 m
}

/// 官方视觉更新收敛阈值（`vio.cpp:1675`）：
/// `rot_add.norm()*57.3<0.001 && t_add.norm()*100<0.001`。
#[must_use]
pub const fn visual_convergence() -> (f64, f64) {
    (0.001 / 57.3, 0.001 / 100.0) // 1.745e-5 rad, 1.5e-5 m
}

impl<M: MeasurementModel> EskfUpdater<M> {
    /// 构造：`max_iterations` 与收敛阈值 `(rot_eps, pos_eps)` 由调用方给定。
    #[must_use]
    pub fn new(model: M, max_iterations: usize, convergence: (f64, f64)) -> Self {
        Self {
            model,
            max_iterations,
            rot_eps: convergence.0,
            pos_eps: convergence.1,
            divergence_factor: 1e6,
            accept_on_error_descent: false,
        }
    }

    /// 设置发散保护倍数（残差放大超过该倍数即判发散）。
    #[must_use]
    pub fn with_divergence_factor(mut self, factor: f64) -> Self {
        self.divergence_factor = factor;
        self
    }

    /// 开启误差下降判据（对照官方视觉更新 `vio.cpp:1643`）。
    #[must_use]
    pub fn with_error_descent_acceptance(mut self) -> Self {
        self.accept_on_error_descent = true;
        self
    }

    /// 顺序更新：`state` 为传播先验（含协方差），就地更新到后验。
    ///
    /// 迭代 `x^{κ+1} = x^κ ⊞ (−Kz^κ − (I − KH)(x^κ ⊟ x̂))` 直至收敛或
    /// 达到迭代上限；返回 `(迭代次数, 是否收敛)`。协方差更新
    /// `P = (I − KH)P̂` 用最后一次迭代的 K/H（对照 `voxel_map.cpp:489`
    /// 与 `vio.cpp:800`）。
    ///
    /// 信息量经 [`MeasurementModel::information`] 聚合为 `(S, b, e)`（对照
    /// 官方 `H_T_H`/`HTz`，`vio.cpp:1660-1662`），迭代 `Kz = (S+P⁻¹)⁻¹b`、
    /// `KH = (S+P⁻¹)⁻¹S`，不构造 `m×19` 矩阵——视觉逐像素 67k 行下
    /// 单帧迭代从 ~2s 降到 ~20ms。
    ///
    /// # Errors
    /// - 测量维度与残差/雅可比不一致（`InvalidArgument`）；
    /// - 协方差或测量噪声不可逆（`Internal`）；
    /// - 残差 NaN 或迭代发散（`Convergence`）。
    #[fastrace::trace]
    pub fn update(&mut self, state: &mut State) -> Result<(usize, bool), Error> {
        let dim = self.model.dim();
        if dim == 0 {
            return Ok((0, false));
        }
        let prior = *state;
        let (_, _, prior_e) = self.model.information(&prior);
        let prior_residual_norm = prior_e.sqrt();

        let p_inv = prior.cov.try_inverse().ok_or_else(|| {
            Error::new(ErrorKind::Internal, "先验协方差不可逆").with_context("state", "prior cov")
        })?;

        let mut x = prior;
        let mut last_s = InfoMatrix::zeros();
        let mut converged_at = None;
        // 误差下降判据（对照官方视觉 `vio.cpp:1643-1672`）：上一接受
        // 迭代的残差与状态；残差上升时回退到上一接受状态并终止。
        let mut last_error = f64::INFINITY;
        let mut last_accepted = prior;

        for iter in 0..self.max_iterations {
            let (s, b, e) = self.model.information(&x);
            let cur_norm = e.sqrt();
            if cur_norm.is_nan() {
                return Err(
                    Error::new(ErrorKind::Convergence, "残差含 NaN").with_context("iter", iter)
                );
            }
            if self.accept_on_error_descent && cur_norm > last_error {
                // 残差上升：回退到上一接受状态并终止（官方 `vio.cpp:1666-1669`）
                x = last_accepted;
                if log::log_enabled!(log::Level::Info) {
                    log::info!(
                        "esikf-iter iter={iter} norm={cur_norm:.6} prior={prior_residual_norm:.6} \
                         d_rot={:.3e} d_pos={:.3e} REJECT(上升回退)",
                        x.boxminus(&last_accepted).fixed_rows::<3>(0).norm(),
                        x.boxminus(&last_accepted).fixed_rows::<3>(3).norm()
                    );
                }
                break;
            }
            if prior_residual_norm > 0.0 && cur_norm > self.divergence_factor * prior_residual_norm
            {
                return Err(Error::new(ErrorKind::Convergence, "残差发散")
                    .with_context("prior_norm", prior_residual_norm)
                    .with_context("cur_norm", cur_norm));
            }
            if self.accept_on_error_descent {
                last_error = cur_norm;
                last_accepted = x;
            }

            // 论文 (11) 式聚合形式（对照官方 `vio.cpp:1666-1669`）：
            //   Kz = (S + P⁻¹)⁻¹·b
            //   KH = (S + P⁻¹)⁻¹·S
            // 迭代增量 = −Kz + (I − KH)·(x ⊟ x̂)
            let s_pinv = (s + p_inv)
                .try_inverse()
                .ok_or_else(|| Error::new(ErrorKind::Internal, "信息矩阵不可逆"))?;
            let kz = s_pinv * &b;
            let kh = s_pinv * s;
            let delta_prior = x.boxminus(&prior);
            let correction = -kz - (InfoMatrix::identity() - kh) * delta_prior;
            let next = x.boxplus(&correction);

            last_s = s;
            let step = next.boxminus(&x);
            // 收敛判据对照 `voxel_map.cpp:477`：只查旋转+平移 6 维
            // （`rot_add.norm()*57.3<0.01 && t_add.norm()*100<0.015`）。
            // 速度/零偏等其余分量由测量持续纠正，不参与收敛判定。
            let rot_n = step.fixed_rows::<3>(0).norm();
            let pos_n = step.fixed_rows::<3>(3).norm();
            if rot_n < self.rot_eps && pos_n < self.pos_eps {
                x = next;
                converged_at = Some(iter + 1);
                break;
            }
            x = next;
        }

        // 协方差更新 P ← (I − KH)P̂，KH = (S+P⁻¹)⁻¹S（对照 `vio.cpp:800`）
        x.cov = Self::cov_update(&prior.cov, &last_s, &p_inv);
        *state = x;
        let n = converged_at.unwrap_or(self.max_iterations);
        if converged_at.is_none() {
            let last_step = state.boxminus(&prior);
            log::info!(
                "esikf-noconv step_norm={:.3e} rot={:.3e} pos={:.3e} vel={:.3e}",
                last_step.norm(),
                last_step.fixed_rows::<3>(0).norm(),
                last_step.fixed_rows::<3>(3).norm(),
                last_step.fixed_rows::<3>(7).norm()
            );
        }
        Ok((n, converged_at.is_some()))
    }

    /// 协方差更新 `P = (I − KH)P`，`KH = (S+P⁻¹)⁻¹S`（对照 `voxel_map.cpp:489`）。
    #[must_use]
    pub fn cov_update(
        p: &StateCovariance,
        s: &InfoMatrix,
        p_inv: &SMatrix<f64, DIM_STATE, DIM_STATE>,
    ) -> StateCovariance {
        let kh = (*s + *p_inv)
            .try_inverse()
            .unwrap_or_else(InfoMatrix::identity)
            * *s;
        p - kh * p
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_void_types::state::StateCovariance;
    use nalgebra::{Rotation3, Vector3};

    /// 恒等测量模型：把位置投影为 3 维观测。
    struct IdentityMeasurement {
        /// 观测值（真实位置）。
        z_obs: Vector3<f64>,
        /// 测量噪声方差。
        r: f64,
    }

    impl MeasurementModel for IdentityMeasurement {
        fn residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
            let z = DVector::from_column_slice((x.pos - self.z_obs).as_slice());
            let mut h = DMatrix::zeros(3, DIM_STATE);
            for i in 0..3 {
                h[(i, 3 + i)] = 1.0;
            }
            let r = DMatrix::identity(3, 3) * self.r;
            (z, h, r)
        }

        fn dim(&self) -> usize {
            3
        }
    }

    /// 完整恒等测量模型：直接观测误差状态（19 维）。
    struct FullIdentity {
        truth: State,
        r: f64,
    }

    impl MeasurementModel for FullIdentity {
        fn residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
            let d = x.boxminus(&self.truth);
            let z = DVector::from_column_slice(d.as_slice());
            let h = DMatrix::identity(DIM_STATE, DIM_STATE);
            let r = DMatrix::identity(DIM_STATE, DIM_STATE) * self.r;
            (z, h, r)
        }

        fn dim(&self) -> usize {
            DIM_STATE
        }
    }

    #[test]
    fn identity_model_converges_to_map() {
        // 先验远离真值：顺序更新收敛到 MAP 解析解 x̂ + P̂(P̂+R)⁻¹(y−x̂)
        let truth = Vector3::new(1.0, -2.0, 0.5);
        let mut state = State::default(); // 先验位置 = 0，协方差 p=0.01
        let r = 1e-3;
        let mut updater =
            EskfUpdater::new(IdentityMeasurement { z_obs: truth, r }, 10, (1e-8, 1e-8));
        let (iters, _) = updater.update(&mut state).unwrap();
        assert!(iters >= 1);
        let p = 0.01;
        let k = p / (p + r);
        let map = Vector3::new(0.0, 0.0, 0.0) + k * (truth - Vector3::zeros());
        assert!(
            (state.pos - map).norm() < 1e-6,
            "pos={}, map={map}",
            state.pos
        );
        // 协方差应缩小到 (1−K·H)P（H=I 位置块）
        assert!(state.cov[(3, 3)] < 0.01);
    }

    #[test]
    fn zero_dim_measurement_is_noop() {
        struct Empty;
        impl MeasurementModel for Empty {
            fn residual(&self, _x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
                (
                    DVector::zeros(0),
                    DMatrix::zeros(0, 0),
                    DMatrix::zeros(0, 0),
                )
            }
            fn dim(&self) -> usize {
                0
            }
        }
        let mut state = State::default();
        let cov_before = state.cov;
        let mut updater = EskfUpdater::new(Empty, 5, (1e-4, 1e-4));
        let (iters, _) = updater.update(&mut state).unwrap();
        assert_eq!(iters, 0);
        assert_eq!(state.cov, cov_before);
    }

    #[test]
    fn information_aggregates_all_rows() {
        // 默认 information 聚合：逐行累加 HᵀR⁻¹H / HᵀR⁻¹z / zᵀR⁻¹z，
        // 与维度声明无关（以 residual 实际行数为准）
        struct Bad;
        impl MeasurementModel for Bad {
            fn residual(&self, _x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
                (
                    DVector::from_column_slice(&[1.0, -2.0]),
                    DMatrix::from_row_slice(
                        2,
                        DIM_STATE,
                        &[
                            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        ],
                    ),
                    DMatrix::from_diagonal(&DVector::from_column_slice(&[0.5, 2.0])),
                )
            }
            fn dim(&self) -> usize {
                3
            }
        }
        let (s, b, e) = Bad.information(&State::default());
        // S[0,0] = 1²/0.5 = 2；S[1,1] = (−1)²/2 = 0.5（H 行 2 在 idx1）
        assert!((s[(0, 0)] - 2.0).abs() < 1e-12);
        assert!((s[(1, 1)] - 0.5).abs() < 1e-12);
        // b[0] = 1·1/0.5 = 2；b[1] = −2·1/2 = −1
        assert!((b[0] - 2.0).abs() < 1e-12);
        assert!((b[1] + 1.0).abs() < 1e-12);
        // e = 1²/0.5 + 4/2 = 4
        assert!((e - 4.0).abs() < 1e-12);
    }

    #[test]
    fn sequential_two_measurements_equals_joint() {
        // 论文 (5)-(8) 式：两次独立测量顺序更新 ≡ 联合更新（线性化模型下）
        // 联合测量模型：一次 38 维测量（两批堆叠）
        struct Joint {
            truth: State,
            r1: f64,
            r2: f64,
        }
        impl MeasurementModel for Joint {
            fn residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
                let d = x.boxminus(&self.truth);
                let mut z = DVector::zeros(2 * DIM_STATE);
                let mut h = DMatrix::zeros(2 * DIM_STATE, DIM_STATE);
                let mut r = DMatrix::zeros(2 * DIM_STATE, 2 * DIM_STATE);
                for i in 0..DIM_STATE {
                    z[i] = d[i];
                    z[DIM_STATE + i] = d[i];
                    h[(i, i)] = 1.0;
                    h[(DIM_STATE + i, i)] = 1.0;
                    r[(i, i)] = self.r1;
                    r[(DIM_STATE + i, DIM_STATE + i)] = self.r2;
                }
                (z, h, r)
            }
            fn dim(&self) -> usize {
                2 * DIM_STATE
            }
        }
        let truth = State {
            rot: Rotation3::from_axis_angle(&Vector3::z_axis(), 0.3),
            pos: Vector3::new(1.0, -2.0, 0.5),
            vel: Vector3::new(0.1, 0.2, -0.3),
            bias_g: Vector3::new(0.01, -0.02, 0.03),
            bias_a: Vector3::new(-0.1, 0.2, -0.3),
            gravity: Vector3::new(0.0, 0.0, -9.8),
            inv_expo_time: 2.0,
            cov: StateCovariance::identity() * 0.01,
        };

        // 顺序：先 m1（r=0.01）后 m2（r=0.02）
        let mut s_seq = State {
            rot: Rotation3::identity(),
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            bias_g: Vector3::zeros(),
            bias_a: Vector3::zeros(),
            gravity: Vector3::new(0.0, 0.0, -9.8),
            inv_expo_time: 1.0,
            cov: StateCovariance::identity() * 0.01,
        };
        let mut upd1 = EskfUpdater::new(FullIdentity { truth, r: 0.01 }, 20, (1e-8, 1e-8));
        let _ = upd1.update(&mut s_seq).unwrap();
        let mut upd2 = EskfUpdater::new(FullIdentity { truth, r: 0.02 }, 20, (1e-8, 1e-8));
        let _ = upd2.update(&mut s_seq).unwrap();

        let mut s_joint = State {
            rot: Rotation3::identity(),
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            bias_g: Vector3::zeros(),
            bias_a: Vector3::zeros(),
            gravity: Vector3::new(0.0, 0.0, -9.8),
            inv_expo_time: 1.0,
            cov: StateCovariance::identity() * 0.01,
        };
        let mut upd_j = EskfUpdater::new(
            Joint {
                truth,
                r1: 0.01,
                r2: 0.02,
            },
            20,
            (1e-8, 1e-8),
        );
        let _ = upd_j.update(&mut s_joint).unwrap();

        let err = s_seq.boxminus(&s_joint).norm();
        assert!(err < 1e-6, "sequential vs joint err={err}");
    }

    #[test]
    fn covariance_shrinks_after_update() {
        let mut state = State::default();
        let mut updater = EskfUpdater::new(
            IdentityMeasurement {
                z_obs: Vector3::new(0.5, 0.5, 0.5),
                r: 0.1,
            },
            5,
            (1e-6, 1e-6),
        );
        let _ = updater.update(&mut state).unwrap();
        assert!(state.cov[(3, 3)] < 0.01);
        // 旋转方差不被位置观测改变（H 不触及该块）
        assert!((state.cov[(0, 0)] - 0.01).abs() < 1e-9);
    }
}
