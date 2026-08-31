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
type InfoMatrix = SMatrix<f64, DIM_STATE, DIM_STATE>;

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
        }
    }

    /// 设置发散保护倍数（残差放大超过该倍数即判发散）。
    #[must_use]
    pub fn with_divergence_factor(mut self, factor: f64) -> Self {
        self.divergence_factor = factor;
        self
    }

    /// 顺序更新：`state` 为传播先验（含协方差），就地更新到后验。
    ///
    /// 迭代 `x^{κ+1} = x^κ ⊞ (−Kz^κ − (I − KH)(x^κ ⊟ x̂))` 直至收敛或
    /// 达到迭代上限；返回 `(迭代次数, 是否收敛)`。协方差更新
    /// `P = (I − KH)P̂` 用最后一次迭代的 K/H（对照 `voxel_map.cpp:489`
    /// 与 `vio.cpp:800`）。
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
        let (z0, h0, r0) = self.model.residual(&prior);
        Self::check_consistent(&z0, &h0, &r0, dim)?;
        let prior_residual_norm = z0.norm();

        let p_inv = prior.cov.try_inverse().ok_or_else(|| {
            Error::new(ErrorKind::Internal, "先验协方差不可逆").with_context("state", "prior cov")
        })?;
        let p_inv = DMatrix::from_row_slice(DIM_STATE, DIM_STATE, p_inv.as_slice());

        let mut x = prior;
        let mut last_k = DMatrix::zeros(DIM_STATE, dim);
        let mut last_h = DMatrix::zeros(dim, DIM_STATE);
        let mut converged_at = None;

        for iter in 0..self.max_iterations {
            let (z, h, r) = self.model.residual(&x);
            Self::check_consistent(&z, &h, &r, dim)?;
            if z.iter().any(|v| v.is_nan()) {
                return Err(
                    Error::new(ErrorKind::Convergence, "残差含 NaN").with_context("iter", iter)
                );
            }
            if prior_residual_norm > 0.0 && z.norm() > self.divergence_factor * prior_residual_norm
            {
                return Err(Error::new(ErrorKind::Convergence, "残差发散")
                    .with_context("prior_norm", prior_residual_norm)
                    .with_context("cur_norm", z.norm()));
            }

            // 论文 (11) 式：K = (HᵀR⁻¹H + P⁻¹)⁻¹ HᵀR⁻¹
            // R 恒为对角阵（逐点测量噪声独立），按对角元求逆 O(n)，
            // 替代 try_inverse 的 O(n³)（n=3000 时 5 迭代约 19s → 0.2s）
            let r_inv = DMatrix::from_diagonal(&r.diagonal().map(|v| {
                if v <= 0.0 || !v.is_finite() {
                    1.0 / 1e12 // 无效点占位（零信息行），与 R=1e12 约定一致
                } else {
                    1.0 / v
                }
            }));
            let htr_inv = h.transpose() * r_inv;
            let htr_inv_h = &htr_inv * &h;
            let k = (&htr_inv_h + &p_inv)
                .try_inverse()
                .ok_or_else(|| Error::new(ErrorKind::Internal, "信息矩阵不可逆"))?
                * htr_inv;

            // x^{κ+1} = x^κ ⊞ (−Kz − (I − KH)(x^κ ⊟ x̂))
            let kh = &k * &h;
            let delta_prior = x.boxminus(&prior);
            let correction = -(&k * &z) - (InfoMatrix::identity() - &kh) * delta_prior;
            let next = x.boxplus(&correction);

            last_k = k.clone();
            last_h = h.clone();
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

        x.cov = Self::cov_update(&prior.cov, &last_k, &last_h);
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

    /// 协方差更新 `P = (I − KH)P`。
    ///
    /// Joseph 形式 `(I−KH)P(I−KH)ᵀ + KRKᵀ` 数值更稳但开销大；官方
    /// （`voxel_map.cpp:489`、`vio.cpp:800`）用简化式，本实现保持对照。
    #[must_use]
    pub fn cov_update(p: &StateCovariance, k: &DMatrix<f64>, h: &DMatrix<f64>) -> StateCovariance {
        let kh = k * h;
        let kh19 = SMatrix::<f64, DIM_STATE, DIM_STATE>::from_fn(|i, j| kh[(i, j)]);
        p - kh19 * p
    }

    fn check_consistent(
        z: &DVector<f64>,
        h: &DMatrix<f64>,
        r: &DMatrix<f64>,
        dim: usize,
    ) -> Result<(), Error> {
        if h.nrows() != dim || h.ncols() != DIM_STATE {
            return Err(
                Error::new(ErrorKind::InvalidArgument, "残差雅可比维度不匹配")
                    .with_context("h_rows", h.nrows())
                    .with_context("h_cols", h.ncols())
                    .with_context("dim", dim),
            );
        }
        if r.nrows() != dim || r.ncols() != dim {
            return Err(Error::new(ErrorKind::InvalidArgument, "测量噪声维度不匹配")
                .with_context("r_dim", format!("{}x{}", r.nrows(), r.ncols()))
                .with_context("dim", dim));
        }
        if z.len() != dim {
            return Err(Error::new(ErrorKind::InvalidArgument, "残差向量维度不匹配")
                .with_context("z_dim", z.len())
                .with_context("dim", dim));
        }
        Ok(())
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
    fn dimension_mismatch_returns_error() {
        struct Bad;
        impl MeasurementModel for Bad {
            fn residual(&self, _x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
                // 声明 3 维但返回 2 行雅可比
                (
                    DVector::zeros(2),
                    DMatrix::zeros(2, DIM_STATE),
                    DMatrix::zeros(2, 2),
                )
            }
            fn dim(&self) -> usize {
                3
            }
        }
        let mut state = State::default();
        let mut updater = EskfUpdater::new(Bad, 5, (1e-4, 1e-4));
        let err = updater.update(&mut state).unwrap_err();
        assert_eq!(err.kind(), ErrorKind::InvalidArgument);
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
