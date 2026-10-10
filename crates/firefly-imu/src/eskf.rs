//! 姿态 ESKF：Robo-Rust `eskf_imu.rs` 的名义/误差状态接口，采用 Solà 右误差方程。
//! 加计 NIS 对照 `WangHongxi2001` `QuaternionEKF.c::IMU_QuaternionEKF_xhatUpdate`。

use crate::{Options, invalid};
use firefly_error::Result;
use nalgebra::{Matrix3, SMatrix, SVector, UnitQuaternion, Vector3};

pub type Matrix6 = SMatrix<f64, 6, 6>;
type Vector6 = SVector<f64, 6>;

/// 独立姿态与机体系陀螺零偏；协方差排列为 `[δθ(rad), δbg(rad/s)]`。
#[derive(Clone, Debug)]
pub struct AttitudeState {
    pub rotation: UnitQuaternion<f64>,
    pub bias: Vector3<f64>,
    pub covariance: Matrix6,
}

/// 加计诊断；NIS 有效不意味着没有运动加速度。
#[derive(Clone, Copy, Debug, Default)]
pub struct Gate {
    pub nis: f64,
    pub accepted: bool,
}

#[must_use]
pub fn right_jacobian(phi: Vector3<f64>) -> Matrix3<f64> {
    let a = phi.norm();
    let w = skew(phi);
    if a < 1e-5 {
        return Matrix3::identity() - w * 0.5 + w * w / 6.;
    }
    Matrix3::identity() - w * ((1. - a.cos()) / a.powi(2)) + w * w * ((a - a.sin()) / a.powi(3))
}

/// 常角速度、机体系常零偏误差的精确离散转移；dt 单位秒。
#[must_use]
pub fn transition(omega: Vector3<f64>, dt: f64) -> Matrix6 {
    let mut f = Matrix6::identity();
    f.fixed_view_mut::<3, 3>(0, 0).copy_from(
        UnitQuaternion::from_scaled_axis(-omega * dt)
            .to_rotation_matrix()
            .matrix(),
    );
    f.fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(-dt * right_jacobian(omega * dt)));
    f
}

/// `h=Rᵀ(0,0,g)`，`R_true=R Exp(δθ)`，因此 `Hθ=+[h]×`。
#[must_use]
pub fn gravity_jacobian(h: Vector3<f64>) -> SMatrix<f64, 3, 6> {
    let mut out = SMatrix::<f64, 3, 6>::zeros();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(&skew(h));
    out
}
fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0., -v.z, v.y, v.z, 0., -v.x, -v.y, v.x, 0.)
}
#[allow(clippy::large_types_passed_by_value)] // 固定维矩阵表达式的值语义
fn symmetric(p: Matrix6) -> Matrix6 {
    (p + p.transpose()) * 0.5
}

impl AttitudeState {
    /// # Errors
    /// 非有限名义状态、非对称或非正定协方差。
    pub fn validate(&self) -> Result<()> {
        if !self
            .rotation
            .coords
            .iter()
            .chain(self.bias.iter())
            .chain(self.covariance.iter())
            .all(|x| x.is_finite())
            || (self.rotation.norm() - 1.).abs() > 1e-8
            || (self.covariance - self.covariance.transpose()).amax() > 1e-10
            || self.covariance.cholesky().is_none()
        {
            return Err(invalid("invalid attitude state or covariance"));
        }
        Ok(())
    }

    /// 名义状态传播与连续白噪声积分；dt=0 保持状态。
    /// # Errors
    /// 非法 gyro/dt，或测量间隔超过上限。
    #[fastrace::trace]
    pub fn predict(&mut self, gyro: Vector3<f64>, dt: f64, o: &Options) -> Result<()> {
        if !dt.is_finite() || dt < 0. || dt > o.max_dt + 1e-9 || !gyro.iter().all(|x| x.is_finite())
        {
            return Err(invalid("invalid gyro or IMU interval"));
        }
        let omega = gyro - self.bias;
        let f = transition(omega, dt);
        let mut qc = Matrix6::zeros();
        for i in 0..3 {
            qc[(i, i)] = o.gyro_density.powi(2);
            qc[(i + 3, i + 3)] = o.bias_walk.powi(2);
        }
        // 五点 Gauss-Legendre 积分 ∫Φ(s)QcΦ(s)ᵀ ds，保留 θ/bg 的交叉项。
        let mut qd = Matrix6::zeros();
        for (x, w) in [
            (0., 0.568_888_888_888_888_9),
            (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
            (0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
            (-0.906_179_845_938_664, 0.236_926_885_056_189_1),
            (0.906_179_845_938_664, 0.236_926_885_056_189_1),
        ] {
            let phi = transition(omega, dt * (x + 1.) * 0.5);
            qd += phi * qc * phi.transpose() * (dt * w * 0.5);
        }
        self.rotation *= UnitQuaternion::from_scaled_axis(omega * dt);
        self.covariance = symmetric(f * self.covariance * f.transpose() + qd);
        Ok(())
    }

    /// 加计校正仅允许调用方已确认的地面静止期；NIS 始终计算供诊断。
    /// # Errors
    /// 非有限测量、非正测量周期或创新协方差非正定。
    #[fastrace::trace]
    #[allow(clippy::many_single_char_names)] // 与卡尔曼标准矩阵记号一致
    pub fn correct_accel(
        &mut self,
        accel: Vector3<f64>,
        dt: f64,
        stationary: bool,
        o: &Options,
    ) -> Result<Gate> {
        if !dt.is_finite() || dt <= 0. || !accel.iter().all(|x| x.is_finite()) {
            return Err(invalid("invalid accelerometer measurement"));
        }
        let predicted = self.rotation.inverse() * Vector3::z() * o.gravity;
        let residual = accel - predicted;
        let h = gravity_jacobian(predicted);
        let r = Matrix3::identity() * (o.accel_density.powi(2) / dt + o.accel_model_std.powi(2));
        let s = h * self.covariance * h.transpose() + r;
        let chol = s
            .cholesky()
            .ok_or_else(|| invalid("non-positive innovation covariance"))?;
        let nis = residual.dot(&chol.solve(&residual));
        let accepted = stationary
            && (accel.norm() - o.gravity).abs() <= o.stationary_accel_error
            && nis <= o.accel_nis_limit;
        if accepted {
            let k = chol.solve(&(h * self.covariance)).transpose();
            let a = Matrix6::identity() - k * h;
            self.covariance =
                symmetric(a * self.covariance * a.transpose() + k * r * k.transpose());
            self.inject(k * residual);
        }
        Ok(Gate { nis, accepted })
    }

    fn inject(&mut self, dx: Vector6) {
        let angle = dx.fixed_rows::<3>(0).into_owned();
        self.rotation *= UnitQuaternion::from_scaled_axis(angle);
        self.bias += dx.fixed_rows::<3>(3);
        let mut reset = Matrix6::identity();
        reset
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&right_jacobian(angle));
        self.covariance = symmetric(reset * self.covariance * reset.transpose());
    }

    /// 未知交叉相关下的协方差交集，在同一右误差切空间内融合完整六维边缘分布。
    /// 返回 false 表示外援差异过大；不会因连续拒绝而强制放行。
    /// # Errors
    /// 外部状态无效或矩阵非正定。
    #[fastrace::trace]
    pub fn intersect(&mut self, external: &Self) -> Result<bool> {
        external.validate()?;
        let delta = (self.rotation.inverse() * external.rotation).scaled_axis();
        if delta.norm() > 0.5 {
            return Ok(false);
        }
        let mut dx = Vector6::zeros();
        dx.fixed_rows_mut::<3>(0).copy_from(&delta);
        dx.fixed_rows_mut::<3>(3)
            .copy_from(&(external.bias - self.bias));
        let mut transport = Matrix6::identity();
        transport.fixed_view_mut::<3, 3>(0, 0).copy_from(
            &right_jacobian(delta)
                .try_inverse()
                .ok_or_else(|| invalid("singular tangent transport"))?,
        );
        let pe = symmetric(transport * external.covariance * transport.transpose());
        // Cov(e1-e2) ≤ 2(P1+P2) 对任意未知相关成立；χ²(6) 99.9% 保守门。
        let bound = ((self.covariance + pe) * 2.)
            .cholesky()
            .ok_or_else(|| invalid("invalid aid covariance"))?;
        if dx.dot(&bound.solve(&dx)) > 22.457_744_484_825_323 {
            return Ok(false);
        }
        let a = self
            .covariance
            .cholesky()
            .ok_or_else(|| invalid("invalid prior covariance"))?
            .inverse();
        let b = pe
            .cholesky()
            .ok_or_else(|| invalid("invalid external covariance"))?
            .inverse();
        let mut best = (f64::INFINITY, Matrix6::zeros(), Vector6::zeros());
        for i in 0..=20 {
            let w = f64::from(i) / 20.;
            let info = (a * w + b * (1. - w))
                .cholesky()
                .ok_or_else(|| invalid("invalid CI information"))?;
            let score = -2. * info.l().diagonal().map(f64::ln).sum();
            if score < best.0 {
                best = (score, info.inverse(), info.solve(&(b * dx * (1. - w))));
            }
        }
        self.covariance = best.1;
        self.inject(best.2);
        Ok(true)
    }
}
