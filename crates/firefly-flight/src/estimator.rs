//! 姿态估计（互补滤波）：陀螺积分 + 加速度计水平修正 + 外部航向修正。
//!
//! 飞控内环的姿态必须由飞控自己给出（真机只有 IMU）：陀螺积分给出高频响应，
//! 加速度计的比力方向在准静态下指向世界 `+Z`（含重力与运动加速度），用于修正
//! 滚转/俯仰的积分漂移；航向在无磁力计时不可观测，由外部航向源（VIO）慢速修正。
//!
//! 约定与 [`QuadState`](crate::QuadState) 一致：姿态为**机体→世界**四元数，
//! 陀螺为**机体系**角速度（rad/s）。加速度计修正带准静态门（`|a|/g` 偏离 1 超过
//! 阈值即跳过）——机动中的比力里含运动加速度，此时只信陀螺。

use glam::{Quat, Vec3};

use crate::G;

/// 互补滤波姿态估计器。
#[derive(Clone, Copy, Debug)]
pub struct AttitudeEstimator {
    /// 机体→世界姿态。
    attitude: Quat,
    /// 水平（加速度计）修正增益（1/s）。
    level_kp: f32,
    /// 航向修正增益（1/s）。
    yaw_kp: f32,
}

/// 准静态门：`|a|/g` 落在 `[1/ACCEL_TOL, ACCEL_TOL]` 外即认为在机动，跳过水平修正。
const ACCEL_TOL: f32 = 1.35;

impl Default for AttitudeEstimator {
    fn default() -> Self {
        Self {
            attitude: Quat::IDENTITY,
            level_kp: 2.0,
            yaw_kp: 0.5,
        }
    }
}

impl AttitudeEstimator {
    /// 以给定初始姿态起步（通常取首个外部姿态或水平）。
    #[must_use]
    pub fn new(attitude: Quat) -> Self {
        Self {
            attitude,
            ..Self::default()
        }
    }

    /// 由加速度计比力定初始姿态：滚转/俯仰取比力方向（准静态下比力指向世界 `+Z`，
    /// 与真值倾斜一致），航向置 0（无磁力计时不可观测，由外部航向源补）——飞控上电初始化。
    #[must_use]
    pub fn from_accel(accel_body: Vec3) -> Self {
        let up = accel_body.try_normalize().unwrap_or(Vec3::Z);
        Self::new(Quat::from_rotation_arc(up, Vec3::Z))
    }

    /// 当前姿态估计（机体→世界）。
    #[must_use]
    pub fn attitude(&self) -> Quat {
        self.attitude
    }

    /// 由外部姿态重置（估计器重启、外部姿态可信时）。
    pub fn reset(&mut self, attitude: Quat) {
        self.attitude = attitude.normalize();
    }

    /// 陀螺积分（机体系角速度，rad/s；`dt` 秒）。
    pub fn predict(&mut self, gyro_body: Vec3, dt: f32) {
        let delta = Quat::from_scaled_axis(gyro_body * dt);
        self.attitude = (self.attitude * delta).normalize();
    }

    /// 加速度计水平修正：比力方向（准静态下 = 世界 `+Z`）与预测值的差作为修正轴。
    pub fn correct_level(&mut self, accel_body: Vec3, dt: f32) {
        let g_meas = accel_body.length();
        if g_meas < 1e-3 || (g_meas / G - 1.0).abs() > (ACCEL_TOL - 1.0) {
            return;
        }
        let up_meas = accel_body / g_meas;
        let up_pred = self.attitude.inverse() * Vec3::Z;
        let err = up_meas.cross(up_pred);
        self.attitude =
            (self.attitude * Quat::from_scaled_axis(err * self.level_kp * dt)).normalize();
    }

    /// 航向修正：外部航向源（VIO 等）给出的世界系航向（rad）。
    pub fn correct_yaw(&mut self, yaw_world: f32, dt: f32) {
        let err = wrap_pi(yaw_world - self.yaw());
        self.attitude = (Quat::from_rotation_z(err * self.yaw_kp * dt) * self.attitude).normalize();
    }

    /// 一步：陀螺积分 + 水平修正（航向另行 [`correct_yaw`](Self::correct_yaw)，其源为低频）。
    pub fn update(&mut self, gyro_body: Vec3, accel_body: Vec3, dt: f32) {
        self.predict(gyro_body, dt);
        self.correct_level(accel_body, dt);
    }

    /// 水平航向（rad，世界 Z；机体 `+X` 在水平面的投影）。
    #[must_use]
    pub fn yaw(&self) -> f32 {
        crate::state::yaw_of(self.attitude)
    }
}

/// 角度折叠到 `[−π, π)`。
fn wrap_pi(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    /// 陀螺积分：绕机体 z 以 1 rad/s 转 1 s → 航向 1 rad。
    #[test]
    fn gyro_integration_yields_expected_yaw() {
        let mut est = AttitudeEstimator::new(Quat::IDENTITY);
        for _ in 0..1000 {
            est.predict(Vec3::Z, DT);
        }
        assert!((est.yaw() - 1.0).abs() < 1e-3, "航向 {:.4} rad", est.yaw());
    }

    /// 水平修正：估计值比真值倾斜 0.3 rad 起步（加速度计给真值方向）→ 收敛回水平。
    #[test]
    fn level_correction_recovers_level() {
        let mut est = AttitudeEstimator::new(Quat::from_rotation_x(0.3));
        let truth = Quat::IDENTITY;
        for _ in 0..3000 {
            let accel = truth.inverse() * (Vec3::Z * G);
            est.update(Vec3::ZERO, accel, DT);
        }
        let tilt = (est.attitude() * Vec3::Z).z.clamp(-1.0, 1.0).acos();
        assert!(tilt < 5e-3, "残倾 {tilt:.4} rad");
    }

    /// 机动门：比力远超 1g 时不修正水平（只信陀螺），姿态保持陀螺积分结果。
    #[test]
    fn level_correction_is_gated_during_maneuver() {
        let mut est = AttitudeEstimator::new(Quat::from_rotation_x(0.3));
        let before = est.attitude();
        for _ in 0..1000 {
            est.update(Vec3::ZERO, Vec3::new(6.0, 0.0, 3.0 * G), DT);
        }
        assert!(
            (est.attitude() - before).length() < 1e-6,
            "机动中不应修正水平"
        );
    }

    /// 航向修正：把航向拉到外部值，且不改变倾斜（绕世界 z 修正）。
    #[test]
    fn yaw_correction_tracks_external_source() {
        let mut est = AttitudeEstimator::new(Quat::from_rotation_z(0.8));
        let tilt_before = (est.attitude() * Vec3::Z).z;
        for _ in 0..20_000 {
            est.correct_yaw(0.0, DT);
        }
        assert!(est.yaw().abs() < 1e-3, "航向 {:.4} rad", est.yaw());
        assert!((tilt_before - (est.attitude() * Vec3::Z).z).abs() < 1e-6);
    }

    /// 上电初始化：加计测的比力方向给出滚转/俯仰（真值倾斜 0.25 rad），航向置 0。
    #[test]
    fn from_accel_recovers_tilt() {
        let truth = Quat::from_rotation_x(0.25);
        let accel = truth.inverse() * (Vec3::Z * G);
        let est = AttitudeEstimator::from_accel(accel);
        let tilt = (est.attitude() * Vec3::Z).z.clamp(-1.0, 1.0).acos();
        assert!((tilt - 0.25).abs() < 1e-3, "初始倾斜 {tilt:.4} rad");
        assert!(est.yaw().abs() < 1e-3, "初始航向应为 0");
    }

    /// 航向在 ±π 回绕处走最近路径（−π+ε → +π−ε 只差 2ε）。
    #[test]
    fn yaw_correction_wraps_shortest_path() {
        let mut est = AttitudeEstimator::new(Quat::from_rotation_z(-std::f32::consts::PI + 0.05));
        for _ in 0..1000 {
            est.correct_yaw(std::f32::consts::PI - 0.05, DT);
        }
        let yaw = est.yaw();
        assert!(yaw.abs() > 3.0, "应停在 ±π 附近而不是绕到 0：{yaw:.4}");
    }
}
