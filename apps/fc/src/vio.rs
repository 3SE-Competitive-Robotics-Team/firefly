//! VIO 估计与飞控姿态的约定换算。
//!
//! `OdomMessage` 的姿态是 JPL `q_GtoI`（全局→机体，标量在最后）。JPL 的
//! `quat_2_rot(q)`（对照 `firefly-vio-types::quat_ops`，与 `OpenVINS` 的 `quat_2_Rot` 同源）
//! 就是 `R_GtoI`；其 Hamilton 形式为 `R_h(q) = quat_2_rot(q)ᵀ = R_ItoG`——即
//! **同一组分量**按 Hamilton 读就是机体→世界姿态（`[x,y,z,w]` 直接可用，无需换符号）。
//!
//! 这条等价关系由 `conversion_matches_reference` 单测对照移植实现钉死：改约定先改它。

use std::collections::VecDeque;

use glam::{Quat, Vec3};

/// 带时间戳的机体系陀螺缓存，用于将 VIO 姿态预测到最新 IMU 时刻。
/// 区间采用右端采样零阶保持；只接受缓存覆盖的时刻，不向外补测量。
#[derive(Default)]
pub struct GyroHistory(VecDeque<(f64, Vec3)>);

impl GyroHistory {
    pub fn push(&mut self, time: f64, gyro: Vec3) {
        if !time.is_finite() || !gyro.is_finite() || self.0.back().is_some_and(|x| time <= x.0) {
            return;
        }
        self.0.push_back((time, gyro));
        while self.0.len() > 256 {
            self.0.pop_front();
        }
    }

    pub fn propagate(&self, time: f64, attitude: Quat) -> Option<Quat> {
        if !time.is_finite() || time < self.0.front()?.0 || time > self.0.back()?.0 {
            return None;
        }
        let mut result = attitude;
        let mut previous = time;
        for (&(left, _), &(stamp, gyro)) in self.0.iter().zip(self.0.iter().skip(1)) {
            if stamp <= time {
                continue;
            }
            let dt = stamp - previous;
            if stamp - left > super::IMU_STALE_LIMIT.as_secs_f64() + 1e-9 {
                return None;
            }
            result = (result * Quat::from_scaled_axis(gyro * dt as f32)).normalize();
            previous = stamp;
        }
        Some(result)
    }
}

/// `OdomMessage` 的 JPL `q_GtoI`（`[x,y,z,w]`）→ 机体→世界 Hamilton 姿态。
#[must_use]
pub fn body_to_world_from_odom(quat_xyzw: [f64; 4]) -> Quat {
    let [x, y, z, w] = quat_xyzw;
    Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32).normalize()
}

#[cfg(test)]
mod tests {
    use firefly_vio_types::quat_ops::quat_2_rot;
    use glam::{Mat3, Quat};
    use nalgebra::Vector4;

    use super::{GyroHistory, body_to_world_from_odom};

    #[test]
    fn delayed_attitude_replays_body_rotation_and_rejects_uncovered_time() {
        let mut history = GyroHistory::default();
        for i in 0..=100 {
            history.push(f64::from(i) * 0.01, glam::Vec3::X * 0.7);
        }
        let anchor = Quat::from_rotation_z(1.1) * Quat::from_rotation_y(0.3);
        let expected = anchor * Quat::from_rotation_x(0.7 * 0.745);
        let actual = history.propagate(0.255, anchor).unwrap();
        assert!(actual.abs_diff_eq(expected, 2e-6));
        // 解析导数 d(Rx(theta)y)/dtheta = [0,-sin(theta),cos(theta)]。
        let h = 1e-3;
        let plus = history.propagate(0.255 + h, anchor).unwrap() * glam::Vec3::Y;
        let minus = history.propagate(0.255 - h, anchor).unwrap() * glam::Vec3::Y;
        let derivative =
            anchor * glam::Vec3::new(0., -(0.7_f32 * 0.745).sin(), (0.7_f32 * 0.745).cos()) * -0.7;
        assert!(((plus - minus) / (2. * h as f32) - derivative).length() < 2e-4);
        assert!(history.propagate(-0.01, anchor).is_none());
        assert!(history.propagate(1.01, anchor).is_none());
        history.push(1.2, glam::Vec3::X);
        assert!(history.propagate(0.9, anchor).is_none());
        assert!(history.propagate(1.19, anchor).is_none());
    }

    /// 姿态换算必须与 `firefly-vio-types` 的 JPL 实现一致：
    /// `R_h(q) == quat_2_rot(q)ᵀ == R_ItoG`（机体→世界）。
    #[test]
    fn conversion_matches_reference() {
        // 覆盖：单位、绕单轴 ±、复合旋转、大角；`quat_2_rot` 假定已归一化，
        // 故测试先归一化（生产路径自带 normalize）。
        let cases: [[f64; 4]; 6] = [
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.382_683_4, 0.923_879_5], // yaw 45°
            [0.0, -0.5, 0.0, 0.866_025_4],        // pitch −60°
            [-0.258_819, 0.0, 0.0, 0.965_925_8],  // roll −30°
            [0.25, 0.5, -0.25, 0.792_812_9],      // 复合（未归一化，测试归一）
            [0.6, 0.0, 0.0, 0.8],                 // 大角
        ];
        for mut q in cases {
            let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            for v in &mut q {
                *v /= norm;
            }
            let reference = quat_2_rot(&Vector4::new(q[0], q[1], q[2], q[3])).transpose();
            let ours = Mat3::from_quat(body_to_world_from_odom(q));
            let mut max_diff = 0.0f64;
            for r in 0..3 {
                for c in 0..3 {
                    max_diff = max_diff.max((f64::from(ours.col(c)[r]) - reference[(r, c)]).abs());
                }
            }
            assert!(
                max_diff < 1e-5,
                "约定不一致（最大偏差 {max_diff:.3e}）：q={q:?}"
            );
        }
    }

    /// 换算不引入额外旋转：单位四元数 → 单位姿态。
    #[test]
    fn identity_stays_identity() {
        let q = body_to_world_from_odom([0.0, 0.0, 0.0, 1.0]);
        assert!(Quat::IDENTITY.abs_diff_eq(q, 1e-9));
    }
}
