//! 静态初始化器（对照 `OpenVINS` `ov_init/src/static/StaticInitializer.cpp`）。
//!
//! 假设 IMU 从静止出发：收集窗口内 IMU 读数，计算两段窗口（2→1 与 1→0）的
//! 样本方差判断是否静止/急动，取 2→1 窗口的均值加速度作为重力方向
//! （`gram_schmidt` 得到 `R_GtoI`），陀螺偏置取角速度均值、加速度计偏置取
//! 加速度均值减去重力投影。无 Ceres，纯 IMU 统计。

#![allow(
    non_snake_case,
    clippy::float_cmp,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_lines
)]

use crate::InitResult;
use crate::helper::gram_schmidt;
use crate::options::InitOptions;
use firefly_vio_core::sensor::ImuData;
use firefly_vio_types::quat_ops::{quat_2_rot, rot_2_quat};
use nalgebra::{DMatrix, Vector3};

/// 静态初始化器（对照 `StaticInitializer`）。
#[derive(Debug, Clone)]
pub struct StaticInitializer {
    /// 初始化器选项。
    pub params: InitOptions,
    /// IMU 测量缓冲（按 `feed_imu` 的 `oldest_time` 清理）。
    pub imu_data: Vec<ImuData>,
}

impl StaticInitializer {
    /// 构造（对照 `StaticInitializer` 构造函数）。
    #[must_use]
    pub fn new(params: InitOptions) -> Self {
        Self {
            params,
            imu_data: Vec::new(),
        }
    }

    /// 喂入 IMU 测量并在必要时清理过期读数
    /// （对照 `InertialInitializer::feed_imu` 的 push + erase 循环）。
    pub fn feed_imu(&mut self, message: &ImuData, oldest_time: f64) {
        self.imu_data.push(*message);
        if oldest_time != -1.0 {
            self.imu_data.retain(|d| d.timestamp >= oldest_time);
        }
    }

    /// 尝试静态初始化（对照 `StaticInitializer::initialize`；返回 `None` 表示
    /// 数据不足 / 未通过静止与急动检测，需要继续等待）。
    pub fn initialize(&mut self, wait_for_jerk: bool) -> Option<InitResult> {
        // 至少 2 条测量，且覆盖完整初始化窗口
        if self.imu_data.len() < 2 {
            return None;
        }
        let newesttime = self.imu_data[self.imu_data.len() - 1].timestamp;
        let oldesttime = self.imu_data[0].timestamp;
        if newesttime - oldesttime < self.params.init_window_time {
            log::debug!("静态初始化：IMU 读数不足一个窗口，无法选取窗口");
            return None;
        }

        // 取两段窗口：1→0 为最新半窗口（应检测到急动），2→1 为其前一窗口（应静止）
        let mut window_1to0: Vec<ImuData> = Vec::new();
        let mut window_2to1: Vec<ImuData> = Vec::new();
        for data in &self.imu_data {
            let t = data.timestamp;
            if t > newesttime - 0.5 * self.params.init_window_time && t <= newesttime {
                window_1to0.push(*data);
            }
            if t > newesttime - self.params.init_window_time
                && t <= newesttime - 0.5 * self.params.init_window_time
            {
                window_2to1.push(*data);
            }
        }
        if window_1to0.len() < 2 || window_2to1.len() < 2 {
            log::debug!(
                "静态初始化：窗口内 IMU 读数不足（1to0={}, 2to1={}）",
                window_1to0.len(),
                window_2to1.len()
            );
            return None;
        }

        // 1→0 窗口加速度均值与样本标准差（检验急动）
        let a_avg_1to0 = mean_accel(&window_1to0);
        let a_var_1to0 = sample_std(&window_1to0, &a_avg_1to0);
        // 2→1 窗口加速度/角速度均值与样本标准差（检验静止）
        let a_avg_2to1 = mean_accel(&window_2to1);
        let w_avg_2to1 = mean_gyro(&window_2to1);
        let a_var_2to1 = sample_std(&window_2to1, &a_avg_2to1);
        log::debug!("静态初始化：IMU 激励统计 {a_var_2to1:.3},{a_var_1to0:.3}");

        // 等待急动：新窗口须超过阈值（检测到急动），旧窗口须低于阈值（彼时静止）
        if a_var_1to0 < self.params.init_imu_thresh && wait_for_jerk {
            log::debug!(
                "静态初始化：无 IMU 激励（{a_var_1to0:.3} < {})",
                self.params.init_imu_thresh
            );
            return None;
        }
        if a_var_2to1 > self.params.init_imu_thresh && wait_for_jerk {
            log::debug!(
                "静态初始化：IMU 激励过大（{a_var_2to1:.3} > {})",
                self.params.init_imu_thresh
            );
            return None;
        }
        // 不等待急动：两段窗口都必须静止
        if (a_var_1to0 > self.params.init_imu_thresh || a_var_2to1 > self.params.init_imu_thresh)
            && !wait_for_jerk
        {
            log::debug!(
                "静态初始化：IMU 激励过大（{a_var_2to1:.3},{a_var_1to0:.3} > {})",
                self.params.init_imu_thresh
            );
            return None;
        }

        // z 轴对齐 -g（z_in_G=[0,0,1] 约定），gram_schmidt 得 R_GtoI
        let z_axis = a_avg_2to1 / a_avg_2to1.norm();
        let ro = gram_schmidt(&z_axis);
        let q_GtoI = rot_2_quat(&ro);

        // 偏置：陀螺取角速度均值，加速度计减去重力投影
        let gravity_inG = Vector3::new(0.0, 0.0, self.params.gravity_mag);
        let bg = w_avg_2to1;
        let ba = a_avg_2to1 - quat_2_rot(&q_GtoI) * gravity_inG;

        // 组装 16 维 IMU 状态 [q(4), p(3), v(3), bg(3), ba(3)]；p/v 置零
        let mut imu_state = [0.0f64; 16];
        imu_state[0..4].copy_from_slice(q_GtoI.as_slice());
        imu_state[10..13].copy_from_slice(bg.as_slice());
        imu_state[13..16].copy_from_slice(ba.as_slice());

        // 协方差 15×15：q 0.02²、p 0.05²、v 0.01²（其余块 0.02²）
        let mut covariance = 0.02f64.powi(2) * DMatrix::<f64>::identity(15, 15);
        // p 块 0.05²、v 块 0.01²（q 块保持 0.02²）
        for i in 0..3 {
            covariance[(3 + i, 3 + i)] = 0.05f64.powi(2);
            covariance[(6 + i, 6 + i)] = 0.01f64.powi(2);
        }

        let timestamp = window_2to1[window_2to1.len() - 1].timestamp;
        log::info!("静态初始化成功：t={timestamp:.3}");
        Some(InitResult {
            timestamp,
            covariance,
            order: vec![(0, 15)],
            imu_state,
            clones_imu: Vec::new(),
            features_slam: Vec::new(),
        })
    }
}

/// 窗口内加速度均值（对照 C++ 的 `a_avg = Σam / n`）。
fn mean_accel(window: &[ImuData]) -> Vector3<f64> {
    let sum = window.iter().fold(Vector3::zeros(), |acc, d| acc + d.am);
    sum / window.len() as f64
}

/// 窗口内角速度均值（对照 C++ 的 `w_avg = Σwm / n`）。
fn mean_gyro(window: &[ImuData]) -> Vector3<f64> {
    let sum = window.iter().fold(Vector3::zeros(), |acc, d| acc + d.wm);
    sum / window.len() as f64
}

/// 相对均值加速度的样本标准差（对照 C++ 的 `sqrt(Σ|am-avg|² / (n-1))`)。
fn sample_std(window: &[ImuData], avg: &Vector3<f64>) -> f64 {
    let sum = window
        .iter()
        .fold(0.0, |acc, d| acc + (d.am - avg).norm_squared());
    (sum / (window.len() as f64 - 1.0)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::InitOptions;
    use nalgebra::Vector3;

    /// 以 100Hz 生成静止 IMU 序列（am≈(0,0,9.81)+微小噪声、wm≈0）。
    fn stationary_imu(duration: f64, seed: u64) -> Vec<ImuData> {
        const HZ: f64 = 100.0;
        let mut x = seed;
        let mut noise = move || {
            // 简单 LCG 伪随机数（避免依赖外部 rand crate）。取两次采样均值使期望为
            // 0，从而静止平台加速度均值收敛到 `(0,0,9.81)`、`ba≈0`。
            let mut draw = || {
                x = x
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                (x >> 33) as f64 / (1u64 << 31) as f64 * 2.0 - 1.0
            };
            0.5 * (draw() + draw())
        };
        (0..=(duration * HZ) as usize)
            .map(|k| {
                let t = f64::from(k as u32) / HZ;
                ImuData {
                    timestamp: t,
                    wm: Vector3::new(noise() * 1e-4, noise() * 1e-4, noise() * 1e-4),
                    am: Vector3::new(noise() * 5e-3, noise() * 5e-3, 9.81 + noise() * 5e-3),
                }
            })
            .collect()
    }

    #[test]
    fn stationary_imu_initializes() {
        let mut init = StaticInitializer::new(InitOptions::default());
        for d in stationary_imu(2.0, 42) {
            init.feed_imu(&d, 0.0);
        }
        let res = init.initialize(false).expect("静止 IMU 应初始化成功");
        // q 单位四元数；bg≈0；ba≈0（加速度均值含重力，投影后抵消）
        let q = Vector3::new(res.imu_state[0], res.imu_state[1], res.imu_state[2]);
        let qw = res.imu_state[3];
        assert!((q.norm_squared() + qw * qw - 1.0).abs() < 1e-9);
        for i in 10..13 {
            assert!(res.imu_state[i].abs() < 1e-3, "bg[{i}] 应≈0");
        }
        for i in 13..16 {
            assert!(
                res.imu_state[i].abs() < 1e-3,
                "ba[{i}] 应≈0，got {}",
                res.imu_state[i]
            );
        }
        for i in 4..10 {
            assert_eq!(res.imu_state[i], 0.0, "p/v 应为 0");
        }
        // 协方差 15×15，对角块 0.02²/0.05²/0.01²
        assert_eq!((res.covariance.nrows(), res.covariance.ncols()), (15, 15));
        assert!((res.covariance[(0, 0)] - 0.02f64.powi(2)).abs() < 1e-12);
        assert!((res.covariance[(3, 3)] - 0.05f64.powi(2)).abs() < 1e-12);
        assert!((res.covariance[(6, 6)] - 0.01f64.powi(2)).abs() < 1e-12);
        assert_eq!(res.order, vec![(0, 15)]);
        assert!(res.clones_imu.is_empty() && res.features_slam.is_empty());
        // 时间戳应为 2→1 窗口最后读数时刻（最晚不超过 newesttime=2.0）
        assert!(res.timestamp > 1.0 && res.timestamp <= 2.0);
    }

    #[test]
    fn too_few_imu_readings_fails() {
        let mut init = StaticInitializer::new(InitOptions::default());
        init.feed_imu(
            &ImuData {
                timestamp: 0.0,
                wm: Vector3::zeros(),
                am: Vector3::zeros(),
            },
            -1.0,
        );
        assert!(init.initialize(false).is_none());
    }

    #[test]
    fn window_too_short_fails() {
        let mut init = StaticInitializer::new(InitOptions::default());
        for k in 0..5 {
            init.feed_imu(
                &ImuData {
                    timestamp: f64::from(k) * 0.01,
                    wm: Vector3::zeros(),
                    am: Vector3::zeros(),
                },
                -1.0,
            );
        }
        // 窗口只覆盖 0.04s < init_window_time=1.0s
        assert!(init.initialize(false).is_none());
    }

    #[test]
    fn wait_for_jerk_requires_excitation() {
        let mut init = StaticInitializer::new(InitOptions::default());
        for d in stationary_imu(2.0, 7) {
            init.feed_imu(&d, 0.0);
        }
        // wait_for_jerk=true 且无急动 → 失败
        assert!(init.initialize(true).is_none());
    }

    #[test]
    fn feed_imu_cleans_by_oldest_time() {
        let mut init = StaticInitializer::new(InitOptions::default());
        for k in 0..10 {
            init.feed_imu(
                &ImuData {
                    timestamp: f64::from(k),
                    wm: Vector3::zeros(),
                    am: Vector3::zeros(),
                },
                3.0,
            );
        }
        assert!(init.imu_data.iter().all(|d| d.timestamp >= 3.0));
        assert_eq!(init.imu_data.len(), 7);
    }
}
