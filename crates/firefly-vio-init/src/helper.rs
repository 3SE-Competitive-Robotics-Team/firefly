//! 动态初始化辅助函数（对照 `ov_init/src/utils/helper.h` 的 `InitializerHelper`）。
//!
//! 全部为无状态的纯函数：IMU 线性插值、积分区间读数选择、重力方向的
//! Gram–Schmidt 旋转构造，以及董氏约束优化的多项式系数。

#![allow(clippy::many_single_char_names, clippy::similar_names)]

use firefly_vio_core::sensor::ImuData;
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

/// 线性插值两个 IMU 测量（对照 `InitializerHelper::interpolate_data`）。
///
/// 以时间戳 `timestamp` 在 `imu_1`/`imu_2` 之间按比例 `lambda` 插值角速度与
/// 加速度；返回的测量时间戳为 `timestamp` 本身。
#[must_use]
pub fn interpolate_data(imu_1: &ImuData, imu_2: &ImuData, timestamp: f64) -> ImuData {
    let lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    ImuData {
        timestamp,
        am: (1.0 - lambda) * imu_1.am + lambda * imu_2.am,
        wm: (1.0 - lambda) * imu_1.wm + lambda * imu_2.wm,
    }
}

/// 从 IMU 缓冲中选取 `[time0, time1]` 区间的读数，首尾线性插值补齐
/// （对照 `InitializerHelper::select_imu_readings`）。
///
/// 忠实复刻 C++ 的四个 `if` 分支（区间起点切断 / 区间中段整段 / 区间末端）与
/// 末段零 `dt` 相邻测量移除；测量不足时返回空向量（与 C++ 一致，返回 `Vec`
/// 而非 `Option`）。注意该实现不含 `firefly-vio-core::propagation` 版本末尾的
/// 外推到 `time1` 逻辑，逐行对照 `helper.h`。
// C++ 端点检查使用严格 `!=`（忠实照搬），故 `float_cmp` 予以允许。
#[allow(clippy::float_cmp)]
#[must_use]
pub fn select_imu_readings(imu_data: &[ImuData], time0: f64, time1: f64) -> Vec<ImuData> {
    // 我们的 imu 读数向量
    let mut prop_data: Vec<ImuData> = Vec::new();

    // 首先要确保有一些测量！
    if imu_data.is_empty() {
        return prop_data;
    }

    // 遍历并找到所有需要传播的测量
    // 注意：我们根据给定状态时刻与更新时刻来切分测量
    let mut i = 0usize;
    while i + 1 < imu_data.len() {
        // 积分区间起点
        if imu_data[i + 1].timestamp > time0 && imu_data[i].timestamp < time0 {
            prop_data.push(interpolate_data(&imu_data[i], &imu_data[i + 1], time0));
            i += 1;
            continue;
        }

        // 积分区间中段
        if imu_data[i].timestamp >= time0 && imu_data[i + 1].timestamp <= time1 {
            prop_data.push(imu_data[i]);
            i += 1;
            continue;
        }

        // 积分区间末端
        if imu_data[i + 1].timestamp > time1 {
            if imu_data[i].timestamp > time1 && i == 0 {
                break;
            } else if imu_data[i].timestamp > time1 {
                prop_data.push(interpolate_data(&imu_data[i - 1], &imu_data[i], time1));
            } else {
                prop_data.push(imu_data[i]);
            }
            if prop_data.last().is_some_and(|d| d.timestamp != time1) {
                prop_data.push(interpolate_data(&imu_data[i], &imu_data[i + 1], time1));
            }
            break;
        }

        i += 1;
    }

    // 确认至少有一个可传播的测量
    if prop_data.is_empty() {
        return prop_data;
    }

    // 遍历并确保没有零 dt 的相邻测量
    // 否则噪声协方差会变成无穷
    let mut j = 0usize;
    while j + 1 < prop_data.len() {
        if (prop_data[j + 1].timestamp - prop_data[j].timestamp).abs() < 1e-12 {
            prop_data.remove(j);
        } else {
            j += 1;
        }
    }

    // 成功 :D
    prop_data
}

/// 给定重力向量，计算惯性系→该重力方向的旋转 `R_GtoI`（Gram–Schmidt，
/// 对照 `InitializerHelper::gram_schmidt`）。
///
/// 假设重力在惯性系中沿竖直方向 `(0,0,1)`；由其找到两个任意切线方向，
/// 正交归一化后拼成旋转矩阵（列依次为 `x`/`y`/`z` 轴）。
#[must_use]
pub fn gram_schmidt(gravity_in_i: &Vector3<f64>) -> Matrix3<f64> {
    // 取与重力（局部 z 轴）正交的向量；每步归一化以获得单位向量
    let z_axis = gravity_in_i / gravity_in_i.norm();
    let (x_axis, y_axis) = {
        let e_1 = Vector3::new(1.0, 0.0, 0.0);
        let e_2 = Vector3::new(0.0, 1.0, 0.0);
        let inner1 = e_1.dot(&z_axis) / z_axis.norm();
        let inner2 = e_2.dot(&z_axis) / z_axis.norm();
        if inner1.abs() < inner2.abs() {
            let x_axis = (z_axis.cross(&e_1)).normalize();
            (x_axis, (z_axis.cross(&x_axis)).normalize())
        } else {
            let x_axis = (z_axis.cross(&e_2)).normalize();
            (x_axis, (z_axis.cross(&x_axis)).normalize())
        }
    };

    // 从（重力仅沿 z 轴的）全局系到局部系的旋转
    let mut r = Matrix3::<f64>::zeros();
    r.set_column(0, &x_axis);
    r.set_column(1, &y_axis);
    r.set_column(2, &z_axis);
    r
}

/// 计算约束优化二次问题的多项式系数（对照
/// `InitializerHelper::compute_dongsi_coeff`）。
///
/// 输入 `d` 为 3×3（重力方向在传感器系），`d_vec` 为 3×1；`gravity_mag` 为
/// 重力大小。返回 7×1 系数向量，`coeff[6]` 为最高次项系数、`coeff[0] = 1`
/// （对照 C++ 直接展开）。
// 系数为 C++ 直接展开的 ~200 行符号表达式（忠实照搬），故 `too_many_lines` 予以允许。
#[allow(clippy::too_many_lines)]
#[must_use]
pub fn compute_dongsi_coeff(
    d: &DMatrix<f64>,
    d_vec: &DMatrix<f64>,
    gravity_mag: f64,
) -> DVector<f64> {
    let d1_1 = d[(0, 0)];
    let d1_2 = d[(0, 1)];
    let d1_3 = d[(0, 2)];
    let d2_1 = d[(1, 0)];
    let d2_2 = d[(1, 1)];
    let d2_3 = d[(1, 2)];
    let d3_1 = d[(2, 0)];
    let d3_2 = d[(2, 1)];
    let d3_3 = d[(2, 2)];
    let d1 = d_vec[(0, 0)];
    let d2 = d_vec[(1, 0)];
    let d3 = d_vec[(2, 0)];
    let g = gravity_mag;

    // 我们用平方做 x^2 的代换
    let d1_1_sq = d1_1 * d1_1;
    let d1_2_sq = d1_2 * d1_2;
    let d1_3_sq = d1_3 * d1_3;
    let d2_1_sq = d2_1 * d2_1;
    let d2_2_sq = d2_2 * d2_2;
    let d2_3_sq = d2_3 * d2_3;
    let d3_1_sq = d3_1 * d3_1;
    let d3_2_sq = d3_2 * d3_2;
    let d3_3_sq = d3_3 * d3_3;
    let d1_sq = d1 * d1;
    let d2_sq = d2 * d2;
    let d3_sq = d3 * d3;
    let g_sq = g * g;

    // 计算系数
    let mut coeff = DVector::<f64>::zeros(7);
    coeff[0] = 1.0;
    coeff[1] = -(2.0 * d1_1 * g_sq + 2.0 * d2_2 * g_sq + 2.0 * d3_3 * g_sq) / g_sq;
    coeff[2] = -(d1_sq + d2_sq + d3_sq
        - d1_1_sq * g_sq
        - d2_2_sq * g_sq
        - d3_3_sq * g_sq
        - 4.0 * d1_1 * d2_2 * g_sq
        + 2.0 * d1_2 * d2_1 * g_sq
        - 4.0 * d1_1 * d3_3 * g_sq
        + 2.0 * d1_3 * d3_1 * g_sq
        - 4.0 * d2_2 * d3_3 * g_sq
        + 2.0 * d2_3 * d3_2 * g_sq)
        / g_sq;
    coeff[3] = (2.0 * d1_1 * d2_sq
        + 2.0 * d1_1 * d3_sq
        + 2.0 * d2_2 * d1_sq
        + 2.0 * d2_2 * d3_sq
        + 2.0 * d3_3 * d1_sq
        + 2.0 * d3_3 * d2_sq
        - 2.0 * d1_1 * d2_2_sq * g_sq
        - 2.0 * d1_1_sq * d2_2 * g_sq
        - 2.0 * d1_1 * d3_3_sq * g_sq
        - 2.0 * d1_1_sq * d3_3 * g_sq
        - 2.0 * d2_2 * d3_3_sq * g_sq
        - 2.0 * d2_2_sq * d3_3 * g_sq
        - 2.0 * d1_2 * d1 * d2
        - 2.0 * d1_3 * d1 * d3
        - 2.0 * d2_1 * d1 * d2
        - 2.0 * d2_3 * d2 * d3
        - 2.0 * d3_1 * d1 * d3
        - 2.0 * d3_2 * d2 * d3
        + 2.0 * d1_1 * d1_2 * d2_1 * g_sq
        + 2.0 * d1_1 * d1_3 * d3_1 * g_sq
        + 2.0 * d1_2 * d2_1 * d2_2 * g_sq
        - 8.0 * d1_1 * d2_2 * d3_3 * g_sq
        + 4.0 * d1_1 * d2_3 * d3_2 * g_sq
        + 4.0 * d1_2 * d2_1 * d3_3 * g_sq
        - 2.0 * d1_2 * d2_3 * d3_1 * g_sq
        - 2.0 * d1_3 * d2_1 * d3_2 * g_sq
        + 4.0 * d1_3 * d2_2 * d3_1 * g_sq
        + 2.0 * d1_3 * d3_1 * d3_3 * g_sq
        + 2.0 * d2_2 * d2_3 * d3_2 * g_sq
        + 2.0 * d2_3 * d3_2 * d3_3 * g_sq)
        / g_sq;
    coeff[4] = (d1_1_sq * d2_2_sq * g_sq + 4.0 * d1_1_sq * d2_2 * d3_3 * g_sq
        - 2.0 * d1_1_sq * d2_3 * d3_2 * g_sq
        + d1_1_sq * d3_3_sq * g_sq
        - d1_1_sq * d2_sq
        - d1_1_sq * d3_sq
        - 2.0 * d1_1 * d1_2 * d2_1 * d2_2 * g_sq
        - 4.0 * d1_1 * d1_2 * d2_1 * d3_3 * g_sq
        + 2.0 * d1_1 * d1_2 * d2_3 * d3_1 * g_sq
        + d1_1 * d1_2 * d1 * d2
        + 2.0 * d1_1 * d1_3 * d2_1 * d3_2 * g_sq
        - 4.0 * d1_1 * d1_3 * d2_2 * d3_1 * g_sq
        - 2.0 * d1_1 * d1_3 * d3_1 * d3_3 * g_sq
        + d1_1 * d1_3 * d1 * d3
        + d1_1 * d2_1 * d1 * d2
        + 4.0 * d1_1 * d2_2_sq * d3_3 * g_sq
        - 4.0 * d1_1 * d2_2 * d2_3 * d3_2 * g_sq
        + 4.0 * d1_1 * d2_2 * d3_3_sq * g_sq
        - 4.0 * d1_1 * d2_2 * d3_sq
        - 4.0 * d1_1 * d2_3 * d3_2 * d3_3 * g_sq
        + 4.0 * d1_1 * d2_3 * d2 * d3
        + d1_1 * d3_1 * d1 * d3
        + 4.0 * d1_1 * d3_2 * d2 * d3
        - 4.0 * d1_1 * d3_3 * d2_sq
        + d1_2_sq * d2_1_sq * g_sq
        + 2.0 * d1_2 * d1_3 * d2_1 * d3_1 * g_sq
        - 4.0 * d1_2 * d2_1 * d2_2 * d3_3 * g_sq
        + 2.0 * d1_2 * d2_1 * d2_3 * d3_2 * g_sq
        - 2.0 * d1_2 * d2_1 * d3_3_sq * g_sq
        - d1_2 * d2_1 * d1_sq
        - d1_2 * d2_1 * d2_sq
        + 2.0 * d1_2 * d2_1 * d3_sq
        + 2.0 * d1_2 * d2_2 * d2_3 * d3_1 * g_sq
        + d1_2 * d2_2 * d1 * d2
        + 2.0 * d1_2 * d2_3 * d3_1 * d3_3 * g_sq
        - 3.0 * d1_2 * d2_3 * d1 * d3
        - 3.0 * d1_2 * d3_1 * d2 * d3
        + 4.0 * d1_2 * d3_3 * d1 * d2
        + d1_3_sq * d3_1_sq * g_sq
        + 2.0 * d1_3 * d2_1 * d2_2 * d3_2 * g_sq
        + 2.0 * d1_3 * d2_1 * d3_2 * d3_3 * g_sq
        - 3.0 * d1_3 * d2_1 * d2 * d3
        - 2.0 * d1_3 * d2_2_sq * d3_1 * g_sq
        - 4.0 * d1_3 * d2_2 * d3_1 * d3_3 * g_sq
        + 4.0 * d1_3 * d2_2 * d1 * d3
        + 2.0 * d1_3 * d2_3 * d3_1 * d3_2 * g_sq
        - d1_3 * d3_1 * d1_sq
        + 2.0 * d1_3 * d3_1 * d2_sq
        - d1_3 * d3_1 * d3_sq
        - 3.0 * d1_3 * d3_2 * d1 * d2
        + d1_3 * d3_3 * d1 * d3
        + d2_1 * d2_2 * d1 * d2
        - 3.0 * d2_1 * d3_2 * d1 * d3
        + 4.0 * d2_1 * d3_3 * d1 * d2
        + d2_2_sq * d3_3_sq * g_sq
        - d2_2_sq * d1_sq
        - d2_2_sq * d3_sq
        - 2.0 * d2_2 * d2_3 * d3_2 * d3_3 * g_sq
        + d2_2 * d2_3 * d2 * d3
        + 4.0 * d2_2 * d3_1 * d1 * d3
        + d2_2 * d3_2 * d2 * d3
        - 4.0 * d2_2 * d3_3 * d1_sq
        + d2_3_sq * d3_2_sq * g_sq
        - 3.0 * d2_3 * d3_1 * d1 * d2
        + 2.0 * d2_3 * d3_2 * d1_sq
        - d2_3 * d3_2 * d2_sq
        - d2_3 * d3_2 * d3_sq
        + d2_3 * d3_3 * d2 * d3
        + d3_1 * d3_3 * d1 * d3
        + d3_2 * d3_3 * d2 * d3
        - d3_3_sq * d1_sq
        - d3_3_sq * d2_sq)
        / g_sq;
    coeff[5] = -(2.0 * d1_1_sq * d2_2_sq * d3_3 * g_sq - 2.0 * d1_1_sq * d2_2 * d2_3 * d3_2 * g_sq
        + 2.0 * d1_1_sq * d2_2 * d3_3_sq * g_sq
        - 2.0 * d1_1_sq * d2_2 * d3_sq
        - 2.0 * d1_1_sq * d2_3 * d3_2 * d3_3 * g_sq
        + 2.0 * d1_1_sq * d2_3 * d2 * d3
        + 2.0 * d1_1_sq * d3_2 * d2 * d3
        - 2.0 * d1_1_sq * d3_3 * d2_sq
        - 4.0 * d1_1 * d1_2 * d2_1 * d2_2 * d3_3 * g_sq
        + 2.0 * d1_1 * d1_2 * d2_1 * d2_3 * d3_2 * g_sq
        - 2.0 * d1_1 * d1_2 * d2_1 * d3_3_sq * g_sq
        + 2.0 * d1_1 * d1_2 * d2_1 * d3_sq
        + 2.0 * d1_1 * d1_2 * d2_2 * d2_3 * d3_1 * g_sq
        + 2.0 * d1_1 * d1_2 * d2_3 * d3_1 * d3_3 * g_sq
        - 2.0 * d1_1 * d1_2 * d2_3 * d1 * d3
        - 2.0 * d1_1 * d1_2 * d3_1 * d2 * d3
        + 2.0 * d1_1 * d1_2 * d3_3 * d1 * d2
        + 2.0 * d1_1 * d1_3 * d2_1 * d2_2 * d3_2 * g_sq
        + 2.0 * d1_1 * d1_3 * d2_1 * d3_2 * d3_3 * g_sq
        - 2.0 * d1_1 * d1_3 * d2_1 * d2 * d3
        - 2.0 * d1_1 * d1_3 * d2_2_sq * d3_1 * g_sq
        - 4.0 * d1_1 * d1_3 * d2_2 * d3_1 * d3_3 * g_sq
        + 2.0 * d1_1 * d1_3 * d2_2 * d1 * d3
        + 2.0 * d1_1 * d1_3 * d2_3 * d3_1 * d3_2 * g_sq
        + 2.0 * d1_1 * d1_3 * d3_1 * d2_sq
        - 2.0 * d1_1 * d1_3 * d3_2 * d1 * d2
        - 2.0 * d1_1 * d2_1 * d3_2 * d1 * d3
        + 2.0 * d1_1 * d2_1 * d3_3 * d1 * d2
        + 2.0 * d1_1 * d2_2_sq * d3_3_sq * g_sq
        - 2.0 * d1_1 * d2_2_sq * d3_sq
        - 4.0 * d1_1 * d2_2 * d2_3 * d3_2 * d3_3 * g_sq
        + 2.0 * d1_1 * d2_2 * d2_3 * d2 * d3
        + 2.0 * d1_1 * d2_2 * d3_1 * d1 * d3
        + 2.0 * d1_1 * d2_2 * d3_2 * d2 * d3
        + 2.0 * d1_1 * d2_3_sq * d3_2_sq * g_sq
        - 2.0 * d1_1 * d2_3 * d3_1 * d1 * d2
        - 2.0 * d1_1 * d2_3 * d3_2 * d2_sq
        - 2.0 * d1_1 * d2_3 * d3_2 * d3_sq
        + 2.0 * d1_1 * d2_3 * d3_3 * d2 * d3
        + 2.0 * d1_1 * d3_2 * d3_3 * d2 * d3
        - 2.0 * d1_1 * d3_3_sq * d2_sq
        + 2.0 * d1_2_sq * d2_1_sq * d3_3 * g_sq
        - 2.0 * d1_2_sq * d2_1 * d2_3 * d3_1 * g_sq
        - 2.0 * d1_2 * d1_3 * d2_1_sq * d3_2 * g_sq
        + 2.0 * d1_2 * d1_3 * d2_1 * d2_2 * d3_1 * g_sq
        + 2.0 * d1_2 * d1_3 * d2_1 * d3_1 * d3_3 * g_sq
        - 2.0 * d1_2 * d1_3 * d2_3 * d3_1_sq * g_sq
        - 2.0 * d1_2 * d2_1 * d2_2 * d3_3_sq * g_sq
        + 2.0 * d1_2 * d2_1 * d2_2 * d3_sq
        + 2.0 * d1_2 * d2_1 * d2_3 * d3_2 * d3_3 * g_sq
        - 2.0 * d1_2 * d2_1 * d3_3 * d1_sq
        - 2.0 * d1_2 * d2_1 * d3_3 * d2_sq
        + 2.0 * d1_2 * d2_2 * d2_3 * d3_1 * d3_3 * g_sq
        - 2.0 * d1_2 * d2_2 * d2_3 * d1 * d3
        - 2.0 * d1_2 * d2_2 * d3_1 * d2 * d3
        + 2.0 * d1_2 * d2_2 * d3_3 * d1 * d2
        - 2.0 * d1_2 * d2_3_sq * d3_1 * d3_2 * g_sq
        + 2.0 * d1_2 * d2_3 * d3_1 * d1_sq
        + 2.0 * d1_2 * d2_3 * d3_1 * d2_sq
        + 2.0 * d1_2 * d2_3 * d3_1 * d3_sq
        - 2.0 * d1_2 * d2_3 * d3_3 * d1 * d3
        - 2.0 * d1_2 * d3_1 * d3_3 * d2 * d3
        + 2.0 * d1_2 * d3_3_sq * d1 * d2
        - 2.0 * d1_3_sq * d2_1 * d3_1 * d3_2 * g_sq
        + 2.0 * d1_3_sq * d2_2 * d3_1_sq * g_sq
        + 2.0 * d1_3 * d2_1 * d2_2 * d3_2 * d3_3 * g_sq
        - 2.0 * d1_3 * d2_1 * d2_2 * d2 * d3
        - 2.0 * d1_3 * d2_1 * d2_3 * d3_2_sq * g_sq
        + 2.0 * d1_3 * d2_1 * d3_2 * d1_sq
        + 2.0 * d1_3 * d2_1 * d3_2 * d2_sq
        + 2.0 * d1_3 * d2_1 * d3_2 * d3_sq
        - 2.0 * d1_3 * d2_1 * d3_3 * d2 * d3
        - 2.0 * d1_3 * d2_2_sq * d3_1 * d3_3 * g_sq
        + 2.0 * d1_3 * d2_2_sq * d1 * d3
        + 2.0 * d1_3 * d2_2 * d2_3 * d3_1 * d3_2 * g_sq
        - 2.0 * d1_3 * d2_2 * d3_1 * d1_sq
        - 2.0 * d1_3 * d2_2 * d3_1 * d3_sq
        - 2.0 * d1_3 * d2_2 * d3_2 * d1 * d2
        + 2.0 * d1_3 * d2_2 * d3_3 * d1 * d3
        + 2.0 * d1_3 * d3_1 * d3_3 * d2_sq
        - 2.0 * d1_3 * d3_2 * d3_3 * d1 * d2
        - 2.0 * d2_1 * d2_2 * d3_2 * d1 * d3
        + 2.0 * d2_1 * d2_2 * d3_3 * d1 * d2
        - 2.0 * d2_1 * d3_2 * d3_3 * d1 * d3
        + 2.0 * d2_1 * d3_3_sq * d1 * d2
        + 2.0 * d2_2_sq * d3_1 * d1 * d3
        - 2.0 * d2_2_sq * d3_3 * d1_sq
        - 2.0 * d2_2 * d2_3 * d3_1 * d1 * d2
        + 2.0 * d2_2 * d2_3 * d3_2 * d1_sq
        + 2.0 * d2_2 * d3_1 * d3_3 * d1 * d3
        - 2.0 * d2_2 * d3_3_sq * d1_sq
        - 2.0 * d2_3 * d3_1 * d3_3 * d1 * d2
        + 2.0 * d2_3 * d3_2 * d3_3 * d1_sq)
        / g_sq;
    coeff[6] = -(-d1_1_sq * d2_2_sq * d3_3_sq * g_sq
        + d1_1_sq * d2_2_sq * d3_sq
        + 2.0 * d1_1_sq * d2_2 * d2_3 * d3_2 * d3_3 * g_sq
        - d1_1_sq * d2_2 * d2_3 * d2 * d3
        - d1_1_sq * d2_2 * d3_2 * d2 * d3
        - d1_1_sq * d2_3_sq * d3_2_sq * g_sq
        + d1_1_sq * d2_3 * d3_2 * d2_sq
        + d1_1_sq * d2_3 * d3_2 * d3_sq
        - d1_1_sq * d2_3 * d3_3 * d2 * d3
        - d1_1_sq * d3_2 * d3_3 * d2 * d3
        + d1_1_sq * d3_3_sq * d2_sq
        + 2.0 * d1_1 * d1_2 * d2_1 * d2_2 * d3_3_sq * g_sq
        - 2.0 * d1_1 * d1_2 * d2_1 * d2_2 * d3_sq
        - 2.0 * d1_1 * d1_2 * d2_1 * d2_3 * d3_2 * d3_3 * g_sq
        + d1_1 * d1_2 * d2_1 * d2_3 * d2 * d3
        + d1_1 * d1_2 * d2_1 * d3_2 * d2 * d3
        - 2.0 * d1_1 * d1_2 * d2_2 * d2_3 * d3_1 * d3_3 * g_sq
        + d1_1 * d1_2 * d2_2 * d2_3 * d1 * d3
        + d1_1 * d1_2 * d2_2 * d3_1 * d2 * d3
        + 2.0 * d1_1 * d1_2 * d2_3_sq * d3_1 * d3_2 * g_sq
        - d1_1 * d1_2 * d2_3 * d3_1 * d2_sq
        - d1_1 * d1_2 * d2_3 * d3_1 * d3_sq
        - d1_1 * d1_2 * d2_3 * d3_2 * d1 * d2
        + d1_1 * d1_2 * d2_3 * d3_3 * d1 * d3
        + d1_1 * d1_2 * d3_1 * d3_3 * d2 * d3
        - d1_1 * d1_2 * d3_3_sq * d1 * d2
        - 2.0 * d1_1 * d1_3 * d2_1 * d2_2 * d3_2 * d3_3 * g_sq
        + d1_1 * d1_3 * d2_1 * d2_2 * d2 * d3
        + 2.0 * d1_1 * d1_3 * d2_1 * d2_3 * d3_2_sq * g_sq
        - d1_1 * d1_3 * d2_1 * d3_2 * d2_sq
        - d1_1 * d1_3 * d2_1 * d3_2 * d3_sq
        + d1_1 * d1_3 * d2_1 * d3_3 * d2 * d3
        + 2.0 * d1_1 * d1_3 * d2_2_sq * d3_1 * d3_3 * g_sq
        - d1_1 * d1_3 * d2_2_sq * d1 * d3
        - 2.0 * d1_1 * d1_3 * d2_2 * d2_3 * d3_1 * d3_2 * g_sq
        + d1_1 * d1_3 * d2_2 * d3_2 * d1 * d2
        + d1_1 * d1_3 * d2_3 * d3_1 * d2 * d3
        - d1_1 * d1_3 * d2_3 * d3_2 * d1 * d3
        + d1_1 * d1_3 * d3_1 * d3_2 * d2 * d3
        - 2.0 * d1_1 * d1_3 * d3_1 * d3_3 * d2_sq
        + d1_1 * d1_3 * d3_2 * d3_3 * d1 * d2
        + d1_1 * d2_1 * d2_2 * d3_2 * d1 * d3
        - d1_1 * d2_1 * d2_3 * d3_2 * d1 * d2
        + d1_1 * d2_1 * d3_2 * d3_3 * d1 * d3
        - d1_1 * d2_1 * d3_3_sq * d1 * d2
        - d1_1 * d2_2_sq * d3_1 * d1 * d3
        + d1_1 * d2_2 * d2_3 * d3_1 * d1 * d2
        - d1_1 * d2_3 * d3_1 * d3_2 * d1 * d3
        + d1_1 * d2_3 * d3_1 * d3_3 * d1 * d2
        - d1_2_sq * d2_1_sq * d3_3_sq * g_sq
        + d1_2_sq * d2_1_sq * d3_sq
        + 2.0 * d1_2_sq * d2_1 * d2_3 * d3_1 * d3_3 * g_sq
        - d1_2_sq * d2_1 * d2_3 * d1 * d3
        - d1_2_sq * d2_1 * d3_1 * d2 * d3
        - d1_2_sq * d2_3_sq * d3_1_sq * g_sq
        + d1_2_sq * d2_3 * d3_1 * d1 * d2
        + 2.0 * d1_2 * d1_3 * d2_1_sq * d3_2 * d3_3 * g_sq
        - d1_2 * d1_3 * d2_1_sq * d2 * d3
        - 2.0 * d1_2 * d1_3 * d2_1 * d2_2 * d3_1 * d3_3 * g_sq
        + d1_2 * d1_3 * d2_1 * d2_2 * d1 * d3
        - 2.0 * d1_2 * d1_3 * d2_1 * d2_3 * d3_1 * d3_2 * g_sq
        + d1_2 * d1_3 * d2_1 * d3_1 * d2_sq
        + d1_2 * d1_3 * d2_1 * d3_1 * d3_sq
        - d1_2 * d1_3 * d2_1 * d3_3 * d1 * d3
        + 2.0 * d1_2 * d1_3 * d2_2 * d2_3 * d3_1_sq * g_sq
        - d1_2 * d1_3 * d2_2 * d3_1 * d1 * d2
        - d1_2 * d1_3 * d3_1_sq * d2 * d3
        + d1_2 * d1_3 * d3_1 * d3_3 * d1 * d2
        - d1_2 * d2_1_sq * d3_2 * d1 * d3
        + d1_2 * d2_1 * d2_2 * d3_1 * d1 * d3
        + d1_2 * d2_1 * d2_3 * d3_2 * d1_sq
        + d1_2 * d2_1 * d2_3 * d3_2 * d3_sq
        - d1_2 * d2_1 * d2_3 * d3_3 * d2 * d3
        - d1_2 * d2_1 * d3_1 * d3_3 * d1 * d3
        - d1_2 * d2_1 * d3_2 * d3_3 * d2 * d3
        + d1_2 * d2_1 * d3_3_sq * d1_sq
        + d1_2 * d2_1 * d3_3_sq * d2_sq
        - d1_2 * d2_2 * d2_3 * d3_1 * d1_sq
        - d1_2 * d2_2 * d2_3 * d3_1 * d3_sq
        + d1_2 * d2_2 * d2_3 * d3_3 * d1 * d3
        + d1_2 * d2_2 * d3_1 * d3_3 * d2 * d3
        - d1_2 * d2_2 * d3_3_sq * d1 * d2
        + d1_2 * d2_3_sq * d3_1 * d2 * d3
        - d1_2 * d2_3_sq * d3_2 * d1 * d3
        + d1_2 * d2_3 * d3_1_sq * d1 * d3
        - d1_2 * d2_3 * d3_1 * d3_3 * d1_sq
        - d1_2 * d2_3 * d3_1 * d3_3 * d2_sq
        + d1_2 * d2_3 * d3_2 * d3_3 * d1 * d2
        - d1_3_sq * d2_1_sq * d3_2_sq * g_sq
        + 2.0 * d1_3_sq * d2_1 * d2_2 * d3_1 * d3_2 * g_sq
        - d1_3_sq * d2_1 * d3_1 * d2 * d3
        + d1_3_sq * d2_1 * d3_2 * d1 * d3
        - d1_3_sq * d2_2_sq * d3_1_sq * g_sq
        + d1_3_sq * d3_1_sq * d2_sq
        - d1_3_sq * d3_1 * d3_2 * d1 * d2
        + d1_3 * d2_1_sq * d3_2 * d1 * d2
        - d1_3 * d2_1 * d2_2 * d3_1 * d1 * d2
        - d1_3 * d2_1 * d2_2 * d3_2 * d1_sq
        - d1_3 * d2_1 * d2_2 * d3_2 * d3_sq
        + d1_3 * d2_1 * d2_2 * d3_3 * d2 * d3
        + d1_3 * d2_1 * d3_1 * d3_3 * d1 * d2
        + d1_3 * d2_1 * d3_2_sq * d2 * d3
        - d1_3 * d2_1 * d3_2 * d3_3 * d1_sq
        - d1_3 * d2_1 * d3_2 * d3_3 * d2_sq
        + d1_3 * d2_2_sq * d3_1 * d1_sq
        + d1_3 * d2_2_sq * d3_1 * d3_sq
        - d1_3 * d2_2_sq * d3_3 * d1 * d3
        - d1_3 * d2_2 * d2_3 * d3_1 * d2 * d3
        + d1_3 * d2_2 * d2_3 * d3_2 * d1 * d3
        - d1_3 * d2_2 * d3_1 * d3_2 * d2 * d3
        + d1_3 * d2_2 * d3_2 * d3_3 * d1 * d2
        - d1_3 * d2_3 * d3_1_sq * d1 * d2
        + d1_3 * d2_3 * d3_1 * d3_2 * d1_sq
        + d1_3 * d2_3 * d3_1 * d3_2 * d2_sq
        - d1_3 * d2_3 * d3_2_sq * d1 * d2
        + d2_1 * d2_2 * d3_2 * d3_3 * d1 * d3
        - d2_1 * d2_2 * d3_3_sq * d1 * d2
        - d2_1 * d2_3 * d3_2_sq * d1 * d3
        + d2_1 * d2_3 * d3_2 * d3_3 * d1 * d2
        - d2_2_sq * d3_1 * d3_3 * d1 * d3
        + d2_2_sq * d3_3_sq * d1_sq
        + d2_2 * d2_3 * d3_1 * d3_2 * d1 * d3
        + d2_2 * d2_3 * d3_1 * d3_3 * d1 * d2
        - 2.0 * d2_2 * d2_3 * d3_2 * d3_3 * d1_sq
        - d2_3_sq * d3_1 * d3_2 * d1 * d2
        + d2_3_sq * d3_2_sq * d1_sq)
        / g_sq;

    // finally return
    coeff
}

#[cfg(test)]
mod tests {
    // 测试内浮点严格比较属断言意图，予以允许。
    #![allow(clippy::float_cmp)]

    use super::*;

    fn assert_close(a: f64, b: f64, eps: f64) {
        assert!((a - b).abs() < eps, "{a} != {b} (eps={eps})");
    }

    #[test]
    fn interpolate_data_is_linear() {
        let d1 = ImuData {
            timestamp: 0.0,
            wm: Vector3::new(1.0, 2.0, 3.0),
            am: Vector3::new(0.1, 0.2, 0.3),
        };
        let d2 = ImuData {
            timestamp: 1.0,
            wm: Vector3::new(3.0, 4.0, 5.0),
            am: Vector3::new(0.3, 0.4, 0.5),
        };
        let mid = interpolate_data(&d1, &d2, 0.5);
        assert_eq!(mid.timestamp, 0.5);
        assert_close(mid.wm[0], 2.0, 1e-12);
        assert_close(mid.wm[1], 3.0, 1e-12);
        assert_close(mid.wm[2], 4.0, 1e-12);
        assert_close(mid.am[0], 0.2, 1e-12);
        assert_close(mid.am[1], 0.3, 1e-12);
        assert_close(mid.am[2], 0.4, 1e-12);
        // 端点：lambda=1 应当是 imu_2
        let end = interpolate_data(&d1, &d2, 1.0);
        assert_close(end.wm[0], 3.0, 1e-12);
        assert_close(end.am[0], 0.3, 1e-12);
    }

    fn readings(times: &[f64]) -> Vec<ImuData> {
        times
            .iter()
            .map(|&t| ImuData {
                timestamp: t,
                wm: Vector3::new(t, t, t),
                am: Vector3::new(t, t, t),
            })
            .collect()
    }

    #[test]
    fn select_imu_readings_empty_input_returns_empty() {
        assert!(select_imu_readings(&[], 0.0, 1.0).is_empty());
    }

    #[test]
    fn select_imu_readings_boundary_interpolation() {
        // 区间 [0.01, 0.15]，数据在 [0.0, 0.20]
        let data = readings(&[0.0, 0.05, 0.10, 0.15, 0.20]);
        let got = select_imu_readings(&data, 0.01, 0.15);
        // 起点 0.01 用 (0.0,0.05) 插值；中段 [0.05,0.10,0.15]；末端恰好在 0.15
        assert!(!got.is_empty());
        assert_eq!(got[0].timestamp, 0.01);
        // 中段测量被完整保留
        assert!(got.iter().any(|d| d.timestamp == 0.05));
        assert!(got.iter().any(|d| d.timestamp == 0.10));
        assert!(got.iter().any(|d| d.timestamp == 0.15));
        // 升序
        for w in got.windows(2) {
            assert!(w[0].timestamp <= w[1].timestamp);
        }
    }

    #[test]
    fn select_imu_readings_all_before_time1_zero_dt_removed() {
        let data = readings(&[0.0, 0.05, 0.05, 0.10]);
        let got = select_imu_readings(&data, 0.0, 0.10);
        // 零 dt 相邻项 (0.05,0.05) 被剔除，保留恰好落在 time1 的端点
        assert!(!got.is_empty());
        for w in got.windows(2) {
            assert!((w[1].timestamp - w[0].timestamp).abs() >= 1e-12);
        }
    }

    #[test]
    fn gram_schmidt_orthogonal_with_z_along_gravity() {
        let grav = Vector3::new(0.3, -1.0, 0.7);
        let r = gram_schmidt(&grav);
        // 第三列沿重力方向（单位化）
        let z = grav.normalize();
        for k in 0..3 {
            assert_close(r[(k, 2)], z[k], 1e-12);
        }
        // 正交：R^T R = I
        let rt_r = r.transpose() * r;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_close(rt_r[(i, j)], expected, 1e-12);
            }
        }
    }

    #[test]
    fn dongsi_coeff_length_and_leading_small_d() {
        // 平凡输入：D=I, d=0, g=9.81
        // D 为 3×3 单位阵、d_vec 零 → 多项式应退化为与 x^2 相关的简单形式
        let d = DMatrix::<f64>::identity(3, 3);
        let dv = DMatrix::<f64>::zeros(3, 1);
        let c = compute_dongsi_coeff(&d, &dv, 9.81);
        assert_eq!(c.len(), 7);
        assert_close(c[0], 1.0, 1e-12);
        // D=I, d=0：coeff[1] = -(2*g_sq+2*g_sq+2*g_sq)/g_sq = -6
        assert_close(c[1], -6.0, 1e-9);
        // 非平凡随机化 D/d 仅验证长度与首系数；数值正确性由机械转换脚本核对
        let mut d2 = DMatrix::<f64>::zeros(3, 3);
        d2[(0, 0)] = 1.5;
        d2[(1, 1)] = 2.0;
        d2[(2, 2)] = 3.0;
        let dv2 = DMatrix::<f64>::from_row_slice(3, 1, &[0.1, 0.2, 0.3]);
        let c2 = compute_dongsi_coeff(&d2, &dv2, 9.81);
        assert_eq!(c2.len(), 7);
        assert_close(c2[0], 1.0, 1e-12);
    }
}
