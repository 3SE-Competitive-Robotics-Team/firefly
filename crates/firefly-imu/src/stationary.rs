//! 静止陀螺均值相对窗口末端零偏的不确定度，供 VIO 与飞控共用。
use nalgebra::{Matrix3, Vector3};

/// 返回三轴均值的协方差，单位 (rad/s)²。
/// 白噪声密度按各采样周期离散化，首样本采用相邻周期；随机游走保留
/// 均值到窗口末端的误差。样本协方差/N 作为模型余量，不能压低标定噪声。
/// 需要至少两个有限、时间严格递增的静止样本；无效输入返回 None。
pub fn gyro_mean_covariance(
    samples: impl ExactSizeIterator<Item = (f64, Vector3<f64>)> + Clone,
    density: f64,
    walk: f64,
) -> Option<Matrix3<f64>> {
    let count = samples.len();
    if count < 2 || !density.is_finite() || density <= 0. || !walk.is_finite() || walk < 0. {
        return None;
    }
    let n = count as f64;
    let mean = samples.clone().map(|(_, gyro)| gyro).sum::<Vector3<f64>>() / n;
    let mut previous = None;
    let mut white = 0.;
    let mut random_walk = 0.;
    let mut scatter = Matrix3::zeros();
    for (i, (time, gyro)) in samples.enumerate() {
        if !time.is_finite() || !gyro.iter().all(|x| x.is_finite()) {
            return None;
        }
        if let Some(last) = previous {
            let dt = time - last;
            if dt <= 0. || !f64::is_finite(dt) {
                return None;
            }
            white += density.powi(2) / dt * if i == 1 { 2. } else { 1. };
            random_walk += walk.powi(2) * dt * (i as f64 / n).powi(2);
        }
        previous = Some(time);
        let residual = gyro - mean;
        scatter += residual * residual.transpose();
    }
    let covariance =
        scatter / (n * (n - 1.)) + Matrix3::identity() * (white / n.powi(2) + random_walk);
    covariance
        .iter()
        .all(|x| x.is_finite())
        .then_some(covariance)
}
