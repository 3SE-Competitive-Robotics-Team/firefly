//! IMU 噪声参数（对照 `OpenVINS` `ov_msckf/utils/NoiseManager.h`）。
//!
//! 存储连续时间的噪声密度（white noise / random walk）及其平方（协方差）。
//! 离散化时（见 `crate::propagation`）再按 `sigma_*_2 / dt` 折算为离散协方差。

/// IMU 连续时间噪声参数（对照 `NoiseManager` 的全部字段与别名）。
#[derive(Debug, Clone)]
pub struct ImuNoise {
    /// 陀螺白噪声 `rad/s/√Hz`（gyroscope white noise）。
    pub sigma_w: f64,
    /// 陀螺白噪声协方差 `sigma_w²`。
    pub sigma_w_2: f64,
    /// 陀螺随机游走 `rad/s²/√Hz`（gyroscope random walk）。
    pub sigma_wb: f64,
    /// 陀螺随机游走协方差 `sigma_wb²`。
    pub sigma_wb_2: f64,
    /// 加速度计白噪声 `m/s²/√Hz`（accelerometer white noise）。
    pub sigma_a: f64,
    /// 加速度计白噪声协方差 `sigma_a²`。
    pub sigma_a_2: f64,
    /// 加速度计随机游走 `m/s³/√Hz`（accelerometer random walk）。
    pub sigma_ab: f64,
    /// 加速度计随机游走协方差 `sigma_ab²`。
    pub sigma_ab_2: f64,
}

impl ImuNoise {
    /// 由四组 `sigma` 构造，并自动刷新对应的平方项。
    #[must_use]
    #[allow(clippy::similar_names)] // sigma_w/sigma_wb/sigma_a/sigma_ab 命名与 OpenVINS 一致
    pub fn new(sigma_w: f64, sigma_wb: f64, sigma_a: f64, sigma_ab: f64) -> Self {
        Self {
            sigma_w,
            sigma_w_2: sigma_w * sigma_w,
            sigma_wb,
            sigma_wb_2: sigma_wb * sigma_wb,
            sigma_a,
            sigma_a_2: sigma_a * sigma_a,
            sigma_ab,
            sigma_ab_2: sigma_ab * sigma_ab,
        }
    }

    /// 重新计算四个平方项（供外部修改原始 `sigma` 后调用，对应 `NoiseManager` 构造函数）。
    pub fn recompute_squares(&mut self) {
        self.sigma_w_2 = self.sigma_w * self.sigma_w;
        self.sigma_wb_2 = self.sigma_wb * self.sigma_wb;
        self.sigma_a_2 = self.sigma_a * self.sigma_a;
        self.sigma_ab_2 = self.sigma_ab * self.sigma_ab;
    }
}

/// 默认噪声，数值对照 `NoiseManager.h` 的成员初始化：
/// 陀螺白噪声 `1.6968e-04`、随机游走 `1.9393e-05`、
/// 加速度计白噪声 `2.0000e-3`、随机游走 `3.0000e-03`。
impl Default for ImuNoise {
    fn default() -> Self {
        Self::new(1.6968e-04, 1.9393e-05, 2.0000e-3, 3.0000e-03)
    }
}

#[cfg(test)]
mod tests {
    // 数值测试断言多采用精确值 assert_eq!；此处显式豁免 float_cmp。
    #![allow(clippy::float_cmp)]

    use super::*;

    #[test]
    fn default_values_match_noise_manager() {
        // 对照 NoiseManager.h 成员初值
        let n = ImuNoise::default();
        assert_eq!(n.sigma_w, 1.6968e-04);
        assert_eq!(n.sigma_wb, 1.9393e-05);
        assert_eq!(n.sigma_a, 2.0000e-3);
        assert_eq!(n.sigma_ab, 3.0000e-03);
        // 平方项 = std::pow(sigma, 2)
        assert_eq!(n.sigma_w_2, (1.6968e-04_f64).powi(2));
        assert_eq!(n.sigma_wb_2, (1.9393e-05_f64).powi(2));
        assert_eq!(n.sigma_a_2, (2.0000e-3_f64).powi(2));
        assert_eq!(n.sigma_ab_2, (3.0000e-03_f64).powi(2));
    }

    #[test]
    fn new_recomputes_squares() {
        let n = ImuNoise::new(0.01, 0.02, 0.03, 0.04);
        assert!((n.sigma_w_2 - 0.01f64.powi(2)).abs() < 1e-18);
        assert!((n.sigma_a_2 - 0.03f64.powi(2)).abs() < 1e-18);
        assert!((n.sigma_wb_2 - 0.02f64.powi(2)).abs() < 1e-18);
        assert!((n.sigma_ab_2 - 0.04f64.powi(2)).abs() < 1e-18);
    }

    #[test]
    fn recompute_squares_refreshes_after_mutation() {
        // 显式构造再加偏置，避免 field_reassign_with_default。
        let mut n = ImuNoise {
            sigma_a: 5.0,
            ..ImuNoise::default()
        };
        n.recompute_squares();
        assert!((n.sigma_a_2 - 25.0).abs() < 1e-12);
    }
}
