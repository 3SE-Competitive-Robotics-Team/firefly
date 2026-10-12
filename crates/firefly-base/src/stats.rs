//! 估计器共用的统计门限。

/// chi2 95% 分位数（自由度 1..=30 的四位小数表值，`>30` 用 Wilson–Hilferty 近似）。
const CHI2_95_TABLE: [f64; 30] = [
    3.8415, 5.9915, 7.8147, 9.4877, 11.0705, 12.5916, 14.0671, 15.5073, 16.9190, 18.3070, 19.6751,
    21.0261, 22.3620, 23.6848, 24.9958, 26.2962, 27.5871, 28.8693, 30.1435, 31.4104, 32.6706,
    33.9244, 35.1725, 36.4150, 37.6525, 38.8851, 40.1133, 41.3372, 42.5570, 43.7730,
];

/// chi2 95% 分位数（对照 `UpdaterMSCKF` 构造中的 `boost::math::chi_squared` 表）。
#[must_use]
pub fn chi2_95(dof: usize) -> f64 {
    if dof == 0 {
        return 0.0;
    }
    if dof <= CHI2_95_TABLE.len() {
        return CHI2_95_TABLE[dof - 1];
    }
    // Wilson–Hilferty 近似：χ²_α(ν) ≈ ν(1 − 2/(9ν) + z_α·√(2/(9ν)))³
    // z_0.95 = 1.6448536269514722
    let nu = dof as f64;
    let t = 1.0 - 2.0 / (9.0 * nu) + 1.644_853_626_951_472_2 * (2.0 / (9.0 * nu)).sqrt();
    nu * t * t * t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chi2_table_matches_known_values() {
        assert_eq!(chi2_95(0), 0.0);
        assert!((chi2_95(30) - 43.7730).abs() < 1e-12);
        assert!((chi2_95(31) - 44.9853).abs() < 0.02);
        // 已知 χ² 95% 分位数：ν=1 → 3.8415，ν=2 → 5.9915，ν=10 → 18.3070
        assert!((chi2_95(1) - 3.8415).abs() < 1e-3);
        assert!((chi2_95(2) - 5.9915).abs() < 1e-3);
        assert!((chi2_95(10) - 18.3070).abs() < 1e-3);
    }

    #[test]
    fn chi2_approx_for_large_dof() {
        // ν=100 的 95% 分位数 ≈ 124.34（Wilson–Hilferty 精度 ~1e-2）
        let v = chi2_95(100);
        assert!((v - 124.34).abs() < 0.1, "chi2(100) = {v}");
        // 单调性
        assert!(chi2_95(100) > chi2_95(50));
    }

    #[test]
    fn chi2_monotonic() {
        for n in 1..60 {
            assert!(chi2_95(n + 1) > chi2_95(n), "dof={n}");
        }
    }
}
