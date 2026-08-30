//! 两测量模型共用的外点剔除与鲁棒核。
//!
//! - 卡方逐点检验（论文 VII-A 末段 + `voxel_map.cpp:737` 的
//!   `dis_to_plane < sigma_num·√σ` 门控）：残差超过 `σ·√(R)` 的测量剔除；
//! - Huber 核（可配 `δ`，`δ = ∞` 时退化为最小二乘）。

/// 卡方检验结果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateVerdict {
    /// 通过（内点）。
    Inlier,
    /// 拒绝（外点）。
    Outlier,
}

/// 卡方门控：`|z| > sigma_num·√r` 判外点（对照 `voxel_map.cpp:737`）。
#[must_use]
pub fn chi2_gate(z: f64, r: f64, sigma_num: f64) -> GateVerdict {
    if r <= 0.0 || z.abs() <= sigma_num * r.sqrt() {
        GateVerdict::Inlier
    } else {
        GateVerdict::Outlier
    }
}

/// Huber 权重（`δ` 为阈值）。
///
/// `|r| ≤ δ` 权重 1；否则 `δ/|r|`。
#[must_use]
pub fn huber_weight(r: f64, delta: f64) -> f64 {
    if !delta.is_finite() {
        return 1.0;
    }
    let a = r.abs();
    if a <= delta { 1.0 } else { delta / a }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chi2_gate_rejects_large_residual() {
        assert_eq!(chi2_gate(0.1, 0.01, 3.0), GateVerdict::Inlier);
        assert_eq!(chi2_gate(0.5, 0.01, 3.0), GateVerdict::Outlier);
        // 零方差永不拒绝（无信息测量）
        assert_eq!(chi2_gate(1e9, 0.0, 3.0), GateVerdict::Inlier);
    }

    #[test]
    fn huber_weights() {
        assert!((huber_weight(0.5, 1.0) - 1.0).abs() < 1e-12);
        assert!((huber_weight(2.0, 1.0) - 0.5).abs() < 1e-12);
        // δ=∞：恒为 1
        assert!((huber_weight(1e6, f64::INFINITY) - 1.0).abs() < 1e-12);
    }
}
