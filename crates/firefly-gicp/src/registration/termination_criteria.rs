//! 收敛判定（对照 `registration/termination_criteria.hpp`）。

use nalgebra::Vector6;

/// 终止判定（对照 `TerminationCriteria`）。
#[derive(Clone, Copy, Debug)]
pub struct TerminationCriteria {
    /// 平移容差 `translation_eps`（默认 1e-3）。
    pub translation_eps: f64,
    /// 旋转容差 `rotation_eps`（默认 0.1°）。
    pub rotation_eps: f64,
}

impl Default for TerminationCriteria {
    fn default() -> Self {
        Self {
            translation_eps: 1e-3,
            rotation_eps: 0.1 * std::f64::consts::PI / 180.0,
        }
    }
}

impl TerminationCriteria {
    /// 是否收敛（对照 `converged(delta)`）。
    ///
    /// `delta = [rx, ry, rz, tx, ty, tz]`。
    pub fn converged(&self, delta: &Vector6<f64>) -> bool {
        let rot = delta.fixed_rows::<3>(0).norm();
        let trans = delta.fixed_rows::<3>(3).norm();
        rot <= self.rotation_eps && trans <= self.translation_eps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converged_threshold() {
        let c = TerminationCriteria::default();
        let mut delta = Vector6::zeros();
        assert!(c.converged(&delta));
        delta[0] = 0.2 * std::f64::consts::PI / 180.0; // 0.2° > 0.1°
        assert!(!c.converged(&delta));
    }
}
