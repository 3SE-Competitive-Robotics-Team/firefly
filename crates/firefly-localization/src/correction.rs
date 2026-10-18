//! 地图坐标切换策略：对齐变化超过门限才提交新坐标系。
use firefly_error::{Error, ErrorKind, Result};

/// 对齐修正的提交门限，长度为米，角度为弧度。
///
/// 低于门限的修正只更新 map←odom 对齐，不递增修订号：下游按旧修订号复用的
/// 局部深度地图与在途参考误差保持有界，避免为厘米级修正清空融合、重建地图。
#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CorrectionOptions {
    /// 位置门限；RootSLAM 以 10 cm 漂移决定是否修正全图。
    pub commit_translation: f64,
    /// 航向门限；10 cm 在 5 m 测距处对应约 1°。
    pub commit_rotation: f64,
}
impl Default for CorrectionOptions {
    fn default() -> Self {
        Self {
            commit_translation: 0.1,
            commit_rotation: 1_f64.to_radians(),
        }
    }
}
impl CorrectionOptions {
    /// 同一机体位姿在两次对齐下的地图系差异是否超过提交门限。
    ///
    /// 差异按机体位置计算，即下游局部地图与在途参考实际经历的跳变。
    #[must_use]
    pub fn committed(
        &self,
        now: &firefly_base::RigidTransform,
        before: &firefly_base::RigidTransform,
    ) -> bool {
        let shift =
            (now.isometry().translation.vector - before.isometry().translation.vector).norm();
        let turn = now
            .isometry()
            .rotation
            .angle_to(&before.isometry().rotation);
        shift > self.commit_translation || turn > self.commit_rotation
    }
    /// # Errors
    /// 门限非有限或为负。
    pub fn validate(&self) -> Result<()> {
        if [self.commit_translation, self.commit_rotation]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.)
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid alignment correction thresholds",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_base::{FrameId, RigidTransform};

    fn map_from_body(translation: [f64; 3], rotation: [f64; 4]) -> RigidTransform {
        RigidTransform::from_parts(FrameId::MAP, FrameId::BODY, translation, rotation).unwrap()
    }

    #[test]
    fn defaults_match_paper_drift_threshold() {
        let options = CorrectionOptions::default();
        assert!((options.commit_translation - 0.1).abs() < 1e-12);
        assert!((options.commit_rotation - 1_f64.to_radians()).abs() < 1e-12);
        assert!(options.validate().is_ok());
        assert!(
            CorrectionOptions {
                commit_translation: -1.,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn commit_needs_paper_threshold_in_translation_or_rotation() {
        let options = CorrectionOptions::default();
        let base = map_from_body([0.; 3], [0., 0., 0., 1.]);
        assert!(!options.committed(&map_from_body([0.09, 0., 0.], [0., 0., 0., 1.]), &base));
        assert!(options.committed(&map_from_body([0.11, 0., 0.], [0., 0., 0., 1.]), &base));
        let half = 0.5 * 1.2_f64.to_radians();
        assert!(options.committed(
            &map_from_body([0.; 3], [0., 0., half.sin(), half.cos()]),
            &base
        ));
        let half = 0.5 * 0.8_f64.to_radians();
        assert!(!options.committed(
            &map_from_body([0.; 3], [0., 0., half.sin(), half.cos()]),
            &base
        ));
    }
}
