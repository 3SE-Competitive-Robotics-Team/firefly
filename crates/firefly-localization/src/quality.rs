//! 评估已接受的外部位姿观测的新鲜度，以及观测位置与同一时刻融合位置的分歧。

/// 外部位姿观测的最大年龄和融合后位置分歧上限。
#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct QualityOptions {
    /// 视觉测量最大年龄，秒；重复和延迟送达不得延长有效期。
    pub max_visual_age: f64,
    /// 融合后位姿与同刻视觉观测的最大位置分歧，米。
    pub max_position_disagreement: f64,
}
impl Default for QualityOptions {
    fn default() -> Self {
        Self {
            max_visual_age: 3.,
            max_position_disagreement: 0.5,
        }
    }
}
impl QualityOptions {
    /// 参数是否有效（有限且为正）。
    #[must_use]
    pub fn valid(&self) -> bool {
        [self.max_visual_age, self.max_position_disagreement]
            .iter()
            .all(|x| x.is_finite() && *x > 0.)
    }
}
/// 最近一次外部位姿观测的时间戳、位置分歧，以及观测是否新鲜且一致。
pub struct ObservationStatus {
    pub visual_timestamp: f64,
    pub position_disagreement: f64,
    pub recent_consistent: bool,
}
#[derive(Default)]
pub struct Quality {
    observation: Option<(f64, f64)>,
}
impl Quality {
    pub fn observe(&mut self, timestamp: f64, disagreement: f64) {
        if timestamp.is_finite()
            && timestamp >= 0.
            && disagreement.is_finite()
            && disagreement >= 0.
            && self.observation.is_none_or(|(last, _)| timestamp > last)
        {
            self.observation = Some((timestamp, disagreement));
        }
    }
    /// 按估计时刻判断观测年龄和位置分歧是否均在阈值内。尚无观测时，时间戳和位置分歧均为 -1。
    #[must_use]
    pub fn status(&self, timestamp: f64, options: &QualityOptions) -> ObservationStatus {
        let (visual_timestamp, position_disagreement) = self.observation.unwrap_or((-1., -1.));
        let age = timestamp - visual_timestamp;
        ObservationStatus {
            visual_timestamp,
            position_disagreement,
            recent_consistent: self.observation.is_some()
                && timestamp.is_finite()
                && age >= 0.
                && age <= options.max_visual_age
                && position_disagreement <= options.max_position_disagreement,
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn delayed_repeated_and_inconsistent_observations_remain_diagnostic() {
        let o = QualityOptions::default();
        let mut q = Quality::default();
        assert!(!q.status(1., &o).recent_consistent);
        q.observe(1., 0.1);
        assert!(q.status(2., &o).recent_consistent);
        assert!(!q.status(4.01, &o).recent_consistent);
        q.observe(1., 0.);
        assert!(!q.status(4.01, &o).recent_consistent);
        q.observe(5., 0.6);
        assert!(!q.status(5.1, &o).recent_consistent);
        q.observe(6., 0.1);
        assert!(q.status(6.1, &o).recent_consistent);
        q.observe(f64::NAN, 0.);
        assert!(!q.status(10., &o).recent_consistent);
        assert!(!q.status(5., &o).recent_consistent);
    }
}
