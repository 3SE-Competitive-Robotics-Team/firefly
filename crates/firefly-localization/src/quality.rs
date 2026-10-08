//! 地图跟踪可用性；仅消费同一时刻配对的估计与视觉观测，不消费真值。

/// 任务层误差预算；不是概率置信区间或碰撞安全保证。
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
    /// 按当前观测与参数给出定位质量状态。
    #[must_use]
    pub fn status(
        &self,
        timestamp: f64,
        options: &QualityOptions,
    ) -> firefly_pubsub::odom::LocalizationStatus {
        let (visual_timestamp, position_disagreement) = self.observation.unwrap_or((-1., -1.));
        let age = timestamp - visual_timestamp;
        firefly_pubsub::odom::LocalizationStatus {
            timestamp,
            visual_timestamp,
            position_disagreement,
            tracking_ready: self.observation.is_some()
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
    fn delayed_repeated_and_inconsistent_observations_do_not_certify_tracking() {
        let o = QualityOptions::default();
        let mut q = Quality::default();
        assert!(!q.status(1., &o).tracking_ready);
        q.observe(1., 0.1);
        assert!(q.status(2., &o).tracking_ready);
        assert!(!q.status(4.01, &o).tracking_ready);
        q.observe(1., 0.);
        assert!(!q.status(4.01, &o).tracking_ready);
        q.observe(5., 0.6);
        assert!(!q.status(5.1, &o).tracking_ready);
        q.observe(6., 0.1);
        assert!(q.status(6.1, &o).tracking_ready);
        q.observe(f64::NAN, 0.);
        assert!(!q.status(10., &o).tracking_ready);
        assert!(!q.status(5., &o).tracking_ready);
    }
}
