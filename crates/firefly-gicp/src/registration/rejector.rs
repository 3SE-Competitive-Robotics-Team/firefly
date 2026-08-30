//! 对应关系剔除（对照 `registration/rejector.hpp`）。

use nalgebra::Matrix4;

use crate::points::traits::PointCloudTrait;

/// 剔除接口：返回 `true` 表示剔除该对应。
pub trait CorrespondenceRejector {
    /// 是否剔除。
    fn should_reject<Target, Source>(
        &self,
        target: &Target,
        source: &Source,
        t: &Matrix4<f64>,
        target_index: usize,
        source_index: usize,
        sq_dist: f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait;
}

/// 空剔除器（永不剔除，对照 `NullRejector`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct NullRejector;

impl CorrespondenceRejector for NullRejector {
    fn should_reject<Target, Source>(
        &self,
        _target: &Target,
        _source: &Source,
        _t: &Matrix4<f64>,
        _target_index: usize,
        _source_index: usize,
        _sq_dist: f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        false
    }
}

/// 距离剔除器（对照 `DistanceRejector`）。
#[derive(Clone, Copy, Debug)]
pub struct DistanceRejector {
    /// 最大平方距离。
    pub max_dist_sq: f64,
}

impl Default for DistanceRejector {
    fn default() -> Self {
        Self { max_dist_sq: 1.0 }
    }
}

impl CorrespondenceRejector for DistanceRejector {
    fn should_reject<Target, Source>(
        &self,
        _target: &Target,
        _source: &Source,
        _t: &Matrix4<f64>,
        _target_index: usize,
        _source_index: usize,
        sq_dist: f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        sq_dist > self.max_dist_sq
    }
}

// 便捷：允许闭包作剔除器
impl<F> CorrespondenceRejector for F
where
    F: Fn(&Matrix4<f64>, usize, usize, f64) -> bool,
{
    fn should_reject<Target, Source>(
        &self,
        _target: &Target,
        _source: &Source,
        t: &Matrix4<f64>,
        target_index: usize,
        source_index: usize,
        sq_dist: f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        self(t, target_index, source_index, sq_dist)
    }
}
