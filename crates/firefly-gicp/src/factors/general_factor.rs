//! 通用因子（对照 `factors/general_factor.hpp`）。

use nalgebra::{Matrix4, Matrix6, Vector6};

use crate::points::traits::PointCloudTrait;

/// 无约束因子（对照 `NullFactor`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct NullFactor;

impl NullFactor {
    /// 更新线性化系统（空操作）。
    pub fn update_linearized_system<Target, Source, Tree>(
        &self,
        _target: &Target,
        _source: &Source,
        _target_tree: &Tree,
        _t: &Matrix4<f64>,
        _h: &mut Matrix6<f64>,
        _b: &mut Vector6<f64>,
        _e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
    }

    /// 更新误差（空操作）。
    pub fn update_error<Target, Source>(
        &self,
        _target: &Target,
        _source: &Source,
        _t: &Matrix4<f64>,
        _e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
    }
}

/// 自由度约束因子（对照 `RestrictDoFFactor`）。
///
/// 通过大系数 `lambda` 软约束特定自由度（`mask` 中 1 表示激活约束的轴，
/// 0 表示自由轴；实现为 `H += lambda * diag(|mask-1|)`）。
#[derive(Clone, Debug)]
pub struct RestrictDofFactor {
    /// 正则化强度。
    pub lambda: f64,
    ///掩码 `[rx, ry, rz, tx, ty, tz]`（1=约束，0=自由）。
    pub mask: [f64; 6],
}

impl Default for RestrictDofFactor {
    fn default() -> Self {
        Self {
            lambda: 1e9,
            mask: [1.0; 6],
        }
    }
}

impl RestrictDofFactor {
    /// 新建（默认全约束）。
    pub fn new() -> Self {
        Self::default()
    }

    /// 设置旋转掩码。
    pub fn set_rotation_mask(&mut self, m: [f64; 3]) {
        self.mask[0] = m[0];
        self.mask[1] = m[1];
        self.mask[2] = m[2];
    }

    /// 设置平移掩码。
    pub fn set_translation_mask(&mut self, m: [f64; 3]) {
        self.mask[3] = m[0];
        self.mask[4] = m[1];
        self.mask[5] = m[2];
    }

    /// 更新线性化系统（对照 `update_linearized_system`）。
    pub fn update_linearized_system<Target, Source, Tree>(
        &self,
        _target: &Target,
        _source: &Source,
        _target_tree: &Tree,
        _t: &Matrix4<f64>,
        h: &mut Matrix6<f64>,
        _b: &mut Vector6<f64>,
        _e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        for i in 0..6 {
            h[(i, i)] += self.lambda * (self.mask[i] - 1.0).abs();
        }
    }

    /// 更新误差（空操作，对照 C++）。
    pub fn update_error<Target, Source>(
        &self,
        _target: &Target,
        _source: &Source,
        _t: &Matrix4<f64>,
        _e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
    }
}

/// 通用因子 trait（供 `Registration` 泛型约束）。
pub trait GeneralFactor {
    /// 更新线性化系统。
    fn update_linearized_system<Target, Source, Tree>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait;

    /// 更新误差。
    fn update_error<Target, Source>(
        &self,
        target: &Target,
        source: &Source,
        t: &Matrix4<f64>,
        e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait;
}

impl GeneralFactor for NullFactor {
    fn update_linearized_system<Target, Source, Tree>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        NullFactor::update_linearized_system(self, target, source, target_tree, t, h, b, e);
    }

    fn update_error<Target, Source>(
        &self,
        target: &Target,
        source: &Source,
        t: &Matrix4<f64>,
        e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        NullFactor::update_error(self, target, source, t, e);
    }
}

impl GeneralFactor for RestrictDofFactor {
    fn update_linearized_system<Target, Source, Tree>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        RestrictDofFactor::update_linearized_system(self, target, source, target_tree, t, h, b, e);
    }

    fn update_error<Target, Source>(
        &self,
        target: &Target,
        source: &Source,
        t: &Matrix4<f64>,
        e: &mut f64,
    ) where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        RestrictDofFactor::update_error(self, target, source, t, e);
    }
}
