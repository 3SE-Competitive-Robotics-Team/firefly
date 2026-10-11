//! 原子导航快照：地图位姿、map←odom 与修正版本必须来自同一次后端提交。
use crate::odom::OdomMessage;
use firefly_base::{FrameId, RigidTransform};
use firefly_error::Result;
use iceoryx2::prelude::*;

pub const NAVIGATION_TOPIC: &str = "Firefly/Navigation";

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, ZeroCopySend)]
#[type_name("FireflyNavigationMessage")]
pub struct NavigationMessage {
    pub odom: OdomMessage,
    /// map←odom 平移（米）与 xyzw 四元数。
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    /// 单会话单调递增；变更后不得复用旧版深度融合或执行轨迹。
    pub revision: u64,
    /// 1 表示配置了外部地图启动先验；0 表示自主建立的会话地图。
    pub external_map: u64,
}

impl NavigationMessage {
    #[must_use]
    pub fn new(odom: OdomMessage, alignment: &RigidTransform, revision: u64) -> Self {
        let q = alignment.isometry().rotation.quaternion();
        Self {
            odom,
            translation: alignment.isometry().translation.vector.into(),
            rotation: [q.i, q.j, q.k, q.w],
            revision,
            external_map: 0,
        }
    }

    /// # Errors
    /// 非有限平移或无效单位四元数。
    pub fn alignment(&self) -> Result<RigidTransform> {
        RigidTransform::from_parts(FrameId::MAP, FrameId::ODOM, self.translation, self.rotation)
    }
}
