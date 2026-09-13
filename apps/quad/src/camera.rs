//! 第三人称追踪相机：机头方向后方偏上，平滑跟随。

use bevy::prelude::*;

use crate::config::QuadConfig;
use crate::quad::Quad;

/// 追踪相机摆位（`Update`，在 `quad::dynamics` 之后读机体位姿）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn chase(
    time: Res<Time>,
    cfg: Res<QuadConfig>,
    drone: Single<(&Quad, &Transform)>,
    mut cam: Single<&mut Transform, (With<Camera3d>, Without<Quad>)>,
) {
    let (quad, body) = drone.into_inner();
    // 只用偏航取向，避免随俯仰/横滚翻滚。
    let heading = Quat::from_rotation_z(quad.yaw);
    let desired = body.translation + heading * Vec3::new(-cfg.camera.back, 0.0, cfg.camera.up);
    let t = (cfg.camera.smooth * time.delta_secs()).clamp(0.0, 1.0);
    cam.translation = cam.translation.lerp(desired, t);
    let look = body.translation + heading * Vec3::new(cfg.camera.look_ahead, 0.0, 0.0);
    cam.look_at(look, Vec3::Z);
}
