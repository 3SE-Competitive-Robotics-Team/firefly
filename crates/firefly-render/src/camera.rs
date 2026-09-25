//! 第三人称追踪相机：机头方向后方偏上，平滑跟随。参数是全局唯一的一套硬编码，
//! 两个可视化应用共用，无按应用覆盖。

use bevy::prelude::*;

/// 相机到机体的后向距离（米）。
pub const BACK: f32 = 1.1;
/// 相机相对机体的抬升（米）。
pub const UP: f32 = 0.45;
/// 视点前移（米，看向机体前方）。
pub const LOOK_AHEAD: f32 = 0.8;
/// 跟随平滑（1/s，越大越跟手）。
pub const SMOOTH: f32 = 6.0;

/// 追踪目标：其 `Transform` 为机体位姿。
#[derive(Component)]
pub struct FollowTarget;

/// 追踪相机：由 [`follow_camera`] 摆位。
#[derive(Component)]
pub struct FollowCamera;

/// 第三人称追踪（`Update`）：只用**偏航**取向摆位（不随俯仰/横滚翻滚），指数平滑跟随。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn follow_camera(
    time: Res<Time>,
    target: Single<&Transform, (With<FollowTarget>, Without<FollowCamera>)>,
    mut cam: Single<&mut Transform, With<FollowCamera>>,
) {
    let body = target.into_inner();
    let fwd = body.rotation * Vec3::X;
    let heading = Quat::from_rotation_z(fwd.y.atan2(fwd.x));
    let desired = body.translation + heading * Vec3::new(-BACK, 0.0, UP);
    let t = (SMOOTH * time.delta_secs()).clamp(0.0, 1.0);
    cam.translation = cam.translation.lerp(desired, t);
    let look = body.translation + heading * Vec3::new(LOOK_AHEAD, 0.0, 0.0);
    cam.look_at(look, Vec3::Z);
}
