//! 场地光照基建：三方向光 + 全局环境光。

use bevy::prelude::*;

/// 全局环境光（cd/m²；`Bevy` 缺省 80，这里按曝光量级压暗）。
pub const AMBIENT: f32 = 30.0;
/// 相机曝光 EV100（越大越暗；与三方向光照度量级匹配）。
pub const EV100: f32 = 14.5;
/// 主视角 bloom 强度（0 关闭）。
pub const BLOOM: f32 = 0.08;
/// 三方向光基线照度（lux，方向比值对照 `MuJoCo` diffuse 0.7:0.3:0.22）。
pub const ILLUMINANCE: [f32; 3] = [32000.0, 14000.0, 10000.0];
/// 三方向光方向（世界系，`Transform::NEG_Z` 旋向该方向）。
pub const DIRECTIONS: [Vec3; 3] = [
    Vec3::new(-0.3, -0.25, -0.92),
    Vec3::new(-0.15, 0.6, -0.78),
    Vec3::new(0.75, 0.1, -0.65),
];

/// 全局环境光资源（`GlobalAmbientLight`）。
#[must_use]
pub fn ambient_light() -> GlobalAmbientLight {
    GlobalAmbientLight {
        color: Color::WHITE,
        brightness: AMBIENT,
        ..default()
    }
}

/// 三方向光（照度 lux 由调用方给，方向固定）。
pub fn spawn_directional_lights(commands: &mut Commands, illuminance: [f32; 3]) {
    for (dir, lux) in DIRECTIONS.iter().zip(illuminance) {
        commands.spawn((
            DirectionalLight {
                illuminance: lux,
                ..default()
            },
            Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, dir.normalize())),
        ));
    }
}
