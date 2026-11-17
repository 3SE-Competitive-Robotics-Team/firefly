//! 多视图共享场景光照，视角亮度由相机曝光与色调映射控制。
//! Bevy 的聚簇点光源与聚光灯不按 `RenderLayers` 隔离，相机专属光源会影响其他视图。

use bevy::camera::Exposure;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;

/// 全局环境光（cd/m²；`Bevy` 缺省 80，这里按曝光量级压暗）。
pub const AMBIENT: f32 = 30.0;
/// 场景三方向光照度缺省（lux）：方向比值对照 `MuJoCo` diffuse `0.7:0.3:0.22`。
pub const ILLUMINANCE: [f32; 3] = [32000.0, 14000.0, 10000.0];
/// 三方向光方向（世界系，`Transform::NEG_Z` 旋向该方向）。
pub const DIRECTIONS: [Vec3; 3] = [
    Vec3::new(-0.3, -0.25, -0.92),
    Vec3::new(-0.15, 0.6, -0.78),
    Vec3::new(0.75, 0.1, -0.65),
];

/// 可视化相机缺省曝光 EV100（越大越暗；与 [`ILLUMINANCE`] 同量级）。
pub const EV100: f32 = 14.5;
/// 可视化相机缺省 bloom 强度（0 关闭）。
pub const BLOOM: f32 = 0.08;

/// 场景光照装配：全局环境光 + 三方向光（不投影，默认层）。
pub fn spawn_scene_lighting(commands: &mut Commands, illuminance: [f32; 3]) {
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: AMBIENT,
        ..default()
    });
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

/// 可视化相机后处理（曝光 + bloom）：相机配置，与场景光照分开。
#[must_use]
pub fn viewer_camera() -> (Exposure, Bloom) {
    (
        Exposure { ev100: EV100 },
        Bloom {
            intensity: BLOOM,
            ..default()
        },
    )
}
