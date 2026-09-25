//! 场景光照（唯一一份）：三方向光 + 全局环境光，所有相机共用。
//!
//! 场景对所有相机只有一个：`apps/render` 的传感器 rig 与主视角、`apps/quad` 共用
//! 本模块的光照；视角之间的差异只允许来自各自的相机配置（曝光、色调映射、Msaa、
//! 目标分辨率）。
//!
//! 约束：**不得给单个相机加专属的可聚簇光源（`PointLight`/`SpotLight`）**——`Bevy`
//! 的 GPU 聚簇在多视图同帧渲染时不按 `RenderLayers` 隔离可聚簇光源，会渗进其他
//! 视角的着色（方向光不受影响）。要各视角亮度不同就改相机曝光，不要加灯。

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
