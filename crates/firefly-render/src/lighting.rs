//! 主视角视觉效果：环境光 + 三方向光 + 曝光 + bloom。
//!
//! `apps/quad` 与 `apps/render` 的**主视角**共用这一套参数——这里是唯一来源，
//! 两边不再各自定制；改观感只改本模块。方向固定（三盏，比值对照 `MuJoCo`
//! diffuse 0.7:0.3:0.22），只调照度/环境光/曝光/bloom。
//!
//! 传感器相机（`apps/render`）的照明是另一回事，不在本模块：它按 VIO 灰度均值
//! 单独标定，见 `configs/render.toml`。

use bevy::camera::Exposure;
use bevy::camera::visibility::RenderLayers;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;

/// 全局环境光（cd/m²；`Bevy` 缺省 80，这里按曝光量级压暗）。
pub const AMBIENT: f32 = 30.0;
/// 主视角曝光 EV100（越大越暗；与三方向光照度同量级）。
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

/// 主视角灯光装配：全局环境光资源 + 三方向光（不投影）。
///
/// `layers` 决定灯光作用域：单相机应用传 [`RenderLayers::default`]；多相机
/// 应用把主视角灯光收进独立层，避免传感器相机吃到同一套光。
pub fn spawn_viewer_lighting(commands: &mut Commands, layers: &RenderLayers) {
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: AMBIENT,
        ..default()
    });
    for (dir, lux) in DIRECTIONS.iter().zip(ILLUMINANCE) {
        commands.spawn((
            DirectionalLight {
                illuminance: lux,
                ..default()
            },
            layers.clone(),
            Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, dir.normalize())),
        ));
    }
}

/// 主视角相机后处理：曝光 + bloom（与 [`spawn_viewer_lighting`] 同源）。
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
