//! 传感器 rig：位姿表示 + `MuJoCo` 相机几何的 `Bevy` 复刻。
//!
//! 相机几何对照 MJCF（`scene.py` 双目/深度三相机）：基线沿机体 y 相距
//! 0.05 米（左 `-0.025` / 右 `+0.025`，深度居中），前向 +x、下倾 20°，
//! 垂直视场 70.88°，分辨率 320×240。
//!
//! 真值四元数按 Hamilton 机体→世界系解读：与 `MuJoCo` 自身相机一致
//! （由构造保证，rrd 里 `Bevy` 图与 `MuJoCo` 图逐像素对照是仲裁依据）。

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{Camera, Exposure, Projection, RenderTarget};
use bevy::core_pipeline::prepass::DepthPrepass;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::image::Image;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevy::render::view::Msaa;
use bevy::ui::IsDefaultUiCamera;
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};
use firefly_pubsub::trace::TraceContext;
use firefly_render::camera::FollowCamera;

use crate::config::RenderConfig;

/// 垂直视场（度，对照 MJCF `fovy="70.88"`）。
pub const FOV_Y_DEG: f32 = 70.88;
/// 相机下倾角（度，对照 MJCF `xyaxes` 的 `0.3420/0.9397 = sin/cos 20°`）。
pub const DOWNTILT_DEG: f32 = 20.0;
/// 左目在机体系偏移（米，对照 MJCF `cam_left pos="0 -0.025 0"`）。
pub const LEFT_OFFSET: Vec3 = Vec3::new(0.0, -0.025, 0.0);
/// 右目在机体系偏移（米，对照 MJCF `cam_right pos="0 0.025 0"`）。
pub const RIGHT_OFFSET: Vec3 = Vec3::new(0.0, 0.025, 0.0);
/// 深度相机在机体系偏移（米，对照 MJCF `cam_depth pos="0 0 0"`）。
pub const DEPTH_OFFSET: Vec3 = Vec3::ZERO;
/// 传感器近平面（米，对照 `MuJoCo` 默认近裁剪，深度有效下限同源）。
pub const SENSOR_NEAR: f32 = 0.05;
/// 传感器远平面（米，仅文档口径：`Bevy` 用无限远反向 Z，远裁剪由
/// [`crate::sensors`] 的有效掩码承担，对照 `env.py` 的 `depth < 100`）。
pub const SENSOR_FAR: f32 = 100.0;

/// 场地 mesh 层：所有相机都渲染。
pub const WORLD_LAYER: usize = 0;
/// 仅主视角渲染的机体层：传感器相机位于机体内，必须排除机体模型，否则自遮挡。
pub const DRONE_LAYER: usize = 1;

/// rig 相机标记（主世界→渲染世界的透传标记，见 [`crate::capture`]）。
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug)]
pub enum Eye {
    /// 左目 RGB（发布 `Firefly/CameraLeft` 的灰度源）。
    Left,
    /// 右目 RGB（发布 `Firefly/CameraRight` 的灰度源）。
    Right,
    /// 深度（预通道纹理即深度源，颜色输出仅占位）。
    Depth,
}

/// 位姿状态（`link` 每收到新真值写入，rig 相机与主视角跟随读取）。
#[derive(Resource, Default)]
pub struct PoseState {
    /// 机体位置（世界系，米）。
    pub pos: Vec3,
    /// 机体姿态（机体→世界，Hamilton）。
    pub quat: Quat,
    /// 真值时间戳（秒，`sim_time`）。
    pub stamp: f64,
    /// 最近一条真值的 trace 上下文（出图发布时续接）。
    pub trace: TraceContext,
    /// 是否收到过真值。
    pub has_pose: bool,
}

/// 传感器回读目标（主世界句柄，`capture` 按 `AssetId` 在渲染世界取 GPU 纹理）。
#[derive(Resource)]
pub struct SensorTargets {
    /// 左目颜色目标。
    pub left: Handle<Image>,
    /// 右目颜色目标。
    pub right: Handle<Image>,
}

/// 机体位姿 + 相机 mounting 偏移 → 相机世界变换。
#[must_use]
pub fn eye_transform(pos: Vec3, quat: Quat, offset: Vec3) -> Transform {
    let tilt = DOWNTILT_DEG.to_radians();
    let x_cam = Vec3::new(0.0, -1.0, 0.0);
    let y_cam = Vec3::new(tilt.sin(), 0.0, tilt.cos());
    let z_cam = x_cam.cross(y_cam);
    let mount = Quat::from_mat3(&Mat3::from_cols(x_cam, y_cam, z_cam));
    Transform {
        translation: pos + quat * offset,
        rotation: quat * mount,
        ..default()
    }
}

/// 传感器投影（固定分辨率，纵横比不跟随窗口）。
#[must_use]
pub fn sensor_projection() -> Projection {
    Projection::Perspective(bevy::camera::PerspectiveProjection {
        fov: FOV_Y_DEG.to_radians(),
        aspect_ratio: IMAGE_WIDTH as f32 / IMAGE_HEIGHT as f32,
        near: SENSOR_NEAR,
        far: SENSOR_FAR,
        near_clip_plane: Vec4::new(0.0, 0.0, -1.0, -SENSOR_NEAR),
    })
}

/// 新建传感器颜色回读目标（`RGBA8 sRGB`，渲染挂载 + CPU 回读两用）。
#[must_use]
pub fn sensor_target() -> Image {
    let mut image = Image::new(
        Extent3d {
            width: IMAGE_WIDTH as u32,
            height: IMAGE_HEIGHT as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        vec![0u8; 4 * IMAGE_WIDTH * IMAGE_HEIGHT],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.texture_descriptor.usage =
        TextureUsages::RENDER_ATTACHMENT | TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_SRC;
    image
}

/// 装配 rig：三传感器相机（左/右/深度）+ 主视角跟随相机。
// 系统参数按值传递（`SystemParam` 契约，见 `main.rs` 同类标注）。
#[allow(clippy::needless_pass_by_value)]
pub fn spawn_rig(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    pose: Res<PoseState>,
    config: Res<RenderConfig>,
) {
    let left = images.add(sensor_target());
    let right = images.add(sensor_target());
    commands.insert_resource(SensorTargets {
        left: left.clone(),
        right: right.clone(),
    });

    // 环境光贴图（IBL）：半球渐变天空，给近黑高光面暗部补光 + 柔和反射（场景级，
    // 所有相机共用）。
    let env = EnvironmentMapLight {
        intensity: config.env.intensity,
        ..EnvironmentMapLight::hemispherical_gradient(
            &mut images,
            Color::srgb(config.env.top[0], config.env.top[1], config.env.top[2]),
            Color::srgb(config.env.mid[0], config.env.mid[1], config.env.mid[2]),
            Color::srgb(
                config.env.bottom[0],
                config.env.bottom[1],
                config.env.bottom[2],
            ),
        )
    };

    let sensor_exposure = Exposure {
        ev100: config.view.sensor_ev100,
    };
    let sensor_layers = RenderLayers::layer(WORLD_LAYER);

    // 双目彩色相机：各出 RGB（回读左/右目颜色目标）。
    // 出图窗口由 `link::poll_pose` 激活：非出图帧不渲染传感器（省 GPU）。
    for (eye, offset, target) in [
        (Eye::Left, LEFT_OFFSET, left),
        (Eye::Right, RIGHT_OFFSET, right),
    ] {
        commands.spawn((
            Camera {
                is_active: false,
                ..default()
            },
            RenderTarget::from(target),
            Camera3d::default(),
            Msaa::Off,
            sensor_projection(),
            Tonemapping::None,
            sensor_exposure,
            sensor_layers.clone(),
            env.clone(),
            eye,
            eye_transform(pose.pos, pose.quat, offset),
        ));
    }

    // 深度相机：只跑深度预通道。`RenderTarget::None` 无颜色输出，Bevy 会移除
    // `ViewTarget`，彩色主 pass 不执行（省掉一整遍 5M 三角面）；深度经
    // `ViewDepthTexture` 回读。
    commands.spawn((
        Camera {
            is_active: false,
            ..default()
        },
        RenderTarget::None {
            size: UVec2::new(IMAGE_WIDTH as u32, IMAGE_HEIGHT as u32),
        },
        Camera3d::default(),
        Msaa::Off,
        sensor_projection(),
        sensor_layers,
        Eye::Depth,
        eye_transform(pose.pos, pose.quat, DEPTH_OFFSET),
        DepthPrepass,
    ));

    // 主视角：相机配置缺省与传感器一致（曝光 `viewer_ev100` 缺省 = `sensor_ev100`），
    // 只多机体层——它是这条链路的 debug 窗口，与发布图像同源观感。
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Tonemapping::None,
        Exposure {
            ev100: config.view.viewer_ev100(),
        },
        RenderLayers::from_layers(&[WORLD_LAYER, DRONE_LAYER]),
        env,
        IsDefaultUiCamera,
        // 第三人称追踪（与 `apps/quad` 同一实现 `firefly-render`）；自由浏览模式下
        // 让位给 `freecam::freecam_move`（见 `main.rs` 的运行条件）。
        FollowCamera,
        Transform::from_translation(
            pose.pos
                + Vec3::new(
                    -firefly_render::camera::BACK,
                    0.0,
                    firefly_render::camera::UP,
                ),
        )
        .looking_at(pose.pos, Vec3::Z),
    ));
}

/// rig 相机摆位（纯函数：`poll_pose` 在同系统内调用，保证渲染用新变换出图）。
pub fn apply_pose_to_eyes(pose: &PoseState, eyes: &mut Query<(&Eye, &mut Transform)>) {
    if !pose.has_pose {
        return;
    }
    for (eye, mut transform) in eyes {
        let offset = match eye {
            Eye::Left => LEFT_OFFSET,
            Eye::Right => RIGHT_OFFSET,
            Eye::Depth => DEPTH_OFFSET,
        };
        *transform = eye_transform(pose.pos, pose.quat, offset);
    }
}
