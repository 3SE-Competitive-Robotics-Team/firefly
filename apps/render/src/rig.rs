//! RMUC 传感器 rig：机体位姿与双目/深度相机几何。
//!
//! 基线沿机体 y 轴相距 0.05 米，深度居中，前向 +x、下倾 20°，
//! 垂直视场 70.88°，分辨率 320×240。标定须与 VIO 和视觉定位模块保持一致。
//! 仿真位姿四元数按 Hamilton 机体→世界系解读。

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{Camera, ClearColorConfig, Exposure, Projection, RenderTarget};
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

/// 垂直视场（度）；来源 [`firefly_base::rig::FOV_Y_DEG`]。
pub const FOV_Y_DEG: f32 = firefly_base::rig::FOV_Y_DEG as f32;
/// 相机下倾角（度）；来源 [`firefly_base::rig::DOWNTILT_DEG`]。
pub const DOWNTILT_DEG: f32 = firefly_base::rig::DOWNTILT_DEG as f32;

/// 机体系位置（f64 米）→ Bevy `Vec3`。
const fn body_vec(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// 左目在机体系偏移（米）；来源 [`firefly_base::rig::LEFT_IN_BODY`]。
pub const LEFT_OFFSET: Vec3 = body_vec(firefly_base::rig::LEFT_IN_BODY);
/// 右目在机体系偏移（米）；来源 [`firefly_base::rig::RIGHT_IN_BODY`]。
pub const RIGHT_OFFSET: Vec3 = body_vec(firefly_base::rig::RIGHT_IN_BODY);
/// 深度相机在机体系偏移（米）；来源 [`firefly_base::rig::DEPTH_IN_BODY`]。
pub const DEPTH_OFFSET: Vec3 = body_vec(firefly_base::rig::DEPTH_IN_BODY);
/// 传感器近平面（米，与深度有效下限一致）。
pub const SENSOR_NEAR: f32 = 0.05;
/// 传感器远平面（米，仅文档口径：`Bevy` 用无限远反向 Z，远裁剪由
/// [`crate::sensors`] 的有效掩码承担）。
pub const SENSOR_FAR: f32 = 100.0;

/// 场地 mesh 层：所有相机都渲染。
pub const WORLD_LAYER: usize = 0;
/// 仅主视角渲染的机体层：传感器相机位于机体内，必须排除机体模型，否则自遮挡。
pub const DRONE_LAYER: usize = 1;
/// 仅主视角渲染的调试层（体素可视化）：传感器图像是 VIO/深度的输入，
/// 任何调试几何都必须排除在这一层之外，否则会被当成真实场景写进发布数据。
pub const VOXEL_LAYER: usize = 2;

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
    Transform {
        translation: pos + quat * offset,
        rotation: quat * cam_mount(),
        ..default()
    }
}

/// 相机 mounting 旋转（相机轴 → 机体系，列 = 相机轴）；来源 [`firefly_base::rig`]。
fn cam_mount() -> Quat {
    let a = firefly_base::rig::cam_axes_in_body();
    let col = |c: usize| Vec3::new(a[(0, c)] as f32, a[(1, c)] as f32, a[(2, c)] as f32);
    Quat::from_mat3(&Mat3::from_cols(col(0), col(1), col(2)))
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
        RenderLayers::from_layers(&[WORLD_LAYER, DRONE_LAYER, VOXEL_LAYER]),
        env,
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

    // UI 单独用一个铺满窗口的相机：主相机被 `layout_viewports` 限到左侧视口后，
    // UI 的坐标空间会跟着缩到左半，面板的 `right: 0` 就落到窗口中间了。
    commands.spawn((
        Camera2d,
        Camera {
            order: 1,
            // 满窗相机默认会清屏；它在主 3D 相机之后渲染，清屏会把 3D 视图整个
            // 擦成背景色（实测左侧全黑）。UI 相机只画 UI，不清屏。
            clear_color: ClearColorConfig::None,
            ..default()
        },
        IsDefaultUiCamera,
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
