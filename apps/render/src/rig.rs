//! RMUC 传感器 rig：机体位姿与双目/深度相机几何。
//!
//! 基线沿机体 y 轴相距 0.05 米，深度居中，前向 +x、下倾 20°，
//! 垂直视场 70.88°，分辨率 320×240。标定须与 VIO 和视觉定位模块保持一致。
//! 仿真位姿四元数按 Hamilton 机体→世界系解读。

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{Camera, Exposure, Projection, RenderTarget};
use bevy::core_pipeline::prepass::DepthPrepass;
use bevy::image::Image;
use bevy::prelude::*;
use bevy::render::RenderApp;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevy::render::view::{Msaa, Tonemapping};
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};
use firefly_pubsub::trace::TraceContext;

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
/// rig 相机标记（主世界→渲染世界的透传标记，见 [`crate::capture`]）。
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug, ExtractComponent)]
#[extract_app(RenderApp)]
pub enum Eye {
    /// 左目 RGB（发布 `Firefly/CameraLeft` 的灰度源）。
    Left,
    /// 右目 RGB（发布 `Firefly/CameraRight` 的灰度源）。
    Right,
    /// 深度（预通道纹理即深度源，颜色输出仅占位）。
    Depth,
}

/// 位姿状态（`link` 每收到新真值写入，rig 相机读取）。
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

/// 装配 rig：三传感器相机（左/右/深度）。
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
            Tonemapping::Linear,
            sensor_exposure,
            sensor_layers.clone(),
            env.clone(),
            eye,
            eye_transform(pose.pos, pose.quat, offset),
        ));
    }

    // 深度相机：只跑深度预通道。`RenderTarget::None` 无颜色输出，Bevy 会移除
    // `ViewTarget`，彩色主 pass 不执行（省掉一整遍 5M 三角面）；深度经
    // `ViewDepthStencilTexture` 回读。
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
}

/// 从传感器几何唯一来源生成可视化标定；相机光学轴采用 RDF。
#[must_use]
pub fn camera_calibration(stamp: f64) -> [firefly_pubsub::viz::VizMessage; 3] {
    use firefly_pubsub::viz::{VizMessage, kind};
    let focal =
        (IMAGE_HEIGHT as f64 / 2.0) / (firefly_base::rig::FOV_Y_DEG.to_radians() / 2.0).tan();
    let [cx, cy] = firefly_base::rig::principal_point(IMAGE_WIDTH, IMAGE_HEIGHT);
    let rotation = cam_mount() * Quat::from_rotation_x(std::f32::consts::PI);
    [
        ("left", LEFT_OFFSET),
        ("right", RIGHT_OFFSET),
        ("depth", DEPTH_OFFSET),
    ]
    .map(|(name, offset)| {
        let mut msg = VizMessage::base(kind::CAMERA, stamp, &format!("cameras/{name}"));
        msg.xyz = offset.to_array().map(f64::from);
        msg.quat_xyzw = rotation.to_array().map(f64::from);
        msg.scalars = [focal, focal, cx, cy];
        msg.scalar_count = 4;
        msg.points[0] = [IMAGE_WIDTH as f64, IMAGE_HEIGHT as f64, 0.];
        msg.point_count = 1;
        msg
    })
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::float_cmp)] // IPC 标定的直接字段拷贝要求精确相等。
    fn rerun_calibration_uses_cv_axes_and_shared_intrinsics() {
        let [left, right, depth] = camera_calibration(1.5);
        let assert_camera = |message: &firefly_pubsub::viz::VizMessage, offset: Vec3| {
            assert_eq!(message.xyz, offset.to_array().map(f64::from));
            assert_eq!(
                message.points[0],
                [IMAGE_WIDTH as f64, IMAGE_HEIGHT as f64, 0.]
            );
            let rotation = Quat::from_array(message.quat_xyzw.map(|v| v as f32));
            let cv_point = Vec3::new(0.2, 0.3, 2.0);
            let bevy_point = Vec3::new(cv_point.x, -cv_point.y, -cv_point.z);
            assert!((rotation * cv_point - cam_mount() * bevy_point).length() < 1e-5);
            assert_eq!(message.scalars[2..], [159.5, 119.5]);
        };
        assert_camera(&left, LEFT_OFFSET);
        assert_camera(&right, RIGHT_OFFSET);
        assert_camera(&depth, DEPTH_OFFSET);
    }

    #[test]
    fn sensor_projection_matches_integer_pixel_centers() {
        let projection = sensor_projection().get_clip_from_view();
        let focal =
            (IMAGE_HEIGHT as f64 / 2.0) / (firefly_base::rig::FOV_Y_DEG.to_radians() / 2.0).tan();
        let [cx, cy] = firefly_base::rig::principal_point(IMAGE_WIDTH, IMAGE_HEIGHT);
        assert!((cx - 159.5).abs() < 1e-12 && (cy - 119.5).abs() < 1e-12);
        // WebGPU viewport 坐标位于像素边界；数组 (u,v) 的采样位置为 (u+.5,v+.5)。
        let pixel = |point: Vec3| {
            let clip = projection * point.extend(1.0);
            Vec2::new(
                (clip.x / clip.w + 1.0) * IMAGE_WIDTH as f32 / 2.0 - 0.5,
                (1.0 - clip.y / clip.w) * IMAGE_HEIGHT as f32 / 2.0 - 0.5,
            )
        };
        for z in [0.1, 1.0, 10.0] {
            for (u, v) in [(0., 0.), (319., 239.), (159.5, 119.5), (43.25, 187.75)] {
                let point = Vec3::new(
                    ((u - cx) * z / focal) as f32,
                    (-(v - cy) * z / focal) as f32,
                    -z as f32,
                );
                assert!((pixel(point) - Vec2::new(u as f32, v as f32)).length() < 1e-4);
            }
        }
        let point = Vec3::new(0.2, -0.3, -2.0);
        let epsilon = 1e-3;
        let dx =
            (pixel(point + Vec3::X * epsilon) - pixel(point - Vec3::X * epsilon)) / (2.0 * epsilon);
        assert!((dx.x - focal as f32 / 2.0).abs() < 0.02 && dx.y.abs() < 0.02);
    }
}
