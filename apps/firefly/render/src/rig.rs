//! 传感器几何由 [`firefly_base::rig`] 派生；仿真姿态按 Hamilton 机体→世界系解读。

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{Camera, CompositingSpace, Exposure, Projection, RenderTarget};
use bevy::core_pipeline::prepass::DepthPrepass;
use bevy::image::Image;
use bevy::prelude::*;
use bevy::render::RenderApp;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevy::render::view::{Msaa, Tonemapping};
use firefly_base::{FrameId, rig};
use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH};
use firefly_pubsub::trace::TraceContext;

use crate::config::RenderConfig;

/// 垂直视场（度）；来源 [`firefly_base::rig::FOV_Y_DEG`]。
pub const FOV_Y_DEG: f32 = firefly_base::rig::FOV_Y_DEG as f32;
/// 相机下倾角（度）；来源 [`firefly_base::rig::DOWNTILT_DEG`]。
pub const DOWNTILT_DEG: f32 = firefly_base::rig::DOWNTILT_DEG as f32;

/// 传感器近平面（米，与深度有效下限一致）。
pub const SENSOR_NEAR: f32 = 0.05;
/// 传感器远平面（米，仅文档口径：`Bevy` 用无限远反向 Z，远裁剪由
/// [`crate::sensors`] 的有效掩码承担）。
pub const SENSOR_FAR: f32 = 100.0;

pub const WORLD_LAYER: usize = 0;
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug, ExtractComponent)]
#[extract_app(RenderApp)]
pub enum Eye {
    Left,
    Right,
    /// 深度（预通道纹理即深度源，颜色输出仅占位）。
    Depth,
    Down,
}

#[derive(Resource, Default)]
pub struct PoseState {
    /// 机体位置（世界系，米）。
    pub pos: Vec3,
    /// 机体姿态（机体→世界，Hamilton）。
    pub quat: Quat,
    /// 真值时间戳（秒，`sim_time`）。
    pub stamp: f64,
    pub trace: TraceContext,
    pub has_pose: bool,
}

#[derive(Resource)]
pub struct SensorTargets {
    pub left: Handle<Image>,
    pub right: Handle<Image>,
    pub down: Handle<Image>,
}

#[derive(Component)]
pub struct CameraMount(Transform);

impl CameraMount {
    fn new(frame: FrameId) -> Self {
        let optical = rig::body_from_optical(frame);
        let pose = optical.isometry();
        // Bevy 看向 −Z，图像上方是 +Y。绕 X 半周后得到共享光学轴。
        let optical_from_bevy = Quat::from_xyzw(1., 0., 0., 0.);
        let rotation = Quat::from_array(pose.rotation.coords.cast::<f32>().into());
        Self(Transform {
            translation: Vec3::from_array(pose.translation.vector.cast::<f32>().into()),
            rotation: rotation * optical_from_bevy,
            ..default()
        })
    }

    fn at_pose(&self, pose: &PoseState) -> Transform {
        Transform {
            translation: pose.pos + pose.quat * self.0.translation,
            rotation: pose.quat * self.0.rotation,
            ..default()
        }
    }

    fn bundle(frame: FrameId, pose: &PoseState) -> (Self, Transform) {
        let mount = Self::new(frame);
        let transform = mount.at_pose(pose);
        (mount, transform)
    }
}

/// 传感器投影（固定分辨率，纵横比不跟随窗口）。
#[must_use]
fn camera_projection(aspect_ratio: f32) -> Projection {
    Projection::Perspective(bevy::camera::PerspectiveProjection {
        fov: FOV_Y_DEG.to_radians(),
        aspect_ratio,
        near: SENSOR_NEAR,
        far: SENSOR_FAR,
        near_clip_plane: Vec4::new(0.0, 0.0, -1.0, -SENSOR_NEAR),
    })
}

/// 新建传感器灰度目标（`R8`；感知灰度由最终 blit 生成）。
#[must_use]
pub fn sensor_target() -> Image {
    sensor_target_with_size(IMAGE_WIDTH, IMAGE_HEIGHT)
}

fn sensor_target_with_size(width: usize, height: usize) -> Image {
    let mut image = Image::new(
        Extent3d {
            width: width as u32,
            height: height as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        vec![0u8; width * height],
        TextureFormat::R8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.texture_descriptor.usage =
        TextureUsages::RENDER_ATTACHMENT | TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_SRC;
    image
}

#[allow(clippy::needless_pass_by_value)]
pub fn spawn_rig(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    pose: Res<PoseState>,
    config: Res<RenderConfig>,
) {
    let left = images.add(sensor_target());
    let right = images.add(sensor_target());
    let down = images.add(sensor_target_with_size(DOWN_WIDTH, DOWN_HEIGHT));
    commands.insert_resource(SensorTargets {
        left: left.clone(),
        right: right.clone(),
        down: down.clone(),
    });

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

    // 双目灰度相机：颜色中间纹理用 sRGB 编码，最终输出单通道 BT.601。
    for (eye, frame, target) in [
        (Eye::Left, FrameId::LEFT_CAMERA, left),
        (Eye::Right, FrameId::RIGHT_CAMERA, right),
    ] {
        commands.spawn((
            Camera {
                is_active: false,
                ..default()
            },
            RenderTarget::from(target),
            Camera3d::default(),
            Msaa::Off,
            camera_projection(IMAGE_WIDTH as f32 / IMAGE_HEIGHT as f32),
            Tonemapping::Linear,
            CompositingSpace::Srgb,
            sensor_exposure,
            sensor_layers.clone(),
            env.clone(),
            eye,
            CameraMount::bundle(frame, &pose),
        ));
    }

    commands.spawn((
        Camera {
            is_active: false,
            ..default()
        },
        RenderTarget::from(down),
        Camera3d::default(),
        Msaa::Off,
        camera_projection(DOWN_WIDTH as f32 / DOWN_HEIGHT as f32),
        Tonemapping::Linear,
        CompositingSpace::Srgb,
        sensor_exposure,
        sensor_layers.clone(),
        env.clone(),
        Eye::Down,
        CameraMount::bundle(FrameId::DOWN_CAMERA, &pose),
    ));

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
        camera_projection(IMAGE_WIDTH as f32 / IMAGE_HEIGHT as f32),
        sensor_layers,
        Eye::Depth,
        CameraMount::bundle(FrameId::DEPTH_CAMERA, &pose),
        DepthPrepass,
    ));
}

/// Fixed simulated camera rig, with RDF optical axes and pixel-center intrinsics.
#[must_use]
pub fn camera_calibrations() -> firefly_pubsub::camera::CameraRig {
    use firefly_pubsub::camera::{CameraCalibration, CameraRig};

    let camera = |width, height, frame| {
        let focal = height as f64 * 0.5 / (rig::FOV_Y_DEG.to_radians() * 0.5).tan();
        let [cx, cy] = rig::principal_point(width, height);
        let mounting = rig::body_from_optical(frame);
        CameraCalibration {
            resolution: [width as u32, height as u32],
            intrinsics: [focal, focal, cx, cy],
            translation: mounting.isometry().translation.vector.into(),
            rotation_xyzw: mounting.isometry().rotation.coords.into(),
        }
    };
    CameraRig {
        left: camera(IMAGE_WIDTH, IMAGE_HEIGHT, FrameId::LEFT_CAMERA),
        right: camera(IMAGE_WIDTH, IMAGE_HEIGHT, FrameId::RIGHT_CAMERA),
        down: camera(DOWN_WIDTH, DOWN_HEIGHT, FrameId::DOWN_CAMERA),
    }
}

/// rig 相机摆位（纯函数：`poll_pose` 在同系统内调用，保证渲染用新变换出图）。
pub fn apply_pose_to_eyes(pose: &PoseState, eyes: &mut Query<(&CameraMount, &mut Transform)>) {
    if !pose.has_pose {
        return;
    }
    for (mount, mut transform) in eyes {
        *transform = mount.at_pose(pose);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn optical_and_bevy_projections_agree_for_each_camera() {
        let calibration = camera_calibrations();
        let body = PoseState {
            pos: Vec3::new(1., -2., 0.5),
            quat: Quat::from_rotation_z(0.7),
            ..default()
        };
        for (frame, camera, resolution) in [
            (
                FrameId::LEFT_CAMERA,
                calibration.left,
                [IMAGE_WIDTH, IMAGE_HEIGHT],
            ),
            (
                FrameId::RIGHT_CAMERA,
                calibration.right,
                [IMAGE_WIDTH, IMAGE_HEIGHT],
            ),
            (
                FrameId::DOWN_CAMERA,
                calibration.down,
                [DOWN_WIDTH, DOWN_HEIGHT],
            ),
        ] {
            assert_eq!(camera.resolution, resolution.map(|v| v as u32));
            assert_eq!(
                camera.translation,
                <[f64; 3]>::from(rig::position_in_body(frame))
            );
            assert_eq!(
                camera.intrinsics[2..],
                rig::principal_point(resolution[0], resolution[1])
            );
            let [width, height] = camera.resolution.map(|v| v as f32);
            let [fx, fy, cx, cy] = camera.intrinsics.map(|v| v as f32);
            let optical_rotation = Quat::from_array(camera.rotation_xyzw.map(|v| v as f32));
            let origin = Vec3::from_array(camera.translation.map(|v| v as f32));
            let world_from_view = CameraMount::new(frame).at_pose(&body);
            let clip_from_view = camera_projection(width / height).get_clip_from_view();
            for point in [Vec3::new(0.2, 0.3, 2.), Vec3::new(-0.4, -0.2, 1.)] {
                let world = body.pos + body.quat * (origin + optical_rotation * point);
                let view =
                    world_from_view.rotation.inverse() * (world - world_from_view.translation);
                let ndc = clip_from_view.project_point3(view);
                let pixel =
                    Vec2::new((ndc.x + 1.) * width, (1. - ndc.y) * height) * 0.5 - Vec2::splat(0.5);
                let expected = Vec2::new(fx * point.x / point.z + cx, fy * point.y / point.z + cy);
                assert!(
                    pixel.distance(expected) < 1e-4,
                    "{frame:?}: {pixel} vs {expected}"
                );
            }
        }
    }

    #[test]
    fn sensor_projection_matches_integer_pixel_centers() {
        let projection =
            camera_projection(IMAGE_WIDTH as f32 / IMAGE_HEIGHT as f32).get_clip_from_view();
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
