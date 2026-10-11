//! 调试面板：底栏展示左/右 RGB、左/右 VIO 跟踪图、深度。
//!
//! 布局（窗口 1280×720，视口由 [`layout_viewports`] 每帧按窗口尺寸与 DPI 重算）：
//! ```text
//! ┌───────────────┬───────────────┐
//! │ 真实 3D（跟随）│ 体素 3D（仅数据层）│
//! ├───────────────┴───────────────┤
//! │ 左 RGB 右 RGB 左灰度 右灰度 深度 │
//! └───────────────────────────────┘
//! ```
//!
//! 显示与发布同源；灰度跟踪图与 VIO 结果按相机时间配对，标记仅写入显示副本。

use bevy::asset::RenderAssetUsages;
use bevy::camera::{Camera, Viewport};
use bevy::image::Image;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::text::FontSize;
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH};
use firefly_render::camera::FollowCamera;

/// 底部面板高度（像素）。
const PANEL_HEIGHT: f32 = 184.0;
/// 缩略图尺寸（像素，`320×240` 等比缩小）。
const THUMB_WIDTH: f32 = 188.0;
/// 缩略图高度（像素）。
const THUMB_HEIGHT: f32 = 141.0;

/// 左上角模式提示（`freecam` 按模式更新）。
#[derive(Component)]
pub struct ModeLabel;

/// 跟随模式提示（ASCII：Bevy 缺省字体无 CJK 字形，中文会触发 ICU4X 报错且不显示）。
pub const FOLLOW_HINT: &str = "Follow drone   |   F: free look";
/// 自由浏览模式提示。
pub const FREE_HINT: &str =
    "Free look   |   WASD: move   mouse: look   Q/E: up-down   Shift: boost   F/Esc: exit";

/// 调试显示图（CPU 写入的 `RGBA8` 图，`ImageNode` 直接引用同句柄）。
#[derive(Resource)]
pub struct DebugViews {
    /// 左目 RGB。
    pub left_rgb: Handle<Image>,
    /// 右目 RGB。
    pub right_rgb: Handle<Image>,
    /// 左目灰度。
    pub left_gray: Handle<Image>,
    /// 右目灰度。
    pub right_gray: Handle<Image>,
    /// 深度伪彩（近白远黑）。
    pub depth: Handle<Image>,
}

/// 新建调试显示图（黑底占位，`link` 每节拍覆盖）。
fn display_image() -> Image {
    Image::new(
        Extent3d {
            width: IMAGE_WIDTH as u32,
            height: IMAGE_HEIGHT as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        vec![0u8; 4 * IMAGE_SIZE],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
}

/// 摆两个 3D 视口：左侧真实场景、右侧体素视图，各占半宽，底部空出面板高度。
///
/// `Camera::viewport` 用物理像素，面板用逻辑像素定位，因此按窗口 `scale_factor`
/// 换算；窗口缩放或 DPI 变化时重算，尺寸未变则不写回（避免每帧触碰 `Camera`）。
#[allow(clippy::needless_pass_by_value)] // `SystemParam` 契约，与同 crate 其它系统一致
pub fn layout_viewports(
    windows: Query<&Window>,
    cameras: Query<(&mut Camera, Has<FollowCamera>, Has<crate::rig::VoxelView>)>,
) {
    let Ok(window) = windows.single() else {
        return;
    };
    let size = window.physical_size();
    let panel = (PANEL_HEIGHT * window.scale_factor()).round() as u32;
    let height = size.y.saturating_sub(panel).max(1);
    let half = (size.x / 2).max(1);
    for (mut camera, is_follow, is_voxel) in cameras {
        let (position, extent) = if is_follow {
            (UVec2::ZERO, UVec2::new(half, height))
        } else if is_voxel {
            (UVec2::new(half, 0), UVec2::new(size.x - half, height))
        } else {
            continue;
        };
        if camera
            .viewport
            .as_ref()
            .is_none_or(|v| v.physical_position != position || v.physical_size != extent)
        {
            camera.viewport = Some(Viewport {
                physical_position: position,
                physical_size: extent,
                ..default()
            });
        }
    }
}

/// 装配五列调试面板；左右 VIO 标题显示点数与配对图像时间。
pub fn setup_panel(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let views = DebugViews {
        left_rgb: images.add(display_image()),
        right_rgb: images.add(display_image()),
        left_gray: images.add(display_image()),
        right_gray: images.add(display_image()),
        depth: images.add(display_image()),
    };
    let tiles = [
        (views.left_rgb.clone(), "Left RGB", None),
        (views.right_rgb.clone(), "Right RGB", None),
        (views.left_gray.clone(), "L gray | waiting for VIO", Some(0)),
        (
            views.right_gray.clone(),
            "R gray | waiting for VIO",
            Some(1),
        ),
        (views.depth.clone(), "Depth", None),
    ];
    commands.insert_resource(views);

    commands.spawn((
        Text::new(FOLLOW_HINT),
        TextFont {
            font_size: FontSize::Px(16.0),
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.92, 0.92)),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(12.0),
            top: Val::Px(10.0),
            ..default()
        },
        ModeLabel,
    ));

    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                bottom: Val::Px(0.0),
                width: Val::Percent(100.0),
                height: Val::Px(PANEL_HEIGHT),
                flex_direction: FlexDirection::Row,
                align_items: AlignItems::Center,
                padding: UiRect::all(Val::Px(8.0)),
                column_gap: Val::Px(8.0),
                ..default()
            },
            BackgroundColor(Color::srgb(0.05, 0.05, 0.07)),
        ))
        .with_children(|panel| {
            for (handle, label, eye) in tiles {
                panel
                    .spawn(Node {
                        flex_direction: FlexDirection::Column,
                        row_gap: Val::Px(4.0),
                        flex_shrink: 0.0,
                        ..default()
                    })
                    .with_children(|tile| {
                        let mut caption = tile.spawn((
                            Text::new(label),
                            TextFont {
                                font_size: FontSize::Px(11.0),
                                ..default()
                            },
                            TextColor(Color::srgb(0.85, 0.9, 0.9)),
                        ));
                        if let Some(eye) = eye {
                            caption.insert(crate::keypoints::KeypointLabel(eye));
                        }
                        tile.spawn((
                            ImageNode::new(handle),
                            Node {
                                width: Val::Px(THUMB_WIDTH),
                                height: Val::Px(THUMB_HEIGHT),
                                flex_shrink: 0.0,
                                ..default()
                            },
                        ));
                    });
            }
        });
}
