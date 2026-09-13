//! 调试面板：右侧固定栏展示左/右 RGB、左/右灰度、深度。
//!
//! 布局（窗口 1280×720，主 3D 视图占满窗口底层，面板浮于右侧）：
//! ```text
//! 3D 场景（追踪无人机）  | 左目 RGB  | 右目 RGB
//!                        | 左目灰度  | 右目灰度
//!                        | 深度（通栏）
//! ```
//!
//! 显示与发布同源（`link` 每节拍写入同一像素，显示侧只做 RGBA 展开），
//! 所见即 VIO 所得。

use bevy::asset::RenderAssetUsages;
use bevy::image::Image;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::text::FontSize;
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH};

/// 面板宽度（像素）。
const PANEL_WIDTH: f32 = 400.0;
/// 缩略图尺寸（像素，`320×240` 等比缩小）。
const THUMB_WIDTH: f32 = 188.0;
/// 缩略图高度（像素）。
const THUMB_HEIGHT: f32 = 141.0;
/// 深度图尺寸（像素，通栏）。
const DEPTH_WIDTH: f32 = 384.0;
/// 深度图高度（像素）。
const DEPTH_HEIGHT: f32 = 288.0;

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

/// 装配调试面板（五行：两行 RGB/灰度缩略对 + 通栏深度）。
pub fn setup_panel(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let views = DebugViews {
        left_rgb: images.add(display_image()),
        right_rgb: images.add(display_image()),
        left_gray: images.add(display_image()),
        right_gray: images.add(display_image()),
        depth: images.add(display_image()),
    };
    let thumbs = [
        (views.left_rgb.clone(), views.right_rgb.clone()),
        (views.left_gray.clone(), views.right_gray.clone()),
    ];
    let depth = views.depth.clone();
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
                right: Val::Px(0.0),
                top: Val::Px(0.0),
                width: Val::Px(PANEL_WIDTH),
                height: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(8.0)),
                row_gap: Val::Px(8.0),
                ..default()
            },
            BackgroundColor(Color::srgb(0.05, 0.05, 0.07)),
        ))
        .with_children(|panel| {
            for (left, right) in thumbs {
                panel
                    .spawn(Node {
                        flex_direction: FlexDirection::Row,
                        column_gap: Val::Px(8.0),
                        ..default()
                    })
                    .with_children(|row| {
                        for handle in [left, right] {
                            row.spawn((
                                ImageNode::new(handle),
                                Node {
                                    width: Val::Px(THUMB_WIDTH),
                                    height: Val::Px(THUMB_HEIGHT),
                                    ..default()
                                },
                            ));
                        }
                    });
            }
            panel.spawn((
                ImageNode::new(depth),
                Node {
                    width: Val::Px(DEPTH_WIDTH),
                    height: Val::Px(DEPTH_HEIGHT),
                    ..default()
                },
            ));
        });
}
