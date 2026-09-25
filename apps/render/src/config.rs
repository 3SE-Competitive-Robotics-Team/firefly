//! `apps/render` **传感器相机**视觉配置（`configs/render.toml`）。
//!
//! 主视角照明/曝光/bloom 已收敛到 `firefly-render` 的共享效果
//!（[`spawn_viewer_lighting`](firefly_render::lighting::spawn_viewer_lighting) /
//! [`viewer_camera`](firefly_render::lighting::viewer_camera)），本配置只管传感器
//! rig 的照明（三方向光 + 顶棚点光灯阵 + IBL）与曝光——它按 VIO 灰度均值单独
//! 标定，与主视角观感解耦。

use bevy::prelude::*;
use serde::Deserialize;

/// 配置路径（编译期绝对路径，与运行 `CWD` 无关）。
const RENDER_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/render.toml");

/// 传感器相机三方向光缺省照度（lux）：主视角照明由 `firefly-render` 的
/// [`spawn_viewer_lighting`](firefly_render::lighting::spawn_viewer_lighting) 共享，
/// 此处仅传感器 rig 用，按 VIO 灰度均值单独标定（见 `configs/render.toml`）。
pub const DEFAULT_SENSOR_DIRECTIONAL: [f32; 3] = firefly_render::lighting::ILLUMINANCE;
/// 传感器相机曝光缺省（EV100）：专供 KLT 前端。
///
/// 传感器图像须落在灰度均值 120~170/255（`main.rs` 的 VIO 特征密度约束），
/// 主视角观感由 `firefly-render` 的共享曝光管。
pub const DEFAULT_SENSOR_EV100: f32 = 11.0;
/// 环境光贴图缺省颜色/强度（sRGB 0~1；强度缩放后单位 cd/m²）。
pub const DEFAULT_ENV_TOP: [f32; 3] = [0.62, 0.68, 0.78];
pub const DEFAULT_ENV_MID: [f32; 3] = [0.35, 0.38, 0.44];
pub const DEFAULT_ENV_BOTTOM: [f32; 3] = [0.09, 0.09, 0.10];
pub const DEFAULT_ENV_INTENSITY: f32 = 1500.0;
/// 顶棚灯阵缺省（`rows × cols` 盏点光源，覆盖 `span`×`span` 区域）。
pub const DEFAULT_GRID_ROWS: u32 = 4;
pub const DEFAULT_GRID_COLS: u32 = 8;
pub const DEFAULT_GRID_HEIGHT: f32 = 4.0;
pub const DEFAULT_GRID_INTENSITY: f32 = 300_000.0;
pub const DEFAULT_GRID_RANGE: f32 = 12.0;
pub const DEFAULT_GRID_SPAN: [f32; 2] = [28.0, 14.0];
pub const DEFAULT_GRID_COLOR: [f32; 3] = [1.0, 0.97, 0.92];

/// 顶棚点光源灯阵配置（场地顶部均匀布灯，营造场馆照明）。
///
/// 点光源按平方反比衰减，`intensity`（流明）与 `range`（米）须一起调：`range`
/// 太小会出现"光斑"硬边。全部不投阴影（一盏点光阴影 = 6 面 cubemap，数十盏
/// 直接压垮渲染）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct GridLightConfig {
    /// 纵向（世界 y）灯数。
    #[serde(default = "default_grid_rows")]
    pub rows: u32,
    /// 横向（世界 x）灯数。
    #[serde(default = "default_grid_cols")]
    pub cols: u32,
    /// 灯高（米）。
    #[serde(default = "default_grid_height")]
    pub height: f32,
    /// 单灯光通量（流明）。
    #[serde(default = "default_grid_intensity")]
    pub intensity: f32,
    /// 单灯作用半径（米）。
    #[serde(default = "default_grid_range")]
    pub range: f32,
    /// 灯阵覆盖范围（米，`[x 跨度, y 跨度]`，以场地中心为原点）。
    #[serde(default = "default_grid_span")]
    pub span: [f32; 2],
    /// 灯光颜色（sRGB 0~1）。
    #[serde(default = "default_grid_color")]
    pub color: [f32; 3],
}

fn default_grid_rows() -> u32 {
    DEFAULT_GRID_ROWS
}

fn default_grid_cols() -> u32 {
    DEFAULT_GRID_COLS
}

fn default_grid_height() -> f32 {
    DEFAULT_GRID_HEIGHT
}

fn default_grid_intensity() -> f32 {
    DEFAULT_GRID_INTENSITY
}

fn default_grid_range() -> f32 {
    DEFAULT_GRID_RANGE
}

fn default_grid_span() -> [f32; 2] {
    DEFAULT_GRID_SPAN
}

fn default_grid_color() -> [f32; 3] {
    DEFAULT_GRID_COLOR
}

impl Default for GridLightConfig {
    fn default() -> Self {
        Self {
            rows: DEFAULT_GRID_ROWS,
            cols: DEFAULT_GRID_COLS,
            height: DEFAULT_GRID_HEIGHT,
            intensity: DEFAULT_GRID_INTENSITY,
            range: DEFAULT_GRID_RANGE,
            span: DEFAULT_GRID_SPAN,
            color: DEFAULT_GRID_COLOR,
        }
    }
}

/// 传感器相机照明配置（主视角照明由 `firefly-render` 共享，不在此配置）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct LightConfig {
    /// 传感器相机三方向光照度（lux）。
    #[serde(default = "default_sensor_directional")]
    pub directional: [f32; 3],
    /// 顶棚点光源灯阵（只作用于传感器相机）。
    #[serde(default)]
    pub grid: GridLightConfig,
}

fn default_sensor_directional() -> [f32; 3] {
    DEFAULT_SENSOR_DIRECTIONAL
}

impl Default for LightConfig {
    fn default() -> Self {
        Self {
            directional: DEFAULT_SENSOR_DIRECTIONAL,
            grid: GridLightConfig::default(),
        }
    }
}

/// 传感器相机曝光配置。
///
/// 传感器图像须落在灰度均值 120~170/255（VIO 特征密度约束，见
/// [`DEFAULT_SENSOR_EV100`]），与主视角观感解耦：主视角曝光由 `firefly-render`
/// 的共享效果管，两边目标不同（前者要特征，后者要观感）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct ViewConfig {
    /// 传感器相机曝光 EV100（越大越暗）。
    #[serde(default = "default_sensor_ev100")]
    pub sensor_ev100: f32,
}

fn default_sensor_ev100() -> f32 {
    DEFAULT_SENSOR_EV100
}

impl Default for ViewConfig {
    fn default() -> Self {
        Self {
            sensor_ev100: DEFAULT_SENSOR_EV100,
        }
    }
}

/// 环境光贴图（IBL）配置。
///
/// 半球渐变"天空"（`EnvironmentMapLight::hemispherical_gradient`）：近黑高光面
/// 只靠方向光会「黑底 + 一点高光」，IBL 同时给暗部补光（漫反射）与柔和反射
/// （镜面），是黑亮面读得出形体的关键。颜色为 sRGB 0~1，`intensity` 缩放后
/// 单位 cd/m²。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct EnvConfig {
    /// 顶色（sRGB 0~1）。
    #[serde(default = "default_env_top")]
    pub top: [f32; 3],
    /// 地平线色（sRGB 0~1）。
    #[serde(default = "default_env_mid")]
    pub mid: [f32; 3],
    /// 底色（sRGB 0~1）。
    #[serde(default = "default_env_bottom")]
    pub bottom: [f32; 3],
    /// 强度缩放（cd/m²）。
    #[serde(default = "default_env_intensity")]
    pub intensity: f32,
}

fn default_env_top() -> [f32; 3] {
    DEFAULT_ENV_TOP
}

fn default_env_mid() -> [f32; 3] {
    DEFAULT_ENV_MID
}

fn default_env_bottom() -> [f32; 3] {
    DEFAULT_ENV_BOTTOM
}

fn default_env_intensity() -> f32 {
    DEFAULT_ENV_INTENSITY
}

impl Default for EnvConfig {
    fn default() -> Self {
        Self {
            top: DEFAULT_ENV_TOP,
            mid: DEFAULT_ENV_MID,
            bottom: DEFAULT_ENV_BOTTOM,
            intensity: DEFAULT_ENV_INTENSITY,
        }
    }
}

/// `configs/render.toml` 顶层（传感器相机专用）。
#[derive(Resource, Deserialize, Clone, Copy, Debug, Default)]
pub struct RenderConfig {
    /// 传感器相机照明。
    #[serde(default)]
    pub light: LightConfig,
    /// 传感器相机曝光。
    #[serde(default)]
    pub view: ViewConfig,
    /// 传感器相机环境光贴图。
    #[serde(default)]
    pub env: EnvConfig,
}

/// 读 `configs/render.toml`；缺文件/解析失败即报错退出。
#[must_use]
pub fn load() -> RenderConfig {
    let text = std::fs::read_to_string(RENDER_CONFIG).unwrap_or_else(|e| {
        log::error!("读取 configs/render.toml 失败：{e}");
        std::process::exit(1);
    });
    toml::from_str(&text).unwrap_or_else(|e| {
        log::error!("解析 configs/render.toml 失败：{e}");
        std::process::exit(1);
    })
}
