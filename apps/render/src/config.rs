//! `apps/render` 视觉配置（`configs/render.toml`）：只管光照，与场景无关联。
//!
//! 光照是目视调参项——场景切换不动它，改亮度不必重编译。方向固定（三盏，比值
//! 对照 `MuJoCo` diffuse 0.7:0.3:0.22），此处只调照度/环境光。

use bevy::prelude::*;
use serde::Deserialize;

/// 配置路径（编译期绝对路径，与运行 `CWD` 无关）。
const RENDER_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/render.toml");

/// 三方向光缺省照度（lux；直射日光量级，方向比值同 `MuJoCo`）。
pub const DEFAULT_DIRECTIONAL: [f32; 3] = firefly_render::lighting::ILLUMINANCE;
/// 全局环境光缺省（cd/m²，`Bevy` 缺省 80）。
pub const DEFAULT_AMBIENT: f32 = firefly_render::lighting::AMBIENT;
/// 相机曝光缺省（EV100；越大越暗。缺省贴合正午阳光，与三方向光照度同量级）。
pub const DEFAULT_EV100: f32 = firefly_render::lighting::EV100;
/// 传感器相机曝光缺省（EV100）：独立于主视角，专供 KLT 前端。
///
/// 传感器图像须落在灰度均值 120~170/255（`main.rs` 的 VIO 特征密度约束），
/// 主视角则可按场馆观感压暗；两者共用曝光会让传感器图在暗场景下无特征可跟。
pub const DEFAULT_SENSOR_EV100: f32 = 11.0;
/// 主视角 bloom 强度缺省（0 关闭）。
pub const DEFAULT_BLOOM: f32 = firefly_render::lighting::BLOOM;
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

/// 方向光阴影缺省（`CascadeShadowConfig`）：级联数与覆盖距离按场景尺度取。
///
/// 阴影按**视角**逐份渲染（`bevy_pbr` 的 `ViewLightEntities` 过滤），成本 ≈ 视角数
/// × 阴影灯数 × 级联数；Bevy 默认 4 级联 / `maximum_distance=150`（对齐
/// Unity/Unreal 的大世界），在 30m 级场地里远超需要。**必须**按实际可见距离收紧，
/// 否则阴影 pass 会吃掉出图帧预算（传感器链路要求相机稳定 10Hz）。
pub const DEFAULT_SHADOW_ENABLED: bool = true;
/// 投阴影的方向光盏数（其余只照明）。
pub const DEFAULT_SHADOW_CAST_LIGHTS: usize = 1;
/// 级联数（1 = 单级，最省）。
pub const DEFAULT_SHADOW_NUM_CASCADES: usize = 1;
/// 阴影最大距离（米）。
pub const DEFAULT_SHADOW_MAX_DISTANCE: f32 = 20.0;
/// 第一级联远界（米；`num_cascades = 1` 时忽略）。
pub const DEFAULT_SHADOW_FIRST_CASCADE_FAR_BOUND: f32 = 10.0;

/// 方向光阴影配置。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct ShadowConfig {
    /// 是否启用方向光阴影。
    #[serde(default = "default_shadow_enabled")]
    pub enabled: bool,
    /// 前 N 盏方向光投阴影（`0` = 全不投，等同 `enabled = false`）。
    #[serde(default = "default_shadow_cast_lights")]
    pub cast_lights: usize,
    /// 级联数（1 = 单级，最省）。
    #[serde(default = "default_shadow_num_cascades")]
    pub num_cascades: usize,
    /// 阴影最大距离（米）：超过此距离不接收阴影。
    #[serde(default = "default_shadow_max_distance")]
    pub maximum_distance: f32,
    /// 第一级联远界（米；`num_cascades = 1` 时忽略）。
    #[serde(default = "default_shadow_first_cascade_far_bound")]
    pub first_cascade_far_bound: f32,
}

fn default_shadow_enabled() -> bool {
    DEFAULT_SHADOW_ENABLED
}

fn default_shadow_cast_lights() -> usize {
    DEFAULT_SHADOW_CAST_LIGHTS
}

fn default_shadow_num_cascades() -> usize {
    DEFAULT_SHADOW_NUM_CASCADES
}

fn default_shadow_max_distance() -> f32 {
    DEFAULT_SHADOW_MAX_DISTANCE
}

fn default_shadow_first_cascade_far_bound() -> f32 {
    DEFAULT_SHADOW_FIRST_CASCADE_FAR_BOUND
}

impl Default for ShadowConfig {
    fn default() -> Self {
        Self {
            enabled: DEFAULT_SHADOW_ENABLED,
            cast_lights: DEFAULT_SHADOW_CAST_LIGHTS,
            num_cascades: DEFAULT_SHADOW_NUM_CASCADES,
            maximum_distance: DEFAULT_SHADOW_MAX_DISTANCE,
            first_cascade_far_bound: DEFAULT_SHADOW_FIRST_CASCADE_FAR_BOUND,
        }
    }
}

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

/// 光照配置。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct LightConfig {
    /// 全局环境光（cd/m²）。
    #[serde(default = "default_ambient")]
    pub ambient: f32,
    /// 三方向光照度（lux）。
    #[serde(default = "default_directional")]
    pub directional: [f32; 3],
    /// 顶棚点光源灯阵。
    #[serde(default)]
    pub grid: GridLightConfig,
    /// 方向光阴影。
    #[serde(default)]
    pub shadow: ShadowConfig,
}

fn default_ambient() -> f32 {
    DEFAULT_AMBIENT
}

fn default_directional() -> [f32; 3] {
    DEFAULT_DIRECTIONAL
}

impl Default for LightConfig {
    fn default() -> Self {
        Self {
            ambient: DEFAULT_AMBIENT,
            directional: DEFAULT_DIRECTIONAL,
            grid: GridLightConfig::default(),
            shadow: ShadowConfig::default(),
        }
    }
}

/// 相机/后处理配置。
///
/// 曝光必须与光照量级匹配：灯光按 lux 锚定日光时 `ev100≈15`，否则整幅惨白
///（缺省 `Exposure::BLENDER = 9.7` 是室内量级）。传感器相机用独立
/// `sensor_ev100`（VIO 特征密度约束见 [`DEFAULT_SENSOR_EV100`]），主视角用
/// `ev100`：两者目标不同（前者要特征，后者要观感），共用会互相迁就。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct ViewConfig {
    /// 主视角相机曝光 EV100（越大越暗）。
    #[serde(default = "default_ev100")]
    pub ev100: f32,
    /// 传感器相机曝光 EV100（越大越暗）。
    #[serde(default = "default_sensor_ev100")]
    pub sensor_ev100: f32,
    /// 主视角 bloom 强度（0 关闭；传感器相机不加）。
    #[serde(default = "default_bloom")]
    pub bloom_intensity: f32,
}

fn default_ev100() -> f32 {
    DEFAULT_EV100
}

fn default_sensor_ev100() -> f32 {
    DEFAULT_SENSOR_EV100
}

fn default_bloom() -> f32 {
    DEFAULT_BLOOM
}

impl Default for ViewConfig {
    fn default() -> Self {
        Self {
            ev100: DEFAULT_EV100,
            sensor_ev100: DEFAULT_SENSOR_EV100,
            bloom_intensity: DEFAULT_BLOOM,
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

/// `configs/render.toml` 顶层。
#[derive(Resource, Deserialize, Clone, Copy, Debug, Default)]
pub struct RenderConfig {
    /// 光照。
    #[serde(default)]
    pub light: LightConfig,
    /// 相机/后处理。
    #[serde(default)]
    pub view: ViewConfig,
    /// 环境光贴图。
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
