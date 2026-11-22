use bevy::prelude::*;
use serde::Deserialize;

#[derive(Resource, Clone, Copy, Debug)]
pub struct SceneSpec {
    /// 配置取值，`<赛季>@<版本>`。
    pub name: &'static str,
    /// `models/` 下的版本目录。
    pub dir: &'static str,
    /// 目录内视觉 glb 文件名。
    pub visual: &'static str,
    /// 机体起点（米，`MuJoCo` 系；真值到达前的初始摆位）。
    pub start: [f32; 3],
}

impl SceneSpec {
    /// 相对 [`firefly_render::scene::models_dir`] 的 asset 路径（对照 `AssetPlugin.file_path`）。
    #[must_use]
    pub fn asset_path(&self) -> String {
        format!("{}/{}", self.dir, self.visual)
    }
}

/// 默认场景；必须与 `configs/scene.toml` 的缺省一致。
pub const DEFAULT_SCENE: &str = "rmuc2027@1.0.0";

/// 已接入的场地版本。新增版本：目录 `models/<赛季>/v<版本>/` 加一行。
const SCENES: &[SceneSpec] = &[SceneSpec {
    name: "rmuc2027@1.0.0",
    dir: "rmuc2027/v1.0.0",
    visual: "field.glb",
    start: [-13.0, 0.0, 0.405],
}];

fn resolve(name: &str) -> Option<SceneSpec> {
    SCENES.iter().find(|spec| spec.name == name).copied()
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SceneConfig {
    #[serde(default = "default_scene")]
    scene: String,
}

fn default_scene() -> String {
    DEFAULT_SCENE.into()
}

/// 加载场景配置并检查视觉资产。
pub fn selected() -> firefly_error::Result<SceneSpec> {
    use firefly_error::{Error, ErrorKind, IoResultExt};
    let config = firefly_config::paths::config("scene.toml");
    let name = firefly_config::load::<SceneConfig>(&config)?.scene;
    let spec = resolve(&name).ok_or_else(|| {
        let supported: Vec<_> = SCENES.iter().map(|s| s.name).collect();
        Error::new(ErrorKind::InvalidArgument, format!("未注册的场景 {name}"))
            .with_context("supported", supported.join(", "))
            .with_context("path", config.display())
    })?;
    let path = firefly_render::scene::models_dir().join(spec.asset_path());
    if !path
        .metadata()
        .at_path("read scene asset", &path)?
        .is_file()
    {
        return Err(Error::new(ErrorKind::InvalidData, "场景视觉资产必须是文件")
            .with_context("path", path.display()));
    }
    Ok(spec)
}
