//! 场景注册表：`FIREFLY_SCENE` → `models/<dir>/` 视觉资产 + 出生点。
//!
//! 统一接入约定（见 `packages/firefly-cad/README.md`）：`Bevy` asset root 恒为
//! [`MODELS_DIR`]，视觉恒为 `models/<dir>/<visual>`；换场地只加一行注册 + 一个
//! 目录，不改加载代码。未注册名称回退 [`DEFAULT_SCENE`] 并告警。
//!
//! `MuJoCo` 侧（`firefly_mujoco/scene.py`）持有同构注册表，同一 `FIREFLY_SCENE`
//! 同时驱动视觉与碰撞。

use bevy::prelude::*;

/// 场地资产根（编译期绝对路径，与运行 `CWD` 无关）。
pub const MODELS_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../models");

/// 一个场地的接入描述。
#[derive(Resource, Clone, Copy, Debug)]
pub struct SceneSpec {
    /// `models/` 下的目录名（即注册键，与 `FIREFLY_SCENE` 取值一致）。
    pub dir: &'static str,
    /// 目录内视觉 glb 文件名。
    pub visual: &'static str,
    /// 机体起点（米，`MuJoCo` 系；真值到达前的初始摆位，对照 `configs/sim.toml`）。
    pub start: [f32; 3],
}

impl SceneSpec {
    /// 相对 [`MODELS_DIR`] 的 asset 路径（对照 `AssetPlugin.file_path`）。
    #[must_use]
    pub fn asset_path(&self) -> String {
        format!("{}/{}", self.dir, self.visual)
    }
}

/// 缺省场景（注册表必须含此项）。
pub const DEFAULT_SCENE: &str = "warehouse";

/// 已注册场地（视觉 glb + 起点；碰撞描述在 `MuJoCo` 侧注册表）。
const SCENES: &[SceneSpec] = &[
    SceneSpec {
        dir: "warehouse",
        visual: "structure.glb",
        start: [2.0, 0.0, 1.0],
    },
    SceneSpec {
        dir: "rmuc2026",
        visual: "field.glb",
        start: [2.0, 0.0, 1.0],
    },
];

/// 按 `FIREFLY_SCENE` 选场景；未注册回退 [`DEFAULT_SCENE`]。
#[must_use]
pub fn selected() -> SceneSpec {
    let name = std::env::var("FIREFLY_SCENE").unwrap_or_else(|_| DEFAULT_SCENE.to_owned());
    SCENES
        .iter()
        .copied()
        .find(|s| s.dir == name)
        .unwrap_or_else(|| {
            log::warn!("未知场景 {name}，回退 {DEFAULT_SCENE}");
            *SCENES
                .iter()
                .find(|s| s.dir == DEFAULT_SCENE)
                .expect("缺省场景必须在注册表")
        })
}
