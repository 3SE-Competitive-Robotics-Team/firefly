//! 构建目标与各目标的 release 应用名单。

use std::path::{Path, PathBuf};

/// 桌面应用：仿真、可视化与地面站 CLI。
const DESKTOP_APPS: &[&str] = &[
    "aliked",
    "fc",
    "ffctl",
    "firefly-rootslam",
    "hil",
    "lightglue",
    "localization",
    "landing",
    "mimir",
    "planner",
    "quad",
    "render",
    "traj",
];

/// 机载应用：Orange Pi Zero 3W 上运行的进程，不含 Bevy 仿真/演示程序。
const BOARD_APPS: &[&str] = &[
    "aliked",
    "camera",
    "fc",
    "ffctl",
    "hil-gadget",
    "imu",
    "firefly-rootslam",
    "lightglue",
    "localization",
    "landing",
    "mimir",
    "motor",
    "planner",
    "traj",
];

/// `--target` 的可选值。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// 本机桌面构建；在 Apple Silicon 上产物是 arm64-apple-darwin。
    Desktop,
    /// 机载 aarch64 Linux；在 Apple container 的 Debian trixie 里原生构建。
    Board,
}

impl Target {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "host" => Ok(Self::Desktop),
            "board" => Ok(Self::Board),
            other => Err(format!("unknown target {other}: expected host or board")),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Desktop => "host",
            Self::Board => "board",
        }
    }

    pub fn apps(self) -> &'static [&'static str] {
        match self {
            Self::Desktop => DESKTOP_APPS,
            Self::Board => BOARD_APPS,
        }
    }

    pub fn release_dir(self, root: &Path) -> PathBuf {
        match self {
            Self::Desktop => root.join("target/release"),
            Self::Board => root.join("target/container/aarch64/release"),
        }
    }
}

pub fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask is a workspace member")
        .to_path_buf()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn desktop_release_includes_vio_planning_and_hil() {
        let apps = Target::Desktop.apps();
        assert!(apps.contains(&"firefly-rootslam"));
        assert!(apps.contains(&"planner"));
        assert!(apps.contains(&"traj"));
        assert!(apps.contains(&"hil"));
    }

    #[test]
    fn board_release_includes_rootslam_and_the_gadget() {
        let apps = Target::Board.apps();
        assert!(apps.contains(&"firefly-rootslam"));
        assert!(apps.contains(&"hil-gadget"));
        assert!(apps.contains(&"planner"));
        assert!(apps.contains(&"traj"));
    }
}
