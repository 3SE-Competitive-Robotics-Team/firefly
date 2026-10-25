//! 可选 TOML 配置：缺文件使用 serde 默认值，其他读取和解析错误向上传播。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use serde::de::DeserializeOwned;

/// 应用启动入口：初始化基础日志、读取并校验配置、报告来源。
///
/// 在命令行解析之后调用；返回成功后才创建 IPC、线程、模型或窗口。
/// `init_logging` 只安装日志设施，`validate` 只校验参数，不创建运行资源。
/// 没有额外参数约束时可传入 `|_| Ok(())`，反序列化仍会检查字段类型。
///
/// # Errors
/// 配置读取、反序列化或应用参数校验失败。
pub fn startup<T: DeserializeOwned>(
    path: impl AsRef<Path>,
    init_logging: impl FnOnce(),
    validate: impl FnOnce(&T) -> Result<()>,
) -> Result<T> {
    init_logging();
    let path = path.as_ref();
    let (config, source) = read(path)?;
    validate(&config).map_err(|error| {
        error
            .with_operation("validate config")
            .with_context("path", path.display())
    })?;
    source.report(path);
    Ok(config)
}

/// 在日志已初始化的进程中加载辅助配置；应用主配置使用 [`startup`]。
/// 缺文件与空 TOML 使用同一套字段默认值，成功后记录一行 INFO 来源。
///
/// # Errors
/// 文件存在但不可读，或内容无法反序列化（包括缺失没有默认值的必填字段）。
pub fn load<T: DeserializeOwned>(path: impl AsRef<Path>) -> Result<T> {
    let path = path.as_ref();
    let (config, source) = read(path)?;
    source.report(path);
    Ok(config)
}

enum Source {
    File,
    Defaults,
}

impl Source {
    fn report(&self, path: &Path) {
        match self {
            Self::File => log::info!("已加载配置 {}", path.display()),
            Self::Defaults => log::info!("配置 {} 不存在，使用默认值", path.display()),
        }
    }
}

fn read<T: DeserializeOwned>(path: &Path) -> Result<(T, Source)> {
    let (raw, source) = match std::fs::read_to_string(path) {
        Ok(raw) => (raw, Source::File),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            (String::new(), Source::Defaults)
        }
        Err(error) => {
            return Err(Error::new(ErrorKind::Internal, "cannot read config")
                .with_operation("load config")
                .with_context("path", path.display())
                .with_source(error));
        }
    };
    let config = toml::from_str(&raw).map_err(|error| {
        Error::new(ErrorKind::InvalidArgument, "invalid config")
            .with_operation("parse config")
            .with_context("path", path.display())
            .with_source(error)
    })?;
    Ok((config, source))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, PartialEq, serde::Deserialize)]
    #[serde(default, deny_unknown_fields)]
    struct Options {
        rate: u32,
    }

    impl Default for Options {
        fn default() -> Self {
            Self { rate: 1000 }
        }
    }

    #[test]
    fn missing_empty_partial_and_invalid_files() {
        let dir = std::env::temp_dir().join(format!("firefly-config-{}", std::process::id()));
        std::fs::create_dir(&dir).unwrap();
        let path = dir.join("options.toml");
        assert_eq!(load::<Options>(&path).unwrap(), Options::default());
        std::fs::write(&path, "# defaults\n").unwrap();
        assert_eq!(load::<Options>(&path).unwrap(), Options::default());
        std::fs::write(&path, "rate = 50").unwrap();
        assert_eq!(load::<Options>(&path).unwrap(), Options { rate: 50 });
        for invalid in ["rate = [", "unknown = 1", "rate = 'bad'"] {
            std::fs::write(&path, invalid).unwrap();
            let error = load::<Options>(&path).unwrap_err();
            assert_eq!(error.kind(), ErrorKind::InvalidArgument);
            assert_eq!(error.operation(), Some("parse config"));
        }
        assert!(load::<Options>(&dir).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }
}

/// 运行期路径约定：安装根 + 其下的 `configs/`、`models/`。
///
/// 部署布局是 `bin/`、`configs/`、`models/` 三个同级目录。资源定位遵循
/// "越显式越优先"的惯例，不依赖启动目录：
///
/// 1. `FIREFLY_ROOT` 环境变量；
/// 2. 可执行文件所在前缀 `<exe>/../..`（部署布局下即安装根）；
/// 3. 编译期默认：板载 `/opt/firefly`，开发机构建为仓库根。
///
/// 不用 `env!("CARGO_MANIFEST_DIR")` 作为**运行时**路径：它是构建机路径，
/// 二进制被安装到别处（`cargo install`、打包、部署到板子）后就失效。这里的
/// 编译期常量只作最后回退，开发机构建保留了仓库内直接运行的能力。
///
/// 也不使用 `directories`/XDG 的用户配置目录：`configs/`、`models/` 是随程序
/// 一起安装的只读资源，不是用户数据，按安装前缀定位才符合 FHS 与部署语义。
pub mod paths {
    use std::{
        ffi::OsStr,
        path::{Component, Path, PathBuf},
        sync::OnceLock,
    };

    /// 覆盖安装根的环境变量。
    pub const ROOT_ENV: &str = "FIREFLY_ROOT";

    /// 板载布局（Orange Pi Zero 3W，aarch64-linux）。
    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    const DEFAULT_ROOT: &str = "/opt/firefly";
    /// 开发机：本 crate 位于 `<repo>/crates/firefly-config`，向上两级即仓库根。
    #[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
    const DEFAULT_ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");

    static ROOT: OnceLock<PathBuf> = OnceLock::new();

    /// firefly 安装根目录；解析一次后缓存。
    #[must_use]
    pub fn root() -> PathBuf {
        ROOT.get_or_init(|| {
            resolve(
                std::env::var_os(ROOT_ENV).as_deref(),
                std::env::current_exe().ok().as_deref(),
                Path::new(DEFAULT_ROOT),
            )
        })
        .clone()
    }

    /// `configs/<name>` 的路径。
    #[must_use]
    pub fn config(name: impl AsRef<Path>) -> PathBuf {
        root().join("configs").join(name)
    }

    /// `models/<name>` 的路径。
    #[must_use]
    pub fn models(name: impl AsRef<Path>) -> PathBuf {
        root().join("models").join(name)
    }

    /// 按优先级解析安装根；纯函数，便于覆盖各分支。
    fn resolve(override_root: Option<&OsStr>, exe: Option<&Path>, default: &Path) -> PathBuf {
        if let Some(dir) = override_root.filter(|dir| !dir.is_empty()) {
            return normalize(Path::new(dir));
        }
        if let Some(prefix) = exe
            .and_then(install_prefix)
            .filter(|prefix| prefix.join("configs").is_dir())
        {
            return normalize(prefix);
        }
        normalize(default)
    }

    /// 可执行文件的上两级：部署布局下 `<prefix>/bin/<app>` 的前缀。
    fn install_prefix(exe: &Path) -> Option<&Path> {
        exe.parent()?.parent()
    }

    /// 去掉 `.`/`..` 的词法归一化：日志与报错里应给出干净路径。
    /// 不访问文件系统，因此不解析符号链接——容器与软链布局下依然可用。
    fn normalize(path: &Path) -> PathBuf {
        let mut out = PathBuf::new();
        for component in path.components() {
            match component {
                Component::CurDir => {}
                Component::ParentDir => {
                    out.pop();
                }
                other => out.push(other),
            }
        }
        out
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use std::fs;

        #[test]
        fn env_override_wins_over_everything() {
            let default = Path::new("/default/root");
            let exe = Path::new("/installed/bin/fc");
            assert_eq!(
                resolve(Some(OsStr::new("/custom/root")), Some(exe), default),
                PathBuf::from("/custom/root")
            );
        }

        #[test]
        fn empty_env_value_is_ignored() {
            let default = Path::new("/default/root");
            assert_eq!(
                resolve(Some(OsStr::new("")), None, default),
                PathBuf::from("/default/root")
            );
        }

        #[test]
        fn executable_prefix_is_used_when_it_has_configs() {
            // 产物写在 target/（磁盘），不写 /tmp。
            let scratch = Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../target/firefly-config-tests")
                .join("prefix");
            let configs = scratch.join("configs");
            fs::create_dir_all(&configs).unwrap();
            let exe = scratch.join("bin").join("fc");
            assert_eq!(
                resolve(None, Some(&exe), Path::new("/default/root")),
                normalize(&scratch)
            );
            assert_eq!(
                install_prefix(&exe),
                Some(scratch.as_path()),
                "exe 的上两级就是安装前缀"
            );
            fs::remove_dir_all(&scratch).unwrap();
        }

        #[test]
        fn executable_without_configs_falls_back_to_default() {
            let exe = Path::new("/usr/local/bin/fc");
            assert_eq!(
                resolve(None, Some(exe), Path::new("/opt/firefly")),
                PathBuf::from("/opt/firefly")
            );
        }

        #[test]
        fn config_and_models_follow_the_deployment_layout() {
            let config = config("fc.toml");
            assert!(config.ends_with("configs/fc.toml"));
            assert!(config.is_absolute());
            assert!(!config.to_string_lossy().contains(".."));
            assert!(models("aliked.onnx").ends_with("models/aliked.onnx"));
        }

        #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
        #[test]
        fn board_default_root_is_the_installed_prefix() {
            assert_eq!(PathBuf::from(DEFAULT_ROOT), PathBuf::from("/opt/firefly"));
        }
    }
}
