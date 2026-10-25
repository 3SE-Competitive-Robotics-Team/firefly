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

/// 运行期路径约定：板上固定 `/opt/firefly`，开发机构建用仓库根。
///
/// 部署布局是 `bin/`、`configs/`、`models/` 三个同级目录；二进制按约定定位，
/// 不依赖启动时的工作目录。板上与开发机由目标三元组区分，不做运行时探测。
pub mod paths {
    use std::path::PathBuf;

    /// 板载布局（Orange Pi Zero 3W，aarch64-linux）。
    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    const ROOT: &str = "/opt/firefly";
    /// 开发机：本 crate 位于 `<repo>/crates/firefly-config`，向上两级即仓库根。
    #[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
    const ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");

    /// firefly 安装根目录。
    #[must_use]
    pub fn root() -> PathBuf {
        normalize(PathBuf::from(ROOT))
    }

    /// 去掉 `.`/`..` 的词法归一化：开发机构建的根路径含 `../..`，日志和报错里应给出
    /// 干净路径；不访问文件系统，符号链接语义不影响。
    fn normalize(path: PathBuf) -> PathBuf {
        let mut out = PathBuf::new();
        for component in path.components() {
            match component {
                std::path::Component::CurDir => {}
                std::path::Component::ParentDir => {
                    out.pop();
                }
                other => out.push(other),
            }
        }
        out
    }

    /// `configs/<name>` 的绝对路径。
    #[must_use]
    pub fn config(name: &str) -> PathBuf {
        root().join("configs").join(name)
    }

    /// `models/` 下的绝对路径。
    #[must_use]
    pub fn models(name: &str) -> PathBuf {
        root().join("models").join(name)
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn config_and_models_follow_the_deployment_layout() {
            assert!(config("fc.toml").ends_with("configs/fc.toml"));
            assert!(models("aliked.onnx").ends_with("models/aliked.onnx"));
            assert!(config("fc.toml").is_absolute());
            assert!(!config("fc.toml").to_string_lossy().contains(".."));
        }

        #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
        #[test]
        fn board_root_is_the_installed_prefix() {
            assert_eq!(root(), PathBuf::from("/opt/firefly"));
        }
    }
}
