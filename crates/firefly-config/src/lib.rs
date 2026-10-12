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
