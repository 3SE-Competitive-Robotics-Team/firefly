//! 可选 TOML 配置：缺文件使用 serde 默认值，其他读取和解析错误向上传播。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use serde::de::DeserializeOwned;

/// 加载配置；缺文件与空 TOML 使用同一套字段默认值，并记录一行 INFO。
///
/// # Errors
/// 文件存在但不可读，或内容无法反序列化（包括缺失没有默认值的必填字段）。
pub fn load<T: DeserializeOwned>(path: impl AsRef<Path>) -> Result<T> {
    let path = path.as_ref();
    let raw = match std::fs::read_to_string(path) {
        Ok(raw) => raw,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            log::info!("配置 {} 不存在，使用默认值", path.display());
            String::new()
        }
        Err(error) => {
            return Err(Error::new(ErrorKind::Internal, "cannot read config")
                .with_operation("load config")
                .with_context("path", path.display())
                .with_source(error));
        }
    };
    toml::from_str(&raw).map_err(|error| {
        Error::new(ErrorKind::InvalidArgument, "invalid config")
            .with_operation("parse config")
            .with_context("path", path.display())
            .with_source(error)
    })
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
