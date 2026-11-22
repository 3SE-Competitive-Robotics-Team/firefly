use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
struct Options {
    rate_hz: f64,
}

impl Default for Options {
    fn default() -> Self {
        Self { rate_hz: 10.0 }
    }
}

impl Options {
    fn validate(&self) -> Result<()> {
        if !self.rate_hz.is_finite() || self.rate_hz <= 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "rate_hz must be finite and positive",
            ));
        }
        Ok(())
    }
}

use std::path::PathBuf;

/// 加载配置并启动示例应用
#[derive(usage_rs::Cli)]
#[usage(bin = "minimal")]
struct Arguments {
    /// 应用配置文件
    #[usage(long)]
    config: Option<PathBuf>,
}

fn main() -> Result<()> {
    let path = Arguments::parse()
        .config
        .unwrap_or_else(|| firefly_config::paths::config("minimal.toml"));
    let options = firefly_config::startup(path, firefly_observability::init, Options::validate)?;
    run(&options);
    Ok(())
}

fn run(options: &Options) {
    // IPC、线程、模型等运行资源从这里开始创建。
    log::info!("应用就绪：{} Hz", options.rate_hz);
}
