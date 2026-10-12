//! 最小 TOML 应用：命令行 → 公共启动入口 → 已校验配置 → 运行。

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

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut path = "configs/minimal.toml".to_owned();
    while let Some(arg) = args.next() {
        if arg != "--config" {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "usage: minimal [--config path]",
            ));
        }
        path = args
            .next()
            .ok_or_else(|| Error::new(ErrorKind::InvalidArgument, "missing --config value"))?;
    }
    let options = firefly_config::startup(path, firefly_observability::init, Options::validate)?;
    run(&options);
    Ok(())
}

fn run(options: &Options) {
    // IPC、线程、模型等运行资源从这里开始创建。
    log::info!("应用就绪：{} Hz", options.rate_hz);
}
