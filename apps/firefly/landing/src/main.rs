mod config;
mod model;
mod offline;
mod runtime;
mod tracking;

use config::{Config, validate};
use firefly_error::{Error, ErrorKind, Result};
use std::{path::PathBuf, sync::atomic::AtomicBool};
static RUNNING: AtomicBool = AtomicBool::new(true);

/// 下视图像识别，机载后端使用 `VIPLite`。
#[derive(usage_rs::Cli)]
#[usage(bin = "landing")]
struct Arguments {
    /// 应用配置文件
    #[usage(long)]
    config: Option<PathBuf>,
    /// 单幅 352×288 图像的离线处理
    #[usage(long)]
    image: Option<PathBuf>,
    /// 显式选择 CPU，用于桌面仿真和离线比较
    #[usage(long)]
    simulation: bool,
}

fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("landing.input")
}

fn external(operation: &'static str, error: impl std::fmt::Display) -> Error {
    Error::new(ErrorKind::Internal, error.to_string()).with_operation(operation)
}

fn main() -> Result<()> {
    let args = Arguments::parse();
    let path = args
        .config
        .unwrap_or_else(|| firefly_config::paths::config("landing.toml"));
    let mut config: Config = firefly_config::startup(path, firefly_observability::init, validate)?;
    let game = firefly_config::game::load()?;
    if args.simulation {
        config.backend = model::Backend::Cpu;
    }
    if config.backend != model::Backend::Viplite && !args.simulation && args.image.is_none() {
        return Err(invalid("live CPU inference requires explicit --simulation"));
    }
    if let Some(path) = args.image {
        return offline::run(&config, &path);
    }
    runtime::run(&config, game.team.code())
}
