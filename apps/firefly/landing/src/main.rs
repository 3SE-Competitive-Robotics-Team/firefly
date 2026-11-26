mod config;
mod model;
mod offline;
mod runtime;
mod takeoff_replay;
mod tracking;

use config::{Config, validate};
use firefly_error::{Error, ErrorKind, Result};
use std::{path::PathBuf, sync::atomic::AtomicBool};
static RUNNING: AtomicBool = AtomicBool::new(true);

/// 下视图像识别，机载后端使用 `VIPLite`。
#[derive(usage_rs::Cli)]
#[usage(bin = "landing")]
struct Arguments {
    /// 单幅 352×288 图像的离线处理
    #[usage(long)]
    image: Option<PathBuf>,
    /// 回放一次停机坪起飞的下视帧和估计量，输出接管判据
    #[usage(long)]
    takeoff_replay: Option<PathBuf>,
    /// 显式选择 CPU，用于桌面仿真和离线比较
    #[usage(long)]
    simulation: bool,
}

fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("landing.input")
}

fn external(
    operation: &'static str,
    error: impl std::error::Error + Send + Sync + 'static,
) -> Error {
    Error::caused_by(ErrorKind::Internal, operation, error)
}

fn main() -> std::process::ExitCode {
    firefly_error::report("landing", run_app())
}

fn run_app() -> firefly_error::Result<()> {
    let args = Arguments::parse();
    let mut config: Config = firefly_config::startup(
        firefly_config::paths::config("landing.toml"),
        firefly_observability::init,
        validate,
    )?;
    if let Some(path) = args.takeoff_replay {
        if args.image.is_some() {
            return Err(invalid("select one offline input"));
        }
        return takeoff_replay::run(&config.tracking(), &path);
    }
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
