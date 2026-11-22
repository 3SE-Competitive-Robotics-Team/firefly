mod config;
mod output;

use std::path::PathBuf;

/// 执行轨迹输出
#[derive(usage_rs::Cli)]
#[usage(bin = "traj")]
struct Arguments {
    /// 应用配置文件
    #[usage(long)]
    config: Option<PathBuf>,
}

fn main() -> std::process::ExitCode {
    let result = run();
    firefly_observability::flush();
    result.unwrap_or_else(|e| firefly_error::report("traj", Err(e)))
}

fn run() -> firefly_error::Result<std::process::ExitCode> {
    let path = Arguments::parse()
        .config
        .unwrap_or_else(|| firefly_config::paths::config("traj.toml"));
    let options =
        firefly_config::startup(path, firefly_observability::init, config::Options::validate)?;
    Ok(firefly_error::report_with(
        output::run(options.heartbeat_timeout),
        |e| log::error!("轨迹输出失败：{e}"),
    ))
}
