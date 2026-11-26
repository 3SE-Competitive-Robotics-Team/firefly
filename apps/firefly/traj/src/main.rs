mod config;
mod output;

/// 执行轨迹输出
#[derive(usage_rs::Cli)]
#[usage(bin = "traj")]
struct Arguments {}

fn main() -> std::process::ExitCode {
    let result = run();
    firefly_observability::flush();
    result.unwrap_or_else(|e| firefly_error::report("traj", Err(e)))
}

fn run() -> firefly_error::Result<std::process::ExitCode> {
    Arguments::parse();
    let options = firefly_config::startup(
        firefly_config::paths::config("traj.toml"),
        firefly_observability::init,
        config::Options::validate,
    )?;
    Ok(firefly_error::report_with(
        output::run(options.heartbeat_timeout),
        |e| log::error!("轨迹输出失败：{e}"),
    ))
}
