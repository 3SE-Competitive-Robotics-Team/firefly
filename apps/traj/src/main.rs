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

fn main() {
    let path = Arguments::parse()
        .config
        .unwrap_or_else(|| firefly_config::paths::config("traj.toml"));
    let options =
        match firefly_config::startup(path, firefly_observability::init, config::Options::validate)
        {
            Ok(options) => options,
            Err(error) => {
                eprintln!("加载配置失败：{error}");
                std::process::exit(1);
            }
        };
    let result = output::run(options.heartbeat_timeout);
    if let Err(error) = &result {
        log::error!("轨迹输出失败：{error}");
    }
    firefly_observability::flush();
    if result.is_err() {
        std::process::exit(1);
    }
}
