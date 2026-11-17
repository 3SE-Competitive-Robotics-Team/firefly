mod config;
mod output;

use std::path::PathBuf;

use firefly_error::{Error, ErrorKind, Result};

fn parse_args() -> Result<Option<PathBuf>> {
    let mut args = std::env::args().skip(1);
    let mut path = firefly_config::paths::config("traj.toml");
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => return Ok(None),
            "--config" => {
                path = PathBuf::from(args.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --config value")
                })?);
            }
            _ => {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    format!("unknown argument {arg}"),
                ));
            }
        }
    }
    Ok(Some(path))
}

fn main() {
    let path = match parse_args() {
        Ok(Some(path)) => path,
        Ok(None) => {
            println!("用法：traj [--config configs/traj.toml]");
            return;
        }
        Err(error) => {
            eprintln!("{error}\n用法：traj [--config configs/traj.toml]");
            std::process::exit(2);
        }
    };
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
