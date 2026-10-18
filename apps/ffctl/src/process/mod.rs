//! 进程管理：清单、supervisor、控制协议与 CLI 共用出口。

pub(crate) mod client;
pub(crate) mod config;
pub(crate) mod metrics;
pub(crate) mod protocol;
pub(crate) mod supervisor;

use std::path::Path;
use std::process::ExitCode;

use protocol::Request;

pub(crate) const SOCKET: &str = "/run/firefly/supervisor.sock";

pub(crate) fn report(result: Result<(), String>) -> ExitCode {
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("ffctl: {error}");
            ExitCode::FAILURE
        }
    }
}

pub(crate) fn request(request: &Request) -> ExitCode {
    match client::send(request) {
        Ok(response) if response.ok => {
            println!("{}", response.message);
            ExitCode::SUCCESS
        }
        Ok(response) => {
            eprintln!("ffctl: {}", response.message);
            ExitCode::FAILURE
        }
        Err(error) => report(Err(error)),
    }
}

pub(crate) fn set_autostart(path: &Path, names: &[String], autostart: bool) -> Result<(), String> {
    let mut config = config::Config::load(path)?;
    let changed = config.set_autostart(names, autostart)?;
    config.save(path)?;
    for name in changed {
        println!("{name}: autostart = {autostart}");
    }
    Ok(())
}
