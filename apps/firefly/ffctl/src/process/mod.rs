pub(crate) mod client;
pub(crate) mod config;
mod journal;
pub(crate) mod metrics;
pub(crate) mod protocol;
pub(crate) mod supervisor;

use std::path::Path;
use std::process::ExitCode;

use protocol::Request;

pub(crate) const SOCKET: &str = "/run/firefly/supervisor.sock";

pub(crate) use crate::ipc::report;

pub(crate) fn request(request: &Request) -> ExitCode {
    match client::send(request) {
        Ok(response) if response.ok => {
            println!("{}", response.message);
            ExitCode::SUCCESS
        }
        Ok(response) => report(Err(firefly_error::Error::new(
            firefly_error::ErrorKind::Unavailable,
            response.message,
        ))),
        Err(error) => report(Err(error)),
    }
}

pub(crate) fn set_autostart(
    path: &Path,
    names: &[String],
    autostart: bool,
) -> firefly_error::Result<()> {
    let mut config = config::Config::load(path)?;
    let changed = config.set_autostart(names, autostart)?;
    config.save(path)?;
    for name in changed {
        println!("{name}: autostart = {autostart}");
    }
    Ok(())
}
