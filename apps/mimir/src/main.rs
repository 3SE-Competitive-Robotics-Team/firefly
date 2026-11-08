use std::path::PathBuf;
use std::sync::{Arc, atomic::AtomicBool};

use firefly_error::{Error, ErrorKind, Result};
use mimir::{config::Options, ingress};

fn main() -> Result<()> {
    let mut config = firefly_config::paths::config("mimir.toml");
    let mut output = None;
    let mut args = std::env::args_os().skip(1);
    while let Some(arg) = args.next() {
        match arg.to_str() {
            Some("--help" | "-h") => {
                eprintln!("mimir --save <new.rrd> [--config <mimir.toml>]");
                return Ok(());
            }
            Some("--save") => {
                output = Some(PathBuf::from(
                    args.next().ok_or_else(|| invalid("missing --save path"))?,
                ));
            }
            Some("--config") => {
                config = PathBuf::from(
                    args.next()
                        .ok_or_else(|| invalid("missing --config path"))?,
                );
            }
            _ => return Err(invalid("unknown argument; use --help")),
        }
    }
    let output = output.ok_or_else(|| invalid("--save is required"))?;
    let options = firefly_config::startup(config, firefly_observability::init, Options::validate)?;
    let stop = Arc::new(AtomicBool::new(false));
    let mut registrations = Vec::new();
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        registrations.push(
            signal_hook::flag::register(signal, Arc::clone(&stop)).map_err(|e| {
                Error::new(ErrorKind::Internal, e.to_string()).with_operation("install stop signal")
            })?,
        );
    }
    let result = ingress::run(&output, &options, &stop);
    for registration in registrations {
        signal_hook::low_level::unregister(registration);
    }
    result
}

fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}
