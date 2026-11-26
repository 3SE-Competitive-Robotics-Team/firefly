mod standalone;

use std::sync::{Arc, atomic::AtomicBool};

use firefly_error::{Error, Result};
use mimir::{config::Options, ingress};

/// 将运行数据录制为 RRD
#[derive(usage_rs::Cli)]
#[usage(bin = "mimir")]
struct Arguments {}

fn main() -> std::process::ExitCode {
    firefly_error::report("mimir", run_app())
}

fn run_app() -> firefly_error::Result<()> {
    Arguments::parse();
    let run = firefly_archive::Run::recorder()?;
    let record = || record(&run);
    if std::env::var_os("FIREFLY_RUN_DIR").is_some() {
        record()
    } else {
        standalone::capture(&run, record)
    }
}

fn record(run: &firefly_archive::Run) -> Result<()> {
    let options = firefly_config::startup(
        firefly_config::paths::config("mimir.toml"),
        firefly_observability::init,
        Options::validate,
    )?;
    let stop = Arc::new(AtomicBool::new(false));
    let mut registrations = Vec::new();
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        registrations.push(
            signal_hook::flag::register(signal, Arc::clone(&stop))
                .map_err(|e| Error::from(e).with_operation("install stop signal"))?,
        );
    }
    let result = ingress::run(&options, &stop, run.clone());
    for registration in registrations {
        signal_hook::low_level::unregister(registration);
    }
    result
}
