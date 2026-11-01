use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam::online::{Options, run};
use serde::Deserialize;
use std::{
    io::Write,
    sync::{Arc, atomic::AtomicBool},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    online: Options,
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let Some(flag) = args.next() else {
        return Err(cli_error(
            "Usage: firefly-rootslam-live --config <path.toml>",
        ));
    };
    if flag == "--help" || flag == "-h" {
        writeln!(std::io::stdout(), "firefly-rootslam-live --config <path.toml>\nSubscribes to Render stereo and Sim IMU, publishes VIO to FC.").map_err(|e| Error::from(e).with_operation("rootslam.live.help"))?;
        return Ok(());
    }
    if flag != "--config" {
        return Err(cli_error(
            "Usage: firefly-rootslam-live --config <path.toml>",
        ));
    }
    let path = args
        .next()
        .ok_or_else(|| cli_error("Missing config path"))?;
    if args.next().is_some() {
        return Err(cli_error("Unexpected argument"));
    }
    let config: Config = firefly_config::startup(path, firefly_observability::init, |_| Ok(()))?;
    let stop = Arc::new(AtomicBool::new(false));
    let mut registrations = Vec::new();
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        registrations.push(
            signal_hook::flag::register(signal, Arc::clone(&stop))
                .map_err(|e| Error::from(e).with_operation("rootslam.live.signal"))?,
        );
    }
    let node = firefly_pubsub::node::create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "rootslam-live");
    std::thread::scope(|scope| {
        let worker = scope.spawn(|| run(&config.online, &stop));
        while !worker.is_finished() {
            firefly_observability::pump_log_ipc(&log_ipc);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let result = worker
            .join()
            .map_err(|_| cli_error("Online VIO worker panicked"))
            .and_then(|r| r);
        firefly_observability::pump_log_ipc(&log_ipc);
        if let Err(error) = &result {
            let _ = writeln!(
                std::io::stderr(),
                "RootSLAM live pipeline stopped with an error: {error:?}"
            );
        }
        result
    })?;
    for registration in registrations {
        signal_hook::low_level::unregister(registration);
    }
    Ok(())
}

fn cli_error(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam.live.cli")
}
