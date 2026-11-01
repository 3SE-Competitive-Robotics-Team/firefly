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
    let config: Config = firefly_config::startup(
        firefly_config::paths::config("firefly-rootslam.toml"),
        firefly_observability::init,
        |_| Ok(()),
    )?;
    let stop = Arc::new(AtomicBool::new(false));
    let mut registrations = Vec::new();
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        registrations.push(
            signal_hook::flag::register(signal, Arc::clone(&stop))
                .map_err(|e| Error::from(e).with_operation("rootslam.live.signal"))?,
        );
    }
    let node = firefly_pubsub::node::create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "rootslam");
    std::thread::scope(|scope| {
        let worker = scope.spawn(|| run(&config.online, &stop));
        while !worker.is_finished() {
            firefly_observability::pump_log_ipc(&log_ipc);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let result = worker
            .join()
            .map_err(|_| {
                Error::new(ErrorKind::Internal, "online VIO worker panicked")
                    .with_operation("rootslam.live.worker")
            })
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
