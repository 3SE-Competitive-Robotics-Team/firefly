use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam::online::{Options, run};
use serde::Deserialize;
use std::{
    io::Write,
    path::PathBuf,
    sync::{Arc, atomic::AtomicBool},
};

const USAGE: &str = "firefly-rootslam [--config <path>] [--no-loop-closures] [--no-mapping]";

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    online: Options,
}

/// 运行时开关；与 `firefly-okvis-x` 的同名选项语义一致。
struct Arguments {
    config: PathBuf,
    loop_closures: bool,
    mapping: bool,
}

fn argument_error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam::arguments")
}

fn parse() -> Result<Option<Arguments>> {
    let mut args = std::env::args_os();
    let _executable = args.next().unwrap_or_else(|| "firefly-rootslam".into());
    let mut config = firefly_config::paths::config("firefly-rootslam.toml");
    let mut loop_closures = true;
    let mut mapping = true;
    while let Some(argument) = args.next() {
        if argument == "--help" || argument == "-h" {
            return Ok(None);
        }
        if argument == "--no-loop-closures" {
            loop_closures = false;
            continue;
        }
        if argument == "--no-mapping" {
            mapping = false;
            continue;
        }
        if argument == "--config" {
            config = args
                .next()
                .ok_or_else(|| argument_error("missing --config value"))?
                .into();
            continue;
        }
        return Err(argument_error(format!(
            "unknown argument: {}",
            argument.to_string_lossy()
        )));
    }
    Ok(Some(Arguments {
        config,
        loop_closures,
        mapping,
    }))
}

fn main() -> Result<()> {
    let Some(arguments) = parse()? else {
        let mut stdout = std::io::stdout();
        writeln!(stdout, "{USAGE}")
            .map_err(|error| Error::new(ErrorKind::Internal, "write help").with_source(error))?;
        return Ok(());
    };
    let mut config: Config =
        firefly_config::startup(&arguments.config, firefly_observability::init, |_| Ok(()))?;
    if !arguments.loop_closures {
        config.online.loops = false;
    }
    if !arguments.mapping {
        config.online.map = false;
    }
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
