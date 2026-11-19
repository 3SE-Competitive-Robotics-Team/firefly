mod bridge;
mod config;
mod host;
mod mission;

use std::process::ExitCode;

use config::Config;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("hil: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let mut path = config::default_config();
    let mut probe = false;
    let mut check = false;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--probe" => probe = true,
            "--check" => check = true,
            "--config" => {
                path = args
                    .next()
                    .map(std::path::PathBuf::from)
                    .ok_or("missing --config value")?;
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let config = firefly_config::startup::<Config>(&path, firefly_observability::init, |config| {
        config.validate()
    })
    .map_err(|error| error.to_string())?;
    if probe {
        return host::probe(&config);
    }
    if check {
        let link = host::HostLink::open(&config)?;
        println!(
            "HIL 接口就绪：interface={} out=0x{:02x} in=0x{:02x}",
            link.interface.interface_number(),
            link.out_endpoint,
            link.in_endpoint
        );
        return Ok(());
    }
    bridge::run(&config)
}
