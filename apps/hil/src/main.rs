//! 主机侧 USB HIL 桥：nusb 端点与本地 IPC 话题互转。
//!
//! 下行把 `Firefly/Imu`、双目/深度与 `Firefly/Airframe` 以及地面站指令
//! 送进板端；上行把板端 `Control`、里程计、姿态与标量诊断发布回本机话题，
//! 供 `firefly-sim`、`firefly-viz` 与验收评分消费。算法进程不感知 USB。
//!
//! 运行：`hil --config configs/hil.toml`；`hil --probe` 只打印 USB 描述符。

mod bridge;
mod config;
mod host;

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
