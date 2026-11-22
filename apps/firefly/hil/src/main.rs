mod bridge;
mod config;
mod host;
mod mission;

use std::process::ExitCode;

use config::Config;

/// 运行 USB HIL 主机桥接
#[derive(usage_rs::Cli)]
#[usage(bin = "hil")]
struct Arguments {
    /// 应用配置文件
    #[usage(long)]
    config: Option<std::path::PathBuf>,
    /// 枚举设备接口
    #[usage(long, conflicts = "--check")]
    probe: bool,
    /// 打开接口并检查端点
    #[usage(long)]
    check: bool,
}

fn main() -> ExitCode {
    firefly_error::report("hil", run())
}

fn run() -> firefly_error::Result<()> {
    let args = Arguments::parse();
    let path = args.config.unwrap_or_else(config::default_config);
    let config = firefly_config::startup::<Config>(&path, firefly_observability::init, |config| {
        config.validate()
    })?;
    if args.probe {
        return host::probe(&config);
    }
    if args.check {
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
