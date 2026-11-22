//! 控制断流、上锁、尚无指令或退出时，四路 PWM 都回到零推力脉宽。

mod actuator;
#[cfg(target_os = "linux")]
mod hil;

/// 输出电机控制信号
#[derive(usage_rs::Cli)]
#[usage(bin = "motor")]
struct Arguments {
    /// 电机驱动配置文件
    #[usage(long)]
    config: Option<std::path::PathBuf>,
    /// 使用板端 HIL 接口
    #[usage(long, conflicts = "--config")]
    hil: bool,
}

fn main() -> std::process::ExitCode {
    firefly_error::report("motor", run_app())
}

fn run_app() -> firefly_error::Result<()> {
    let args = Arguments::parse();
    if args.hil {
        #[cfg(target_os = "linux")]
        return hil::run();
        #[cfg(not(target_os = "linux"))]
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::Unsupported,
            "HIL 接口需要 Linux",
        ));
    }
    let config = args
        .config
        .unwrap_or_else(|| firefly_config::paths::config("motor.toml"));
    actuator::run(&config)
}
