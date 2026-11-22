#[cfg(target_os = "linux")]
mod capture;
#[cfg(target_os = "linux")]
mod hil;

#[cfg(target_os = "linux")]
use capture::run;

/// 采集相机图像
#[derive(usage_rs::Cli)]
#[usage(bin = "camera")]
struct Arguments {
    /// 相机采集配置文件
    #[usage(long)]
    config: Option<std::path::PathBuf>,
    /// 使用板端 HIL 接口
    #[usage(long, conflicts = "--config")]
    hil: bool,
}

fn main() -> std::process::ExitCode {
    firefly_error::report("camera", run_app())
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
        .unwrap_or_else(|| firefly_config::paths::config("camera.toml"));
    run(&config)
}

#[cfg(not(target_os = "linux"))]
fn run(_config: &std::path::Path) -> firefly_error::Result<()> {
    Err(firefly_error::Error::new(
        firefly_error::ErrorKind::Unsupported,
        "camera 依赖 Linux V4L2",
    ))
}
