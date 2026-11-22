#[cfg(target_os = "linux")]
mod source;

#[cfg(target_os = "linux")]
use source::run;

/// 采集 IMU 数据
#[derive(usage_rs::Cli)]
#[usage(bin = "imu")]
struct Arguments {
    /// 应用配置文件
    #[usage(long)]
    config: Option<std::path::PathBuf>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Arguments::parse();
    let config = args
        .config
        .unwrap_or_else(|| firefly_config::paths::config("imu.toml"));
    run(&config)
}

#[cfg(not(target_os = "linux"))]
fn run(_config: &std::path::Path) -> Result<(), Box<dyn std::error::Error>> {
    Err("imu 依赖 Linux spidev".into())
}
