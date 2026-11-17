#[cfg(target_os = "linux")]
mod source;

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    source::run()
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("imu 依赖 Linux spidev，只在 Linux 上构建运行");
    std::process::exit(1);
}
