#[cfg(target_os = "linux")]
mod capture;
#[cfg(target_os = "linux")]
mod hil;

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::args().any(|arg| arg == "--hil") {
        hil::run()
    } else {
        capture::run()
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("camera 依赖 V4L2，只在 Linux 上构建运行");
    std::process::exit(1);
}
