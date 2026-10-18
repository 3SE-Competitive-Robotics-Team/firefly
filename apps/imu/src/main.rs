//! 板载 IMU 源：`BMI088`（SPI）→ `Firefly/Imu`。
//!
//! 加速度计与陀螺是同一封装内的两片独立 SPI 从机，各占一条片选，因此配置里是
//! 两个设备节点。本进程只做寄存器初始化、按固定速率取原始计数并换算成
//! `rad/s` 与 `m/s²` 发布；姿态解算（`firefly-imu` 的 ESKF）在飞控进程内。
//!
//! 时间戳取 `CLOCK_MONOTONIC` 秒，与 `camera` 共用同一时钟域。
//!
//! 运行：`imu --config configs/imu.toml`

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
