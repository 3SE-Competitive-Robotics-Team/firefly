//! 板载相机源：`V4L2` 采集双目与深度 → `Firefly/CameraLeft`、`Firefly/CameraRight`、
//! `Firefly/Depth`。
//!
//! 板载相机走 `MIPI-CSI`（见 `docs/hw/spec.md`），在 Linux 上表现为 `/dev/videoN`；
//! 本进程只做格式协商、帧取回与消息发布，标定与算法留在消费者侧。深度可选：
//! 配置里不写 `[depth]` 就不发布深度话题，`planner` 会按无深度源等待。
//!
//! 时间戳取 `CLOCK_MONOTONIC` 秒，与 `imu` 共用同一时钟域——板载各进程据此对齐
//! 相机与 IMU，仿真侧的 `sim_time` 语义不适用于真机。
//!
//! 运行：`camera --config configs/camera.toml`

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
