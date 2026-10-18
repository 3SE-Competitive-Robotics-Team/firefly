//! 板载硬件的纯逻辑：像素格式换算、`BMI088` 寄存器表、电调推力映射。
//!
//! 设备 I/O 分别留在 `apps/camera`（`V4L2`）、`apps/imu`（`SPI`）与
//! `apps/motor`（`sysfs` PWM）。本 crate 不依赖平台相关库，因此能在宿主机上
//! 编译并测试——驱动进程本身只在 aarch64 Linux 上构建。

pub mod bmi088;
pub mod esc;
pub mod pixel;
