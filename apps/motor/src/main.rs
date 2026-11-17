//! 控制断流、上锁、尚无指令或退出时，四路 PWM 都回到零推力脉宽。

mod actuator;
#[cfg(target_os = "linux")]
mod hil;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(target_os = "linux")]
    if std::env::args().any(|arg| arg == "--hil") {
        return hil::run();
    }
    actuator::run()
}
