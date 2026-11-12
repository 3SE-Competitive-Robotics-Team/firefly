//! 板载执行器：`Firefly/Control`（4 电机推力）→ `sysfs` PWM → 电调。
//!
//! 板端没有仿真器充当被控对象，因此本进程同时承担「被控对象属性发布者」这一
//! 角色：按部署配置周期性发布 `Firefly/Airframe`（质量/惯量/旋翼几何/推力上限），
//! 飞控订阅后装配控制参数。仿真侧这份描述来自 `MuJoCo` 模型，真机侧来自
//! `configs/motor.toml`，两边都是各自部署里的唯一来源。
//!
//! 安全语义：`Control` 断流、上锁或从未收到指令时，四路都输出零推力脉宽；
//! 进程退出前同样回到零推力脉宽，不把上一条推力留在电调上。
//!
//! 运行：`motor --config configs/motor.toml`

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
