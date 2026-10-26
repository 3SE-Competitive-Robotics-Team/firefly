//! `cargo xtask deploy-hil`：部署机载 release，切换到 HIL gadget 并校验主机枚举。
//!
//! 部署沿用普通 `deploy`（含服务就绪检查）；之后把活动进程清单切到 HIL、
//! 由 `hil-gadget up` 重建 gadget。切换后 `adb` 会消失，这是设计使然：
//! 这块 UDC 的端点不足以同时承载 adb 与 HIL。主机侧用 nusb 直接认领两个
//! 厂商接口；`hil --probe` 通过即视为链路就绪。

use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

use crate::deploy;
use crate::target;

const DEFAULT_DEVICE: &str = "firefly-0001";
/// 等待 HIL 设备枚举的上限。
const ENUMERATION_TIMEOUT: Duration = Duration::from_secs(20);

pub fn run(args: &[String]) -> Result<(), String> {
    let device = parse_device(args)?;
    deploy::deploy(&device)?;
    ensure_hil_processes(&device)?;
    build_host_bridge()?;
    println!("切换 HIL gadget（adb 将临时消失）");
    let command = concat!(
        "rm -f /opt/firefly/logs/hil/up.pid /opt/firefly/logs/hil/watchdog.pid; ",
        "/usr/sbin/start-stop-daemon --start --background --make-pidfile ",
        "--pidfile /opt/firefly/logs/hil/up.pid ",
        "--startas /opt/firefly/bin/hil-gadget -- up"
    );
    // 切换会杀掉 adbd，adb 的返回状态不可靠；以主机枚举为准。
    let _ = deploy::shell(&device, command);
    wait_for_enumeration()
}

fn parse_device(args: &[String]) -> Result<String, String> {
    let mut device = DEFAULT_DEVICE.to_owned();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--device" => {
                device = it
                    .next()
                    .ok_or_else(|| "missing --device value".to_owned())?
                    .clone();
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(device)
}

/// `deploy` 会把 `configs/*.toml` 一起发布，这里只确认 HIL 清单确实在位。
fn ensure_hil_processes(device: &str) -> Result<(), String> {
    let output = deploy::adb_output(
        device,
        &[
            "shell",
            "test -f /opt/firefly/configs/processes-hil.toml && echo HIL_PROCESSES_OK",
        ],
    )?;
    if !output.contains("HIL_PROCESSES_OK") {
        return Err("板上缺少 configs/processes-hil.toml；检查 deploy 包".to_owned());
    }
    Ok(())
}

fn build_host_bridge() -> Result<(), String> {
    let status = Command::new("cargo")
        .current_dir(target::workspace_root())
        .args(["build", "--release", "-p", "hil"])
        .status()
        .map_err(|error| format!("failed to run cargo: {error}"))?;
    if !status.success() {
        return Err(format!("cargo build -p hil failed: {status}"));
    }
    Ok(())
}

fn wait_for_enumeration() -> Result<(), String> {
    let probe = target::workspace_root().join("target/release/hil");
    let deadline = Instant::now() + ENUMERATION_TIMEOUT;
    while Instant::now() < deadline {
        let output = Command::new(&probe)
            .current_dir(target::workspace_root())
            .args(["--check", "--config", "configs/hil.toml"])
            .output();
        if let Ok(output) = output
            && output.status.success()
        {
            println!(
                "HIL 链路就绪：\n{}",
                String::from_utf8_lossy(&output.stdout)
            );
            return Ok(());
        }
        thread::sleep(Duration::from_millis(500));
    }
    Err(format!(
        "HIL 设备未在 {}s 内枚举；看门狗会在 60s 无数据后恢复 adb，届时查看 /opt/firefly/logs/hil/up.log",
        ENUMERATION_TIMEOUT.as_secs()
    ))
}
