//! HIL 切换后 ADB 不可用；以 `hil --check` 对单个厂商接口及两个端点的检查确认就绪。

use firefly_error::{Error, ErrorKind, Result};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

use crate::deploy;
use crate::target;

/// 等待 HIL 设备枚举的上限。
const ENUMERATION_TIMEOUT: Duration = Duration::from_secs(20);
const MANIFEST: &str = "processes-hil.toml";

pub fn run(device: &str) -> Result<()> {
    deploy::deploy(device)?;
    ensure_hil_processes(device)?;
    build_host_bridge()?;
    println!("切换 HIL gadget（adb 将临时消失）");
    let command = "rm -f /opt/firefly/logs/hil/up.pid; \
         taskset -c 0 /usr/sbin/start-stop-daemon --start --background --make-pidfile \
         --pidfile /opt/firefly/logs/hil/up.pid \
         --startas /opt/firefly/bin/hil-gadget -- up";
    // 切换会杀掉 adbd，adb 的返回状态不可靠；以主机枚举为准。
    let _ = deploy::shell(device, command);
    wait_for_enumeration()
}

/// `deploy` 会把 `configs/*.toml` 一起发布，这里只确认 HIL 清单确实在位。
fn ensure_hil_processes(device: &str) -> Result<()> {
    let output = deploy::adb_output(
        device,
        &[
            "shell",
            &format!("test -f /opt/firefly/configs/{MANIFEST} && echo HIL_PROCESSES_OK"),
        ],
    )?;
    if !output.contains("HIL_PROCESSES_OK") {
        return Err(Error::new(
            ErrorKind::NotFound,
            format!("板上缺少 configs/{MANIFEST}；检查 deploy 包"),
        ));
    }
    Ok(())
}

fn build_host_bridge() -> Result<()> {
    let status = Command::new("cargo")
        .current_dir(target::workspace_root())
        .args(["build", "--release", "-p", "hil"])
        .status()
        .map_err(|error| Error::from(error).with_operation("failed to run cargo"))?;
    if !status.success() {
        return Err(Error::new(
            ErrorKind::Unavailable,
            format!("cargo build -p hil failed: {status}"),
        ));
    }
    Ok(())
}

fn wait_for_enumeration() -> Result<()> {
    let probe = target::workspace_root().join("target/release/hil");
    let deadline = Instant::now() + ENUMERATION_TIMEOUT;
    while Instant::now() < deadline {
        let output = Command::new(&probe)
            .current_dir(target::workspace_root())
            .arg("--check")
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
    Err(Error::new(
        ErrorKind::Timeout,
        format!(
            "HIL 设备未在 {}s 内枚举；看门狗会在 60s 无数据后恢复 adb，届时用 journalctl -b -t firefly-hil-gadget 查看切换日志",
            ENUMERATION_TIMEOUT.as_secs()
        ),
    ))
}
