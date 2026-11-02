//! `cargo xtask deploy-hil`：部署机载 release，切换到 HIL gadget 并校验主机枚举。
//!
//! 部署沿用普通 `deploy`（含服务就绪检查）；之后把活动进程清单切到 HIL、
//! 由 `hil-gadget up` 重建 gadget。切换后 `adb` 会消失，这是设计使然：
//! 这块 UDC 的端点不足以同时承载 adb 与 HIL。主机侧用 nusb 直接认领两个
//! 厂商接口；`hil --probe` 通过即视为链路就绪。
//!
//! 板端 HIL 同时只跑一个估计器，`--estimator` 决定用哪份进程清单：
//! `okvis2-x` 跑 `processes-hil.toml`，`rootslam` 跑
//! `processes-hil-rootslam.toml`。

use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

use crate::deploy;
use crate::target;

const DEFAULT_DEVICE: &str = "firefly-0001";
/// 等待 HIL 设备枚举的上限。
const ENUMERATION_TIMEOUT: Duration = Duration::from_secs(20);
/// 板端 HIL 可选的估计器及其进程清单。
const ESTIMATORS: [(&str, &str); 2] = [
    ("okvis2-x", "processes-hil.toml"),
    ("rootslam", "processes-hil-rootslam.toml"),
];

pub fn run(args: &[String]) -> Result<(), String> {
    let (device, estimator) = parse_arguments(args)?;
    let manifest = manifest_for(&estimator)?;
    deploy::deploy(&device)?;
    ensure_hil_processes(&device, manifest)?;
    build_host_bridge()?;
    println!("切换 HIL gadget（adb 将临时消失）");
    let command = format!(
        "rm -f /opt/firefly/logs/hil/up.pid /opt/firefly/logs/hil/watchdog.pid; \
         /usr/sbin/start-stop-daemon --start --background --make-pidfile \
         --pidfile /opt/firefly/logs/hil/up.pid \
         --startas /opt/firefly/bin/hil-gadget -- up {manifest}"
    );
    // 切换会杀掉 adbd，adb 的返回状态不可靠；以主机枚举为准。
    let _ = deploy::shell(&device, &command);
    wait_for_enumeration()
}

fn parse_arguments(args: &[String]) -> Result<(String, String), String> {
    let mut device = DEFAULT_DEVICE.to_owned();
    let mut estimator = ESTIMATORS[0].0.to_owned();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--device" => {
                device = it
                    .next()
                    .ok_or_else(|| "missing --device value".to_owned())?
                    .clone();
            }
            "--estimator" => {
                estimator = it
                    .next()
                    .ok_or_else(|| "missing --estimator value".to_owned())?
                    .clone();
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok((device, estimator))
}

/// 估计器名到板端进程清单的映射；未知估计器直接报错，不透传到板端命令。
fn manifest_for(estimator: &str) -> Result<&'static str, String> {
    ESTIMATORS
        .iter()
        .find(|(name, _)| *name == estimator)
        .map(|(_, manifest)| *manifest)
        .ok_or_else(|| {
            format!(
                "unknown estimator {estimator}: expected {}",
                ESTIMATORS
                    .iter()
                    .map(|(name, _)| *name)
                    .collect::<Vec<_>>()
                    .join(" or ")
            )
        })
}

/// `deploy` 会把 `configs/*.toml` 一起发布，这里只确认 HIL 清单确实在位。
fn ensure_hil_processes(device: &str, manifest: &str) -> Result<(), String> {
    let output = deploy::adb_output(
        device,
        &[
            "shell",
            &format!("test -f /opt/firefly/configs/{manifest} && echo HIL_PROCESSES_OK"),
        ],
    )?;
    if !output.contains("HIL_PROCESSES_OK") {
        return Err(format!("板上缺少 configs/{manifest}；检查 deploy 包"));
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
