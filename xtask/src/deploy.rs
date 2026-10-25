//! `cargo xtask deploy`：构建机载 release，打成 zstd 包并推到设备解包，最后拉起服务。
//!
//! 解包前会停掉 `firefly.service` 并清空 `bin`、`configs`、`models`，因此已退役的
//! 进程与配置不会残留；`logs` 保留。部署流程自己停的服务由自己拉起，失败不算部署成功。
//!
//! 设备端由同一个 `xtask` 二进制的 `unpack` 子命令解包，不依赖 tar 或 zstd CLI。
//!
//! 板载 adbd 走 legacy shell 协议，`adb shell` 不传递远端退出码（远端失败也返回 0），
//! 所以解包完整性与服务就绪都靠远端输出判定，不能依赖 `adb` 的退出状态。

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::archive::{self, Entry};
use crate::release;
use crate::target::{self, Target};

const DEFAULT_DEVICE: &str = "firefly-0001";
const DESTINATION: &str = "/opt/firefly";
const PACKAGE: &str = "dist/firefly-aarch64.zst";
const REMOTE_PACKAGE: &str = "/tmp/firefly-package.zst";
const REMOTE_UNPACKER: &str = "/tmp/firefly-xtask";
/// 只部署默认模型；`lightglue` 的权重带外部数据文件，两者必须同批部署。
const MODELS: &[&str] = &[
    "aliked-n16-k512.onnx",
    "lightglue-aliked-k512.onnx",
    "lightglue-aliked-k512.onnx.data",
];

pub fn run(args: &[String]) -> Result<(), String> {
    let device = parse_device(args)?;
    release::build(Target::Board)?;
    let package = package()?;
    push(&device, &package)?;
    start(&device)?;
    println!("部署完成");
    Ok(())
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

fn package() -> Result<PathBuf, String> {
    let root = target::workspace_root();
    let release_dir = Target::Board.release_dir(&root);
    let mut entries = Vec::new();

    for app in Target::Board.apps() {
        let source = release_dir.join(app);
        if !source.is_file() {
            return Err(format!(
                "missing {}: run cargo xtask release --target board",
                source.display()
            ));
        }
        entries.push(Entry {
            path: format!("bin/{app}"),
            mode: 0o755,
            source,
        });
    }

    let config_dir = root.join("configs");
    let mut configs = fs::read_dir(&config_dir)
        .map_err(|error| format!("read {}: {error}", config_dir.display()))?
        .map(|item| item.map(|entry| entry.path()))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| format!("read {}: {error}", config_dir.display()))?;
    configs.sort();
    for config in configs {
        let Some(name) = config.file_name() else {
            continue;
        };
        if config
            .extension()
            .is_none_or(|extension| extension != "toml")
            || !config.is_file()
        {
            continue;
        }
        entries.push(Entry {
            path: format!("configs/{}", name.to_string_lossy()),
            mode: 0o644,
            source: config,
        });
    }

    for model in MODELS {
        let source = root.join("models").join(model);
        if source.is_file() {
            entries.push(Entry {
                path: format!("models/{model}"),
                mode: 0o644,
                source,
            });
        } else {
            eprintln!(
                "warning: missing {}; aliked/lightglue will refuse to start",
                source.display()
            );
        }
    }

    let package = root.join(PACKAGE);
    if let Some(parent) = package.parent() {
        fs::create_dir_all(parent)
            .map_err(|error| format!("create {}: {error}", parent.display()))?;
    }
    let bytes = archive::write(&package, &entries)
        .map_err(|error| format!("write {}: {error}", package.display()))?;
    println!(
        "package {}（{:.1} MiB，{} 个文件）",
        package.display(),
        bytes as f64 / (1024.0 * 1024.0),
        entries.len()
    );
    Ok(package)
}

fn push(device: &str, package: &Path) -> Result<(), String> {
    check_device(device)?;
    let unpacker = Target::Board
        .release_dir(&target::workspace_root())
        .join("xtask");
    let package_arg = package.display().to_string();
    let unpacker_arg = unpacker.display().to_string();

    adb(device, &["push", &package_arg, REMOTE_PACKAGE])?;
    adb(device, &["push", &unpacker_arg, REMOTE_UNPACKER])?;
    adb(device, &["shell", &format!("chmod 755 {REMOTE_UNPACKER}")])?;

    // 部署是覆盖语义会留下已退役的二进制与配置，解包前先停服务并清空受管目录；
    // `logs` 与镜像预置的 `systemd` 不动。包已推到设备，这一步之后解包失败才会
    // 留下空的受管目录，所以清理放在推送之后。
    adb(device, &["shell", "systemctl stop firefly || true"])?;
    adb(
        device,
        &[
            "shell",
            &format!("rm -rf {DESTINATION}/bin {DESTINATION}/configs {DESTINATION}/models"),
        ],
    )?;
    adb(
        device,
        &[
            "shell",
            &format!("{REMOTE_UNPACKER} unpack {REMOTE_PACKAGE} {DESTINATION}"),
        ],
    )?;
    check_unpacked(device)?;
    adb(device, &["shell", "sync"])?;
    link(device)?;
    adb(
        device,
        &[
            "shell",
            &format!("rm -f {REMOTE_PACKAGE} {REMOTE_UNPACKER}"),
        ],
    )?;
    Ok(())
}

/// 解包失败在板载 adbd 上不会反映成 `adb` 的退出码，所以按应用名单逐个确认可执行位。
fn check_unpacked(device: &str) -> Result<(), String> {
    let expected = Target::Board
        .apps()
        .iter()
        .map(|app| format!("{DESTINATION}/bin/{app}"))
        .collect::<Vec<_>>()
        .join(" ");
    let missing = adb_output(
        device,
        &[
            "shell",
            &format!("for file in {expected}; do test -x $file || echo $file; done"),
        ],
    )?;
    if missing.trim().is_empty() {
        Ok(())
    } else {
        Err(format!("解包后缺少可执行文件：\n{missing}"))
    }
}

/// 部署流程自己停了 `firefly.service`，解包完成后必须重新拉起。
///
/// `systemd` 的 active 状态早于 supervisor 的 socket 就绪，只看 `is-active` 会把
/// “起来了但不应答”判成成功，所以以 `ffctl process status` 在超时内应答为就绪判据。
fn start(device: &str) -> Result<(), String> {
    adb(device, &["shell", "systemctl start firefly"])?;
    let probe = concat!(
        "for attempt in $(seq 1 20); do ",
        "if ffctl process status >/dev/null 2>&1; then echo SUPERVISOR_READY; break; fi; ",
        "sleep 0.5; ",
        "done"
    );
    if !adb_output(device, &["shell", probe])?.contains("SUPERVISOR_READY") {
        let status = adb_output(
            device,
            &["shell", "systemctl status firefly --no-pager -l | head -20"],
        )?;
        return Err(format!(
            "firefly.service 启动后 supervisor 未应答：\n{status}"
        ));
    }
    Ok(())
}

/// 把已部署的二进制软链到 `/usr/local/bin`，让任意目录、非交互 shell 都能直接调用。
/// 板上 `PATH` 默认包含该目录，因此不修改镜像的 shell 配置。
fn link(device: &str) -> Result<(), String> {
    let mut commands = Vec::with_capacity(Target::Board.apps().len());
    for app in Target::Board.apps() {
        commands.push(format!(
            "ln -sf {DESTINATION}/bin/{app} /usr/local/bin/{app}"
        ));
    }
    adb(device, &["shell", &commands.join(" && ")])?;
    Ok(())
}

fn check_device(device: &str) -> Result<(), String> {
    let output = Command::new("adb")
        .args(["-s", device, "get-state"])
        .output()
        .map_err(|error| format!("failed to run adb: {error}"))?;
    let state = String::from_utf8_lossy(&output.stdout).trim().to_owned();
    if !output.status.success() || state != "device" {
        return Err(format!(
            "adb device {device} is not ready（state: {state}）；先确认 adb devices 能看到它"
        ));
    }
    Ok(())
}

/// 捕获远端输出。板载 `adb shell` 不传递远端退出码，调用方应检查输出内容而非状态。
fn adb_output(device: &str, args: &[&str]) -> Result<String, String> {
    let output = Command::new("adb")
        .args(["-s", device])
        .args(args)
        .output()
        .map_err(|error| format!("failed to run adb: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "adb {} failed: {}\n{}",
            args.join(" "),
            output.status,
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    Ok(String::from_utf8_lossy(&output.stdout).into_owned())
}

fn adb(device: &str, args: &[&str]) -> Result<(), String> {
    let status = Command::new("adb")
        .args(["-s", device])
        .args(args)
        .status()
        .map_err(|error| format!("failed to run adb: {error}"))?;
    if !status.success() {
        return Err(format!("adb {} failed: {status}", args.join(" ")));
    }
    Ok(())
}
