//! `cargo xtask deploy`：构建机载 release，打成 zstd 包并推到设备解包。
//!
//! 设备端由同一个 `xtask` 二进制的 `unpack` 子命令解包，不依赖 tar 或 zstd CLI。

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
    println!(
        "已部署到 {device}:{DESTINATION}；验证：adb -s {device} shell 'cd {DESTINATION} && ./bin/ffctl --version'"
    );
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
                "missing {}: run cargo xtask release --target aarch64",
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
    adb(
        device,
        &[
            "shell",
            &format!("{REMOTE_UNPACKER} unpack {REMOTE_PACKAGE} {DESTINATION}"),
        ],
    )?;
    adb(device, &["shell", "sync"])?;
    adb(
        device,
        &[
            "shell",
            &format!("rm -f {REMOTE_PACKAGE} {REMOTE_UNPACKER}"),
        ],
    )?;
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
