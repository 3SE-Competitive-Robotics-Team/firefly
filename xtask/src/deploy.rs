//! 部署替换 bin/configs/models，保留 logs。安装成功后启动服务，以 supervisor 应答确认就绪。
//! 板载 legacy adb shell 的退出码反映传输结果，远端步骤通过输出标记确认成功。

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
const LANDING_MODELS: &[&str] = &[
    "h-marker-yolo26n-cif-v1/manifest.json",
    "h-marker-yolo26n-cif-v1/int8.onnx",
];

pub fn run(args: &[String]) -> Result<(), String> {
    let device = parse_device(args)?;
    deploy(&device)?;
    println!("部署完成");
    Ok(())
}

/// 构建并部署机载 release，最后拉起 `firefly.service` 并确认 supervisor 就绪。
pub(crate) fn deploy(device: &str) -> Result<(), String> {
    release::build(Target::Board)?;
    let package = package()?;
    push(device, &package)?;
    start(device)
}

/// 在板上执行一条 shell 命令；远端退出码不可靠，调用方按输出判定。
pub(crate) fn shell(device: &str, command: &str) -> Result<(), String> {
    adb(device, &["shell", command])
}

/// 捕获远端输出（板载 `adb shell` 不传递远端退出码）。
pub(crate) fn adb_output(device: &str, args: &[&str]) -> Result<String, String> {
    adb_output_inner(device, args)
}

fn parse_device(args: &[String]) -> Result<String, String> {
    let mut device = DEFAULT_DEVICE.to_owned();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--device" => device.clone_from(
                it.next()
                    .ok_or_else(|| "missing --device value".to_owned())?,
            ),
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

    let runtime = root.join("models/npu");
    if !runtime.join("libonnxruntime.so").is_file() || !runtime.join("manifest.json").is_file() {
        return Err("missing VSINPU runtime bundle; see scripts/build_npu_runtime.py".into());
    }
    for file in fs::read_dir(&runtime).map_err(|e| e.to_string())? {
        let file = file.map_err(|e| e.to_string())?;
        if file.path().is_file() {
            entries.push(Entry {
                path: format!("models/npu/{}", file.file_name().to_string_lossy()),
                mode: 0o644,
                source: file.path(),
            });
        }
    }
    for model in LANDING_MODELS {
        let source = root.join("models").join(model);
        if !source.is_file() {
            return Err(format!(
                "missing {}; export the quantized landing model before deployment",
                source.display()
            ));
        }
        entries.push(Entry {
            path: format!("models/{model}"),
            mode: 0o644,
            source,
        });
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
    remote_step(device, &format!("chmod 755 {REMOTE_UNPACKER}"))?;

    prepare_destination_with(|args| adb_output(device, args))?;
    unpack_with(|args| adb_output(device, args))?;
    check_unpacked(device)?;
    remote_step(device, "sync")?;
    link(device)?;
    remote_step(device, &format!("rm -f {REMOTE_PACKAGE} {REMOTE_UNPACKER}"))?;
    Ok(())
}

fn prepare_destination_with(
    mut output: impl FnMut(&[&str]) -> Result<String, String>,
) -> Result<(), String> {
    remote_step_with("systemctl stop firefly", &mut output)?;
    remote_step_with(
        &format!("rm -rf {DESTINATION}/bin {DESTINATION}/configs {DESTINATION}/models"),
        &mut output,
    )
}

fn remote_step(device: &str, command: &str) -> Result<(), String> {
    remote_step_with(command, |args| adb_output(device, args))
}

fn remote_step_with(
    command: &str,
    mut output: impl FnMut(&[&str]) -> Result<String, String>,
) -> Result<(), String> {
    const COMPLETE: &str = "FIREFLY_STEP_COMPLETE";
    let script = format!("({command}) && echo {COMPLETE}");
    let result = output(&["shell", &script])?;
    if !has_marker(&result, COMPLETE) {
        return Err(format!("远端步骤未完成（{command}）：\n{result}"));
    }
    Ok(())
}

fn has_marker(output: &str, marker: &str) -> bool {
    output.lines().any(|line| line.trim() == marker)
}

fn unpack_with(mut output: impl FnMut(&[&str]) -> Result<String, String>) -> Result<(), String> {
    const COMPLETE: &str = "FIREFLY_UNPACK_COMPLETE";
    let command =
        format!("{REMOTE_UNPACKER} unpack {REMOTE_PACKAGE} {DESTINATION} && echo {COMPLETE}");
    let result = output(&["shell", &command])?;
    if !has_marker(&result, COMPLETE) {
        return Err(format!("远端解包未完成：\n{result}"));
    }
    Ok(())
}

/// 检查机载清单中的每个程序在部署路径上都有执行权限。
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

fn start(device: &str) -> Result<(), String> {
    remote_step(device, "systemctl start firefly")?;
    let probe = concat!(
        "for attempt in $(seq 1 20); do ",
        "if ffctl process status >/dev/null 2>&1; then echo SUPERVISOR_READY; break; fi; ",
        "sleep 0.5; ",
        "done"
    );
    if !has_marker(&adb_output(device, &["shell", probe])?, "SUPERVISOR_READY") {
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
    remote_step(device, &commands.join(" && "))?;
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

fn adb_output_inner(device: &str, args: &[&str]) -> Result<String, String> {
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

#[cfg(test)]
mod tests {
    use super::{has_marker, prepare_destination_with, remote_step_with, unpack_with};

    #[test]
    fn failed_stop_returns_before_sending_cleanup_command() {
        let mut commands = Vec::new();
        let result = prepare_destination_with(|args| {
            commands.push(args[1].to_owned());
            Ok("Failed to stop firefly.service".into())
        });
        assert!(result.is_err());
        assert_eq!(commands.len(), 1);
        assert!(commands[0].contains("systemctl stop firefly"));
        assert!(!commands[0].contains("rm -rf"));
    }

    #[test]
    fn remote_transitions_require_exact_success_markers() {
        assert!(!has_marker("NOT_SUPERVISOR_READY", "SUPERVISOR_READY"));
        assert!(!has_marker("echo SUPERVISOR_READY", "SUPERVISOR_READY"));
        assert!(has_marker(
            "starting\r\nSUPERVISOR_READY\r\n",
            "SUPERVISOR_READY"
        ));
        assert!(
            remote_step_with("false", |args| {
                let output = std::process::Command::new("sh")
                    .args(["-c", args[1]])
                    .output()
                    .unwrap();
                Ok(String::from_utf8(output.stdout).unwrap())
            })
            .is_err()
        );
        assert!(
            remote_step_with("true", |args| {
                let output = std::process::Command::new("sh")
                    .args(["-c", args[1]])
                    .output()
                    .unwrap();
                Ok(String::from_utf8(output.stdout).unwrap())
            })
            .is_ok()
        );
    }

    #[test]
    fn unpack_requires_completion_marker_and_reports_remote_output() {
        let diagnostic = "configs/fc.toml: permission denied";
        let error = unpack_with(|args| {
            assert_eq!(args[0], "shell");
            assert!(args[1].contains(" && echo FIREFLY_UNPACK_COMPLETE"));
            Ok(format!("bin/fc\n{diagnostic}\n"))
        })
        .unwrap_err();
        assert!(error.contains(diagnostic), "{error}");
        assert!(unpack_with(|_| Ok("unpacked\nFIREFLY_UNPACK_COMPLETE\n".into())).is_ok());
    }
}
