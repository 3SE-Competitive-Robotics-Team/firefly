//! 原生 aarch64 Debian 构建；`v4l2-sys-mit` 的 bindgen 需要 `libclang-dev`。

use firefly_error::{Error, ErrorKind, IoResultExt, Result};
use std::fs;
use std::path::Path;
use std::process::Command;

/// Rust 1.100.0 + Debian trixie。升级镜像必须同时更新 digest。
pub const IMAGE: &str =
    "rust:1.100.0-trixie@sha256:e84068000361bf3d4a5ba2b7647f62893528b1eb9f847d7a23b493f5ce8ea254";
const TOOLCHAIN: &str = "1.100.0";
/// Apple container 默认只给约 1GB，release 并行编译会被 OOM kill。
const MEMORY: &str = "8G";
const BUILDER_IMAGE: &str = "firefly-board-builder";

pub fn build(root: &Path, apps: &[&str], with_unpacker: bool) -> Result<()> {
    let uid = id("-u")?;
    let gid = id("-g")?;
    let cache = root.join("target/container");
    for name in ["cargo", "home", "cache", "tmp"] {
        let path = cache.join(name);
        fs::create_dir_all(&path).at_path("create", &path)?;
    }

    let image = builder_tag();
    ensure_builder_image(&cache, &image)?;
    println!("release board（container {image}）");
    let mount = format!("{}:/work", root.display());
    let mut command = Command::new("container");
    command.args([
        "run",
        "--rm",
        "--arch",
        "arm64",
        "--memory",
        MEMORY,
        "--uid",
        &uid,
        "--gid",
        &gid,
        "--volume",
        &mount,
        "--workdir",
        "/work",
        "--env",
        &format!("RUSTUP_TOOLCHAIN={TOOLCHAIN}"),
        "--env",
        "RUSTUP_AUTO_INSTALL=0",
        "--env",
        "CARGO_HOME=/work/target/container/cargo",
        "--env",
        "CARGO_TARGET_DIR=/work/target/container/aarch64",
        "--env",
        "HOME=/work/target/container/home",
        "--env",
        "XDG_CACHE_HOME=/work/target/container/cache",
        "--env",
        "TMPDIR=/work/target/container/tmp",
        &image,
        "cargo",
        "build",
        "--release",
        "--locked",
    ]);
    for app in apps {
        command.args(["-p", app]);
    }
    if with_unpacker {
        command.args(["-p", "xtask"]);
    }

    let status = command.status().map_err(|error| {
        Error::from(error)
            .with_operation("run container")
            .with_context("hint", "安装 Apple container 并运行 container system start")
    })?;
    if !status.success() {
        return Err(Error::new(
            ErrorKind::Unavailable,
            format!(
                "container build failed: {status}；若提示 XPC 连接失败，先运行 container system start"
            ),
        ));
    }
    Ok(())
}

/// 派生镜像标签：工具链 + 基础镜像 digest 前缀。换 digest 即换标签，
/// 不会静默复用旧层。
fn builder_tag() -> String {
    let digest = IMAGE.split_once("sha256:").map_or("", |(_, digest)| digest);
    let short = digest.get(..12).unwrap_or(digest);
    format!("{BUILDER_IMAGE}:{TOOLCHAIN}-{short}-opencv")
}

/// 准备带有 bindgen 和双目匹配所需开发库的板端构建镜像。
fn ensure_builder_image(cache: &Path, image: &str) -> Result<()> {
    let inspect = Command::new("container")
        .args(["image", "inspect", image])
        .output()
        .map_err(|error| Error::from(error).with_operation("failed to run container"))?;
    if inspect.status.success() {
        return Ok(());
    }

    let containerfile = cache.join("Containerfile");
    let recipe = format!(
        "FROM {IMAGE}\nRUN apt-get update && apt-get install -y --no-install-recommends libclang-dev libopencv-core-dev libopencv-imgproc-dev libopencv-video-dev && rm -rf /var/lib/apt/lists/*\n"
    );
    fs::write(&containerfile, recipe).at_path("write", &containerfile)?;
    println!("构建板端镜像 {image}");
    let status = Command::new("container")
        .args(["build", "--arch", "arm64", "--tag", image, "--file"])
        .arg(&containerfile)
        .arg(cache)
        .status()
        .map_err(|error| Error::from(error).with_operation("failed to run container build"))?;
    if !status.success() {
        return Err(Error::new(
            ErrorKind::Unavailable,
            format!("container build failed: {status}；构建镜像需要能访问 Debian 软件源"),
        ));
    }
    Ok(())
}

fn id(flag: &str) -> Result<String> {
    let output = Command::new("id")
        .arg(flag)
        .output()
        .map_err(|error| Error::from(error).with_operation("failed to run id"))?;
    if !output.status.success() {
        return Err(Error::new(
            ErrorKind::Unavailable,
            format!("id {flag} failed: {}", output.status),
        ));
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_owned())
}
