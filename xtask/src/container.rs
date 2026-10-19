//! 机载构建：在 Apple container 里用 aarch64 Debian trixie 原生编译。
//!
//! 基础镜像固定到 digest，系统 glibc 2.41 与 Orange Pi Zero 3W 一致；ONNX Runtime
//! 静态库因此不需要交叉 sysroot。构建产物在 `target/container/aarch64/release`。
//!
//! 构建镜像在基础镜像上加装 `libclang-dev`：`v4l` 依赖的 `v4l2-sys-mit` 用 bindgen
//! 生成 V4L2 绑定，需要 libclang 的共享库。`Containerfile` 由基础镜像常量生成，
//! 保证 digest 只有一处来源。

use std::fs;
use std::path::Path;
use std::process::Command;

/// Rust 1.98.1 + Debian trixie。升级镜像必须同时更新 digest。
pub const IMAGE: &str =
    "rust:1.98.1-trixie@sha256:a8a5f0a1e5fe7dfe1d352591e4a1c7dd2c08fd70475cae872cf3458ba0df0546";
const TOOLCHAIN: &str = "1.98.1";
/// Apple container 默认只给约 1GB，release 并行编译会被 OOM kill。
const MEMORY: &str = "8G";
/// 派生构建镜像的仓库名。
const BUILDER_IMAGE: &str = "firefly-board-builder";

pub fn build(root: &Path, apps: &[&str], with_unpacker: bool) -> Result<(), String> {
    let uid = id("-u")?;
    let gid = id("-g")?;
    let cache = root.join("target/container");
    for name in ["cargo", "home", "cache", "tmp"] {
        let path = cache.join(name);
        fs::create_dir_all(&path).map_err(|error| format!("create {}: {error}", path.display()))?;
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
        format!(
            "failed to run container: {error}；安装 Apple container 并运行 container system start"
        )
    })?;
    if !status.success() {
        return Err(format!(
            "container build failed: {status}；若提示 XPC 连接失败，先运行 container system start"
        ));
    }
    Ok(())
}

/// 派生镜像标签：工具链 + 基础镜像 digest 前缀。换 digest 即换标签，
/// 不会静默复用旧层。
fn builder_tag() -> String {
    let digest = IMAGE.split_once("sha256:").map_or("", |(_, digest)| digest);
    let short = digest.get(..12).unwrap_or(digest);
    format!("{BUILDER_IMAGE}:{TOOLCHAIN}-{short}")
}

/// 生成并（首次）构建构建镜像：基础镜像 + `libclang-dev`。
fn ensure_builder_image(cache: &Path, image: &str) -> Result<(), String> {
    let inspect = Command::new("container")
        .args(["image", "inspect", image])
        .output()
        .map_err(|error| format!("failed to run container: {error}"))?;
    if inspect.status.success() {
        return Ok(());
    }

    let containerfile = cache.join("Containerfile");
    let recipe = format!(
        "FROM {IMAGE}\nRUN apt-get update && apt-get install -y --no-install-recommends libclang-dev && rm -rf /var/lib/apt/lists/*\n"
    );
    fs::write(&containerfile, recipe)
        .map_err(|error| format!("write {}: {error}", containerfile.display()))?;
    println!("构建板载构建镜像 {image}（首次需要下载 libclang）");
    let status = Command::new("container")
        .args(["build", "--arch", "arm64", "--tag", image, "--file"])
        .arg(&containerfile)
        .arg(cache)
        .status()
        .map_err(|error| format!("failed to run container build: {error}"))?;
    if !status.success() {
        return Err(format!(
            "container build failed: {status}；构建镜像需要能访问 Debian 软件源"
        ));
    }
    Ok(())
}

fn id(flag: &str) -> Result<String, String> {
    let output = Command::new("id")
        .arg(flag)
        .output()
        .map_err(|error| format!("failed to run id: {error}"))?;
    if !output.status.success() {
        return Err(format!("id {flag} failed: {}", output.status));
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_owned())
}
