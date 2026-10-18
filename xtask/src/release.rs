//! `cargo xtask release`：构建指定目标的全部 release 应用。

use std::process::Command;

use crate::container;
use crate::target::{self, Target};

pub fn run(args: &[String]) -> Result<(), String> {
    let mut target = Target::Desktop;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--target" => {
                let value = it
                    .next()
                    .ok_or_else(|| "missing --target value (host or aarch64)".to_owned())?;
                target = Target::parse(value)?;
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    build(target)
}

pub fn build(target: Target) -> Result<(), String> {
    match target {
        Target::Desktop => build_desktop(target),
        Target::Board => container::build(&target::workspace_root(), target.apps(), true),
    }
}

fn build_desktop(target: Target) -> Result<(), String> {
    println!("release {}（host）", target.name());
    let mut command = Command::new("cargo");
    command
        .current_dir(target::workspace_root())
        .args(["build", "--release"]);
    for app in target.apps() {
        command.args(["-p", app]);
    }

    let status = command
        .status()
        .map_err(|error| format!("failed to run cargo: {error}"))?;
    if !status.success() {
        return Err(format!("cargo build failed: {status}"));
    }
    Ok(())
}
