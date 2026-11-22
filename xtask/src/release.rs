use std::process::Command;

use crate::container;
use crate::target::{self, Target};

pub fn build(target: Target) -> Result<(), String> {
    match target {
        Target::Desktop => build_desktop(target),
        Target::Board => container::build(&target::workspace_root(), target.apps(), true),
    }
}

fn build_desktop(target: Target) -> Result<(), String> {
    println!("release {}", target.name());
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
