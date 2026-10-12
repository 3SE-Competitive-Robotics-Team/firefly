use std::process::{Command, ExitCode};

const APPS: &[&str] = &[
    "aliked",
    "fc",
    "ffctl",
    "lightglue",
    "localization",
    "planner",
    "quad",
    "render",
    "slam",
    "vio",
];

fn main() -> ExitCode {
    let Some(command) = std::env::args().nth(1) else {
        eprintln!("usage: cargo xtask build-all-release");
        return ExitCode::FAILURE;
    };

    if command != "build-all-release" {
        eprintln!("unknown xtask command: {command}\nusage: cargo xtask build-all-release");
        return ExitCode::FAILURE;
    }

    let mut cargo = Command::new("cargo");
    cargo.args(["build", "--release"]);
    for app in APPS {
        cargo.args(["-p", app]);
    }

    match cargo.status() {
        Ok(status) if status.success() => ExitCode::SUCCESS,
        Ok(_) => ExitCode::FAILURE,
        Err(error) => {
            eprintln!("failed to run cargo build: {error}");
            ExitCode::FAILURE
        }
    }
}
