#![cfg(feature = "realsense")]
use std::{path::PathBuf, process::Command};

#[test]
fn help_and_invalid_arguments_do_not_start_device() {
    let output = Command::new(env!("CARGO_BIN_EXE_firefly-basalt-live"))
        .arg("--help")
        .output()
        .unwrap();
    assert!(output.status.success());
    assert!(String::from_utf8(output.stdout).unwrap().contains("SIGINT"));
    let output = Command::new(env!("CARGO_BIN_EXE_firefly-basalt-live"))
        .arg("--invalid")
        .output()
        .unwrap();
    assert!(!output.status.success());
}

/// Runs the actual SDK and application startup; requires no connected T265.
#[test]
#[ignore = "requires an SDK installation and a host with no RealSense devices"]
fn missing_device_fails_without_creating_trajectory() {
    let config = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/reference/live.toml");
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let trajectory = root.join("logs/basalt-live-device-test.tum");
    assert!(!trajectory.exists());
    let output = Command::new(env!("CARGO_BIN_EXE_firefly-basalt-live"))
        .current_dir(root)
        .args(["--config", config.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(
        String::from_utf8(output.stderr)
            .unwrap()
            .contains("No RealSense T265 device")
    );
    assert!(!trajectory.exists());
}
