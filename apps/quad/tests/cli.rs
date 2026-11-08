use std::process::Command;

#[test]
fn help_and_usage_errors_exit_before_resources() {
    let binary = env!("CARGO_BIN_EXE_quad");
    let help = Command::new(binary).arg("--help").output().unwrap();
    assert!(help.status.success());
    assert!(String::from_utf8_lossy(&help.stdout).contains("--config"));
    for args in [&["--unknown"][..], &["--config"]] {
        let output = Command::new(binary).args(args).output().unwrap();
        assert_eq!(output.status.code(), Some(2));
    }
}

#[test]
fn explicit_invalid_config_is_rejected_before_window_creation() {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../logs")
        .join(format!("test_quad_cli_{}_{stamp}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let config = directory.join("quad.toml");
    std::fs::write(&config, "[control.allocation]\nairmode = 255\n").unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_quad"))
        .arg("--config")
        .arg(config)
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("invalid PX4 control parameters"));
    std::fs::remove_dir_all(directory).unwrap();
}
