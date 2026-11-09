use std::process::Command;

#[test]
fn help_and_usage_errors_exit_before_resources() {
    let binary = env!("CARGO_BIN_EXE_render");
    let help = Command::new(binary).arg("--help").output().unwrap();
    assert!(help.status.success());
    assert!(String::from_utf8_lossy(&help.stdout).contains("--config"));
    for args in [
        &["--unknown"][..],
        &["--config"],
        &["--export-asset-contract"],
    ] {
        let output = Command::new(binary).args(args).output().unwrap();
        assert_eq!(output.status.code(), Some(2));
    }
}

#[test]
fn explicit_config_controls_export_without_opening_gpu_or_ipc() {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../logs")
        .join(format!("test_render_cli_{}_{stamp}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let config = directory.join("render.toml");
    let contract = directory.join("contract.toml");
    std::fs::write(&config, "[view]\nsensor_ev100 = 13.5\n").unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_render"))
        .arg("--config")
        .arg(&config)
        .arg("--export-asset-contract")
        .arg(&contract)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let value: toml::Table = toml::from_str(&std::fs::read_to_string(contract).unwrap()).unwrap();
    assert!((value["sensor_ev100"].as_float().unwrap() - 13.5).abs() < f64::EPSILON);
    std::fs::remove_dir_all(directory).unwrap();
}
