use std::process::Command;

#[test]
fn diagnostic_gates_follow_rust_log() {
    if let Ok(mode) = std::env::var("FIREFLY_LOG_FILTER_PROBE") {
        firefly_observability::init();
        for target in [
            "fc::diagnostics",
            "vio-diagnostics",
            "solver-profile",
            "basalt-profile",
        ] {
            let expected = mode == "debug" || (mode == "vio" && target == "vio-diagnostics");
            assert_eq!(
                log::log_enabled!(target: target, log::Level::Debug),
                expected,
                "{target}"
            );
            assert!(!log::log_enabled!(target: target, log::Level::Trace));
            assert!(log::log_enabled!(target: target, log::Level::Error));
        }
        return;
    }

    // Logger installation and environment selection belong to a fresh process.
    for (mode, filter) in [
        ("warn", "warn"),
        ("vio", "warn,vio-diagnostics=debug"),
        ("debug", "debug"),
    ] {
        let output = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "diagnostic_gates_follow_rust_log", "--nocapture"])
            .env("FIREFLY_LOG_FILTER_PROBE", mode)
            .env("RUST_LOG", filter)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "RUST_LOG={filter}: {}{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}
