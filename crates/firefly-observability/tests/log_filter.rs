use std::process::Command;

const TARGETS: [&str; 4] = [
    "fc::diagnostics",
    "vio-diagnostics",
    "solver-profile",
    "basalt-profile",
];

fn check_filter(test_name: &str, filter: &str, debug_targets: &[&str]) {
    // Each filter needs a process with its own global logger.
    if std::env::var("RUST_LOG").as_deref() != Ok(filter) {
        let status = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", test_name])
            .env("RUST_LOG", filter)
            .status()
            .unwrap();
        assert!(status.success(), "RUST_LOG={filter}: {status}");
        return;
    }

    firefly_observability::init();
    for target in TARGETS {
        assert_eq!(
            log::log_enabled!(target: target, log::Level::Debug),
            debug_targets.contains(&target),
            "{target}"
        );
        assert!(!log::log_enabled!(target: target, log::Level::Trace));
        assert!(log::log_enabled!(target: target, log::Level::Error));
    }
}

#[test]
fn warn_filter() {
    check_filter("warn_filter", "warn", &[]);
}

#[test]
fn vio_filter() {
    check_filter(
        "vio_filter",
        "warn,vio-diagnostics=debug",
        &["vio-diagnostics"],
    );
}

#[test]
fn debug_filter() {
    check_filter("debug_filter", "debug", &TARGETS);
}
