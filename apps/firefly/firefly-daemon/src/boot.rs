use std::io;

#[cfg(target_os = "linux")]
pub fn run_id() -> io::Result<String> {
    const SOURCE: &str = "/proc/sys/kernel/random/boot_id";
    std::fs::read_to_string(SOURCE)
        .map(|s| s.trim().to_owned())
        .map_err(|e| io::Error::new(e.kind(), format!("{SOURCE}: {e}")))
}

/// Native macOS SIL uses the developer machine's current boot session.
#[cfg(target_os = "macos")]
pub fn run_id() -> io::Result<String> {
    use sysctl::{Ctl, Sysctl};
    const SOURCE: &str = "kern.bootsessionuuid";
    Ctl::new(SOURCE)
        .and_then(|ctl| ctl.value_string())
        .map_err(|e| io::Error::other(format!("{SOURCE}: {e}")))
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
pub fn run_id() -> io::Result<String> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "Firefly boot identity requires Linux or macOS SIL",
    ))
}

#[cfg(all(test, any(target_os = "linux", target_os = "macos")))]
mod tests {
    #[test]
    fn current_boot_yields_a_stable_directory_component() {
        let id = super::run_id().unwrap();
        assert!(firefly_mission::valid_id(&id));
        assert_eq!(super::run_id().unwrap(), id);
    }
}
