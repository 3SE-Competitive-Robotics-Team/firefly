/// Linux counters are sampled outside timed batches; unavailable files are explicit.
pub fn host_snapshot(label: &str) {
    for path in ["/proc/stat", "/proc/self/status"] {
        match std::fs::read_to_string(path) {
            Ok(text) => {
                let counters = text.lines().filter(|line| {
                    line.starts_with("cpu") || line.starts_with("Cpus_allowed_list")
                });
                for counter in counters {
                    log::info!("host {label} {path}: {counter}");
                }
            }
            Err(error) => log::info!("host {label} {path}: unavailable ({error})"),
        }
    }
    for path in [
        "/sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_count",
        "/sys/devices/system/cpu/cpu0/thermal_throttle/core_throttle_count",
        "/sys/devices/system/cpu/cpu1/thermal_throttle/core_throttle_count",
        "/sys/class/drm/card1/device/tile0/gt0/gtidle/idle_residency_ms",
    ] {
        match std::fs::read_to_string(path) {
            Ok(value) => log::info!("host {label} {path}: {}", value.trim()),
            Err(error) => log::info!("host {label} {path}: unavailable ({error})"),
        }
    }
    if let Ok(entries) = std::fs::read_dir("/sys/class/thermal") {
        for entry in entries.flatten() {
            let path = entry.path();
            if let (Ok(kind), Ok(value)) = (
                std::fs::read_to_string(path.join("type")),
                std::fs::read_to_string(path.join("temp")),
            ) {
                log::info!("host {label} temperature {}: {}", kind.trim(), value.trim());
            }
        }
    }
}
