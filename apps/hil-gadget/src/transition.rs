//! Serialize complete USB mode transitions, including their final state markers.

use std::{fs, path::Path};

pub fn lock(directory: &Path) -> Result<fs::File, String> {
    fs::create_dir_all(directory)
        .map_err(|error| format!("create transition directory: {error}"))?;
    let path = directory.join("transition.lock");
    let file = fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(&path)
        .map_err(|error| format!("open {}: {error}", path.display()))?;
    file.lock()
        .map_err(|error| format!("lock {}: {error}", path.display()))?;
    Ok(file)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{sync::mpsc, thread, time::Duration};

    #[test]
    fn new_session_waits_for_previous_recovery_marker() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../logs")
            .join(format!("test_hil_transition_{}", std::process::id()));
        let recovery = lock(&directory).unwrap();
        let (attempt_tx, attempt_rx) = mpsc::channel();
        let (started_tx, started_rx) = mpsc::channel();
        let next_directory = directory.clone();
        let next = thread::spawn(move || {
            attempt_tx.send(()).unwrap();
            let _transition = lock(&next_directory).unwrap();
            let restored = next_directory.join("restored");
            if restored.exists() {
                fs::remove_file(restored).unwrap();
            }
            fs::write(next_directory.join("up"), "new session").unwrap();
            started_tx.send(()).unwrap();
        });
        attempt_rx.recv().unwrap();
        assert!(matches!(
            started_rx.recv_timeout(Duration::from_millis(50)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ));
        fs::write(directory.join("restored"), "previous session").unwrap();
        drop(recovery);
        started_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        next.join().unwrap();
        assert!(directory.join("up").is_file());
        assert!(!directory.join("restored").exists());
        fs::remove_dir_all(directory).unwrap();
    }
}
