use firefly_error::{IoResultExt, Result};
use std::{fs, path::Path};

fn open(directory: &Path, name: &str) -> Result<fs::File> {
    fs::create_dir_all(directory).at_path("create transition directory", directory)?;
    let path = directory.join(name);
    fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(&path)
        .at_path("open", &path)
}

pub fn lock(directory: &Path) -> Result<fs::File> {
    let file = open(directory, "transition.lock")?;
    file.lock().at_path("lock transition", directory)?;
    Ok(file)
}

pub fn try_lock(directory: &Path, name: &str) -> Result<Option<fs::File>> {
    let file = open(directory, name)?;
    match file.try_lock() {
        Ok(()) => Ok(Some(file)),
        Err(fs::TryLockError::WouldBlock) => Ok(None),
        Err(fs::TryLockError::Error(error)) => Err(error).at_path("lock", &directory.join(name)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{sync::mpsc, thread, time::Duration};

    #[test]
    fn new_session_waits_for_previous_recovery_to_finish() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../logs")
            .join(format!("test_hil_transition_{}", std::process::id()));
        let recovery = lock(&directory).unwrap();
        fs::write(directory.join("up"), "previous session").unwrap();
        let (attempt_tx, attempt_rx) = mpsc::channel();
        let (started_tx, started_rx) = mpsc::channel();
        let next_directory = directory.clone();
        let next = thread::spawn(move || {
            attempt_tx.send(()).unwrap();
            let _transition = lock(&next_directory).unwrap();
            assert!(!next_directory.join("up").exists());
            fs::write(next_directory.join("up"), "new session").unwrap();
            started_tx.send(()).unwrap();
        });
        attempt_rx.recv().unwrap();
        assert!(matches!(
            started_rx.recv_timeout(Duration::from_millis(50)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ));
        fs::remove_file(directory.join("up")).unwrap();
        drop(recovery);
        started_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        next.join().unwrap();
        assert!(directory.join("up").is_file());
        fs::remove_dir_all(directory).unwrap();
    }
}
