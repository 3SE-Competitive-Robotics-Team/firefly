use firefly_mission::{Request, call_at, runtime_dir_for};
use std::{
    fs,
    os::unix::fs::symlink,
    path::Path,
    process::{Child, Command},
    thread,
    time::{Duration, Instant},
};
use xtask::processes::active;

struct Daemon(Child);

impl Drop for Daemon {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn only_the_owning_checkout_reports_a_live_daemon() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap();
    let owner = tempfile::tempdir_in(root.join("target")).unwrap();
    let other = tempfile::tempdir_in(root.join("target")).unwrap();
    let prefix = owner.path().join("target");
    let bin = prefix.join("bin");
    fs::create_dir_all(&bin).unwrap();
    symlink(root.join("configs"), prefix.join("configs")).unwrap();
    let exe = bin.join("firefly-daemon");
    fs::copy(env!("CARGO_BIN_EXE_firefly-daemon"), &exe).unwrap();

    let mut daemon = Daemon(Command::new(exe).current_dir(other.path()).spawn().unwrap());
    let pid = daemon.0.id();
    let socket = runtime_dir_for(&prefix).join("mission.sock");
    let deadline = Instant::now() + Duration::from_secs(10);
    while call_at(&socket, &Request::Status { id: None }).is_err() {
        assert_eq!(
            daemon.0.try_wait().unwrap(),
            None,
            "daemon exited during startup"
        );
        assert!(
            Instant::now() < deadline,
            "daemon failed to serve its status socket"
        );
        thread::sleep(Duration::from_millis(20));
    }
    assert!(active(owner.path()).unwrap().contains(&pid));
    assert!(!active(other.path()).unwrap().contains(&pid));
    drop(daemon);
    assert!(!active(owner.path()).unwrap().contains(&pid));
}
