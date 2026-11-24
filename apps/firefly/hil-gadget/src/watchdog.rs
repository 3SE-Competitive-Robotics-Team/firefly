use crate::transition;
use firefly_error::{Error, ErrorKind, IoResultExt, Result};
use std::fs;
use std::io::{self, Write};
use std::path::Path;
use std::time::{Duration, Instant, SystemTime};

const HIL_TIMEOUT: Duration = Duration::from_secs(60);
pub const BIND_TIMEOUT: Duration = Duration::from_secs(3);

pub struct Watchdog<'a> {
    pub gadget: &'a Path,
    pub adb_endpoints: &'a Path,
    pub controllers: &'a Path,
    pub state: &'a Path,
    pub pending: &'a Path,
}

enum Function {
    Adb,
    Hil,
}

#[derive(PartialEq)]
enum HilActivity {
    Switching(SystemTime),
    Serving {
        up: Option<SystemTime>,
        host_seen: Option<SystemTime>,
    },
}

#[derive(Default)]
pub struct HilTimer(Option<(HilActivity, Instant)>);

impl HilTimer {
    fn expired(&mut self, activity: HilActivity, now: Instant) -> bool {
        if let Some((last, since)) = &self.0
            && *last == activity
        {
            return now.duration_since(*since) >= HIL_TIMEOUT;
        }
        self.0 = Some((activity, now));
        false
    }
}

impl Watchdog<'_> {
    pub fn poll(
        &self,
        timer: &mut HilTimer,
        now: Instant,
        restore_adb: impl FnOnce() -> Result<()>,
    ) -> Result<()> {
        let Some(_lock) = transition::try_lock(self.state, "transition.lock")? else {
            return Ok(());
        };
        let activity = if let Some(since) = modified(self.pending)? {
            HilActivity::Switching(since)
        } else {
            match self.configured_function() {
                Some(Function::Hil) => HilActivity::Serving {
                    up: modified(&self.state.join("up"))?,
                    host_seen: modified(&self.state.join("host_seen"))?,
                },
                Some(Function::Adb) => {
                    timer.0 = None;
                    return self.rebind_adb();
                }
                None => {
                    timer.0 = None;
                    return Ok(());
                }
            }
        };
        if timer.expired(activity, now) {
            restore_adb()?;
        }
        Ok(())
    }

    fn configured_function(&self) -> Option<Function> {
        let configs = self.gadget.join("configs");
        let mut entries = fs::read_dir(configs).ok()?;
        let config = entries.next()?.ok()?;
        if entries.next().is_some()
            || config.file_name() != "c.1"
            || !config.file_type().ok()?.is_dir()
        {
            return None;
        }
        let mut function = None;
        for entry in fs::read_dir(config.path()).ok()? {
            let entry = entry.ok()?;
            let kind = entry.file_type().ok()?;
            let name = entry.file_name().into_string().ok()?;
            match name.as_str() {
                "MaxPower" | "bmAttributes" if kind.is_file() => {}
                "strings" if kind.is_dir() => {}
                "ffs.adb" | "ffs.hil" if kind.is_symlink() && function.is_none() => {
                    let expected = self.gadget.join("functions").join(&name);
                    if !expected.is_dir()
                        || entry.path().canonicalize().ok()? != expected.canonicalize().ok()?
                    {
                        return None;
                    }
                    function = Some(if name == "ffs.adb" {
                        Function::Adb
                    } else {
                        Function::Hil
                    });
                }
                _ => return None,
            }
        }
        function
    }

    fn rebind_adb(&self) -> Result<()> {
        let binding = self.gadget.join("UDC");
        if !fs::read(&binding).is_ok_and(|value| value.is_empty() || value == b"\n")
            || !["ep1", "ep2"]
                .iter()
                .all(|ep| self.adb_endpoints.join(ep).is_file())
        {
            return Ok(());
        }
        let Ok(name) = controller_name(self.controllers) else {
            return Ok(());
        };
        let state = self.controllers.join(&name).join("state");
        if fs::read_to_string(state).is_ok_and(|state| state.trim_end() == "not attached") {
            bind(&binding, &name)?;
            eprintln!("watchdog：ADB 已重新绑定到 {name}");
        }
        Ok(())
    }
}

fn modified(path: &Path) -> Result<Option<SystemTime>> {
    match fs::metadata(path).and_then(|meta| meta.modified()) {
        Ok(time) => Ok(Some(time)),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error).at_path("read modification time", path),
    }
}

pub fn controller_name(controllers: &Path) -> Result<String> {
    let mut names = fs::read_dir(controllers).at_path("list controllers", controllers)?;
    if let Some(entry) = names
        .next()
        .transpose()
        .at_path("read controller", controllers)?
        && names.next().is_none()
        && let Ok(name) = entry.file_name().into_string()
    {
        return Ok(name);
    }
    Err(Error::new(ErrorKind::Unavailable, "期望唯一 USB 控制器"))
}

pub fn bind(path: &Path, name: &str) -> Result<()> {
    let mut udc = fs::OpenOptions::new()
        .write(true)
        .truncate(true)
        .open(path)
        .at_path("open UDC", path)?;
    udc.write_all(format!("{name}\n").as_bytes())
        .at_path("bind UDC", path)?;
    let deadline = Instant::now() + BIND_TIMEOUT;
    loop {
        if fs::read_to_string(path).is_ok_and(|value| value.trim_end() == name) {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(Error::new(
                ErrorKind::Timeout,
                format!("UDC {name} 绑定后未生效"),
            ));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::symlink;

    fn fixture(name: &str, run: impl FnOnce(&Watchdog<'_>)) {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../logs")
            .join(format!("test_watchdog_{name}_{}", std::process::id()));
        let gadget = root.join("g1");
        let adb_endpoints = root.join("adb");
        let controllers = root.join("udc");
        let state = root.join("hil");
        let pending = root.join("pending");
        for dir in [
            gadget.join("configs/c.1/strings"),
            gadget.join("functions/ffs.adb"),
            gadget.join("functions/ffs.hil"),
            controllers.join("test-udc"),
            adb_endpoints.clone(),
            state.clone(),
        ] {
            fs::create_dir_all(dir).unwrap();
        }
        for ep in ["ep0", "ep1", "ep2"] {
            fs::write(adb_endpoints.join(ep), "").unwrap();
        }
        fs::write(controllers.join("test-udc/state"), "not attached\n").unwrap();
        fs::write(gadget.join("UDC"), "\n").unwrap();
        link(&gadget, "ffs.adb");
        run(&Watchdog {
            gadget: &gadget,
            controllers: &controllers,
            adb_endpoints: &adb_endpoints,
            state: &state,
            pending: &pending,
        });
        fs::remove_dir_all(root).unwrap();
    }

    fn link(gadget: &Path, name: &str) {
        symlink(
            format!("../../functions/{name}"),
            gadget.join("configs/c.1").join(name),
        )
        .unwrap();
    }

    fn tick(watchdog: &Watchdog<'_>) {
        watchdog
            .poll(&mut HilTimer::default(), Instant::now(), || {
                panic!("unexpected HIL recovery")
            })
            .unwrap();
    }

    #[test]
    fn only_unbound_ready_adb_is_repaired_under_the_transition_lock() {
        fixture("rebind", |w| {
            let udc = w.gadget.join("UDC");
            for stale in ["up", "restored", "host_seen"] {
                fs::write(w.state.join(stale), "previous boot").unwrap();
            }
            let lock = transition::lock(w.state).unwrap();
            tick(w);
            assert_eq!(fs::read(&udc).unwrap(), b"\n");
            drop(lock);
            tick(w);
            assert_eq!(fs::read(&udc).unwrap(), b"test-udc\n");
            let bound = modified(&udc).unwrap();
            tick(w);
            assert_eq!(modified(&udc).unwrap(), bound);
        });
    }

    #[test]
    fn incomplete_mixed_and_ambiguous_adb_are_left_alone() {
        fixture("unknown", |w| {
            let udc = w.gadget.join("UDC");
            let untouched = || {
                tick(w);
                assert_eq!(fs::read(&udc).unwrap(), b"\n");
            };
            let state = w.controllers.join("test-udc/state");
            for usb_state in ["configured", "suspended", "unknown", ""] {
                fs::write(&state, usb_state).unwrap();
                untouched();
            }
            fs::remove_file(&state).unwrap();
            untouched();
            fs::write(&state, "not attached").unwrap();
            fs::create_dir(w.controllers.join("second")).unwrap();
            untouched();
            fs::remove_dir(w.controllers.join("second")).unwrap();
            for ep in ["ep1", "ep2"] {
                fs::remove_file(w.adb_endpoints.join(ep)).unwrap();
                untouched();
                fs::write(w.adb_endpoints.join(ep), "").unwrap();
            }
            let configs = w.gadget.join("configs");
            fs::create_dir(configs.join("c.2")).unwrap();
            untouched();
            fs::remove_dir(configs.join("c.2")).unwrap();
            link(w.gadget, "ffs.hil");
            untouched();
            fs::remove_file(configs.join("c.1/ffs.hil")).unwrap();
            let adb = configs.join("c.1/ffs.adb");
            fs::remove_file(&adb).unwrap();
            untouched();
            symlink("../../functions/ffs.hil", &adb).unwrap();
            untouched();
            fs::remove_dir(w.gadget.join("functions/ffs.hil")).unwrap();
            untouched();
            fs::remove_file(&adb).unwrap();
            link(w.gadget, "ffs.adb");
            for value in [b"other-controller\n".as_slice(), b" \n", b"\xff"] {
                fs::write(&udc, value).unwrap();
                tick(w);
                assert_eq!(fs::read(&udc).unwrap(), value);
            }
            fs::remove_file(&udc).unwrap();
            tick(w);
            assert!(!udc.exists());
        });
    }

    #[test]
    fn each_hil_session_and_each_heartbeat_get_a_full_timeout() {
        fixture("hil", |w| {
            fs::remove_file(w.gadget.join("configs/c.1/ffs.adb")).unwrap();
            link(w.gadget, "ffs.hil");
            let mut timer = HilTimer::default();
            let start = Instant::now();
            let mut poll = |seconds, expect_recovery| {
                let mut recovered = false;
                w.poll(&mut timer, start + Duration::from_secs(seconds), || {
                    assert!(transition::try_lock(w.state, "transition.lock")?.is_none());
                    recovered = true;
                    Ok(())
                })
                .unwrap();
                assert_eq!(recovered, expect_recovery, "t={seconds}");
            };
            poll(0, false);
            poll(59, false);
            fs::write(w.state.join("host_seen"), "").unwrap();
            poll(60, false);
            poll(119, false);
            poll(120, true);
            fs::write(w.state.join("up"), "new session").unwrap();
            poll(121, false);
            poll(180, false);
            poll(181, true);
            fs::remove_file(w.gadget.join("configs/c.1/ffs.hil")).unwrap();
            link(w.gadget, "ffs.adb");
            poll(241, false);
            assert_eq!(fs::read(w.gadget.join("UDC")).unwrap(), b"test-udc\n");
        });
    }

    #[test]
    fn only_an_explicit_interrupted_transition_recovers_an_incomplete_gadget() {
        fixture("pending", |w| {
            fs::remove_dir_all(w.gadget.join("configs")).unwrap();
            tick(w);
            fs::write(w.pending, "").unwrap();
            let start = Instant::now();
            let mut timer = HilTimer::default();
            w.poll(&mut timer, start, || panic!("missing startup grace"))
                .unwrap();
            let mut restored = false;
            w.poll(&mut timer, start + HIL_TIMEOUT, || {
                restored = true;
                Ok(())
            })
            .unwrap();
            assert!(restored);
        });
    }

    #[test]
    fn successful_write_requires_matching_readback() {
        assert_eq!(
            bind(Path::new("/dev/null"), "test-udc").unwrap_err().kind(),
            ErrorKind::Timeout
        );
    }
}
