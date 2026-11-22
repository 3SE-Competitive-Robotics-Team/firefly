use firefly_archive::{Run, output::Output};
use firefly_error::{Error, ErrorKind, IoResultExt};
use firefly_mission::{Phase, Plan, Request, call_at};
use nix::{
    sys::signal::{Signal, kill},
    unistd::Pid,
};
use serde_json::{Value, json};
use std::{
    collections::BTreeSet,
    fs,
    io::{self, BufRead, BufReader},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

struct Stack {
    children: Vec<Child>,
    stopped: Arc<AtomicBool>,
    run: Run,
    output: Vec<Output>,
    start: SystemTime,
    pids: BTreeSet<u32>,
}

impl Stack {
    fn new(root: &Path, source: &str) -> io::Result<Self> {
        let run = Run::create_at(root, source)?;
        eprintln!("archive: {}", run.path().display());
        Ok(Self {
            children: Vec::new(),
            stopped: Arc::default(),
            run,
            output: Vec::new(),
            start: SystemTime::now(),
            pids: BTreeSet::new(),
        })
    }
    fn spawn(&mut self, mut cmd: Command, ready: Option<mpsc::Sender<PathBuf>>) -> io::Result<()> {
        let source = Path::new(cmd.get_program())
            .file_name()
            .and_then(|s| s.to_str())
            .ok_or_else(|| io::Error::other("missing child process name"))?;
        let output = Output::new(self.run.clone(), source)?;
        let mut child = cmd
            .env("FIREFLY_RUN_DIR", self.run.path())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()?;
        self.pids.insert(child.id());
        if let Some(ready) = ready {
            let pid = child.id();
            let stdout = child.stdout.take().expect("piped recorder stdout");
            let tee = output.clone();
            thread::spawn(move || {
                for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                    tee.line(pid, "stdout", line.as_bytes());
                    if let Some(path) = line.strip_prefix("recording: ") {
                        let _ = ready.send(PathBuf::from(path));
                    }
                }
            });
        }
        output.attach(&mut child);
        self.children.push(child);
        self.output.push(output);
        Ok(())
    }
    fn launch(&mut self, cmd: Command) -> io::Result<()> {
        self.spawn(cmd, None)
    }
    fn healthy(&mut self) -> io::Result<()> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(io::Error::other("run interrupted"));
        }
        for child in &mut self.children {
            if let Some(status) = child.try_wait()? {
                return Err(io::Error::other(format!(
                    "child {} exited {status}",
                    child.id()
                )));
            }
        }
        Ok(())
    }
    fn wait(
        &mut self,
        limit: Duration,
        mut sample: impl FnMut() -> firefly_error::Result<bool>,
    ) -> firefly_error::Result<()> {
        let start = Instant::now();
        loop {
            self.healthy()?;
            if sample()? {
                return Ok(());
            }
            if start.elapsed() >= limit {
                return Err(Error::new(
                    ErrorKind::Timeout,
                    "mission client deadline exceeded",
                ));
            }
            thread::sleep(Duration::from_millis(200));
        }
    }
    fn board_snapshot(
        &mut self,
        socket: &Path,
    ) -> io::Result<Option<firefly_archive::transfer::Snapshot>> {
        let mut start = None;
        let mut reason = String::new();
        let ready = self.wait(Duration::from_secs(15), || {
            match super::board_logs::snapshot(socket) {
                Ok(snapshot) => start = Some(snapshot),
                Err(error) => reason = error.to_string(),
            }
            Ok(start.is_some())
        });
        if let Err(error) = ready {
            self.run.gap(
                "board",
                &format!("HIL start cursor unavailable: {reason} ({error})"),
            )?;
        }
        Ok(start)
    }
    fn shutdown(&mut self) -> Vec<String> {
        let mut errors = Vec::new();
        while let Some(mut child) = self.children.pop() {
            let _ = kill(Pid::from_raw(child.id() as i32), Signal::SIGINT);
            let start = Instant::now();
            loop {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        use std::os::unix::process::ExitStatusExt;
                        if !status.success() && status.signal() != Some(2) {
                            errors.push(format!("child {} exited {status}", child.id()));
                        }
                        break;
                    }
                    Ok(None) if start.elapsed() < Duration::from_secs(20) => {
                        thread::sleep(Duration::from_millis(50));
                    }
                    _ => {
                        errors.push(format!("forced shutdown of {}", child.id()));
                        let _ = child.kill();
                        let _ = child.wait();
                        break;
                    }
                }
            }
        }
        for output in self.output.drain(..) {
            if let Err(error) = output.finish() {
                errors.push(error.to_string());
            }
        }
        let pids = std::mem::take(&mut self.pids);
        if let Err(error) = firefly_archive::host::system(&self.run, self.start, &pids) {
            eprintln!("host system log: {error}");
            if let Err(error) = self.run.gap("system-host", &error.to_string()) {
                errors.push(error.to_string());
            }
        }
        errors
    }
}
impl Drop for Stack {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn record(stack: &mut Stack, release: &Path) -> firefly_error::Result<PathBuf> {
    let (tx, rx) = mpsc::channel();
    stack.spawn(program(release.join("mimir"), &[]), Some(tx))?;
    let path = rx.recv_timeout(Duration::from_secs(15)).map_err(|e| {
        let kind = match e {
            mpsc::RecvTimeoutError::Timeout => ErrorKind::Timeout,
            mpsc::RecvTimeoutError::Disconnected => ErrorKind::Unavailable,
        };
        Error::caused_by(kind, "wait for recording path", e)
    })?;
    if path != stack.run.path() {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "Mimir opened a different run archive",
        ));
    }
    Ok(path)
}

fn launch_stack(
    stack: &mut Stack,
    release: &Path,
    python: &Path,
    prefix: &Path,
    hil: bool,
    plans: &[Plan],
    seed: u64,
) -> io::Result<()> {
    let navigates = plans.iter().any(|plan| !plan.waypoints.is_empty());
    let scene = if navigates { "sim-mission" } else { "sim" };
    let sim = format!(
        "from firefly_sim.main import cli; raise SystemExit(cli(['--no-trace','--config','configs/{scene}.toml'], publish_plant_state=False, seed={seed}))"
    );
    stack.launch(program(python, &["-c", &sim]))?;
    stack.launch(program(release.join("render"), &[]))?;
    stack.launch(program(
        release.join("fc"),
        if hil {
            &["--config", "configs/fc-hil.toml"][..]
        } else {
            &[]
        },
    ))?;
    if hil {
        stack.launch(program(release.join("hil"), &[]))?;
    } else {
        let mut vio = program(
            release.join("firefly-rootslam"),
            &["--no-loop-closures", "--no-mapping"],
        );
        vio.env("RAYON_NUM_THREADS", "4");
        stack.launch(vio)?;
        stack.launch(program(release.join("pilot"), &["--simulation"]))?;
        stack.launch(program(release.join("depth"), &["--simulation"]))?;
        if plans.iter().any(|p| p.precision_land) {
            stack.launch(program(release.join("landing"), &["--simulation"]))?;
        }
        if navigates {
            stack.launch(program(release.join("traj"), &[]))?;
            let high = plans
                .iter()
                .flat_map(|p| &p.waypoints)
                .any(|w| w.offset[2] > 2.0);
            let config = if high {
                "configs/planner-cuboid-sil.toml"
            } else {
                "configs/planner.toml"
            };
            stack.launch(program(release.join("planner"), &["--config", config]))?;
        }
        stack.launch(program(prefix.join("bin/firefly-daemon"), &[]))?;
    }
    Ok(())
}

fn fly(
    stack: &mut Stack,
    socket: &Path,
    plans: Vec<Plan>,
    nonce: u128,
    expected: &mut Vec<String>,
) -> firefly_error::Result<()> {
    for (i, plan) in plans.into_iter().enumerate() {
        stack.wait(Duration::from_secs(90), || {
            Ok(
                call_at(socket, &Request::Status { id: None }).is_ok_and(|s| {
                    s["ready"] == true && (plan.waypoints.is_empty() || s["planner_ready"] == true)
                }),
            )
        })?;
        let id = format!("session-{nonce}-{}", i + 1);
        expected.push(id.clone());
        let limit = Duration::from_secs_f64(
            (plan.hover_seconds + 150.) * (plan.waypoints.len() + 3) as f64,
        );
        call_at(
            socket,
            &Request::Submit {
                id: id.clone(),
                plan,
            },
        )?;
        let result = stack.wait(limit, || {
            let receipt = call_at(
                socket,
                &Request::Status {
                    id: Some(id.clone()),
                },
            )?;
            let phase = serde_json::from_value::<Phase>(receipt["phase"].clone())
                .map_err(|e| Error::caused_by(ErrorKind::InvalidData, "decode mission phase", e))?;
            match phase {
                Phase::Complete => Ok(true),
                Phase::Interrupted | Phase::NeedsOperator => {
                    Err(Error::new(ErrorKind::Rejected, receipt.to_string()))
                }
                _ => Ok(false),
            }
        });
        if result.is_err() {
            let _ = call_at(socket, &Request::Cancel { id });
        }
        result?;
    }
    thread::sleep(Duration::from_secs(1));
    Ok(())
}

fn claim_environment(root: &Path) -> firefly_error::Result<fs::File> {
    fs::create_dir_all(root.join("logs"))?;
    let lock = fs::File::options()
        .write(true)
        .create(true)
        .truncate(false)
        .open(root.join("logs/run.lock"))?;
    lock.try_lock()
        .map_err(|e| Error::from(e).with_operation("claim simulation environment"))?;
    let occupied = xtask::processes::active(root)?;
    if !occupied.is_empty() {
        return Err(Error::new(
            ErrorKind::Unavailable,
            format!("repository flight processes already running: {occupied:?}"),
        ));
    }
    Ok(lock)
}

fn install_daemon(root: &Path, prefix: &Path) -> io::Result<PathBuf> {
    fs::create_dir(prefix.join("bin"))?;
    std::os::unix::fs::symlink(root.join("configs"), prefix.join("configs"))?;
    fs::copy(
        root.join("target/release/firefly-daemon"),
        prefix.join("bin/firefly-daemon"),
    )?;
    Ok(firefly_mission::runtime_dir_for(prefix).join("mission.sock"))
}

fn program(path: impl AsRef<Path>, args: &[&str]) -> Command {
    let mut cmd = Command::new(path.as_ref());
    cmd.current_dir(super::target::workspace_root()).args(args);
    cmd
}

fn read_plan(path: &Path) -> firefly_error::Result<Plan> {
    let plan: Plan = toml::from_str(&fs::read_to_string(path).at_path("read mission plan", path)?)
        .map_err(|e| {
            Error::caused_by(ErrorKind::InvalidArgument, "parse mission plan", e)
                .with_context("path", path.display())
        })?;
    plan.validate()
        .map_err(|e| e.with_context("path", path.display()))?;
    Ok(plan)
}

pub fn run(hil: bool, paths: &[PathBuf], seed: u64) -> firefly_error::Result<()> {
    let root = super::target::workspace_root();
    std::env::set_current_dir(&root)?;
    let mut plans = Vec::with_capacity(paths.len());
    for path in paths {
        let plan = read_plan(path)?;
        plans.push(plan);
    }
    let release = root.join("target/release");
    let verifier = release.join("firefly-verify");
    if !verifier.is_file() {
        return Err(Error::new(
            ErrorKind::NotFound,
            "missing firefly-verify; build the host tools with cargo xtask release",
        ));
    }
    let _lock = claim_environment(&root)?;
    let mut stack = Stack::new(&root, if hil { "hil" } else { "sil" })?;
    let python = root.join(".venv/bin/python");
    if hil
        && !program(release.join("hil"), &["--check"])
            .status()?
            .success()
    {
        return Err(Error::new(ErrorKind::Unavailable, "USB HIL is unavailable"));
    }
    let prefix = tempfile::Builder::new()
        .prefix("sil-")
        .tempdir_in(root.join("target"))?;
    let socket = if hil {
        firefly_mission::runtime_dir().join("hil-mission.sock")
    } else {
        install_daemon(&root, prefix.path())?
    };
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, stack.stopped.clone())?;
    }
    let path = record(&mut stack, &release)?;
    launch_stack(
        &mut stack,
        &release,
        &python,
        prefix.path(),
        hil,
        &plans,
        seed,
    )?;
    let board_start = if hil {
        stack.board_snapshot(&socket)?
    } else {
        None
    };
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| {
            Error::caused_by(
                ErrorKind::InvalidData,
                "system clock precedes Unix epoch",
                e,
            )
        })?
        .as_nanos();
    let mut expected = Vec::new();
    let mission = fly(&mut stack, &socket, plans, nonce, &mut expected);
    if let Some(start) = board_start
        && let Err(error) = super::board_logs::collect(&stack.run, &socket, &start)
    {
        stack
            .run
            .gap("board", &format!("HIL log download interrupted: {error}"))?;
    }
    let cleanup = stack.shutdown();
    let score = Command::new(verifier)
        .arg(&path)
        .args(["--transport", if hil { "hil" } else { "sil" }])
        .output()?;
    let report: Value = serde_json::from_slice(&score.stdout).map_err(|e| {
        Error::caused_by(ErrorKind::InvalidData, "decode verifier report", e)
            .with_context("stderr", String::from_utf8_lossy(&score.stderr))
    })?;
    let ids: Vec<_> = report["metrics"]["flights"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|f| f["mission_id"].as_str())
        .collect();
    println!(
        "{}",
        json!({"commands_completed":mission.is_ok(),"cleanup_errors":cleanup,"recording":path,"verification":report})
    );
    mission?;
    if !cleanup.is_empty() || !score.status.success() || ids != expected {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "mission or recording acceptance failed",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires release SIL stack, GPU and scene assets"]
    fn daemon_restart_preserves_receipts_and_recovers_airborne_mission() {
        let root = super::super::target::workspace_root();
        let _lock = claim_environment(&root).unwrap();
        let release = root.join("target/release");
        let prefix = tempfile::Builder::new()
            .prefix("sil-")
            .tempdir_in(root.join("target"))
            .unwrap();
        let socket = install_daemon(&root, prefix.path()).unwrap();
        let binary = prefix.path().join("bin/firefly-daemon");
        let mut stack = Stack::new(&root, "sil-test").unwrap();
        let path = record(&mut stack, &release).unwrap();
        let status = || call_at(&socket, &Request::Status { id: None });
        stack.launch(program(&binary, &[])).unwrap();
        stack
            .wait(Duration::from_secs(10), || Ok(status().is_ok()))
            .unwrap();
        let cold = status().unwrap();
        assert_eq!(cold["ready"], false);
        let plan = Plan {
            altitude: 0.8,
            hover_seconds: 20.,
            precision_land: false,
            return_offset: [0.; 2],
            waypoints: vec![],
        };
        let submit = || {
            call_at(
                &socket,
                &Request::Submit {
                    id: "restart-test".into(),
                    plan: plan.clone(),
                },
            )
        };
        assert!(submit().is_err());
        assert!(!program(&binary, &[]).status().unwrap().success());
        let mut cold_child = stack.children.pop().unwrap();
        cold_child.kill().unwrap();
        cold_child.wait().unwrap();
        launch_stack(
            &mut stack,
            &release,
            &root.join(".venv/bin/python"),
            prefix.path(),
            false,
            std::slice::from_ref(&plan),
            42,
        )
        .unwrap();
        stack
            .wait(Duration::from_secs(90), || {
                Ok(status().is_ok_and(|s| s["ready"] == true))
            })
            .unwrap();
        let receipt = submit().unwrap();
        assert_eq!(receipt["run"], cold["run"]);
        assert_eq!(submit().unwrap()["session"], receipt["session"]);
        let query = || {
            call_at(
                &socket,
                &Request::Status {
                    id: Some("restart-test".into()),
                },
            )
        };
        stack
            .wait(Duration::from_secs(60), || Ok(query()?["phase"] == "hover"))
            .unwrap();
        let mut child = stack.children.pop().unwrap();
        child.kill().unwrap();
        child.wait().unwrap();
        stack.launch(program(&binary, &[])).unwrap();
        stack
            .wait(Duration::from_secs(60), || {
                Ok(query().is_ok_and(|s| {
                    matches!(s["phase"].as_str(), Some("interrupted" | "needs_operator"))
                }))
            })
            .unwrap();
        let end = query().unwrap();
        assert_eq!(end["session"], receipt["session"]);
        assert_eq!(end["reason"], "daemon restarted during mission");
        assert_eq!(end["flight_number"], 1);
        assert_eq!(submit().unwrap(), end);
        assert_eq!(
            end["phase"], "interrupted",
            "recovery requires operator: {end}"
        );
        assert_eq!(stack.shutdown(), Vec::<String>::new());
        eprintln!("recovery recording: {}", path.display());
    }
}
