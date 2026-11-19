use firefly_mission::{Plan, Request, call_at};
use nix::{
    sys::signal::{Signal, kill},
    unistd::Pid,
};
use serde_json::{Value, json};
use std::{
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
}

impl Stack {
    fn launch(&mut self, mut cmd: Command) -> io::Result<&mut Child> {
        self.children.push(cmd.spawn()?);
        Ok(self.children.last_mut().expect("just pushed"))
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
        mut sample: impl FnMut() -> io::Result<bool>,
    ) -> io::Result<()> {
        let start = Instant::now();
        loop {
            self.healthy()?;
            if sample()? {
                return Ok(());
            }
            if start.elapsed() >= limit {
                return Err(io::Error::other("mission client deadline exceeded"));
            }
            thread::sleep(Duration::from_millis(200));
        }
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
        errors
    }
}
impl Drop for Stack {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn record(stack: &mut Stack, release: &Path) -> Result<PathBuf, Box<dyn std::error::Error>> {
    let mut recorder = program(release.join("mimir"), &[]);
    recorder.stdout(Stdio::piped());
    let out = stack
        .launch(recorder)?
        .stdout
        .take()
        .ok_or("missing recorder pipe")?;
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        for line in BufReader::new(out).lines().map_while(Result::ok) {
            if let Some(path) = line.strip_prefix("recording: ") {
                let _ = tx.send(PathBuf::from(path));
            }
            eprintln!("{line}");
        }
    });
    Ok(rx.recv_timeout(Duration::from_secs(15))?)
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
        "from firefly_sim.main import main; main(['--no-trace','--config','configs/{scene}.toml'], publish_plant_state=False, seed={seed})"
    );
    stack.launch(program(python, &["-c", &sim]))?;
    stack.launch(program(release.join("render"), &[]))?;
    if hil {
        stack.launch(program(release.join("hil"), &[]))?;
    } else {
        stack.launch(program(
            release.join("firefly-rootslam"),
            &["--no-loop-closures", "--no-mapping"],
        ))?;
        stack.launch(program(release.join("fc"), &[]))?;
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
) -> io::Result<()> {
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
            match receipt["phase"].as_str() {
                Some("complete") => Ok(true),
                Some("interrupted" | "needs_operator") => {
                    Err(io::Error::other(receipt.to_string()))
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

fn claim_environment(root: &Path) -> Result<fs::File, Box<dyn std::error::Error>> {
    fs::create_dir_all(root.join("logs"))?;
    let lock = fs::File::options()
        .write(true)
        .create(true)
        .truncate(false)
        .open(root.join("logs/run.lock"))?;
    lock.try_lock()?;
    let python = root.join(".venv/bin/python");
    if !program(&python, &["-c","from pathlib import Path; from firefly_workspace.process import require_idle; require_idle(Path.cwd())"]).status()?.success() {
        return Err("could not acquire idle simulation environment".into())
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

pub fn run(hil: bool, args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let root = super::target::workspace_root();
    std::env::set_current_dir(&root)?;
    let mut args = args.iter();
    let mut plans = Vec::new();
    let mut seed = 42u64;
    while let Some(arg) = args.next() {
        if arg == "--seed" {
            seed = args.next().ok_or("missing seed")?.parse()?;
            continue;
        }
        let plan: Plan = toml::from_str(&fs::read_to_string(arg)?)?;
        plan.validate()?;
        plans.push(plan);
    }
    if plans.is_empty() {
        return Err("provide one or more mission plan files".into());
    }
    let release = root.join("target/release");
    let _lock = claim_environment(&root)?;
    let python = root.join(".venv/bin/python");
    if hil
        && !program(release.join("hil"), &["--check"])
            .status()?
            .success()
    {
        return Err("USB HIL is unavailable".into());
    }
    let prefix = tempfile::Builder::new()
        .prefix("sil-")
        .tempdir_in(root.join("target"))?;
    let socket = if hil {
        firefly_mission::runtime_dir().join("hil-mission.sock")
    } else {
        install_daemon(&root, prefix.path())?
    };
    let stopped = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, stopped.clone())?;
    }
    let mut stack = Stack {
        children: Vec::new(),
        stopped,
    };
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
    let nonce = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
    let mut expected = Vec::new();
    let mission = fly(&mut stack, &socket, plans, nonce, &mut expected);
    let cleanup = stack.shutdown();
    let score = Command::new(&python)
        .args(["scripts/verify_flight.py"])
        .arg(&path)
        .args(["--transport", if hil { "hil" } else { "sil" }])
        .output()?;
    let report: Value = serde_json::from_slice(&score.stdout).map_err(|e| {
        io::Error::other(format!("{e}: {}", String::from_utf8_lossy(&score.stderr)))
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
        return Err("mission or recording acceptance failed".into());
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
        let mut stack = Stack {
            children: Vec::new(),
            stopped: Arc::default(),
        };
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
