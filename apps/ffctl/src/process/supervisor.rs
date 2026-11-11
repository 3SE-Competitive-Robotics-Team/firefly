//! 进程 supervisor：按清单顺序拉起、监控、重启，并通过 Unix socket 接受控制。
//!
//! 清单只在启动时读取；`enable`/`disable` 的改动由下一次 supervisor 启动生效。

use std::fs::{self, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::os::unix::process::CommandExt;
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nix::errno::Errno;
use nix::sys::signal::{Signal, kill};
use nix::unistd::Pid;
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;

use crate::process::SOCKET;
use crate::process::config::{Config, Process};
use crate::process::protocol::{ProcessState, ProcessStatus, Request, Response};

const LOG_DIR: &str = "logs/process";
const START_DELAY: Duration = Duration::from_millis(300);
const STOP_TIMEOUT: Duration = Duration::from_secs(5);
const RESTART_BASE: Duration = Duration::from_secs(1);
const RESTART_MAX: Duration = Duration::from_secs(30);
const STABLE_RESET: Duration = Duration::from_secs(60);
const TICK: Duration = Duration::from_millis(100);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(60);
const MAX_REQUEST: usize = 64 * 1024;

struct Managed {
    process: Process,
    lifecycle: Lifecycle,
    restarts: u32,
}

/// Running owns a child with restart permission; Stopping retains it until reaped.
/// Backoff owns only a retry deadline. Stopped owns neither resource.
enum Lifecycle {
    Stopped,
    Running { child: Child, started_at: Instant },
    Backoff { restart_at: Instant },
    Stopping { child: Child, started_at: Instant },
}

impl Lifecycle {
    fn child(&self) -> Option<&Child> {
        match self {
            Self::Running { child, .. } | Self::Stopping { child, .. } => Some(child),
            Self::Stopped | Self::Backoff { .. } => None,
        }
    }

    fn child_mut(&mut self) -> Option<&mut Child> {
        match self {
            Self::Running { child, .. } | Self::Stopping { child, .. } => Some(child),
            Self::Stopped | Self::Backoff { .. } => None,
        }
    }

    fn state(&self) -> ProcessState {
        match self {
            Self::Stopped => ProcessState::Stopped,
            Self::Running { .. } => ProcessState::Running,
            Self::Backoff { .. } => ProcessState::Backoff,
            Self::Stopping { .. } => ProcessState::Stopping,
        }
    }

    /// Revoke restart permission before signaling; repeated stops retain the child.
    fn request_stop(&mut self) {
        *self = match std::mem::replace(self, Self::Stopped) {
            Self::Running { child, started_at } | Self::Stopping { child, started_at } => {
                Self::Stopping { child, started_at }
            }
            Self::Stopped | Self::Backoff { .. } => Self::Stopped,
        };
    }
}

#[allow(clippy::too_many_lines)]
pub(crate) fn run(path: &Path) -> Result<(), String> {
    let config = Config::load(path)?;
    let shutdown = Arc::new(AtomicBool::new(false));
    for signal in [SIGTERM, SIGINT] {
        flag::register(signal, Arc::clone(&shutdown))
            .map_err(|error| format!("安装信号处理: {error}"))?;
    }

    let listener = bind_socket()?;
    let (sender, receiver) = mpsc::channel::<(Request, mpsc::SyncSender<Response>)>();
    spawn_server(listener, sender, Arc::clone(&shutdown));

    let mut managed: Vec<Managed> = config
        .processes
        .into_iter()
        .map(|process| Managed {
            process,
            lifecycle: Lifecycle::Stopped,
            restarts: 0,
        })
        .collect();

    if let Some(failed) = bring_up(&mut managed) {
        eprintln!("ffctl: 自启动在 {failed} 处失败，保留已启动进程，supervisor 继续运行");
    }

    while !shutdown.load(Ordering::Relaxed) {
        while let Ok((request, reply)) = receiver.try_recv() {
            let _ = reply.send(handle(request, &mut managed));
        }
        monitor(&mut managed);
        thread::sleep(TICK);
    }

    let stopped = stop(&mut managed, &[]);
    let _ = fs::remove_file(SOCKET);
    stopped.map(|_| ())
}

/// 按清单顺序拉起 `autostart = true` 的进程；遇到失败立即停止继续，返回失败的描述。
fn bring_up(managed: &mut [Managed]) -> Option<String> {
    for entry in managed {
        if !entry.process.autostart {
            continue;
        }
        let name = entry.process.name.clone();
        match spawn(&entry.process) {
            Ok(child) => {
                entry.lifecycle = Lifecycle::Running {
                    child,
                    started_at: Instant::now(),
                };
                entry.restarts = 0;
                thread::sleep(START_DELAY);
                let exited = entry
                    .lifecycle
                    .child_mut()
                    .and_then(|child| child.try_wait().ok())
                    .flatten();
                if let Some(status) = exited {
                    entry.lifecycle = Lifecycle::Stopped;
                    eprintln!("ffctl: {name} 启动后立即退出（{status}）");
                    return Some(format!("{name}（{status}）"));
                }
            }
            Err(error) => {
                entry.lifecycle = Lifecycle::Stopped;
                eprintln!("ffctl: {name} 启动失败：{error}");
                return Some(format!("{name}（{error}）"));
            }
        }
    }
    None
}

fn monitor(managed: &mut [Managed]) {
    let now = Instant::now();
    for entry in &mut *managed {
        let exited = match entry.lifecycle.child_mut() {
            Some(child) => match child.try_wait() {
                Ok(status) => status,
                Err(error) => {
                    eprintln!("ffctl: {} try_wait: {error}", entry.process.name);
                    None
                }
            },
            None => None,
        };

        if let Some(status) = exited {
            if matches!(entry.lifecycle, Lifecycle::Running { .. }) {
                entry.restarts = entry.restarts.saturating_add(1);
                let delay = backoff(entry.restarts);
                eprintln!(
                    "ffctl: {} 退出（{status}），{}s 后重启",
                    entry.process.name,
                    delay.as_secs()
                );
                entry.lifecycle = Lifecycle::Backoff {
                    restart_at: now + delay,
                };
            } else {
                entry.lifecycle = Lifecycle::Stopped;
                eprintln!("ffctl: {} 已停止（{status}）", entry.process.name);
            }
        } else if entry.restarts > 0
            && matches!(entry.lifecycle,
                Lifecycle::Running { started_at, .. } | Lifecycle::Stopping { started_at, .. }
                    if now.saturating_duration_since(started_at) >= STABLE_RESET)
        {
            entry.restarts = 0;
        }

        if matches!(entry.lifecycle, Lifecycle::Backoff { restart_at } if now >= restart_at) {
            match spawn(&entry.process) {
                Ok(child) => {
                    entry.lifecycle = Lifecycle::Running {
                        child,
                        started_at: now,
                    };
                }
                Err(error) => {
                    entry.restarts = entry.restarts.saturating_add(1);
                    let delay = backoff(entry.restarts);
                    eprintln!(
                        "ffctl: {} 重启失败（{error}），{}s 后重试",
                        entry.process.name,
                        delay.as_secs()
                    );
                    entry.lifecycle = Lifecycle::Backoff {
                        restart_at: now + delay,
                    };
                }
            }
        }
    }
}

fn handle(request: Request, managed: &mut [Managed]) -> Response {
    let result = match request {
        Request::Start { names } => start(managed, &names),
        Request::Stop { names } => stop(managed, &names),
        Request::Restart { names } => restart(managed, &names),
        Request::Status => Ok(String::new()),
    };
    match result {
        Ok(message) => Response {
            ok: true,
            message,
            processes: statuses(managed),
        },
        Err(message) => Response {
            ok: false,
            message,
            processes: statuses(managed),
        },
    }
}

fn start(managed: &mut [Managed], names: &[String]) -> Result<String, String> {
    let mut indices = resolve_indices(managed, names, Defaults::Autostart)?;
    indices.sort_unstable();
    let mut started = Vec::new();
    for index in indices {
        if managed[index].lifecycle.child().is_some() {
            continue;
        }
        match spawn(&managed[index].process) {
            Ok(child) => {
                managed[index].lifecycle = Lifecycle::Running {
                    child,
                    started_at: Instant::now(),
                };
                managed[index].restarts = 0;
                started.push(managed[index].process.name.clone());
            }
            Err(error) => return Err(format!("{} 启动失败：{error}", managed[index].process.name)),
        }
    }
    if started.is_empty() {
        Ok("没有需要启动的进程".to_owned())
    } else {
        Ok(format!("已启动：{}", started.join(", ")))
    }
}

fn stop(managed: &mut [Managed], names: &[String]) -> Result<String, String> {
    let mut indices = resolve_indices(managed, names, Defaults::All)?;
    indices.sort_unstable();
    let mut stopped = Vec::new();
    let mut errors = Vec::new();
    for index in indices.into_iter().rev() {
        let entry = &mut managed[index];
        let scheduled = matches!(entry.lifecycle, Lifecycle::Backoff { .. });
        entry.lifecycle.request_stop();
        if entry.lifecycle.child().is_none() {
            if scheduled {
                stopped.push(entry.process.name.clone());
            }
            continue;
        }
        match stop_child(entry, STOP_TIMEOUT) {
            Ok(()) => stopped.push(entry.process.name.clone()),
            Err(error) => errors.push(error),
        }
    }
    if errors.is_empty() {
        if stopped.is_empty() {
            Ok("没有运行中的进程".to_owned())
        } else {
            Ok(format!("已停止：{}", stopped.join(", ")))
        }
    } else {
        Err(errors.join("；"))
    }
}

fn restart(managed: &mut [Managed], names: &[String]) -> Result<String, String> {
    let stop_result = stop(managed, names);
    let start_result = start(managed, names);
    match (stop_result, start_result) {
        (Ok(stop), Ok(start)) => Ok(format!("{stop}；{start}")),
        (stop, start) => Err(format!(
            "{}；{}",
            stop.unwrap_or_else(|error| error),
            start.unwrap_or_else(|error| error)
        )),
    }
}

fn stop_child(entry: &mut Managed, timeout: Duration) -> Result<(), String> {
    let name = entry.process.name.clone();
    let Some(pid) = entry.lifecycle.child().map(Child::id) else {
        return Ok(());
    };
    let pid = i32::try_from(pid).map_err(|_| format!("{name} 的 pid 超出范围"))?;
    match kill(Pid::from_raw(pid), Signal::SIGINT) {
        Ok(()) | Err(Errno::ESRCH) => {}
        Err(error) => return Err(format!("{name} 发送 SIGINT 失败：{error}")),
    }

    let deadline = Instant::now() + timeout;
    loop {
        let exited = {
            let child = entry
                .lifecycle
                .child_mut()
                .ok_or_else(|| format!("{name} 状态不一致"))?;
            match child.try_wait() {
                Ok(status) => status,
                Err(error) => return Err(format!("{name} try_wait: {error}")),
            }
        };
        if let Some(status) = exited {
            entry.lifecycle = Lifecycle::Stopped;
            eprintln!("ffctl: {name} 已停止（{status}）");
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!("{name} 在 {}s 内未退出", timeout.as_secs()));
        }
        thread::sleep(Duration::from_millis(50));
    }
}

#[derive(Clone, Copy)]
enum Defaults {
    All,
    Autostart,
}

fn resolve_indices(
    managed: &[Managed],
    names: &[String],
    defaults: Defaults,
) -> Result<Vec<usize>, String> {
    if names.is_empty() {
        return Ok(managed
            .iter()
            .enumerate()
            .filter(|(_, entry)| match defaults {
                Defaults::All => true,
                Defaults::Autostart => entry.process.autostart,
            })
            .map(|(index, _)| index)
            .collect());
    }
    let mut indices = Vec::new();
    for name in names {
        let index = managed
            .iter()
            .position(|entry| &entry.process.name == name)
            .ok_or_else(|| format!("未知进程：{name}"))?;
        if !indices.contains(&index) {
            indices.push(index);
        }
    }
    Ok(indices)
}

fn statuses(managed: &[Managed]) -> Vec<ProcessStatus> {
    managed
        .iter()
        .map(|entry| ProcessStatus {
            name: entry.process.name.clone(),
            running: entry.lifecycle.child().is_some(),
            pid: entry.lifecycle.child().map(Child::id),
            restarts: entry.restarts,
            state: Some(entry.lifecycle.state()),
        })
        .collect()
}

fn spawn(process: &Process) -> Result<Child, String> {
    let program = process
        .command
        .first()
        .ok_or_else(|| "command 为空".to_owned())?;
    fs::create_dir_all(LOG_DIR).map_err(|error| format!("创建 {LOG_DIR}: {error}"))?;
    let log_path = Path::new(LOG_DIR).join(format!("{}.log", process.name));
    let mut log = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&log_path)
        .map_err(|error| format!("打开 {}: {error}", log_path.display()))?;
    writeln!(log, "=== {} 启动 {} ===", unix_seconds(), process.name)
        .map_err(|error| format!("写 {}: {error}", log_path.display()))?;
    log.sync_all()
        .map_err(|error| format!("fsync {}: {error}", log_path.display()))?;

    Command::new(program)
        .args(&process.command[1..])
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            log.try_clone()
                .map_err(|error| format!("复制日志句柄: {error}"))?,
        ))
        .stderr(Stdio::from(log))
        .process_group(0)
        .spawn()
        .map_err(|error| format!("启动 {}: {error}", process.name))
}

fn backoff(attempt: u32) -> Duration {
    let shift = attempt.saturating_sub(1).min(5);
    (RESTART_BASE * (1_u32 << shift)).min(RESTART_MAX)
}

fn unix_seconds() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |duration| duration.as_secs())
}

fn bind_socket() -> Result<UnixListener, String> {
    let path = Path::new(SOCKET);
    let directory = path
        .parent()
        .ok_or_else(|| format!("{SOCKET} 没有父目录"))?;
    fs::create_dir_all(directory)
        .map_err(|error| format!("创建 {}: {error}", directory.display()))?;
    fs::set_permissions(directory, fs::Permissions::from_mode(0o755))
        .map_err(|error| format!("chmod {}: {error}", directory.display()))?;

    if path.exists() {
        if UnixStream::connect(path).is_ok() {
            return Err("supervisor 已在运行".to_owned());
        }
        fs::remove_file(path).map_err(|error| format!("删除旧 socket: {error}"))?;
    }
    let listener = UnixListener::bind(path).map_err(|error| format!("绑定 {SOCKET}: {error}"))?;
    fs::set_permissions(path, fs::Permissions::from_mode(0o600))
        .map_err(|error| format!("chmod {SOCKET}: {error}"))?;
    Ok(listener)
}

fn spawn_server(
    listener: UnixListener,
    sender: mpsc::Sender<(Request, mpsc::SyncSender<Response>)>,
    shutdown: Arc<AtomicBool>,
) {
    if let Err(error) = listener.set_nonblocking(true) {
        eprintln!("ffctl: supervisor socket 设为非阻塞失败: {error}");
        return;
    }
    let _ = thread::spawn(move || {
        while !shutdown.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, _)) => {
                    let sender = sender.clone();
                    let _ = thread::spawn(move || serve(stream, &sender));
                }
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                    thread::sleep(TICK);
                }
                Err(error) => {
                    eprintln!("ffctl: supervisor accept: {error}");
                    break;
                }
            }
        }
    });
}

fn serve(mut stream: UnixStream, sender: &mpsc::Sender<(Request, mpsc::SyncSender<Response>)>) {
    let response = match read_request(&stream) {
        Ok(request) => {
            let (reply_sender, reply_receiver) = mpsc::sync_channel(1);
            if sender.send((request, reply_sender)).is_err() {
                error_response("supervisor 已停止")
            } else {
                reply_receiver
                    .recv_timeout(REQUEST_TIMEOUT)
                    .unwrap_or_else(|_| error_response("supervisor 响应超时"))
            }
        }
        Err(error) => error_response(&error),
    };
    if let Ok(text) = serde_json::to_string(&response) {
        let _ = stream.write_all(text.as_bytes());
        let _ = stream.write_all(b"\n");
        let _ = stream.flush();
    }
}

fn read_request(stream: &UnixStream) -> Result<Request, String> {
    let clone = stream
        .try_clone()
        .map_err(|error| format!("复制 socket: {error}"))?;
    let mut reader = BufReader::new(clone);
    let mut line = String::new();
    let read = reader
        .read_line(&mut line)
        .map_err(|error| format!("读取请求: {error}"))?;
    if read == 0 {
        return Err("空请求".to_owned());
    }
    if line.len() > MAX_REQUEST {
        return Err("请求过大".to_owned());
    }
    serde_json::from_str(&line).map_err(|error| format!("解析请求: {error}"))
}

fn error_response(message: &str) -> Response {
    Response {
        ok: false,
        message: message.to_owned(),
        processes: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// stdin keeps the child alive; closing it lets every test reap its own process.
    struct Fixture(Managed);

    impl Fixture {
        fn running() -> Self {
            let mut child = Command::new("/bin/sh")
                .args(["-c", "trap '' INT; printf 'ready\\n'; read -r line"])
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(Stdio::null())
                .spawn()
                .unwrap();
            let mut ready = String::new();
            BufReader::new(child.stdout.take().unwrap())
                .read_line(&mut ready)
                .unwrap();
            assert_eq!(ready, "ready\n");
            let mut entry = pending_restart("fixture");
            entry.lifecycle = Lifecycle::Running {
                child,
                started_at: Instant::now(),
            };
            Self(entry)
        }

        fn exit(&mut self) {
            if let Some(child) = self.0.lifecycle.child_mut() {
                drop(child.stdin.take());
                child.wait().unwrap();
            }
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            self.exit();
        }
    }

    fn pending_restart(name: &str) -> Managed {
        Managed {
            process: Process {
                name: name.to_owned(),
                command: vec!["pending-restart-fixture".to_owned()],
                autostart: true,
            },
            lifecycle: Lifecycle::Backoff {
                restart_at: Instant::now(),
            },
            restarts: 1,
        }
    }

    #[test]
    fn stop_cancels_a_selected_pending_restart() {
        let mut managed = [pending_restart("selected"), pending_restart("other")];

        let message = stop(&mut managed, &["selected".to_owned()]).unwrap();

        assert!(matches!(managed[0].lifecycle, Lifecycle::Stopped));
        assert!(message.contains("selected"));
        assert!(matches!(managed[1].lifecycle, Lifecycle::Backoff { .. }));
    }

    #[test]
    fn stop_without_names_cancels_pending_restarts() {
        let mut managed = [pending_restart("first"), pending_restart("second")];

        stop(&mut managed, &[]).unwrap();
        monitor(&mut managed);

        assert!(
            managed
                .iter()
                .all(|entry| matches!(entry.lifecycle, Lifecycle::Stopped))
        );
        assert!(managed.iter().all(|entry| entry.restarts == 1));
    }

    #[test]
    fn repeated_stop_is_idle_and_unknown_names_leave_backoff_intact() {
        let mut managed = [pending_restart("selected")];
        assert!(stop(&mut managed, &["unknown".to_owned()]).is_err());
        assert!(matches!(managed[0].lifecycle, Lifecycle::Backoff { .. }));
        stop(&mut managed, &[]).unwrap();
        assert_eq!(stop(&mut managed, &[]).unwrap(), "没有运行中的进程");
        monitor(&mut managed);
        assert_eq!(statuses(&managed)[0].state, Some(ProcessState::Stopped));
    }

    #[test]
    fn unexpected_exit_schedules_one_retry_and_stop_cancels_it() {
        let mut fixture = Fixture::running();
        fixture.exit();
        monitor(std::slice::from_mut(&mut fixture.0));
        assert_eq!(fixture.0.restarts, 2);
        assert!(
            matches!(fixture.0.lifecycle, Lifecycle::Backoff { restart_at }
            if restart_at > Instant::now())
        );
        let status = statuses(std::slice::from_ref(&fixture.0)).remove(0);
        assert_eq!(status.state, Some(ProcessState::Backoff));
        assert!(!status.running);
        assert!(status.pid.is_none());
        stop(std::slice::from_mut(&mut fixture.0), &[]).unwrap();
        monitor(std::slice::from_mut(&mut fixture.0));
        assert!(matches!(fixture.0.lifecycle, Lifecycle::Stopped));
        assert_eq!(fixture.0.restarts, 2);
    }

    #[test]
    fn stop_timeout_retains_child_and_prevents_restart_after_exit() {
        let mut fixture = Fixture::running();
        let pid = fixture.0.lifecycle.child().unwrap().id();
        fixture.0.lifecycle.request_stop();
        assert!(stop_child(&mut fixture.0, Duration::ZERO).is_err());
        fixture.0.lifecycle.request_stop();
        assert!(stop_child(&mut fixture.0, Duration::ZERO).is_err());
        assert_eq!(
            start(std::slice::from_mut(&mut fixture.0), &[]).unwrap(),
            "没有需要启动的进程"
        );
        let status = statuses(std::slice::from_ref(&fixture.0)).remove(0);
        assert_eq!(status.state, Some(ProcessState::Stopping));
        assert_eq!(status.pid, Some(pid));
        assert!(status.running);
        fixture.exit();
        monitor(std::slice::from_mut(&mut fixture.0));
        assert!(matches!(fixture.0.lifecycle, Lifecycle::Stopped));
        assert_eq!(fixture.0.restarts, 1);
    }

    #[test]
    fn stable_running_child_resets_restart_count() {
        let mut fixture = Fixture::running();
        if let Lifecycle::Running { started_at, .. } = &mut fixture.0.lifecycle {
            *started_at = Instant::now().checked_sub(STABLE_RESET).unwrap();
        }
        monitor(std::slice::from_mut(&mut fixture.0));
        assert_eq!(fixture.0.restarts, 0);
        assert!(matches!(fixture.0.lifecycle, Lifecycle::Running { .. }));
    }

    #[test]
    fn failed_manual_start_preserves_retry_and_failed_retry_advances_backoff() {
        let mut managed = [pending_restart("empty-command")];
        managed[0].process.command.clear();
        assert!(start(&mut managed, &[]).is_err());
        assert!(matches!(managed[0].lifecycle, Lifecycle::Backoff { .. }));
        assert_eq!(managed[0].restarts, 1);
        monitor(&mut managed);
        assert_eq!(managed[0].restarts, 2);
        assert!(
            matches!(managed[0].lifecycle, Lifecycle::Backoff { restart_at }
            if restart_at > Instant::now())
        );
        monitor(&mut managed);
        assert_eq!(managed[0].restarts, 2);
    }

    #[test]
    fn start_and_retry_reap_children_and_preserve_restart_policy() {
        let name = format!("ffctl-lifecycle-test-{}", std::process::id());
        let log_path = Path::new(LOG_DIR).join(format!("{name}.log"));
        assert!(!log_path.exists());
        let mut fixture = Fixture(pending_restart(&name));
        fixture.0.lifecycle = Lifecycle::Stopped;
        fixture.0.process.command = vec!["/bin/sh".into(), "-c".into(), "exit 0".into()];

        start(std::slice::from_mut(&mut fixture.0), &[]).unwrap();
        assert_eq!(fixture.0.restarts, 0);
        let pid = fixture.0.lifecycle.child().unwrap().id();
        start(std::slice::from_mut(&mut fixture.0), &[]).unwrap();
        assert_eq!(fixture.0.lifecycle.child().unwrap().id(), pid);
        fixture.exit();
        monitor(std::slice::from_mut(&mut fixture.0));
        assert_eq!(fixture.0.restarts, 1);
        assert!(matches!(fixture.0.lifecycle, Lifecycle::Backoff { .. }));

        fixture.0.lifecycle = Lifecycle::Backoff {
            restart_at: Instant::now(),
        };
        monitor(std::slice::from_mut(&mut fixture.0));
        assert!(matches!(fixture.0.lifecycle, Lifecycle::Running { .. }));
        assert_eq!(fixture.0.restarts, 1);
        fixture.exit();
        monitor(std::slice::from_mut(&mut fixture.0));
        assert_eq!(fixture.0.restarts, 2);
        start(std::slice::from_mut(&mut fixture.0), &[]).unwrap();
        assert_eq!(fixture.0.restarts, 0);
        fixture.exit();
        stop(std::slice::from_mut(&mut fixture.0), &[]).unwrap();
        assert!(matches!(fixture.0.lifecycle, Lifecycle::Stopped));

        assert!(bring_up(std::slice::from_mut(&mut fixture.0)).is_some());
        assert!(matches!(fixture.0.lifecycle, Lifecycle::Stopped));
        fs::remove_file(log_path).unwrap();
    }

    #[test]
    fn unix_socket_status_preserves_lifecycle_and_legacy_fields() {
        let (mut client, server) = UnixStream::pair().unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        let (sender, receiver) = mpsc::channel();
        let serving = thread::spawn(move || serve(server, &sender));
        client.write_all(b"{\"cmd\":\"status\"}\n").unwrap();
        let (request, reply) = receiver.recv_timeout(Duration::from_secs(5)).unwrap();
        reply
            .send(handle(request, &mut [pending_restart("fixture")]))
            .unwrap();
        let mut text = String::new();
        BufReader::new(client).read_line(&mut text).unwrap();
        serving.join().unwrap();
        let response: Response = serde_json::from_str(&text).unwrap();
        assert!(response.ok);
        assert_eq!(response.processes[0].state, Some(ProcessState::Backoff));
        assert!(!response.processes[0].running);
        assert!(response.processes[0].pid.is_none());
    }

    #[test]
    fn backoff_is_bounded() {
        assert_eq!(backoff(1), Duration::from_secs(1));
        assert_eq!(backoff(2), Duration::from_secs(2));
        assert_eq!(backoff(3), Duration::from_secs(4));
        assert_eq!(backoff(6), RESTART_MAX);
        assert_eq!(backoff(100), RESTART_MAX);
    }
}
