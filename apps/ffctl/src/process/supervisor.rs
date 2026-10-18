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
use crate::process::protocol::{ProcessStatus, Request, Response};

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
    child: Option<Child>,
    desired: bool,
    restarts: u32,
    started_at: Option<Instant>,
    restart_at: Option<Instant>,
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
            child: None,
            desired: false,
            restarts: 0,
            started_at: None,
            restart_at: None,
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

    stop_all(&mut managed);
    let _ = fs::remove_file(SOCKET);
    Ok(())
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
                entry.child = Some(child);
                entry.desired = true;
                entry.started_at = Some(Instant::now());
                entry.restarts = 0;
                thread::sleep(START_DELAY);
                let exited = entry
                    .child
                    .as_mut()
                    .and_then(|child| child.try_wait().ok())
                    .flatten();
                if let Some(status) = exited {
                    entry.child = None;
                    entry.desired = false;
                    entry.started_at = None;
                    eprintln!("ffctl: {name} 启动后立即退出（{status}）");
                    return Some(format!("{name}（{status}）"));
                }
            }
            Err(error) => {
                entry.desired = false;
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
        let exited = match entry.child.as_mut() {
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
            entry.child = None;
            entry.started_at = None;
            if entry.desired {
                entry.restarts = entry.restarts.saturating_add(1);
                let delay = backoff(entry.restarts);
                eprintln!(
                    "ffctl: {} 退出（{status}），{}s 后重启",
                    entry.process.name,
                    delay.as_secs()
                );
                entry.restart_at = Some(now + delay);
            } else {
                eprintln!("ffctl: {} 已停止（{status}）", entry.process.name);
            }
        } else if entry.restarts > 0
            && entry
                .started_at
                .is_some_and(|started| now.saturating_duration_since(started) >= STABLE_RESET)
        {
            entry.restarts = 0;
        }

        if entry.child.is_none() && entry.desired && entry.restart_at.is_some_and(|at| now >= at) {
            entry.restart_at = None;
            match spawn(&entry.process) {
                Ok(child) => {
                    entry.child = Some(child);
                    entry.started_at = Some(now);
                }
                Err(error) => {
                    entry.restarts = entry.restarts.saturating_add(1);
                    let delay = backoff(entry.restarts);
                    eprintln!(
                        "ffctl: {} 重启失败（{error}），{}s 后重试",
                        entry.process.name,
                        delay.as_secs()
                    );
                    entry.restart_at = Some(now + delay);
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
        if managed[index].child.is_some() {
            continue;
        }
        match spawn(&managed[index].process) {
            Ok(child) => {
                managed[index].child = Some(child);
                managed[index].desired = true;
                managed[index].started_at = Some(Instant::now());
                managed[index].restart_at = None;
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
        if entry.child.is_none() {
            continue;
        }
        entry.desired = false;
        entry.restart_at = None;
        match stop_child(entry) {
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

fn stop_all(managed: &mut [Managed]) {
    for index in (0..managed.len()).rev() {
        let entry = &mut managed[index];
        if entry.child.is_none() {
            continue;
        }
        entry.desired = false;
        entry.restart_at = None;
        if let Err(error) = stop_child(entry) {
            eprintln!("ffctl: {error}");
        }
    }
}

fn stop_child(entry: &mut Managed) -> Result<(), String> {
    let name = entry.process.name.clone();
    let Some(pid) = entry.child.as_ref().map(Child::id) else {
        return Ok(());
    };
    let pid = i32::try_from(pid).map_err(|_| format!("{name} 的 pid 超出范围"))?;
    match kill(Pid::from_raw(pid), Signal::SIGINT) {
        Ok(()) | Err(Errno::ESRCH) => {}
        Err(error) => return Err(format!("{name} 发送 SIGINT 失败：{error}")),
    }

    let deadline = Instant::now() + STOP_TIMEOUT;
    loop {
        let exited = {
            let child = entry
                .child
                .as_mut()
                .ok_or_else(|| format!("{name} 状态不一致"))?;
            match child.try_wait() {
                Ok(status) => status,
                Err(error) => return Err(format!("{name} try_wait: {error}")),
            }
        };
        if let Some(status) = exited {
            entry.child = None;
            entry.started_at = None;
            eprintln!("ffctl: {name} 已停止（{status}）");
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!("{name} 在 {}s 内未退出", STOP_TIMEOUT.as_secs()));
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
            running: entry.child.is_some(),
            pid: entry.child.as_ref().map(Child::id),
            restarts: entry.restarts,
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

    #[test]
    fn backoff_is_bounded() {
        assert_eq!(backoff(1), Duration::from_secs(1));
        assert_eq!(backoff(2), Duration::from_secs(2));
        assert_eq!(backoff(3), Duration::from_secs(4));
        assert_eq!(backoff(6), RESTART_MAX);
        assert_eq!(backoff(100), RESTART_MAX);
    }
}
