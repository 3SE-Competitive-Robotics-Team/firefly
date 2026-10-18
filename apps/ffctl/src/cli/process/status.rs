use std::collections::HashMap;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use nix::sys::termios::{self, LocalFlags, SetArg, Termios};
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;
use usage_rs::{Args, Run};

use crate::process::client;
use crate::process::config::{self, Config};
use crate::process::metrics::{self, Sample};
use crate::process::protocol::{ProcessStatus, Request};

const SAMPLE: Duration = Duration::from_millis(250);
const DEFAULT_INTERVAL: f64 = 1.0;

/// 查看 supervisor 中的进程状态、CPU 与内存占用
#[derive(Args)]
#[usage(effect = "read")]
pub(crate) struct Status {
    /// 进程清单
    #[usage(long)]
    config: Option<PathBuf>,
    /// 持续刷新（类似 top；q 或 Ctrl-C 退出）
    #[usage(short, long)]
    watch: bool,
    /// 刷新间隔（秒）
    #[usage(
        short,
        long,
        validate = "float(value) > 0",
        validate_error = "刷新间隔必须是正数秒"
    )]
    interval: Option<f64>,
}

impl Run for Status {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let path = self
            .config
            .unwrap_or_else(|| PathBuf::from(config::DEFAULT_PATH));
        let config = match Config::load(&path) {
            Ok(config) => config,
            Err(error) => return crate::process::report(Err(error)),
        };
        let seconds = self.interval.unwrap_or(DEFAULT_INTERVAL);
        if !(seconds.is_finite() && seconds > 0.0) {
            return crate::process::report(Err("刷新间隔必须是正数秒".to_owned()));
        }
        if self.watch {
            watch(&config, Duration::from_secs_f64(seconds))
        } else {
            once(&config)
        }
    }
}

fn once(config: &Config) -> ExitCode {
    let statuses = match query() {
        Ok(statuses) => statuses,
        Err(error) => {
            eprintln!("ffctl: {error}");
            Vec::new()
        }
    };
    let previous = collect(&statuses);
    let started = Instant::now();
    if !previous.is_empty() {
        thread::sleep(SAMPLE);
    }
    let current = collect(&statuses);
    render(config, &statuses, &previous, &current, started.elapsed());
    ExitCode::SUCCESS
}

fn watch(config: &Config, interval: Duration) -> ExitCode {
    let mut statuses = match query() {
        Ok(statuses) => statuses,
        Err(error) => return crate::process::report(Err(error)),
    };
    let quit = Arc::new(AtomicBool::new(false));
    let stop = Arc::new(AtomicBool::new(false));
    for signal in [SIGINT, SIGTERM] {
        if let Err(error) = flag::register(signal, Arc::clone(&stop)) {
            eprintln!("ffctl: 安装信号处理失败: {error}");
            return ExitCode::FAILURE;
        }
    }

    let raw = RawMode::enable();
    if raw.is_some() {
        let quit = Arc::clone(&quit);
        let _ = thread::spawn(move || {
            let mut byte = [0_u8; 1];
            while std::io::stdin().read_exact(&mut byte).is_ok() {
                if matches!(byte[0], b'q' | b'Q') {
                    quit.store(true, Ordering::Relaxed);
                    break;
                }
            }
        });
    }

    let mut previous = collect(&statuses);
    let mut last = Instant::now();
    loop {
        if sleep_or_stop(interval, &quit, &stop) {
            break;
        }
        statuses = match query() {
            Ok(statuses) => statuses,
            Err(error) => return crate::process::report(Err(error)),
        };
        let now = Instant::now();
        let current = collect(&statuses);
        print!("\x1b[2J\x1b[H");
        render(
            config,
            &statuses,
            &previous,
            &current,
            now.saturating_duration_since(last),
        );
        let _ = std::io::stdout().flush();
        previous = current;
        last = now;
    }
    drop(raw);
    println!();
    ExitCode::SUCCESS
}

fn sleep_or_stop(duration: Duration, quit: &AtomicBool, stop: &AtomicBool) -> bool {
    let deadline = Instant::now() + duration;
    loop {
        if quit.load(Ordering::Relaxed) || stop.load(Ordering::Relaxed) {
            return true;
        }
        let now = Instant::now();
        if now >= deadline {
            return false;
        }
        thread::sleep((deadline - now).min(Duration::from_millis(100)));
    }
}

struct RawMode {
    original: Termios,
}

impl RawMode {
    fn enable() -> Option<Self> {
        let original = termios::tcgetattr(std::io::stdin()).ok()?;
        let mut raw = original.clone();
        raw.local_flags
            .remove(LocalFlags::ICANON | LocalFlags::ECHO);
        termios::tcsetattr(std::io::stdin(), SetArg::TCSANOW, &raw).ok()?;
        Some(Self { original })
    }
}

impl Drop for RawMode {
    fn drop(&mut self) {
        let _ = termios::tcsetattr(std::io::stdin(), SetArg::TCSANOW, &self.original);
    }
}

fn query() -> Result<Vec<ProcessStatus>, String> {
    client::send(&Request::Status).map(|response| response.processes)
}

fn collect(statuses: &[ProcessStatus]) -> HashMap<u32, Sample> {
    statuses
        .iter()
        .filter_map(|status| {
            let pid = status.pid?;
            metrics::sample(pid).map(|sample| (pid, sample))
        })
        .collect()
}

fn render(
    config: &Config,
    statuses: &[ProcessStatus],
    previous: &HashMap<u32, Sample>,
    current: &HashMap<u32, Sample>,
    elapsed: Duration,
) {
    println!(
        "{:<14} {:<8} {:<8} {:<7} {:<9} {:<9} AUTOSTART",
        "NAME", "RUNNING", "PID", "CPU%", "MEM(MiB)", "RESTARTS"
    );
    for process in &config.processes {
        let status = statuses.iter().find(|status| status.name == process.name);
        let running = status.is_some_and(|status| status.running);
        let pid = status.and_then(|status| status.pid);
        let usage = pid.and_then(|pid| {
            current
                .get(&pid)
                .map(|sample| metrics::usage(previous.get(&pid).copied(), *sample, elapsed))
        });
        let cpu = usage.map_or_else(
            || "-".to_owned(),
            |usage| format!("{:.1}", usage.cpu_percent),
        );
        let memory = usage.map_or_else(|| "-".to_owned(), |usage| format!("{:.1}", usage.rss_mib));
        println!(
            "{:<14} {:<8} {:<8} {:<7} {:<9} {:<9} {}",
            process.name,
            if running { "yes" } else { "no" },
            pid.map_or_else(|| "-".to_owned(), |pid| pid.to_string()),
            cpu,
            memory,
            status.map_or(0, |status| status.restarts),
            if process.autostart { "yes" } else { "no" }
        );
    }
}
