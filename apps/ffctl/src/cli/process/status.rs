use std::path::PathBuf;
use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::client;
use crate::process::config;
use crate::process::protocol::Request;

/// 查看 supervisor 中的进程状态与自启动标记
#[derive(Args)]
#[usage(effect = "read")]
pub(crate) struct Status {
    /// 进程清单
    #[usage(long)]
    config: Option<PathBuf>,
}

impl Run for Status {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let path = self
            .config
            .unwrap_or_else(|| PathBuf::from(config::DEFAULT_PATH));
        let config = match config::Config::load(&path) {
            Ok(config) => config,
            Err(error) => return crate::process::report(Err(error)),
        };
        let statuses = match client::send(&Request::Status) {
            Ok(response) => response.processes,
            Err(error) => {
                eprintln!("ffctl: {error}");
                Vec::new()
            }
        };

        println!(
            "{:<14} {:<8} {:<8} {:<10} AUTOSTART",
            "NAME", "RUNNING", "PID", "RESTARTS"
        );
        for process in &config.processes {
            let status = statuses.iter().find(|status| status.name == process.name);
            let running = status.is_some_and(|status| status.running);
            let pid = status
                .and_then(|status| status.pid)
                .map_or_else(|| "-".to_owned(), |pid| pid.to_string());
            println!(
                "{:<14} {:<8} {:<8} {:<10} {}",
                process.name,
                if running { "yes" } else { "no" },
                pid,
                status.map_or(0, |status| status.restarts),
                if process.autostart { "yes" } else { "no" }
            );
        }
        ExitCode::SUCCESS
    }
}
