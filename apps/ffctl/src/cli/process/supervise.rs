use std::path::PathBuf;
use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::{config, report, supervisor};

/// 常驻 supervisor：按清单拉起并监控进程（systemd `ExecStart`）
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Supervise {
    /// 进程清单
    #[usage(long)]
    config: Option<PathBuf>,
}

impl Run for Supervise {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let path = self
            .config
            .unwrap_or_else(|| PathBuf::from(config::DEFAULT_PATH));
        report(supervisor::run(&path))
    }
}
