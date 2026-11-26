use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::{config, report, supervisor};

/// 常驻 supervisor：按清单拉起并监控进程（systemd `ExecStart`）
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Supervise;

impl Run for Supervise {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(supervisor::run(&config::default_path()))
    }
}
