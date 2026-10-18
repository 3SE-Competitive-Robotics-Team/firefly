use std::path::PathBuf;
use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::{config, report, set_autostart};

/// 允许进程在下次 supervisor 启动时自启动
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Enable {
    /// 进程名
    names: Vec<String>,
    /// 进程清单
    #[usage(long)]
    config: Option<PathBuf>,
}

impl Run for Enable {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let path = self
            .config
            .unwrap_or_else(|| PathBuf::from(config::DEFAULT_PATH));
        report(set_autostart(&path, &self.names, true))
    }
}
