use std::path::PathBuf;
use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::{config, report, set_autostart};

/// 禁止进程在下次 supervisor 启动时自启动（不停止当前进程）
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Disable {
    /// 进程名
    names: Vec<String>,
    /// 进程清单
    #[usage(long)]
    config: Option<PathBuf>,
}

impl Run for Disable {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let path = self
            .config
            .unwrap_or_else(|| PathBuf::from(config::DEFAULT_PATH));
        report(set_autostart(&path, &self.names, false))
    }
}
