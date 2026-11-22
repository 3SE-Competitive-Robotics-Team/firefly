use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::protocol::Request;
use crate::process::request;

/// 重启进程（省略名字 = 停止全部运行中的进程，再启动所有 autostart 进程）
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Restart {
    /// 进程名
    names: Vec<String>,
}

impl Run for Restart {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        request(&Request::Restart { names: self.names })
    }
}
