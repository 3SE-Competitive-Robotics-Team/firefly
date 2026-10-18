use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::protocol::Request;
use crate::process::request;

/// 启动进程（省略名字 = 所有 autostart 进程；显式名字忽略 autostart）
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Start {
    /// 进程名
    names: Vec<String>,
}

impl Run for Start {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        request(&Request::Start { names: self.names })
    }
}
