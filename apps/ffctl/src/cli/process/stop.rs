use std::process::ExitCode;

use usage_rs::{Args, Run};

use crate::process::protocol::Request;
use crate::process::request;

/// 停止进程（省略名字 = 全部，逆序）
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Stop {
    /// 进程名
    names: Vec<String>,
}

impl Run for Stop {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        request(&Request::Stop { names: self.names })
    }
}
