//! 退出码：成功 0；用法错误 2；其余失败 1。

mod cli;
mod ipc;
mod process;

use std::process::ExitCode;

use iceoryx2::prelude::{LogLevel, set_log_level};
use usage_rs::Run;

fn main() -> ExitCode {
    set_log_level(LogLevel::Error);
    cli::Ffctl::parse().command.run()
}
