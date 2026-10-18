//! `ffctl`：地面站 CLI —— 把操作员意图发成 IPC 消息，或管理机载进程。
//!
//! 子命令是 **`<域> <动词>`** 两层（noun-verb，对照 `kubectl get pods` /
//! `docker container create` 的惯例）：域 = 被控进程，动词 = 对该域的动作。新增可控制面
//! 时加域，不加顶层动词——顶层只有域与自用命令（`fc`、`planner`、`process`，以及自用的
//! `completion` 与合成的 `help`）。
//!
//! 命令声明（域与动词、参数、校验）在 [`cli`]，域下每个动词一个模块；投递语义与
//! “无应答通道”见 [`ipc`]。用法错误（退出码 2）与 `--help`/`--version`/
//! `__complete_word__` 由 `usage-rs` 的 `parse()` 处理，规格可 `ffctl __usage_spec__` 导出。
//!
//! 退出码：成功 0；用法错误 2（`parse()`）；其余失败 1。

mod cli;
mod ipc;
mod process;

use std::process::ExitCode;

use iceoryx2::prelude::{LogLevel, set_log_level};
use usage_rs::Run;

fn main() -> ExitCode {
    // iceoryx2 缺省级别会往 stderr 打配置行；本进程的 stderr 只留给错误
    set_log_level(LogLevel::Error);
    // `parse()` 自己处理 `--help`/`--version`/用法错误（clap 风格，退出码 2）与补全请求
    cli::Ffctl::parse().command.run()
}
