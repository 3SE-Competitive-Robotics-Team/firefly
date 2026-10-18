//! `ffctl` 的命令声明：域（子命令组）与自用命令。
//!
//! 顶层只有域与自用命令——`fc`、`planner` 是被控进程（域），`completion` 是自用命令，
//! 帮助由 `usage-rs` 合成。域下每个动词一个模块，声明（`Args`/`Subcommands`）与实现
//! （`Run`）放在一起，对照官方 quickstart 的「一命令一模块」。

mod completion;
mod fc;
mod meters;
mod planner;
mod process;

pub(crate) use completion::Completion;
pub(crate) use fc::Fc;
pub(crate) use meters::Meters;
pub(crate) use planner::Planner;
pub(crate) use process::Process;

use usage_rs::{Cli, Subcommands};

/// 飞行控制与任务指令的地面站 CLI
#[derive(Cli)]
#[usage(bin = "ffctl", version, completion)]
pub(crate) struct Ffctl {
    #[usage(subcommand)]
    pub(crate) command: Command,
}

/// `ffctl` 能控制的域
#[derive(Subcommands)]
#[usage(run)]
pub(crate) enum Command {
    #[usage(display_order = 0)]
    Fc(Fc),
    #[usage(display_order = 1)]
    Planner(Planner),
    #[usage(display_order = 2)]
    Process(Process),
    #[usage(display_order = 3)]
    Completion(Completion),
}
