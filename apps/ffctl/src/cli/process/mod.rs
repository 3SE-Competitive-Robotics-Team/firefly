//! 进程域（`ffctl process`）：supervisor 与进程清单管理。

mod disable;
mod enable;
mod restart;
mod start;
mod status;
mod stop;
mod supervise;

use usage_rs::{Args, Subcommands};

use disable::Disable;
use enable::Enable;
use restart::Restart;
use start::Start;
use status::Status;
use stop::Stop;
use supervise::Supervise;

/// 机载进程（supervisor）
#[derive(Args)]
#[usage(effect = "write", run)]
pub(crate) struct Process {
    #[usage(subcommand)]
    command: ProcessCommand,
}

/// 进程动词
#[derive(Subcommands)]
#[usage(run)]
pub(crate) enum ProcessCommand {
    Start(Start),
    Stop(Stop),
    Restart(Restart),
    Enable(Enable),
    Disable(Disable),
    Status(Status),
    Supervise(Supervise),
}
