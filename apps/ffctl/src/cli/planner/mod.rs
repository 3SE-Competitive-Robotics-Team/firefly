//! 规划域（`apps/planner`）：目标。
//!
//! 目标点经 `Firefly/Goal` 投递，planner 重算全局路径并重新规划（飞控只在 `TRACK` 下
//! 消费它的参考流）。

mod goal;

use usage_rs::{Args, Subcommands};

use goal::Goal;

/// 规划（`apps/planner`）：目标
#[derive(Args)]
#[usage(effect = "write", run)]
pub(crate) struct Planner {
    #[usage(subcommand)]
    command: PlannerCommand,
}

/// 规划动词
#[derive(Subcommands)]
#[usage(run)]
pub(crate) enum PlannerCommand {
    Goal(Goal),
}
