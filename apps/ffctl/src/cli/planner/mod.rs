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
