use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::ipc::{report, send_command};

/// 位置保持：中止当前段（起飞/降落）并原地悬停
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Hold;

impl Run for Hold {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(send_command(kind::HOLD, 0.0, "HOLD"))
    }
}
