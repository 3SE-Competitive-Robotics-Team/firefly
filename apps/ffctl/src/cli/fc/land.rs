use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::ipc::{report, send_command};

/// 自动降落：降到起飞点地面，落地判定成立后自动上锁
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Land;

impl Run for Land {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(send_command(kind::LAND, 0.0, "LAND"))
    }
}
