use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::ipc::{report, send_command};

/// 上锁：仅允许在地面（空中上锁 = 摔机）
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Disarm;

impl Run for Disarm {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(send_command(kind::DISARM, 0.0, "DISARM"))
    }
}
