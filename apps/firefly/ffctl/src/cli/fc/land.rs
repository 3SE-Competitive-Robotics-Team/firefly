use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::ipc::{report, send_command};

/// 自动降落并在触地确认后上锁
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Land {
    /// 使用下视视觉对准己方机库，再下降并确认触地。
    #[usage(long)]
    precision: bool,
}

impl Run for Land {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let (kind, name) = if self.precision {
            (kind::PRECISION_LAND, "PRECISION_LAND")
        } else {
            (kind::LAND, "LAND")
        };
        report(send_command(kind, 0.0, name))
    }
}
