use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::ipc::{report, send_command};

/// 跟踪外部参考流：交给 planner（要求参考流新鲜，否则被拒）
///
/// 跟踪期间参考流持续缺失达到飞控配置的 `reference_timeout` 后，切回位置保持。
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Track;

impl Run for Track {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(send_command(kind::TRACK, 0.0, "TRACK"))
    }
}
