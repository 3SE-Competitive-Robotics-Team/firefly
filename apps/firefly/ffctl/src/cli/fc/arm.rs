use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::ipc::{report, send_command};

/// 解锁：通过全部解锁前检查后电机可出力
///
/// 飞控按顺序检查解锁条件，拒绝时报告首个未通过项。
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Arm;

impl Run for Arm {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(send_command(kind::ARM, 0.0, "ARM"))
    }
}
