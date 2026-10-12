use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::ipc::{report, send_command};

/// 解锁：通过全部解锁前检查后电机可出力
///
/// 检查项（缺一即拒，原因逐条打印在飞控日志）：状态估计就绪、已收到机体描述、
/// 被控对象状态与 IMU 新鲜、停在起飞点地面。
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Arm;

impl Run for Arm {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(send_command(kind::ARM, 0.0, "ARM"))
    }
}
