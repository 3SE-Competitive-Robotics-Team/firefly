use std::process::ExitCode;

use firefly_pubsub::command::kind;
use usage_rs::{Args, Run};

use crate::cli::Meters;
use crate::ipc::{report, send_command};

/// 自动起飞：爬到相对起飞点的给定高度后自动进入位置保持
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Takeoff {
    /// 相对起飞点的高度（米；上限由飞控配置约束，省略 = 飞控配置的缺省起飞高度）
    #[usage(
        validate = "float(value) >= 0",
        validate_error = "高度必须是非负的米数"
    )]
    altitude: Option<Meters>,
}

impl Run for Takeoff {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let altitude = self.altitude.map_or(0.0, Meters::get);
        let label = if altitude > 0.0 {
            format!("TAKEOFF {altitude:.2} m")
        } else {
            "TAKEOFF（飞控缺省高度）".to_owned()
        };
        report(send_command(kind::TAKEOFF, altitude, &label))
    }
}
