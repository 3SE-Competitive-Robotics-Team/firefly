use std::process::ExitCode;

use firefly_error::Error;
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use usage_rs::{Args, Run};

use crate::cli::Meters;
use crate::ipc::{deliver, new_publisher, report, wall_seconds};

/// 发布规划目标点（地图系，米）：planner 重算全局路径并重新规划
///
/// 只是给规划侧的目标输入，不直接动飞机：要让飞机跟过去，还需 `fc track`
/// （飞控在 `TRACK` 下才消费 planner 的参考流）。
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Goal {
    /// 目标 x（地图系，米）
    x: Meters,
    /// 目标 y（地图系，米）
    y: Meters,
    /// 目标 z（地图系，米）
    z: Meters,
}

impl Run for Goal {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(publish(&self))
    }
}

/// 发布目标点（`Firefly/Goal`）。
///
/// # Errors
/// 端口创建失败，或投递期限内无订阅端（planner 未启动）。
fn publish(goal: &Goal) -> Result<(), Error> {
    let (x, y, z) = (goal.x.get(), goal.y.get(), goal.z.get());
    let publisher = new_publisher::<GoalMessage>(GOAL_TOPIC)?;
    let msg = GoalMessage {
        timestamp: wall_seconds(),
        position_x: x,
        position_y: y,
        position_z: z,
    };
    let receivers = deliver(&publisher, msg, GOAL_TOPIC)?;
    println!("已投递目标 ({x:.2}, {y:.2}, {z:.2}) → {GOAL_TOPIC}（{receivers} 个订阅端）");
    Ok(())
}
