use std::process::ExitCode;

use firefly_error::Error;
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use usage_rs::{Args, Run};

use crate::cli::Meters;
use crate::ipc::{deliver, new_publisher, report, wall_seconds};

/// 发布规划目标点（odom 系，米）：planner 重算全局路径并重新规划
///
/// 只是给规划侧的目标输入，不直接动飞机：要让飞机跟过去，还需 `fc track`
/// （飞控在 `TRACK` 下才消费 planner 的参考流）。
#[derive(Args)]
#[usage(effect = "write")]
pub(crate) struct Goal {
    /// 目标 x（odom 系，米）
    x: Meters,
    /// 目标 y（odom 系，米）
    y: Meters,
    /// 目标 z（odom 系，米）
    z: Meters,
    /// 终点航向（度，odom +X 为零，逆时针为正）；省略则由路径决定
    #[usage(long)]
    yaw: Option<f64>,
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
    if goal.yaw.is_some_and(|yaw| !yaw.is_finite()) {
        return Err(Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "yaw must be finite",
        ));
    }
    let (x, y, z) = (goal.x.get(), goal.y.get(), goal.z.get());
    let publisher = new_publisher::<GoalMessage>(GOAL_TOPIC)?;
    let msg = GoalMessage {
        timestamp: wall_seconds(),
        position_x: x,
        position_y: y,
        position_z: z,
        yaw: goal.yaw.unwrap_or(0.).to_radians(),
        has_yaw: u64::from(goal.yaw.is_some()),
    };
    let receivers = deliver(&publisher, msg, GOAL_TOPIC)?;
    println!("已投递目标 ({x:.2}, {y:.2}, {z:.2}) → {GOAL_TOPIC}（{receivers} 个订阅端）");
    Ok(())
}
