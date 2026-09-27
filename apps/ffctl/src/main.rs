//! `ffctl`：地面站 CLI —— 把操作员意图发成 IPC 消息（一次性进程，发完即退）。
//!
//! 子命令是 **`<域> <动词>`** 两层（noun-verb，对照 `kubectl get pods` /
//! `docker container create` 的惯例）：域 = 被控进程，动词 = 对该域的动作。新增可控制面
//! 时加域，不加顶层动词——顶层只有域与自用命令（`completion`、合成的 `help`）。
//!
//! - `fc`：飞控（`apps/fc`）——解锁/模式/起降，经 `Firefly/Command` 投递，
//!   由 `firefly_flight::FlightFsm` 逐项判定后执行；
//! - `planner`：规划（`apps/planner`）——目标点，经 `Firefly/Goal` 投递，
//!   planner 重算全局路径并重新规划（飞控只在 `TRACK` 下消费它的参考流）。
//!
//! 投递语义与“无应答通道”见 [`ipc`]；用法错误（退出码 2）与 `--help`/`--version`/
//! `__complete_word__` 由 `usage-rs` 的 `parse()` 处理，规格可 `ffctl __usage_spec__` 导出。
//!
//! 退出码：成功 0；用法错误 2（`parse()`）；其余失败 1（[`ipc::report`]）。

mod ipc;

use std::process::ExitCode;

use firefly_error::{Error, ErrorKind};
use firefly_pubsub::command::kind;
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use iceoryx2::prelude::{LogLevel, set_log_level};
use usage::{Args, Cli, Run, Subcommands};

use ipc::{deliver, new_publisher, report, wall_seconds};

/// 飞行控制与任务指令的地面站 CLI
#[derive(Cli)]
#[usage(bin = "ffctl", version, completion)]
struct Ffctl {
    #[usage(subcommand)]
    command: Command,
}

/// `ffctl` 能控制的域
#[derive(Subcommands)]
#[usage(run)]
enum Command {
    Completion(Completion),
    Fc(Fc),
    Planner(Planner),
}

/// 飞控（`apps/fc`）：模式与起降
///
/// 指令经 `Firefly/Command` 投递，由飞控的模式与安全层
/// （`firefly_flight::FlightFsm`）逐项判定后执行。
#[derive(Args)]
#[usage(effect = "write", run)]
struct Fc {
    #[usage(subcommand)]
    command: FcCommand,
}

/// 飞控动词
#[derive(Subcommands)]
#[usage(run)]
enum FcCommand {
    Arm(Arm),
    Disarm(Disarm),
    Hold(Hold),
    Land(Land),
    Takeoff(Takeoff),
    Track(Track),
}

/// 规划（`apps/planner`）：目标
///
/// 目标点经 `Firefly/Goal` 投递，planner 重算全局路径并重新规划。
#[derive(Args)]
#[usage(effect = "write", run)]
struct Planner {
    #[usage(subcommand)]
    command: PlannerCommand,
}

/// 规划动词
#[derive(Subcommands)]
#[usage(run)]
enum PlannerCommand {
    Goal(Goal),
}

/// 打印某个 shell 的补全脚本（自用命令，不属于任何域）
///
/// 脚本在补全时回调本二进制（`parse()` 拦截隐藏请求），因此改子命令即改补全，无需重发脚本。
#[derive(Args)]
#[usage(effect = "read")]
struct Completion {
    /// 目标 shell
    #[usage(choices("bash", "elvish", "fish", "nu", "powershell", "zsh"))]
    shell: String,
}

impl Run for Completion {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(print_completion(&self.shell))
    }
}

/// 打印某 shell 的补全脚本。
///
/// # Errors
/// shell 名不在 `Shell::from_name` 支持集内（`choices` 已先挡一层）。
fn print_completion(shell: &str) -> Result<(), Error> {
    let shell = usage::complete::Shell::from_name(shell).ok_or_else(|| {
        Error::new(
            ErrorKind::InvalidArgument,
            format!("不支持的 shell：{shell}"),
        )
    })?;
    print!("{}", Ffctl::completion_script(shell));
    Ok(())
}

/// 解锁：通过全部解锁前检查后电机可出力
///
/// 检查项（缺一即拒，原因逐条打印在飞控日志）：状态估计就绪、已收到机体描述、
/// 被控对象状态与 IMU 新鲜、停在起飞点地面。
#[derive(Args)]
#[usage(effect = "write")]
struct Arm;

impl Run for Arm {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(ipc::send_command(kind::ARM, 0.0, "ARM"))
    }
}

/// 上锁：仅允许在地面（空中上锁 = 摔机）
#[derive(Args)]
#[usage(effect = "write")]
struct Disarm;

impl Run for Disarm {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(ipc::send_command(kind::DISARM, 0.0, "DISARM"))
    }
}

/// 位置保持：中止当前段（起飞/降落）并原地悬停
#[derive(Args)]
#[usage(effect = "write")]
struct Hold;

impl Run for Hold {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(ipc::send_command(kind::HOLD, 0.0, "HOLD"))
    }
}

/// 自动降落：降到起飞点地面，落地判定成立后自动上锁
#[derive(Args)]
#[usage(effect = "write")]
struct Land;

impl Run for Land {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(ipc::send_command(kind::LAND, 0.0, "LAND"))
    }
}

/// 自动起飞：爬到相对起飞点的给定高度后自动进入位置保持
#[derive(Args)]
#[usage(effect = "write")]
struct Takeoff {
    /// 相对起飞点的高度（米，0.3~10；省略 = 飞控配置的缺省起飞高度）
    altitude: Option<f64>,
}

impl Run for Takeoff {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        let altitude = self.altitude.unwrap_or(0.0);
        if !altitude.is_finite() || altitude < 0.0 {
            return report(Err(Error::new(
                ErrorKind::InvalidArgument,
                format!("高度必须是正数（米），收到 {altitude}"),
            )));
        }
        let label = if altitude > 0.0 {
            format!("TAKEOFF {altitude:.2} m")
        } else {
            "TAKEOFF（飞控缺省高度）".to_owned()
        };
        report(ipc::send_command(kind::TAKEOFF, altitude, &label))
    }
}

/// 跟踪外部参考流：交给 planner（要求参考流新鲜，否则被拒）
///
/// 接管后参考流断超过 1.0s 会被失效保护拉回位置保持。
#[derive(Args)]
#[usage(effect = "write")]
struct Track;

impl Run for Track {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        report(ipc::send_command(kind::TRACK, 0.0, "TRACK"))
    }
}

/// 发布规划目标点（地图系，米）：planner 重算全局路径并重新规划
///
/// 只是给规划侧的目标输入，不直接动飞机：要让飞机跟过去，还需 `fc track`
/// （飞控在 `TRACK` 下才消费 planner 的参考流）。
#[derive(Args)]
#[usage(effect = "write")]
struct Goal {
    /// 目标 x（地图系，米）
    x: f64,
    /// 目标 y（地图系，米）
    y: f64,
    /// 目标 z（地图系，米）
    z: f64,
}

impl Run for Goal {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        for (axis, value) in [("x", self.x), ("y", self.y), ("z", self.z)] {
            if !value.is_finite() {
                return report(Err(Error::new(
                    ErrorKind::InvalidArgument,
                    format!("目标 {axis} 必须是有限数值，收到 {value}"),
                )));
            }
        }
        report(publish_goal(&self))
    }
}

/// 发布目标点（`Firefly/Goal`）。
///
/// # Errors
/// 端口创建失败，或投递期限内无订阅端（planner 未启动）。
fn publish_goal(goal: &Goal) -> Result<(), Error> {
    let publisher = new_publisher::<GoalMessage>(GOAL_TOPIC)?;
    let msg = GoalMessage {
        timestamp: wall_seconds(),
        position_x: goal.x,
        position_y: goal.y,
        position_z: goal.z,
    };
    let receivers = deliver(&publisher, msg, GOAL_TOPIC)?;
    println!(
        "已投递目标 ({:.2}, {:.2}, {:.2}) → {GOAL_TOPIC}（{receivers} 个订阅端）",
        goal.x, goal.y, goal.z
    );
    Ok(())
}

fn main() -> ExitCode {
    // iceoryx2 缺省级别会往 stderr 打配置行；本进程的 stderr 只留给错误
    set_log_level(LogLevel::Error);
    // `parse()` 自己处理 `--help`/`--version`/用法错误（clap 风格，退出码 2）与补全请求
    Ffctl::parse().command.run()
}
