//! `ffctl`：地面站 CLI —— 把操作员意图发成 IPC 消息（一次性进程，发完即退）。
//!
//! 子命令是 **`<域> <动词>`** 两层（noun-verb，对照 `kubectl get pods` /
//! `docker container create` 的惯例）：域 = 被控进程，动词 = 对该域的动作。新增可控制面
//! 时加域，不加顶层动词——顶层只有域和 `help`，看一眼就知道系统有哪些可操作的面。
//!
//! - `fc`：飞控（`apps/fc`）——解锁/模式/起降，经 `Firefly/Command` 投递，
//!   由 `firefly_flight::FlightFsm` 逐项判定后执行；
//! - `planner`：规划（`apps/planner`）——目标点，经 `Firefly/Goal` 投递，
//!   planner 重算全局路径并重新规划（飞控只在 `TRACK` 下消费它的参考流）。
//!
//! **投递语义**：`iceoryx2` 的 `send` 在无订阅端时不报错（样本直接丢弃），所以每条消息
//! 循环投递到**至少一个订阅端收到**为止（连接异步建立，首次投递往往只建立连接）；
//! 期限内没人订阅即报错退出——这区分了“投递成功”与“没人听”。指令按 `sequence` 去重，
//! 同一条重复投递只生效一次。
//!
//! **无应答通道**：本进程不知道指令是否被接受。被拒原因在飞控终端日志与 rrd 的
//! `logs/fc`（一行中文原因），当前模式看 rrd 的 `fc/debug/state`（见 `docs/how_to_run.md`
//! §3.1）。
//!
//! 输出就是接口本身：成功一行到 stdout（给人/脚本），失败到 stderr 并非零退出
//! （用法错误 2、投递失败 1）；进程诊断才走 log 宏，本进程不写日志。

use std::fmt::Debug;
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_error::{Error, ErrorKind};
use firefly_pubsub::command::{COMMAND_TOPIC, CommandMessage, kind};
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use iceoryx2::prelude::{LogLevel, ZeroCopySend, set_log_level};
use usage::{Args, Cli, Run, Subcommands};

/// 投递期限（一次性 CLI 与订阅端的连接是异步建立的，需重试到连接建立）。
const DELIVER_TIMEOUT: Duration = Duration::from_millis(300);
/// 投递重试间隔。
const DELIVER_RETRY: Duration = Duration::from_millis(30);

/// 飞行控制与任务指令的地面站 CLI
#[derive(Cli)]
#[usage(bin = "ffctl", version)]
struct Ffctl {
    #[usage(subcommand)]
    command: Command,
}

/// `ffctl` 能控制的域
#[derive(Subcommands)]
#[usage(run)]
enum Command {
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

/// 解锁：通过全部解锁前检查后电机可出力
///
/// 检查项（缺一即拒，原因逐条打印在飞控日志）：状态估计就绪、已收到机体描述、
/// 被控对象状态与 IMU 新鲜、停在起飞点地面。
#[derive(Args)]
#[usage(effect = "write")]
struct Arm;

impl Run for Arm {
    type Output = Result<(), Error>;

    fn run(self) -> Self::Output {
        send_command(kind::ARM, 0.0, "ARM")
    }
}

/// 上锁：仅允许在地面（空中上锁 = 摔机）
#[derive(Args)]
#[usage(effect = "write")]
struct Disarm;

impl Run for Disarm {
    type Output = Result<(), Error>;

    fn run(self) -> Self::Output {
        send_command(kind::DISARM, 0.0, "DISARM")
    }
}

/// 位置保持：中止当前段（起飞/降落）并原地悬停
#[derive(Args)]
#[usage(effect = "write")]
struct Hold;

impl Run for Hold {
    type Output = Result<(), Error>;

    fn run(self) -> Self::Output {
        send_command(kind::HOLD, 0.0, "HOLD")
    }
}

/// 自动降落：降到起飞点地面，落地判定成立后自动上锁
#[derive(Args)]
#[usage(effect = "write")]
struct Land;

impl Run for Land {
    type Output = Result<(), Error>;

    fn run(self) -> Self::Output {
        send_command(kind::LAND, 0.0, "LAND")
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
    type Output = Result<(), Error>;

    fn run(self) -> Self::Output {
        let altitude = self.altitude.unwrap_or(0.0);
        if !altitude.is_finite() || altitude < 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                format!("高度必须是正数（米），收到 {altitude}"),
            ));
        }
        let label = if altitude > 0.0 {
            format!("TAKEOFF {altitude:.2} m")
        } else {
            "TAKEOFF（飞控缺省高度）".to_owned()
        };
        send_command(kind::TAKEOFF, altitude, &label)
    }
}

/// 跟踪外部参考流：交给 planner（要求参考流新鲜，否则被拒）
///
/// 接管后参考流断超过 1.0s 会被失效保护拉回位置保持。
#[derive(Args)]
#[usage(effect = "write")]
struct Track;

impl Run for Track {
    type Output = Result<(), Error>;

    fn run(self) -> Self::Output {
        send_command(kind::TRACK, 0.0, "TRACK")
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
    type Output = Result<(), Error>;

    fn run(self) -> Self::Output {
        for (axis, value) in [("x", self.x), ("y", self.y), ("z", self.z)] {
            if !value.is_finite() {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    format!("目标 {axis} 必须是有限数值，收到 {value}"),
                ));
            }
        }
        let node = create_node()?;
        let publisher = Publisher::<GoalMessage>::with_topic(&node, GOAL_TOPIC)?;
        let msg = GoalMessage {
            timestamp: wall_seconds(),
            position_x: self.x,
            position_y: self.y,
            position_z: self.z,
        };
        let receivers = deliver(&publisher, msg, GOAL_TOPIC)?;
        println!(
            "已投递目标 ({:.2}, {:.2}, {:.2}) → {GOAL_TOPIC}（{receivers} 个订阅端）",
            self.x, self.y, self.z
        );
        Ok(())
    }
}

/// 投递一条地面站指令（`Firefly/Command`）。
///
/// # Errors
/// 节点/端口创建失败，或投递期限内无订阅端（飞控未启动）。
fn send_command(kind: u8, altitude: f64, label: &str) -> Result<(), Error> {
    let node = create_node()?;
    let publisher = Publisher::<CommandMessage>::with_topic(&node, COMMAND_TOPIC)?;
    let nanos = wall_nanos();
    let msg = CommandMessage {
        timestamp: nanos as f64 / 1e9,
        kind,
        altitude,
        // 指令序号：跨进程调用必须严格递增，飞控按它去重（重复投递只生效一次）
        sequence: nanos,
    };
    let receivers = deliver(&publisher, msg, COMMAND_TOPIC)?;
    println!("已投递指令 {label} → {COMMAND_TOPIC}（{receivers} 个订阅端）");
    Ok(())
}

/// 投递到至少一个订阅端（或超时报错），返回最后一次投递的接收端数。
///
/// 重试是必要的：`send` 内部会刷新连接，但**首次**投递往往只建立连接、样本进不了订阅端
/// 缓冲区（连接是异步建立的）——只有后续投递才算数。同一条消息重复投递无害：
/// 指令按 `sequence` 去重、目标点取最新。
///
/// # Errors
/// 发送失败，或期限内始终无订阅端（目标进程没起）。
fn deliver<T: Copy + Debug + ZeroCopySend + 'static>(
    publisher: &Publisher<T>,
    msg: T,
    topic: &str,
) -> Result<usize, Error> {
    let deadline = Instant::now() + DELIVER_TIMEOUT;
    // 投递到期限为止（不看中间计数）：首次投递可能只建立连接而样本丢失，
    // 以最后一次的计数为准才可靠。
    let mut receivers;
    loop {
        receivers = publisher.publish_counted(msg)?.0;
        if Instant::now() >= deadline {
            break;
        }
        thread::sleep(DELIVER_RETRY);
    }
    if receivers == 0 {
        return Err(Error::new(
            ErrorKind::Timeout,
            format!("`{topic}` 无订阅端（目标进程未启动？），消息未投递"),
        ));
    }
    Ok(receivers)
}

/// 墙钟纳秒（自 Unix 纪元；跨调用严格递增，用作指令序号）。
fn wall_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos() as u64)
}

/// 墙钟秒（消息里的诊断时间戳；飞控/planner 只当参考，不参与时序判断）。
fn wall_seconds() -> f64 {
    wall_nanos() as f64 / 1e9
}

fn main() -> ExitCode {
    // iceoryx2 缺省级别会往 stderr 打配置行；本进程的 stderr 只留给错误
    set_log_level(LogLevel::Error);
    // `parse()` 自己处理 `--help`/`--version`/用法错误（clap 风格，退出码 2）
    match Ffctl::parse().command.run() {
        Ok(()) => ExitCode::SUCCESS,
        // 只打消息本体：错误类型里的 kind/位置是给日志的，操作员要的是一行原因
        Err(err) => {
            eprintln!("ffctl: {}", err.message());
            ExitCode::FAILURE
        }
    }
}
