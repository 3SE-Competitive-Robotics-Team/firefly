//! 板端 HIL USB gadget 管理：准备 configfs 功能、写入 `FunctionFS` 描述符并绑定 UDC。
//!
//! HIL 与 `adb` 二选一：UDC 的端点数量不足以同时容纳 adb 与 HIL 双向端点，
//! HIL 期间 adb 不可用。`down`/看门狗先停 HIL 进程集、写回正常进程清单，再重建
//! adb-only gadget 并启动正常 `firefly`。`up` 必须在 UDC 未绑定时重建功能
//! （`FunctionFS` 要求描述符在绑定前写入）。重启后镜像自带的启动流程会恢复
//! 原状，不依赖本工具的清理。
//!
//! 用法（板端 root）：
//!
//! ```text
//! hil-gadget up      # 重建 gadget，启动描述符守护进程并绑定
//! hil-gadget serve   # 仅写描述符并保持 ep0（由 up 派生）
//! hil-gadget down    # 解绑、移除 HIL 功能并恢复 adb
//! hil-gadget status  # 打印接口/端点状态
//! ```

#[cfg(target_os = "linux")]
mod setup;

use std::process::ExitCode;

const USAGE: &str = "\
usage:
  hil-gadget up
  hil-gadget serve
  hil-gadget down
  hil-gadget watchdog
  hil-gadget diag
  hil-gadget status";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(command) = args.first() else {
        eprintln!("missing command\n{USAGE}");
        return ExitCode::FAILURE;
    };
    if args.len() > 2 || (args.len() == 2 && command != "diag") {
        eprintln!("unexpected argument\n{USAGE}");
        return ExitCode::FAILURE;
    }
    match dispatch(command.as_str(), args.get(1).map_or("", String::as_str)) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("hil-gadget {command}: {error}");
            #[cfg(target_os = "linux")]
            setup::note(&format!("{command} 失败：{error}"));
            ExitCode::FAILURE
        }
    }
}

#[cfg(target_os = "linux")]
fn dispatch(command: &str, argument: &str) -> Result<(), String> {
    match command {
        "up" => setup::up(),
        "serve" => setup::serve(),
        "down" => setup::down(),
        "watchdog" => setup::watchdog(),
        "diag" => setup::diag(if argument.is_empty() { "hil" } else { argument }),
        "status" => {
            setup::status();
            Ok(())
        }
        other => Err(format!("unknown command: {other}")),
    }
}

#[cfg(not(target_os = "linux"))]
fn dispatch(command: &str, _argument: &str) -> Result<(), String> {
    Err(format!("{command} 依赖 Linux FunctionFS，只在板端构建运行"))
}
