#[cfg(target_os = "linux")]
mod setup;

#[cfg(any(target_os = "linux", test))]
mod transition;

use std::process::ExitCode;

/// 管理板端 USB HIL gadget
#[derive(usage_rs::Cli)]
#[usage(bin = "hil-gadget")]
struct Arguments {
    #[usage(subcommand)]
    command: Command,
}

#[derive(Debug, usage_rs::Subcommands)]
enum Command {
    Up,
    Serve,
    Down,
    Watchdog,
    /// 检查 HIL USB 功能并恢复 ADB
    Diag,
    Status,
}

fn main() -> ExitCode {
    let command = Arguments::parse().command;
    match dispatch(&command) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("hil-gadget {command:?}: {error}");
            #[cfg(target_os = "linux")]
            setup::note(&format!("{command:?} 失败：{error}"));
            ExitCode::FAILURE
        }
    }
}

#[cfg(target_os = "linux")]
fn dispatch(command: &Command) -> Result<(), String> {
    match command {
        Command::Up => setup::up(),
        Command::Serve => setup::serve(),
        Command::Down => setup::down(),
        Command::Watchdog => setup::watchdog(),
        Command::Diag => setup::diag(),
        Command::Status => {
            setup::status();
            Ok(())
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn dispatch(_command: &Command) -> Result<(), String> {
    Err("HIL gadget 依赖 Linux FunctionFS，只在板端运行".into())
}
