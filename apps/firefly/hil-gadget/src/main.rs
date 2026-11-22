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
    let result =
        dispatch(&command).map_err(|e| e.with_context("command", format_args!("{command:?}")));
    #[cfg(target_os = "linux")]
    if let Err(error) = &result {
        setup::note(&format!("{command:?} 失败：{error}"));
    }
    firefly_error::report("hil-gadget", result)
}

#[cfg(target_os = "linux")]
fn dispatch(command: &Command) -> firefly_error::Result<()> {
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
fn dispatch(_command: &Command) -> firefly_error::Result<()> {
    Err(firefly_error::Error::new(
        firefly_error::ErrorKind::Unsupported,
        "HIL gadget 依赖 Linux FunctionFS，只在板端运行",
    ))
}
