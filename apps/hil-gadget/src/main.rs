#[cfg(target_os = "linux")]
mod setup;

#[cfg(any(target_os = "linux", test))]
mod transition;

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
    let takes_argument = command == "diag";
    if args.len() > 2 || (args.len() == 2 && !takes_argument) {
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
