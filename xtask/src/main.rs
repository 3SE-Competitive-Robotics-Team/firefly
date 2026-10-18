//! 仓库构建与机载部署入口。

mod archive;
mod container;
mod deploy;
mod release;
mod target;

use std::path::Path;
use std::process::ExitCode;

const USAGE: &str = "\
usage:
  cargo xtask release [--target x64|aarch64]
  cargo xtask deploy [--device <serial>]
  cargo xtask unpack <package> <directory>";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}\n{USAGE}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (command, rest) = args
        .split_first()
        .ok_or_else(|| "missing command".to_owned())?;
    match command.as_str() {
        "release" => release::run(rest),
        "deploy" => deploy::run(rest),
        "unpack" => unpack(rest),
        other => Err(format!("unknown command: {other}")),
    }
}

fn unpack(args: &[String]) -> Result<(), String> {
    let [package, destination] = args else {
        return Err("unpack expects <package> <directory>".to_owned());
    };
    let count = archive::unpack(Path::new(package), Path::new(destination))
        .map_err(|error| format!("unpack {package}: {error}"))?;
    println!("unpacked {count} files into {destination}");
    Ok(())
}
