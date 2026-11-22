mod archive;
mod container;
mod deploy;
mod deploy_hil;
mod mission;
mod release;
mod target;

use std::path::PathBuf;
use std::process::ExitCode;

use target::Target;
use usage_rs::{Args, Cli, Subcommands};

/// 构建、部署和验证 Firefly
#[derive(Cli)]
#[usage(bin = "cargo xtask")]
struct Arguments {
    #[usage(subcommand)]
    command: Command,
}

#[derive(Subcommands)]
enum Command {
    /// 构建 release 程序
    Release {
        #[usage(long, value_enum, default = "host")]
        target: Target,
    },
    /// 安装板端程序
    Deploy(Device),
    /// 部署并切换到 USB HIL
    DeployHil(Device),
    /// 在本机仿真中运行计划
    Sil(Mission),
    /// 在硬件在环仿真中运行计划
    Hil(Mission),
    /// 列出工作区的飞行进程
    Processes { workspace: PathBuf },
    /// 解压部署包
    Unpack {
        package: PathBuf,
        directory: PathBuf,
    },
}

#[derive(Args)]
struct Device {
    /// ADB 设备序列号
    #[usage(long, default = "firefly-0001")]
    device: String,
}

#[derive(Args)]
struct Mission {
    #[usage(long, default = "42")]
    seed: u64,
    /// 按执行顺序列出的任务计划
    #[usage(required, value_name = "PLAN")]
    plans: Vec<PathBuf>,
}

fn main() -> ExitCode {
    match run(Arguments::parse().command) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run(command: Command) -> Result<(), String> {
    match command {
        Command::Processes { workspace } => {
            let ids = xtask::processes::active(&workspace).map_err(|e| e.to_string())?;
            println!("{}", serde_json::json!(ids));
            Ok(())
        }
        Command::Release { target } => release::build(target),
        Command::Deploy(args) => deploy::run(&args.device),
        Command::DeployHil(args) => deploy_hil::run(&args.device),
        Command::Sil(args) => {
            mission::run(false, &args.plans, args.seed).map_err(|e| e.to_string())
        }
        Command::Hil(args) => mission::run(true, &args.plans, args.seed).map_err(|e| e.to_string()),
        Command::Unpack { package, directory } => {
            let count = archive::unpack(&package, &directory)
                .map_err(|error| format!("unpack {}: {error}", package.display()))?;
            println!("unpacked {count} files into {}", directory.display());
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::OsStr;

    #[test]
    fn mission_cli_preserves_plan_order_and_requires_at_least_one_plan() {
        let args = ["sil", "first.toml", "--seed", "7", "second.toml"].map(OsStr::new);
        let Command::Sil(mission) = Arguments::parse_from(&args).unwrap().command else {
            panic!("expected sil");
        };
        assert_eq!(mission.seed, 7);
        assert_eq!(
            mission.plans,
            [PathBuf::from("first.toml"), PathBuf::from("second.toml")]
        );
        assert!(Arguments::parse_from(&[OsStr::new("hil")]).is_err());
    }
}
