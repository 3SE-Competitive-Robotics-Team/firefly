use firefly_mission::{Plan, Request};
use std::{path::PathBuf, process::ExitCode};
use usage_rs::{Args, Run, Subcommands};

/// Control onboard missions.
#[derive(Args)]
#[usage(run)]
pub(crate) struct Mission {
    #[usage(subcommand)]
    command: MissionCommand,
}

#[derive(Subcommands)]
#[usage(run)]
pub(crate) enum MissionCommand {
    Status(Status),
    Submit(Submit),
    Cancel(Cancel),
}

/// Show run readiness or an individual mission receipt.
#[derive(Args)]
pub(crate) struct Status {
    id: Option<String>,
}
/// Execute a plan once under a caller-chosen mission ID.
#[derive(Args)]
pub(crate) struct Submit {
    id: String,
    plan: PathBuf,
}
/// End this mission and request a controlled landing.
#[derive(Args)]
pub(crate) struct Cancel {
    id: String,
}

fn report(result: Result<serde_json::Value, String>) -> ExitCode {
    match result {
        Ok(value) => {
            println!("{value}");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
impl Run for Status {
    type Output = ExitCode;
    fn run(self) -> ExitCode {
        report(firefly_mission::call(&Request::Status { id: self.id }).map_err(|e| e.to_string()))
    }
}
impl Run for Submit {
    type Output = ExitCode;
    fn run(self) -> ExitCode {
        report(
            firefly_config::load::<Plan>(self.plan)
                .map_err(|e| e.to_string())
                .and_then(|plan| {
                    firefly_mission::call(&Request::Submit { id: self.id, plan })
                        .map_err(|e| e.to_string())
                }),
        )
    }
}
impl Run for Cancel {
    type Output = ExitCode;
    fn run(self) -> ExitCode {
        report(firefly_mission::call(&Request::Cancel { id: self.id }).map_err(|e| e.to_string()))
    }
}
