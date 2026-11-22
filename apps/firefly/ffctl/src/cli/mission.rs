use firefly_mission::{Plan, Request, call};
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

fn report(result: firefly_error::Result<serde_json::Value>) -> ExitCode {
    crate::ipc::report(result.map(|value| println!("{value}")))
}

impl Run for Status {
    type Output = ExitCode;
    fn run(self) -> ExitCode {
        report(call(&Request::Status { id: self.id }))
    }
}
impl Run for Submit {
    type Output = ExitCode;
    fn run(self) -> ExitCode {
        report(
            firefly_config::load::<Plan>(self.plan)
                .and_then(|plan| call(&Request::Submit { id: self.id, plan })),
        )
    }
}
impl Run for Cancel {
    type Output = ExitCode;
    fn run(self) -> ExitCode {
        report(call(&Request::Cancel { id: self.id }))
    }
}
