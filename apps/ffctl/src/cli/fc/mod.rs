mod arm;
mod disarm;
mod hold;
mod land;
mod takeoff;
mod track;

use usage_rs::{Args, Subcommands};

use arm::Arm;
use disarm::Disarm;
use hold::Hold;
use land::Land;
use takeoff::Takeoff;
use track::Track;

/// 飞控（`apps/fc`）：模式与起降
#[derive(Args)]
#[usage(effect = "write", run)]
pub(crate) struct Fc {
    #[usage(subcommand)]
    command: FcCommand,
}

/// 飞控动词
#[derive(Subcommands)]
#[usage(run)]
pub(crate) enum FcCommand {
    Arm(Arm),
    Disarm(Disarm),
    Hold(Hold),
    Land(Land),
    Takeoff(Takeoff),
    Track(Track),
}
