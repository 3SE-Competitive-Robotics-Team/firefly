use std::{path::PathBuf, process::ExitCode};

/// Grade the onboard lifecycle. SIL and HIL also grade flight against recorded truth.
#[derive(usage_rs::Cli)]
#[usage(bin = "firefly-verify")]
struct Arguments {
    /// Recording to verify.
    recording: PathBuf,
    #[usage(long, choices("sil", "hil", "board"))]
    transport: Option<String>,
}

fn main() -> ExitCode {
    let args = Arguments::parse();
    let report = firefly_verify::verify(&args.recording, args.transport.as_deref());
    println!("{report}");
    ExitCode::from(u8::from(report["passed"] != true))
}
