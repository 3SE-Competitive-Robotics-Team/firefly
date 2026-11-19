use std::{path::PathBuf, process::ExitCode};
const USAGE: &str = "usage: firefly-verify [--transport sil|hil|board] <recording.rrd>";

fn main() -> ExitCode {
    let mut path = None;
    let mut mode = None;
    let mut args = std::env::args_os().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--help" || arg == "-h" {
            println!(
                "{USAGE}\nBoard grades onboard lifecycle. SIL and HIL also grade physical flight against recorded truth."
            );
            return ExitCode::SUCCESS;
        }
        if arg == "--transport" && mode.is_none() {
            mode = args.next().and_then(|v| v.into_string().ok());
            if !matches!(mode.as_deref(), Some("board" | "sil" | "hil")) {
                return usage();
            }
        } else if path.is_none() && !arg.as_encoded_bytes().starts_with(b"-") {
            path = Some(PathBuf::from(arg));
        } else {
            return usage();
        }
    }
    let Some(path) = path else { return usage() };
    let report = firefly_verify::verify(&path, mode.as_deref());
    println!("{report}");
    ExitCode::from(u8::from(report["passed"] != true))
}
fn usage() -> ExitCode {
    eprintln!("{USAGE}");
    ExitCode::from(2)
}
