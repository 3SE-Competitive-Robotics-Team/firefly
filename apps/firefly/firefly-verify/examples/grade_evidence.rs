//! Entry point for the synthetic fault-injection tests.
use std::io;

/// Grade JSON evidence from standard input.
#[derive(usage_rs::Cli)]
#[usage(bin = "grade_evidence")]
struct Arguments {}

fn main() -> std::process::ExitCode {
    firefly_error::report("grade_evidence", run_app())
}

fn run_app() -> firefly_error::Result<()> {
    Arguments::parse();
    let (data, mode): (firefly_verify::Evidence, Option<String>) =
        serde_json::from_reader(io::stdin().lock()).map_err(|e| {
            firefly_error::Error::caused_by(
                firefly_error::ErrorKind::InvalidData,
                "read JSON evidence from stdin",
                e,
            )
        })?;
    println!("{}", firefly_verify::evaluate(&data, mode.as_deref()));
    Ok(())
}
