//! Entry point for the synthetic fault-injection tests.
use std::io;

/// Grade JSON evidence from standard input.
#[derive(usage_rs::Cli)]
#[usage(bin = "grade_evidence")]
struct Arguments {}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    Arguments::parse();
    let (data, mode): (firefly_verify::Evidence, Option<String>) =
        serde_json::from_reader(io::stdin().lock())?;
    println!("{}", firefly_verify::evaluate(&data, mode.as_deref()));
    Ok(())
}
