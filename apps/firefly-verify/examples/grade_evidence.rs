//! Entry point for the synthetic fault-injection tests.
use std::io;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (data, mode): (firefly_verify::Evidence, Option<String>) =
        serde_json::from_reader(io::stdin().lock())?;
    println!("{}", firefly_verify::evaluate(&data, mode.as_deref()));
    Ok(())
}
