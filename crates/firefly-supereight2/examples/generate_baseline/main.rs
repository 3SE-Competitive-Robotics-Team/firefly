// SPDX-License-Identifier: BSD-3-Clause
//! Explicitly regenerate Rust regression baselines using only the checked-in scene inputs.
mod maps;
mod transforms;
use std::{error::Error, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn main() -> Result<()> {
    let mut inputs = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/reference");
    let mut output = None;
    let mut args = std::env::args().skip(1);
    while let Some(option) = args.next() {
        let path = args.next().ok_or("expected a directory after the option")?;
        match option.as_str() {
            "--inputs-dir" => inputs = PathBuf::from(path),
            "--output-dir" => output = Some(PathBuf::from(path)),
            _ => return Err(format!("unknown option: {option}").into()),
        }
    }
    let output = output.ok_or("usage: generate_baseline --output-dir DIR [--inputs-dir DIR]")?;
    // Read all inputs before writing so explicit in-place regeneration remains well-defined.
    let transforms = transforms::generate(&std::fs::read_to_string(
        inputs.join("general_transforms.txt"),
    )?)?;
    let rotated = maps::generate(
        &std::fs::read_to_string(inputs.join("rotated_map.txt"))?,
        false,
    )?;
    let general = maps::generate(
        &std::fs::read_to_string(inputs.join("general_axis_map.txt"))?,
        true,
    )?;
    std::fs::create_dir_all(&output)?;
    for (name, text) in [
        ("general_transforms.txt", transforms),
        ("rotated_map.txt", rotated),
        ("general_axis_map.txt", general),
    ] {
        std::fs::write(output.join(name), text)?;
    }
    Ok(())
}
