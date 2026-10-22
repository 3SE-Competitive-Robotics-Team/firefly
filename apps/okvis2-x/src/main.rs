// SPDX-License-Identifier: BSD-3-Clause

//! Online IPC and offline dataset entry points for OKVIS2-X.

use firefly_error::{Error, ErrorKind, Result};
use firefly_okvis_x::application::{OfflineApplicationFactory, okvis2x_app_synchronous};
use std::{ffi::OsString, path::PathBuf, process::ExitCode};

const USAGE: &str = "firefly-okvis-x [--config configs/okvis2-x.toml] [--dataset <directory>] [--output <directory>] [--vocabulary <directory>]";

struct Arguments {
    executable: OsString,
    config: PathBuf,
    dataset: Option<PathBuf>,
    output: Option<PathBuf>,
    vocabulary: Option<PathBuf>,
}

fn parse() -> Result<Option<Arguments>> {
    let mut args = std::env::args_os();
    let executable = args.next().unwrap_or_else(|| "firefly-okvis-x".into());
    let mut config = PathBuf::from("configs/okvis2-x.toml");
    let mut vocabulary = None;
    let mut dataset = None;
    let mut output = None;
    while let Some(argument) = args.next() {
        if argument == "--help" || argument == "-h" {
            return Ok(None);
        }
        let target = if argument == "--config" {
            &mut config
        } else if argument == "--vocabulary" {
            vocabulary =
                Some(PathBuf::from(args.next().ok_or_else(|| {
                    argument_error("missing --vocabulary value")
                })?));
            continue;
        } else if argument == "--dataset" {
            dataset = Some(PathBuf::from(
                args.next()
                    .ok_or_else(|| argument_error("missing --dataset value"))?,
            ));
            continue;
        } else if argument == "--output" {
            output = Some(PathBuf::from(
                args.next()
                    .ok_or_else(|| argument_error("missing --output value"))?,
            ));
            continue;
        } else {
            return Err(argument_error(format!(
                "unknown argument: {}",
                argument.to_string_lossy()
            )));
        };
        *target = args
            .next()
            .ok_or_else(|| argument_error(format!("missing {} value", argument.to_string_lossy())))?
            .into();
    }
    if dataset.is_none() && output.is_some() {
        return Err(argument_error("--output requires --dataset"));
    }
    Ok(Some(Arguments {
        executable,
        config,
        dataset,
        output,
        vocabulary,
    }))
}

fn argument_error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("okvis2_x::arguments")
}

fn run() -> Result<()> {
    let Some(arguments) = parse()? else {
        use std::io::Write;
        writeln!(std::io::stdout(), "{USAGE}")
            .map_err(|error| Error::new(ErrorKind::Internal, "write help").with_source(error))?;
        return Ok(());
    };
    let Some(dataset) = arguments.dataset else {
        let config = firefly_config::startup(
            &arguments.config,
            firefly_observability::init,
            firefly_okvis_x::application::live::config::Config::validate,
        )?;
        return firefly_okvis_x::application::live::run(config, arguments.vocabulary.as_deref());
    };
    let mut factory = OfflineApplicationFactory::load(&arguments.config)?;
    let mut source_arguments = vec![
        arguments.executable,
        arguments.config.clone().into_os_string(),
        arguments.config.into_os_string(),
        dataset.into_os_string(),
    ];
    if let Some(output) = arguments.output {
        source_arguments.push(output.into_os_string());
    }
    okvis2x_app_synchronous::run_with_vocabulary(
        &source_arguments,
        &mut factory,
        Some(
            arguments
                .vocabulary
                .as_deref()
                .unwrap_or(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("resources")),
        ),
    )
    .map_err(|error| Error::new(ErrorKind::Internal, "offline OKVIS2-X failed").with_source(error))
}

fn main() -> ExitCode {
    let result = run();
    firefly_observability::flush();
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}\n{USAGE}");
            ExitCode::FAILURE
        }
    }
}
