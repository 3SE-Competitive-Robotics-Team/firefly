use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam::{config::Options, dataset_app, input::Source, online};
use firefly_rootslam_mapping::global_optimization::Precision;
use serde::Deserialize;
use std::{
    io::Write,
    path::PathBuf,
    sync::{Arc, atomic::AtomicBool},
};

const USAGE: &str = "firefly-rootslam [--config <path>] [--no-loop-closures] [--no-mapping] [--external-localization]";

#[derive(Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct Config {
    /// Arithmetic for local BA, IMU preintegration, marginalization and global BA.
    precision: Precision,
    input: Source,
    vio: Options,
    dataset: Option<dataset_app::Options>,
}

impl Config {
    fn validate(&self) -> Result<()> {
        self.vio.validate()?;
        match self.input {
            Source::Ipc if self.dataset.is_some() => {
                return Err(argument_error("dataset options require dataset input"));
            }
            Source::Dataset { .. } => {
                if self.vio.external_localization {
                    return Err(argument_error("external localization requires IPC input"));
                }
                if self.vio.stats_directory.is_some() {
                    return Err(argument_error(
                        "dataset statistics use dataset.debug_directory",
                    ));
                }
            }
            Source::Ipc => {}
        }
        if let Some(dataset) = &self.dataset {
            dataset.validate(self.vio.map)?;
        }
        Ok(())
    }
}

/// Online mapping and navigation ownership switches.
struct Arguments {
    config: PathBuf,
    loop_closures: bool,
    mapping: bool,
    external_localization: bool,
}

fn argument_error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam::arguments")
}

fn parse() -> Result<Option<Arguments>> {
    let mut args = std::env::args_os();
    let _executable = args.next().unwrap_or_else(|| "firefly-rootslam".into());
    let mut config = firefly_config::paths::config("firefly-rootslam.toml");
    let mut loop_closures = true;
    let mut mapping = true;
    let mut external_localization = false;
    while let Some(argument) = args.next() {
        if argument == "--help" || argument == "-h" {
            return Ok(None);
        }
        if argument == "--no-loop-closures" {
            loop_closures = false;
            continue;
        }
        if argument == "--no-mapping" {
            mapping = false;
            continue;
        }
        if argument == "--config" {
            config = args
                .next()
                .ok_or_else(|| argument_error("missing --config value"))?
                .into();
            continue;
        }
        if argument == "--external-localization" {
            external_localization = true;
            continue;
        }
        return Err(argument_error(format!(
            "unknown argument: {}",
            argument.to_string_lossy()
        )));
    }
    Ok(Some(Arguments {
        config,
        loop_closures,
        mapping,
        external_localization,
    }))
}

fn main() -> Result<()> {
    let Some(arguments) = parse()? else {
        let mut stdout = std::io::stdout();
        writeln!(stdout, "{USAGE}")
            .map_err(|error| Error::new(ErrorKind::Internal, "write help").with_source(error))?;
        return Ok(());
    };
    let mut config: Config = firefly_config::startup(
        &arguments.config,
        firefly_observability::init,
        |config: &Config| config.vio.validate(),
    )?;
    if !arguments.loop_closures {
        config.vio.loops = false;
    }
    if !arguments.mapping {
        config.vio.map = false;
    }
    config.vio.external_localization |= arguments.external_localization;
    config.validate()?;
    let model = config.input.model()?;
    config.vio.validate_model(&model)?;
    let stop = Arc::new(AtomicBool::new(false));
    let mut registrations = Vec::new();
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        registrations.push(
            signal_hook::flag::register(signal, Arc::clone(&stop))
                .map_err(|e| Error::from(e).with_operation("rootslam.live.signal"))?,
        );
    }
    let node = firefly_pubsub::node::create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "rootslam");
    let result = std::thread::scope(|scope| {
        let worker = scope.spawn(|| match &config.input {
            Source::Ipc => online::run(&config.vio, &model, config.precision, &stop),
            Source::Dataset { .. } => dataset_app::run(
                config
                    .dataset
                    .as_ref()
                    .unwrap_or(&dataset_app::Options::default()),
                &config.input,
                &model,
                &config.vio,
                config.precision,
                &stop,
            ),
        });
        while !worker.is_finished() {
            firefly_observability::pump_log_ipc(&log_ipc);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let result = worker
            .join()
            .map_err(|_| {
                Error::new(ErrorKind::Internal, "VIO worker panicked")
                    .with_operation("rootslam.live.worker")
            })
            .and_then(|r| r);
        firefly_observability::pump_log_ipc(&log_ipc);
        if let Err(error) = &result {
            let _ = writeln!(
                std::io::stderr(),
                "RootSLAM pipeline stopped with an error: {error:?}"
            );
        }
        result
    });
    for registration in registrations {
        signal_hook::low_level::unregister(registration);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::Config;

    #[test]
    fn precision_has_one_configuration_owner() {
        for precision in ["float32", "float64"] {
            let config: Config = toml::from_str(&format!("precision = '{precision}'")).unwrap();
            assert!(config.validate().is_ok());
        }
        for source in [
            "precision = 'float16'",
            "[vio]\nuse_double = true",
            "[vio]\nprecision = 'float32'",
            "[dataset.optimization]\nprecision = 'float32'",
            "[dataset.optimization]\nkrylov_precision = 'native'",
            "[dataset.optimization]\nunknown = 1",
        ] {
            assert!(toml::from_str::<Config>(source).is_err(), "{source}");
        }
    }

    #[test]
    fn source_specific_options_cannot_be_silently_ignored() {
        let ipc: Config = toml::from_str("[dataset]\ntrajectory = 'poses.txt'").unwrap();
        assert!(ipc.validate().is_err());
        let mut dataset: Config = toml::from_str(include_str!(
            "../../../../configs/firefly-rootslam-dataset.toml"
        ))
        .unwrap();
        assert!(dataset.validate().is_ok());
        dataset.vio.external_localization = true;
        assert!(dataset.validate().is_err());
        dataset.vio.external_localization = false;
        dataset.vio.stats_directory = Some("stats".into());
        assert!(dataset.validate().is_err());
    }

    #[test]
    fn default_and_shipped_config_use_vio_only() {
        let shipped: Config =
            toml::from_str(include_str!("../../../../configs/firefly-rootslam.toml")).unwrap();
        for config in [Config::default(), shipped] {
            assert!(!config.vio.map && !config.vio.loops && !config.vio.external_localization);
            assert_eq!(config.precision, super::Precision::Float64);
        }
    }

    #[test]
    fn dataset_examples_enable_the_complete_mapping_pipeline() {
        for source in [
            include_str!("../../../../configs/firefly-rootslam-dataset.toml"),
            include_str!("../../../../configs/firefly-rootslam-uzh.toml"),
            include_str!("../../../../configs/firefly-rootslam-euroc.toml"),
            include_str!("../../../../configs/firefly-rootslam-tumvi.toml"),
        ] {
            let config: Config = toml::from_str(source).unwrap();
            assert!(config.vio.map && config.vio.loops);
            assert_eq!(config.precision, super::Precision::Float64);
            assert!(config.validate().is_ok());
        }
    }

    #[test]
    fn missing_empty_and_partial_configs_share_online_defaults() {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../logs")
            .join(format!(
                "test_rootslam_config_{}_{stamp}",
                std::process::id()
            ));
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join("online.toml");
        let missing: Config = firefly_config::load(&path).unwrap();
        std::fs::write(&path, "").unwrap();
        let empty: Config = firefly_config::load(&path).unwrap();
        let partial: Config = toml::from_str("precision = 'float32'").unwrap();
        assert_eq!(missing.vio.map, empty.vio.map);
        assert_eq!(missing.vio.loops, empty.vio.loops);
        assert_eq!(missing.precision, empty.precision);
        assert_eq!(missing.precision, super::Precision::Float64);
        assert_eq!(missing.vio.map, partial.vio.map);
        assert_eq!(partial.precision, super::Precision::Float32);
        assert!(missing.vio.validate().is_ok());
        assert!(empty.vio.validate().is_ok());
        assert!(toml::from_str::<Config>("onlne = {}").is_err());
        std::fs::remove_dir_all(directory).unwrap();
    }
}
