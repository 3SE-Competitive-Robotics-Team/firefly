use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam::{config::Options, dataset_app, input::Source, online};
use firefly_rootslam_mapping::global_optimization::Precision;
use serde::Deserialize;
use std::{
    io::Write,
    path::PathBuf,
    sync::{Arc, atomic::AtomicBool},
};

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

/// Run visual-inertial estimation online or on a dataset.
#[derive(usage_rs::Cli)]
#[usage(bin = "firefly-rootslam")]
struct Arguments {
    /// Application configuration file.
    #[usage(long)]
    config: Option<PathBuf>,
    /// Disable loop closures.
    #[usage(long)]
    no_loop_closures: bool,
    /// Disable mapping.
    #[usage(long)]
    no_mapping: bool,
    /// Receive external localization.
    #[usage(long)]
    external_localization: bool,
}

fn argument_error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam::arguments")
}

fn main() -> Result<()> {
    let arguments = Arguments::parse();
    let mut config: Config = firefly_config::startup(
        arguments
            .config
            .unwrap_or_else(|| firefly_config::paths::config("firefly-rootslam.toml")),
        firefly_observability::init,
        |config: &Config| config.vio.validate(),
    )?;
    if arguments.no_loop_closures {
        config.vio.loops = false;
    }
    if arguments.no_mapping {
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
    fn online_configs_preserve_vio_defaults() {
        let shipped: Config =
            toml::from_str(include_str!("../../../../configs/firefly-rootslam.toml")).unwrap();
        let empty: Config = toml::from_str("").unwrap();
        for config in [Config::default(), empty, shipped] {
            assert!(!config.vio.map && !config.vio.loops && !config.vio.external_localization);
            assert_eq!(config.precision, super::Precision::Float64);
            assert!(config.validate().is_ok());
        }
        let partial: Config = toml::from_str("precision = 'float32'").unwrap();
        assert!(!partial.vio.map && !partial.vio.loops && !partial.vio.external_localization);
        assert_eq!(partial.precision, super::Precision::Float32);
        assert!(partial.validate().is_ok());
        assert!(toml::from_str::<Config>("onlne = {}").is_err());
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
}
