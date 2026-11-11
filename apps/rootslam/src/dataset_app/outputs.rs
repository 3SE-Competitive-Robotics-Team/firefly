//! Each dataset output owns a new file or a dedicated directory tree.
use super::{Options, Source};
use firefly_error::{Error, ErrorKind, Result};
use std::path::{Component, Path, PathBuf};

struct Target<'a> {
    name: &'static str,
    path: &'a Path,
    directory: bool,
}

fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam.dataset.outputs")
}

fn io_error(source: std::io::Error) -> Error {
    Error::from(source).with_operation("rootslam.dataset.outputs")
}

/// Resolves existing symlinks as components are visited, then normalizes the
/// remaining suffix. In particular, `link/..` follows the link before ascending.
fn resolve(path: &Path) -> Result<PathBuf> {
    let mut resolved = if path.is_absolute() {
        PathBuf::new()
    } else {
        std::env::current_dir().map_err(io_error)?
    };
    for component in path.components() {
        if resolved.is_file() {
            return Err(invalid("Dataset output ancestors must be directories")
                .with_context("path", path.display()));
        }
        match component {
            Component::Prefix(prefix) => resolved.push(prefix.as_os_str()),
            Component::RootDir => resolved.push(component.as_os_str()),
            Component::CurDir => {}
            Component::ParentDir => {
                resolved.pop();
            }
            Component::Normal(name) => {
                resolved.push(name);
                match std::fs::symlink_metadata(&resolved) {
                    Ok(_) => {
                        resolved = resolved.canonicalize().map_err(io_error)?;
                    }
                    Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                    Err(error) => return Err(io_error(error)),
                }
            }
        }
    }
    Ok(resolved)
}

pub(super) fn validate(
    options: &Options,
    source: &Source,
    vio_config: Option<&Path>,
) -> Result<()> {
    let files = [
        ("trajectory", &options.trajectory),
        ("local_trajectory", &options.local_trajectory),
        ("groundtruth", &options.groundtruth),
        ("result_path", &options.result_path),
        ("metrics_path", &options.metrics_path),
        ("global_result_path", &options.global_result_path),
        (
            "optimization.problem_archive",
            &options.optimization.problem_archive,
        ),
        (
            "optimization.linear_system_archive",
            &options.optimization.linear_system_archive,
        ),
    ];
    let directories = [
        ("marg_data", &options.marg_data),
        ("debug_directory", &options.debug_directory),
    ];
    let targets: Vec<_> = files
        .into_iter()
        .map(|(name, path)| (name, path, false))
        .chain(
            directories
                .into_iter()
                .map(|(name, path)| (name, path, true)),
        )
        .filter_map(|(name, path, directory)| {
            path.as_deref().map(|path| Target {
                name,
                path,
                directory,
            })
        })
        .collect();
    let mut protected = Vec::new();
    if let Source::Dataset {
        path, calibration, ..
    } = source
    {
        protected.push((resolve(path)?, true));
        protected.push((resolve(calibration)?, calibration.is_dir()));
    }
    if let Some(path) = vio_config {
        protected.push((resolve(path)?, false));
    }
    validate_targets(&targets, &protected)
}

fn validate_targets(targets: &[Target<'_>], protected: &[(PathBuf, bool)]) -> Result<()> {
    let mut resolved_targets: Vec<(&Target<'_>, PathBuf)> = Vec::new();
    for target in targets {
        let path = resolve(target.path)?;
        for (input, directory) in protected {
            if path == *input || (*directory && path.starts_with(input)) {
                return Err(
                    invalid("Dataset outputs require paths separate from their inputs")
                        .with_context("output", target.name)
                        .with_context("path", target.path.display()),
                );
            }
        }
        match std::fs::symlink_metadata(target.path) {
            Ok(_) => {
                return Err(invalid("Dataset output path already exists")
                    .with_context("output", target.name)
                    .with_context("path", target.path.display()));
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(io_error(error)),
        }
        if !target.directory || target.name == "marg_data" {
            let parent = target
                .path
                .parent()
                .filter(|p| !p.as_os_str().is_empty())
                .unwrap_or(Path::new("."));
            if !parent.is_dir() {
                return Err(invalid("Dataset output parent directory must exist")
                    .with_context("output", target.name)
                    .with_context("path", parent.display()));
            }
        }
        for (other, other_path) in &resolved_targets {
            if path == *other_path || path.starts_with(other_path) || other_path.starts_with(&path)
            {
                return Err(invalid(
                    "Dataset output paths must own separate files or directory trees",
                )
                .with_context("first", other.name)
                .with_context("second", target.name)
                .with_context("path", target.path.display()));
            }
        }
        resolved_targets.push((target, path));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct Directory(PathBuf);
    impl Directory {
        fn new() -> Self {
            static NEXT: AtomicUsize = AtomicUsize::new(0);
            let stamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let path = Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../logs")
                .join(format!(
                    "test_rootslam_outputs_{}_{stamp}_{}",
                    std::process::id(),
                    NEXT.fetch_add(1, Ordering::Relaxed)
                ));
            std::fs::create_dir_all(&path).unwrap();
            Self(path)
        }
    }
    impl Drop for Directory {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.0).unwrap();
        }
    }

    #[test]
    fn final_trajectory_preserves_existing_evidence() {
        let directory = Directory::new();
        let path = directory.0.join("trajectory.txt");
        std::fs::write(&path, "existing trajectory").unwrap();
        assert!(
            super::super::save_trajectory(&path, &[], super::super::TrajectoryFormat::Tum).is_err()
        );
        assert_eq!(
            std::fs::read_to_string(path).unwrap(),
            "existing trajectory"
        );
    }

    #[test]
    fn output_preflight_precedes_loading_dataset_measurements() {
        let directory = Directory::new();
        let path = directory.0.join("trajectory.txt");
        std::fs::write(&path, "prior evidence").unwrap();
        let options = Options {
            trajectory: Some(path.clone()),
            ..Options::default()
        };
        let source = Source::Dataset {
            format: crate::input::Format::Euroc,
            path: directory.0.join("unavailable-input"),
            calibration: directory.0.join("calibration.json"),
        };
        let failure = super::super::run(
            &options,
            &source,
            &crate::input::SensorModel {
                calibration: firefly_calibration::calibration::Calibration::default(),
                gravity: nalgebra::Vector3::new(0., 0., -9.81),
            },
            &crate::config::Options::default(),
            super::super::Precision::Float64,
            &std::sync::atomic::AtomicBool::new(false),
        )
        .unwrap_err();
        assert!(failure.to_string().contains("output path already exists"));
        assert_eq!(std::fs::read_to_string(path).unwrap(), "prior evidence");
    }

    #[test]
    fn validates_all_output_roles_and_lexical_aliases() {
        let directory = Directory::new();
        let mut options = Options {
            trajectory: Some(directory.0.join("trajectory.txt")),
            metrics_path: Some(directory.0.join("./trajectory.txt")),
            ..Options::default()
        };
        assert!(validate(&options, &Source::Ipc, None).is_err());
        options.metrics_path = Some(directory.0.join("metrics.json"));
        assert!(validate(&options, &Source::Ipc, None).is_ok());
        options.optimization.problem_archive = options.metrics_path.clone();
        assert!(validate(&options, &Source::Ipc, None).is_err());
        options.optimization.problem_archive = None;
        std::fs::write(options.metrics_path.as_ref().unwrap(), "prior metrics").unwrap();
        assert!(validate(&options, &Source::Ipc, None).is_err());
    }

    #[test]
    fn protects_dataset_calibration_and_output_directory_ownership() {
        let directory = Directory::new();
        let dataset = directory.0.join("input");
        std::fs::create_dir(&dataset).unwrap();
        let calibration = directory.0.join("calibration.json");
        std::fs::write(&calibration, "calibration").unwrap();
        let source = Source::Dataset {
            format: crate::input::Format::Euroc,
            path: dataset.clone(),
            calibration: calibration.clone(),
        };
        let mut options = Options {
            trajectory: Some(dataset.join("poses.txt")),
            ..Options::default()
        };
        assert!(validate(&options, &source, None).is_err());
        options.trajectory = Some(calibration.clone());
        assert!(validate(&options, &source, None).is_err());
        assert_eq!(std::fs::read_to_string(calibration).unwrap(), "calibration");
        options.trajectory = Some(directory.0.join("stats"));
        options.debug_directory = Some(directory.0.join("stats/nested"));
        assert!(validate(&options, &source, None).is_err());
        options.trajectory = None;
        options.debug_directory = Some(directory.0.join("stats"));
        options.marg_data = Some(directory.0.join("stats"));
        assert!(validate(&options, &source, None).is_err());
    }

    #[cfg(unix)]
    #[test]
    fn resolves_parent_symlinks_before_comparing_outputs() {
        let directory = Directory::new();
        let real = directory.0.join("real");
        let alias = directory.0.join("alias");
        std::fs::create_dir(&real).unwrap();
        std::os::unix::fs::symlink(&real, &alias).unwrap();
        let mut options = Options {
            trajectory: Some(real.join("poses.txt")),
            metrics_path: Some(alias.join("poses.txt")),
            ..Options::default()
        };
        assert!(validate(&options, &Source::Ipc, None).is_err());
        options.metrics_path = Some(alias.join("../real/poses.txt"));
        assert!(validate(&options, &Source::Ipc, None).is_err());
    }
}
