/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Network synchronous application: processor selection, image mosaics and shutdown.
use super::okvis2x_app_synchronous::{
    self as app, ApplicationFactory, Arguments, EstimatorHandle, MappingHandle,
};
use crate::{
    config::parameters::ViParameters,
    datasets::x_dataset_reader::XDatasetReader,
    deep_learning::deep_learning_processor::{DeepLearningProcessor, DisplayImages},
    output::trajectory_output::TrajectoryOutput,
    sensors::vi_sensor_base::{DatasetReaderBase, ViSensorBase},
};
use purecv::core::{DynamicData, DynamicMatrix, Matrix};
use std::{
    collections::BTreeMap,
    ffi::OsString,
    io,
    path::Path,
    sync::{Arc, Mutex},
    time::Instant,
};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetworkKind {
    Stereo,
    DepthFusion,
    Language,
}
pub trait NetworkApplicationFactory: ApplicationFactory {
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn create_processor(
        &mut self,
        kind: NetworkKind,
        parameters: &ViParameters,
        vocabulary_directory: &Path,
    ) -> io::Result<Arc<dyn DeepLearningProcessor>>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn shutdown_processor(&mut self, processor: &Arc<dyn DeepLearningProcessor>) -> io::Result<()>;
}
fn error(s: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, s)
}
fn gray_to_colour(m: &DynamicMatrix) -> io::Result<DynamicMatrix> {
    let DynamicData::U8(d) = &m.data else {
        return Err(error("gray image must be u8"));
    };
    if d.channels != 1 {
        return Err(error("gray image must have one channel"));
    }
    Ok(DynamicMatrix {
        data: DynamicData::U8(Matrix::from_vec(
            d.rows,
            d.cols,
            3,
            d.data.iter().flat_map(|v| [*v, *v, *v]).collect(),
        )),
    })
}
fn concatenate(images: &[&DynamicMatrix], horizontal: bool) -> io::Result<DynamicMatrix> {
    let first = images.first().ok_or_else(|| error("empty image mosaic"))?;
    macro_rules! c {
        ($v:ident,$d:expr) => {{
            let d = $d;
            let mut mats = Vec::new();
            for im in images {
                let DynamicData::$v(m) = &im.data else {
                    return Err(error("mosaic pixel formats disagree"));
                };
                if m.channels != d.channels
                    || (horizontal && m.rows != d.rows)
                    || (!horizontal && m.cols != d.cols)
                {
                    return Err(error("mosaic dimensions disagree"));
                }
                mats.push(m);
            }
            let rows = if horizontal {
                d.rows
            } else {
                mats.iter().map(|m| m.rows).sum()
            };
            let cols = if horizontal {
                mats.iter().map(|m| m.cols).sum()
            } else {
                d.cols
            };
            let mut data = Vec::with_capacity(rows * cols * d.channels);
            if horizontal {
                for y in 0..rows {
                    for m in &mats {
                        data.extend_from_slice(
                            &m.data[y * m.cols * m.channels..(y + 1) * m.cols * m.channels],
                        );
                    }
                }
            } else {
                for m in &mats {
                    data.extend_from_slice(&m.data);
                }
            }
            DynamicMatrix {
                data: DynamicData::$v(Matrix::from_vec(rows, cols, d.channels, data)),
            }
        }};
    }
    Ok(match &first.data {
        DynamicData::U8(d) => c!(U8, d),
        DynamicData::I8(d) => c!(I8, d),
        DynamicData::U16(d) => c!(U16, d),
        DynamicData::I16(d) => c!(I16, d),
        DynamicData::I32(d) => c!(I32, d),
        DynamicData::F32(d) => c!(F32, d),
        DynamicData::F64(d) => c!(F64, d),
    })
}
fn get<'a>(images: &'a DisplayImages, name: &str) -> io::Result<&'a DynamicMatrix> {
    images.get(name).map(AsRef::as_ref).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("network display image {name} missing"),
        )
    })
}
/// # Errors
/// Returns an error if command-line/configuration input is invalid, an input/output file
/// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
pub fn display_processor(
    factory: &mut impl ApplicationFactory,
    kind: NetworkKind,
    estimator: &EstimatorHandle,
    processor: &Arc<dyn DeepLearningProcessor>,
) -> io::Result<()> {
    let mut images = DisplayImages::new();
    processor
        .display(&mut images)
        .map_err(|e| io::Error::other(e.to_string()))?;
    match kind {
        NetworkKind::Stereo => {
            if images.get("leftImage").is_some_and(|i| i.rows() > 0)
                && images.get("rightImage").is_some_and(|i| i.rows() > 0)
            {
                let left = gray_to_colour(get(&images, "leftImage")?)?;
                let right = gray_to_colour(get(&images, "rightImage")?)?;
                let stereo = concatenate(&[&left, &right], true)?;
                let mosaic = if let (Some(d), Some(s)) =
                    (images.get("stereoDepth"), images.get("stereoSigma"))
                {
                    let depth = concatenate(&[d, s], true)?;
                    concatenate(&[&stereo, &depth], false)?
                } else {
                    stereo
                };
                factory.show_image("Left & Right images; Depth & Sigma images", &mosaic)?;
            }
        }
        NetworkKind::DepthFusion => {
            let mut debug = BTreeMap::new();
            estimator
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .display(&mut debug);
            for (name, image) in debug {
                factory.show_image(&name, &image)?;
            }
            if images.get("srcImage0").is_some_and(|i| i.rows() > 0) {
                let rgb = (0..7)
                    .map(|i| gray_to_colour(get(&images, &format!("srcImage{i}"))?))
                    .collect::<io::Result<Vec<_>>>()?;
                let a = concatenate(&[&rgb[0], &rgb[1], &rgb[2], &rgb[3]], true)?;
                let b = concatenate(
                    &[&rgb[4], &rgb[5], &rgb[6], get(&images, "sparseDepth")?],
                    true,
                )?;
                factory.show_image("Input images", &concatenate(&[&a, &b], false)?)?;
            }
            if ["stereoDepth", "mvsDepth", "fuseDepth"]
                .iter()
                .all(|k| images.contains_key(*k))
            {
                let depth = concatenate(
                    &[
                        get(&images, "stereoDepth")?,
                        get(&images, "mvsDepth")?,
                        get(&images, "fuseDepth")?,
                    ],
                    true,
                )?;
                let sigma = concatenate(
                    &[
                        get(&images, "stereoSigma")?,
                        get(&images, "mvsSigma")?,
                        get(&images, "fuseSigma")?,
                    ],
                    true,
                )?;
                factory.show_image(
                    "Depth stereo/mvs/fuse; Inverse depth sigma stereo/mvs/fuse",
                    &concatenate(&[&depth, &sigma], false)?,
                )?;
            }
        }
        NetworkKind::Language => {
            let display = images
                .iter()
                .filter(|(n, _)| n.as_str() != "language_rgb")
                .map(|(_, i)| i.as_ref())
                .collect::<Vec<_>>();
            if !display.is_empty() {
                factory.show_image("RGB images display", &concatenate(&display, true)?)?;
            }
            if let Some(i) = images.get("language_rgb") {
                factory.show_image("Language activations", i)?;
            }
        }
    }
    Ok(())
}
/// # Errors
/// Returns an error if command-line/configuration input is invalid, an input/output file
/// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
pub fn run(
    args: &[OsString],
    kind: NetworkKind,
    factory: &mut impl NetworkApplicationFactory,
) -> io::Result<()> {
    let arguments = Arguments::parse(args)?;
    factory.init_logging(&arguments.executable)?;
    app::omp_warning();
    let mut parameters = factory.read_vi_parameters(&arguments.vi_config)?;
    let (mut config, map_config, data_config) =
        factory.read_mapping_config(&arguments.mapping_config)?;
    if !parameters.output.enable_submapping {
        log::warn!("Network app requires submapping; enabling submapping");
        parameters.output.enable_submapping = true;
    }
    let rgb = (0..parameters.n_camera_system.num_cameras())
        .any(|index| parameters.n_camera_system.camera_type(index).is_colour);
    let codec = factory.image_codec();
    let mut reader = XDatasetReader::new(
        &arguments.dataset,
        0.0,
        &parameters,
        false,
        (
            true,
            kind == NetworkKind::Language,
            kind == NetworkKind::Language || rgb,
        ),
        codec.clone(),
    )?;
    let voc = app::vocabulary_directory(&arguments.executable)?;
    let save = arguments
        .output
        .as_ref()
        .unwrap_or(&arguments.dataset)
        .clone();
    if arguments.output.is_some() {
        config.results_directory = save.to_string_lossy().into_owned();
    }
    let estimator = Arc::new(Mutex::new(factory.create_estimator(
        &parameters,
        &voc,
        &config,
    )?));
    let mut mapping = factory.create_mapping(map_config, data_config, &config, &parameters)?;
    configure_mapping(mapping.as_mut(), &estimator, &parameters, &config);
    let mapping: MappingHandle = Arc::new(Mutex::new(mapping));
    let processor = factory.create_processor(kind, &parameters, &voc)?;
    processor.set_blocking(true);
    let mode = app::output_mode(&parameters);
    app::configure_estimator_output(&estimator, &save, &mode);
    let (writer, failure) = connect_graph_output(
        kind,
        &estimator,
        &mapping,
        &processor,
        (&save, &mode),
        parameters.output.display.display_topview,
    )?;
    let estimator_inputs = estimator
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .input_callbacks();
    let mapping_inputs = mapping
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .input_callbacks();
    app::connect_reader(&mut reader, &estimator_inputs, &parameters);
    connect_processor(
        &mut reader,
        kind,
        &processor,
        estimator_inputs,
        mapping_inputs,
        &failure,
    );
    let result = process_dataset(
        &mut reader,
        factory,
        kind,
        &app::RunOutput {
            estimator: &estimator,
            mapping: Some(&mapping),
            processor: Some(&processor),
            writer: &writer,
            parameters: &parameters,
            save: &save,
            mode: &mode,
            codec: codec.as_ref(),
            failure: &failure,
        },
    );
    shutdown_dataset(
        &mut reader,
        factory,
        (&estimator, &mapping, &processor),
        &failure,
        result,
    )
}
fn connect_graph_output(
    kind: NetworkKind,
    estimator: &app::EstimatorHandle,
    mapping: &MappingHandle,
    processor: &Arc<dyn DeepLearningProcessor>,
    (save, mode): (&std::path::Path, &str),
    topview: bool,
) -> io::Result<(Arc<Mutex<TrajectoryOutput>>, app::Failure)> {
    let writer = Arc::new(Mutex::new(TrajectoryOutput::with_csv(
        &save.join(format!("okvis2-{mode}_trajectory.csv")),
        false,
        topview,
    )?));
    let failure = Arc::new(Mutex::new(None));
    app::set_graph_callback(
        estimator,
        Some(
            mapping
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .input_callbacks(),
        ),
        writer.clone(),
        if kind == NetworkKind::DepthFusion {
            Some(processor.clone())
        } else {
            None
        },
        failure.clone(),
    );
    Ok((writer, failure))
}

fn configure_mapping(
    mapping: &mut dyn app::MappingApplication,
    estimator: &app::EstimatorHandle,
    parameters: &crate::config::parameters::ViParameters,
    config: &crate::mapping::config_mapping::SubMapConfig,
) {
    mapping.set_t_bs(parameters.imu.t_bs.clone());
    mapping.set_blocking(true);
    if config.factors.use_map2map_factors || config.factors.use_map2live_factors {
        mapping.set_align_callback(
            estimator
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .alignment_callback(),
        );
    }
}

fn shutdown_dataset(
    reader: &mut XDatasetReader,
    factory: &mut impl NetworkApplicationFactory,
    (estimator, mapping, processor): (
        &app::EstimatorHandle,
        &MappingHandle,
        &Arc<dyn DeepLearningProcessor>,
    ),
    failure: &app::Failure,
    result: io::Result<()>,
) -> io::Result<()> {
    let reader_stop = reader.stop_streaming();
    let processor_stop = factory.shutdown_processor(processor);
    let map_stop = mapping
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .set_finished();
    let estimator_stop = estimator
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .shutdown();
    result?;
    reader_stop?;
    processor_stop?;
    map_stop?;
    estimator_stop?;
    app::check_failure(failure)
}

fn connect_processor(
    reader: &mut XDatasetReader,
    kind: NetworkKind,
    processor: &Arc<dyn DeepLearningProcessor>,
    estimator_inputs: app::EstimatorInputs,
    mapping_inputs: app::MappingInputs,
    failure: &app::Failure,
) {
    let (network_processor, callback_failure) = (processor.clone(), failure.clone());
    reader.set_images_network_callback(Arc::new(move |im, depth| {
        app::latch(
            &callback_failure,
            network_processor
                .add_images(im, depth)
                .map_err(|estimator_callbacks| io::Error::other(estimator_callbacks.to_string())),
        )
    }));
    if kind == NetworkKind::DepthFusion {
        let estimator_callbacks = estimator_inputs.clone();
        processor.set_live_depth_image_callback(Arc::new(move |timestamp, depth_value, state| {
            Ok((estimator_callbacks.depth)(timestamp, depth_value, state))
        }));
        let (map_inputs, callback_failure) = (mapping_inputs.clone(), failure.clone());
        processor.set_image_callback(Arc::new(move |frames| {
            Ok(app::latch(&callback_failure, (map_inputs.depth)(frames)))
        }));
    } else {
        let cb = app::depth_callback(estimator_inputs, mapping_inputs, failure.clone());
        processor.set_image_callback(Arc::new(move |frames| Ok(cb(frames))));
    }
}

fn process_dataset(
    reader: &mut XDatasetReader,
    factory: &mut impl NetworkApplicationFactory,
    kind: NetworkKind,
    output: &app::RunOutput<'_>,
) -> io::Result<()> {
    let app::RunOutput {
        estimator,
        mapping,
        processor,
        writer,
        parameters,
        save,
        mode,
        codec,
        failure,
    } = *output;

    if !mapping
        .expect("network mapping initialized")
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .start()?
    {
        return Err(io::Error::other("submapping failed to start"));
    }
    if !reader.start_streaming()? {
        return Err(io::Error::other("dataset streaming failed"));
    }
    let start = Instant::now();
    let mut progress = 0;
    loop {
        app::check_failure(failure)?;
        estimator
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .process_frame()?;
        display_processor(
            factory,
            kind,
            estimator,
            processor.expect("network processor initialized"),
        )?;
        app::display_top_view(factory, writer)?;
        factory.wait_key(2)?;
        if !reader.is_streaming() {
            log::info!("DatasetReader Finished!");
            app::finish_output(
                (estimator, mapping, processor),
                parameters,
                (save, mode, codec),
                failure,
                start,
            )?;
            break;
        }
        let new = (reader.completion() * 100.0) as i32;
        if new > progress {
            factory.print_timing();
            progress = new;
            log::info!("Progress: {progress}%");
        }
    }
    Ok(())
}
