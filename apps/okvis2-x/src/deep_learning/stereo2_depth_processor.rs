/* OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! Stereo model input preparation, disparity propagation and asynchronous publication.
use super::{
    deep_learning_processor::{
        CameraMeasurements, CameraWorker, DeepLearningProcessor, DisplayImages, ImageCallback,
        Images, ProcessingControl, camera_measurements, send_dropping_oldest,
    },
    utils::{
        ColourMap, Tensor, VisualizationRaster, byte_image, colour_values, cv_mat_to_tensor,
        image_f32, image_u8, invalid,
    },
};
use crate::{
    cameras::camera_base::RectifyMaps, config::parameters::ViParameters,
    kinematics::transformation::Transformation,
};
use firefly_error::Result;
use nalgebra::DVector;
use purecv::{
    core::{
        Matrix,
        types::{BorderTypes, Scalar},
    },
    imgproc::geometric::{InterpolationFlags, remap},
};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{
        Arc, Mutex,
        mpsc::{self, Receiver},
    },
};

/// Stereo inference backend receiving two contiguous `[H,W]` uint8 inputs.
/// The output is `[2,H,W]` (disparity and sigma) or a single `[H,W]` disparity.
pub trait StereoModel: Send {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn forward(&mut self, left: &Matrix<u8>, right: &Matrix<u8>) -> Result<Tensor>;
}
#[derive(Clone)]
pub struct StereoCalibration {
    pub id_left: usize,
    pub id_right: usize,
    pub width: usize,
    pub height: usize,
    pub intrinsics: [f64; 4],
    pub focal_length: f64,
    pub baseline: f64,
    pub t_sc: Transformation,
    pub rect_left: Option<RectifyMaps>,
    pub rect_right: Option<RectifyMaps>,
}
impl StereoCalibration {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn from_parameters(parameters: &ViParameters) -> Result<Self> {
        let system = &parameters.n_camera_system;
        let gray_count = parameters
            .camera
            .stereo_indices
            .iter()
            .filter(|&&i| !system.camera_type(i).is_colour)
            .copied()
            .collect::<BTreeSet<_>>()
            .len();
        if gray_count != 2 || parameters.camera.stereo_indices.len() < 2 {
            return Err(invalid(
                "stereo network requires exactly two grayscale cameras",
                "StereoCalibration::from_parameters",
            ));
        }
        let id_left = parameters.camera.stereo_indices[0];
        let id_right = parameters.camera.stereo_indices[1];
        let rectified = system.camera_type(id_left).depth_type.need_rectify;
        let (camera, t_sc, t_right) = if rectified {
            (
                system.rectify_camera_geometry(id_left),
                system.rectify_t_sc(id_left),
                system.rectify_t_sc(id_right),
            )
        } else {
            (
                system.camera_geometry(id_left),
                system.t_sc(id_left),
                system.t_sc(id_right),
            )
        };
        let mut intrinsics = DVector::zeros(0);
        camera.get_intrinsics(&mut intrinsics);
        if intrinsics.len() < 4 {
            return Err(invalid(
                "camera requires four intrinsics",
                "StereoCalibration::from_parameters",
            ));
        }
        let mut rect_left = None;
        let mut rect_right = None;
        if rectified {
            camera.get_rectify_map(&mut rect_left);
            system
                .rectify_camera_geometry(id_right)
                .get_rectify_map(&mut rect_right);
            if rect_left.is_none() || rect_right.is_none() {
                return Err(invalid(
                    "rectification maps are absent",
                    "StereoCalibration::from_parameters",
                ));
            }
        }
        Ok(Self {
            id_left,
            id_right,
            width: camera.image_width() as usize,
            height: camera.image_height() as usize,
            intrinsics: [intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]],
            focal_length: intrinsics[0],
            baseline: t_sc.inverse().compose(t_right).translation().norm(),
            t_sc: (**t_sc).clone(),
            rect_left,
            rect_right,
        })
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn rectify(&self, frames: &mut CameraMeasurements) -> Result<(Matrix<u8>, Matrix<u8>)> {
        for (id, maps) in [
            (self.id_left, &self.rect_left),
            (self.id_right, &self.rect_right),
        ] {
            let entry = frames
                .get_mut(&id)
                .and_then(|m| m.first_mut())
                .ok_or_else(|| {
                    invalid(
                        "stereo camera measurement missing",
                        "StereoCalibration::rectify",
                    )
                })?;
            let image = entry
                .measurement
                .image
                .as_ref()
                .ok_or_else(|| invalid("stereo image absent", "StereoCalibration::rectify"))?;
            if let Some(maps) = maps {
                let src = byte_image(image)?;
                let (x, y) = float_rectify_maps(maps)?;
                let rect = remap(
                    src,
                    &x,
                    &y,
                    InterpolationFlags::Linear,
                    BorderTypes::Constant,
                    Scalar::all(0),
                )
                .map_err(|e| invalid(e.to_string(), "StereoCalibration::rectify"))?;
                entry.measurement.image = Some(image_u8(rect));
            }
        }
        let left = cv_mat_to_tensor(frames[&self.id_left][0].measurement.image.as_ref().unwrap())?;
        let right = cv_mat_to_tensor(
            frames[&self.id_right][0]
                .measurement
                .image
                .as_ref()
                .unwrap(),
        )?;
        Ok((left, right))
    }
}
/// The fixed maps preserve `OpenCV`'s five-bit subpixel coordinate quantization.
fn float_rectify_maps(maps: &RectifyMaps) -> Result<(Matrix<f32>, Matrix<f32>)> {
    let m = &maps.coordinates;
    let f = &maps.fractions;
    if m.channels != 2 || f.channels != 1 || (m.rows, m.cols) != (f.rows, f.cols) {
        return Err(invalid(
            "invalid fixed rectification maps",
            "float_rectify_maps",
        ));
    }
    let mut x = Matrix::new(m.rows, m.cols, 1);
    let mut y = x.clone();
    for i in 0..m.rows * m.cols {
        x.data[i] = f32::from(m.data[2 * i]) + f32::from(f.data[i] & 31) / 32.;
        y.data[i] = f32::from(m.data[2 * i + 1]) + f32::from(f.data[i] >> 5) / 32.;
    }
    Ok((x, y))
}
#[derive(Clone)]
pub struct StereoPrediction {
    pub time_stamp: jiff::Timestamp,
    pub left: Matrix<u8>,
    pub right: Matrix<u8>,
    pub rgb: Option<Arc<purecv::core::DynamicMatrix>>,
    pub depth: Tensor,
    pub sigma: Tensor,
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn disparity_to_depth(
    output: &Tensor,
    fb: f64,
    minimum_sigma: Option<f32>,
) -> Result<(Tensor, Tensor, Tensor, Tensor)> {
    let (disparity, disparity_sigma) = if output.shape.first() == Some(&2) {
        (output.plane(0)?, output.plane(1)?)
    } else if output.shape.len() == 2 {
        (output.clone(), output.map(|_| 0.))
    } else {
        return Err(invalid(
            "expected disparity [H,W] or [2,H,W]",
            "disparity_to_depth",
        ));
    };
    let depth = disparity.map(|d| (fb as f32) / d);
    let sigma = Tensor::new(
        depth.shape.clone(),
        depth
            .data
            .iter()
            .zip(disparity.data.iter())
            .zip(disparity_sigma.data.iter())
            .map(|((z, d), s)| {
                let mut value = if minimum_sigma.is_some() {
                    (z / d) * (2. * s)
                } else {
                    2. * (z / d) * s
                };
                if !value.is_finite() {
                    value = 100.;
                }
                if let Some(minimum) = minimum_sigma
                    && value < minimum
                {
                    value = minimum;
                }
                value
            })
            .collect(),
    )?;
    Ok((depth, sigma, disparity, disparity_sigma))
}

pub struct Stereo2DepthProcessor {
    worker: CameraWorker,
    visualizations: Arc<Mutex<Receiver<DisplayImages>>>,
}
impl Stereo2DepthProcessor {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn from_parameters(
        parameters: &ViParameters,
        model: Box<dyn StereoModel>,
        raster: Arc<dyn VisualizationRaster>,
    ) -> Result<Self> {
        Self::new(
            StereoCalibration::from_parameters(parameters)?,
            model,
            raster,
        )
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn new(
        calibration: StereoCalibration,
        mut model: Box<dyn StereoModel>,
        raster: Arc<dyn VisualizationRaster>,
    ) -> Result<Self> {
        // The standalone stereo processor warms up at the source's fixed 480×640.
        let warm = Matrix::new(480, 640, 1);
        for _ in 0..10 {
            model.forward(&warm, &warm)?;
        }
        let control = ProcessingControl::new();
        let (sender, receiver) = mpsc::sync_channel(1);
        let receiver = Arc::new(Mutex::new(receiver));
        let vis = receiver.clone();
        let c = control.clone();
        let worker = CameraWorker::start(control, move |mut frames| {
            let (left, right) = calibration.rectify(&mut frames)?;
            let output = model.forward(&left, &right)?;
            let (depth, sigma, disparity, disparity_sigma) = disparity_to_depth(
                &output,
                calibration.focal_length * calibration.baseline,
                None,
            )?;
            let frame = &mut frames.get_mut(&calibration.id_left).unwrap()[0];
            frame.measurement.depth_image = Some(image_f32(depth.matrix()?));
            frame.measurement.sigma_image = Some(image_f32(sigma.matrix()?));
            c.invoke(&mut frames)?;
            let images = BTreeMap::from([
                ("leftImage".into(), image_u8(left)),
                ("rightImage".into(), image_u8(right)),
                (
                    "stereoDepth".into(),
                    image_u8(colour_values(
                        &disparity,
                        60.,
                        ColourMap::Inferno,
                        raster.as_ref(),
                    )?),
                ),
                (
                    "stereoSigma".into(),
                    image_u8(colour_values(
                        &disparity_sigma,
                        10.,
                        ColourMap::Jet,
                        raster.as_ref(),
                    )?),
                ),
            ]);
            send_dropping_oldest(&sender, &vis, images)?;
            Ok(())
        });
        Ok(Self {
            worker,
            visualizations: receiver,
        })
    }
}
impl DeepLearningProcessor for Stereo2DepthProcessor {
    fn set_blocking(&self, value: bool) {
        *self.worker.control.blocking.write().unwrap() = Some(value);
    }
    fn set_image_callback(&self, callback: ImageCallback) {
        *self.worker.control.callback.write().unwrap() = Some(callback);
    }
    fn add_images(&self, images: &Images, _depths: &Images) -> Result<bool> {
        self.worker.submit(camera_measurements(images, None)?)
    }
    fn display(&self, images: &mut DisplayImages) -> Result<()> {
        self.worker.control.check_error()?;
        if let Ok(vis) = self.visualizations.lock().unwrap().try_recv() {
            images.extend(vis);
        }
        Ok(())
    }
    fn finished_processing(&self) -> Result<bool> {
        self.worker.finished()
    }
}
