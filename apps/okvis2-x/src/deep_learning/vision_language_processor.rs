/* OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! Vision-language encoding, SAM NMS, synchronized depth batches and activation visualization.
use super::{
    deep_learning_processor::{
        CameraWorker, DeepLearningProcessor, DisplayImages, ImageCallback, Images,
        ProcessingControl, camera_measurements, send_dropping_oldest,
    },
    utils::{
        ColourMap, Tensor, VisualizationRaster, add_weighted, byte_image, float_image, image_u8,
        invalid,
    },
};
use crate::{mapping::map_typedefs::CHAIR_EMBEDDING, sensors::measurements::DeepLearningImageData};
use firefly_error::Result;
use purecv::core::Matrix;
use rand::RngExt;
use std::{
    fs::{self, File},
    io::Write,
    path::Path,
    sync::{
        Arc, Mutex, RwLock,
        mpsc::{self, Receiver},
    },
};

/// Vision encoding backend receiving uint8 HWC RGB and returning `[1,C,H,W]`.
pub trait VisionEncoder: Send {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn forward(&mut self, rgb: &Matrix<u8>) -> Result<Tensor>;
}
/// Segmentation backend receiving uint8 HWC RGB and returning masks and XYXY boxes.
pub trait SamModel: Send {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn forward(&mut self, rgb: &Matrix<u8>) -> Result<(Tensor, Tensor)>;
}
struct VisualizationData {
    rgb: Matrix<u8>,
    depth: Option<Matrix<f32>>,
    features: Tensor,
    masks: Tensor,
}
pub struct VlProcessor {
    worker: CameraWorker,
    text_embedding: Arc<RwLock<Tensor>>,
    visualizations: Arc<Mutex<Receiver<VisualizationData>>>,
    raster: Arc<dyn VisualizationRaster>,
    dataset_files: Option<[File; 3]>,
}
impl VlProcessor {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn new(
        rgb_camera: usize,
        height: usize,
        width: usize,
        mut encoder: Box<dyn VisionEncoder>,
        mut sam: Box<dyn SamModel>,
        raster: Arc<dyn VisualizationRaster>,
    ) -> Result<Self> {
        let warm = Matrix::new(height, width, 3);
        for _ in 0..10 {
            sam.forward(&warm)?;
        }
        for _ in 0..10 {
            encoder.forward(&warm)?;
        }
        let text_embedding = Arc::new(RwLock::new(normalize_embedding(&CHAIR_EMBEDDING)?));
        let control = ProcessingControl::new();
        let c = control.clone();
        let (sender, receiver) = mpsc::sync_channel(1);
        let receiver = Arc::new(Mutex::new(receiver));
        let vis = receiver.clone();
        let worker = CameraWorker::start(control, move |mut frames| {
            let index = frames
                .get(&rgb_camera)
                .and_then(|entries| {
                    entries.iter().position(|m| {
                        m.measurement
                            .image
                            .as_ref()
                            .is_some_and(|image| image.rows() > 0 && image.cols() > 0)
                    })
                })
                .ok_or_else(|| {
                    invalid(
                        "RGB camera image missing",
                        "VlProcessor::process_language_network",
                    )
                })?;
            let rgb = byte_image(
                frames[&rgb_camera][index]
                    .measurement
                    .image
                    .as_ref()
                    .unwrap(),
            )?
            .clone();
            if rgb.channels != 3 {
                return Err(invalid(
                    "RGB image requires three channels",
                    "VlProcessor::process_language_network",
                ));
            }
            let output = encoder.forward(&rgb)?;
            let mut features = nchw_to_hwc(&output)?;
            let (masks, boxes) = sam.forward(&rgb)?;
            let filtered = segment_post_processing(&boxes)?;
            let masks = select_masks(&masks, &filtered)?;
            Self::normalize_feature_width(&mut features);
            let measurement = &mut frames.get_mut(&rgb_camera).unwrap()[index];
            measurement
                .measurement
                .deep_learning_data
                .insert("language_features".into(), deep_data(&features));
            measurement
                .measurement
                .deep_learning_data
                .insert("sam_masks".into(), deep_data(&masks));
            c.invoke(&mut frames)?;
            let mut depth = None;
            for entries in frames.values() {
                for measurement in entries {
                    if let Some(image) = measurement
                        .measurement
                        .depth_image
                        .as_ref()
                        .filter(|image| image.rows() > 0 && image.cols() > 0)
                    {
                        depth = Some(float_image(image)?.clone());
                        break;
                    }
                }
            }
            if depth.is_none() {
                return Err(invalid(
                    "depth image absent after image callback",
                    "VlProcessor::process_language_network",
                ));
            }
            send_dropping_oldest(
                &sender,
                &vis,
                VisualizationData {
                    rgb,
                    depth,
                    features,
                    masks,
                },
            )?;
            Ok(())
        });
        Ok(Self {
            worker,
            text_embedding,
            visualizations: receiver,
            raster,
            dataset_files: None,
        })
    }
    fn normalize_feature_width(features: &mut Tensor) {
        // Source normalizes dimension 1 of HWC, i.e. width, preserving that axis.
        let (height, width, channels) = (features.shape[0], features.shape[1], features.shape[2]);
        let mut values = features.data.to_vec();
        for y in 0..height {
            for channel in 0..channels {
                let norm = (0..width)
                    .map(|x| values[(y * width + x) * channels + channel].powi(2))
                    .sum::<f32>()
                    .sqrt();
                for x in 0..width {
                    values[(y * width + x) * channels + channel] /= norm;
                }
            }
        }
        features.data = values.into();
        features.shape.retain(|&size| size != 1);
    }

    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn from_parameters(
        parameters: &crate::config::parameters::ViParameters,
        encoder: Box<dyn VisionEncoder>,
        sam: Box<dyn SamModel>,
        raster: Arc<dyn VisualizationRaster>,
    ) -> Result<Self> {
        let system = &parameters.n_camera_system;
        let index = (0..system.num_cameras())
            .find(|&i| system.camera_type(i).is_colour)
            .ok_or_else(|| {
                invalid(
                    "vision language requires an RGB camera",
                    "VlProcessor::from_parameters",
                )
            })?;
        Self::new(
            index,
            system.camera_geometry(index).image_height() as usize,
            system.camera_geometry(index).image_width() as usize,
            encoder,
            sam,
            raster,
        )
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn set_language_embedding(&mut self, embedding: &[f32]) -> Result<()> {
        *self.text_embedding.write().unwrap() = normalize_embedding(embedding)?;
        Ok(())
    }
    /// The source's dataset switch creates directories and index headers; its
    /// image/index row writes are disabled, so no image rows are synthesized.
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn enable_dataset(&mut self, path: &Path) -> Result<()> {
        let run = || -> std::io::Result<[File; 3]> {
            fs::create_dir_all(path)?;
            for name in ["depth", "rgb", "sam"] {
                fs::create_dir_all(path.join(name))?;
            }
            let mut files = [
                File::create(path.join("depth.txt"))?,
                File::create(path.join("rgb.txt"))?,
                File::create(path.join("sam.txt"))?,
            ];
            for (file, name) in files.iter_mut().zip(["depth", "rgb", "segment"]) {
                writeln!(file, "# {name} images \n# timestamp filename ")?;
            }
            Ok(files)
        };
        self.dataset_files =
            Some(run().map_err(|e| invalid(e.to_string(), "VlProcessor::enable_dataset"))?);
        Ok(())
    }
}
impl DeepLearningProcessor for VlProcessor {
    fn set_blocking(&self, value: bool) {
        *self.worker.control.blocking.write().unwrap() = Some(value);
    }
    fn set_image_callback(&self, callback: ImageCallback) {
        *self.worker.control.callback.write().unwrap() = Some(callback);
    }
    fn add_images(&self, images: &Images, depths: &Images) -> Result<bool> {
        if depths.is_empty() {
            return Ok(false);
        }
        self.worker
            .submit(camera_measurements(images, Some(depths))?)
    }
    fn finished_processing(&self) -> Result<bool> {
        self.worker.finished()
    }
    fn display(&self, images: &mut DisplayImages) -> Result<()> {
        self.worker.control.check_error()?;
        let Ok(vis) = self.visualizations.lock().unwrap().try_recv() else {
            return Ok(());
        };
        if let Some(depth) = vis.depth {
            let max = depth
                .data
                .iter()
                .copied()
                .filter(|x| !x.is_nan())
                .fold(f32::NEG_INFINITY, f32::max);
            let values = Matrix::from_vec(
                depth.rows,
                depth.cols,
                1,
                depth
                    .data
                    .iter()
                    .map(|z| {
                        let value = z / max;
                        let value = if value > 1. { 1. } else { value };
                        (value * 255.).round_ties_even() as u8
                    })
                    .collect(),
            );
            images.insert(
                "depth".into(),
                image_u8(self.raster.colour_map(&values, ColourMap::Inferno)?),
            );
        }
        if vis.features.data.is_empty() || vis.rgb.rows == 0 || vis.rgb.cols == 0 {
            return Ok(());
        }
        if vis.features.shape.len() != 3 {
            return Err(invalid(
                "squeezed language feature map is not rank three",
                "VlProcessor::display",
            ));
        }
        let (h, w, c) = (
            vis.features.shape[0],
            vis.features.shape[1],
            vis.features.shape[2],
        );
        let embedding = self.text_embedding.read().unwrap();
        if embedding.data.len() != c {
            return Err(invalid(
                "text and image feature dimension mismatch",
                "VlProcessor::display",
            ));
        }
        let values = Matrix::from_vec(
            h,
            w,
            1,
            vis.features
                .data
                .chunks_exact(c)
                .map(|pixel| {
                    (pixel
                        .iter()
                        .zip(embedding.data.iter())
                        .map(|(a, b)| a * b)
                        .sum::<f32>()
                        .clamp(0., 1.)
                        * 254.) as u8
                })
                .collect(),
        );
        let colour = self.raster.colour_map(&values, ColourMap::Jet)?;
        let colour = self
            .raster
            .resize_cubic(&colour, vis.rgb.rows, vis.rgb.cols)?;
        images.insert(
            "language_rgb".into(),
            image_u8(add_weighted(&vis.rgb, 0.2, &colour, 0.8)?),
        );
        images.insert("raw_rgb".into(), image_u8(vis.rgb.clone()));
        Self::display_masks(&vis.rgb, &vis.masks, images)
    }
}
fn normalize_embedding(values: &[f32]) -> Result<Tensor> {
    let norm = values.iter().map(|v| v * v).sum::<f32>().sqrt();
    Tensor::new(
        vec![values.len()],
        values.iter().map(|v| v / norm).collect(),
    )
}
fn deep_data(tensor: &Tensor) -> DeepLearningImageData {
    DeepLearningImageData {
        shape: tensor.shape.iter().map(|x| *x as i64).collect(),
        data: Some(tensor.data.clone()),
    }
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn nchw_to_hwc(tensor: &Tensor) -> Result<Tensor> {
    if tensor.shape.len() != 4 || tensor.shape[0] == 0 {
        return Err(invalid(
            "encoder output must be [N,C,H,W] with N>0",
            "nchw_to_hwc",
        ));
    }
    let (c, h, w) = (tensor.shape[1], tensor.shape[2], tensor.shape[3]);
    let mut data = Vec::with_capacity(h * w * c);
    for y in 0..h {
        for x in 0..w {
            for channel in 0..c {
                data.push(tensor.data[(channel * h + y) * w + x]);
            }
        }
    }
    Tensor::new(vec![h, w, c], data)
}
fn select_masks(masks: &Tensor, indices: &[usize]) -> Result<Tensor> {
    if masks.shape.is_empty() {
        return Err(invalid("SAM mask batch dimension absent", "select_masks"));
    }
    let size = masks.shape[1..].iter().product::<usize>();
    let mut data = Vec::with_capacity(size * indices.len());
    for &index in indices {
        if index >= masks.shape[0] {
            return Err(invalid("SAM mask index out of range", "select_masks"));
        }
        data.extend_from_slice(&masks.data[index * size..(index + 1) * size]);
    }
    let mut shape = masks.shape.clone();
    shape[0] = indices.len();
    Tensor::new(shape, data)
}
/// Equal scores are processed in their original order. XYXY coordinates are
/// individually truncated after the source's float width/height subtraction.
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn segment_post_processing(boxes: &Tensor) -> Result<Vec<usize>> {
    if boxes.shape.len() != 2 || boxes.shape[1] != 4 {
        return Err(invalid(
            "SAM boxes require [N,4]",
            "segment_post_processing",
        ));
    }
    let rectangles = boxes
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| {
            let values = [b[0], b[1], b[2] - b[0], b[3] - b[1]];
            if values.iter().any(|v| {
                !v.is_finite()
                    || f64::from(*v) < f64::from(i32::MIN)
                    || f64::from(*v) > f64::from(i32::MAX)
            }) {
                return Err(invalid(
                    "box coordinates cannot be represented as i32",
                    "segment_post_processing",
                ));
            }
            Ok(values.map(|v| v as i32))
        })
        .collect::<Result<Vec<_>>>()?;
    let mut keep = Vec::new();
    for (index, b) in rectangles.iter().enumerate() {
        let mut retain = true;
        for &selected in &keep {
            let a: &[i32; 4] = &rectangles[selected];
            let a_right = a[0]
                .checked_add(a[2])
                .ok_or_else(|| invalid("box coordinate overflows", "segment_post_processing"))?;
            let b_right = b[0]
                .checked_add(b[2])
                .ok_or_else(|| invalid("box coordinate overflows", "segment_post_processing"))?;
            let a_bottom = a[1]
                .checked_add(a[3])
                .ok_or_else(|| invalid("box coordinate overflows", "segment_post_processing"))?;
            let b_bottom = b[1]
                .checked_add(b[3])
                .ok_or_else(|| invalid("box coordinate overflows", "segment_post_processing"))?;
            let width = i64::from(a_right.min(b_right)) - i64::from(a[0].max(b[0]));
            let height = i64::from(a_bottom.min(b_bottom)) - i64::from(a[1].max(b[1]));
            let area_a = a[2]
                .checked_mul(a[3])
                .ok_or_else(|| invalid("box area overflows", "segment_post_processing"))?;
            let area_b = b[2]
                .checked_mul(b[3])
                .ok_or_else(|| invalid("box area overflows", "segment_post_processing"))?;
            let areas = area_a
                .checked_add(area_b)
                .ok_or_else(|| invalid("box area sum overflows", "segment_post_processing"))?;
            let intersection = (width.max(0) * height.max(0)) as f64;
            let distance = if areas <= 0 {
                0.0
            } else {
                1.0 - intersection / (f64::from(areas) - intersection)
            };
            let overlap = (1.0 - distance) as f32;
            if overlap > 0.7 {
                retain = false;
                break;
            }
        }
        if retain {
            keep.push(index);
        }
    }
    Ok(keep)
}
/// Mask averaging variant in the vision-language source uses a fixed 768 channels
/// and an explicit image-to-feature resize ratio.
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn average_descriptor(
    features: &crate::sensors::measurements::DeepLearningImageData,
    mask: &Matrix<u8>,
    resize_ratio: usize,
) -> Result<([f32; 768], usize)> {
    if resize_ratio == 0 || features.shape.len() < 2 {
        return Err(invalid(
            "invalid feature shape or resize ratio",
            "average_descriptor",
        ));
    }
    let width = usize::try_from(features.shape[1])
        .map_err(|_| invalid("negative feature width", "average_descriptor"))?;
    let data = features
        .data
        .as_ref()
        .ok_or_else(|| invalid("feature storage absent", "average_descriptor"))?;
    let mut result = [0.; 768];
    let mut count = 0;
    for y in 0..mask.rows {
        for x in 0..mask.cols {
            if mask.data[y * mask.cols + x] == 0 {
                continue;
            }
            let offset = (((y as f32 / resize_ratio as f32) as usize) * width
                + (x as f32 / resize_ratio as f32) as usize)
                * 768;
            let pixel = data.get(offset..offset + 768).ok_or_else(|| {
                invalid(
                    "feature mask projection outside allocation",
                    "average_descriptor",
                )
            })?;
            for (sum, value) in result.iter_mut().zip(pixel) {
                *sum += value;
            }
            count += 1;
        }
    }
    for value in &mut result {
        *value /= count as f32;
    }
    Ok((result, count))
}

impl VlProcessor {
    fn display_masks(rgb: &Matrix<u8>, masks: &Tensor, images: &mut DisplayImages) -> Result<()> {
        // Source accesses size(3) while calculating the mask stride. Require that
        // dimension explicitly; a three-dimensional tensor has no defined stride.
        if masks.shape.len() != 4 {
            return Err(invalid(
                "SAM display requires a fourth mask dimension for source stride",
                "VlProcessor::display",
            ));
        }
        let (n, mh, mw, stride) = (
            masks.shape[0],
            masks.shape[1],
            masks.shape[2],
            masks.shape[3],
        );
        let mut mask_image = Matrix::new(mh, mw, 3);
        let mut rng = rand::rng();
        for mask in 0..n {
            let colour = [
                rng.random_range(1..=255),
                rng.random_range(1..=255),
                rng.random_range(1..=255),
            ];
            for y in 0..mh {
                for x in 0..mw {
                    let offset = mask * mh * mw + y * mw + x * stride;
                    let value = masks.data.get(offset).ok_or_else(|| {
                        invalid(
                            "SAM display offset exceeds mask allocation",
                            "VlProcessor::display",
                        )
                    })?;
                    if *value == 1. {
                        mask_image.data[(y * mw + x) * 3..(y * mw + x) * 3 + 3]
                            .copy_from_slice(&colour);
                    }
                }
            }
        }
        images.insert(
            "sam_masks".into(),
            image_u8(add_weighted(rgb, 0.5, &mask_image, 0.5)?),
        );
        Ok(())
    }
}
