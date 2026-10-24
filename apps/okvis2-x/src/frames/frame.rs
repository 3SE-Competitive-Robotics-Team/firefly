/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! A camera image, feature observations, cached rays and optional segmentation.

use crate::cameras::camera_base::CameraBase;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Vector2, Vector3, Vector4};
use purecv::core::{DynamicMatrix, Matrix, types::Size};
pub use purecv::features2d::KeyPoint;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

pub type Image = Arc<DynamicMatrix>;

/// Detector capability; the implementation supplies the configured detector algorithm.
pub trait FeatureDetector {
    /// # Errors
    /// Returns an error if the image or detector is missing, or if feature detection fails.
    fn detect(&self, image: &DynamicMatrix, keypoints: &mut Vec<KeyPoint>) -> Result<()>;
}
/// Descriptor computation may remove keypoints, as `OpenCV`'s compute does.
pub trait DescriptorExtractor {
    /// 更新描述子提取的重力方向；实现可忽略，默认无操作。
    fn set_extraction_direction(&self, _direction: Vector3<f32>) {}
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    fn compute(
        &self,
        image: &DynamicMatrix,
        keypoints: &mut Vec<KeyPoint>,
        descriptors: &mut Matrix<u8>,
    ) -> Result<()>;
}
/// Segmentation logits in contiguous class/row/column order, with 19 classes.
pub struct SegmentationOutput {
    pub width: usize,
    pub height: usize,
    pub logits: Vec<f32>,
}
/// The backend runs a batch of one normalized RGB CHW tensor and returns the first
/// output tuple element, without softmax. CPU implementations must apply and
/// restore the requested thread count around inference, including error paths.
pub trait Network {
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    fn forward(
        &self,
        input_chw: &[f32],
        width: usize,
        height: usize,
        num_threads: usize,
    ) -> Result<SegmentationOutput>;
}

#[derive(Default)]
pub struct Frame {
    image: Option<Image>,
    depth_image: Option<Image>,
    camera_geometry: Option<Arc<dyn CameraBase>>,
    detector: Option<Arc<dyn FeatureDetector>>,
    extractor: Option<Arc<dyn DescriptorExtractor>>,
    network: Option<Arc<dyn Network>>,
    keypoints: Vec<KeyPoint>,
    back_projections: Vec<Option<Vector3<f64>>>,
    back_projections_valid: Vec<bool>,
    descriptors: Option<Arc<Matrix<u8>>>,
    classifications: Vec<[f32; 19]>,
    landmark_ids: Vec<u64>,
    landmarks: Vec<Vector4<f64>>,
    landmark_initialisations: Vec<bool>,
    is_classified: AtomicBool,
}
/// C++'s copy constructor copies resources and the classification flag only.
impl Clone for Frame {
    fn clone(&self) -> Self {
        Self {
            image: self.image.clone(),
            depth_image: self.depth_image.clone(),
            camera_geometry: self.camera_geometry.clone(),
            detector: self.detector.clone(),
            extractor: self.extractor.clone(),
            network: self.network.clone(),
            is_classified: AtomicBool::new(self.is_classified()),
            ..Self::default()
        }
    }
}
impl Frame {
    pub fn new(
        image: Image,
        geometry: Arc<dyn CameraBase>,
        detector: Arc<dyn FeatureDetector>,
        extractor: Arc<dyn DescriptorExtractor>,
        network: Option<Arc<dyn Network>>,
    ) -> Self {
        Self {
            image: Some(image),
            camera_geometry: Some(geometry),
            detector: Some(detector),
            extractor: Some(extractor),
            network,
            ..Self::default()
        }
    }
    pub fn set_image(&mut self, image: Image) {
        self.image = Some(image);
    }
    pub fn set_depth_image(&mut self, image: Image) {
        self.depth_image = Some(image);
    }
    pub fn set_geometry(&mut self, geometry: Arc<dyn CameraBase>) {
        self.camera_geometry = Some(geometry);
    }
    pub fn set_detector(&mut self, detector: Arc<dyn FeatureDetector>) {
        self.detector = Some(detector);
    }
    pub fn set_extractor(&mut self, extractor: Arc<dyn DescriptorExtractor>) {
        self.extractor = Some(extractor);
    }
    pub fn set_network(&mut self, network: Arc<dyn Network>) {
        self.network = Some(network);
    }
    pub fn image(&self) -> Option<&Image> {
        self.image.as_ref()
    }
    pub fn depth_image(&self) -> Option<&Image> {
        self.depth_image.as_ref()
    }
    /// # Panics
    /// Panics if a required resource/invariant is absent: `camera geometry not initialized`.
    pub fn geometry(&self) -> &Arc<dyn CameraBase> {
        self.camera_geometry
            .as_ref()
            .expect("camera geometry not initialized")
    }
    /// Returns the concrete camera geometry, rejecting a mismatched model type.
    /// # Panics
    /// Panics if a required resource/invariant is absent: `incorrect camera geometry cast`.
    pub fn geometry_as<G: CameraBase>(&self) -> &G {
        let geometry: &dyn std::any::Any = self.geometry().as_ref();
        geometry
            .downcast_ref::<G>()
            .expect("incorrect camera geometry cast")
    }
    /// # Errors
    /// Returns an error if the image or detector is missing, or if feature detection fails.
    pub fn detect(&mut self) -> Result<usize> {
        self.keypoints.clear();
        self.descriptors = None;
        self.landmark_ids.clear();
        let detector = self
            .detector
            .as_ref()
            .ok_or_else(|| missing("detector", "Frame::detect"))?;
        let image = self
            .image
            .as_ref()
            .ok_or_else(|| missing("image", "Frame::detect"))?;
        detector.detect(image, &mut self.keypoints)?;
        Ok(self.keypoints.len())
    }
    /// # Errors
    /// Returns an error if the image/extractor is missing or descriptor extraction fails.
    pub fn describe(&mut self) -> Result<usize> {
        let extractor = self
            .extractor
            .as_ref()
            .ok_or_else(|| missing("extractor", "Frame::describe"))?;
        let image = self
            .image
            .as_ref()
            .ok_or_else(|| missing("image", "Frame::describe"))?;
        let mut descriptors = self
            .descriptors
            .as_ref()
            .map_or_else(|| Matrix::new(0, 0, 1), |d| (**d).clone());
        extractor.compute(image, &mut self.keypoints, &mut descriptors)?;
        self.descriptors = Some(Arc::new(descriptors));
        self.reset_landmark_storage();
        Ok(self.keypoints.len())
    }
    /// # Panics
    /// Panics if a required resource/invariant is absent: `camera geometry not initialized`.
    pub fn compute_back_projections(&mut self) -> usize {
        let camera = self
            .camera_geometry
            .as_ref()
            .expect("camera geometry not initialized");
        self.back_projections.resize(self.keypoints.len(), None);
        self.back_projections_valid
            .resize(self.keypoints.len(), false);
        let mut count = 0;
        for (k, keypoint) in self.keypoints.iter().enumerate() {
            let mut ray = self.back_projections[k].unwrap_or_else(|| Vector3::repeat(f64::NAN));
            let success = camera.back_project(
                &Vector2::new(f64::from(keypoint.pt.x), f64::from(keypoint.pt.y)),
                &mut ray,
            );
            self.back_projections[k] = Some(ray);
            self.back_projections_valid[k] = success;
            count += usize::from(success);
        }
        count
    }
    /// # Panics
    /// Panics if the required assertion fails: `index < self.keypoints.len(), "keypoint index out
    /// of range"`.
    pub fn get_back_projection(&self, index: usize, ray: &mut Vector3<f64>) -> bool {
        assert!(index < self.keypoints.len(), "keypoint index out of range");
        *ray = self.back_projections[index].expect("back projection not computed");
        self.back_projections_valid[index]
    }
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    pub fn compute_classifications(
        &mut self,
        size_u: usize,
        size_v: usize,
        num_threads: usize,
    ) -> Result<usize> {
        let network = self
            .network
            .as_ref()
            .ok_or_else(|| missing("segmentation network", "Frame::compute_classifications"))?;
        let image = self
            .image
            .as_ref()
            .ok_or_else(|| missing("image", "Frame::compute_classifications"))?;
        let gray = image
            .as_matrix_u8()
            .filter(|m| m.channels == 1 && m.rows > 0 && m.cols > 0)
            .ok_or_else(|| {
                Error::new(
                    ErrorKind::InvalidArgument,
                    "segmentation requires a nonempty grayscale u8 image",
                )
                .with_operation("Frame::compute_classifications")
            })?;
        let resized = purecv::imgproc::resize(gray, Size::new(size_u, size_v)).map_err(|e| {
            Error::new(ErrorKind::InvalidArgument, e.to_string())
                .with_operation("Frame::compute_classifications")
        })?;
        let plane = size_u * size_v;
        let mut input = vec![0.0_f32; 3 * plane];
        let mean = [0.485_f32, 0.456, 0.406];
        let std = [0.229_f32, 0.224, 0.225];
        for c in 0..3 {
            for p in 0..plane {
                input[c * plane + p] =
                    (f32::from(resized.data[p]) * (1.0_f32 / 255.0) - mean[c]) / std[c];
            }
        }
        let output = network.forward(&input, size_u, size_v, num_threads)?;
        if output.width == 0
            || output.height == 0
            || output
                .width
                .checked_mul(output.height)
                .and_then(|n| n.checked_mul(19))
                != Some(output.logits.len())
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "segmentation output must have shape [1,19,H,W]",
            )
            .with_operation("Frame::compute_classifications"));
        }
        let scale_u = size_u as f64 / gray.cols as f64;
        let scale_v = size_v as f64 / gray.rows as f64;
        let mut classifications = Vec::with_capacity(self.keypoints.len());
        for keypoint in &self.keypoints {
            let u = (scale_u * (f64::from(keypoint.pt.x) + 0.5) - 0.5).round();
            let v = (scale_v * (f64::from(keypoint.pt.y) + 0.5) - 0.5).round();
            if !u.is_finite()
                || !v.is_finite()
                || u < 0.0
                || v < 0.0
                || u >= output.width as f64
                || v >= output.height as f64
            {
                return Err(Error::new(
                    ErrorKind::OutOfRange,
                    "keypoint outside segmentation output",
                )
                .with_operation("Frame::compute_classifications"));
            }
            classifications.push(std::array::from_fn(|c| {
                output.logits[(c * output.height + v as usize) * output.width + u as usize]
            }));
        }
        self.set_classifications(classifications)
    }
    /// Publish the classifications computed for this frame's keypoints.
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    pub fn set_classifications(&mut self, classifications: Vec<[f32; 19]>) -> Result<usize> {
        if classifications.len() != self.keypoints.len() {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "classification count must match the frame's keypoints",
            )
            .with_operation("Frame::set_classifications"));
        }
        self.classifications = classifications;
        self.is_classified.store(true, Ordering::Release);
        Ok(self.keypoints.len())
    }
    pub fn is_classified(&self) -> bool {
        self.is_classified.load(Ordering::Acquire)
    }
    pub fn get_classification(&self, index: usize, classification: &mut [f32; 19]) -> bool {
        if !self.is_classified() {
            return false;
        }
        *classification = self.classifications[index];
        true
    }
    pub fn get_cv_keypoint(&self, index: usize, keypoint: &mut KeyPoint) -> bool {
        *keypoint = self.keypoints[index].clone();
        true
    }
    pub fn get_keypoint(&self, index: usize, keypoint: &mut Vector2<f64>) -> bool {
        let k = &self.keypoints[index];
        *keypoint = Vector2::new(f64::from(k.pt.x), f64::from(k.pt.y));
        true
    }
    pub fn get_keypoint_size(&self, index: usize, size: &mut f64) -> bool {
        *size = f64::from(self.keypoints[index].size);
        true
    }
    /// # Panics
    /// Panics if the required assertion fails: `index < self.keypoints.len()`.
    pub fn keypoint_descriptor(&self, index: usize) -> &[u8] {
        assert!(index < self.keypoints.len());
        let descriptors = self.descriptors.as_ref().expect("descriptors not computed");
        &descriptors.data[index * descriptors.cols..(index + 1) * descriptors.cols]
    }
    pub fn set_landmark_id(&mut self, index: usize, id: u64) -> bool {
        self.landmark_ids[index] = id;
        true
    }
    pub fn set_landmark(
        &mut self,
        index: usize,
        landmark: Vector4<f64>,
        initialized: bool,
    ) -> bool {
        self.landmarks[index] = landmark;
        self.landmark_initialisations[index] = initialized;
        true
    }
    pub fn landmark_id(&self, index: usize) -> u64 {
        self.landmark_ids[index]
    }
    pub fn get_landmark(
        &self,
        index: usize,
        landmark: &mut Vector4<f64>,
        initialized: &mut bool,
    ) -> bool {
        *landmark = self.landmarks[index];
        *initialized = self.landmark_initialisations[index];
        true
    }
    pub fn reset_keypoints(&mut self, keypoints: &[KeyPoint]) -> bool {
        self.keypoints = keypoints.to_vec();
        self.reset_landmark_storage();
        true
    }
    pub fn reset_descriptors(&mut self, descriptors: Arc<Matrix<u8>>) -> bool {
        self.descriptors = Some(descriptors);
        true
    }
    pub fn num_keypoints(&self) -> usize {
        self.keypoints.len()
    }
    fn reset_landmark_storage(&mut self) {
        let count = self.keypoints.len();
        self.landmark_ids.resize(count, 0);
        self.landmark_ids.fill(0);
        self.landmarks.resize(count, Vector4::zeros());
        self.landmarks.fill(Vector4::zeros());
        self.landmark_initialisations.resize(count, false);
        self.landmark_initialisations.fill(false);
    }
}
fn missing(what: &str, operation: &'static str) -> Error {
    Error::new(ErrorKind::NotFound, format!("{what} not initialized")).with_operation(operation)
}

/// Harris scale-space detector with the OKVIS uniformity and scale settings.
pub struct BriskDetector {
    detector:
        std::sync::Mutex<firefly_brisk::scale_space_feature_detector::ScaleSpaceFeatureDetector>,
}
impl BriskDetector {
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    pub fn new(
        uniformity_radius: f64,
        octaves: usize,
        absolute_threshold: f64,
        maximum_keypoints: usize,
    ) -> Result<Self> {
        Ok(Self {
            detector: std::sync::Mutex::new(
                firefly_brisk::scale_space_feature_detector::ScaleSpaceFeatureDetector::new(
                    uniformity_radius,
                    octaves,
                    absolute_threshold,
                    maximum_keypoints,
                )?,
            ),
        })
    }
}
impl FeatureDetector for BriskDetector {
    fn detect(&self, image: &DynamicMatrix, keypoints: &mut Vec<KeyPoint>) -> Result<()> {
        let purecv::core::DynamicData::U8(image) = &image.data else {
            return Err(missing("8-bit grayscale image", "BriskDetector::detect"));
        };
        self.detector
            .lock()
            .map_err(|_| {
                Error::new(ErrorKind::Internal, "BRISK detector lock poisoned")
                    .with_operation("BriskDetector::detect")
            })?
            .detect(image, keypoints, None)
    }
}

/// Camera-aware BRISK descriptor computation using the source BRISK2 pattern.
pub struct BriskExtractor {
    pub extractor:
        std::sync::Mutex<firefly_brisk::brisk_descriptor_extractor::BriskDescriptorExtractor>,
}
impl DescriptorExtractor for BriskExtractor {
    fn set_extraction_direction(&self, direction: Vector3<f32>) {
        if let Ok(mut extractor) = self.extractor.lock() {
            extractor.set_extraction_direction(direction);
        }
    }
    fn compute(
        &self,
        image: &DynamicMatrix,
        keypoints: &mut Vec<KeyPoint>,
        descriptors: &mut Matrix<u8>,
    ) -> Result<()> {
        let purecv::core::DynamicData::U8(image) = &image.data else {
            return Err(missing("8-bit grayscale image", "BriskExtractor::compute"));
        };
        let extractor = self
            .extractor
            .lock()
            .map_err(|_| missing("BRISK extractor lock", "BriskExtractor::compute"))?;
        *descriptors = extractor.compute(image, keypoints)?;
        Ok(())
    }
}
