//! BRISK configuration and camera-aware construction, independent of graph orchestration.
use super::error as frontend_error;
use crate::{
    cameras::camera_base::CameraBase,
    frames::frame::{DescriptorExtractor, FeatureDetector, Network},
};
use firefly_error::Result;
use nalgebra::{DVector, Vector3};
use std::sync::Arc;

/// Creates a classification network inside its owning worker thread.
pub type NetworkInitializer = Box<dyn FnOnce() -> Result<Arc<dyn Network>> + Send>;

#[derive(Clone, Debug)]
pub struct BriskSettings {
    pub octaves: usize,
    pub threshold: f64,
    pub absolute_threshold: f64,
    pub maximum_keypoints: usize,
    pub rotation_invariance: bool,
    pub scale_invariance: bool,
}
impl Default for BriskSettings {
    fn default() -> Self {
        Self {
            octaves: 0,
            threshold: 40.0,
            absolute_threshold: 200.0,
            maximum_keypoints: 450,
            rotation_invariance: true,
            scale_invariance: false,
        }
    }
}
/// Camera-aware BRISK constructor and gravity extraction direction. Implementations
/// configure the supplied camera's ray/Jacobian maps and focal length before extraction.
pub trait BriskFeatureFactory: Send + Sync + 'static {
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn create(
        &self,
        camera: &dyn CameraBase,
        settings: &BriskSettings,
        direction: Vector3<f32>,
    ) -> Result<(Arc<dyn FeatureDetector>, Arc<dyn DescriptorExtractor>)>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn classification_network_factory(&self, _camera: usize) -> Result<NetworkInitializer> {
        Err(frontend_error(
            "CNN network constructor capability is required",
        ))
    }
}
/// BRISK2 with camera ray/Jacobian maps and a gravity-aligned extraction direction.
#[derive(Clone, Copy, Debug, Default)]
pub struct RustBriskFeatureFactory;
impl BriskFeatureFactory for RustBriskFeatureFactory {
    fn create(
        &self,
        camera: &dyn CameraBase,
        settings: &BriskSettings,
        direction: Vector3<f32>,
    ) -> Result<(Arc<dyn FeatureDetector>, Arc<dyn DescriptorExtractor>)> {
        use crate::frames::frame::{BriskDetector, BriskExtractor};
        use firefly_brisk::brisk_descriptor_extractor::{BriskDescriptorExtractor, BriskVersion};
        let maps = camera
            .camera_awareness_maps()
            .ok_or_else(|| frontend_error("camera awareness maps not initialised"))?;
        let mut intrinsics = DVector::zeros(camera.num_intrinsics());
        camera.get_intrinsics(&mut intrinsics);
        let focal = *intrinsics
            .get(0)
            .ok_or_else(|| frontend_error("camera focal length absent"))?
            as f32;
        let detector = BriskDetector::new(
            settings.threshold,
            settings.octaves,
            settings.absolute_threshold,
            settings.maximum_keypoints,
        )?;
        let mut extractor = BriskDescriptorExtractor::new(
            settings.rotation_invariance,
            settings.scale_invariance,
            BriskVersion::V2,
            1.0,
        )?;
        extractor.set_camera_properties(maps.rays.clone(), maps.image_jacobians.clone(), focal)?;
        extractor.set_extraction_direction(direction);
        Ok((
            Arc::new(detector),
            Arc::new(BriskExtractor {
                extractor: std::sync::Mutex::new(extractor),
            }),
        ))
    }
}
