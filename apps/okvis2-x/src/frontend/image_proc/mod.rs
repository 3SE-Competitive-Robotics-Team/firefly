//! Camera-local feature processing. Only owned images and feature values cross threads.
pub(crate) mod brisk;
mod pipeline;
mod worker;

pub(crate) use pipeline::Pipeline;

use crate::{
    cameras::camera_base::CameraBase,
    frames::frame::{DescriptorExtractor, FeatureDetector, Frame, FrameFeatures, Image},
};
use brisk::{BriskFeatureFactory, BriskSettings};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;
use std::sync::Arc;

pub(crate) struct Input {
    pub camera: usize,
    pub image: Image,
    pub geometry: Arc<dyn CameraBase>,
    /// Gravity direction in this camera's coordinates, computed from the predicted pose.
    pub direction: Vector3<f32>,
}

pub(crate) struct Output {
    pub camera: usize,
    pub features: FrameFeatures,
}

/// Construct and use inside one worker; detector/extractor need not implement Send.
struct Processor {
    geometry: Arc<dyn CameraBase>,
    detector: Arc<dyn FeatureDetector>,
    extractor: Arc<dyn DescriptorExtractor>,
}

impl Processor {
    fn new(
        factory: &impl BriskFeatureFactory,
        geometry: Arc<dyn CameraBase>,
        settings: &BriskSettings,
        direction: Vector3<f32>,
    ) -> Result<Self> {
        let (detector, extractor) = factory.create(geometry.as_ref(), settings, direction)?;
        Ok(Self {
            geometry,
            detector,
            extractor,
        })
    }

    #[fastrace::trace]
    fn process(&mut self, input: Input) -> Result<Output> {
        if !Arc::ptr_eq(&self.geometry, &input.geometry) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "camera geometry changed while extractor is active",
            )
            .with_operation("image_proc"));
        }
        self.extractor.set_extraction_direction(input.direction);
        let mut frame = Frame::new(
            input.image,
            input.geometry,
            self.detector.clone(),
            self.extractor.clone(),
            None,
        );
        frame.detect()?;
        frame.describe()?;
        frame.compute_back_projections();
        Ok(Output {
            camera: input.camera,
            features: frame.take_features(),
        })
    }
}

fn error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into()).with_operation("image_proc")
}

#[cfg(test)]
mod tests;
