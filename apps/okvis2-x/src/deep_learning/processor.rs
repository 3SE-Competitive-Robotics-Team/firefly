/* OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! Callback wiring and input routing between SLAM, deep inference and Supereight.
use super::{
    deep_learning_processor::{
        CameraMeasurements, DeepLearningProcessor, DisplayImages, Images, UpdatedStates,
    },
    utils::invalid,
};
use crate::{
    kinematics::transformation::Transformation,
    runtime::vi_interface::{State, TrackingState},
};
use firefly_error::Result;
use jiff::Timestamp;
use nalgebra::Vector3;
use purecv::core::DynamicMatrix;
use std::{
    collections::{BTreeMap, HashMap},
    sync::{Arc, RwLock},
    thread,
    time::Duration,
};
pub type OptimizedGraphCallback =
    Arc<dyn Fn(&State, &TrackingState, UpdatedStates) -> Result<()> + Send + Sync>;
pub type AlignmentPublishCallback =
    Arc<dyn Fn(Timestamp, &Transformation, &[Vector3<f32>], bool) -> Result<()> + Send + Sync>;
pub type AlignCallback<Map> = Arc<
    dyn Fn(
            Option<Arc<Map>>,
            Option<Arc<Map>>,
            u64,
            u64,
            &mut Vec<Vector3<f32>>,
            Vec<f32>,
        ) -> Result<bool>
        + Send
        + Sync,
>;
pub type SubmapCallback<Map, Objects> = Arc<
    dyn Fn(HashMap<u64, Transformation>, HashMap<u64, Arc<Map>>, Arc<Objects>) -> Result<()>
        + Send
        + Sync,
>;
pub type FieldCallback<Submap> =
    Arc<dyn Fn(&State, &HashMap<u64, Submap>) -> Result<()> + Send + Sync>;
pub type ImageMap = BTreeMap<usize, Arc<DynamicMatrix>>;
/// Graph optimization and sensor ingestion capability of `ThreadedSlam`.
/// Implementations invoke registered callbacks without holding a method-entry lock.
pub trait SlamBackend<Map>: Send + Sync {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_optimized_graph_callback(&self, callback: OptimizedGraphCallback) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_blocking(&self, value: bool) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn display(&self, images: &mut DisplayImages) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn stop_threading(&self) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn process_frame(&self) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn add_images(&self, stamp: Timestamp, images: &ImageMap, depth: &ImageMap) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn add_depth_measurement(
        &self,
        stamp: Timestamp,
        depth: &DynamicMatrix,
        sigma: Option<&DynamicMatrix>,
    ) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn add_imu_measurement(
        &self,
        stamp: Timestamp,
        alpha: &Vector3<f64>,
        omega: &Vector3<f64>,
    ) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn add_lidar_measurement(&self, stamp: Timestamp, ray: &Vector3<f64>) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn add_submap_alignment_constraints(
        &self,
        a: Option<Arc<Map>>,
        b: Option<Arc<Map>>,
        a_id: u64,
        b_id: u64,
        points: &mut Vec<Vector3<f32>>,
        sensor_error: Vec<f32>,
    ) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_alignment_publish_callback(&self, callback: AlignmentPublishCallback) -> Result<()>;
}
/// Supereight integration and publication capability; concrete map, submap and
/// object storage types stay visible in callback signatures.
pub trait SubmappingBackend: Send + Sync {
    type Map: Send + Sync + 'static;
    type Submap: Send + Sync + 'static;
    type Objects: Send + Sync + 'static;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn start(&self) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_blocking(&self, value: bool) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_t_bs(&self, t_bs: &Transformation) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn collect_info(&self) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn finished_integrating(&self) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_finished(&self) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn add_depth_measurement(&self, frames: &mut CameraMeasurements) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn state_update_callback(
        &self,
        state: &State,
        tracking: &TrackingState,
        updated: UpdatedStates,
    ) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_align_callback(&self, callback: AlignCallback<Self::Map>) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_submap_callback(&self, callback: SubmapCallback<Self::Map, Self::Objects>)
    -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_field_slice_callback(&self, callback: FieldCallback<Self::Submap>) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn set_alignment_publish_callback(&self, callback: AlignmentPublishCallback) -> Result<()>;
}
pub struct Processor<S, M>
where
    M: SubmappingBackend,
    S: SlamBackend<M::Map>,
{
    pub slam: Arc<S>,
    pub se_interface: Arc<M>,
    deep_learning: Arc<dyn DeepLearningProcessor>,
    external_callback: Arc<RwLock<Option<OptimizedGraphCallback>>>,
}
impl<S, M> Processor<S, M>
where
    M: SubmappingBackend + 'static,
    S: SlamBackend<M::Map> + 'static,
{
    /// Backend instances are fully configured with the source constructor's VI,
    /// vocabulary, map/data and submap configurations before callback registration.
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn new(
        slam: Arc<S>,
        mapping: Arc<M>,
        deep_learning: Box<dyn DeepLearningProcessor>,
    ) -> Result<Self> {
        let fusion = deep_learning.is_depth_fusion();
        let deep_learning: Arc<dyn DeepLearningProcessor> = Arc::from(deep_learning);
        let external_callback = Arc::new(RwLock::new(None::<OptimizedGraphCallback>));
        let dl = deep_learning.clone();
        let map = mapping.clone();
        let external = external_callback.clone();
        slam.set_optimized_graph_callback(Arc::new(move |state, tracking, updated| {
            map.state_update_callback(state, tracking, updated.clone())?;
            if fusion {
                dl.state_update_callback(state, tracking, updated.clone())?;
            }
            if let Some(callback) = external.read().unwrap().as_ref() {
                callback(state, tracking, updated)?;
            }
            Ok(())
        }))?;
        if fusion {
            let slam = slam.clone();
            deep_learning.set_live_depth_image_callback(Arc::new(move |stamp, depth, sigma| {
                slam.add_depth_measurement(stamp, depth, sigma)
            }));
        }
        let estimator = slam.clone();
        let map = mapping.clone();
        deep_learning.set_image_callback(Arc::new(move |frames| {
            if fusion {
                return map.add_depth_measurement(frames);
            }
            let mut estimator_add = false;
            for entries in frames.values() {
                for frame in entries {
                    if let Some(depth) = frame
                        .measurement
                        .depth_image
                        .as_ref()
                        .filter(|image| image.rows() > 0 && image.cols() > 0)
                    {
                        // Preserve the source's sigma-presence branch exactly.
                        estimator_add = if frame
                            .measurement
                            .sigma_image
                            .as_ref()
                            .is_some_and(|image| image.rows() > 0 && image.cols() > 0)
                        {
                            estimator.add_depth_measurement(frame.time_stamp, depth, None)?
                        } else {
                            estimator.add_depth_measurement(
                                frame.time_stamp,
                                depth,
                                frame.measurement.sigma_image.as_deref(),
                            )?
                        };
                    }
                }
            }
            let map_add = map.add_depth_measurement(frames)?;
            Ok(estimator_add && map_add)
        }));
        let estimator = slam.clone();
        mapping.set_align_callback(Arc::new(move |a, b, a_id, b_id, points, error| {
            estimator.add_submap_alignment_constraints(a, b, a_id, b_id, points, error)
        }))?;
        mapping.start()?;
        Ok(Self {
            slam,
            se_interface: mapping,
            deep_learning,
            external_callback,
        })
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn display(&self, images: &mut DisplayImages) -> Result<()> {
        self.slam.display(images)?;
        if !images.is_empty() {
            self.deep_learning.display(images)?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn collect_info(&self) -> Result<()> {
        self.se_interface.collect_info()
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn finish(&self) -> Result<()> {
        self.slam.stop_threading()?;
        while !self.se_interface.finished_integrating()? {
            thread::sleep(Duration::from_millis(1));
        }
        self.se_interface.set_finished()
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn set_blocking(&self, value: bool) -> Result<()> {
        self.slam.set_blocking(value)?;
        self.deep_learning.set_blocking(value);
        self.se_interface.set_blocking(value)
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn set_t_bs(&self, transform: &Transformation) -> Result<()> {
        self.se_interface.set_t_bs(transform)
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn process_frame(&self) -> Result<bool> {
        self.slam.process_frame()
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn set_submap_callback(&self, callback: SubmapCallback<M::Map, M::Objects>) -> Result<()> {
        self.se_interface.set_submap_callback(callback)
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn set_field_slice_callback(&self, callback: FieldCallback<M::Submap>) -> Result<()> {
        self.se_interface.set_field_slice_callback(callback)
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn set_alignment_publish_callback(&self, callback: AlignmentPublishCallback) -> Result<()> {
        self.slam.set_alignment_publish_callback(callback.clone())?;
        self.se_interface.set_alignment_publish_callback(callback)
    }
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn set_optimized_graph_callback(&self, callback: OptimizedGraphCallback) {
        *self.external_callback.write().unwrap() = Some(callback);
    }
    /// The inherited single-timestamp overload returns false in the source.
    #[must_use]
    pub fn add_images_at(&self, _stamp: Timestamp, _images: &ImageMap, _depth: &ImageMap) -> bool {
        false
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn add_images(&self, images: &Images, depths: &Images) -> Result<bool> {
        let stamp = images
            .first_key_value()
            .ok_or_else(|| invalid("image batch is empty", "Processor::add_images"))?
            .1
            .0;
        let ir = images
            .iter()
            .map(|(&id, (_, image))| (id, image.clone()))
            .collect();
        let depth = depths
            .iter()
            .map(|(&id, (_, image))| (id, image.clone()))
            .collect();
        let result = self.slam.add_images(stamp, &ir, &depth)?;
        let accepted = self.deep_learning.add_images(images, depths)?;
        if !accepted {
            log::debug!("deep-learning image batch dropped at {stamp}");
        }
        Ok(result)
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn add_imu_measurement(
        &self,
        stamp: Timestamp,
        alpha: &Vector3<f64>,
        omega: &Vector3<f64>,
    ) -> Result<bool> {
        self.slam.add_imu_measurement(stamp, alpha, omega)
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn add_lidar_measurement(&self, stamp: Timestamp, ray: &Vector3<f64>) -> Result<bool> {
        self.slam.add_lidar_measurement(stamp, ray)
    }
    /// The public processor overload intentionally omits sigma when forwarding.
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn add_depth_measurement(
        &self,
        stamp: Timestamp,
        depth: &DynamicMatrix,
        _sigma: Option<&DynamicMatrix>,
    ) -> Result<bool> {
        self.slam.add_depth_measurement(stamp, depth, None)
    }
}
