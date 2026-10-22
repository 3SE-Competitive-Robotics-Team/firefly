/* OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! Match projection, tracking overlays and keypoint orientation visualization.
use crate::{
    cameras::{camera_base::ProjectionStatus, n_camera_system::NCameraSystem},
    config::parameters::ViParameters,
    deep_learning::utils::{VisualizationRaster, add_weighted, byte_image, gray_to_bgr, invalid},
    frames::{frame_typedefs::ObservationVector, multi_frame::MultiFrame},
    kinematics::transformation::Transformation,
};
use firefly_error::Result;
use nalgebra::Vector2;
use purecv::{core::Matrix, features2d::KeyPoint};
use std::sync::Arc;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingQuality {
    Good,
    Marginal,
    Lost,
}
pub struct VisualizationData {
    pub observations: ObservationVector,
    pub current_frames: Arc<MultiFrame>,
    pub t_ws: Transformation,
    pub t_sc_i: Vec<Transformation>,
    pub is_keyframe: bool,
    pub recognised_place: bool,
    pub tracking_quality: TrackingQuality,
}
impl VisualizationData {
    #[must_use]
    pub fn new(
        current_frames: Arc<MultiFrame>,
        t_ws: Transformation,
        t_sc_i: Vec<Transformation>,
    ) -> Self {
        Self {
            observations: Vec::new(),
            current_frames,
            t_ws,
            t_sc_i,
            is_keyframe: false,
            recognised_place: false,
            tracking_quality: TrackingQuality::Good,
        }
    }
}
pub struct ViVisualizer {
    camera_system: NCameraSystem,
    raster: Arc<dyn VisualizationRaster>,
}
impl ViVisualizer {
    pub fn new(parameters: &ViParameters, raster: Arc<dyn VisualizationRaster>) -> Self {
        Self {
            camera_system: parameters.n_camera_system.clone(),
            raster,
        }
    }
    pub fn init(&mut self, parameters: &ViParameters) {
        self.camera_system = parameters.n_camera_system.clone();
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn draw_matches(&self, data: &VisualizationData, camera: usize) -> Result<Matrix<u8>> {
        let t_sc = data
            .t_sc_i
            .get(camera)
            .ok_or_else(|| invalid("camera extrinsics missing", "ViVisualizer::draw_matches"))?;
        let t_cw = data.t_ws.compose(t_sc).inverse();
        let frame = &data.current_frames;
        let image = frame
            .image(camera)
            .ok_or_else(|| invalid("frame image absent", "ViVisualizer::draw_matches"))?;
        let mut current = gray_to_bgr(byte_image(image)?)?;
        self.annotate_tracking(data, &mut current)?;
        for observation in &data.observations {
            if observation.camera_idx != camera {
                continue;
            }
            let keypoint = observation.keypoint_measurement.as_ref().ok_or_else(|| {
                invalid(
                    "keypoint coordinates uninitialized",
                    "ViVisualizer::draw_matches",
                )
            })?;
            let landmark = observation.landmark_w.as_ref().ok_or_else(|| {
                invalid(
                    "landmark coordinates uninitialized",
                    "ViVisualizer::draw_matches",
                )
            })?;
            let mut colour = if observation.landmark_id != 0 {
                [255, 0, 0]
            } else {
                [0, 0, 255]
            };
            if landmark[3].abs() > 1.0e-8 {
                colour = if observation.is_initialized {
                    if observation.classification == 10 || observation.classification == 11 {
                        [0, 80, 0]
                    } else {
                        [0, 255, 0]
                    }
                } else if observation.classification == 10 || observation.classification == 11 {
                    [0, 50, 50]
                } else {
                    [0, 255, 255]
                };
                let camera_point = t_cw.transform_homogeneous(landmark);
                let mut projection = Vector2::zeros();
                let mut visible = frame
                    .geometry(camera)
                    .project_homogeneous(&camera_point, &mut projection)
                    == ProjectionStatus::Successful;
                if camera_point[3].abs() > 1.0e-8 && camera_point[2] / camera_point[3] < 0.03 {
                    visible = false;
                }
                if visible {
                    let p = [projection.x as f32, projection.y as f32];
                    self.raster.line(
                        &mut current,
                        p,
                        [keypoint.x as f32, keypoint.y as f32],
                        colour,
                        1,
                        true,
                    )?;
                    self.raster
                        .circle(&mut current, p, 2, [255, 255, 0], -1, true)?;
                    self.raster.text(
                        &mut current,
                        &observation.landmark_id.to_string(),
                        [p[0] + 10., p[1] + 5.],
                        0.35,
                        [255, 255, 0],
                        true,
                    )?;
                }
            }
            let radius = 0.5 * observation.keypoint_size;
            self.raster.circle(
                &mut current,
                [keypoint.x as f32, keypoint.y as f32],
                radius as i32,
                colour,
                1,
                true,
            )?;
            self.draw_keypoint_orientation(frame, observation, &mut current, radius, colour)?;
        }
        Ok(current)
    }
    fn draw_keypoint_orientation(
        &self,
        frame: &crate::frames::multi_frame::MultiFrame,
        observation: &crate::frames::frame_typedefs::Observation,
        current: &mut Matrix<u8>,
        radius: f64,
        colour: [u8; 3],
    ) -> Result<()> {
        let mut cv_keypoint = KeyPoint::default();
        if !frame.get_cv_keypoint(
            observation.camera_idx,
            observation.keypoint_idx,
            &mut cv_keypoint,
        ) {
            return Err(invalid(
                "keypoint index absent",
                "ViVisualizer::draw_matches",
            ));
        }
        let angle = cv_keypoint.angle / 180. * std::f32::consts::PI;
        self.raster.line(
            current,
            [cv_keypoint.pt.x, cv_keypoint.pt.y],
            [
                cv_keypoint.pt.x + radius as f32 * angle.cos(),
                cv_keypoint.pt.y + radius as f32 * angle.sin(),
            ],
            colour,
            1,
            true,
        )?;
        Ok(())
    }

    fn annotate_tracking(&self, data: &VisualizationData, current: &mut Matrix<u8>) -> Result<()> {
        if data.recognised_place {
            tint(current, [255, 0, 0], 0.5)?;
            self.raster.text(
                current,
                "Place recognised",
                [200., 10.],
                0.3,
                [255, 0, 0],
                true,
            )?;
        }
        if data.is_keyframe {
            tint(current, [255, 255, 0], 0.3)?;
            self.raster
                .text(current, "Keyframe", [120., 10.], 0.3, [255, 255, 0], true)?;
        }
        let (text, colour) = match data.tracking_quality {
            TrackingQuality::Lost => ("TRACKING LOST", [0, 0, 255]),
            TrackingQuality::Marginal => ("Tracking marginal", [0, 255, 255]),
            TrackingQuality::Good => ("Tracking good", [0, 255, 0]),
        };
        self.raster
            .text(current, text, [5., 10.], 0.3, colour, true)?;
        if self.camera_system.num_cameras() == 0 {
            return Err(invalid(
                "camera system is empty",
                "ViVisualizer::draw_matches",
            ));
        }
        let distortion = self.camera_system.distortion_type(0);
        for index in 1..self.camera_system.num_cameras() {
            if self.camera_system.distortion_type(index) != distortion {
                return Err(invalid(
                    "mixed frame types are not supported",
                    "ViVisualizer::draw_matches",
                ));
            }
        }
        Ok(())
    }

    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn draw_keypoints(&self, data: &VisualizationData, camera: usize) -> Result<Matrix<u8>> {
        let frame = &data.current_frames;
        let image = frame
            .image(camera)
            .ok_or_else(|| invalid("frame image absent", "ViVisualizer::draw_keypoints"))?;
        let mut output = gray_to_bgr(byte_image(image)?)?;
        for index in 0..frame.num_keypoints(camera) {
            let mut keypoint = KeyPoint::default();
            if !frame.get_cv_keypoint(camera, index, &mut keypoint) {
                return Err(invalid(
                    "keypoint index absent",
                    "ViVisualizer::draw_keypoints",
                ));
            }
            let radius = keypoint.size;
            let angle = keypoint.angle / 180. * std::f32::consts::PI;
            let p = [keypoint.pt.x, keypoint.pt.y];
            self.raster
                .circle(&mut output, p, radius as i32, [0, 255, 0], 1, false)?;
            self.raster.line(
                &mut output,
                p,
                [p[0] + radius * angle.cos(), p[1] - radius * angle.sin()],
                [0, 255, 0],
                1,
                false,
            )?;
        }
        Ok(output)
    }
}
fn tint(image: &mut Matrix<u8>, bgr: [u8; 3], weight: f64) -> Result<()> {
    let mut overlay = Matrix::new(image.rows, image.cols, 3);
    for pixel in overlay.data.as_chunks_mut::<3>().0 {
        pixel.copy_from_slice(&bgr);
    }
    *image = add_weighted(&overlay, weight, image, 1. - weight)?;
    Ok(())
}
