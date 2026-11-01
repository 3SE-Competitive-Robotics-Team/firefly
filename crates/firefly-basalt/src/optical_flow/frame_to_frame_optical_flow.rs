/*
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
//! Translation of `include/basalt/optical_flow/frame_to_frame_optical_flow.h`
//! at official Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
//! The factory instantiates the original float tracker for each sampling pattern.
use super::{
    AffineCompact2, KeypointId, OpticalFlowInput, OpticalFlowResult, patch::OpticalFlowPatch,
    patterns::Pattern,
};
use crate::image::image_pyr::ManagedImagePyr;
use crate::utils::sophus_utils::so3_inverse;
use crate::utils::sophus_utils::so3_product;
use crate::utils::{
    common_types::KeypointsData,
    keypoints::{compute_essential, detect_keypoints},
    vio_config::VioConfig,
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::image::image::Image;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Matrix2, Matrix4, SVector, Vector2, Vector3, Vector4};
use rayon::prelude::*;
use std::{collections::BTreeMap, marker::PhantomData, sync::Arc};

#[cfg(test)]
#[path = "frame_to_frame_tests.rs"]
mod tests;

/// Stateful, frame-to-frame tracking core, separated from queue ownership.
pub struct FrameToFrameOpticalFlow<P: Pattern<f32, N>, const N: usize> {
    t_ns: i64,
    frame_counter: usize,
    last_keypoint_id: KeypointId,
    config: VioConfig,
    calib: Calibration<f32>,
    transforms: Option<Arc<OpticalFlowResult>>,
    pyramid: Vec<ManagedImagePyr<u16>>,
    essential: Matrix4<f32>,
    pattern: PhantomData<P>,
}
impl<P: Pattern<f32, N> + Sync, const N: usize> FrameToFrameOpticalFlow<P, N> {
    /// # Errors
    /// Rejects inconsistent camera arrays or invalid loop/grid parameters.
    pub fn new(config: VioConfig, calib: &Calibration<f64>) -> Result<Self> {
        if calib.intrinsics.is_empty()
            || calib.t_i_c.len() != calib.intrinsics.len()
            || config.optical_flow_levels < 0
            || config.optical_flow_max_iterations < 0
            || config.optical_flow_detection_grid_size <= 0
            || config.optical_flow_skip_frames <= 0
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid optical-flow calibration or configuration",
            ));
        }
        let essential = if calib.intrinsics.len() > 1 {
            let inverse_rotation = so3_inverse(&calib.t_i_c[0].rotation);
            let relative = nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::from(
                    -(inverse_rotation * calib.t_i_c[0].translation.vector)
                        + inverse_rotation * calib.t_i_c[1].translation.vector,
                ),
                so3_product(&inverse_rotation, &calib.t_i_c[1].rotation),
            );
            compute_essential(&relative).map(|v| v as f32)
        } else {
            Matrix4::zeros()
        };
        Ok(Self {
            t_ns: -1,
            frame_counter: 0,
            last_keypoint_id: 0,
            config,
            calib: calib.cast(),
            transforms: None,
            pyramid: Vec::new(),
            essential,
            pattern: PhantomData,
        })
    }

    /// # Errors
    /// Rejects camera-count mismatches and invalid detection image dimensions.
    /// Missing image pointers preserve the upstream early-return behavior.
    /// # Panics
    /// Requires image dimensions large enough for all configured pyramid levels.
    pub fn process_frame(
        &mut self,
        input: Arc<OpticalFlowInput>,
    ) -> Result<Option<Arc<OpticalFlowResult>>> {
        if input.img_data.iter().any(|v| v.img.is_none()) {
            return Ok(None);
        }
        if input.img_data.len() != self.calib.intrinsics.len() {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "image count differs from camera calibration",
            ));
        }
        let levels = usize::try_from(self.config.optical_flow_levels).unwrap();
        let pyramid: Vec<_> = input
            .img_data
            .par_iter()
            .map(|v| ManagedImagePyr::new(v.img.as_ref().unwrap(), levels))
            .collect();
        let mut next = OpticalFlowResult {
            t_ns: input.t_ns,
            observations: vec![BTreeMap::new(); self.calib.intrinsics.len()],
            input_images: Some(input),
            ..OpticalFlowResult::default()
        };
        if self.t_ns >= 0 {
            let previous = self.transforms.as_ref().unwrap();
            for (i, observations) in next.observations.iter_mut().enumerate() {
                *observations =
                    self.track_points(&self.pyramid[i], &pyramid[i], &previous.observations[i]);
            }
        }
        self.t_ns = next.t_ns;
        self.pyramid = pyramid;
        self.add_points(&mut next)?;
        self.filter_points(&mut next);
        let next = Arc::new(next);
        self.transforms = Some(Arc::clone(&next));
        let publish = self
            .frame_counter
            .is_multiple_of(usize::try_from(self.config.optical_flow_skip_frames).unwrap());
        self.frame_counter += 1;
        Ok(publish.then_some(next))
    }

    #[must_use]
    pub fn track_points(
        &self,
        first: &ManagedImagePyr<u16>,
        second: &ManagedImagePyr<u16>,
        transforms: &BTreeMap<KeypointId, AffineCompact2>,
    ) -> BTreeMap<KeypointId, AffineCompact2> {
        transforms
            .par_iter()
            .filter_map(|(&id, &source)| {
                let mut target = source;
                if !self.track_point(first, second, &source, &mut target) {
                    return None;
                }
                let mut recovered = target;
                if !self.track_point(second, first, &target, &mut recovered) {
                    return None;
                }
                ((source.translation - recovered.translation).norm_squared()
                    < self.config.optical_flow_max_recovered_dist2)
                    .then_some((id, target))
            })
            .collect()
    }

    #[must_use]
    pub fn track_point(
        &self,
        old_pyramid: &ManagedImagePyr<u16>,
        pyramid: &ManagedImagePyr<u16>,
        old_transform: &AffineCompact2,
        transform: &mut AffineCompact2,
    ) -> bool {
        transform.linear = Matrix2::identity();
        let mut valid = true;
        for level in (0..=self.config.optical_flow_levels).rev() {
            if !valid {
                break;
            }
            let scale = (1_i32 << level) as f32;
            transform.translation /= scale;
            let patch = OpticalFlowPatch::<f32, P, N>::new(
                &old_pyramid.lvl(level as usize),
                &(old_transform.translation / scale),
            );
            valid &= patch.valid;
            if valid {
                valid &= self.track_point_at_level(&pyramid.lvl(level as usize), &patch, transform);
            }
            transform.translation *= scale;
        }
        transform.linear = old_transform.linear * transform.linear;
        valid
    }

    #[must_use]
    pub fn track_point_at_level(
        &self,
        image: &Image<'_, u16>,
        patch: &OpticalFlowPatch<f32, P, N>,
        transform: &mut AffineCompact2,
    ) -> bool {
        let mut valid = true;
        for _ in 0..self.config.optical_flow_max_iterations {
            if !valid {
                break;
            }
            let mut transformed = transform.linear * P::pattern2();
            for mut p in transformed.column_iter_mut() {
                p += transform.translation;
            }
            let mut residual = SVector::zeros();
            valid &= patch.residual(image, &transformed, &mut residual);
            if valid {
                let increment = super::patch::patch_increment(&patch.h_se2_inv_j_se2_t, &residual);
                valid &= increment.iter().all(|v| v.is_finite()) && increment.amax() < 1e6;
                if valid {
                    let update = se2_exp(&increment);
                    apply_se2_update(transform, &update);
                    valid &= image.in_bounds(&transform.translation, 2.0);
                }
            }
        }
        valid
    }

    fn add_points(&mut self, result: &mut OpticalFlowResult) -> Result<()> {
        let existing: Vec<Vector2<f64>> = result.observations[0]
            .values()
            .map(|t| t.translation.map(f64::from))
            .collect();
        let mut keypoints = KeypointsData::default();
        detect_keypoints(
            &self.pyramid[0].lvl(0),
            &mut keypoints,
            usize::try_from(self.config.optical_flow_detection_grid_size).unwrap(),
            1,
            &existing,
        )?;
        let mut new_points = BTreeMap::new();
        for p in keypoints.corners {
            let transform = AffineCompact2 {
                translation: p.map(|v| v as f32),
                ..AffineCompact2::default()
            };
            result.observations[0].insert(self.last_keypoint_id, transform);
            new_points.insert(self.last_keypoint_id, transform);
            self.last_keypoint_id += 1;
        }
        if self.calib.intrinsics.len() > 1 {
            let stereo = self.track_points(&self.pyramid[0], &self.pyramid[1], &new_points);
            for (id, transform) in stereo {
                result.observations[1].entry(id).or_insert(transform);
            }
        }
        Ok(())
    }

    fn filter_points(&self, result: &mut OpticalFlowResult) {
        if self.calib.intrinsics.len() < 2 {
            return;
        }
        let mut remove = Vec::new();
        for (&id, transform1) in &result.observations[1] {
            if let Some(transform0) = result.observations[0].get(&id) {
                let mut a = Vector4::zeros();
                let mut b = Vector4::zeros();
                let first =
                    self.calib.intrinsics[0].unproject(&transform0.translation, &mut a, None);
                let second =
                    self.calib.intrinsics[1].unproject(&transform1.translation, &mut b, None);
                if !first
                    || !second
                    || f64::from(a.dot(&(self.essential * b)).abs())
                        > f64::from(self.config.optical_flow_epipolar_error)
                {
                    remove.push(id);
                }
            }
        }
        for id in remove {
            result.observations[1].remove(&id);
        }
    }
}

// Sophus 1.24.6 SE2::exp and SO2::normalize from the official vcpkg dependency.
// Copyright 2011-2017 Hauke Strasdat, 2012-2017 Steven Lovegrove. MIT license.
pub(super) fn se2_exp(increment: &Vector3<f32>) -> AffineCompact2 {
    let theta = increment.z;
    let (sin, cos) = theta.sin_cos();
    // SO2::exp constructs SO2(real,imag), whose constructor normalizes.
    let norm = cos.hypot(sin);
    let (sin, cos) = (sin / norm, cos / norm);
    let (sin_by_theta, one_minus_cos_by_theta) = if theta.abs() < 1e-5 {
        let squared = theta * theta;
        (
            1.0 - (1.0 / 6.0) * squared,
            0.5 * theta - (1.0 / 24.0) * theta * squared,
        )
    } else {
        (sin / theta, (1.0 - cos) / theta)
    };
    AffineCompact2 {
        linear: Matrix2::new(cos, -sin, sin, cos),
        translation: Vector2::new(
            sin_by_theta * increment.x - one_minus_cos_by_theta * increment.y,
            one_minus_cos_by_theta * increment.x + sin_by_theta * increment.y,
        ),
    }
}

/// Original `AffineCompact2 *= SE2::matrix()` multiplies the full homogeneous
/// matrix. Preserve its three-term scalar reduction for the translation.
pub(super) fn apply_se2_update(transform: &mut AffineCompact2, update: &AffineCompact2) {
    for row in 0..2 {
        transform.translation[row] = transform.linear[(row, 0)] * update.translation.x
            + (transform.linear[(row, 1)] * update.translation.y + transform.translation[row]);
    }
    transform.linear *= update.linear;
}
