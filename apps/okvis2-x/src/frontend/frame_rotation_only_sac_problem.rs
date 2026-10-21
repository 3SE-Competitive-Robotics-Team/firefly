/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Rotation-only sample consensus with symmetric variance-weighted bearing scores.

use crate::frontend::frame_relative_adapter::FrameRelativeAdapter;
use firefly_error::Result;
use nalgebra::Matrix3;

/// `OpenGV`'s inherited solver: `twopt_rotationOnly` for a sample and
/// `rotationOnly` for all inliers. The inlier refit ignores the input model.
pub trait RotationOnlySolver {
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn compute_model_coefficients(
        &self,
        adapter: &FrameRelativeAdapter,
        indices: &[usize],
    ) -> Result<Option<Matrix3<f64>>>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn optimize_model_coefficients(
        &self,
        adapter: &FrameRelativeAdapter,
        inliers: &[usize],
        model: &Matrix3<f64>,
    ) -> Result<Matrix3<f64>>;
}
pub struct FrameRotationOnlySacProblem<'a, S: RotationOnlySolver> {
    adapter: &'a FrameRelativeAdapter,
    solver: &'a S,
    indices: Vec<usize>,
}
impl<'a, S: RotationOnlySolver> FrameRotationOnlySacProblem<'a, S> {
    pub fn new(adapter: &'a FrameRelativeAdapter, solver: &'a S) -> Self {
        Self::with_indices(
            adapter,
            (0..adapter.number_correspondences()).collect(),
            solver,
        )
    }
    pub fn with_indices(
        adapter: &'a FrameRelativeAdapter,
        indices: Vec<usize>,
        solver: &'a S,
    ) -> Self {
        Self {
            adapter,
            solver,
            indices,
        }
    }
    #[must_use]
    pub fn indices(&self) -> &[usize] {
        &self.indices
    }
    #[must_use]
    pub const fn sample_size(&self) -> usize {
        2
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn compute_model_coefficients(&self, indices: &[usize]) -> Result<Option<Matrix3<f64>>> {
        self.solver
            .compute_model_coefficients(self.adapter, indices)
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn optimize_model_coefficients(
        &self,
        inliers: &[usize],
        model: &Matrix3<f64>,
    ) -> Result<Matrix3<f64>> {
        self.solver
            .optimize_model_coefficients(self.adapter, inliers, model)
    }
    pub fn get_selected_distances_to_model(
        &self,
        model: &Matrix3<f64>,
        indices: &[usize],
        scores: &mut Vec<f64>,
    ) {
        for &index in indices {
            let f1 = self.adapter.bearing_vector1(index);
            let f2 = self.adapter.bearing_vector2(index);
            let error1 = model * f2 - f1;
            let error2 = model.transpose() * f1 - f2;
            scores.push(
                error1.norm_squared() * 0.5 / self.adapter.sigma_angle1(index)
                    + error2.norm_squared() * 0.5 / self.adapter.sigma_angle2(index),
            );
        }
    }
}

impl<B> RotationOnlySolver for super::frame_absolute_pose_sac_problem::OpenGvSolver<B> {
    fn compute_model_coefficients(
        &self,
        adapter: &FrameRelativeAdapter,
        indices: &[usize],
    ) -> Result<Option<Matrix3<f64>>> {
        if indices.len() < 2
            || indices
                .iter()
                .any(|&i| i >= adapter.number_correspondences())
        {
            return Ok(None);
        }
        Ok(Some(
            firefly_opengv::relative_pose::methods::twopt_rotation_only(adapter, indices),
        ))
    }
    fn optimize_model_coefficients(
        &self,
        adapter: &FrameRelativeAdapter,
        inliers: &[usize],
        _model: &Matrix3<f64>,
    ) -> Result<Matrix3<f64>> {
        Ok(firefly_opengv::relative_pose::methods::rotation_only(
            adapter, inliers,
        ))
    }
}
