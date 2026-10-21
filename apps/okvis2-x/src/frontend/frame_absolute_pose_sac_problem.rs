/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Absolute-pose sample consensus with the source's uncertainty-normalized score.

use crate::frontend::frame_noncentral_absolute_adapter::AbsoluteAdapter;
use firefly_error::Result;
use nalgebra::SMatrix;

pub type Model = SMatrix<f64, 3, 4>;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Algorithm {
    TwoPt = 0,
    Kneip = 1,
    Gao = 2,
    Epnp = 3,
    Gp3p = 4,
}

/// `OpenGV`'s inherited `AbsolutePoseSacProblem` capability. compute must dispatch to
/// `p2p/p3p_kneip/p3p_gao/epnp/gp3p`, transform central solutions to the body frame,
/// and disambiguate multiple solutions with the fourth sample as the base class
/// does. None represents the base class's false result. optimize runs
/// `absolute_pose::optimize_nonlinear` using the adapter's current pose prior.
pub trait AbsolutePoseSolver {
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn compute_model_coefficients(
        &self,
        adapter: &dyn AbsoluteAdapter,
        algorithm: Algorithm,
        indices: &[usize],
    ) -> Result<Option<Model>>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn optimize_nonlinear(&self, adapter: &dyn AbsoluteAdapter, inliers: &[usize])
    -> Result<Model>;
}

pub struct FrameAbsolutePoseSacProblem<'a, A: AbsoluteAdapter, S: AbsolutePoseSolver> {
    adapter: &'a mut A,
    solver: &'a S,
    algorithm: Algorithm,
    indices: Vec<usize>,
}
impl<'a, A: AbsoluteAdapter, S: AbsolutePoseSolver> FrameAbsolutePoseSacProblem<'a, A, S> {
    pub fn new(adapter: &'a mut A, algorithm: Algorithm, solver: &'a S) -> Self {
        let indices = (0..adapter.number_correspondences()).collect();
        Self {
            adapter,
            solver,
            algorithm,
            indices,
        }
    }
    pub fn with_indices(
        adapter: &'a mut A,
        algorithm: Algorithm,
        indices: Vec<usize>,
        solver: &'a S,
    ) -> Self {
        Self {
            adapter,
            solver,
            algorithm,
            indices,
        }
    }
    #[must_use]
    pub fn indices(&self) -> &[usize] {
        &self.indices
    }
    #[must_use]
    pub fn sample_size(&self) -> usize {
        match self.algorithm {
            Algorithm::TwoPt => 2,
            Algorithm::Epnp => 6,
            _ => 4,
        }
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn compute_model_coefficients(&self, indices: &[usize]) -> Result<Option<Model>> {
        self.solver
            .compute_model_coefficients(self.adapter, self.algorithm, indices)
    }
    /// Scores are appended, matching the vector passed to the C++ method.
    pub fn get_selected_distances_to_model(
        &self,
        model: &Model,
        indices: &[usize],
        scores: &mut Vec<f64>,
    ) {
        let inverse_rotation = model.fixed_view::<3, 3>(0, 0).transpose();
        let inverse_translation = -inverse_rotation * model.column(3);
        scores.reserve(indices.len());
        for &index in indices {
            let body_reprojection =
                inverse_rotation * self.adapter.point(index) + inverse_translation;
            let reprojection = self.adapter.cam_rotation(index).transpose()
                * (body_reprojection - self.adapter.cam_offset(index));
            // This explicit division preserves the source score's zero-norm behavior.
            let reprojection = reprojection / reprojection.norm();
            let error = reprojection - self.adapter.bearing_vector(index);
            scores.push(error.norm_squared() / self.adapter.sigma_angle(index));
        }
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        model: &Model,
    ) -> Result<Model> {
        self.adapter.set_translation(model.column(3).into_owned());
        self.adapter
            .set_rotation(model.fixed_view::<3, 3>(0, 0).into_owned());
        self.solver.optimize_nonlinear(self.adapter, inliers)
    }
}

/// `OpenGV` SAC dispatch with the concrete pure Rust numerical backend.
#[derive(Clone, Copy, Debug, Default)]
pub struct OpenGvSolver<B = firefly_opengv::optimization_functor::RustNumericalBackend> {
    pub backend: B,
}
struct OpenGvAbsoluteAdapter<'a> {
    source: &'a dyn AbsoluteAdapter,
    translation: nalgebra::Vector3<f64>,
    rotation: nalgebra::Matrix3<f64>,
}
impl<'a> OpenGvAbsoluteAdapter<'a> {
    fn new(source: &'a dyn AbsoluteAdapter) -> Self {
        Self {
            source,
            translation: source.translation(),
            rotation: source.rotation(),
        }
    }
}
impl firefly_opengv::absolute_pose::absolute_adapter_base::AbsoluteAdapterBase
    for OpenGvAbsoluteAdapter<'_>
{
    fn bearing_vector(&self, i: usize) -> nalgebra::Vector3<f64> {
        self.source.bearing_vector(i)
    }
    fn point(&self, i: usize) -> nalgebra::Vector3<f64> {
        self.source.point(i)
    }
    fn cam_offset(&self, i: usize) -> nalgebra::Vector3<f64> {
        self.source.cam_offset(i)
    }
    fn cam_rotation(&self, i: usize) -> nalgebra::Matrix3<f64> {
        self.source.cam_rotation(i)
    }
    fn weight(&self, i: usize) -> f64 {
        self.source.weight(i)
    }
    fn number_correspondences(&self) -> usize {
        self.source.number_correspondences()
    }
    fn t(&self) -> nalgebra::Vector3<f64> {
        self.translation
    }
    fn r(&self) -> nalgebra::Matrix3<f64> {
        self.rotation
    }
    fn set_t(&mut self, t: nalgebra::Vector3<f64>) {
        self.translation = t;
    }
    fn set_r(&mut self, r: nalgebra::Matrix3<f64>) {
        self.rotation = r;
    }
}
impl<B: firefly_opengv::optimization_functor::NumericalBackend> AbsolutePoseSolver
    for OpenGvSolver<B>
{
    fn compute_model_coefficients(
        &self,
        adapter: &dyn AbsoluteAdapter,
        algorithm: Algorithm,
        indices: &[usize],
    ) -> Result<Option<Model>> {
        use firefly_opengv::{
            sac::sample_consensus_problem::SampleConsensusProblem,
            sac_problems::absolute_pose::absolute_pose_sac_problem::{
                AbsolutePoseSacProblem, Algorithm as GvAlgorithm,
            },
        };
        let mut adapter = OpenGvAbsoluteAdapter::new(adapter);
        let algorithm = match algorithm {
            Algorithm::TwoPt => GvAlgorithm::Twopt,
            Algorithm::Kneip => GvAlgorithm::Kneip,
            Algorithm::Gao => GvAlgorithm::Gao,
            Algorithm::Epnp => GvAlgorithm::Epnp,
            Algorithm::Gp3p => GvAlgorithm::Gp3p,
        };
        let mut problem =
            AbsolutePoseSacProblem::new(&mut adapter, algorithm, &self.backend, false);
        Ok(problem.compute_model_coefficients(indices))
    }
    fn optimize_nonlinear(
        &self,
        adapter: &dyn AbsoluteAdapter,
        inliers: &[usize],
    ) -> Result<Model> {
        firefly_opengv::absolute_pose::methods::optimize_nonlinear(
            &OpenGvAbsoluteAdapter::new(adapter),
            inliers,
            &self.backend,
        )
        .ok_or_else(|| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Convergence,
                "OpenGV absolute-pose refinement failed",
            )
            .with_operation("OpenGvSolver::absolute_pose")
        })
    }
}
