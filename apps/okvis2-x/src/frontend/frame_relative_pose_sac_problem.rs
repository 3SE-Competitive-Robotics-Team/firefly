/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Central relative-pose consensus with symmetric normalized reprojection scores.

use crate::frontend::frame_relative_adapter::FrameRelativeAdapter;
use firefly_error::Result;
use nalgebra::{Matrix2, SMatrix, Vector2, Vector3};

pub type Model = SMatrix<f64, 3, 4>;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Algorithm {
    Stewenius = 0,
    Nister = 1,
    SevenPt = 2,
    EightPt = 3,
}

/// The inherited `CentralRelativePoseSacProblem` algorithm dispatch, essential
/// matrix decomposition, cheirality disambiguation, and nonlinear refinement.
/// None represents the base class's unsuccessful model computation. The backend
/// must preserve `OpenGV`'s per-algorithm sample handling and solution selection.
pub trait CentralRelativePoseSolver {
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn compute_model_coefficients(
        &self,
        adapter: &mut FrameRelativeAdapter,
        algorithm: Algorithm,
        indices: &[usize],
    ) -> Result<Option<Model>>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn optimize_nonlinear(
        &self,
        adapter: &mut FrameRelativeAdapter,
        inliers: &[usize],
    ) -> Result<Model>;
}

pub struct FrameRelativePoseSacProblem<'a, S: CentralRelativePoseSolver> {
    adapter: &'a mut FrameRelativeAdapter,
    solver: &'a S,
    algorithm: Algorithm,
    indices: Vec<usize>,
}
impl<'a, S: CentralRelativePoseSolver> FrameRelativePoseSacProblem<'a, S> {
    pub fn new(adapter: &'a mut FrameRelativeAdapter, algorithm: Algorithm, solver: &'a S) -> Self {
        let indices = (0..adapter.number_correspondences()).collect();
        Self {
            adapter,
            solver,
            algorithm,
            indices,
        }
    }
    pub fn with_indices(
        adapter: &'a mut FrameRelativeAdapter,
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
            Algorithm::Stewenius | Algorithm::Nister => 8,
            Algorithm::SevenPt | Algorithm::EightPt => 9,
        }
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn compute_model_coefficients(&mut self, indices: &[usize]) -> Result<Option<Model>> {
        self.solver
            .compute_model_coefficients(self.adapter, self.algorithm, indices)
    }
    /// Updates the adapter's relative pose before triangulating, as in C++.
    /// Scores append to the caller's vector.
    pub fn get_selected_distances_to_model(
        &mut self,
        model: &Model,
        indices: &[usize],
        scores: &mut Vec<f64>,
    ) {
        let rotation = model.fixed_view::<3, 3>(0, 0).into_owned();
        let translation = model.column(3).into_owned();
        self.adapter.set_t12(translation);
        self.adapter.set_r12(rotation);
        let inverse_rotation = rotation.transpose();
        let inverse_translation = -inverse_rotation * translation;
        for &index in indices {
            let point = triangulate2(self.adapter, index);
            let reprojection2 = inverse_rotation * point + inverse_translation;
            let reprojection1 = point / point.norm();
            let reprojection2 = reprojection2 / reprojection2.norm();
            let error1 = reprojection1 - self.adapter.bearing_vector1(index);
            let error2 = reprojection2 - self.adapter.bearing_vector2(index);
            scores.push(
                error1.norm_squared() * 0.5 / self.adapter.sigma_angle1(index)
                    + error2.norm_squared() * 0.5 / self.adapter.sigma_angle2(index),
            );
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
        self.adapter.set_t12(model.column(3).into_owned());
        self.adapter
            .set_r12(model.fixed_view::<3, 3>(0, 0).into_owned());
        self.solver.optimize_nonlinear(self.adapter, inliers)
    }
}

/// `OpenGV` `external/opengv/src/triangulation/methods.cpp::triangulate2` midpoint
/// formula. The unchecked inverse retains IEEE results for singular rays.
fn triangulate2(adapter: &FrameRelativeAdapter, index: usize) -> Vector3<f64> {
    let t12 = adapter.t12();
    let f1 = adapter.bearing_vector1(index);
    let f2 = adapter.r12() * adapter.bearing_vector2(index);
    let b = Vector2::new(t12.dot(&f1), t12.dot(&f2));
    let a = Matrix2::new(f1.dot(&f1), -f1.dot(&f2), f1.dot(&f2), -f2.dot(&f2));
    let determinant = a[(0, 0)] * a[(1, 1)] - a[(0, 1)] * a[(1, 0)];
    let inverse = Matrix2::new(a[(1, 1)], -a[(0, 1)], -a[(1, 0)], a[(0, 0)]) / determinant;
    let lambda = inverse * b;
    let xm = lambda[0] * f1;
    let xn = t12 + lambda[1] * f2;
    (xm + xn) / 2.0
}

impl firefly_opengv::relative_pose::relative_adapter_base::RelativeAdapterBase
    for FrameRelativeAdapter
{
    fn bearing_vector1(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector1(i)
    }
    fn bearing_vector2(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector2(i)
    }
    fn cam_offset1(&self, i: usize) -> Vector3<f64> {
        self.cam_offset1(i)
    }
    fn cam_offset2(&self, i: usize) -> Vector3<f64> {
        self.cam_offset2(i)
    }
    fn cam_rotation1(&self, i: usize) -> nalgebra::Matrix3<f64> {
        self.cam_rotation1(i)
    }
    fn cam_rotation2(&self, i: usize) -> nalgebra::Matrix3<f64> {
        self.cam_rotation2(i)
    }
    fn weight(&self, i: usize) -> f64 {
        self.weight(i)
    }
    fn number_correspondences(&self) -> usize {
        self.number_correspondences()
    }
    fn t12(&self) -> Vector3<f64> {
        self.t12()
    }
    fn r12(&self) -> nalgebra::Matrix3<f64> {
        self.r12()
    }
    fn set_t12(&mut self, t: Vector3<f64>) {
        self.set_t12(t);
    }
    fn set_r12(&mut self, r: nalgebra::Matrix3<f64>) {
        self.set_r12(r);
    }
}
impl<B: firefly_opengv::optimization_functor::NumericalBackend> CentralRelativePoseSolver
    for super::frame_absolute_pose_sac_problem::OpenGvSolver<B>
{
    fn compute_model_coefficients(
        &self,
        adapter: &mut FrameRelativeAdapter,
        algorithm: Algorithm,
        indices: &[usize],
    ) -> Result<Option<Model>> {
        use firefly_opengv::{
            sac::sample_consensus_problem::SampleConsensusProblem,
            sac_problems::relative_pose::central_relative_pose_sac_problem::{
                Algorithm as GvAlgorithm, CentralRelativePoseSacProblem,
            },
        };
        let algorithm = match algorithm {
            Algorithm::Stewenius => GvAlgorithm::Stewenius,
            Algorithm::Nister => GvAlgorithm::Nister,
            Algorithm::SevenPt => GvAlgorithm::Sevenpt,
            Algorithm::EightPt => GvAlgorithm::Eightpt,
        };
        let mut problem =
            CentralRelativePoseSacProblem::new(adapter, algorithm, &self.backend, false);
        Ok(problem.compute_model_coefficients(indices))
    }
    fn optimize_nonlinear(
        &self,
        adapter: &mut FrameRelativeAdapter,
        inliers: &[usize],
    ) -> Result<Model> {
        firefly_opengv::relative_pose::methods::optimize_nonlinear(adapter, inliers, &self.backend)
            .ok_or_else(|| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Convergence,
                    "OpenGV relative-pose refinement failed",
                )
                .with_operation("OpenGvSolver::relative_pose")
            })
    }
}
