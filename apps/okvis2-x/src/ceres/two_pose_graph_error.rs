/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Two-pose visual factors, landmark Schur elimination, and constant factor evaluation.

use crate::{
    ceres::{
        error_interface::{ErrorEvaluation, ErrorInterface},
        homogeneous_point_parameter_block::HomogeneousPointParameterBlock,
        parameter_block::ParameterBlock,
        pose_local_parameterization::PoseManifold,
        pose_parameter_block::PoseParameterBlock,
        pseudo_inverse::PseudoInverse,
        reprojection_error_base::ReprojectionError2dBase,
    },
    frames::frame_typedefs::{KeypointIdentifier, StateId},
    kinematics::{
        operators::{cross_mx, oplus, plus},
        transformation::{Transformation, normalized_quaternion, quaternion_inverse},
    },
};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{
    DMatrix, DMatrixView, DVector, Dyn, Quaternion, SMatrix, SVector, SymmetricEigen, Vector3,
    Vector4,
};
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    ops::{Deref, DerefMut},
    rc::Rc,
};

pub type SharedPose = Rc<RefCell<PoseParameterBlock>>;
pub type SharedPoint = Rc<RefCell<HomogeneousPointParameterBlock>>;
/// Ceres loss value and its first two derivatives at a squared residual norm.
pub trait LossFunction {
    fn evaluate(&self, squared_norm: f64) -> [f64; 3];
}
#[derive(Clone)]
pub struct Observation {
    pub keypoint_identifier: KeypointIdentifier,
    pub pose: SharedPose,
    pub h_point: SharedPoint,
    pub extrinsics: SharedPose,
    pub reprojection_error: Rc<RefCell<Box<dyn ReprojectionError2dBase>>>,
    pub loss_function: Option<Rc<dyn LossFunction>>,
    pub is_marginalised: bool,
    pub is_duplication: bool,
}
#[derive(Clone)]
pub struct ParameterBlockInfo<const N: usize, P> {
    pub parameters: SVector<f64, N>,
    pub parameter_block: Rc<RefCell<P>>,
    pub idx: usize,
}
impl<const N: usize, P: ParameterBlock> ParameterBlockInfo<N, P> {
    fn new(parameter_block: Rc<RefCell<P>>, idx: usize) -> Result<Self> {
        let parameters = SVector::from_column_slice(
            parameter_block
                .borrow()
                .parameters()
                .ok_or_else(|| graph_error("parameter estimate is absent"))?,
        );
        Ok(Self {
            parameters,
            parameter_block,
            idx,
        })
    }
}
/// Observation bookkeeping shared by the fixed and calibrated-extrinsics factors.
pub struct TwoPoseGraphError {
    pub error_computation_valid: bool,
    pub pose_parameter_block_infos: [Option<ParameterBlockInfo<7, PoseParameterBlock>>; 2],
    pub extrinsics_parameter_block_infos: Vec<Option<ParameterBlockInfo<7, PoseParameterBlock>>>,
    pub landmark_parameter_block_infos: Vec<ParameterBlockInfo<4, HomogeneousPointParameterBlock>>,
    pub landmark_parameter_block_id_to_idx: BTreeMap<u64, usize>,
    pub extrinsics_size: usize,
    pub sparse_size: usize,
    pub reference_pose_id: StateId,
    pub other_pose_id: StateId,
    pub is_computed: bool,
    pub num_cams: usize,
    pub stay_const: bool,
    pub observations: BTreeMap<u64, Vec<Observation>>,
    pub landmarks: BTreeMap<u64, Vector4<f64>>,
    pub linearisation_point_t_s0s1: Transformation,
    pub linearisation_points_t_sc: Vec<Option<Transformation>>,
}
type ReducedNormalEquations = (DMatrix<f64>, DVector<f64>, DMatrix<f64>, DVector<f64>);
impl TwoPoseGraphError {
    #[must_use]
    pub fn new(
        reference_pose_id: StateId,
        other_pose_id: StateId,
        num_cams: usize,
        stay_const: bool,
    ) -> Self {
        let count = if stay_const { num_cams } else { 2 * num_cams };
        Self {
            error_computation_valid: false,
            pose_parameter_block_infos: [None, None],
            extrinsics_parameter_block_infos: vec![None; count],
            landmark_parameter_block_infos: Vec::new(),
            landmark_parameter_block_id_to_idx: BTreeMap::new(),
            extrinsics_size: 6 * count,
            sparse_size: 0,
            reference_pose_id,
            other_pose_id,
            is_computed: false,
            num_cams,
            stay_const,
            observations: BTreeMap::new(),
            landmarks: BTreeMap::new(),
            linearisation_point_t_s0s1: Transformation::identity(),
            linearisation_points_t_sc: vec![None; count],
        }
    }
    pub fn set_pose_ids(&mut self, reference_pose_id: StateId, other_pose_id: StateId) {
        self.reference_pose_id = reference_pose_id;
        self.other_pose_id = other_pose_id;
    }
    #[must_use]
    pub fn reference_pose_id(&self) -> StateId {
        self.reference_pose_id
    }
    #[must_use]
    pub fn other_pose_id(&self) -> StateId {
        self.other_pose_id
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_observation(
        &mut self,
        keypoint_identifier: KeypointIdentifier,
        reprojection_error: &dyn ReprojectionError2dBase,
        loss_function: Option<Rc<dyn LossFunction>>,
        (pose, h_point, extrinsics): (SharedPose, SharedPoint, SharedPose),
        is_duplication: bool,
        weight: f64,
    ) -> Result<bool> {
        self.error_computation_valid = false;
        if !self.reference_pose_id.is_initialised() || !self.other_pose_id.is_initialised() {
            return Err(graph_error("pose IDs are not initialized"));
        }
        let ci = reprojection_error
            .camera_id()
            .ok_or_else(|| graph_error("camera ID is absent"))? as usize;
        let mut cloned = reprojection_error.clone_box();
        if weight != 1.0 {
            let information = *cloned
                .information()
                .ok_or_else(|| graph_error("reprojection information is absent"))?;
            cloned.set_information(weight * information);
        } else if is_duplication {
            let information = *cloned
                .information()
                .ok_or_else(|| graph_error("reprojection information is absent"))?;
            cloned.set_information(information);
        }
        let point_id = h_point.borrow().id();
        self.observations
            .entry(point_id)
            .or_default()
            .push(Observation {
                keypoint_identifier,
                pose: pose.clone(),
                h_point: h_point.clone(),
                extrinsics: extrinsics.clone(),
                reprojection_error: Rc::new(RefCell::new(cloned)),
                loss_function,
                is_marginalised: false,
                is_duplication,
            });
        let mut pose_idx = 0;
        if pose.borrow().id() == self.reference_pose_id.value() {
            if self.pose_parameter_block_infos[0].is_none() {
                self.pose_parameter_block_infos[0] = Some(ParameterBlockInfo::new(pose, 0)?);
                pose_idx = 0;
            }
        } else if pose.borrow().id() == self.other_pose_id.value() {
            if self.pose_parameter_block_infos[1].is_none() {
                self.pose_parameter_block_infos[1] = Some(ParameterBlockInfo::new(pose, 0)?);
                pose_idx = 1;
            }
        } else {
            return Err(graph_error("pose not registered"));
        }
        if !self
            .landmark_parameter_block_id_to_idx
            .contains_key(&point_id)
        {
            self.landmark_parameter_block_infos
                .push(ParameterBlockInfo::new(h_point, self.sparse_size)?);
            self.landmark_parameter_block_id_to_idx
                .insert(point_id, self.landmark_parameter_block_infos.len() - 1);
            self.sparse_size += 3;
        }
        let offset = if self.stay_const {
            0
        } else {
            pose_idx * self.num_cams
        };
        let slot = self
            .extrinsics_parameter_block_infos
            .get_mut(offset + ci)
            .ok_or_else(|| graph_error("camera index outside extrinsics"))?;
        if slot.is_none() {
            let estimate = Transformation::from_other(
                extrinsics
                    .borrow()
                    .estimate()
                    .ok_or_else(|| graph_error("extrinsics estimate is absent"))?,
            );
            *slot = Some(ParameterBlockInfo::new(extrinsics, 6 + offset + ci * 6)?);
            self.linearisation_points_t_sc[offset + ci] = Some(estimate);
        }
        Ok(true)
    }
    pub fn get_landmarks(&self, landmarks: &mut BTreeSet<u64>) {
        landmarks.extend(self.landmark_parameter_block_id_to_idx.keys().copied());
    }
    pub fn get_observations(&self, observations: &mut Vec<Observation>) -> usize {
        observations.extend(self.observations.values().flatten().cloned());
        observations.len()
    }
    pub(crate) fn reference_pose(&self) -> Result<Transformation> {
        let info = self.pose_parameter_block_infos[0]
            .as_ref()
            .ok_or_else(|| graph_error("reference pose not observed"))?;
        Ok(Transformation::from_other(
            info.parameter_block
                .borrow()
                .estimate()
                .ok_or_else(|| graph_error("reference pose estimate is absent"))?,
        ))
    }
    pub(crate) fn convert_observations(
        &mut self,
        observations: &mut Vec<Observation>,
        duplicates: &mut Vec<KeypointIdentifier>,
    ) -> Result<bool> {
        let t_ws0 = self.reference_pose()?;
        for group in self.observations.values_mut() {
            for observation in group {
                if !observation.is_marginalised {
                    continue;
                }
                if observation.is_duplication {
                    duplicates.push(observation.keypoint_identifier);
                }
                let id = observation.h_point.borrow().id();
                let initialized = observation.h_point.borrow().initialized();
                let point = t_ws0.transform_homogeneous(
                    self.landmarks
                        .get(&id)
                        .ok_or_else(|| graph_error("landmark linearisation is absent"))?,
                );
                observation.h_point = Rc::new(RefCell::new(HomogeneousPointParameterBlock::new(
                    point,
                    id,
                    initialized,
                )));
                observations.push(observation.clone());
            }
        }
        self.landmark_parameter_block_id_to_idx.clear();
        self.landmark_parameter_block_infos.clear();
        self.observations.clear();
        self.landmarks.clear();
        self.extrinsics_parameter_block_infos.fill(None);
        self.linearisation_points_t_sc.fill(None);
        self.sparse_size = 0;
        Ok(true)
    }
    /// Forms each landmark's normal equations before eliminating its three tangent coordinates.
    pub(crate) fn compute_system(
        &mut self,
        with_extrinsics: bool,
    ) -> Result<ReducedNormalEquations> {
        let t_s0w = self.reference_pose()?.inverse();
        if self.pose_parameter_block_infos[1].is_none() {
            return Err(graph_error("other pose not observed"));
        }
        let n = if with_extrinsics {
            6 + self.extrinsics_size
        } else {
            6
        };
        let mut h_total = DMatrix::zeros(n, n);
        let mut b_total = DVector::zeros(n);
        let mut mh = DMatrix::zeros(n, n);
        let mut mb = DVector::zeros(n);
        let mut relative_pose_set = false;
        for (&landmark_id, group) in &mut self.observations {
            let mut system = LandmarkSystem {
                h00: DMatrix::zeros(n, n),
                b0: DVector::zeros(n),
                h01: DMatrix::zeros(n, 3),
                h11: DMatrix::zeros(3, 3),
                b1: DVector::zeros(3),
            };
            let point_idx = *self
                .landmark_parameter_block_id_to_idx
                .get(&landmark_id)
                .ok_or_else(|| graph_error("landmark index is absent"))?;
            let hp_s0 = t_s0w
                .transform_homogeneous(&self.landmark_parameter_block_infos[point_idx].parameters);
            self.landmarks.insert(landmark_id, hp_s0);
            let min_dist = hp_s0[2] / hp_s0[3];
            for observation in group {
                let id0 = observation.keypoint_identifier.frame_id;
                let ci = observation.keypoint_identifier.camera_index;
                let is_reference = id0 == self.reference_pose_id.value();
                let pose_idx = usize::from(!is_reference);
                let offset = if self.stay_const {
                    0
                } else {
                    pose_idx * self.num_cams
                };
                let info0 = self.pose_parameter_block_infos[pose_idx]
                    .as_ref()
                    .ok_or_else(|| graph_error("pose snapshot is absent"))?;
                let info2 = self
                    .extrinsics_parameter_block_infos
                    .get(offset + ci)
                    .and_then(Option::as_ref)
                    .ok_or_else(|| graph_error("extrinsics snapshot is absent"))?;
                let reference_from_sensor = if is_reference {
                    Transformation::identity()
                } else {
                    t_s0w.compose(&pose_from_parameters(info0.parameters.as_slice(), false))
                };
                if !is_reference && !relative_pose_set {
                    self.linearisation_point_t_s0s1 = reference_from_sensor.clone();
                    relative_pose_set = true;
                }
                let masks = [!is_reference, true, with_extrinsics];
                let evaluation = observation
                    .reprojection_error
                    .borrow()
                    .evaluate_with_minimal_jacobians(
                        &[
                            reference_from_sensor.coeffs().as_slice(),
                            hp_s0.as_slice(),
                            info2.parameters.as_slice(),
                        ],
                        Some(&masks),
                        Some(&masks),
                    )
                    .ok_or_else(|| graph_error("reprojection evaluation failed"))?;
                if !Self::accumulate_observation(
                    evaluation,
                    observation.loss_function.as_ref(),
                    info2.idx,
                    &mut system,
                )? {
                    continue;
                }
                observation.is_marginalised = true;
            }
            Self::accumulate_schur(
                system,
                min_dist,
                (&mut h_total, &mut b_total, &mut mh, &mut mb),
            );
        }
        h_total -= mh;
        b_total -= mb;
        let (j, delta_x) = factor_system(&h_total, &b_total);
        self.sparse_size = 0;
        self.landmark_parameter_block_id_to_idx.clear();
        self.is_computed = true;
        Ok((h_total, b_total, j, delta_x))
    }
    fn accumulate_schur(
        system: LandmarkSystem,
        min_dist: f64,
        (h_total, b_total, mh, mb): (
            &mut DMatrix<f64>,
            &mut DVector<f64>,
            &mut DMatrix<f64>,
            &mut DVector<f64>,
        ),
    ) {
        let LandmarkSystem {
            h00,
            b0,
            h01,
            h11,
            b1,
        } = system;
        let mut inverse_sqrt = DMatrix::zeros(3, 3);
        let mut rank = 0;
        PseudoInverse::symm_sqrt(&h11, &mut inverse_sqrt, 1.0e-7, Some(&mut rank));
        if !(rank < 3 && min_dist < 2.99) {
            *h_total += h00;
            *b_total += b0;
            let m = h01 * &inverse_sqrt;
            // Swap strides without copying; dot products must finish before updating the totals.
            let transposed = DMatrixView::<f64, Dyn, Dyn>::from_slice_with_strides(
                m.as_slice(),
                m.ncols(),
                m.nrows(),
                m.nrows(),
                1,
            );
            mh.gemm_tr(1.0, &transposed, &transposed, 1.0);
            let projected = inverse_sqrt.tr_mul(&b1);
            mb.gemv_tr(1.0, &transposed, &projected, 1.0);
        }
    }

    fn accumulate_observation(
        mut evaluation: ErrorEvaluation,
        loss_function: Option<&Rc<dyn LossFunction>>,
        extrinsics_index: usize,
        system: &mut LandmarkSystem,
    ) -> Result<bool> {
        let LandmarkSystem {
            h00,
            b0,
            h01,
            h11,
            b1,
        } = system;
        let n = h00.nrows();
        let mut residual = evaluation.residuals;
        if residual.norm() > 3.0 {
            return Ok(false);
        }
        let mut j0 = evaluation.minimal_jacobians[0].take();
        let mut j1 = evaluation.minimal_jacobians[1]
            .take()
            .ok_or_else(|| graph_error("landmark Jacobian is absent"))?;
        let mut j2 = evaluation.minimal_jacobians[2].take();
        if let Some(loss) = loss_function {
            let sq_norm = residual.norm_squared();
            let rho = loss.evaluate(sq_norm);
            let sqrt_rho1 = rho[1].sqrt();
            let (residual_scaling, alpha_sq_norm) = if sq_norm == 0.0 || rho[2] <= 0.0 {
                (sqrt_rho1, 0.0)
            } else {
                let d = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
                let alpha = 1.0 - d.sqrt();
                (sqrt_rho1 / (1.0 - alpha), alpha / sq_norm)
            };
            for j in j0
                .iter_mut()
                .chain(std::iter::once(&mut j1))
                .chain(j2.iter_mut())
            {
                if alpha_sq_norm == 0.0 {
                    *j *= sqrt_rho1;
                } else {
                    *j = sqrt_rho1 * (&*j - alpha_sq_norm * &residual * residual.tr_mul(&*j));
                }
            }
            residual *= residual_scaling;
        }
        if let Some(j0) = &j0 {
            add_normal_block(h00, 0, 0, j0, j0);
            add_gradient(b0, 0, j0, &residual);
            add_normal_block(h01, 0, 0, j0, &j1);
            if let Some(j2) = &j2 {
                if extrinsics_index + 6 > n {
                    return Err(graph_error(
                        "extrinsics ordering exceeds residual dimension",
                    ));
                }
                add_normal_block(h00, 0, extrinsics_index, j0, j2);
                add_normal_block(h00, extrinsics_index, 0, j2, j0);
            }
        }
        if let Some(j2) = &j2 {
            if extrinsics_index + 6 > n {
                return Err(graph_error(
                    "extrinsics ordering exceeds residual dimension",
                ));
            }
            add_normal_block(h01, extrinsics_index, 0, j2, &j1);
            add_normal_block(h00, extrinsics_index, extrinsics_index, j2, j2);
            add_gradient(b0, extrinsics_index, j2, &residual);
        }
        h11.gemm_tr(1.0, &j1, &j1, 1.0);
        b1.gemv_tr(-1.0, &j1, &residual, 1.0);
        Ok(true)
    }
}

pub struct TwoPoseStandardGraphError {
    pub base: TwoPoseGraphError,
    pub h00: DMatrix<f64>,
    pub b0: DVector<f64>,
    pub delta_x: DVector<f64>,
    pub j: DMatrix<f64>,
}
impl Deref for TwoPoseStandardGraphError {
    type Target = TwoPoseGraphError;
    fn deref(&self) -> &Self::Target {
        &self.base
    }
}
impl DerefMut for TwoPoseStandardGraphError {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.base
    }
}
impl TwoPoseStandardGraphError {
    #[must_use]
    pub fn new(reference: StateId, other: StateId, num_cams: usize, stay_const: bool) -> Self {
        Self {
            base: TwoPoseGraphError::new(reference, other, num_cams, stay_const),
            h00: DMatrix::zeros(6, 6),
            b0: DVector::zeros(6),
            delta_x: DVector::zeros(6),
            j: DMatrix::zeros(6, 6),
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn compute(&mut self) -> Result<bool> {
        if self.is_computed {
            return Ok(true);
        }
        let (h, b, _, _) = self.base.compute_system(false)?;
        self.h00 += h;
        self.b0 += b;
        let (j, d) = factor_system(&self.h00, &self.b0);
        self.j = j;
        self.delta_x = d;
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn convert_to_reprojection_errors(
        &mut self,
        observations: &mut Vec<Observation>,
        duplicates: &mut Vec<KeypointIdentifier>,
    ) -> Result<bool> {
        self.base.convert_observations(observations, duplicates)?;
        self.j.fill(0.0);
        self.h00.fill(0.0);
        self.b0.fill(0.0);
        Ok(true)
    }
    #[must_use]
    pub fn strength(&self) -> f64 {
        if self.is_computed {
            system_strength(&self.h00)
        } else {
            0.0
        }
    }
    #[must_use]
    pub fn clone_two_pose_graph_error_const(&self) -> TwoPoseStandardGraphErrorConst {
        TwoPoseStandardGraphErrorConst::new(
            self.delta_x.clone(),
            self.j.clone(),
            self.linearisation_point_t_s0s1.clone(),
        )
    }
}
#[derive(Clone)]
pub struct TwoPoseStandardGraphErrorConst {
    pub delta_x: DVector<f64>,
    pub j: DMatrix<f64>,
    pub linearisation_point_t_s0s1: Transformation,
}
impl TwoPoseStandardGraphErrorConst {
    #[must_use]
    pub fn new(
        delta_x: DVector<f64>,
        j: DMatrix<f64>,
        linearisation_point_t_s0s1: Transformation,
    ) -> Self {
        Self {
            delta_x,
            j,
            linearisation_point_t_s0s1,
        }
    }
}
pub trait TwoPoseGraphErrorConst: ErrorInterface {
    fn snapshot_send(&self) -> Box<dyn TwoPoseGraphErrorConst + Send>;
}
impl TwoPoseGraphErrorConst for TwoPoseStandardGraphErrorConst {
    fn snapshot_send(&self) -> Box<dyn TwoPoseGraphErrorConst + Send> {
        Box::new(self.clone())
    }
}
impl ErrorInterface for TwoPoseStandardGraphError {
    fn residual_dim(&self) -> usize {
        6
    }
    fn parameter_blocks(&self) -> usize {
        2
    }
    fn parameter_block_dim(&self, block: usize) -> usize {
        [7, 7][block]
    }
    fn type_info(&self) -> &'static str {
        "TwoPoseStandardGraphError"
    }
    fn evaluate_with_minimal_jacobians(
        &self,
        parameters: &[&[f64]],
        jacobians: Option<&[bool]>,
        minimal: Option<&[bool]>,
    ) -> Option<ErrorEvaluation> {
        if !self.is_computed {
            return None;
        }
        evaluate_graph(
            &self.delta_x,
            &self.j,
            &self.linearisation_point_t_s0s1,
            &[],
            None,
            parameters,
            (jacobians, minimal),
        )
    }
}
impl ErrorInterface for TwoPoseStandardGraphErrorConst {
    fn residual_dim(&self) -> usize {
        6
    }
    fn parameter_blocks(&self) -> usize {
        2
    }
    fn parameter_block_dim(&self, block: usize) -> usize {
        [7, 7][block]
    }
    fn type_info(&self) -> &'static str {
        "TwoPoseStandardGraphErrorConst"
    }
    fn evaluate_with_minimal_jacobians(
        &self,
        parameters: &[&[f64]],
        jacobians: Option<&[bool]>,
        minimal: Option<&[bool]>,
    ) -> Option<ErrorEvaluation> {
        evaluate_graph(
            &self.delta_x,
            &self.j,
            &self.linearisation_point_t_s0s1,
            &[],
            None,
            parameters,
            (jacobians, minimal),
        )
    }
}
/// Preserves the Ceres output-pointer gating, including reference dependence on the other pose request.
pub(crate) fn evaluate_graph(
    delta_x: &DVector<f64>,
    j: &DMatrix<f64>,
    linearisation: &Transformation,
    extrinsics: &[Option<Transformation>],
    extrinsics_indices: Option<&[Option<usize>]>,
    parameters: &[&[f64]],
    (jacobians, minimal): (Option<&[bool]>, Option<&[bool]>),
) -> Option<ErrorEvaluation> {
    let blocks = 2 + extrinsics.len();
    let n = delta_x.len();
    if parameters.len() != blocks
        || parameters.iter().any(|p| p.len() != 7)
        || j.shape() != (n, n)
        || n != 6 + 6 * extrinsics.len()
    {
        return None;
    }
    if jacobians.is_some_and(|x| x.len() != blocks) || minimal.is_some_and(|x| x.len() != blocks) {
        return None;
    }
    // A minimal-only Ceres request dereferences an absent ambient pointer array.
    if jacobians.is_none() && minimal.is_some() {
        return None;
    }
    let t_ws0 = pose_from_parameters(parameters[0], true);
    let t_s0w = t_ws0.inverse();
    let t_ws1 = pose_from_parameters(parameters[1], true);
    let relative = t_s0w.compose(&t_ws1);
    let mut dx = DVector::zeros(n);
    assign_deviation(&mut dx, 0, &relative, linearisation);
    for (i, point) in extrinsics.iter().enumerate() {
        if let Some(point) = point {
            let start = if let Some(indices) = extrinsics_indices {
                indices.get(i).copied().flatten()?
            } else {
                6 + 6 * i
            };
            if start + 6 > n {
                return None;
            }
            assign_deviation(
                &mut dx,
                start,
                &pose_from_parameters(parameters[i + 2], true),
                point,
            );
        }
    }
    let residuals = j * (delta_x + dx);
    let mut result = ErrorEvaluation {
        residuals,
        jacobians: vec![None; blocks],
        minimal_jacobians: vec![None; blocks],
    };
    let Some(mask) = jacobians else {
        return Some(result);
    };
    let mut jerr_ref = SMatrix::<f64, 6, 6>::zeros();
    if mask[1] {
        let t_ws = pose_from_parameters(parameters[1], false);
        let mut jerr = SMatrix::<f64, 6, 6>::zeros();
        jerr.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&t_s0w.rotation_matrix());
        let rotation = (plus(&quaternion_inverse(&t_ws0.quaternion()))
            * oplus(&(t_ws.quaternion() * quaternion_inverse(&linearisation.quaternion()))))
        .fixed_view::<3, 3>(0, 0)
        .into_owned();
        jerr.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
        if mask[0] {
            jerr_ref
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&(-t_s0w.rotation_matrix()));
            jerr_ref.fixed_view_mut::<3, 3>(0, 3).copy_from(
                &(t_s0w.rotation_matrix() * cross_mx(&(t_ws.translation() - t_ws0.translation()))),
            );
            jerr_ref
                .fixed_view_mut::<3, 3>(3, 3)
                .copy_from(&(-rotation));
        }
        let jm = j.columns(0, 6) * jerr;
        store_graph_jacobian(&mut result, 1, &jm, parameters[1], minimal);
    }
    if mask[0] {
        let jm = j.columns(0, 6) * jerr_ref;
        store_graph_jacobian(&mut result, 0, &jm, parameters[0], minimal);
    }
    for (i, point) in extrinsics.iter().enumerate() {
        if mask[i + 2] {
            let mut jerr = SMatrix::<f64, 6, 6>::zeros();
            if let Some(point) = point {
                let t = pose_from_parameters(parameters[i + 2], true);
                jerr.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();
                jerr.fixed_view_mut::<3, 3>(3, 3).copy_from(
                    &oplus(&(t.quaternion() * quaternion_inverse(&point.quaternion())))
                        .fixed_view::<3, 3>(0, 0),
                );
            }
            let jm = j.columns(6 + 6 * i, 6) * jerr;
            store_graph_jacobian(&mut result, i + 2, &jm, parameters[i + 2], minimal);
        }
    }
    Some(result)
}
fn store_graph_jacobian(
    result: &mut ErrorEvaluation,
    index: usize,
    jm: &nalgebra::Matrix<
        f64,
        nalgebra::Dyn,
        nalgebra::Const<6>,
        nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<6>>,
    >,
    parameters: &[f64],
    minimal: Option<&[bool]>,
) {
    let full = jm * PoseManifold::minus_jacobian(&SVector::from_column_slice(parameters));
    result.jacobians[index] = Some(DMatrix::from_column_slice(full.nrows(), 7, full.as_slice()));
    if minimal.is_some_and(|mask| mask[index]) {
        result.minimal_jacobians[index] =
            Some(DMatrix::from_column_slice(jm.nrows(), 6, jm.as_slice()));
    }
}
fn assign_deviation(
    dx: &mut DVector<f64>,
    start: usize,
    t: &Transformation,
    linearisation: &Transformation,
) {
    dx.rows_mut(start, 3)
        .copy_from(&(t.translation() - linearisation.translation()));
    dx.rows_mut(start + 3, 3).copy_from(
        &(2.0
            * (t.quaternion() * quaternion_inverse(&linearisation.quaternion()))
                .coords
                .fixed_rows::<3>(0)),
    );
}
pub(crate) fn pose_from_parameters(p: &[f64], normalize: bool) -> Transformation {
    let q = Quaternion::new(p[6], p[3], p[4], p[5]);
    Transformation::new(
        Vector3::new(p[0], p[1], p[2]),
        if normalize {
            normalized_quaternion(q)
        } else {
            q
        },
    )
}
pub(crate) fn factor_system(h: &DMatrix<f64>, b: &DVector<f64>) -> (DMatrix<f64>, DVector<f64>) {
    let eigen = SymmetricEigen::new(h.clone());
    let mut order: Vec<_> = (0..h.ncols()).collect();
    order.sort_by(|&a, &b| eigen.eigenvalues[a].total_cmp(&eigen.eigenvalues[b]));
    let values = DVector::from_iterator(order.len(), order.iter().map(|&i| eigen.eigenvalues[i]));
    let vectors = DMatrix::from_fn(h.nrows(), h.ncols(), |r, c| {
        eigen.eigenvectors[(r, order[c])]
    });
    let tolerance = 1.0e-8 * h.ncols() as f64 * values.max();
    let sqrt = values.map(|v| if v > tolerance { v.sqrt() } else { 0.0 });
    let inverse = values.map(|v| if v > tolerance { (1.0 / v).sqrt() } else { 0.0 });
    let mut j = vectors.transpose();
    for (mut row, scale) in j.row_iter_mut().zip(sqrt.iter()) {
        row.scale_mut(*scale);
    }
    let mut m = vectors;
    for (mut column, scale) in m.column_iter_mut().zip(inverse.iter()) {
        column.scale_mut(*scale);
    }
    let dx = -(&m * m.tr_mul(b));
    (j, dx)
}
pub(crate) fn system_strength(h: &DMatrix<f64>) -> f64 {
    let h = h.view((0, 0), (6, 6)).into_owned();
    let mut inverse = DMatrix::zeros(6, 6);
    let mut rank = 0;
    PseudoInverse::symm(&h, &mut inverse, 1.0e-6, Some(&mut rank));
    if rank < 6 {
        0.0
    } else {
        1.0 / inverse.norm().sqrt()
    }
}
fn add_normal_block(
    target: &mut DMatrix<f64>,
    row: usize,
    col: usize,
    left: &DMatrix<f64>,
    right: &DMatrix<f64>,
) {
    target
        .view_mut((row, col), (left.ncols(), right.ncols()))
        .gemm_tr(1.0, left, right, 1.0);
}
fn add_gradient(
    target: &mut DVector<f64>,
    start: usize,
    jacobian: &DMatrix<f64>,
    residual: &DVector<f64>,
) {
    target
        .rows_mut(start, jacobian.ncols())
        .gemv_tr(-1.0, jacobian, residual, 1.0);
}
pub(crate) fn graph_error(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("TwoPoseGraphError")
}

struct LandmarkSystem {
    h00: DMatrix<f64>,
    b0: DVector<f64>,
    h01: DMatrix<f64>,
    h11: DMatrix<f64>,
    b1: DVector<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ceres::vi_graph::RobustLoss;

    struct ConvexLoss;
    impl LossFunction for ConvexLoss {
        fn evaluate(&self, s: f64) -> [f64; 3] {
            [s + s * s, 1. + 2. * s, 2.]
        }
    }

    fn observation(
        pose: bool,
        extrinsics: bool,
        residual: &DVector<f64>,
    ) -> (ErrorEvaluation, DMatrix<f64>) {
        let dense = DMatrix::from_fn(2, 15, |row, col| {
            if (col < 6 && !pose) || ((6..12).contains(&col) && !extrinsics) {
                0.
            } else {
                (row as f64 - col as f64 + 2.) / 10.
            }
        });
        let evaluation = ErrorEvaluation {
            residuals: residual.clone(),
            jacobians: vec![None; 3],
            minimal_jacobians: vec![
                pose.then(|| dense.columns(0, 6).into_owned()),
                Some(dense.columns(12, 3).into_owned()),
                extrinsics.then(|| dense.columns(6, 6).into_owned()),
            ],
        };
        (evaluation, dense)
    }

    fn empty_system() -> LandmarkSystem {
        LandmarkSystem {
            h00: DMatrix::zeros(12, 12),
            b0: DVector::zeros(12),
            h01: DMatrix::zeros(12, 3),
            h11: DMatrix::zeros(3, 3),
            b1: DVector::zeros(3),
        }
    }

    #[test]
    fn schur_accumulation_matches_analytic_inverse_and_preserves_rank_distance_gate() {
        for eigenvalue in [0., 4.] {
            for distance in [2., 3.] {
                let mut system = empty_system();
                system.h00.fill(0.5);
                system.b0.fill(-0.25);
                system.h01 = DMatrix::from_fn(12, 3, |r, c| (r + 2 * c + 1) as f64 / 10.);
                system.h11 =
                    DMatrix::from_diagonal(&DVector::from_row_slice(&[1., 2., eigenvalue]));
                system.b1 = DVector::from_row_slice(&[0.5, -1., 2.]);
                let floor = 1e-7 * 3. * 2.;
                let inverse = DMatrix::from_diagonal(&DVector::from_row_slice(&[
                    1.,
                    0.5,
                    if eigenvalue == 0. { 1. / floor } else { 0.25 },
                ]));
                let expected_h = &system.h01 * &inverse * system.h01.transpose();
                let expected_b = &system.h01 * inverse * &system.b1;
                let mut h = DMatrix::repeat(12, 12, 1.);
                let mut b = DVector::repeat(12, 2.);
                let mut mh = DMatrix::repeat(12, 12, 3.);
                let mut mb = DVector::repeat(12, 4.);
                TwoPoseGraphError::accumulate_schur(
                    system,
                    distance,
                    (&mut h, &mut b, &mut mh, &mut mb),
                );
                if eigenvalue == 0. && distance < 2.99 {
                    assert_eq!(h, DMatrix::repeat(12, 12, 1.));
                    assert_eq!(b, DVector::repeat(12, 2.));
                    assert_eq!(mh, DMatrix::repeat(12, 12, 3.));
                    assert_eq!(mb, DVector::repeat(12, 4.));
                } else {
                    assert_eq!(h, DMatrix::repeat(12, 12, 1.5));
                    assert_eq!(b, DVector::repeat(12, 1.75));
                    let h_error = (mh - DMatrix::repeat(12, 12, 3.) - &expected_h).amax();
                    let b_error = (mb - DVector::repeat(12, 4.) - &expected_b).amax();
                    assert!(h_error < 1e-12 * expected_h.amax().max(1.));
                    assert!(b_error < 1e-12 * expected_b.amax().max(1.));
                }
            }
        }
    }

    #[test]
    fn schur_accumulation_preserves_prior_contributions_during_landmark_cancellation() {
        let mut system = empty_system();
        system.h11 = DMatrix::identity(3, 3);
        system
            .h01
            .row_mut(0)
            .copy_from(&nalgebra::RowVector3::new(1e8, 1e8, 0.));
        system
            .h01
            .row_mut(1)
            .copy_from(&nalgebra::RowVector3::new(1e8, -1e8, 0.));
        system.b1 = DVector::from_row_slice(&[1e8, -1e8, 0.]);
        let mut h = DMatrix::zeros(12, 12);
        let mut b = DVector::zeros(12);
        let mut mh = DMatrix::repeat(12, 12, 1.);
        let mut mb = DVector::repeat(12, 1.);
        TwoPoseGraphError::accumulate_schur(system, 3., (&mut h, &mut b, &mut mh, &mut mb));
        assert_eq!(mh[(0, 1)], 1.);
        assert_eq!(mh[(1, 0)], 1.);
        assert_eq!(mb[0], 1.);
    }

    #[test]
    fn observation_accumulation_matches_independent_robust_normal_equations() {
        let losses: [Option<Rc<dyn LossFunction>>; 5] = [
            None,
            Some(Rc::new(RobustLoss::Huber(0.2))),
            Some(Rc::new(RobustLoss::Cauchy(0.2))),
            Some(Rc::new(RobustLoss::Tukey(0.2))),
            Some(Rc::new(ConvexLoss)),
        ];
        for residual in [DVector::from_row_slice(&[0.3, -0.4]), DVector::zeros(2)] {
            for (pose, extrinsics) in [(true, true), (false, true), (true, false), (false, false)] {
                for loss in &losses {
                    let mut system = empty_system();
                    let (evaluation, dense) = observation(pose, extrinsics, &residual);
                    let rho = loss
                        .as_ref()
                        .map_or([residual.norm_squared(), 1., 0.], |loss| {
                            loss.evaluate(residual.norm_squared())
                        });
                    let gradient = DVector::from_fn(15, |col, _| {
                        (0..2)
                            .map(|row| dense[(row, col)] * residual[row])
                            .sum::<f64>()
                    });
                    let expected_h = DMatrix::from_fn(15, 15, |a, b| {
                        rho[1]
                            * (0..2)
                                .map(|row| dense[(row, a)] * dense[(row, b)])
                                .sum::<f64>()
                            + 2. * rho[2].max(0.) * gradient[a] * gradient[b]
                    });
                    let expected_b = -rho[1] * gradient;
                    for count in 1..=2 {
                        assert!(
                            TwoPoseGraphError::accumulate_observation(
                                evaluation.clone(),
                                loss.as_ref(),
                                6,
                                &mut system,
                            )
                            .unwrap()
                        );
                        let count = f64::from(count);
                        assert!(
                            (&system.h00 - expected_h.view((0, 0), (12, 12)) * count).amax()
                                < 1e-12
                        );
                        assert!(
                            (&system.h01 - expected_h.view((0, 12), (12, 3)) * count).amax()
                                < 1e-12
                        );
                        assert!(
                            (&system.h11 - expected_h.view((12, 12), (3, 3)) * count).amax()
                                < 1e-12
                        );
                        assert!((&system.b0 - expected_b.rows(0, 12) * count).amax() < 1e-12);
                        assert!((&system.b1 - expected_b.rows(12, 3) * count).amax() < 1e-12);
                    }
                }
            }
        }
    }

    #[test]
    fn observation_accumulation_preserves_rejection_and_missing_parameter_errors() {
        let mut system = empty_system();
        let (mut evaluation, _) = observation(true, true, &DVector::from_element(2, 4.));
        assert!(
            !TwoPoseGraphError::accumulate_observation(evaluation.clone(), None, 6, &mut system)
                .unwrap()
        );
        assert_eq!(system.h00.norm(), 0.);
        evaluation.residuals.fill(0.);
        evaluation.minimal_jacobians[1] = None;
        assert!(
            TwoPoseGraphError::accumulate_observation(evaluation, None, 6, &mut system).is_err()
        );
        let (evaluation, _) = observation(false, true, &DVector::zeros(2));
        assert!(
            TwoPoseGraphError::accumulate_observation(evaluation, None, 7, &mut system).is_err()
        );
    }

    #[test]
    fn factor_system_reconstructs_a_rotated_rank_deficient_quadratic() {
        let h = DMatrix::from_row_slice(3, 3, &[0.64, -0.48, 0., -0.48, 0.36, 0., 0., 0., 4.]);
        let b = DVector::from_row_slice(&[2., -1., 3.]);
        let (j, dx) = factor_system(&h, &b);
        assert!((j.tr_mul(&j) - h).amax() < 1e-12);
        assert!((dx - DVector::from_row_slice(&[-1.76, 1.32, -0.75])).amax() < 1e-12);
        assert!(
            (&j * j.transpose() - DMatrix::from_diagonal(&DVector::from_row_slice(&[0., 1., 4.])))
                .amax()
                < 1e-12
        );
    }
}
