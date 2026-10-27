/*
 * OKVIS2-X, Copyright (c) ETH Zurich, Imperial College London and TUM.
 * Ceres Solver, Copyright 2023 Google Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of Google Inc. nor the names of its contributors may be
 * used to endorse or promote products derived from this software without
 * specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
//! Owned graph evaluation and Ceres traditional Dogleg/LM trust-region equations.
//! Sparse normal Cholesky and dense augmented QR use the pure-Rust faer backend.
use super::{
    BTreeMap, BTreeSet, ErrorEvaluation, GpsErrorAsynchronous, GraphFactor, GraphProblem,
    GraphSolver, ImuError, LossFunction, ManifoldKind, Matrix3, OccupancySubmap,
    OneSidedDepthError, Parameter, ParameterBlock, PoseError, PseudoImuError, Quaternion, Rc,
    RelativePoseError, ReprojectionError2dBase, ResidualId, Result, RobustLoss, SMatrix, SVector,
    SolverFactorCache, SolverOptions, SolverStage, SolverSummary, SolverUpdate, SpeedAndBiasError,
    Transformation, TwoPoseGraphErrorConst, Vector3, Vector4, VisualFactor, cross_mx, graph_error,
};
use crate::ceres::{
    depth_error::DepthJacobians,
    homogeneous_point_local_parameterization::HomogeneousPointManifold,
    pose_local_parameterization::{PoseManifold, PoseManifold4d},
    submap_icp_error::SubmapIcpError,
    two_pose_graph_error::evaluate_graph,
};
use faer::linalg::solvers::{Solve, SolveLstsq};
use faer::sparse::{SparseColMat, Triplet};
use nalgebra::{DMatrix, DMatrixView, DVector, DVectorView, DVectorViewMut};
use std::{sync::Arc, time::Instant};

#[path = "schur.rs"]
mod schur;
use schur::{Row, SchurSystem};

#[cfg(test)]
#[path = "schur/tests.rs"]
mod schur_tests;

#[path = "evaluation.rs"]
mod evaluation;
#[path = "execution.rs"]
mod execution;
use evaluation::Workspace;
use execution::Execution;

#[cfg(feature = "solver-bench")]
#[path = "fixed_bench.rs"]
mod fixed_bench;

#[derive(Default)]
pub struct RustGraphSolver {
    execution: Execution,
    workspace: Workspace,
}

#[derive(Clone)]
struct Values {
    data: Vec<f64>,
    manifold: ManifoldKind,
    fixed: bool,
}
impl Values {
    fn tangent_size(&self) -> usize {
        match self.manifold {
            ManifoldKind::Pose => 6,
            ManifoldKind::Pose4d => 4,
            ManifoldKind::HomogeneousPoint => 3,
            ManifoldKind::Euclidean => self.data.len(),
        }
    }
    fn plus(&self, delta: &[f64]) -> Result<Vec<f64>> {
        if delta.len() != self.tangent_size() {
            return Err(graph_error("tangent dimension mismatch"));
        }
        let v = match self.manifold {
            ManifoldKind::Pose => {
                if self.data.len() != 7 {
                    return Err(graph_error("pose dimension mismatch"));
                }
                PoseManifold::plus(
                    &SVector::from_column_slice(&self.data),
                    &SVector::from_column_slice(delta),
                )
                .as_slice()
                .to_vec()
            }
            ManifoldKind::Pose4d => {
                if self.data.len() != 7 {
                    return Err(graph_error("4DoF pose dimension mismatch"));
                }
                PoseManifold4d::plus(
                    &SVector::from_column_slice(&self.data),
                    &SVector::from_column_slice(delta),
                )
                .as_slice()
                .to_vec()
            }
            ManifoldKind::HomogeneousPoint => {
                if self.data.len() != 4 {
                    return Err(graph_error("point dimension mismatch"));
                }
                HomogeneousPointManifold::plus(
                    &Vector4::from_column_slice(&self.data),
                    &Vector3::from_column_slice(delta),
                )
                .as_slice()
                .to_vec()
            }
            ManifoldKind::Euclidean => self.data.iter().zip(delta).map(|(x, d)| x + d).collect(),
        };
        if v.iter().any(|x| !x.is_finite()) {
            return Err(graph_error("non-finite manifold update"));
        }
        Ok(v)
    }
    fn apply_plus_jacobian(&self, jacobian: DMatrix<f64>) -> DMatrix<f64> {
        match self.manifold {
            ManifoldKind::Pose => {
                let j = PoseManifold::plus_jacobian(&SVector::from_column_slice(&self.data));
                jacobian * DMatrix::from_column_slice(7, 6, j.as_slice())
            }
            ManifoldKind::Pose4d => {
                let j = PoseManifold4d::plus_jacobian(&SVector::from_column_slice(&self.data));
                jacobian * DMatrix::from_column_slice(7, 4, j.as_slice())
            }
            ManifoldKind::HomogeneousPoint => {
                let j = HomogeneousPointManifold::plus_jacobian(&Vector4::from_column_slice(
                    &self.data,
                ));
                jacobian * DMatrix::from_column_slice(4, 3, j.as_slice())
            }
            ManifoldKind::Euclidean => jacobian,
        }
    }
}

enum Factor {
    Reprojection(Box<dyn ReprojectionError2dBase + Send>),
    Depth(OneSidedDepthError),
    Imu(Box<ImuError>),
    PseudoImu(Box<PseudoImuError>),
    PosePrior(Box<PoseError>),
    SpeedBiasPrior(Box<SpeedAndBiasError>),
    RelativePose(Box<RelativePoseError>),
    Gps(Box<GpsErrorAsynchronous>),
    Visual {
        delta: DVector<f64>,
        j: DMatrix<f64>,
        point: Transformation,
        extrinsics: Vec<Option<Transformation>>,
        indices: Option<Vec<Option<usize>>>,
    },
    VisualConst(Box<dyn TwoPoseGraphErrorConst + Send>),
    Submap {
        map: Arc<dyn OccupancySubmap + Send + Sync>,
        point: Vector3<f64>,
        sigma: f64,
    },
    Alignment {
        gps: Vector3<f64>,
        world: Vector3<f64>,
    },
}
struct FactorRecord {
    id: ResidualId,
    factor: Factor,
    keys: Vec<usize>,
    loss: Option<RobustLoss>,
}
struct Snapshot {
    values: BTreeMap<usize, Values>,
    factors: Vec<FactorRecord>,
}

fn output<const N: usize>(
    residuals: DVector<f64>,
    jacobians: [DMatrixView<'_, f64>; N],
    mask: Option<&[bool]>,
) -> ErrorEvaluation {
    ErrorEvaluation {
        residuals,
        minimal_jacobians: vec![None; jacobians.len()],
        jacobians: jacobians
            .into_iter()
            .enumerate()
            .map(|(i, j)| mask.is_some_and(|m| m[i]).then(|| j.into_owned()))
            .collect(),
    }
}
fn imu_output(
    e: &crate::ceres::imu_error::ImuEvaluation,
    mask: Option<&[bool]>,
) -> ErrorEvaluation {
    output(
        DVector::from_column_slice(e.residual.as_slice()),
        [
            e.pose0.as_view(),
            e.speed_bias0.as_view(),
            e.pose1.as_view(),
            e.speed_bias1.as_view(),
        ],
        mask,
    )
}
impl Factor {
    fn evaluate(&mut self, p: &[&[f64]], mask: Option<&[bool]>) -> Result<ErrorEvaluation> {
        let failed = || graph_error("factor evaluation failed");
        match self {
            Self::Reprojection(f) => f
                .evaluate_with_minimal_jacobians(p, mask, None)
                .ok_or_else(failed),
            Self::VisualConst(f) => f
                .evaluate_with_minimal_jacobians(p, mask, None)
                .ok_or_else(failed),
            Self::Visual {
                delta,
                j,
                point,
                extrinsics,
                indices,
            } => evaluate_graph(
                delta,
                j,
                point,
                extrinsics,
                indices.as_deref(),
                p,
                (mask, None),
            )
            .ok_or_else(failed),
            Self::Depth(_) | Self::PosePrior(_) | Self::SpeedBiasPrior(_) => {
                self.evaluate_prior(p, mask)
            }
            Self::Imu(_) | Self::PseudoImu(_) | Self::RelativePose(_) => {
                self.evaluate_motion(p, mask)
            }
            Self::Alignment { .. } | Self::Gps(_) | Self::Submap { .. } => {
                self.evaluate_mapping(p, mask)
            }
        }
    }
    fn evaluate_prior(&mut self, p: &[&[f64]], mask: Option<&[bool]>) -> Result<ErrorEvaluation> {
        let failed = || graph_error("factor evaluation failed");
        match self {
            Self::Depth(f) => {
                if p.len() != 3 || p.iter().map(|p| p.len()).ne([7, 4, 7]) {
                    return Err(failed());
                }
                if f.measurement().is_none() || f.information().is_none() {
                    return Err(failed());
                }
                let (mut r, mut j0, mut j1, mut j2) = (
                    0.0,
                    SMatrix::<f64, 1, 7>::zeros(),
                    SMatrix::<f64, 1, 4>::zeros(),
                    SMatrix::<f64, 1, 7>::zeros(),
                );
                let js = mask.map(|m| DepthJacobians {
                    pose: m[0].then_some(&mut j0),
                    point: m[1].then_some(&mut j1),
                    extrinsics: m[2].then_some(&mut j2),
                });
                let ok = f.evaluate(
                    (
                        &SVector::from_column_slice(p[0]),
                        &Vector4::from_column_slice(p[1]),
                        &SVector::from_column_slice(p[2]),
                    ),
                    &mut r,
                    js,
                );
                if !ok {
                    return Err(failed());
                }
                Ok(output(
                    DVector::from_element(1, r),
                    [j0.as_view(), j1.as_view(), j2.as_view()],
                    mask,
                ))
            }
            Self::PosePrior(f) => {
                if p.len() != 1 || p[0].len() != 7 || f.information().is_none() {
                    return Err(failed());
                }
                let (mut r, mut j) = (SVector::<f64, 6>::zeros(), SMatrix::<f64, 6, 7>::zeros());
                if !f.evaluate(
                    &SVector::from_column_slice(p[0]),
                    &mut r,
                    mask.is_some_and(|m| m[0]).then_some(&mut j),
                ) {
                    return Err(failed());
                }
                Ok(output(
                    DVector::from_column_slice(r.as_slice()),
                    [j.as_view()],
                    mask,
                ))
            }
            Self::SpeedBiasPrior(f) => {
                if p.len() != 1 || p[0].len() != 9 {
                    return Err(failed());
                }
                let (mut r, mut j) = (SVector::<f64, 9>::zeros(), SMatrix::<f64, 9, 9>::zeros());
                if !f.evaluate(
                    &SVector::from_column_slice(p[0]),
                    &mut r,
                    mask.is_some_and(|m| m[0]).then_some(&mut j),
                ) {
                    return Err(failed());
                }
                Ok(output(
                    DVector::from_column_slice(r.as_slice()),
                    [j.as_view()],
                    mask,
                ))
            }
            _ => Err(failed()),
        }
    }
    fn evaluate_motion(&mut self, p: &[&[f64]], mask: Option<&[bool]>) -> Result<ErrorEvaluation> {
        let failed = || graph_error("factor evaluation failed");
        match self {
            Self::Imu(f) => {
                if p.len() != 4 || p.iter().map(|p| p.len()).ne([7, 9, 7, 9]) {
                    return Err(failed());
                }
                let pose0 = SVector::from_column_slice(p[0]);
                let sb0 = SVector::from_column_slice(p[1]);
                let pose1 = SVector::from_column_slice(p[2]);
                let sb1 = SVector::from_column_slice(p[3]);
                let evaluation_error = |e| graph_error(&format!("IMU evaluation: {e:?}"));
                if !mask.is_some_and(|m| m.iter().any(|&requested| requested)) {
                    let residual = f
                        .evaluate_residual(&pose0, &sb0, &pose1, &sb1)
                        .map_err(evaluation_error)?;
                    return Ok(ErrorEvaluation {
                        residuals: DVector::from_column_slice(residual.as_slice()),
                        jacobians: vec![None; 4],
                        minimal_jacobians: vec![None; 4],
                    });
                }
                let e = f
                    .evaluate(&pose0, &sb0, &pose1, &sb1)
                    .map_err(evaluation_error)?;
                Ok(imu_output(&e, mask))
            }
            Self::PseudoImu(f) => {
                if p.len() != 4 || p.iter().map(|p| p.len()).ne([7, 9, 7, 9]) {
                    return Err(failed());
                }
                Ok(imu_output(
                    &f.evaluate(
                        &SVector::from_column_slice(p[0]),
                        &SVector::from_column_slice(p[1]),
                        &SVector::from_column_slice(p[2]),
                        &SVector::from_column_slice(p[3]),
                    ),
                    mask,
                ))
            }
            Self::RelativePose(f) => {
                if p.len() != 2 || p.iter().any(|p| p.len() != 7) {
                    return Err(failed());
                }
                let (mut r, mut j0, mut j1) = (
                    SVector::<f64, 6>::zeros(),
                    SMatrix::<f64, 6, 7>::zeros(),
                    SMatrix::<f64, 6, 7>::zeros(),
                );
                let js = [
                    mask.is_some_and(|m| m[0]).then_some(&mut j0),
                    mask.is_some_and(|m| m[1]).then_some(&mut j1),
                ];
                if !f.evaluate(
                    [
                        &SVector::from_column_slice(p[0]),
                        &SVector::from_column_slice(p[1]),
                    ],
                    &mut r,
                    js,
                ) {
                    return Err(failed());
                }
                Ok(output(
                    DVector::from_column_slice(r.as_slice()),
                    [j0.as_view(), j1.as_view()],
                    mask,
                ))
            }
            _ => Err(failed()),
        }
    }
    fn evaluate_mapping(&mut self, p: &[&[f64]], mask: Option<&[bool]>) -> Result<ErrorEvaluation> {
        let failed = || graph_error("factor evaluation failed");
        match self {
            Self::Gps(f) => {
                if p.len() != 3 || p.iter().map(|p| p.len()).ne([7, 9, 7]) {
                    return Err(failed());
                }
                let e = f
                    .evaluate(
                        &SVector::from_column_slice(p[0]),
                        &SVector::from_column_slice(p[1]),
                        &SVector::from_column_slice(p[2]),
                    )
                    .map_err(|e| graph_error(&format!("GPS evaluation: {e:?}")))?;
                Ok(output(
                    DVector::from_column_slice(e.residual.as_slice()),
                    [
                        e.pose.as_view(),
                        e.speed_bias.as_view(),
                        e.gps_world.as_view(),
                    ],
                    mask,
                ))
            }
            Self::Submap { map, point, sigma } => {
                if p.len() != 2 || p.iter().any(|p| p.len() != 7) {
                    return Err(failed());
                }
                let (mut r, mut j0, mut j1) = (
                    0.0,
                    SMatrix::<f64, 1, 7>::zeros(),
                    SMatrix::<f64, 1, 7>::zeros(),
                );
                let js = mask.map(|m| [m[0].then_some(&mut j0), m[1].then_some(&mut j1)]);
                let f = SubmapIcpError::new(map.as_ref(), *point, *sigma);
                if !f.evaluate(
                    [
                        &SVector::from_column_slice(p[0]),
                        &SVector::from_column_slice(p[1]),
                    ],
                    &mut r,
                    js,
                ) {
                    return Err(failed());
                }
                Ok(output(
                    DVector::from_element(1, r),
                    [j0.as_view(), j1.as_view()],
                    mask,
                ))
            }
            Self::Alignment { gps, world } => {
                if p.len() != 1 || p[0].len() != 7 {
                    return Err(failed());
                }
                let pose = Transformation::new(
                    Vector3::from_column_slice(&p[0][..3]),
                    Quaternion::new(p[0][6], p[0][3], p[0][4], p[0][5]),
                );
                let rotated = pose.rotation_matrix() * *world;
                let r = rotated + pose.translation() - *gps;
                let mut minimal = SMatrix::<f64, 3, 6>::zeros();
                minimal
                    .fixed_view_mut::<3, 3>(0, 0)
                    .copy_from(&Matrix3::identity());
                minimal
                    .fixed_view_mut::<3, 3>(0, 3)
                    .copy_from(&-cross_mx(&rotated));
                let j = minimal * PoseManifold::minus_jacobian(&SVector::from_column_slice(p[0]));
                Ok(output(
                    DVector::from_column_slice(r.as_slice()),
                    [j.as_view()],
                    mask,
                ))
            }
            _ => Err(failed()),
        }
    }
}
impl Snapshot {
    #[fastrace::trace]
    fn capture(problem: &GraphProblem) -> Result<Self> {
        let values = Self::capture_values(problem)?;
        let mut factors = Vec::new();
        let mut map_snapshots: BTreeMap<usize, Arc<dyn OccupancySubmap + Send + Sync>> =
            BTreeMap::new();
        for (&id, r) in &problem.residuals {
            let factor = match r.factor.as_ref() {
                GraphFactor::Reprojection(f) => Factor::Reprojection(
                    f.borrow()
                        .snapshot_send()
                        .ok_or_else(|| graph_error("camera snapshot unavailable"))?,
                ),
                GraphFactor::Depth(f) => Factor::Depth(*f),
                GraphFactor::Imu(f) => Factor::Imu(Box::new(f.borrow().clone())),
                GraphFactor::PseudoImu(f) => Factor::PseudoImu(Box::new(f.borrow().clone())),
                GraphFactor::PosePrior(f) => Factor::PosePrior(f.clone()),
                GraphFactor::SpeedBiasPrior(f) => Factor::SpeedBiasPrior(f.clone()),
                GraphFactor::RelativePose(f) => Factor::RelativePose(f.clone()),
                GraphFactor::Gps(f) => Factor::Gps(Box::new(f.borrow().clone())),
                GraphFactor::VisualConst(f) => Factor::VisualConst(f.snapshot_send()),
                GraphFactor::Visual(f) => {
                    let mut f = f.borrow_mut();
                    if !f.base().is_computed && !f.compute()? {
                        return Err(graph_error("visual factor Schur computation failed"));
                    }
                    match &*f {
                        VisualFactor::Standard(f) => Factor::Visual {
                            delta: f.delta_x.clone(),
                            j: f.j.clone(),
                            point: f.linearisation_point_t_s0s1.clone(),
                            extrinsics: vec![],
                            indices: None,
                        },
                        VisualFactor::Extrinsics(f) => Factor::Visual {
                            delta: f
                                .delta_x
                                .clone()
                                .ok_or_else(|| graph_error("visual delta absent"))?,
                            j: f.j
                                .clone()
                                .ok_or_else(|| graph_error("visual Jacobian absent"))?,
                            point: f.linearisation_point_t_s0s1.clone(),
                            extrinsics: f.linearisation_points_t_sc.clone(),
                            indices: Some(
                                f.extrinsics_parameter_block_infos
                                    .iter()
                                    .map(|p| p.as_ref().map(|p| p.idx))
                                    .collect(),
                            ),
                        },
                    }
                }
                GraphFactor::Submap { map, point, sigma } => Factor::Submap {
                    map: {
                        let key = map
                            .snapshot_identity()
                            .unwrap_or(Rc::as_ptr(map).cast::<()>() as usize);
                        if let std::collections::btree_map::Entry::Vacant(entry) =
                            map_snapshots.entry(key)
                        {
                            entry.insert(map.snapshot()?);
                        }
                        map_snapshots[&key].clone()
                    },
                    point: *point,
                    sigma: *sigma,
                },
            };
            let keys: Vec<_> = r.parameters.iter().map(Parameter::key).collect();
            if keys.iter().any(|k| !values.contains_key(k)) {
                return Err(graph_error("unregistered residual parameter"));
            }
            factors.push(FactorRecord {
                id,
                factor,
                keys,
                loss: r.loss,
            });
        }
        Ok(Self { values, factors })
    }
    fn capture_values(problem: &GraphProblem) -> Result<BTreeMap<usize, Values>> {
        let mut values = BTreeMap::new();
        for (&key, (p, m)) in &problem.parameters {
            let data = p.values()?;
            if data.iter().any(|x| !x.is_finite()) {
                return Err(graph_error("non-finite graph parameter"));
            }
            let valid = match m {
                ManifoldKind::Pose | ManifoldKind::Pose4d => data.len() == 7,
                ManifoldKind::HomogeneousPoint => data.len() == 4,
                ManifoldKind::Euclidean => true,
            };
            if !valid {
                return Err(graph_error("parameter manifold dimension mismatch"));
            }
            values.insert(
                key,
                Values {
                    data,
                    manifold: *m,
                    fixed: p.fixed(),
                },
            );
        }
        Ok(values)
    }

    #[fastrace::trace]
    fn restore_caches(&self, problem: &GraphProblem) {
        for f in &self.factors {
            let Some(r) = problem.residuals.get(&f.id) else {
                continue;
            };
            match (&f.factor, r.factor.as_ref()) {
                (Factor::Imu(s), GraphFactor::Imu(d)) => *d.borrow_mut() = s.as_ref().clone(),
                (Factor::Gps(s), GraphFactor::Gps(d)) => *d.borrow_mut() = s.as_ref().clone(),
                _ => (),
            }
        }
    }
    #[fastrace::trace]
    fn write_values(&self, problem: &GraphProblem) -> Result<()> {
        for (&key, v) in &self.values {
            let (p, _) = problem
                .parameters
                .get(&key)
                .ok_or_else(|| graph_error("snapshot parameter disappeared"))?;
            if v.fixed {
                continue;
            }
            match p {
                Parameter::Pose(p) => p.borrow_mut().set_parameters(&v.data),
                Parameter::SpeedBias(p) => p.borrow_mut().set_parameters(&v.data),
                Parameter::Point(p) => p.borrow_mut().set_parameters(&v.data),
            }
        }
        Ok(())
    }
    fn plus(
        values: &BTreeMap<usize, Values>,
        offsets: &BTreeMap<usize, usize>,
        delta: &DVector<f64>,
    ) -> Result<BTreeMap<usize, Values>> {
        values
            .iter()
            .map(|(&key, v)| {
                let data = match offsets.get(&key) {
                    Some(&offset) => {
                        v.plus(&delta.as_slice()[offset..offset + v.tangent_size()])?
                    }
                    None => v.data.clone(),
                };
                Ok((
                    key,
                    Values {
                        data,
                        manifold: v.manifold,
                        fixed: v.fixed,
                    },
                ))
            })
            .collect()
    }
    fn evaluate(
        &mut self,
        values: &BTreeMap<usize, Values>,
        offsets: &BTreeMap<usize, usize>,
        size: usize,
        disabled: &BTreeSet<ResidualId>,
        jacobians: bool,
    ) -> Result<Linearization> {
        self.evaluate_with(
            values,
            offsets,
            size,
            disabled,
            jacobians,
            &mut Workspace::default(),
        )
    }
}
struct Linearization {
    parallel: bool,
    cost: f64,
    fixed_cost: f64,
    rows: Vec<Row>,
    gradient: DVector<f64>,
    diagonal: DVector<f64>,
    eliminated_points: Vec<usize>,
}
impl Linearization {
    fn system(&self) -> SchurSystem<'_> {
        SchurSystem {
            rows: &self.rows,
            gradient: &self.gradient,
            eliminated_points: &self.eliminated_points,
        }
    }

    fn lower_normal(&self, regularization: &DVector<f64>) -> DMatrix<f64> {
        self.system().lower_normal(regularization)
    }
    fn scaled(&mut self, scale: &DVector<f64>) {
        for row in &mut self.rows {
            for (offset, j) in &mut row.blocks {
                for c in 0..j.ncols() {
                    j.column_mut(c).scale_mut(scale[*offset + c]);
                }
            }
        }
        self.gradient.component_mul_assign(scale);
        self.diagonal
            .zip_apply(scale, |diagonal, scale| *diagonal *= scale * scale);
    }
    fn product_sum(
        &self,
        v: &DVector<f64>,
        mut contribution: impl FnMut(DVectorView<'_, f64>, DVectorViewMut<'_, f64>, &Row) -> f64,
    ) -> f64 {
        let max_rows = self
            .rows
            .iter()
            .map(|row| row.residual.len())
            .max()
            .unwrap_or(0);
        let mut scratch = DVector::zeros(max_rows);
        let mut block_scratch = DVector::zeros(max_rows);
        self.rows
            .iter()
            .map(|row| {
                let mut product = scratch.rows_mut(0, row.residual.len());
                let mut block_product = block_scratch.rows_mut(0, row.residual.len());
                product.fill(0.0);
                for (offset, j) in &row.blocks {
                    // Complete block-local cancellation before adding earlier blocks.
                    block_product.gemv(1.0, j, &v.rows(*offset, j.ncols()), 0.0);
                    product += &block_product;
                }
                contribution(product.as_view(), block_product, row)
            })
            .sum()
    }
    fn predicted_change(&self, step: &DVector<f64>) -> f64 {
        self.product_sum(step, |product, mut workspace, row| {
            // Cancel the residual and half-step before multiplication to avoid overflow.
            workspace.copy_from(&row.residual);
            workspace.axpy(0.5, &product, 1.0);
            -product.dot(&workspace)
        })
    }
    #[cfg(test)]
    fn solve_schur(&self, lower: &DMatrix<f64>) -> Option<DVector<f64>> {
        self.system().solve_dense_normal(lower)
    }
    fn solve(&self, regularization: &DVector<f64>, kind: &str) -> Option<DVector<f64>> {
        let count = self.gradient.len();
        if kind == "DENSE_QR" {
            let measurement: usize = self
                .rows
                .iter()
                .map(|block_row| block_row.residual.len())
                .sum();
            let mut design_matrix = faer::Mat::<f64>::zeros(measurement + count, count);
            let mut rhs = faer::Mat::<f64>::zeros(measurement + count, 1);
            let mut start = 0;
            for row in &self.rows {
                for index in 0..row.residual.len() {
                    rhs[(start + index, 0)] = -row.residual[index];
                }
                for (offset, column) in &row.blocks {
                    for block_column in 0..column.ncols() {
                        for block_row in 0..column.nrows() {
                            design_matrix[(start + block_row, offset + block_column)] +=
                                column[(block_row, block_column)];
                        }
                    }
                }
                start += row.residual.len();
            }
            for index in 0..count {
                design_matrix[(measurement + index, index)] = regularization[index].sqrt();
            }
            let value = design_matrix.qr().solve_lstsq(rhs.as_ref());
            let out = DVector::from_iterator(count, (0..count).map(|index| value[(index, 0)]));
            return out.iter().all(|value| value.is_finite()).then_some(out);
        }
        if !matches!(kind, "SPARSE_NORMAL_CHOLESKY" | "DENSE_SCHUR") {
            return None;
        }
        if kind == "DENSE_SCHUR" {
            return if self.parallel {
                self.system().block_with_parallel(regularization, true)
            } else {
                self.system().block(regularization)
            };
        }
        let lower = self.lower_normal(regularization);
        let triplet_source = &lower;
        let triplets: Vec<_> = (0..count)
            .flat_map(|index| {
                (0..=index).filter_map(move |column| {
                    let value = triplet_source[(index, column)];
                    (index == column || value != 0.0).then(|| Triplet::new(index, column, value))
                })
            })
            .collect();
        let normal_matrix =
            SparseColMat::<usize, f64>::try_new_from_triplets(count, count, &triplets).ok()?;
        let factor = normal_matrix.sp_cholesky(faer::Side::Lower).ok()?;
        let rhs = faer::Mat::from_fn(count, 1, |index, _| -self.gradient[index]);
        let value = factor.solve(rhs.as_ref());
        let out = DVector::from_iterator(count, (0..count).map(|index| value[(index, 0)]));
        out.iter().all(|value| value.is_finite()).then_some(out)
    }
}

struct Strategy {
    radius: f64,
    mu: f64,
    decrease: f64,
    gn: Option<DVector<f64>>,
    diagonal: DVector<f64>,
    gradient: DVector<f64>,
    alpha: f64,
    dogleg_norm: f64,
}
impl Strategy {
    fn new(n: usize) -> Self {
        Self {
            radius: 1e4,
            mu: 1e-8,
            decrease: 2.0,
            gn: None,
            diagonal: DVector::zeros(n),
            gradient: DVector::zeros(n),
            alpha: 0.0,
            dogleg_norm: 0.0,
        }
    }
    fn step(&mut self, m: &Linearization, options: &SolverOptions) -> Option<DVector<f64>> {
        if options.trust_region == "LEVENBERG_MARQUARDT" {
            let regularization = m.diagonal.map(|x| x.clamp(1e-6, 1e32) / self.radius);
            return m.solve(&regularization, options.linear_solver);
        }
        if self.gn.is_none() {
            self.diagonal = m.diagonal.map(|x| x.clamp(1e-6, 1e32).sqrt());
            self.gradient = m.gradient.component_div(&self.diagonal);
            let v = self.gradient.component_div(&self.diagonal);
            let jg_squared = m.product_sum(&v, |product, _, _| product.norm_squared());
            self.alpha = self.gradient.norm_squared() / jg_squared;
            while self.mu < 1.0 {
                let r = self.diagonal.component_mul(&self.diagonal) * self.mu;
                if let Some(x) = m.solve(&r, options.linear_solver) {
                    self.gn = Some(x.component_mul(&self.diagonal));
                    break;
                }
                self.mu *= 10.0;
            }
        }
        let gn = self.gn.as_ref()?;
        let gnorm = self.gradient.norm();
        let bnorm = gn.norm();
        let mut s = if bnorm <= self.radius {
            gn.clone()
        } else if gnorm * self.alpha >= self.radius {
            (-self.radius / gnorm) * &self.gradient
        } else {
            let ba = -self.alpha * self.gradient.dot(gn);
            let a2 = (self.alpha * gnorm).powi(2);
            let bm_a2 = a2 - 2.0 * ba + bnorm.powi(2);
            let c = ba - a2;
            let d = (c * c + bm_a2 * (self.radius.powi(2) - a2)).sqrt();
            let beta = if c <= 0.0 {
                (d - c) / bm_a2
            } else {
                (self.radius.powi(2) - a2) / (d + c)
            };
            (-self.alpha * (1.0 - beta)) * &self.gradient + beta * gn
        };
        self.dogleg_norm = s.norm();
        s.component_div_assign(&self.diagonal);
        Some(s)
    }
    fn accepted(&mut self, quality: f64, lm: bool) {
        if lm {
            self.radius =
                (self.radius / (1.0 / 3.0_f64).max(1.0 - (2.0 * quality - 1.0).powi(3))).min(1e16);
            self.decrease = 2.0;
        } else {
            if quality < 0.25 {
                self.radius *= 0.5;
            }
            if quality > 0.75 {
                self.radius = self.radius.max(3.0 * self.dogleg_norm);
            }
            self.mu = (2.0 * self.mu / 10.0).max(1e-8);
            self.gn = None;
        }
    }
    fn rejected(&mut self, lm: bool) {
        if lm {
            self.radius /= self.decrease;
            self.decrease *= 2.0;
        } else {
            self.radius *= 0.5;
        }
    }
    fn invalid(&mut self, lm: bool) {
        if lm {
            self.rejected(true);
        } else {
            self.mu *= 10.0;
            self.gn = None;
        }
    }
}

fn ambient_norm(values: &BTreeMap<usize, Values>, offsets: &BTreeMap<usize, usize>) -> f64 {
    offsets
        .keys()
        .flat_map(|k| values[k].data.iter())
        .map(|x| x * x)
        .sum::<f64>()
        .sqrt()
}
fn ambient_difference(
    a: &BTreeMap<usize, Values>,
    b: &BTreeMap<usize, Values>,
    offsets: &BTreeMap<usize, usize>,
) -> (f64, f64) {
    let mut norm = 0.0;
    let mut maximum = 0.0_f64;
    for key in offsets.keys() {
        for (a, b) in a[key].data.iter().zip(&b[key].data) {
            let d = a - b;
            norm += d * d;
            maximum = maximum.max(d.abs());
        }
    }
    (norm.sqrt(), maximum)
}
impl Snapshot {
    #[cfg(test)]
    fn run(
        &mut self,
        options: &SolverOptions,
        disabled: &BTreeSet<ResidualId>,
    ) -> Result<SolverSummary> {
        let mut execution = Execution::default();
        execution.configure(options.num_threads)?;
        let mut workspace = Workspace::default();
        match execution.pool() {
            Some(pool) => pool.install(|| self.run_with(options, disabled, &mut workspace)),
            None => self.run_with(options, disabled, &mut workspace),
        }
    }
    fn run_with(
        &mut self,
        options: &SolverOptions,
        disabled: &BTreeSet<ResidualId>,
        workspace: &mut Workspace,
    ) -> Result<SolverSummary> {
        workspace.begin(options.num_threads > 1, options.cache_fixed_factors);
        let (offsets, size) = self.variable_offsets(options, disabled)?;
        let start = Instant::now();
        let mut values = self.values.clone();
        let mut model = self.evaluate_with(&values, &offsets, size, disabled, true, workspace)?;
        let mut summary = Self::initial_summary(&model);
        if size == 0 {
            summary.report = "No varying parameter blocks".into();
            return Ok(summary);
        }
        let scale = model.diagonal.map(|x| 1.0 / (1.0 + x.sqrt()));
        let mut strategy = Strategy::new(size);
        let mut invalid_steps = 0;
        let lm = options.trust_region == "LEVENBERG_MARQUARDT";
        let mut iteration_time = start.elapsed().as_secs_f64();
        let mut accepted = true;
        let mut had_successful_step = false;
        let mut report = "Maximum iteration count reached";
        for iteration in 0..=options.max_iterations {
            let elapsed = start.elapsed().as_secs_f64();
            if options.time_limit.is_some_and(|limit| {
                iteration >= options.minimum_iterations && elapsed + iteration_time > limit
            }) {
                report = "Optimization time callback terminated successfully";
                break;
            }
            if accepted {
                let negative = -&model.gradient;
                let projected = Self::plus(&values, &offsets, &negative)?;
                if ambient_difference(&values, &projected, &offsets).1 <= 1e-10 {
                    report = "Gradient tolerance reached";
                    break;
                }
            }
            if strategy.radius <= 1e-32 {
                report = "Minimum trust region radius reached";
                break;
            }
            if iteration == options.max_iterations {
                break;
            }
            let iteration_start = Instant::now();
            if accepted {
                model.scaled(&scale);
            }
            accepted = false;
            summary.iterations = iteration + 1;
            let Some((delta, predicted)) =
                Self::propose_step(&mut strategy, &model, options, &mut invalid_steps, &scale)?
            else {
                iteration_time = iteration_start.elapsed().as_secs_f64();
                continue;
            };
            let candidate = Self::plus(&values, &offsets, &delta);
            let cost = candidate
                .as_ref()
                .ok()
                .and_then(|x| {
                    self.evaluate_with(x, &offsets, size, disabled, false, workspace)
                        .ok()
                })
                .map_or(f64::MAX, |m| m.cost);
            if let Ok(candidate) = candidate {
                let terminate = Self::candidate_termination(
                    &values,
                    &candidate,
                    &offsets,
                    &model,
                    cost,
                    (had_successful_step, options.function_tolerance),
                );
                if let Some(reason) = terminate {
                    summary.iterations = iteration;
                    report = reason;
                    break;
                }
                let quality = if cost >= f64::MAX {
                    f64::MIN
                } else {
                    (model.cost - cost) / predicted
                };
                if quality > 1e-3 {
                    values = candidate;
                    had_successful_step = true;
                    strategy.accepted(quality, lm);
                    workspace.recycle(model);
                    model =
                        self.evaluate_with(&values, &offsets, size, disabled, true, workspace)?;
                    summary.final_cost = Some(model.cost + model.fixed_cost);
                    accepted = true;
                } else {
                    strategy.rejected(lm);
                }
                Self::log_iteration(iteration, &model, cost, quality, &strategy, options.verbose);
            } else {
                strategy.rejected(lm);
            }
            iteration_time = iteration_start.elapsed().as_secs_f64();
        }
        Ok(self.finish_solve(values, model, summary, report, workspace))
    }
    fn finish_solve(
        &mut self,
        values: BTreeMap<usize, Values>,
        model: Linearization,
        mut summary: SolverSummary,
        report: &str,
        workspace: &mut Workspace,
    ) -> SolverSummary {
        self.values = values;
        summary.report = report.into();
        summary.final_cost = Some(model.cost + model.fixed_cost);
        workspace.recycle(model);
        summary
    }
    fn log_iteration(
        iteration: usize,
        model: &Linearization,
        cost: f64,
        quality: f64,
        strategy: &Strategy,
        verbose: bool,
    ) {
        if verbose {
            log::info!(
                "iteration={} cost={} candidate={} relative_decrease={} radius={}",
                iteration + 1,
                model.cost,
                cost,
                quality,
                strategy.radius
            );
        }
    }

    fn propose_step(
        strategy: &mut Strategy,
        model: &Linearization,
        options: &SolverOptions,
        invalid_steps: &mut usize,
        scale: &DVector<f64>,
    ) -> Result<Option<(DVector<f64>, f64)>> {
        let lm = options.trust_region == "LEVENBERG_MARQUARDT";
        let step = strategy.step(model, options);
        let predicted = step
            .as_ref()
            .map_or(f64::NEG_INFINITY, |s| model.predicted_change(s));
        if step
            .as_ref()
            .is_none_or(|s| s.iter().any(|x| !x.is_finite()))
            || predicted.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater)
        {
            *invalid_steps += 1;
            strategy.invalid(lm);
            if *invalid_steps >= 5 {
                return Err(graph_error("five consecutive invalid trust region steps"));
            }
            return Ok(None);
        }
        *invalid_steps = 0;
        let delta = step.expect("valid step").component_mul(scale);
        Ok(Some((delta, predicted)))
    }

    fn candidate_termination(
        values: &BTreeMap<usize, Values>,
        candidate: &BTreeMap<usize, Values>,
        offsets: &BTreeMap<usize, usize>,
        model: &Linearization,
        cost: f64,
        (had_successful_step, function_tolerance): (bool, f64),
    ) -> Option<&'static str> {
        let mut terminate = None;
        let step_norm = ambient_difference(values, candidate, offsets).0;
        if had_successful_step && step_norm <= 1e-8 * (ambient_norm(values, offsets) + 1e-8) {
            terminate = Some("Parameter tolerance reached");
        }
        if (model.cost - cost).abs() <= function_tolerance * model.cost {
            terminate = Some("Function tolerance reached");
        }
        terminate
    }

    fn initial_summary(model: &Linearization) -> SolverSummary {
        let initial_cost = model.cost + model.fixed_cost;

        SolverSummary {
            initial_cost: Some(initial_cost),
            final_cost: Some(initial_cost),
            successful: true,
            ..Default::default()
        }
    }

    fn variable_offsets(
        &self,
        options: &SolverOptions,
        disabled: &BTreeSet<ResidualId>,
    ) -> Result<(BTreeMap<usize, usize>, usize)> {
        if !matches!(options.trust_region, "DOGLEG" | "LEVENBERG_MARQUARDT")
            || !matches!(
                options.linear_solver,
                "SPARSE_NORMAL_CHOLESKY" | "DENSE_QR" | "DENSE_SCHUR"
            )
            || options.function_tolerance < 0.0
            || options.num_threads == 0
        {
            return Err(graph_error("invalid graph solver options"));
        }
        let mut offsets = BTreeMap::new();
        let mut size = 0;
        // Ceres removes variable blocks that participate in no active residual.
        let participating: BTreeSet<_> = self
            .factors
            .iter()
            .filter(|f| !disabled.contains(&f.id))
            .flat_map(|f| f.keys.iter().copied())
            .collect();
        for (&key, v) in &self.values {
            if !v.fixed && participating.contains(&key) {
                offsets.insert(key, size);
                size += v.tangent_size();
            }
        }
        Ok((offsets, size))
    }
}
impl GraphSolver for RustGraphSolver {
    fn solve(
        &mut self,
        problem: &mut GraphProblem,
        options: &SolverOptions,
    ) -> Result<SolverSummary> {
        let start =
            log::log_enabled!(target: "solver-profile", log::Level::Debug).then(Instant::now);
        let mut snapshot = Snapshot::capture(problem)?;
        let captured = start.map(|s| s.elapsed().as_secs_f64());
        self.execution.configure(options.num_threads)?;
        let summary = self
            .execution
            .install(|| snapshot.run_with(options, &BTreeSet::new(), &mut self.workspace))?;
        let solved = start.map(|s| s.elapsed().as_secs_f64());
        snapshot.write_values(problem)?;
        let written = start.map(|s| s.elapsed().as_secs_f64());
        snapshot.restore_caches(problem);
        if let Some(start) = start {
            let restored = start.elapsed().as_secs_f64();
            let (offsets, dimensions) = snapshot.variable_offsets(options, &BTreeSet::new())?;
            let constant = snapshot
                .factors
                .iter()
                .filter(|f| f.keys.iter().all(|k| !offsets.contains_key(k)))
                .count();
            let points = offsets
                .keys()
                .filter(|k| matches!(snapshot.values[k].manifold, ManifoldKind::HomogeneousPoint))
                .count();
            log::debug!(target: "solver-profile", "factors={} constant={constant} dimensions={dimensions} points={points} iterations={} evaluations={} factor_evaluations={} fixed_cache={} capture_ms={:.6} solve_ms={:.6} write_ms={:.6} caches_ms={:.6}",
                snapshot.factors.len(), summary.iterations, self.workspace.evaluation_calls,
                self.workspace.factor_evaluations(), options.cache_fixed_factors,
                captured.unwrap() * 1000.0, (solved.unwrap() - captured.unwrap()) * 1000.0,
                (written.unwrap() - solved.unwrap()) * 1000.0, (restored - written.unwrap()) * 1000.0);
        }
        Ok(summary)
    }
    fn prepare_solve(
        &mut self,
        problem: &GraphProblem,
        stages: Vec<SolverStage>,
    ) -> Result<Box<dyn FnOnce() -> Result<SolverUpdate> + Send>> {
        let mut snapshot = Snapshot::capture(problem)?;
        // Prepared jobs own their workspace and never borrow the live graph.
        let mut execution = Execution::default();
        let mut workspace = Workspace::default();
        Ok(Box::new(move || {
            let mut summary = SolverSummary::default();
            for stage in stages {
                execution.configure(stage.options.num_threads)?;
                summary = execution.install(|| {
                    snapshot.run_with(&stage.options, &stage.disabled_residuals, &mut workspace)
                })?;
            }
            let caches = snapshot
                .factors
                .iter()
                .filter_map(|f| match &f.factor {
                    Factor::Imu(factor) => Some((f.id, SolverFactorCache::Imu(factor.clone()))),
                    Factor::Gps(factor) => Some((f.id, SolverFactorCache::Gps(factor.clone()))),
                    _ => None,
                })
                .collect();
            Ok(SolverUpdate {
                caches,
                parameters: snapshot
                    .values
                    .into_iter()
                    .map(|(key, v)| (key, v.data))
                    .collect(),
                summary,
            })
        }))
    }
    fn refine_alignment_4d(
        &mut self,
        gps: &[Vector3<f64>],
        world: &[Vector3<f64>],
        initial: &Transformation,
        iterations: usize,
        cauchy_scale: f64,
    ) -> Result<Transformation> {
        if gps.is_empty() || gps.len() != world.len() {
            return Err(graph_error(
                "alignment point clouds must match and be nonempty",
            ));
        }
        let mut snapshot = Snapshot {
            values: BTreeMap::from([(
                0,
                Values {
                    data: initial.coeffs().as_slice().to_vec(),
                    manifold: ManifoldKind::Pose4d,
                    fixed: false,
                },
            )]),
            factors: gps
                .iter()
                .zip(world)
                .enumerate()
                .map(|(i, (gps, world))| FactorRecord {
                    id: i as u64,
                    factor: Factor::Alignment {
                        gps: *gps,
                        world: *world,
                    },
                    keys: vec![0],
                    loss: Some(RobustLoss::Cauchy(cauchy_scale)),
                })
                .collect(),
        };
        snapshot.run_with(
            &SolverOptions {
                max_iterations: iterations,
                trust_region: "LEVENBERG_MARQUARDT",
                linear_solver: "DENSE_QR",
                ..Default::default()
            },
            &BTreeSet::new(),
            &mut self.workspace,
        )?;
        let p = &snapshot.values[&0].data;
        Ok(Transformation::new(
            Vector3::from_column_slice(&p[..3]),
            Quaternion::new(p[6], p[3], p[4], p[5]),
        ))
    }
}

impl RustGraphSolver {
    /// Extracts an undamped marginal block, conditional on the graph's fixed gauge.
    /// Singular normal equations return None; no epsilon prior is introduced.
    /// # Errors
    /// Returns an error if factor snapshots or linearization are invalid.
    pub fn marginal_covariance(
        problem: &GraphProblem,
        parameters: &[Parameter],
    ) -> Result<Option<DMatrix<f64>>> {
        let mut snapshot = Snapshot::capture(problem)?;
        let disabled = BTreeSet::new();
        let (offsets, size) = snapshot.variable_offsets(&SolverOptions::default(), &disabled)?;
        let values = snapshot.values.clone();
        let model = snapshot.evaluate(&values, &offsets, size, &disabled, true)?;
        let mut columns = Vec::new();
        for parameter in parameters {
            let key = parameter.key();
            let value = values
                .get(&key)
                .ok_or_else(|| graph_error("covariance parameter is not registered"))?;
            if !value.fixed && !offsets.contains_key(&key) {
                return Ok(None);
            }
            let offset = offsets.get(&key).copied();
            columns.extend((0..value.tangent_size()).map(|axis| offset.map(|start| start + axis)));
        }
        if size == 0 {
            return Ok(Some(DMatrix::zeros(columns.len(), columns.len())));
        }
        let lower = model.lower_normal(&DVector::zeros(size));
        let mut triplets = Vec::with_capacity(size * (size + 1) / 2);
        for row in 0..size {
            for col in 0..=row {
                triplets.push(Triplet::new(row, col, lower[(row, col)]));
            }
        }
        let Ok(normal) = SparseColMat::<usize, f64>::try_new_from_triplets(size, size, &triplets)
        else {
            return Ok(None);
        };
        let Ok(factor) = normal.sp_cholesky(faer::Side::Lower) else {
            return Ok(None);
        };
        let rhs = faer::Mat::from_fn(size, columns.len(), |row, col| {
            f64::from(columns[col] == Some(row))
        });
        let solved = factor.solve(rhs.as_ref());
        let covariance = DMatrix::from_fn(columns.len(), columns.len(), |row, col| {
            columns[row].map_or(0.0, |index| solved[(index, col)])
        });
        if covariance.iter().any(|x| !x.is_finite())
            || covariance.diagonal().iter().any(|x| *x < 0.0)
        {
            return Ok(None);
        }
        Ok(Some((&covariance + covariance.transpose()) * 0.5))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block_linearization() -> (Linearization, DMatrix<f64>, DVector<f64>) {
        let design = DMatrix::from_row_slice(
            6,
            4,
            &[
                1., 2., -1., 0.5, 3., 4., 2., -3., 0., 2., -1., 3., 0., 0., 4., -2., 0., 1., 0.,
                2., 0., 0., 0., 0.,
            ],
        );
        let residual = DVector::from_row_slice(&[1., -2., 0.5, 1., -0.25, 7.]);
        let model = Linearization {
            parallel: false,
            cost: 0.5 * residual.norm_squared(),
            fixed_cost: 0.,
            rows: vec![
                Row {
                    residual: DVector::from_row_slice(&[1., -2.]),
                    blocks: vec![
                        (0, DMatrix::from_row_slice(2, 2, &[1., 2., 3., 4.])),
                        (2, DMatrix::from_row_slice(2, 2, &[-1., 0.5, 2., -3.])),
                        (0, DMatrix::zeros(2, 0)),
                    ],
                },
                Row {
                    residual: DVector::from_row_slice(&[0.5, 1., -0.25]),
                    blocks: vec![(
                        1,
                        DMatrix::from_row_slice(3, 3, &[2., -1., 3., 0., 4., -2., 1., 0., 2.]),
                    )],
                },
                Row {
                    residual: DVector::from_element(1, 7.),
                    blocks: vec![],
                },
                Row {
                    residual: DVector::zeros(0),
                    blocks: vec![(0, DMatrix::zeros(0, 4))],
                },
            ],
            gradient: design.tr_mul(&residual),
            diagonal: DVector::from_iterator(4, design.column_iter().map(|c| c.norm_squared())),
            eliminated_points: vec![],
        };
        (model, design, residual)
    }

    #[test]
    fn streamed_products_and_prediction_match_the_dense_quadratic() {
        let (mut model, design, residual) = block_linearization();
        for scale in [0., -0.25, 1., 2.] {
            let step = DVector::from_row_slice(&[0.5, -1., 0.25, 2.]) * scale;
            let product = &design * &step;
            let squared_norm = model.product_sum(&step, |product, _, _| product.norm_squared());
            assert!((squared_norm - product.norm_squared()).abs() < 1e-12);
            let expected = 0.5 * (residual.norm_squared() - (&residual + product).norm_squared());
            assert!((model.predicted_change(&step) - expected).abs() < 1e-12);
        }
        model.rows.clear();
        let step = DVector::repeat(4, 1.);
        assert_eq!(
            model.product_sum(&step, |product, _, _| product.norm_squared()),
            0.
        );
        assert_eq!(model.predicted_change(&step), 0.);
    }

    #[test]
    fn predicted_change_keeps_cancellation_before_multiplication_at_large_magnitudes() {
        let mut model = Linearization {
            parallel: false,
            cost: 0.,
            fixed_cost: 0.,
            rows: vec![Row {
                residual: DVector::from_element(1, -0.75e154),
                blocks: vec![(0, DMatrix::identity(1, 1))],
            }],
            gradient: DVector::zeros(1),
            diagonal: DVector::repeat(1, 1.),
            eliminated_points: vec![],
        };
        let step = DVector::<f64>::from_element(1, 1.5e154);
        assert!(step.norm_squared().is_infinite());
        assert_eq!(model.predicted_change(&step), 0.);
        model.rows[0].residual[0] = -0.8e154;
        let expected = -step[0] * (model.rows[0].residual[0] + 0.5 * step[0]);
        assert!(expected.is_finite() && expected > 0.);
        assert_eq!(model.predicted_change(&step), expected);
    }

    #[test]
    fn streamed_product_preserves_small_blocks_when_another_block_cancels() {
        let model = Linearization {
            parallel: false,
            cost: 0.5,
            fixed_cost: 0.,
            rows: vec![Row {
                residual: DVector::from_element(1, -1.),
                blocks: vec![
                    (0, DMatrix::from_element(1, 1, 1.)),
                    (1, DMatrix::from_row_slice(1, 2, &[1e16, -1e16])),
                ],
            }],
            gradient: DVector::zeros(3),
            diagonal: DVector::repeat(3, 1.),
            eliminated_points: vec![],
        };
        let step = DVector::repeat(3, 1.);
        assert_eq!(
            model.product_sum(&step, |product, _, _| product.norm_squared()),
            1.
        );
        assert_eq!(model.predicted_change(&step), 0.5);
    }

    #[test]
    fn streamed_reduction_preserves_owned_product_and_dot_evaluation() {
        for rows in [1, 2, 9, 15] {
            let row = Row {
                residual: DVector::from_fn(rows, |r, _| (r as f64 + 0.5).cos()),
                blocks: [(0, 3), (3, 9), (12, 6)]
                    .into_iter()
                    .map(|(offset, columns)| {
                        (
                            offset,
                            DMatrix::from_fn(rows, columns, |r, c| {
                                ((r + offset + 1) as f64 * (c + 1) as f64).sin()
                            }),
                        )
                    })
                    .collect(),
            };
            let step = DVector::from_fn(18, |i, _| (i as f64 + 0.25).sin());
            let owned_product = row
                .blocks
                .iter()
                .fold(DVector::zeros(rows), |sum, (offset, j)| {
                    sum + j * step.rows(*offset, j.ncols())
                });
            let expected = -owned_product.dot(&(&row.residual + &owned_product * 0.5));
            let model = Linearization {
                parallel: false,
                cost: 0.,
                fixed_cost: 0.,
                rows: vec![row],
                gradient: DVector::zeros(18),
                diagonal: DVector::repeat(18, 1.),
                eliminated_points: vec![],
            };
            assert_eq!(model.predicted_change(&step), expected);
            assert_eq!(
                model.product_sum(&step, |product, _, _| product.norm_squared()),
                owned_product.norm_squared()
            );
        }
    }

    #[test]
    fn diagonal_scaling_matches_dense_column_scaling_and_reuses_storage() {
        let (mut model, design, residual) = block_linearization();
        let scale = DVector::from_row_slice(&[0., -2., 0.5, 3.]);
        let gradient_pointer = model.gradient.as_ptr();
        let diagonal_pointer = model.diagonal.as_ptr();
        model.scaled(&scale);
        let scaled_design = &design * DMatrix::from_diagonal(&scale);
        let expected_gradient = scaled_design.tr_mul(&residual);
        let expected_diagonal = DVector::from_iterator(
            4,
            scaled_design
                .column_iter()
                .map(|column| column.norm_squared()),
        );
        assert_eq!(model.gradient.as_ptr(), gradient_pointer);
        assert_eq!(model.diagonal.as_ptr(), diagonal_pointer);
        assert!((&model.gradient - expected_gradient).amax() < 1e-12);
        assert!((&model.diagonal - expected_diagonal).amax() < 1e-12);
        let step = DVector::from_row_slice(&[0.5, -1., 0.25, 2.]);
        let expected =
            0.5 * (residual.norm_squared() - (&residual + scaled_design * &step).norm_squared());
        assert!((model.predicted_change(&step) - expected).abs() < 1e-12);
    }

    #[test]
    fn gradient_accumulation_matches_weighted_priors_at_distinct_offsets() {
        let values = BTreeMap::from([1, 2].map(|key| {
            (
                key,
                Values {
                    data: (0..9).map(|i| (key + i) as f64 / 10.).collect(),
                    manifold: ManifoldKind::Euclidean,
                    fixed: false,
                },
            )
        }));
        let factors = (0..)
            .zip([(1, 4., -2.), (1, 9., 3.), (2, 1., 0.)])
            .map(|(id, (key, weight, measurement))| FactorRecord {
                id,
                factor: Factor::SpeedBiasPrior(Box::new(SpeedAndBiasError::new(
                    SVector::repeat(measurement),
                    SMatrix::identity() * weight,
                ))),
                keys: vec![key],
                loss: None,
            })
            .collect();
        let mut snapshot = Snapshot {
            values: values.clone(),
            factors,
        };
        let model = snapshot
            .evaluate(
                &values,
                &BTreeMap::from([(1, 0), (2, 9)]),
                18,
                &BTreeSet::new(),
                true,
            )
            .unwrap();
        for i in 0..9 {
            let x = values[&1].data[i];
            let expected = 4. * (x + 2.) + 9. * (x - 3.);
            assert!((model.gradient[i] - expected).abs() < 1e-12);
            assert_eq!(model.diagonal[i], 13.);
            assert_eq!(model.gradient[9 + i], values[&2].data[i]);
            assert_eq!(model.diagonal[9 + i], 1.);
        }
    }

    #[test]
    fn euclidean_jacobian_conversion_reuses_the_input_buffer() {
        for dimension in [1, 9] {
            let value = Values {
                data: vec![0.; dimension],
                manifold: ManifoldKind::Euclidean,
                fixed: false,
            };
            let jacobian = DMatrix::from_fn(15, dimension, |r, c| (r as f64 - 2. * c as f64) / 3.);
            let expected = jacobian.clone();
            let pointer = jacobian.as_ptr();
            let result = value.apply_plus_jacobian(jacobian);
            assert_eq!(result.as_ptr(), pointer);
            assert_eq!(result, expected);
        }
    }

    #[test]
    fn manifold_jacobian_conversion_matches_update_differences() {
        for (manifold, data) in [
            (ManifoldKind::Pose, vec![1., -2., 3., 0., 0., 0.6, 0.8]),
            (ManifoldKind::Pose4d, vec![1., -2., 3., 0., 0., 0.6, 0.8]),
            (ManifoldKind::HomogeneousPoint, vec![2., -1., 3., 1.]),
        ] {
            let value = Values {
                data,
                manifold,
                fixed: false,
            };
            let jacobian =
                DMatrix::from_fn(5, value.data.len(), |r, c| (r as f64 - 2. * c as f64) / 3.);
            let epsilon = 1e-6;
            let mut differential = DMatrix::zeros(value.data.len(), value.tangent_size());
            for column in 0..value.tangent_size() {
                let mut delta = vec![0.; value.tangent_size()];
                delta[column] = epsilon;
                let positive = value.plus(&delta).unwrap();
                delta[column] = -epsilon;
                let negative = value.plus(&delta).unwrap();
                for row in 0..value.data.len() {
                    differential[(row, column)] = (positive[row] - negative[row]) / (2. * epsilon);
                }
            }
            let expected = &jacobian * differential;
            let result = value.apply_plus_jacobian(jacobian);
            assert_eq!(result.shape(), (5, value.tangent_size()));
            assert!((result - expected).amax() < 1e-8);
        }
    }

    fn mixed_factor_snapshot() -> Snapshot {
        let values = BTreeMap::from([
            (
                1,
                Values {
                    data: vec![0., 0., 0., 0., 0., 0., 1.],
                    manifold: ManifoldKind::Pose,
                    fixed: false,
                },
            ),
            (
                2,
                Values {
                    data: vec![0., 0., 0., 0., 0., 0., 1.],
                    manifold: ManifoldKind::Pose,
                    fixed: true,
                },
            ),
            (
                3,
                Values {
                    data: vec![0., 0., 2., 1.],
                    manifold: ManifoldKind::HomogeneousPoint,
                    fixed: false,
                },
            ),
        ]);
        Snapshot {
            values,
            factors: vec![
                FactorRecord {
                    id: 1,
                    factor: Factor::Alignment {
                        gps: Vector3::zeros(),
                        world: Vector3::new(3., 0., 0.),
                    },
                    keys: vec![1],
                    loss: Some(RobustLoss::Huber(1.)),
                },
                FactorRecord {
                    id: 2,
                    factor: Factor::Alignment {
                        gps: Vector3::zeros(),
                        world: Vector3::new(2., 0., 0.),
                    },
                    keys: vec![2],
                    loss: None,
                },
                FactorRecord {
                    id: 3,
                    factor: Factor::Depth(OneSidedDepthError::new(5., 1.)),
                    keys: vec![1, 3, 2],
                    loss: None,
                },
            ],
        }
    }

    #[test]
    fn production_schur_matches_qr_for_scaled_manifold_factors() {
        let mut snapshot = mixed_factor_snapshot();
        let values = snapshot.values.clone();
        let offsets = BTreeMap::from([(1, 0), (3, 6)]);
        let mut model = snapshot
            .evaluate(&values, &offsets, 9, &BTreeSet::new(), true)
            .unwrap();
        assert_eq!(model.eliminated_points, vec![6]);
        model.scaled(&DVector::from_fn(9, |i, _| 0.5 + i as f64));
        for damping in [1e-5, 0.1, 10.0] {
            let regularization = DVector::from_element(9, damping);
            let expected = model.solve(&regularization, "DENSE_QR").unwrap();
            let actual = model.solve(&regularization, "DENSE_SCHUR").unwrap();
            assert!((&actual - &expected).amax() <= 1e-9 * (1.0 + expected.amax()));
        }
    }

    #[test]
    fn cost_only_matches_linearization_without_allocating_linearization_buffers() {
        let mut snapshot = mixed_factor_snapshot();
        let values = snapshot.values.clone();
        let offsets = BTreeMap::from([(1, 0), (3, 6)]);
        for disabled in [BTreeSet::new(), BTreeSet::from([1])] {
            let full = snapshot
                .evaluate(&values, &offsets, 9, &disabled, true)
                .unwrap();
            let cost = snapshot
                .evaluate(&values, &offsets, 9, &disabled, false)
                .unwrap();
            assert_eq!(full.cost, if disabled.contains(&1) { 4.5 } else { 7. });
            assert_eq!(full.fixed_cost, 2.);
            assert_eq!(cost.cost, full.cost);
            assert_eq!(cost.fixed_cost, full.fixed_cost);
            assert_eq!(full.gradient.len(), 9);
            assert_eq!(full.diagonal.len(), 9);
            assert_eq!(full.eliminated_points, vec![6]);
            assert!(!full.rows.is_empty());
            assert_eq!(cost.gradient.data.as_vec().capacity(), 0);
            assert_eq!(cost.diagonal.data.as_vec().capacity(), 0);
            assert_eq!(cost.eliminated_points.capacity(), 0);
            assert_eq!(cost.rows.capacity(), 0);
        }
        snapshot.factors[0].factor = Factor::Alignment {
            gps: Vector3::new(f64::NAN, 0., 0.),
            world: Vector3::zeros(),
        };
        for jacobians in [false, true] {
            assert!(
                snapshot
                    .evaluate(&values, &offsets, 9, &BTreeSet::new(), jacobians)
                    .is_err()
            );
        }
    }

    fn assert_parallel_evaluation(
        snapshot: &mut Snapshot,
        values: &BTreeMap<usize, Values>,
        offsets: &BTreeMap<usize, usize>,
        size: usize,
        expected: &Linearization,
    ) {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        let mut workspace = Workspace::default();
        workspace.parallel = true;
        let parallel = pool
            .install(|| {
                snapshot.evaluate_with(
                    values,
                    offsets,
                    size,
                    &BTreeSet::new(),
                    true,
                    &mut workspace,
                )
            })
            .unwrap();
        assert_eq!(parallel.cost, expected.cost);
        assert_eq!(parallel.fixed_cost, expected.fixed_cost);
        assert_eq!(parallel.gradient, expected.gradient);
        assert_eq!(parallel.diagonal, expected.diagonal);
        for (a, b) in parallel.rows.iter().zip(&expected.rows) {
            assert_eq!(a.residual, b.residual);
            assert_eq!(a.blocks, b.blocks);
        }
    }

    fn imu_factor_fixture() -> ImuError {
        use crate::{
            config::parameters::ImuParameters,
            sensors::measurements::{ImuMeasurement, ImuSensorReadings},
        };
        use jiff::Timestamp;

        let parameters = ImuParameters {
            r#use: true,
            t_bs: Transformation::identity(),
            a_max: 100.,
            g_max: 20.,
            sigma_g_c: 0.002,
            sigma_bg: 0.01,
            sigma_a_c: 0.02,
            sigma_ba: 0.02,
            sigma_gw_c: 0.0002,
            sigma_aw_c: 0.002,
            g0: Vector3::zeros(),
            a0: Vector3::zeros(),
            g: 9.81,
            s_a: Vector3::repeat(1.),
        };
        let samples = (0..=10)
            .map(|i| {
                ImuMeasurement::new(
                    Timestamp::from_millisecond(i * 10).unwrap(),
                    ImuSensorReadings::new(
                        Vector3::new(0.1, 0.2, -0.1),
                        Vector3::new(0.2, 0.1, 9.81),
                    ),
                    -1,
                )
            })
            .collect();
        ImuError::new(
            samples,
            parameters,
            Timestamp::UNIX_EPOCH,
            Timestamp::from_millisecond(100).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn imu_cost_only_matches_linearization_for_active_and_fixed_factors() {
        let factor = imu_factor_fixture();
        for fixed in [false, true] {
            let values = [
                (vec![0., 0., 0., 0., 0., 0., 1.], ManifoldKind::Pose),
                (vec![0.; 9], ManifoldKind::Euclidean),
                (vec![0.1, 0., 0., 0., 0., 0., 1.], ManifoldKind::Pose),
                (vec![0.; 9], ManifoldKind::Euclidean),
            ]
            .into_iter()
            .enumerate()
            .map(|(key, (data, manifold))| {
                (
                    key,
                    Values {
                        data,
                        manifold,
                        fixed,
                    },
                )
            })
            .collect::<BTreeMap<_, _>>();
            let mut snapshot = Snapshot {
                values: values.clone(),
                factors: vec![FactorRecord {
                    id: 1,
                    factor: Factor::Imu(Box::new(factor.clone())),
                    keys: vec![0, 1, 2, 3],
                    loss: Some(RobustLoss::Huber(1.)),
                }],
            };
            let offsets = if fixed {
                BTreeMap::new()
            } else {
                BTreeMap::from([(0, 0), (1, 6), (2, 15), (3, 21)])
            };
            let size = if fixed { 0 } else { 30 };
            let mut cached_snapshot = Snapshot {
                values: values.clone(),
                factors: vec![FactorRecord {
                    id: 1,
                    factor: Factor::Imu(Box::new(factor.clone())),
                    keys: vec![0, 1, 2, 3],
                    loss: Some(RobustLoss::Huber(1.)),
                }],
            };
            let mut workspace = Workspace::default();
            workspace.begin(false, true);
            for jacobians in [true, false, true] {
                let cached = cached_snapshot
                    .evaluate_with(
                        &values,
                        &offsets,
                        size,
                        &BTreeSet::new(),
                        jacobians,
                        &mut workspace,
                    )
                    .unwrap();
                let repeated = snapshot
                    .evaluate(&values, &offsets, size, &BTreeSet::new(), jacobians)
                    .unwrap();
                assert_eq!(cached.cost, repeated.cost);
                assert_eq!(cached.fixed_cost, repeated.fixed_cost);
                assert_eq!(cached.gradient, repeated.gradient);
                assert_eq!(cached.diagonal, repeated.diagonal);
                let Factor::Imu(a) = &cached_snapshot.factors[0].factor else {
                    unreachable!()
                };
                let Factor::Imu(b) = &snapshot.factors[0].factor else {
                    unreachable!()
                };
                assert_eq!(a.redo_counter, b.redo_counter);
                assert_eq!(a.preintegration.reference, b.preintegration.reference);
            }
            assert_eq!(workspace.factor_evaluations(), if fixed { 1 } else { 3 });
            let cost = snapshot
                .evaluate(&values, &offsets, size, &BTreeSet::new(), false)
                .unwrap();
            let full = snapshot
                .evaluate(&values, &offsets, size, &BTreeSet::new(), true)
                .unwrap();
            assert_parallel_evaluation(&mut snapshot, &values, &offsets, size, &full);
            assert_eq!(cost.cost, full.cost);
            assert_eq!(cost.fixed_cost, full.fixed_cost);
            assert!(cost.cost + cost.fixed_cost > 0.);
            assert_eq!(cost.cost == 0., fixed);
            assert_eq!(cost.fixed_cost == 0., !fixed);
            assert!(cost.rows.is_empty() && cost.gradient.is_empty());
            assert_eq!(full.rows.is_empty(), fixed);
            let inputs: Vec<_> = values.values().map(|v| v.data.as_slice()).collect();
            let factor = &mut snapshot.factors[0].factor;
            for mask in [None, Some([false; 4].as_slice())] {
                let result = factor.evaluate_motion(&inputs, mask).unwrap();
                assert!(result.jacobians.iter().all(Option::is_none));
                assert!(result.minimal_jacobians.iter().all(Option::is_none));
            }
        }
    }

    #[test]
    fn evaluation_reuses_scratch_with_shrinking_and_alternating_factor_inputs() {
        let mut snapshot = mixed_factor_snapshot();
        let values = snapshot.values.clone();
        let offsets = BTreeMap::from([(1, 0), (3, 6)]);
        let baseline = snapshot
            .evaluate(&values, &offsets, 9, &BTreeSet::new(), true)
            .unwrap();
        snapshot.factors.reverse();
        for disabled in [BTreeSet::new(), BTreeSet::from([2])] {
            let full = snapshot
                .evaluate(&values, &offsets, 9, &disabled, true)
                .unwrap();
            let cost = snapshot
                .evaluate(&values, &offsets, 9, &disabled, false)
                .unwrap();
            assert_eq!(full.cost, 7.);
            assert_eq!(full.fixed_cost, if disabled.contains(&2) { 0. } else { 2. });
            assert_eq!(cost.cost, full.cost);
            assert_eq!(cost.fixed_cost, full.fixed_cost);
            assert_eq!(full.gradient, baseline.gradient);
            assert_eq!(full.diagonal, baseline.diagonal);
            assert_eq!(full.rows.len(), 2);
            assert_eq!(full.rows[0].blocks.len(), 2);
            assert_eq!(full.rows[1].blocks.len(), 1);
            assert_eq!(full.rows[0].blocks[0].0, 0);
            assert_eq!(full.rows[0].blocks[1].0, 6);
            assert_eq!(full.rows[1].blocks[0].0, 0);
        }
    }

    #[test]
    fn parameter_updates_read_the_source_and_preserve_inactive_blocks() {
        let values = BTreeMap::from([
            (
                1,
                Values {
                    data: vec![1., 2., 3., 0., 0., 0., 1.],
                    manifold: ManifoldKind::Pose,
                    fixed: false,
                },
            ),
            (
                2,
                Values {
                    data: vec![4., 5., 6.],
                    manifold: ManifoldKind::Euclidean,
                    fixed: false,
                },
            ),
            (
                3,
                Values {
                    data: vec![2., 3., 4., 1.],
                    manifold: ManifoldKind::HomogeneousPoint,
                    fixed: true,
                },
            ),
            (
                4,
                Values {
                    data: vec![0., 0., 0., 0., 0., 0., 1.],
                    manifold: ManifoldKind::Pose4d,
                    fixed: false,
                },
            ),
        ]);
        let original = values.clone();
        let offsets = BTreeMap::from([(1, 0), (2, 6)]);
        for scale in [1., -2.] {
            let delta = DVector::from_row_slice(&[
                scale,
                2. * scale,
                3. * scale,
                0.,
                0.,
                0.,
                scale,
                -scale,
                2. * scale,
            ]);
            let next = Snapshot::plus(&values, &offsets, &delta).unwrap();
            assert_eq!(next.len(), values.len());
            assert_eq!(
                next[&1].data,
                vec![1. + scale, 2. + 2. * scale, 3. + 3. * scale, 0., 0., 0., 1.]
            );
            assert_eq!(next[&2].data, vec![4. + scale, 5. - scale, 6. + 2. * scale]);
            for key in [3, 4] {
                assert_eq!(next[&key].data, original[&key].data);
            }
            for (key, value) in &values {
                assert_eq!(value.data, original[key].data);
                assert_eq!(
                    std::mem::discriminant(&next[key].manifold),
                    std::mem::discriminant(&value.manifold)
                );
                assert_eq!(next[key].fixed, value.fixed);
            }
        }
        let mut invalid = DVector::zeros(9);
        invalid[6] = f64::NAN;
        assert!(Snapshot::plus(&values, &offsets, &invalid).is_err());
        for (key, value) in &values {
            assert_eq!(value.data, original[key].data);
        }
    }

    #[test]
    fn staged_solves_use_committed_values_and_preserve_nonparticipating_blocks() {
        for trust_region in ["DOGLEG", "LEVENBERG_MARQUARDT"] {
            for linear_solver in ["DENSE_QR", "DENSE_SCHUR", "SPARSE_NORMAL_CHOLESKY"] {
                let inactive = vec![0., 0., 0., 0., 0., 0., 1.];
                let mut snapshot = Snapshot {
                    values: BTreeMap::from([
                        (
                            1,
                            Values {
                                data: vec![5.; 9],
                                manifold: ManifoldKind::Euclidean,
                                fixed: false,
                            },
                        ),
                        (
                            2,
                            Values {
                                data: inactive.clone(),
                                manifold: ManifoldKind::Pose,
                                fixed: false,
                            },
                        ),
                    ]),
                    factors: [(1, 2., 1.), (2, -1., 2.)]
                        .into_iter()
                        .map(|(id, target, weight)| FactorRecord {
                            id,
                            factor: Factor::SpeedBiasPrior(Box::new(SpeedAndBiasError::new(
                                SVector::repeat(target),
                                SMatrix::identity() * weight,
                            ))),
                            keys: vec![1],
                            loss: None,
                        })
                        .collect(),
                };
                let options = SolverOptions {
                    trust_region,
                    linear_solver,
                    function_tolerance: 1e-12,
                    ..Default::default()
                };
                let first = snapshot.run(&options, &BTreeSet::from([2])).unwrap();
                assert!(first.final_cost.unwrap() < 1e-14);
                assert!(
                    snapshot.values[&1]
                        .data
                        .iter()
                        .all(|x| (x - 2.).abs() < 1e-8)
                );
                let second = snapshot.run(&options, &BTreeSet::from([1])).unwrap();
                assert!((second.initial_cost.unwrap() - 81.).abs() < 1e-6);
                assert!(second.final_cost.unwrap() < 1e-14);
                assert!(
                    snapshot.values[&1]
                        .data
                        .iter()
                        .all(|x| (x + 1.).abs() < 1e-8),
                    "{trust_region}/{linear_solver}: {:?}, {second:?}",
                    snapshot.values[&1].data
                );
                assert_eq!(snapshot.values[&2].data, inactive);
                let before = snapshot.values[&1].data.clone();
                let disabled = snapshot.run(&options, &BTreeSet::from([1, 2])).unwrap();
                assert_eq!(disabled.final_cost, Some(0.));
                assert_eq!(snapshot.values[&1].data, before);
            }
        }
    }

    #[test]
    fn output_preserves_residuals_and_only_copies_requested_jacobians() {
        let residuals = DVector::from_row_slice(&[1.0, -2.0]);
        let pose = SMatrix::<f64, 2, 7>::from_fn(|r, c| (r + 2 * c) as f64);
        let point = SMatrix::<f64, 2, 4>::from_fn(|r, c| (10 + r + 2 * c) as f64);
        let masks = [[false, false], [true, false], [false, true], [true, true]];
        for mask in std::iter::once(None).chain(masks.iter().map(|m| Some(m.as_slice()))) {
            let result = output(residuals.clone(), [pose.as_view(), point.as_view()], mask);
            assert_eq!(result.residuals, residuals);
            assert_eq!(result.minimal_jacobians, vec![None, None]);
            assert_eq!(result.jacobians.len(), 2);
            let expected: [DMatrixView<'_, f64>; 2] = [pose.as_view(), point.as_view()];
            for (i, expected) in expected.into_iter().enumerate() {
                if mask.is_some_and(|m| m[i]) {
                    assert_eq!(
                        result.jacobians[i].as_ref().unwrap(),
                        &expected.into_owned()
                    );
                } else {
                    assert!(result.jacobians[i].is_none());
                }
            }
        }
    }

    #[test]
    fn dense_schur_matches_full_system_with_multiple_landmarks_and_regularization() {
        let mut h =
            DMatrix::from_diagonal(&DVector::from_row_slice(&[2., 3., 4., 5., 6., 7., 9., 10.]));
        for point in 0..6 {
            for retained in 6..8 {
                let v = (point + retained + 1) as f64 / 30.;
                h[(point, retained)] = v;
                h[(retained, point)] = v;
            }
        }
        let gradient = DVector::from_row_slice(&[1., -2., 3., -4., 5., -6., 7., -8.]);
        for regularization in [0., 1e-4, 0.7] {
            let matrix = &h + DMatrix::identity(8, 8) * regularization;
            let lower = matrix.clone();
            let expected = matrix.lu().solve(&(-&gradient)).unwrap();
            for points in [vec![0, 3], vec![]] {
                let model = Linearization {
                    parallel: false,
                    cost: 0.,
                    fixed_cost: 0.,
                    rows: vec![],
                    diagonal: DVector::zeros(8),
                    gradient: gradient.clone(),
                    eliminated_points: points,
                };
                let actual = model.solve_schur(&lower).unwrap();
                assert!((&actual - &expected).amax() < 2e-14);
            }
        }
    }

    #[test]
    fn dense_schur_handles_all_points_and_coupled_landmark_fallback() {
        let model = Linearization {
            parallel: false,
            cost: 0.,
            fixed_cost: 0.,
            rows: vec![],
            gradient: DVector::from_element(6, 2.),
            diagonal: DVector::zeros(6),
            eliminated_points: vec![0, 3],
        };
        let mut lower = DMatrix::identity(6, 6) * 4.;
        assert_eq!(
            model.solve_schur(&lower).unwrap(),
            DVector::from_element(6, -0.5)
        );
        lower[(3, 0)] = 0.1;
        lower[(0, 3)] = 0.1;
        let coupled = model.solve_schur(&lower).unwrap();
        assert!((coupled[0] + 2. / 4.1).abs() < 1e-14);
        assert!((coupled[3] + 2. / 4.1).abs() < 1e-14);
        lower[(3, 0)] = 0.;
        lower[(0, 3)] = 0.;
        lower[(0, 0)] = 0.;
        assert!(model.solve_schur(&lower).is_none());
    }
}
