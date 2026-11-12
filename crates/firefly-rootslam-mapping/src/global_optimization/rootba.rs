//! `RootBA`'s staged QR / PCG, adapted to fixed-calibration rigs and pose clusters.
//! Reference: `rootba/solver/linearizor_qr.cpp` and `qr/impl/landmark_block_base.ipp`.
//! Pose-graph factors enter the same square-root operator (`RootSLAM` eq. 11).
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2023, Nikolaus Demmel.
// Copyright 2015 Google Inc. All rights reserved.
// Rust rig adaptation: Copyright (c) 2026, Firefly contributors.
// License: docs/licenses/crates/firefly-rootslam-mapping/LICENSE
use super::{
    BTreeMap, BTreeSet, BasaltScalar, Calibration, DMatrix, DVector, DampingCoordinates, Error,
    ErrorKind, KrylovPrecision, MapSnapshot, OptimizationOptions, OptimizationResult,
    Preconditioner, Result, SMatrix, SquareRootSystem, State, Vector2, Vector3, Vector4,
    append_graph_factors, calibrated_rig, camera_factor_on_unit_ray, pose_graph, pose_order,
    save_problem, state_from_snapshot, validate_scalar_inputs,
};
use faer::prelude::Solve;
use rayon::prelude::*;

type Matrix6<S> = SMatrix<S, 6, 6>;
const MAX_CLUSTER_POSES: usize = 512;
const CLUSTER_STORAGE_BYTES: usize = 64 * 1024 * 1024;

fn cluster_width(columns: usize) -> usize {
    (CLUSTER_STORAGE_BYTES / std::mem::size_of::<f64>() / columns.max(1) / 6)
        .clamp(1, MAX_CLUSTER_POSES)
        * 6
}

fn apply_clusters<W: BasaltScalar>(
    clusters: &[faer::linalg::solvers::Llt<f64>],
    width: usize,
    residual: &DVector<W>,
    output: &mut DVector<W>,
) {
    for (i, factor) in clusters.iter().enumerate() {
        let offset = i * width;
        let rhs = faer::Mat::from_fn(factor.L().nrows(), 1, |j, _| residual[offset + j].as_f64());
        let solved = factor.solve(rhs.as_ref());
        for j in 0..solved.nrows() {
            output[offset + j] = W::from_f64_lossy(solved[(j, 0)]);
        }
    }
}

fn add_short_normal_rows<S: BasaltScalar, W: BasaltScalar, const ROWS: usize>(
    matrix: nalgebra::DMatrixView<'_, S>,
    columns: &[usize],
    input: &DVector<W>,
    output: &mut DVector<W>,
) {
    let mut values = [W::zero(); ROWS];
    for (j, &global) in columns.iter().enumerate() {
        let column = matrix.column(j);
        let coefficients: &[S; ROWS] = column.as_slice().try_into().expect("fixed row count");
        let value = input[global];
        for i in 0..ROWS {
            values[i] += W::from_f64_lossy(coefficients[i].as_f64()) * value;
        }
    }
    for (j, &global) in columns.iter().enumerate() {
        let column = matrix.column(j);
        let coefficients: &[S; ROWS] = column.as_slice().try_into().expect("fixed row count");
        let mut sum = W::zero();
        for i in 0..ROWS {
            sum += W::from_f64_lossy(coefficients[i].as_f64()) * values[i];
        }
        output[global] += sum;
    }
}

fn add_normal_rows<S: BasaltScalar, W: BasaltScalar>(
    matrix: nalgebra::DMatrixView<'_, S>,
    columns: &[usize],
    input: &DVector<W>,
    output: &mut DVector<W>,
    scratch: &mut Vec<W>,
) {
    match matrix.nrows() {
        1 => return add_short_normal_rows::<S, W, 1>(matrix, columns, input, output),
        3 => return add_short_normal_rows::<S, W, 3>(matrix, columns, input, output),
        5 => return add_short_normal_rows::<S, W, 5>(matrix, columns, input, output),
        _ => {}
    }
    scratch.resize(matrix.nrows(), W::zero());
    scratch.fill(W::zero());
    for (j, &global) in columns.iter().enumerate() {
        for (value, &coefficient) in scratch.iter_mut().zip(matrix.column(j).iter()) {
            *value += W::from_f64_lossy(coefficient.as_f64()) * input[global];
        }
    }
    for (j, &global) in columns.iter().enumerate() {
        let column = matrix.column(j);
        let (coefficients, tail) = column.as_slice().as_chunks::<4>();
        let (values, tail_values) = scratch.as_chunks::<4>();
        let mut sums = [W::zero(); 4];
        for (coefficient, value) in coefficients.iter().zip(values) {
            for i in 0..4 {
                sums[i] += W::from_f64_lossy(coefficient[i].as_f64()) * value[i];
            }
        }
        let mut sum = (sums[0] + sums[1]) + (sums[2] + sums[3]);
        for (&coefficient, &value) in tail.iter().zip(tail_values) {
            sum += W::from_f64_lossy(coefficient.as_f64()) * value;
        }
        output[global] += sum;
    }
}

fn profile_start() -> Option<std::time::Instant> {
    log::log_enabled!(target: "rootslam-global-profile", log::Level::Debug)
        .then(std::time::Instant::now)
}
fn profile_end(start: Option<std::time::Instant>, stage: &str) {
    if let Some(start) = start {
        log::debug!(target: "rootslam-global-profile", "stage={stage} elapsed_ms={:.3}", start.elapsed().as_secs_f64() * 1000.0);
    }
}

struct Block<S: BasaltScalar> {
    id: Option<u64>,
    observations: Vec<crate::map::ObservationKey>,
    columns: Vec<usize>,
    // Landmark columns, incident pose columns, residual; graph blocks omit landmarks.
    matrix: DMatrix<S>,
    landmark_scale: Vector3<S>,
    damping_scale: Vector3<S>,
    rotations: Vec<(usize, usize, S, S)>,
    // Sparse Jp and orthogonal transforms apply Q2ᵀJp without scanning dense blocks.
    pose_rows: Vec<(usize, usize, SMatrix<S, 2, 6>)>,
    reflectors: Vec<(usize, DVector<S>, S)>,
    landmark_rows: DMatrix<S>,
    undamped_rhs: DVector<S>,
    undamped_diagonal: Vec<Matrix6<S>>,
    short_reduced: Option<DMatrix<S>>,
}
impl<S: BasaltScalar> Block<S> {
    fn start(&self) -> usize {
        if self.id.is_some() { 3 } else { 0 }
    }
    #[expect(
        clippy::many_single_char_names,
        reason = "Givens rotation coefficients"
    )]
    fn rotate(&mut self, a: usize, b: usize, c: S, s: S) {
        for mut column in self.matrix.column_iter_mut() {
            let x = column[a];
            let y = column[b];
            column[a] = c * x + s * y;
            column[b] = -s * x + c * y;
        }
    }
    #[expect(
        clippy::many_single_char_names,
        reason = "Givens rotation coefficients"
    )]
    fn damp(&mut self, lambda: S) {
        if self.id.is_none() {
            return;
        }
        while let Some((a, b, c, s)) = self.rotations.pop() {
            self.rotate(a, b, c, -s);
        }
        let rows = self.matrix.nrows();
        self.matrix.rows_mut(rows - 3, 3).fill(S::zero());
        if lambda == S::zero() {
            return;
        }
        for i in 0..3 {
            self.matrix[(rows - 3 + i, i)] = lambda.sqrt() * self.damping_scale[i];
        }
        // A triangular 3x3 landmark factor needs exactly six new rotations.
        for a in 0..3 {
            for m in 0..=a {
                let b = rows - 3 + a - m;
                let x = self.matrix[(a, a)];
                let y = self.matrix[(b, a)];
                let norm = x.hypot(y);
                let (c, s) = if norm == S::zero() {
                    (S::one(), S::zero())
                } else {
                    (x / norm, y / norm)
                };
                self.rotate(a, b, c, s);
                self.rotations.push((a, b, c, s));
            }
        }
    }
    fn qr(&mut self) {
        let rows = self.matrix.nrows() - 3;
        // Householder elimination of only the three landmark columns.
        for k in 0..3.min(rows) {
            let mut v = self.matrix.column(k).rows(k, rows - k).into_owned();
            let norm = v.norm();
            if norm == S::zero() {
                continue;
            }
            let beta = if v[0] >= S::zero() { -norm } else { norm };
            v[0] -= beta;
            let denominator = v.norm_squared();
            for j in k..self.matrix.ncols() {
                let mut whole_column = self.matrix.column_mut(j);
                let mut column = whole_column.rows_mut(k, rows - k);
                let factor = S::from_f64_lossy(2.0) * v.dot(&column) / denominator;
                column.axpy(-factor, &v, S::one());
            }
            self.matrix[(k, k)] = beta;
            self.matrix
                .view_mut((k + 1, k), (rows - k - 1, 1))
                .fill(S::zero());
            self.reflectors
                .push((k, v, S::from_f64_lossy(2.0) / denominator));
        }
    }

    fn implicit_product<W: BasaltScalar>(
        &self,
        input: &DVector<W>,
        output: &mut DVector<W>,
        scratch: &mut Vec<W>,
    ) {
        let observation_rows = self.landmark_rows.nrows();
        scratch.resize(observation_rows + 3, W::zero());
        scratch.fill(W::zero());
        for &(row, local, ref jacobian) in &self.pose_rows {
            let global = self.columns[local];
            let values = &input.as_slice()[global..global + 6];
            let mut sums = [W::zero(); 2];
            for (coefficient, &value) in jacobian.as_slice().as_chunks::<2>().0.iter().zip(values) {
                sums[0] += W::from_f64_lossy(coefficient[0].as_f64()) * value;
                sums[1] += W::from_f64_lossy(coefficient[1].as_f64()) * value;
            }
            scratch[row] = sums[0];
            scratch[row + 1] = sums[1];
        }
        let reflect = |scratch: &mut [W], start: usize, vector: &DVector<S>, tau: S| {
            let slice = &mut scratch[start..start + vector.len()];
            let (values, value_tail) = slice.as_chunks::<4>();
            let (coefficients, coefficient_tail) = vector.as_slice().as_chunks::<4>();
            let mut sums = [W::zero(); 4];
            for (value, coefficient) in values.iter().zip(coefficients) {
                for i in 0..4 {
                    sums[i] += value[i] * W::from_f64_lossy(coefficient[i].as_f64());
                }
            }
            let mut dot = (sums[0] + sums[1]) + (sums[2] + sums[3]);
            for (&value, &coefficient) in value_tail.iter().zip(coefficient_tail) {
                dot += value * W::from_f64_lossy(coefficient.as_f64());
            }
            let factor = dot * W::from_f64_lossy(tau.as_f64());
            for (value, &coefficient) in slice.iter_mut().zip(vector.iter()) {
                *value -= factor * W::from_f64_lossy(coefficient.as_f64());
            }
        };
        let rotate = |scratch: &mut [W], a: usize, b: usize, cosine: S, sine: S| {
            let cosine = W::from_f64_lossy(cosine.as_f64());
            let sine = W::from_f64_lossy(sine.as_f64());
            let x = scratch[a];
            let y = scratch[b];
            scratch[a] = cosine * x + sine * y;
            scratch[b] = -sine * x + cosine * y;
        };
        for (start, vector, tau) in &self.reflectors {
            reflect(scratch, *start, vector, *tau);
        }
        for &(a, b, cosine, sine) in &self.rotations {
            rotate(scratch, a, b + observation_rows - 3, cosine, sine);
        }
        scratch[..3].fill(W::zero());
        for &(a, b, cosine, sine) in self.rotations.iter().rev() {
            rotate(scratch, a, b + observation_rows - 3, cosine, -sine);
        }
        for (start, vector, tau) in self.reflectors.iter().rev() {
            reflect(scratch, *start, vector, *tau);
        }
        for &(row, local, ref jacobian) in &self.pose_rows {
            let global = self.columns[local];
            let values = &mut output.as_mut_slice()[global..global + 6];
            let x = scratch[row];
            let y = scratch[row + 1];
            for (coefficient, value) in jacobian.as_slice().as_chunks::<2>().0.iter().zip(values) {
                *value += W::from_f64_lossy(coefficient[0].as_f64()) * x
                    + W::from_f64_lossy(coefficient[1].as_f64()) * y;
            }
        }
    }

    fn compress(&mut self) {
        let rows = self.matrix.nrows() - 3;
        self.landmark_rows = self.matrix.rows(0, rows).into_owned();
        self.qr();
        let mut factor = DMatrix::zeros(6, 3 + self.columns.len() + 1);
        factor
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&self.matrix.fixed_view::<3, 3>(0, 0));
        factor
            .column_mut(3 + self.columns.len())
            .fixed_rows_mut::<3>(0)
            .copy_from(&self.matrix.column(3).fixed_rows::<3>(0));
        self.undamped_rhs = DVector::zeros(self.columns.len());
        self.undamped_diagonal = Vec::with_capacity(self.columns.len() / 6);
        // For at most two incident poses, explicit multiplication is cheaper
        // than applying six reflectors and still has linear observation storage.
        self.short_reduced =
            (self.columns.len() <= 12).then(|| DMatrix::zeros(rows - 3, self.columns.len()));
        let mut projected = DMatrix::<S>::zeros(rows, 6);
        for local in (0..self.columns.len()).step_by(6) {
            projected.fill(S::zero());
            for &(row, incident, ref jacobian) in &self.pose_rows {
                if incident == local {
                    projected.fixed_view_mut::<2, 6>(row, 0).copy_from(jacobian);
                }
            }
            for (start, vector, tau) in &self.reflectors {
                for j in 0..6 {
                    let mut column = projected.column_mut(j);
                    let mut tail = column.rows_mut(*start, vector.len());
                    let coefficient = *tau * vector.dot(&tail);
                    tail.axpy(-coefficient, vector, S::one());
                }
            }
            factor
                .fixed_view_mut::<3, 6>(0, 3 + local)
                .copy_from(&projected.fixed_rows::<3>(0));
            let reduced = projected.rows(3, rows - 3);
            if let Some(cache) = &mut self.short_reduced {
                cache.columns_mut(local, 6).copy_from(&reduced);
            }
            let residual = self.matrix.column(3);
            let gradient = reduced.transpose() * residual.rows(3, rows - 3);
            self.undamped_rhs.rows_mut(local, 6).copy_from(&(-gradient));
            let diagonal = reduced.transpose() * reduced;
            self.undamped_diagonal
                .push(Matrix6::from_column_slice(diagonal.as_slice()));
        }
        self.matrix = factor;
    }
}

struct System<S: BasaltScalar> {
    blocks: Vec<Block<S>>,
    scaling: DVector<S>,
    damping_scale: DVector<S>,
    lambda: S,
    cost: S,
    preconditioner: Option<Vec<faer::linalg::solvers::Llt<f64>>>,
}

fn machine_epsilon<S: BasaltScalar>() -> S {
    S::from_f64_lossy(if std::mem::size_of::<S>() == 4 {
        f64::from(f32::EPSILON)
    } else {
        f64::EPSILON
    })
}

fn column_scale<S: BasaltScalar>(norm: S) -> S {
    // RootBA LinearizorBase::get_effective_jacobi_scaling_epsilon uses
    // RootBA's default column regularizer uses Sophus epsilonSqrt.
    S::one() / (S::epsilon_sqrt() + norm)
}

#[expect(
    clippy::too_many_lines,
    reason = "Staged landmark and graph linearization"
)]
fn linearize<S: BasaltScalar>(
    snapshot: &MapSnapshot,
    state: &State<S>,
    calib: &Calibration<S>,
    order: &BTreeMap<i64, usize>,
    options: &OptimizationOptions,
) -> Result<System<S>> {
    let started = profile_start();
    let extrinsics: Vec<_> = calib.t_i_c.iter().map(pose_graph::inverse).collect();
    let entries: Vec<_> = snapshot
        .landmarks
        .iter()
        .filter(|(id, _)| state.points.contains_key(id))
        .collect();
    let mut blocks: Vec<_> = entries
        .par_iter()
        .map(|&(id, landmark)| -> Result<Option<Block<S>>> {
            let id = *id;
            let mut factors = Vec::new();
            let mut incident = BTreeSet::new();
            for (key, observation) in &landmark.observations {
                let Some(pose) = state.poses.get(&key.keyframe) else {
                    continue;
                };
                let camera = calib.intrinsics.get(key.camera).ok_or_else(|| {
                    Error::new(
                        ErrorKind::InvalidArgument,
                        "observation camera outside calibration",
                    )
                })?;
                if let Some(factor) = camera_factor_on_unit_ray(
                    pose,
                    &state.points[&id],
                    &extrinsics[key.camera],
                    camera,
                    &observation.pixel.map(S::from_f64_lossy),
                ) {
                    factors.push((*key, factor));
                    if order.contains_key(&key.keyframe) {
                        incident.insert(key.keyframe);
                    }
                } else {
                    return Err(Error::new(
                        ErrorKind::InvalidArgument,
                        "invalid global camera factor",
                    )
                    .with_operation("rootslam.global.linearize")
                    .with_context("landmark", id)
                    .with_context("keyframe", key.keyframe)
                    .with_context("camera", key.camera));
                }
            }
            if factors.is_empty() {
                return Ok(None);
            }
            let local: BTreeMap<_, _> = incident
                .into_iter()
                .enumerate()
                .map(|(i, t)| (t, i * 6))
                .collect();
            let columns: Vec<_> = local.keys().flat_map(|t| order[t]..order[t] + 6).collect();
            // Eligible landmarks have at least four residual rows; the final
            // three rows are reserved for reversible landmark damping.
            let mut matrix = DMatrix::zeros(2 * factors.len() + 3, 4);
            let observations = factors.iter().map(|(key, _)| *key).collect();
            let mut pose_rows = Vec::with_capacity(factors.len());
            let residual = matrix.ncols() - 1;
            for (i, (key, (r, jp, jl))) in factors.into_iter().enumerate() {
                matrix.view_mut((2 * i, 0), (2, 3)).copy_from(&jl);
                matrix.view_mut((2 * i, residual), (2, 1)).copy_from(&r);
                if let Some(&offset) = local.get(&key.keyframe) {
                    pose_rows.push((2 * i, offset, jp));
                }
            }
            Ok(Some(Block {
                id: Some(id),
                observations,
                columns,
                matrix,
                landmark_scale: Vector3::repeat(S::one()),
                damping_scale: Vector3::repeat(S::one()),
                rotations: Vec::with_capacity(6),
                pose_rows,
                reflectors: Vec::with_capacity(3),
                landmark_rows: DMatrix::zeros(0, 4),
                undamped_rhs: DVector::zeros(0),
                undamped_diagonal: vec![],
                short_reduced: None,
            }))
        })
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .flatten()
        .collect();
    if options.pose_graph_terms {
        let mut graph = SquareRootSystem {
            columns: order.len() * 6,
            blocks: vec![],
            landmarks: vec![],
            damping: S::zero(),
        };
        append_graph_factors(snapshot, state, order, &mut graph);
        for block in graph.blocks {
            let mut matrix = DMatrix::zeros(6, block.columns.len() + 1);
            matrix
                .columns_mut(0, block.columns.len())
                .copy_from(&block.matrix);
            matrix
                .column_mut(block.columns.len())
                .copy_from(&block.residual);
            blocks.push(Block {
                id: None,
                observations: vec![],
                columns: block.columns,
                matrix,
                landmark_scale: Vector3::zeros(),
                damping_scale: Vector3::zeros(),
                rotations: vec![],
                pose_rows: vec![],
                reflectors: vec![],
                landmark_rows: DMatrix::zeros(0, 4),
                undamped_rhs: DVector::zeros(0),
                undamped_diagonal: vec![],
                short_reduced: None,
            });
        }
    }
    let columns = order.len() * 6;
    let (norms, cost) = blocks
        .par_iter()
        .fold(
            || (DVector::<S>::zeros(columns), 0.0),
            |(mut norms, mut cost), block| {
                if block.id.is_some() {
                    for (_, local, jacobian) in &block.pose_rows {
                        for column in 0..6 {
                            norms[block.columns[*local + column]] +=
                                jacobian.column(column).norm_squared();
                        }
                    }
                } else {
                    for (local, &global) in block.columns.iter().enumerate() {
                        norms[global] += block.matrix.column(local).norm_squared();
                    }
                }
                cost += block
                    .matrix
                    .column(block.matrix.ncols() - 1)
                    .iter()
                    .map(|r| r.as_f64().powi(2))
                    .sum::<f64>()
                    * 0.5;
                (norms, cost)
            },
        )
        .reduce(
            || (DVector::zeros(columns), 0.0),
            |(a, x), (b, y)| (a + b, x + y),
        );
    let scaling = norms.map(|x| column_scale(x.sqrt()));
    blocks.par_iter_mut().for_each(|block| {
        if block.id.is_some() {
            for j in 0..3 {
                block.landmark_scale[j] = column_scale(block.matrix.column(j).norm());
                block
                    .matrix
                    .column_mut(j)
                    .scale_mut(block.landmark_scale[j]);
            }
        } else {
            for (local, &global) in block.columns.iter().enumerate() {
                block.matrix.column_mut(local).scale_mut(scaling[global]);
            }
        }
        for (_, local, jacobian) in &mut block.pose_rows {
            for column in 0..6 {
                jacobian
                    .column_mut(column)
                    .scale_mut(scaling[block.columns[*local + column]]);
            }
        }
        if block.id.is_some() {
            if options.damping_coordinates == DampingCoordinates::Physical {
                block.damping_scale = block.landmark_scale;
            }
            block.compress();
        }
    });
    let cost = S::from_f64_lossy(cost);
    if !cost.is_finite()
        || blocks
            .iter()
            .any(|b| !b.matrix.iter().all(nalgebra::ComplexField::is_finite))
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "nonfinite scaled global linearization",
        ));
    }
    profile_end(started, "linearize");
    Ok(System {
        blocks,
        damping_scale: if options.damping_coordinates == DampingCoordinates::Physical {
            scaling.clone()
        } else {
            DVector::repeat(columns, S::one())
        },
        scaling,
        lambda: S::zero(),
        cost,
        preconditioner: None,
    })
}

impl<S: BasaltScalar> System<S> {
    fn storage_bytes(&self) -> usize {
        let blocks: usize = self
            .blocks
            .iter()
            .map(|b| {
                (b.matrix.len()
                    + b.short_reduced.as_ref().map_or(0, DMatrix::len)
                    + b.landmark_rows.len()
                    + 12 * b.pose_rows.len()
                    + b.reflectors.iter().map(|(_, v, _)| v.len()).sum::<usize>()
                    + b.undamped_rhs.len()
                    + 36 * b.undamped_diagonal.len())
                    * std::mem::size_of::<S>()
            })
            .sum();
        blocks
            + self.preconditioner.as_ref().map_or(0, |clusters| {
                clusters
                    .iter()
                    .map(|factor| {
                        factor.L().nrows() * factor.L().ncols() * std::mem::size_of::<f64>()
                    })
                    .sum::<usize>()
            })
    }
    fn damp(&mut self, lambda: S) {
        let started = profile_start();
        self.lambda = lambda;
        self.blocks
            .par_iter_mut()
            .for_each(|block| block.damp(lambda));
        profile_end(started, "damping");
    }
    fn normal_product<W: BasaltScalar>(&self, input: &DVector<W>) -> DVector<W> {
        let columns = self.scaling.len();
        let mut output = self
            .blocks
            .par_iter()
            // Each reduction owns a full pose vector. Bound task subdivision
            // while retaining several tasks per worker for uneven track lengths.
            .with_min_len(self.blocks.len().div_ceil(4 * rayon::current_num_threads()))
            .fold(
                || (DVector::<W>::zeros(columns), Vec::new()),
                |(mut output, mut scratch), block| {
                    if block.id.is_some() && block.short_reduced.is_none() {
                        block.implicit_product(input, &mut output, &mut scratch);
                        return (output, scratch);
                    }
                    if let Some(reduced) = &block.short_reduced {
                        add_normal_rows(
                            reduced.as_view(),
                            &block.columns,
                            input,
                            &mut output,
                            &mut scratch,
                        );
                    }
                    let start = block.start();
                    let a = block.matrix.view(
                        (start, start),
                        (block.matrix.nrows() - start, block.columns.len()),
                    );
                    add_normal_rows(a, &block.columns, input, &mut output, &mut scratch);
                    (output, scratch)
                },
            )
            .map(|(output, _)| output)
            .reduce(|| DVector::zeros(columns), |a, b| a + b);
        for i in 0..columns {
            output[i] += W::from_f64_lossy((self.lambda * self.damping_scale[i].powi(2)).as_f64())
                * input[i];
        }
        output
    }
    fn prepare<W: BasaltScalar>(&self) -> (DVector<W>, Vec<Matrix6<W>>) {
        let columns = self.scaling.len();
        let (rhs, diagonal) = self
            .blocks
            .par_iter()
            .with_min_len(self.blocks.len().div_ceil(4 * rayon::current_num_threads()))
            .fold(
                || {
                    (
                        DVector::<W>::zeros(columns),
                        vec![Matrix6::<W>::zeros(); columns / 6],
                    )
                },
                |(mut rhs, mut diagonal), block| {
                    let start = block.start();
                    let rows = block.matrix.nrows() - start;
                    let residual = block
                        .matrix
                        .view((start, block.matrix.ncols() - 1), (rows, 1));
                    for (i, indices) in block.columns.as_chunks::<6>().0.iter().enumerate() {
                        let a = block.matrix.view((start, start + 6 * i), (rows, 6));
                        let mut b = a.transpose() * residual;
                        let mut h = a.transpose() * a;
                        if block.id.is_some() {
                            b -= block.undamped_rhs.rows(6 * i, 6);
                            h += block.undamped_diagonal[i];
                        }
                        for j in 0..6 {
                            rhs[indices[j]] -= W::from_f64_lossy(b[j].as_f64());
                            for k in 0..6 {
                                diagonal[indices[0] / 6][(j, k)] +=
                                    W::from_f64_lossy(h[(j, k)].as_f64());
                            }
                        }
                    }
                    (rhs, diagonal)
                },
            )
            .reduce(
                || (DVector::zeros(columns), vec![Matrix6::zeros(); columns / 6]),
                |(a, mut x), (b, y)| {
                    for (x, y) in x.iter_mut().zip(y) {
                        *x += y;
                    }
                    (a + b, x)
                },
            );
        let mut diagonal = diagonal;
        for (i, block) in diagonal.iter_mut().enumerate() {
            for j in 0..6 {
                block[(j, j)] += W::from_f64_lossy(
                    (self.lambda * self.damping_scale[6 * i + j].powi(2)).as_f64(),
                );
            }
        }
        (rhs, diagonal)
    }
    fn cluster_preconditioner<W: BasaltScalar>(
        &self,
        diagonal: &[Matrix6<W>],
        width: usize,
    ) -> Option<Vec<faer::linalg::solvers::Llt<f64>>> {
        // Off-diagonal pose blocks are -B_i^T B_j, where B is the first three
        // rows after damped landmark QR: distinct poses share no original row.
        // This approximate Hessian is used only as an SPD preconditioner.
        // The operator, residual certificate and LM acceptance retain QR arithmetic.
        let zero = || {
            (0..self.scaling.len().div_ceil(width))
                .map(|i| {
                    DMatrix::<f64>::zeros(
                        width.min(self.scaling.len() - width * i),
                        width.min(self.scaling.len() - width * i),
                    )
                })
                .collect::<Vec<_>>()
        };
        let mut clusters = self
            .blocks
            .par_iter()
            // Four partial reductions bound construction storage to four times
            // the cluster budget, independently of the Rayon worker count.
            .with_min_len(self.blocks.len().div_ceil(4))
            .fold(zero, |mut clusters, block| {
                let rows = if block.id.is_some() {
                    3
                } else {
                    block.matrix.nrows()
                };
                let sign = if block.id.is_some() { -1.0 } else { 1.0 };
                let top = block
                    .matrix
                    .view((0, block.start()), (rows, block.columns.len()))
                    .map(BasaltScalar::as_f64);
                for i in (0..block.columns.len()).step_by(6) {
                    for j in (0..i).step_by(6).rev() {
                        let a = block.columns[i];
                        let b = block.columns[j];
                        if a / width != b / width {
                            break;
                        }
                        let cross = if block.id.is_some() {
                            top.fixed_view::<3, 6>(0, i).transpose() * top.fixed_view::<3, 6>(0, j)
                        } else {
                            Matrix6::from_column_slice(
                                (top.columns(i, 6).transpose() * top.columns(j, 6)).as_slice(),
                            )
                        };
                        for r in 0..6 {
                            for c in 0..6 {
                                clusters[a / width][(a % width + r, b % width + c)] +=
                                    sign * cross[(r, c)];
                                clusters[a / width][(b % width + c, a % width + r)] +=
                                    sign * cross[(r, c)];
                            }
                        }
                    }
                }
                clusters
            })
            .reduce_with(|mut left, right| {
                for (a, b) in left.iter_mut().zip(right) {
                    *a += b;
                }
                left
            })
            .unwrap_or_else(zero);
        for (i, block) in diagonal.iter().enumerate() {
            clusters[6 * i / width]
                .fixed_view_mut::<6, 6>(6 * i % width, 6 * i % width)
                .copy_from(&block.map(BasaltScalar::as_f64));
        }
        clusters
            .into_par_iter()
            .map(|mut block| {
                if !block.iter().all(|value| value.is_finite()) {
                    return None;
                }
                // A roundoff-scale ridge protects the approximate cross terms.
                // It belongs only to the preconditioner, not the solved system.
                let ridge = block.diagonal().amax() * 8.0 * machine_epsilon::<S>().as_f64();
                for i in 0..block.nrows() {
                    block[(i, i)] += ridge;
                }
                faer::Mat::from_fn(block.nrows(), block.ncols(), |i, j| block[(i, j)])
                    .llt(faer::Side::Lower)
                    .ok()
            })
            .collect()
    }
    #[expect(
        clippy::many_single_char_names,
        reason = "Standard PCG recurrence notation"
    )]
    fn solve<W: BasaltScalar>(
        &mut self,
        options: &OptimizationOptions,
    ) -> (DVector<S>, usize, bool) {
        let prepared = profile_start();
        let mut x = DVector::<W>::zeros(self.scaling.len());
        let (b, diagonal) = self.prepare::<W>();
        if b.norm_squared() == W::zero() {
            profile_end(prepared, "preconditioner");
            return (x.map(|v| S::from_f64_lossy(v.as_f64())), 0, true);
        }
        let width = cluster_width(self.scaling.len());
        if options.preconditioner == Preconditioner::ClusterJacobi
            && self.preconditioner.is_none()
            && self.scaling.len() >= 96
        {
            // Every accepted step constructs a new System. Rejected LM steps
            // reuse this SPD factor while testing the current damped QR operator.
            self.preconditioner = self.cluster_preconditioner(&diagonal, width);
        }
        let clusters = self
            .preconditioner
            .as_ref()
            .filter(|_| options.preconditioner == Preconditioner::ClusterJacobi);
        let inverse: Option<Vec<_>> = if clusters.is_some() {
            Some(Vec::new())
        } else {
            diagonal
                .into_iter()
                .map(|block| block.cholesky().map(|factor| factor.inverse()))
                .collect()
        };
        let Some(inverse) = inverse else {
            return (x.map(|v| S::from_f64_lossy(v.as_f64())), 0, false);
        };
        profile_end(prepared, "preconditioner");
        let mut r = b.clone();
        let physical_norm = |residual: &DVector<W>| {
            residual
                .iter()
                .zip(self.scaling.iter())
                .map(|(value, scale)| (value.as_f64() / scale.as_f64()).powi(2))
                .sum::<f64>()
                .sqrt()
        };
        let initial_gradient = physical_norm(&b);
        let gradient_tolerance = options
            .linear_solver_gradient_tolerance
            .max(machine_epsilon::<W>().as_f64().sqrt());
        let mut p = x.clone();
        // RootBA/Ceres keeps recurrence coefficients in double precision even
        // when matrix products and work vectors use float storage.
        let mut rho_previous = 1.0_f64;
        let mut model_previous = 0.0;
        for iteration in 1..=options.linear_solver_max_iterations {
            let mut z = DVector::<W>::zeros(x.len());
            if let Some(clusters) = clusters {
                apply_clusters(clusters, width, &r, &mut z);
            } else {
                for (i, block) in inverse.iter().enumerate() {
                    z.fixed_rows_mut::<6>(6 * i)
                        .copy_from(&(block * r.fixed_rows::<6>(6 * i)));
                }
            }
            let rho = r.dot(&z).as_f64();
            if !rho.is_finite() || rho <= 0.0 {
                return (x.map(|v| S::from_f64_lossy(v.as_f64())), iteration, false);
            }
            if iteration == 1 {
                p = z;
            } else {
                p = z + p * W::from_f64_lossy(rho / rho_previous);
            }
            let q = self.normal_product(&p);
            let pq = p.dot(&q).as_f64();
            if !pq.is_finite() || pq <= 0.0 {
                return (x.map(|v| S::from_f64_lossy(v.as_f64())), iteration, false);
            }
            let alpha = W::from_f64_lossy(rho / pq);
            x.axpy(alpha, &p, W::one());
            if iteration % 10 == 0 {
                r = &b - self.normal_product(&x);
            } else {
                r.axpy(-alpha, &q, W::one());
            }
            let model = -x.dot(&(&b + &r)).as_f64();
            let progress = iteration as f64 * (model - model_previous) / model;
            if r.norm_squared() == W::zero()
                || (model.is_finite() && model < 0.0 && progress < options.linear_solver_eta)
            {
                // A small truncated-model improvement can coexist with a large
                // physical gradient on highly unequal rig/landmark scales.
                // Certify the requested rig accuracy against a fresh residual.
                if options.linear_solver_gradient_tolerance == 0.0
                    || (physical_norm(&r) <= gradient_tolerance * initial_gradient
                        && physical_norm(&(&b - self.normal_product(&x)))
                            <= gradient_tolerance * initial_gradient)
                {
                    return (x.map(|v| S::from_f64_lossy(v.as_f64())), iteration, true);
                }
            }
            model_previous = model;
            rho_previous = rho;
        }
        // RootBA passes finite truncated steps to LM even at the PCG iteration limit.
        let finite = x.iter().all(nalgebra::ComplexField::is_finite);
        (
            x.map(|v| S::from_f64_lossy(v.as_f64())),
            options.linear_solver_max_iterations,
            finite,
        )
    }
    fn apply(
        &mut self,
        state: &State<S>,
        order: &BTreeMap<i64, usize>,
        step: &DVector<S>,
    ) -> (State<S>, S) {
        let started = profile_start();
        let mut next = state.clone();
        for (&time, &offset) in order {
            let increment = step
                .fixed_rows::<6>(offset)
                .component_mul(&self.scaling.fixed_rows::<6>(offset));
            next.poses.insert(
                time,
                pose_graph::product(&pose_graph::se3_exp(&increment), &state.poses[&time]),
            );
        }
        let updates: Vec<_> = self
            .blocks
            .par_iter_mut()
            .map(|block| {
                let start = block.start();
                let mut local = DVector::<S>::zeros(start + block.columns.len());
                for (i, &global) in block.columns.iter().enumerate() {
                    local[start + i] = step[global];
                }
                if block.id.is_some() {
                    for i in (0..3).rev() {
                        let mut rhs = -block.matrix[(i, block.matrix.ncols() - 1)];
                        for j in i + 1..local.len() {
                            rhs -= block.matrix[(i, j)] * local[j];
                        }
                        local[i] = rhs / block.matrix[(i, i)];
                    }
                    block.damp(S::zero());
                }
                let prediction = if block.id.is_some() {
                    let mut change = block.landmark_rows.columns(0, 3) * local.fixed_rows::<3>(0);
                    for &(row, incident, ref jacobian) in &block.pose_rows {
                        let value = jacobian * local.fixed_rows::<6>(3 + incident);
                        change[row] += value[0];
                        change[row + 1] += value[1];
                    }
                    -change.dot(&(block.landmark_rows.column(3) + &change * S::from_f64_lossy(0.5)))
                } else {
                    let change = block.matrix.columns(0, local.len()) * &local;
                    -change.dot(
                        &(block.matrix.column(block.matrix.ncols() - 1)
                            + &change * S::from_f64_lossy(0.5)),
                    )
                };
                let point = block.id.map(|id| {
                    (
                        id,
                        state.points[&id]
                            + local
                                .fixed_rows::<3>(0)
                                .component_mul(&block.landmark_scale),
                    )
                });
                (point, prediction)
            })
            .collect();
        let mut prediction = S::zero();
        for (point, value) in updates {
            prediction += value;
            if let Some((id, point)) = point {
                next.points.insert(id, point);
            }
        }
        profile_end(started, "back_substitution");
        (next, prediction)
    }
    fn trial_cost(&self, snapshot: &MapSnapshot, state: &State<S>, calib: &Calibration<S>) -> S {
        let extrinsics: Vec<_> = calib.t_i_c.iter().map(pose_graph::inverse).collect();
        // Each trial has one camera transform per pose, shared by all landmarks.
        let camera_poses: BTreeMap<_, Vec<_>> = state
            .poses
            .iter()
            .map(|(&time, pose)| {
                (
                    time,
                    extrinsics
                        .iter()
                        .map(|extrinsic| pose_graph::product(extrinsic, pose))
                        .collect(),
                )
            })
            .collect();
        let visual: f64 = self
            .blocks
            .par_iter()
            .filter_map(|block| block.id.map(|id| (block, id)))
            .map(|(block, id)| {
                let mut cost = 0.0;
                let point = &state.points[&id];
                let observations = &snapshot.landmarks[&id].observations;
                for key in &block.observations {
                    let transform = &camera_poses[&key.keyframe][key.camera];
                    let p = transform.rotation * point + transform.translation.vector;
                    let p = p / p.x.hypot(p.y).hypot(p.z);
                    let mut pixel = Vector2::zeros();
                    // Generalized rig cameras can have finite projections on a
                    // second, non-injective branch. Preserve the camera domain
                    // and the complete observation set when accepting an LM step.
                    let valid = calib.intrinsics[key.camera].project(
                        &Vector4::new(p.x, p.y, p.z, S::one()),
                        &mut pixel,
                        None,
                    );
                    if !valid || !pixel.iter().all(nalgebra::ComplexField::is_finite) {
                        return f64::INFINITY;
                    }
                    cost += (pixel - observations[key].pixel.map(S::from_f64_lossy))
                        .iter()
                        .map(|v| v.as_f64().powi(2))
                        .sum::<f64>();
                }
                cost * 0.5
            })
            .sum();
        let graph = if self.blocks.iter().any(|b| b.id.is_none()) {
            snapshot
                .edges
                .iter()
                .filter(|e| e.kind.is_measurement_bearing())
                .filter_map(|edge| {
                    let (a, b) = (state.poses.get(&edge.from)?, state.poses.get(&edge.to)?);
                    let prediction = pose_graph::product(a, &pose_graph::inverse(b));
                    let error = pose_graph::se3_log(&pose_graph::product(
                        &pose_graph::cast_pose(&edge.measurement),
                        &pose_graph::inverse(&prediction),
                    ));
                    Some(
                        (edge.sqrt_information.map(S::from_f64_lossy) * error)
                            .iter()
                            .map(|v| v.as_f64().powi(2))
                            .sum::<f64>()
                            * 0.5,
                    )
                })
                .sum()
        } else {
            0.0
        };
        S::from_f64_lossy(visual + graph)
    }
}

#[expect(
    clippy::too_many_lines,
    reason = "LM acceptance and damping state machine"
)]
pub(super) fn optimize<S: BasaltScalar>(
    snapshot: &MapSnapshot,
    calibration: &Calibration<f64>,
    options: &OptimizationOptions,
) -> Result<OptimizationResult> {
    options.validate()?;
    if options.linear_system_archive.is_some() {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "linear_system_archive is only supported by lsqr_reference; use problem_archive for RootBA replay",
        ));
    }
    validate_scalar_inputs::<S>(snapshot, options)?;
    let calib = calibrated_rig::<S>(calibration)?;
    let mut state = state_from_snapshot(snapshot)?;
    save_problem(snapshot, calibration, options)?;
    for landmark in snapshot.landmarks.values() {
        for key in landmark.observations.keys() {
            if state.poses.contains_key(&key.keyframe) && key.camera >= calib.intrinsics.len() {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    "observation camera outside calibration",
                ));
            }
        }
    }
    // RootBA's landmark blocks require at least two camera observations.
    // Depth requires distinct camera views. Deferred points stay in the map
    // and follow their host correction through PersistentMap::apply_updates.
    state.points.retain(|id, _| {
        let mut observations = snapshot.landmarks[id]
            .observations
            .keys()
            .filter(|key| state.poses.contains_key(&key.keyframe));
        observations.next().is_some_and(|first| {
            observations.any(|key| (key.keyframe, key.camera) != (first.keyframe, first.camera))
        })
    });
    log::info!(
        "Global BA landmarks: optimized={} deferred_underconstrained={}",
        state.points.len(),
        snapshot.landmarks.len() - state.points.len()
    );
    let order = pose_order(snapshot);
    let mut system = linearize(snapshot, &state, &calib, &order, options)?;
    let initial = system.cost;
    let mut cost = initial;
    let mut lambda = S::from_f64_lossy(options.initial_damping);
    let mut multiplier = S::from_f64_lossy(2.0);
    let (mut accepted, mut iterations, mut linear_iterations, mut peak) = (0, 0, 0, 0);
    let mut converged = false;
    let mut changed = false;
    for iteration in 0..options.max_iterations {
        iterations = iteration + 1;
        if changed {
            system = linearize(snapshot, &state, &calib, &order, options)?;
            changed = false;
        }
        cost = system.cost;
        system.damp(lambda);
        let started = profile_start();
        let (step, count, valid) = match options.krylov_precision {
            KrylovPrecision::Native => system.solve::<S>(options),
            KrylovPrecision::Float64 => system.solve::<f64>(options),
        };
        profile_end(started, "linear_solve_including_preconditioner");
        peak = peak.max(system.storage_bytes());
        linear_iterations += count;
        if valid {
            let (next, prediction) = system.apply(&state, &order, &step);
            let started = profile_start();
            let next_cost = system.trial_cost(snapshot, &next, &calib);
            profile_end(started, "trial_cost");
            let decrease = cost - next_cost;
            let quality = decrease / prediction;
            let norm = next.points.iter().fold(
                step.component_mul(&system.scaling).amax(),
                |norm, (id, point)| norm.max((*point - state.points[id]).amax()),
            );
            log::info!(
                "Global BA iteration={iteration} cost={} trial={} damping={} linear_iterations={count} quality={}",
                cost.as_f64(),
                next_cost.as_f64(),
                lambda.as_f64(),
                quality.as_f64()
            );
            if next_cost.is_finite() && prediction > S::zero() && quality > S::zero() {
                state = next;
                cost = next_cost;
                accepted += 1;
                changed = true;
                lambda = (lambda
                    * (S::one() / S::from_f64_lossy(3.0))
                        .max(S::one() - (S::from_f64_lossy(2.0) * quality - S::one()).powi(3)))
                .max(S::from_f64_lossy(options.minimum_damping));
                multiplier = S::from_f64_lossy(2.0);
                if norm < S::from_f64_lossy(options.step_tolerance)
                    || decrease
                        <= S::from_f64_lossy(options.relative_cost_tolerance)
                            * cost.abs().max(S::one())
                {
                    converged = true;
                    break;
                }
                continue;
            }
            if prediction.abs() < S::from_f64_lossy(options.relative_cost_tolerance)
                && norm < S::from_f64_lossy(options.step_tolerance)
                && next_cost.is_finite()
            {
                converged = true;
                break;
            }
        }
        lambda *= multiplier;
        multiplier *= S::from_f64_lossy(2.0);
        if lambda > S::from_f64_lossy(options.maximum_damping) {
            break;
        }
    }
    Ok(OptimizationResult {
        revision: snapshot.revision,
        poses: state
            .poses
            .into_iter()
            .map(|(id, p)| (id, pose_graph::cast_pose(&pose_graph::inverse(&p))))
            .collect(),
        points: state
            .points
            .into_iter()
            .map(|(id, p)| (id, p.map(BasaltScalar::as_f64)))
            .collect(),
        initial_cost: initial.as_f64(),
        final_cost: cost.as_f64(),
        iterations,
        accepted_steps: accepted,
        converged,
        linear_iterations,
        peak_linear_storage_bytes: peak,
        precision: options.precision,
        krylov_precision: options.krylov_precision,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kb4_global_projection_is_independent_of_point_distance_and_precision() {
        use firefly_calibration::camera::{
            generic_camera::GenericCamera, kannala_brandt_camera4::KannalaBrandtCamera4,
        };
        use nalgebra::{Isometry3, SVector};
        let parameters = SVector::<f64, 8>::from([
            278.667_230_661_490_86,
            278.489_914_097_402_96,
            319.752_212_005_935_35,
            241.968_589_103_581_73,
            -0.013_721_808_247_486_035,
            0.020_727_425_669_427_896,
            -0.012_786_476_702_685_545,
            0.002_524_226_732_068_762_5,
        ]);
        let camera = GenericCamera::KannalaBrandt4(KannalaBrandtCamera4::new(parameters));
        let camera_float = camera.cast::<f32>();
        let point = Vector3::new(-0.001_055_62, -0.000_704_1, 0.000_301_71);
        let reference = |point: Vector3<f64>| {
            let radius = point.x.hypot(point.y);
            let theta = radius.atan2(point.z);
            let distorted = theta
                + (0..4)
                    .map(|i| parameters[4 + i] * theta.powi(3 + 2 * i as i32))
                    .sum::<f64>();
            Vector2::new(
                parameters[0] * point.x / radius * distorted + parameters[2],
                parameters[1] * point.y / radius * distorted + parameters[3],
            )
        };
        assert!(
            super::super::camera_factor(
                &Isometry3::identity(),
                &point.cast::<f32>(),
                &Isometry3::identity(),
                &camera_float,
                &Vector2::zeros()
            )
            .is_none()
        );
        for distance_scale in [1e-6, 1.0, 1e6] {
            let point = point * distance_scale;
            let (pixel, _, jacobian) = camera_factor_on_unit_ray(
                &Isometry3::identity(),
                &point,
                &Isometry3::identity(),
                &camera,
                &Vector2::zeros(),
            )
            .unwrap();
            let (pixel_float, _, _) = camera_factor_on_unit_ray(
                &Isometry3::identity(),
                &point.cast::<f32>(),
                &Isometry3::identity(),
                &camera_float,
                &Vector2::zeros(),
            )
            .unwrap();
            assert!((pixel - reference(point)).amax() < 1e-10);
            assert!((pixel_float.cast::<f64>() - reference(point)).amax() < 2e-4);
            let epsilon = point.norm() * 1e-6;
            for axis in 0..3 {
                let mut delta = Vector3::zeros();
                delta[axis] = epsilon;
                let numerical =
                    (reference(point + delta) - reference(point - delta)) / (2.0 * epsilon);
                assert!(
                    (numerical - jacobian.column(axis)).norm() / numerical.norm().max(1.0) < 1e-7
                );
            }
        }
    }

    fn fixture(rows: usize) -> Block<f64> {
        let matrix = DMatrix::from_fn(rows.max(3) + 3, 10, |i, j| {
            if i >= rows {
                0.0
            } else {
                ((i * 11 + j * 7 + 3) as f64).sin() + if i == j { 2.0 } else { 0.0 }
            }
        });
        let pose_rows = (0..rows / 2)
            .map(|i| (2 * i, 0, matrix.fixed_view::<2, 6>(2 * i, 3).into_owned()))
            .collect();
        let mut compact = DMatrix::zeros(matrix.nrows(), 4);
        compact.columns_mut(0, 3).copy_from(&matrix.columns(0, 3));
        compact.column_mut(3).copy_from(&matrix.column(9));
        Block {
            id: Some(1),
            observations: vec![],
            columns: (0..6).collect(),
            matrix: compact,
            landmark_scale: Vector3::repeat(1.0),
            damping_scale: Vector3::repeat(1.0),
            rotations: vec![],
            pose_rows,
            reflectors: vec![],
            landmark_rows: DMatrix::zeros(0, 4),
            undamped_rhs: DVector::zeros(0),
            undamped_diagonal: vec![],
            short_reduced: None,
        }
    }

    #[test]
    fn staged_qr_pcg_matches_independent_full_damped_svd() {
        for rows in [2, 16] {
            let mut block = fixture(rows);
            let mut full = DMatrix::zeros(rows + 9, 9);
            full.view_mut((0, 0), (rows, 3))
                .copy_from(&block.matrix.view((0, 0), (rows, 3)));
            for &(row, _, ref jacobian) in &block.pose_rows {
                full.fixed_view_mut::<2, 6>(row, 3).copy_from(jacobian);
            }
            for i in 0..9 {
                full[(rows + i, i)] = 0.1_f64.sqrt();
            }
            let mut rhs = DVector::zeros(rows + 9);
            rhs.rows_mut(0, rows)
                .copy_from(&(-block.matrix.column(3).rows(0, rows)));
            let expected = full.svd(true, true).solve(&rhs, 1e-12).unwrap();
            block.compress();
            let mut system = System {
                blocks: vec![block],
                scaling: DVector::repeat(6, 1.0),
                damping_scale: DVector::repeat(6, 1.0),
                lambda: 0.0,
                cost: 0.0,
                preconditioner: None,
            };
            system.damp(0.1);
            let options = OptimizationOptions {
                linear_solver_eta: 1e-12,
                ..Default::default()
            };
            let (pose, _, valid) = system.solve::<f64>(&options);
            assert!(valid);
            assert!((&pose - expected.rows(3, 6)).norm() < 1e-9);
            let block = &system.blocks[0];
            let landmark = block
                .matrix
                .fixed_view::<3, 3>(0, 0)
                .upper_triangle()
                .solve_upper_triangular(
                    &(-block.matrix.column(9).fixed_rows::<3>(0)
                        - block.matrix.view((0, 3), (3, 6)) * pose),
                )
                .unwrap();
            assert!((landmark - expected.fixed_rows::<3>(0)).norm() < 1e-9);
        }
    }

    #[test]
    fn damping_retries_reuse_qr_without_changing_the_linearization() {
        let mut reused = fixture(16);
        reused.compress();
        let undamped = reused.matrix.clone();
        for lambda in [0.001, 0.002, 0.008, 0.064, 1.024, 32.768] {
            reused.damp(lambda);
            let mut fresh = fixture(16);
            fresh.compress();
            fresh.damp(lambda);
            assert!((&reused.matrix - fresh.matrix).norm() < 1e-12);
        }
        reused.damp(0.0);
        assert!((&reused.matrix - undamped).norm() < 1e-12);
    }

    fn check_implicit_operator<S: BasaltScalar>(tolerance: f64) {
        let rows = 24;
        let mut block = Block {
            id: Some(1),
            observations: vec![],
            columns: (0..12).collect(),
            matrix: DMatrix::from_fn(rows + 3, 4, |i, j| {
                S::from_f64_lossy(if i < rows {
                    ((i * 11 + j * 7 + 3) as f64).sin()
                } else {
                    0.0
                })
            }),
            landmark_scale: Vector3::repeat(S::one()),
            damping_scale: Vector3::repeat(S::one()),
            rotations: vec![],
            pose_rows: (0..rows / 2)
                .map(|i| {
                    (
                        2 * i,
                        6 * (i % 2),
                        SMatrix::<S, 2, 6>::from_fn(|r, c| {
                            S::from_f64_lossy(((i * 13 + r * 7 + c * 3) as f64).cos())
                        }),
                    )
                })
                .collect(),
            reflectors: vec![],
            landmark_rows: DMatrix::zeros(0, 4),
            undamped_rhs: DVector::zeros(0),
            undamped_diagonal: vec![],
            short_reduced: None,
        };
        let mut full = DMatrix::<f64>::zeros(rows + 3, 15);
        full.columns_mut(0, 3)
            .copy_from(&block.matrix.columns(0, 3).map(BasaltScalar::as_f64));
        for &(row, offset, ref jacobian) in &block.pose_rows {
            full.fixed_view_mut::<2, 6>(row, 3 + offset)
                .copy_from(&jacobian.map(BasaltScalar::as_f64));
        }
        let residual = block.matrix.column(3).map(BasaltScalar::as_f64);
        block.compress();
        let mut system = System {
            blocks: vec![block],
            scaling: DVector::repeat(12, S::one()),
            damping_scale: DVector::repeat(12, S::one()),
            lambda: S::zero(),
            cost: S::zero(),
            preconditioner: None,
        };
        // The independent oracle projects through the thin SVD of the augmented
        // landmark Jacobian, without using the production QR or compressed rows.
        for lambda in [0.001_f64, 0.1, 10.0, 0.01] {
            for i in 0..3 {
                full[(rows + i, i)] = lambda.sqrt();
            }
            let landmark = full.columns(0, 3).into_owned();
            let pose = full.columns(3, 12).into_owned();
            let inverse = landmark.svd(true, true).pseudo_inverse(1e-12).unwrap();
            let projection =
                DMatrix::<f64>::identity(rows + 3, rows + 3) - full.columns(0, 3) * inverse;
            let normal =
                pose.transpose() * &projection * &pose + DMatrix::<f64>::identity(12, 12) * lambda;
            let expected_rhs = -pose.transpose() * projection * &residual;
            system.damp(S::from_f64_lossy(lambda));
            let (rhs, diagonal_blocks) = system.prepare::<S>();
            let inverse_blocks: Vec<_> = diagonal_blocks
                .into_iter()
                .map(|b| b.cholesky().unwrap().inverse())
                .collect();
            assert!((rhs.map(BasaltScalar::as_f64) - expected_rhs).amax() < tolerance);
            for axis in 0..12 {
                let mut input = DVector::zeros(12);
                input[axis] = S::one();
                assert!(
                    (system.normal_product(&input).map(BasaltScalar::as_f64) - normal.column(axis))
                        .amax()
                        < tolerance
                );
            }
            let cached = system.blocks[0].short_reduced.take();
            for axis in 0..12 {
                let mut input = DVector::zeros(12);
                input[axis] = S::one();
                assert!(
                    (system.normal_product(&input).map(BasaltScalar::as_f64) - normal.column(axis))
                        .amax()
                        < tolerance
                );
            }
            system.blocks[0].short_reduced = cached;
            for (i, inverse) in inverse_blocks.iter().enumerate() {
                let expected = normal.fixed_view::<6, 6>(6 * i, 6 * i).into_owned();
                let inverse = inverse.map(BasaltScalar::as_f64);
                // The sinusoidal fixture is rank deficient before damping.
                // Certify inverse backward error; a raw identity residual is
                // amplified by its roughly 70,000 condition number in float.
                let backward_error = (expected * inverse - Matrix6::<f64>::identity()).norm()
                    / (expected.norm() * inverse.norm());
                assert!(
                    backward_error < 8.0 * machine_epsilon::<S>().as_f64(),
                    "inverse backward error={backward_error}, lambda={lambda}"
                );
            }
        }
    }

    #[test]
    fn implicit_multi_pose_operator_and_preconditioner_match_svd_projection() {
        check_implicit_operator::<f64>(1e-10);
        check_implicit_operator::<f32>(1e-4);
    }

    fn check_cluster_preconditioner<S: BasaltScalar>(tolerance: f64) {
        let original = fixture(24);
        let block = Block {
            id: Some(1),
            observations: vec![],
            columns: [0, 6, 18, 36]
                .into_iter()
                .flat_map(|start| start..start + 6)
                .collect(),
            matrix: original.matrix.map(S::from_f64_lossy),
            landmark_scale: Vector3::repeat(S::one()),
            damping_scale: Vector3::repeat(S::one()),
            rotations: vec![],
            pose_rows: original
                .pose_rows
                .into_iter()
                .enumerate()
                .map(|(i, (row, _, matrix))| (row, 6 * (i % 4), matrix.map(S::from_f64_lossy)))
                .collect(),
            reflectors: vec![],
            landmark_rows: DMatrix::zeros(0, 4),
            undamped_rhs: DVector::zeros(0),
            undamped_diagonal: vec![],
            short_reduced: None,
        };
        let mut system = System {
            blocks: vec![block],
            scaling: DVector::repeat(42, S::one()),
            damping_scale: DVector::repeat(42, S::one()),
            lambda: S::zero(),
            cost: S::zero(),
            preconditioner: None,
        };
        system.blocks[0].compress();
        for columns in [[12, 6], [18, 12]] {
            system.blocks.push(Block {
                id: None,
                observations: vec![],
                columns: columns
                    .into_iter()
                    .flat_map(|start| start..start + 6)
                    .collect(),
                matrix: DMatrix::from_fn(6, 13, |r, c| {
                    S::from_f64_lossy(((r * 13 + c * 7 + 1) as f64).cos())
                }),
                landmark_scale: Vector3::zeros(),
                damping_scale: Vector3::zeros(),
                rotations: vec![],
                pose_rows: vec![],
                reflectors: vec![],
                landmark_rows: DMatrix::zeros(0, 4),
                undamped_rhs: DVector::zeros(0),
                undamped_diagonal: vec![],
                short_reduced: None,
            });
        }
        for lambda in [0.001, 0.1, 10.0, 0.01] {
            system.damp(S::from_f64_lossy(lambda));
            let (_, diagonal) = system.prepare::<S>();
            let factors = system.cluster_preconditioner(&diagonal, 18).unwrap();
            let normal = DMatrix::from_columns(
                &(0..42)
                    .map(|i| {
                        let mut input = DVector::<S>::zeros(42);
                        input[i] = S::one();
                        system.normal_product(&input).map(BasaltScalar::as_f64)
                    })
                    .collect::<Vec<_>>(),
            );
            for (i, factor) in factors.iter().enumerate() {
                let rows = factor.L().nrows();
                let lower =
                    DMatrix::from_fn(
                        rows,
                        rows,
                        |r, c| if r >= c { factor.L()[(r, c)] } else { 0.0 },
                    );
                let mut expected = normal.view((18 * i, 18 * i), (rows, rows)).into_owned();
                let ridge = expected.diagonal().amax() * 8.0 * machine_epsilon::<S>().as_f64();
                for j in 0..rows {
                    expected[(j, j)] += ridge;
                }
                assert!(
                    (&lower * lower.transpose() - &expected).amax()
                        < tolerance * expected.amax().max(1.0)
                );
            }
            // Reversed graph endpoints, cluster boundaries and a short final
            // cluster are all represented in the independently applied operator.
            assert_eq!(factors.len(), 3);
            assert_eq!(factors[2].L().nrows(), 6);
        }
        let (_, mut diagonal) = system.prepare::<S>();
        diagonal[0].fill(S::from_f64_lossy(f64::NAN));
        assert!(system.cluster_preconditioner(&diagonal, 18).is_none());
    }

    #[test]
    fn clustered_preconditioner_matches_qr_blocks_in_both_precisions() {
        check_cluster_preconditioner::<f64>(1e-12);
        check_cluster_preconditioner::<f32>(3e-6);
        for columns in [6, 96, 2376, 6000, 60000] {
            let width = cluster_width(columns);
            assert!((6..=6 * MAX_CLUSTER_POSES).contains(&width) && width.is_multiple_of(6));
            let bytes: usize = (0..columns)
                .step_by(width)
                .map(|i| (columns - i).min(width).pow(2) * 8)
                .sum();
            assert!(bytes <= CLUSTER_STORAGE_BYTES);
        }
    }

    fn check_cached_cluster_solve<S: BasaltScalar>(tolerance: f64) {
        let columns = 96;
        let matrix = DMatrix::from_fn(columns + 8, columns + 1, |row, column| {
            S::from_f64_lossy(
                ((row * 13 + column * 7 + 1) as f64).sin() + if row == column { 2.0 } else { 0.0 },
            )
        });
        let mut system = System {
            blocks: vec![Block {
                id: None,
                observations: vec![],
                columns: (0..columns).collect(),
                matrix,
                landmark_scale: Vector3::zeros(),
                damping_scale: Vector3::zeros(),
                rotations: vec![],
                pose_rows: vec![],
                reflectors: vec![],
                landmark_rows: DMatrix::zeros(0, 4),
                undamped_rhs: DVector::zeros(0),
                undamped_diagonal: vec![],
                short_reduced: None,
            }],
            scaling: DVector::repeat(columns, S::one()),
            damping_scale: DVector::repeat(columns, S::one()),
            lambda: S::zero(),
            cost: S::zero(),
            preconditioner: None,
        };
        let mut cached_pointer = None;
        for lambda in [0.001, 10.0, 0.01] {
            system.damp(S::from_f64_lossy(lambda));
            let (step, _, valid) = system.solve::<S>(&OptimizationOptions {
                linear_solver_gradient_tolerance: 1e-10,
                ..Default::default()
            });
            assert!(valid);
            let factor = &system.preconditioner.as_ref().unwrap()[0];
            let pointer = factor.L().as_ptr();
            assert_eq!(*cached_pointer.get_or_insert(pointer), pointer);
            let rows = system.blocks[0].matrix.nrows();
            let mut full = DMatrix::<f64>::zeros(rows + columns, columns);
            full.rows_mut(0, rows).copy_from(
                &system.blocks[0]
                    .matrix
                    .columns(0, columns)
                    .map(BasaltScalar::as_f64),
            );
            for i in 0..columns {
                full[(rows + i, i)] = lambda.sqrt();
            }
            let mut rhs = DVector::<f64>::zeros(rows + columns);
            rhs.rows_mut(0, rows).copy_from(
                &(-system.blocks[0]
                    .matrix
                    .column(columns)
                    .map(BasaltScalar::as_f64)),
            );
            let gradient = full.transpose() * (&full * step.map(BasaltScalar::as_f64) - &rhs);
            let initial = (full.transpose() * &rhs).norm();
            assert!(
                gradient.norm() / initial
                    < 1.05 * 1e-10_f64.max(machine_epsilon::<S>().as_f64().sqrt())
            );
            let expected = full.svd(true, true).solve(&rhs, 1e-12).unwrap();
            let error = (step.map(BasaltScalar::as_f64) - &expected).norm() / expected.norm();
            assert!(
                error < tolerance,
                "precision={} lambda={lambda} relative step error={error}, tolerance={tolerance}",
                std::mem::size_of::<S>()
            );
        }
        let (step, _, valid) = system.solve::<S>(&OptimizationOptions {
            preconditioner: Preconditioner::SchurJacobi,
            ..Default::default()
        });
        assert!(valid && step.iter().all(nalgebra::ComplexField::is_finite));
    }

    #[test]
    fn cached_cluster_factor_survives_damping_retries_with_svd_accuracy() {
        check_cached_cluster_solve::<f64>(2e-5);
        check_cached_cluster_solve::<f32>(2e-3);
    }

    fn check_row_kernels<S: BasaltScalar, W: BasaltScalar>(tolerance: f64) {
        for rows in [0, 1, 2, 3, 4, 5, 6, 9, 16] {
            let matrix = DMatrix::from_fn(rows, 12, |r, c| {
                S::from_f64_lossy(((r * 13 + c * 7 + 1) as f64).sin())
            });
            let columns: Vec<_> = (6..12).chain(18..24).collect();
            let input = DVector::from_fn(30, |i, _| W::from_f64_lossy((i as f64).cos()));
            let mut output = DVector::repeat(30, W::from_f64_lossy(2.0));
            add_normal_rows(
                matrix.as_view(),
                &columns,
                &input,
                &mut output,
                &mut Vec::new(),
            );
            let local = DVector::from_iterator(12, columns.iter().map(|&i| input[i].as_f64()));
            let matrix = matrix.map(BasaltScalar::as_f64);
            let normal = matrix.transpose() * (&matrix * local);
            let mut expected = DVector::repeat(30, 2.0);
            for (i, &global) in columns.iter().enumerate() {
                expected[global] += normal[i];
            }
            assert!((output.map(BasaltScalar::as_f64) - expected).amax() < tolerance);
        }
    }

    #[test]
    fn fixed_and_dynamic_row_kernels_preserve_scattered_accumulation() {
        check_row_kernels::<f64, f64>(1e-12);
        check_row_kernels::<f32, f32>(1e-5);
        check_row_kernels::<f32, f64>(1e-12);
    }

    fn check_upstream_cg<S: BasaltScalar>(reference: &serde_json::Value, tolerance: f64) {
        let matrix = DMatrix::from_fn(18, 13, |i, j| {
            S::from_f64_lossy(if j == 12 {
                (i as f64 * 5.0 + 1.0).cos()
            } else {
                (((i + 1) * (j + 2)) as f64 * 0.37).sin() + if i == j { 0.5 } else { 0.0 }
            })
        });
        let block = Block {
            id: None,
            observations: vec![],
            columns: (0..12).collect(),
            matrix,
            landmark_scale: Vector3::zeros(),
            damping_scale: Vector3::zeros(),
            rotations: vec![],
            pose_rows: vec![],
            reflectors: vec![],
            landmark_rows: DMatrix::zeros(0, 4),
            undamped_rhs: DVector::zeros(0),
            undamped_diagonal: vec![],
            short_reduced: None,
        };
        let mut system = System {
            blocks: vec![block],
            scaling: DVector::repeat(12, S::one()),
            damping_scale: DVector::repeat(12, S::one()),
            lambda: S::from_f64_lossy(0.01),
            cost: S::zero(),
            preconditioner: None,
        };
        let (step, iterations, valid) = system.solve::<S>(&OptimizationOptions {
            preconditioner: Preconditioner::SchurJacobi,
            linear_solver_gradient_tolerance: 0.0,
            ..Default::default()
        });
        assert!(valid);
        assert_eq!(
            iterations,
            reference["iterations"].as_u64().unwrap() as usize
        );
        for (value, expected) in step.iter().zip(reference["step"].as_array().unwrap()) {
            assert!((value.as_f64() - expected.as_f64().unwrap()).abs() < tolerance);
        }
        let (certified, _, valid) = system.solve::<S>(&OptimizationOptions::default());
        assert!(valid);
        let matrix = system.blocks[0]
            .matrix
            .columns(0, 12)
            .map(BasaltScalar::as_f64);
        let residual = system.blocks[0].matrix.column(12).map(BasaltScalar::as_f64);
        let gradient =
            |step: DVector<f64>| matrix.transpose() * (&matrix * &step + &residual) + step * 0.01;
        let initial = (matrix.transpose() * &residual).norm();
        assert!(gradient(step.map(BasaltScalar::as_f64)).norm() / initial > 1e-3);
        assert!(
            gradient(certified.map(BasaltScalar::as_f64)).norm() / initial
                <= 1e-5_f64.max(machine_epsilon::<S>().as_f64().sqrt())
        );
    }

    #[test]
    fn pcg_step_and_stopping_match_unmodified_upstream_cpp() {
        let reference: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/reference/rootba_cg.json")).unwrap();
        check_upstream_cg::<f64>(&reference[0], 1e-12);
        check_upstream_cg::<f32>(&reference[1], 2e-6);
    }

    fn check_upstream_scaling<S: BasaltScalar>(reference: &serde_json::Value) {
        let epsilon = reference["jacobi_scaling_epsilon"].as_f64().unwrap();
        for norm in [0.0, 1e-8, 0.001, 1.0, 100.0] {
            let expected = 1.0 / (epsilon + norm);
            let actual = column_scale(S::from_f64_lossy(norm)).as_f64();
            assert!(
                (actual - expected).abs() / expected < 4.0 * machine_epsilon::<S>().as_f64(),
                "norm={norm}, upstream={expected}, actual={actual}"
            );
        }
    }

    #[test]
    fn jacobian_scaling_matches_upstream_including_weak_and_zero_columns() {
        let reference: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/reference/rootba_cg.json")).unwrap();
        check_upstream_scaling::<f64>(&reference[0]);
        check_upstream_scaling::<f32>(&reference[1]);
    }
}
