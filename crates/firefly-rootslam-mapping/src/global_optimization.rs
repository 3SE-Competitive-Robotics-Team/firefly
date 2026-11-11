//! Rig square-root bundle adjustment, `RootSLAM` equations (2)-(11).
//! Landmark blocks retain their sparse pose-column incidence. The reduced
//! camera operator is applied implicitly after landmark QR elimination.
mod rootba;
use crate::{map::MapSnapshot, pose_graph};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::scalar::BasaltScalar;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{DMatrix, DVector, Isometry3, SMatrix, Vector2, Vector3, Vector4};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Precision {
    Float32,
    #[default]
    Float64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum KrylovPrecision {
    Native,
    Float64,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinearSolver {
    #[default]
    Rootba,
    /// Slow, independent numerical reference with physical-coordinate damping.
    LsqrReference,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DampingCoordinates {
    /// Preserve the rig's physical-coordinate trust region under column scaling.
    #[default]
    Physical,
    /// `RootBA`'s default trust region on Jacobian-scaled coordinates.
    Scaled,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct OptimizationOptions {
    pub linear_solver: LinearSolver,
    pub damping_coordinates: DampingCoordinates,
    /// `RootBA`'s truncated-Newton quadratic-model stopping criterion.
    pub linear_solver_eta: f64,
    /// Additional physical-gradient accuracy guard; zero selects only `RootBA`'s model test.
    pub linear_solver_gradient_tolerance: f64,
    pub linear_solver_max_iterations: usize,
    pub precision: Precision,
    pub krylov_precision: KrylovPrecision,
    pub max_iterations: usize,
    pub initial_damping: f64,
    pub minimum_damping: f64,
    pub maximum_damping: f64,
    pub step_tolerance: f64,
    pub relative_cost_tolerance: f64,
    pub lsqr_tolerance: f64,
    pub lsqr_max_iterations: usize,
    pub pose_graph_terms: bool,
    /// LSQR reference only: numerical fixture of the first linear solve that misses its
    /// convergence criterion. The archive includes the quantized matrix and RHS.
    pub linear_system_archive: Option<std::path::PathBuf>,
    /// Optional frozen geometry, observations, calibration and solver settings.
    pub problem_archive: Option<std::path::PathBuf>,
}
impl Default for OptimizationOptions {
    fn default() -> Self {
        Self {
            linear_solver: LinearSolver::Rootba,
            damping_coordinates: DampingCoordinates::Physical,
            linear_solver_eta: 0.1,
            linear_solver_gradient_tolerance: 1e-5,
            linear_solver_max_iterations: 500,
            precision: Precision::Float64,
            // Matrix products and work vectors follow the storage precision.
            // RootBA's recurrence coefficients and objective sums use f64;
            // Float64 below additionally promotes the Krylov work vectors.
            krylov_precision: KrylovPrecision::Native,
            max_iterations: 50,
            initial_damping: 1e-3,
            minimum_damping: 1e-8,
            maximum_damping: 1e12,
            step_tolerance: 1e-6,
            relative_cost_tolerance: 1e-6,
            lsqr_tolerance: 1e-5,
            lsqr_max_iterations: 1000,
            pose_graph_terms: true,
            linear_system_archive: None,
            problem_archive: None,
        }
    }
}
impl OptimizationOptions {
    /// # Errors
    /// Returns an error for invalid damping, tolerances, or iteration limits.
    pub fn validate(&self) -> Result<()> {
        let positive = [
            self.initial_damping,
            self.minimum_damping,
            self.maximum_damping,
            self.lsqr_tolerance,
            self.linear_solver_eta,
        ];
        if positive.iter().any(|x| !x.is_finite() || *x <= 0.0)
            || self.initial_damping < self.minimum_damping
            || self.initial_damping > self.maximum_damping
            || !self.step_tolerance.is_finite()
            || self.step_tolerance < 0.0
            || !self.relative_cost_tolerance.is_finite()
            || !self.linear_solver_gradient_tolerance.is_finite()
            || self.linear_solver_gradient_tolerance < 0.0
            || self.relative_cost_tolerance < 0.0
            || self.max_iterations == 0
            || self.lsqr_max_iterations == 0
            || self.linear_solver_max_iterations == 0
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid optimization damping, tolerances, or iteration limits",
            ));
        }
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct OptimizationResult {
    pub revision: u64,
    pub poses: BTreeMap<i64, Isometry3<f64>>,
    /// Optimized points only; deferred landmarks follow their host correction
    /// when these updates are applied to the persistent map.
    pub points: BTreeMap<u64, Vector3<f64>>,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub accepted_steps: usize,
    pub converged: bool,
    pub linear_iterations: usize,
    pub peak_linear_storage_bytes: usize,
    /// Precision of the matrix storage and the landmark elimination.
    pub precision: Precision,
    /// Precision of the Krylov work vectors and accumulation.
    pub krylov_precision: KrylovPrecision,
}

struct RowBlock<S: BasaltScalar> {
    columns: Vec<usize>,
    matrix: DMatrix<S>,
    residual: DVector<S>,
}
struct LandmarkBlock<S: BasaltScalar> {
    id: u64,
    observations: Vec<crate::map::ObservationKey>,
    columns: Vec<usize>,
    matrix: DMatrix<S>,
    residual: DVector<S>,
    original_jacobian: DMatrix<S>,
    original_residual: DVector<S>,
}
struct SquareRootSystem<S: BasaltScalar> {
    columns: usize,
    blocks: Vec<RowBlock<S>>,
    landmarks: Vec<LandmarkBlock<S>>,
    damping: S,
}
impl<S: BasaltScalar> SquareRootSystem<S> {
    fn is_finite(&self) -> bool {
        self.blocks.iter().all(|block| {
            block
                .matrix
                .iter()
                .chain(block.residual.iter())
                .all(nalgebra::ComplexField::is_finite)
        }) && self.landmarks.iter().all(|block| {
            block
                .matrix
                .iter()
                .chain(block.residual.iter())
                .all(nalgebra::ComplexField::is_finite)
        })
    }
    fn rows(&self) -> usize {
        self.blocks.iter().map(|b| b.matrix.nrows()).sum::<usize>() + self.columns
    }
    fn multiply<W: BasaltScalar>(&self, input: &DVector<W>) -> DVector<W> {
        let mut output = DVector::zeros(self.rows());
        let mut offset = 0;
        for block in &self.blocks {
            let rows = block.matrix.nrows();
            let target = &mut output.as_mut_slice()[offset..offset + rows];
            // Columns are contiguous. Each output still accumulates columns
            // in the original order, including its initial positive zero.
            for (local, &global) in block.columns.iter().enumerate() {
                let scale = input[global];
                let coefficients = &block.matrix.as_slice()[local * rows..(local + 1) * rows];
                for (value, coefficient) in target.iter_mut().zip(coefficients) {
                    *value += W::from_f64_lossy(coefficient.as_f64()) * scale;
                }
            }
            offset += rows;
        }
        for i in 0..self.columns {
            output[offset + i] = W::from_f64_lossy(self.damping.as_f64()) * input[i];
        }
        output
    }
    fn transpose_multiply<W: BasaltScalar>(&self, input: &DVector<W>) -> DVector<W> {
        let mut output = DVector::zeros(self.columns);
        let mut offset = 0;
        for block in &self.blocks {
            for (local, &global) in block.columns.iter().enumerate() {
                let mut sum = W::zero();
                for row in 0..block.matrix.nrows() {
                    sum += W::from_f64_lossy(block.matrix[(row, local)].as_f64())
                        * input[offset + row];
                }
                output[global] += sum;
            }
            offset += block.matrix.nrows();
        }
        for i in 0..self.columns {
            output[i] += W::from_f64_lossy(self.damping.as_f64()) * input[offset + i];
        }
        output
    }
    fn rhs(&self) -> DVector<S> {
        let mut rhs = DVector::zeros(self.rows());
        let mut offset = 0;
        for block in &self.blocks {
            rhs.rows_mut(offset, block.residual.len())
                .copy_from(&(-&block.residual));
            offset += block.residual.len();
        }
        rhs
    }
    fn inverse_column_norms(&self) -> DVector<S> {
        // Scale the complete damped operator A, including its regularization
        // rows: A D y = b with the physical update x = D y.
        let mut norms = DVector::from_element(self.columns, self.damping.abs());
        for block in &self.blocks {
            for (local, &global) in block.columns.iter().enumerate() {
                for value in block.matrix.column(local).iter() {
                    norms[global] = norms[global].hypot(*value);
                }
            }
        }
        norms.map(|norm| {
            let inverse = S::one() / norm;
            if inverse > S::zero() && inverse.is_finite() {
                inverse
            } else {
                S::one()
            }
        })
    }
    fn physical_gradient_norm(&self, step: Option<&DVector<S>>) -> f64 {
        // Evaluate the same quantized operator with wider accumulation. Near
        // an inconsistent least-squares optimum, float residual cancellation
        // can otherwise reject a representable step satisfying the criterion.
        let mut gradient = DVector::<f64>::zeros(self.columns);
        let mut residual = Vec::new();
        for block in &self.blocks {
            residual.resize(block.matrix.nrows(), 0.0);
            for (row, value) in residual.iter_mut().enumerate() {
                let mut sum = 0.0;
                if let Some(step) = step {
                    for (local, &global) in block.columns.iter().enumerate() {
                        sum += block.matrix[(row, local)].as_f64() * step[global].as_f64();
                    }
                }
                *value = sum + block.residual[row].as_f64();
            }
            for (local, &global) in block.columns.iter().enumerate() {
                let mut sum = 0.0;
                for (row, value) in residual.iter().enumerate() {
                    sum += block.matrix[(row, local)].as_f64() * value;
                }
                gradient[global] += sum;
            }
        }
        if let Some(step) = step {
            let damping = self.damping.as_f64();
            for i in 0..self.columns {
                gradient[i] += damping * (damping * step[i].as_f64());
            }
        }
        gradient.norm()
    }
    fn storage_bytes(&self) -> usize {
        let values = self
            .blocks
            .iter()
            .map(|b| b.matrix.len() + b.residual.len())
            .sum::<usize>()
            + self
                .landmarks
                .iter()
                .map(|b| {
                    b.matrix.len()
                        + b.residual.len()
                        + b.original_jacobian.len()
                        + b.original_residual.len()
                })
                .sum::<usize>();
        values * std::mem::size_of::<S>()
    }
    fn save_archive_once(
        &self,
        destination: &mut Option<&std::path::Path>,
        step: &DVector<S>,
        tolerance: f64,
    ) -> Result<()> {
        let Some(path) = destination.take() else {
            return Ok(());
        };
        let blocks: Vec<_> = self.blocks.iter().map(|block| serde_json::json!({
            "columns": block.columns,
            "rows": block.matrix.nrows(),
            "matrix_column_major": block.matrix.iter().map(|x| x.as_f64()).collect::<Vec<_>>(),
            "residual": block.residual.iter().map(|x| x.as_f64()).collect::<Vec<_>>()
        })).collect();
        let archive = serde_json::json!({
            "scalar_bytes": std::mem::size_of::<S>(),
            "columns": self.columns,
            "damping": self.damping.as_f64(),
            "blocks": blocks,
            "step": step.iter().map(|x| x.as_f64()).collect::<Vec<_>>(),
            "relative_gradient_tolerance": tolerance
        });
        let error = |source: std::io::Error| {
            Error::new(ErrorKind::Internal, "Cannot write linear-system fixture")
                .with_operation("rootslam.global.linear_system_archive")
                .with_source(source)
        };
        let file = std::fs::File::create_new(path).map_err(error)?;
        let mut writer = std::io::BufWriter::new(file);
        serde_json::to_writer(&mut writer, &archive).map_err(|source| {
            Error::new(ErrorKind::Internal, "Cannot encode linear-system fixture")
                .with_operation("rootslam.global.linear_system_archive")
                .with_source(source)
        })?;
        std::io::Write::flush(&mut writer).map_err(error)
    }
}

/// Paige-Saunders bidiagonal LSQR with column-norm right preconditioning.
/// The stopping criterion remains the gradient in physical coordinates.
fn solve_lsqr<S: BasaltScalar>(
    system: &SquareRootSystem<S>,
    options: &OptimizationOptions,
) -> (DVector<S>, usize, bool) {
    match options.krylov_precision {
        KrylovPrecision::Native => solve_lsqr_with_workspace::<S, S>(system, options),
        KrylovPrecision::Float64 => solve_lsqr_with_workspace::<S, f64>(system, options),
    }
}

/// Relative gradient reduction the solver may demand from the working precision.
///
/// The damped-system gradient reduction uses a square-root-epsilon floor from
/// the Krylov working precision. Matrix storage and Krylov arithmetic may have
/// separate precisions in frozen-problem numerical comparisons.
fn effective_lsqr_tolerance<W: BasaltScalar>(requested: f64) -> f64 {
    requested.max(W::epsilon_sqrt().as_f64())
}

fn solve_lsqr_with_workspace<S: BasaltScalar, W: BasaltScalar>(
    system: &SquareRootSystem<S>,
    options: &OptimizationOptions,
) -> (DVector<S>, usize, bool) {
    let mut x = DVector::zeros(system.columns);
    let scaling = system
        .inverse_column_norms()
        .map(|value| W::from_f64_lossy(value.as_f64()));
    let physical_step = |x: &DVector<W>| {
        x.component_mul(&scaling)
            .map(|value| S::from_f64_lossy(value.as_f64()))
    };
    let rhs = system.rhs().map(|value| W::from_f64_lossy(value.as_f64()));
    let mut u = rhs.clone();
    let mut beta = u.norm();
    if beta == W::zero() {
        return (physical_step(&x), 0, true);
    }
    u /= beta;
    let mut v = system.transpose_multiply(&u).component_mul(&scaling);
    let mut alpha = v.norm();
    if alpha == W::zero() {
        return (physical_step(&x), 0, true);
    }
    v /= alpha;
    let mut w = v.clone();
    let mut phi_bar = beta;
    let mut rho_bar = alpha;
    let tolerance = effective_lsqr_tolerance::<W>(options.lsqr_tolerance);
    let initial_gradient = system.physical_gradient_norm(None);
    for iteration in 1..=options.lsqr_max_iterations {
        u = system.multiply(&v.component_mul(&scaling)) - u * alpha;
        beta = u.norm();
        if beta > W::zero() {
            u /= beta;
        }
        v = system.transpose_multiply(&u).component_mul(&scaling) - v * beta;
        alpha = v.norm();
        if alpha > W::zero() {
            v /= alpha;
        }
        let rho = (rho_bar * rho_bar + beta * beta).sqrt();
        if rho == W::zero() {
            let physical = physical_step(&x);
            let converged =
                system.physical_gradient_norm(Some(&physical)) <= tolerance * initial_gradient;
            return (physical, iteration, converged);
        }
        let cosine = rho_bar / rho;
        let sine = beta / rho;
        let theta = sine * alpha;
        rho_bar = -cosine * alpha;
        let phi = cosine * phi_bar;
        phi_bar *= sine;
        x += &w * (phi / rho);
        w = &v - w * (theta / rho);
        if iteration % 8 == 0
            || iteration == options.lsqr_max_iterations
            || alpha == W::zero()
            || beta == W::zero()
        {
            let physical = physical_step(&x);
            let gradient = system.physical_gradient_norm(Some(&physical));
            if gradient <= tolerance * initial_gradient {
                return (physical, iteration, true);
            }
        }
        if !x.iter().all(nalgebra::ComplexField::is_finite) {
            return (physical_step(&x), iteration, false);
        }
    }
    (physical_step(&x), options.lsqr_max_iterations, false)
}

fn eliminate_landmark<S: BasaltScalar>(matrix: &mut DMatrix<S>, residual: &mut DVector<S>) {
    // Givens rotations annihilate landmark columns 0..3 and apply the same
    // orthogonal transforms to sparse incident pose columns and the residual.
    for col in 0..3 {
        for row in (col + 1..matrix.nrows()).rev() {
            let top = matrix[(col, col)];
            let bottom = matrix[(row, col)];
            if bottom == S::zero() {
                continue;
            }
            let scale = top.abs().max(bottom.abs());
            let radius = scale * ((top / scale).powi(2) + (bottom / scale).powi(2)).sqrt();
            let cosine = top / radius;
            let sine = bottom / radius;
            for j in col..matrix.ncols() {
                let a = matrix[(col, j)];
                let b = matrix[(row, j)];
                matrix[(col, j)] = cosine * a + sine * b;
                matrix[(row, j)] = -sine * a + cosine * b;
            }
            let a = residual[col];
            let b = residual[row];
            residual[col] = cosine * a + sine * b;
            residual[row] = -sine * a + cosine * b;
        }
    }
}

fn camera_factor<S: BasaltScalar>(
    pose: &Isometry3<S>,
    point: &Vector3<S>,
    extrinsic: &Isometry3<S>,
    camera: &firefly_calibration::camera::generic_camera::GenericCamera<S>,
    pixel: &Vector2<S>,
) -> Option<(Vector2<S>, SMatrix<S, 2, 6>, SMatrix<S, 2, 3>)> {
    camera_factor_impl(pose, point, extrinsic, camera, pixel, false)
}

fn camera_factor_on_unit_ray<S: BasaltScalar>(
    pose: &Isometry3<S>,
    point: &Vector3<S>,
    extrinsic: &Isometry3<S>,
    camera: &firefly_calibration::camera::generic_camera::GenericCamera<S>,
    pixel: &Vector2<S>,
) -> Option<(Vector2<S>, SMatrix<S, 2, 6>, SMatrix<S, 2, 3>)> {
    camera_factor_impl(pose, point, extrinsic, camera, pixel, true)
}

fn camera_factor_impl<S: BasaltScalar>(
    pose: &Isometry3<S>,
    point: &Vector3<S>,
    extrinsic: &Isometry3<S>,
    camera: &firefly_calibration::camera::generic_camera::GenericCamera<S>,
    pixel: &Vector2<S>,
    unit_ray: bool,
) -> Option<(Vector2<S>, SMatrix<S, 2, 6>, SMatrix<S, 2, 3>)> {
    let camera_world = pose_graph::product(extrinsic, pose);
    let camera_point = camera_world.rotation * point + camera_world.translation.vector;
    // Basalt's optical-axis fallbacks use a geometric epsilon. Supplying a unit
    // ray makes that threshold angular rather than dependent on landmark depth.
    let scale = if unit_ray {
        S::one() / camera_point.x.hypot(camera_point.y).hypot(camera_point.z)
    } else {
        S::one()
    };
    let homogeneous = Vector4::new(
        camera_point.x * scale,
        camera_point.y * scale,
        camera_point.z * scale,
        S::one(),
    );
    let mut prediction = Vector2::zeros();
    let mut projection = SMatrix::<S, 2, 4>::zeros();
    let in_domain = camera.project(&homogeneous, &mut prediction, Some(&mut projection));
    if !in_domain
        || !prediction.iter().all(nalgebra::ComplexField::is_finite)
        || !projection.iter().all(nalgebra::ComplexField::is_finite)
    {
        return None;
    }
    // Central projection is homogeneous: J(unit_ray) * unit_ray = 0, so the
    // normalization chain rule reduces to J(unit_ray) / |camera_point|.
    let projection = projection.fixed_columns::<3>(0).into_owned() * scale;
    let mut motion = SMatrix::<S, 3, 6>::zeros();
    motion
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&SMatrix::<S, 3, 3>::identity());
    motion
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&-pose_graph::hat(&camera_point));
    Some((
        prediction - pixel,
        projection * motion * pose_graph::adjoint(extrinsic),
        projection * camera_world.rotation.to_rotation_matrix().into_inner(),
    ))
}

struct State<S: BasaltScalar> {
    poses: BTreeMap<i64, Isometry3<S>>,
    points: BTreeMap<u64, Vector3<S>>,
}
impl<S: BasaltScalar> Clone for State<S> {
    fn clone(&self) -> Self {
        Self {
            poses: self.poses.clone(),
            points: self.points.clone(),
        }
    }
}

fn build_system<S: BasaltScalar>(
    snapshot: &MapSnapshot,
    state: &State<S>,
    calib: &Calibration<S>,
    order: &BTreeMap<i64, usize>,
    lambda: S,
    options: &OptimizationOptions,
) -> Result<(SquareRootSystem<S>, S)> {
    let mut system = SquareRootSystem {
        columns: order.len() * 6,
        blocks: Vec::new(),
        landmarks: Vec::new(),
        damping: lambda.sqrt(),
    };
    let mut cost = 0.0;
    for (&id, landmark) in &snapshot.landmarks {
        let mut factors = Vec::new();
        let mut incident = BTreeSet::new();
        for (key, observation) in &landmark.observations {
            let Some(pose) = state.poses.get(&key.keyframe) else {
                continue;
            };
            let camera = calib.intrinsics.get(key.camera).ok_or_else(|| {
                Error::new(
                    ErrorKind::InvalidArgument,
                    "observation camera index outside calibration",
                )
            })?;
            let extrinsic = pose_graph::inverse(&calib.t_i_c[key.camera]);
            if let Some(factor) = camera_factor(
                pose,
                &state.points[&id],
                &extrinsic,
                camera,
                &observation.pixel.map(S::from_f64_lossy),
            ) {
                factors.push((*key, factor));
                if order.contains_key(&key.keyframe) {
                    incident.insert(key.keyframe);
                }
            }
        }
        if factors.is_empty() {
            continue;
        }
        let local: BTreeMap<_, _> = incident
            .into_iter()
            .enumerate()
            .map(|(i, t)| (t, i * 6))
            .collect();
        let columns: Vec<_> = local
            .keys()
            .flat_map(|time| order[time]..order[time] + 6)
            .collect();
        let rows = 2 * factors.len();
        let mut matrix = DMatrix::zeros(rows + 3, columns.len() + 3);
        let mut residual = DVector::zeros(rows + 3);
        let observations = factors.iter().map(|(key, _)| *key).collect();
        for (index, (key, (r, jp, jl))) in factors.into_iter().enumerate() {
            cost += r.iter().map(|v| v.as_f64().powi(2)).sum::<f64>();
            residual.rows_mut(index * 2, 2).copy_from(&r);
            matrix.view_mut((index * 2, 0), (2, 3)).copy_from(&jl);
            if let Some(&offset) = local.get(&key.keyframe) {
                matrix
                    .view_mut((index * 2, offset + 3), (2, 6))
                    .copy_from(&jp);
            }
        }
        let original_jacobian = matrix.rows(0, rows).into_owned();
        let original_residual = residual.rows(0, rows).into_owned();
        for i in 0..3 {
            matrix[(rows + i, i)] = lambda.sqrt();
        }
        eliminate_landmark(&mut matrix, &mut residual);
        system.blocks.push(RowBlock {
            columns: columns.clone(),
            matrix: matrix
                .view((3, 3), (matrix.nrows() - 3, columns.len()))
                .into_owned(),
            residual: residual.rows(3, residual.len() - 3).into_owned(),
        });
        system.landmarks.push(LandmarkBlock {
            id,
            observations,
            columns,
            matrix,
            residual,
            original_jacobian,
            original_residual,
        });
    }
    if options.pose_graph_terms {
        cost += append_graph_factors(snapshot, state, order, &mut system);
    }
    let cost = S::from_f64_lossy(cost * 0.5);
    if !cost.is_finite() || !system.is_finite() {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "Nonfinite global linearization or objective overflow",
        )
        .with_operation("rootslam.global.linearize"));
    }
    Ok((system, cost))
}

fn append_graph_factors<S: BasaltScalar>(
    snapshot: &MapSnapshot,
    state: &State<S>,
    order: &BTreeMap<i64, usize>,
    system: &mut SquareRootSystem<S>,
) -> f64 {
    let mut cost = 0.0;
    for edge in &snapshot.edges {
        if !edge.kind.is_measurement_bearing() {
            continue;
        }
        let (Some(first), Some(second)) = (state.poses.get(&edge.from), state.poses.get(&edge.to))
        else {
            continue;
        };
        let information = edge.sqrt_information.map(S::from_f64_lossy);
        let (residual, first_j, second_j) = pose_graph::linearize_edge(
            first,
            second,
            &pose_graph::cast_pose(&edge.measurement),
            &information,
        );
        cost += residual.iter().map(|v| v.as_f64().powi(2)).sum::<f64>();
        let mut columns = Vec::new();
        let mut pieces = Vec::new();
        for (time, jacobian) in [(edge.from, first_j), (edge.to, second_j)] {
            if let Some(&offset) = order.get(&time) {
                columns.extend(offset..offset + 6);
                pieces.push(jacobian);
            }
        }
        let mut matrix = DMatrix::zeros(6, columns.len());
        for (i, jacobian) in pieces.iter().enumerate() {
            matrix.view_mut((0, i * 6), (6, 6)).copy_from(jacobian);
        }
        system.blocks.push(RowBlock {
            columns,
            matrix,
            residual: DVector::from_column_slice(residual.as_slice()),
        });
    }
    cost
}

fn apply_step<S: BasaltScalar>(
    state: &State<S>,
    system: &SquareRootSystem<S>,
    order: &BTreeMap<i64, usize>,
    pose_increment: &DVector<S>,
) -> (State<S>, S) {
    let mut next = state.clone();
    let mut prediction = S::zero();
    for (&time, &offset) in order {
        let increment = pose_increment.fixed_rows::<6>(offset).into_owned();
        next.poses.insert(
            time,
            pose_graph::product(&pose_graph::se3_exp(&increment), &state.poses[&time]),
        );
    }
    for block in &system.landmarks {
        let mut local = DVector::zeros(block.matrix.ncols());
        for (i, &global) in block.columns.iter().enumerate() {
            local[i + 3] = pose_increment[global];
        }
        for i in (0..3).rev() {
            let mut rhs = -block.residual[i];
            for j in i + 1..block.matrix.ncols() {
                rhs -= block.matrix[(i, j)] * local[j];
            }
            local[i] = rhs / block.matrix[(i, i)];
        }
        *next.points.get_mut(&block.id).expect("landmark state") += local.fixed_rows::<3>(0);
        let change = &block.original_jacobian * &local;
        prediction -= change.dot(&(&block.original_residual + &change * S::from_f64_lossy(0.5)));
    }
    // Landmark reduced blocks already contributed through the original rows.
    for block in system.blocks.iter().skip(system.landmarks.len()) {
        let local = DVector::from_iterator(
            block.columns.len(),
            block.columns.iter().map(|&i| pose_increment[i]),
        );
        let change = &block.matrix * local;
        prediction -= change.dot(&(&block.residual + &change * S::from_f64_lossy(0.5)));
    }
    (next, prediction)
}

fn trial_cost<S: BasaltScalar>(
    snapshot: &MapSnapshot,
    state: &State<S>,
    calibration: &Calibration<S>,
    system: &SquareRootSystem<S>,
    options: &OptimizationOptions,
) -> S {
    let mut cost = 0.0;
    // Evaluate this LM proposal on the exact observation set used by linearization.
    for block in &system.landmarks {
        for key in &block.observations {
            let transform = pose_graph::product(
                &pose_graph::inverse(&calibration.t_i_c[key.camera]),
                &state.poses[&key.keyframe],
            );
            let p = transform.rotation * state.points[&block.id] + transform.translation.vector;
            let mut pixel = Vector2::zeros();
            if !calibration.intrinsics[key.camera].project(
                &Vector4::new(p.x, p.y, p.z, S::one()),
                &mut pixel,
                None,
            ) || !pixel.iter().all(nalgebra::ComplexField::is_finite)
            {
                return S::from_f64_lossy(f64::INFINITY);
            }
            let observation = &snapshot.landmarks[&block.id].observations[key];
            cost += (pixel - observation.pixel.map(S::from_f64_lossy))
                .iter()
                .map(|v| v.as_f64().powi(2))
                .sum::<f64>();
        }
    }
    if options.pose_graph_terms {
        let mut graph_cost = 0.0;
        for edge in &snapshot.edges {
            if !edge.kind.is_measurement_bearing() {
                continue;
            }
            let (Some(a), Some(b)) = (state.poses.get(&edge.from), state.poses.get(&edge.to))
            else {
                continue;
            };
            let prediction = pose_graph::product(a, &pose_graph::inverse(b));
            let error = pose_graph::se3_log(&pose_graph::product(
                &pose_graph::cast_pose(&edge.measurement),
                &pose_graph::inverse(&prediction),
            ));
            graph_cost += (edge.sqrt_information.map(S::from_f64_lossy) * error)
                .iter()
                .map(|v| v.as_f64().powi(2))
                .sum::<f64>();
        }
        cost += graph_cost;
    }
    S::from_f64_lossy(cost * 0.5)
}

fn state_from_snapshot<S: BasaltScalar>(snapshot: &MapSnapshot) -> Result<State<S>> {
    let state = State {
        poses: snapshot
            .keyframes
            .iter()
            .map(|(&id, k)| (id, pose_graph::inverse(&pose_graph::cast_pose(&k.t_w_b))))
            .collect(),
        points: snapshot
            .landmarks
            .iter()
            .map(|(&id, p)| (id, p.position.map(S::from_f64_lossy)))
            .collect(),
    };
    if state.poses.values().any(|p| {
        !p.to_homogeneous()
            .iter()
            .all(nalgebra::ComplexField::is_finite)
    }) || state
        .points
        .values()
        .any(|p| !p.iter().all(nalgebra::ComplexField::is_finite))
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "nonfinite map geometry or scalar conversion overflow",
        ));
    }
    Ok(state)
}

fn calibrated_rig<S: BasaltScalar>(calibration: &Calibration<f64>) -> Result<Calibration<S>> {
    if calibration.t_i_c.len() != calibration.intrinsics.len() {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "camera calibration dimensions differ",
        ));
    }
    if calibration.intrinsics.iter().any(|camera| {
        camera
            .param()
            .iter()
            .any(|&value| !S::from_f64_lossy(value).is_finite())
    }) || calibration.t_i_c.iter().any(|pose| {
        pose.to_homogeneous()
            .iter()
            .any(|&value| !S::from_f64_lossy(value).is_finite())
    }) {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "Nonfinite calibration or scalar conversion overflow",
        )
        .with_operation("rootslam.global.validate"));
    }
    Ok(calibration.cast::<S>())
}

fn validate_scalar_inputs<S: BasaltScalar>(
    snapshot: &MapSnapshot,
    options: &OptimizationOptions,
) -> Result<()> {
    for value in [
        options.initial_damping,
        options.minimum_damping,
        options.maximum_damping,
    ] {
        let value = S::from_f64_lossy(value);
        if !value.is_finite() || value <= S::zero() {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "Damping is not finite and positive in solver precision",
            )
            .with_operation("rootslam.global.validate"));
        }
    }
    if snapshot.landmarks.values().any(|landmark| {
        landmark.observations.values().any(|observation| {
            observation
                .pixel
                .iter()
                .any(|&value| !S::from_f64_lossy(value).is_finite())
        })
    }) || (options.pose_graph_terms
        && snapshot.edges.iter().any(|edge| {
            edge.sqrt_information
                .iter()
                .chain(edge.measurement.to_homogeneous().iter())
                .any(|&value| !S::from_f64_lossy(value).is_finite())
        }))
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "Nonfinite measurement or scalar conversion overflow",
        )
        .with_operation("rootslam.global.validate"));
    }
    Ok(())
}
fn pose_order(snapshot: &MapSnapshot) -> BTreeMap<i64, usize> {
    snapshot
        .keyframes
        .keys()
        .skip(1)
        .enumerate()
        .map(|(i, &id)| (id, i * 6))
        .collect()
}
#[expect(
    clippy::too_many_lines,
    reason = "Linearization, trial step and result assembly stay in one function"
)]
fn optimize<S: BasaltScalar>(
    snapshot: &MapSnapshot,
    calibration: &Calibration<f64>,
    options: &OptimizationOptions,
) -> Result<OptimizationResult> {
    options.validate()?;
    validate_scalar_inputs::<S>(snapshot, options)?;
    let calib = calibrated_rig::<S>(calibration)?;
    let mut state = state_from_snapshot(snapshot)?;
    save_problem(snapshot, calibration, options)?;
    let order = pose_order(snapshot);
    let mut lambda = S::from_f64_lossy(options.initial_damping);
    let mut multiplier = S::from_f64_lossy(2.0);
    let (_, initial) = build_system(snapshot, &state, &calib, &order, lambda, options)?;
    let mut cost = initial;
    let (mut accepted, mut iterations, mut linear_iterations, mut peak) = (0, 0, 0, 0);
    let mut converged = false;
    let mut archive_path = options.linear_system_archive.as_deref();
    for iteration in 0..options.max_iterations {
        iterations = iteration + 1;
        let (system, linearization_cost) =
            build_system(snapshot, &state, &calib, &order, lambda, options)?;
        cost = linearization_cost;
        peak = peak.max(system.storage_bytes());
        let (increment, num_linear, linear_converged) = solve_lsqr(&system, options);
        linear_iterations += num_linear;
        log::info!(
            "Global BA iteration={iteration} cost={} damping={} linear_iterations={num_linear} linear_converged={linear_converged}",
            cost.as_f64(),
            lambda.as_f64()
        );
        if !linear_converged {
            system.save_archive_once(&mut archive_path, &increment, options.lsqr_tolerance)?;
            lambda *= multiplier;
            multiplier *= S::from_f64_lossy(2.0);
            if lambda > S::from_f64_lossy(options.maximum_damping) {
                break;
            }
            continue;
        }
        let (next, prediction) = apply_step(&state, &system, &order, &increment);
        // Landmarks are optimized even when every pose is fixed. A pose-only
        // norm would declare convergence after the first such back-substitution.
        let full_step_norm = next
            .points
            .iter()
            .fold(increment.amax(), |norm, (id, point)| {
                norm.max((*point - state.points[id]).amax())
            });
        let next_cost = trial_cost(snapshot, &next, &calib, &system, options);
        let decrease = cost - next_cost;
        let quality = decrease / prediction;
        log::info!(
            "Global BA trial iteration={iteration} cost={} prediction={} quality={}",
            next_cost.as_f64(),
            prediction.as_f64(),
            quality.as_f64()
        );
        if next_cost.is_finite() && prediction > S::zero() && quality > S::zero() {
            state = next;
            cost = next_cost;
            accepted += 1;
            lambda = (lambda
                * (S::one() / S::from_f64_lossy(3.0))
                    .max(S::one() - (S::from_f64_lossy(2.0) * quality - S::one()).powi(3)))
            .max(S::from_f64_lossy(options.minimum_damping));
            multiplier = S::from_f64_lossy(2.0);
            if full_step_norm < S::from_f64_lossy(options.step_tolerance)
                || decrease
                    <= S::from_f64_lossy(options.relative_cost_tolerance) * cost.abs().max(S::one())
            {
                converged = true;
                break;
            }
        } else if full_step_norm < S::from_f64_lossy(options.step_tolerance)
            && prediction.abs() < S::from_f64_lossy(options.relative_cost_tolerance)
        {
            converged = true;
            break;
        } else {
            lambda *= multiplier;
            multiplier *= S::from_f64_lossy(2.0);
            if lambda > S::from_f64_lossy(options.maximum_damping) {
                break;
            }
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

fn save_problem(
    snapshot: &MapSnapshot,
    calibration: &Calibration<f64>,
    options: &OptimizationOptions,
) -> Result<()> {
    if let Some(path) = &options.problem_archive {
        crate::global_problem::GlobalProblem::capture(snapshot, calibration, options)?
            .write_new(path)?;
    }
    Ok(())
}

/// # Errors
/// Rejects invalid optimizer settings, inconsistent camera calibration and
/// nonfinite map geometry or scalar-conversion overflow.
pub fn optimize_global(
    snapshot: &MapSnapshot,
    calibration: &Calibration<f64>,
    options: &OptimizationOptions,
) -> Result<OptimizationResult> {
    if options.linear_solver == LinearSolver::LsqrReference
        && options.damping_coordinates != DampingCoordinates::Physical
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "lsqr_reference requires physical damping coordinates",
        ));
    }
    match (options.precision, options.krylov_precision) {
        (Precision::Float32, KrylovPrecision::Native) => {
            log::info!("Global bundle adjustment uses float32 matrices and Krylov vectors");
        }
        (Precision::Float32, KrylovPrecision::Float64) => {
            log::warn!(
                "Global bundle adjustment uses float32 matrices with float64 Krylov vectors"
            );
        }
        (Precision::Float64, _) => {
            log::info!("Global bundle adjustment runs in double precision");
        }
    }
    match options.precision {
        Precision::Float32 => match options.linear_solver {
            LinearSolver::Rootba => rootba::optimize::<f32>(snapshot, calibration, options),
            LinearSolver::LsqrReference => optimize::<f32>(snapshot, calibration, options),
        },
        Precision::Float64 => match options.linear_solver {
            LinearSolver::Rootba => rootba::optimize::<f64>(snapshot, calibration, options),
            LinearSolver::LsqrReference => optimize::<f64>(snapshot, calibration, options),
        },
    }
}
/// # Errors
/// Rejects invalid pose-graph optimizer settings.
pub fn optimize_pose_graph(
    snapshot: &MapSnapshot,
    options: &OptimizationOptions,
) -> Result<OptimizationResult> {
    pose_graph::optimize(snapshot, options)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn compare_multiply_traversals<S: BasaltScalar, W: BasaltScalar>() {
        for damping in [0., -0., 0.5] {
            let mut blocks = Vec::new();
            for (columns, rows, coefficients) in [
                (vec![1, 0], 0, vec![]),
                (vec![], 3, vec![]),
                (
                    vec![2, 0, 2, 1],
                    7,
                    (0..28)
                        .map(|i| {
                            [
                                0.,
                                -0.,
                                1e20,
                                -1e20,
                                1e-20,
                                -1e-20,
                                f64::from(f32::from_bits(1)),
                            ][i % 7]
                        })
                        .collect(),
                ),
                (vec![1, 2], 1, vec![-0., 0.]),
            ] {
                blocks.push(RowBlock {
                    matrix: DMatrix::from_iterator(
                        rows,
                        columns.len(),
                        coefficients.into_iter().map(S::from_f64_lossy),
                    ),
                    residual: DVector::zeros(rows),
                    columns,
                });
            }
            let system = SquareRootSystem {
                columns: 3,
                blocks,
                landmarks: vec![],
                damping: S::from_f64_lossy(damping),
            };
            for values in [[-0., 0., 3.], [0., -0., -3.], [1e-20, -2., 1.]] {
                let input = DVector::from_iterator(3, values.into_iter().map(W::from_f64_lossy));
                let mut reference = DVector::<W>::zeros(system.rows());
                let mut offset = 0;
                for block in &system.blocks {
                    for row in 0..block.matrix.nrows() {
                        let mut sum = W::zero();
                        for (local, &global) in block.columns.iter().enumerate() {
                            sum += W::from_f64_lossy(block.matrix[(row, local)].as_f64())
                                * input[global];
                        }
                        reference[offset + row] = sum;
                    }
                    offset += block.matrix.nrows();
                }
                for i in 0..system.columns {
                    reference[offset + i] = W::from_f64_lossy(system.damping.as_f64()) * input[i];
                }
                let actual = system.multiply(&input);
                // Widening is injective for these finite f32 values and retains
                // the sign of zero; compare bits, not floating equality.
                assert!(
                    actual
                        .iter()
                        .zip(reference.iter())
                        .all(|(a, b)| a.as_f64().to_bits() == b.as_f64().to_bits())
                );
            }
        }
    }
    #[test]
    fn contiguous_multiply_preserves_bits_and_incidence_boundaries() {
        compare_multiply_traversals::<f32, f32>();
        compare_multiply_traversals::<f32, f64>();
        compare_multiply_traversals::<f64, f64>();
    }
    use crate::pose_graph::Matrix6;
    use crate::{
        map::{MapKeyframe, MapLandmark, MapObservation, ObservationKey},
        pose_graph::{EdgeKind, PoseGraphEdge},
    };
    use firefly_calibration::camera::generic_camera::GenericCamera;

    fn scene() -> (MapSnapshot, Calibration<f64>) {
        let mut calibration = Calibration::default();
        let mut camera = GenericCamera::from_name("pinhole");
        camera.set_from_init(Vector4::new(300.0, 310.0, 320.0, 240.0));
        calibration.intrinsics = vec![camera.clone(), camera];
        calibration.t_i_c = vec![Isometry3::identity(), Isometry3::translation(0.2, 0.0, 0.0)];
        let mut snapshot = MapSnapshot::default();
        for id in 0..3 {
            snapshot.keyframes.insert(
                id,
                MapKeyframe {
                    id,
                    t_w_b: Isometry3::translation(id as f64 * 0.3, 0.0, 0.0),
                    active: id == 0,
                    features: vec![],
                    images: None,
                },
            );
        }
        for id in 0..18 {
            let point = Vector3::new(
                (id % 6) as f64 * 0.3 - 0.7,
                (id / 6) as f64 * 0.3 - 0.3,
                3.0 + (id % 3) as f64 * 0.5,
            );
            let mut observations = BTreeMap::new();
            for (&time, keyframe) in &snapshot.keyframes {
                for camera in 0..2 {
                    let position = point
                        - keyframe.t_w_b.translation.vector
                        - calibration.t_i_c[camera].translation.vector;
                    observations.insert(
                        ObservationKey {
                            keyframe: time,
                            camera,
                            feature: id as usize,
                        },
                        MapObservation {
                            pixel: Vector2::new(
                                300.0 * position.x / position.z + 320.0,
                                310.0 * position.y / position.z + 240.0,
                            ),
                        },
                    );
                }
            }
            snapshot.landmarks.insert(
                id,
                MapLandmark {
                    id,
                    host: 0,
                    position: point + Vector3::new(0.02, -0.01, 0.03),
                    observations,
                },
            );
        }
        for id in 0..2 {
            snapshot.edges.push(PoseGraphEdge {
                from: id,
                to: id + 1,
                measurement: Isometry3::translation(0.3, 0.0, 0.0),
                kind: EdgeKind::Temporal,
                sqrt_information: Matrix6::identity() * 10.0,
            });
        }
        snapshot
            .keyframes
            .get_mut(&1)
            .unwrap()
            .t_w_b
            .translation
            .vector += Vector3::new(0.03, -0.02, 0.01);
        snapshot
            .keyframes
            .get_mut(&2)
            .unwrap()
            .t_w_b
            .translation
            .vector += Vector3::new(-0.02, 0.01, -0.02);
        (snapshot, calibration)
    }

    /// A scene whose initial residual is small.
    ///
    /// The relative gradient criterion of the damped normal equations then
    /// demands a step below the resolution of the working precision, which is
    /// the regime where a single-precision Krylov solve cannot converge.
    fn nearly_converged_scene() -> (MapSnapshot, Calibration<f64>) {
        let mut calibration = Calibration::default();
        let mut camera = GenericCamera::from_name("pinhole");
        camera.set_from_init(Vector4::new(300.0, 310.0, 320.0, 240.0));
        calibration.intrinsics = vec![camera.clone(), camera];
        calibration.t_i_c = vec![Isometry3::identity(), Isometry3::translation(0.2, 0.0, 0.0)];
        let mut snapshot = MapSnapshot::default();
        for id in 0..20_i64 {
            snapshot.keyframes.insert(
                id,
                MapKeyframe {
                    id,
                    t_w_b: Isometry3::translation(id as f64 * 0.3, 0.0, 0.0),
                    active: id == 0,
                    features: vec![],
                    images: None,
                },
            );
        }
        for id in 0..100_u64 {
            let point = Vector3::new(
                (id % 6) as f64 * 0.3 - 0.7,
                (id / 6) as f64 * 0.3 - 0.3,
                3.0 + (id % 3) as f64 * 0.5,
            );
            let host = (id % 20) as i64;
            let mut observations = BTreeMap::new();
            for keyframe in host..(host + 4).min(20) {
                for camera in 0..2 {
                    let position = point
                        - snapshot.keyframes[&keyframe].t_w_b.translation.vector
                        - calibration.t_i_c[camera].translation.vector;
                    observations.insert(
                        ObservationKey {
                            keyframe,
                            camera,
                            feature: id as usize,
                        },
                        MapObservation {
                            pixel: Vector2::new(
                                300.0 * position.x / position.z + 320.0,
                                310.0 * position.y / position.z + 240.0,
                            ),
                        },
                    );
                }
            }
            snapshot.landmarks.insert(
                id,
                MapLandmark {
                    id,
                    host,
                    position: point + Vector3::new(0.02, -0.01, 0.03) * 0.01,
                    observations,
                },
            );
        }
        for id in 0..19_i64 {
            snapshot.edges.push(PoseGraphEdge {
                from: id,
                to: id + 1,
                measurement: Isometry3::translation(0.3, 0.0, 0.0),
                kind: EdgeKind::Temporal,
                sqrt_information: Matrix6::identity() * 60.0,
            });
        }
        (snapshot, calibration)
    }

    #[test]
    fn the_krylov_tolerance_follows_the_working_precision() {
        // Double-precision work vectors resolve the requested criterion.
        assert_eq!(effective_lsqr_tolerance::<f64>(1e-5), 1e-5);
        // Single-precision work vectors cannot resolve it, so the criterion is
        // floored at the resolution of the working precision.
        assert_eq!(
            effective_lsqr_tolerance::<f32>(1e-5),
            f64::from(1e-5_f32.sqrt())
        );
        // An already loose criterion is not tightened.
        assert_eq!(effective_lsqr_tolerance::<f32>(1e-2), 1e-2);
    }

    #[test]
    fn single_precision_does_not_spend_the_krylov_budget_on_an_unreachable_criterion() {
        let (snapshot, calibration) = nearly_converged_scene();
        let options = OptimizationOptions {
            precision: Precision::Float32,
            krylov_precision: KrylovPrecision::Native,
            lsqr_tolerance: 1e-5,
            max_iterations: 12,
            ..Default::default()
        };
        let result = optimize_global(&snapshot, &calibration, &options).unwrap();
        // Demanding the requested criterion spends the whole Krylov budget on an
        // unreachable target and leaves the outer loop without a usable step.
        assert!(
            result.linear_iterations < 1_000,
            "single precision spent {} Krylov iterations",
            result.linear_iterations
        );
        assert!(
            result.final_cost < result.initial_cost,
            "single precision did not reduce the cost"
        );
    }

    #[test]
    fn rig_projection_jacobians_match_numeric() {
        let (_, calib) = scene();
        let pose = pose_graph::se3_exp(&pose_graph::Vector6::new(0.1, -0.2, 0.05, 0.1, -0.2, 0.1));
        let point = Vector3::new(0.2, -0.3, 4.0);
        let extrinsic = pose_graph::inverse(&calib.t_i_c[1]);
        let pixel = Vector2::new(320.0, 240.0);
        let (_, jp, jl) =
            camera_factor(&pose, &point, &extrinsic, &calib.intrinsics[1], &pixel).unwrap();
        for axis in 0..6 {
            let mut d = pose_graph::Vector6::zeros();
            d[axis] = 1e-6;
            let plus = pose_graph::product(&pose_graph::se3_exp(&d), &pose);
            let minus = pose_graph::product(&pose_graph::se3_exp(&-d), &pose);
            let numeric = (camera_factor(&plus, &point, &extrinsic, &calib.intrinsics[1], &pixel)
                .unwrap()
                .0
                - camera_factor(&minus, &point, &extrinsic, &calib.intrinsics[1], &pixel)
                    .unwrap()
                    .0)
                / 2e-6;
            assert!((numeric - jp.column(axis)).norm() < 2e-7);
        }
        for axis in 0..3 {
            let mut d = Vector3::zeros();
            d[axis] = 1e-6;
            let numeric = (camera_factor(
                &pose,
                &(point + d),
                &extrinsic,
                &calib.intrinsics[1],
                &pixel,
            )
            .unwrap()
            .0 - camera_factor(
                &pose,
                &(point - d),
                &extrinsic,
                &calib.intrinsics[1],
                &pixel,
            )
            .unwrap()
            .0) / 2e-6;
            assert!((numeric - jl.column(axis)).norm() < 2e-7);
        }
    }

    #[test]
    fn zero_step_cost_matches_linearization_with_many_small_graph_terms() {
        let (mut snapshot, calibration) = scene();
        let mut edge = snapshot.edges[0].clone();
        edge.sqrt_information *= 0.001;
        snapshot.edges = vec![edge; 5000];
        let state = state_from_snapshot::<f32>(&snapshot).unwrap();
        let calibration = calibration.cast::<f32>();
        let order = pose_order(&snapshot);
        let options = OptimizationOptions::default();
        let (system, cost) =
            build_system(&snapshot, &state, &calibration, &order, 1e-3, &options).unwrap();
        let unchanged = trial_cost(&snapshot, &state, &calibration, &system, &options);
        assert_eq!(
            cost, unchanged,
            "Identical states must have identical LM cost"
        );
        let without_graph = OptimizationOptions {
            pose_graph_terms: false,
            ..options
        };
        let visual_cost = trial_cost(&snapshot, &state, &calibration, &system, &without_graph);
        assert!(
            cost > visual_cost,
            "Small pose-graph terms must survive accumulation"
        );
    }

    #[test]
    fn global_square_root_float_and_double_reduce_same_scene() {
        let (snapshot, calibration) = scene();
        for precision in [Precision::Float32, Precision::Float64] {
            for krylov_precision in [KrylovPrecision::Native, KrylovPrecision::Float64] {
                let options = OptimizationOptions {
                    precision,
                    krylov_precision,
                    lsqr_tolerance: if precision == Precision::Float32 {
                        1e-5
                    } else {
                        1e-10
                    },
                    max_iterations: 30,
                    ..Default::default()
                };
                let result = optimize_global(&snapshot, &calibration, &options).unwrap();
                assert!(
                    result.final_cost < result.initial_cost * 1e-4,
                    "{precision:?}: {} -> {}",
                    result.initial_cost,
                    result.final_cost
                );
                for id in 0..3 {
                    assert!(
                        (result.poses[&id].translation.vector
                            - Vector3::new(id as f64 * 0.3, 0.0, 0.0))
                        .norm()
                            < 1e-3
                    );
                }
            }
        }
    }

    #[test]
    fn covisibility_edges_do_not_enter_the_global_pose_graph_term() {
        let (mut snapshot, calibration) = scene();
        let options = OptimizationOptions {
            pose_graph_terms: true,
            ..Default::default()
        };
        let plain = optimize_global(&snapshot, &calibration, &options).unwrap();
        // A covisibility edge restates the current relative estimate, so it
        // carries no measurement. If it entered Equation (9) it would add
        // stiffness that pins the map to its present shape.
        snapshot.edges.push(PoseGraphEdge {
            from: 0,
            to: 2,
            measurement: Isometry3::translation(5., 0., 0.),
            kind: EdgeKind::Covisibility,
            sqrt_information: nalgebra::SMatrix::identity(),
        });
        let with_covisibility = optimize_global(&snapshot, &calibration, &options).unwrap();
        assert_eq!(plain.initial_cost, with_covisibility.initial_cost);
        assert_eq!(plain.final_cost, with_covisibility.final_cost);
        for id in 0..3 {
            assert_eq!(plain.poses[&id], with_covisibility.poses[&id]);
        }
    }
    #[test]
    fn single_precision_and_double_precision_agree_on_the_paper_scene() {
        let (snapshot, calibration) = scene();
        let mut reference: Vec<Vector3<f64>> = Vec::new();
        for (precision, krylov) in [
            (Precision::Float64, KrylovPrecision::Native),
            (Precision::Float32, KrylovPrecision::Native),
        ] {
            let options = OptimizationOptions {
                precision,
                krylov_precision: krylov,
                lsqr_tolerance: if precision == Precision::Float32 {
                    1e-5
                } else {
                    1e-10
                },
                max_iterations: 30,
                ..Default::default()
            };
            let result = optimize_global(&snapshot, &calibration, &options).unwrap();
            assert_eq!(result.precision, precision);
            assert_eq!(result.krylov_precision, krylov);
            assert!(
                result.final_cost < result.initial_cost * 1e-4,
                "{precision:?}: {} -> {}",
                result.initial_cost,
                result.final_cost
            );
            let poses: Vec<Vector3<f64>> = (0..3)
                .map(|id| result.poses[&id].translation.vector)
                .collect();
            if reference.is_empty() {
                reference = poses;
            } else {
                for (id, (expected, actual)) in reference.iter().zip(poses.iter()).enumerate() {
                    assert!(
                        (expected - actual).norm() < 1e-3,
                        "single and double precision disagree for keyframe {id}"
                    );
                }
            }
        }
    }
    #[test]
    fn the_edge_residual_is_the_papers_left_relative_error() {
        // Equation (8) is e_ab = log(T_bar_ab * T_hat_ab^-1) with
        // T_hat_ab = T_aw * T_wb. The residual lives in the world-to-body
        // chart, so its translation part is the measurement minus the current
        // relative estimate and its rotation part is the relative rotation
        // error.
        let (mut snapshot, _calibration) = scene();
        for edge in &mut snapshot.edges {
            edge.measurement *= Isometry3::translation(0.1, 0., 0.);
            edge.sqrt_information = Matrix6::identity();
        }
        let state = state_from_snapshot::<f64>(&snapshot).unwrap();
        let edge = &snapshot.edges[0];
        let (residual, _, _) = pose_graph::linearize_edge(
            state.poses.get(&edge.from).unwrap(),
            state.poses.get(&edge.to).unwrap(),
            &pose_graph::cast_pose(&edge.measurement),
            &edge.sqrt_information,
        );
        // Keyframe 1 sits at (0.33, -0.02, 0.01) while the measurement claims
        // (0.40, 0, 0), and the scene carries no rotation, so the residual is
        // exactly their difference.
        let expected = crate::pose_graph::Vector6::new(0.07, 0.02, -0.01, 0., 0., 0.);
        assert!(
            (residual - expected).norm() < 1e-12,
            "residual {residual:?} differs from {expected:?}"
        );
    }
    #[test]
    fn the_pose_graph_term_scales_with_the_edge_information() {
        let (mut snapshot, calibration) = scene();
        // Move the odometry measurements away from the current estimates so
        // that the pose-graph term is not zero at the initial state.
        for edge in &mut snapshot.edges {
            edge.measurement *= Isometry3::translation(0.1, 0., 0.);
            edge.sqrt_information = Matrix6::identity();
        }
        let visual = OptimizationOptions {
            pose_graph_terms: false,
            ..Default::default()
        };
        let plain = OptimizationOptions {
            pose_graph_terms: true,
            ..Default::default()
        };
        let visual_cost = optimize_global(&snapshot, &calibration, &visual)
            .unwrap()
            .initial_cost;
        let plain_cost = optimize_global(&snapshot, &calibration, &plain)
            .unwrap()
            .initial_cost;
        let mut scaled = snapshot.clone();
        for edge in &mut scaled.edges {
            edge.sqrt_information = Matrix6::identity() * 10.0;
        }
        let scaled_cost = optimize_global(&scaled, &calibration, &plain)
            .unwrap()
            .initial_cost;
        let plain_graph = plain_cost - visual_cost;
        let scaled_graph = scaled_cost - visual_cost;
        assert!(plain_graph > 0.0);
        // The square root of the information scales the weighted residual by
        // ten, so its square scales the pose-graph term by a hundred.
        assert!(
            (scaled_graph - 100.0 * plain_graph).abs() < 1e-9 * scaled_graph,
            "plain graph {plain_graph}, scaled graph {scaled_graph}"
        );
    }
    #[test]
    fn landmark_qr_and_lsqr_match_independent_full_damped_svd() {
        let (snapshot, calibration) = scene();
        let state = State {
            poses: snapshot
                .keyframes
                .iter()
                .map(|(&id, k)| (id, pose_graph::inverse(&k.t_w_b)))
                .collect(),
            points: snapshot
                .landmarks
                .iter()
                .map(|(&id, p)| (id, p.position))
                .collect(),
        };
        let order = BTreeMap::from([(1, 0), (2, 6)]);
        let options = OptimizationOptions {
            precision: Precision::Float64,
            lsqr_tolerance: 1e-12,
            lsqr_max_iterations: 2000,
            ..Default::default()
        };
        for lambda in [1e-6, 1e-2, 10.0] {
            let (system, _) =
                build_system(&snapshot, &state, &calibration, &order, lambda, &options).unwrap();
            let (increment, _, converged) = solve_lsqr(&system, &options);
            assert!(converged);
            let (next, _) = apply_step(&state, &system, &order, &increment);
            let pose_columns = order.len() * 6;
            let columns = pose_columns + 3 * system.landmarks.len();
            let observation_rows = system
                .landmarks
                .iter()
                .map(|b| b.original_residual.len())
                .sum::<usize>();
            let graph_rows = system
                .blocks
                .iter()
                .skip(system.landmarks.len())
                .map(|b| b.residual.len())
                .sum::<usize>();
            let mut full = DMatrix::<f64>::zeros(observation_rows + graph_rows + columns, columns);
            let mut rhs = DVector::zeros(full.nrows());
            let mut row = 0;
            for (point, block) in system.landmarks.iter().enumerate() {
                let rows = block.original_residual.len();
                full.view_mut((row, pose_columns + point * 3), (rows, 3))
                    .copy_from(&block.original_jacobian.columns(0, 3));
                for (local, &global) in block.columns.iter().enumerate() {
                    full.view_mut((row, global), (rows, 1))
                        .copy_from(&block.original_jacobian.column(local + 3));
                }
                rhs.rows_mut(row, rows)
                    .copy_from(&(-&block.original_residual));
                row += rows;
            }
            for block in system.blocks.iter().skip(system.landmarks.len()) {
                let rows = block.residual.len();
                for (local, &global) in block.columns.iter().enumerate() {
                    full.view_mut((row, global), (rows, 1))
                        .copy_from(&block.matrix.column(local));
                }
                rhs.rows_mut(row, rows).copy_from(&(-&block.residual));
                row += rows;
            }
            for i in 0..columns {
                full[(row + i, i)] = lambda.sqrt();
            }
            let reference = full.svd(true, true).solve(&rhs, 1e-12).unwrap();
            assert!(
                (&increment - reference.rows(0, pose_columns)).norm() < 1e-8,
                "pose damping {lambda}"
            );
            for (point, block) in system.landmarks.iter().enumerate() {
                let actual = next.points[&block.id] - state.points[&block.id];
                assert!(
                    (actual - reference.fixed_rows::<3>(pose_columns + point * 3)).norm() < 1e-8,
                    "landmark damping {lambda}"
                );
            }
        }
    }

    #[test]
    #[ignore = "requires ROOTSLAM_LINEAR_SYSTEM_ARCHIVE from an actual failed solve"]
    fn frozen_single_precision_system_meets_unchanged_gradient_criterion() {
        let path = std::env::var_os("ROOTSLAM_LINEAR_SYSTEM_ARCHIVE").unwrap();
        let archive: serde_json::Value =
            serde_json::from_reader(std::io::BufReader::new(std::fs::File::open(path).unwrap()))
                .unwrap();
        let integer = |v: &serde_json::Value| usize::try_from(v.as_u64().unwrap()).unwrap();
        #[expect(
            clippy::cast_possible_truncation,
            reason = "Restore explicitly quantized float fixture values"
        )]
        let scalar = |v: &serde_json::Value| v.as_f64().unwrap() as f32;
        assert_eq!(archive["scalar_bytes"], 4);
        let system = SquareRootSystem {
            columns: integer(&archive["columns"]),
            blocks: archive["blocks"]
                .as_array()
                .unwrap()
                .iter()
                .map(|block| {
                    let columns: Vec<_> = block["columns"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(integer)
                        .collect();
                    let values: Vec<_> = block["matrix_column_major"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(scalar)
                        .collect();
                    RowBlock {
                        matrix: DMatrix::from_column_slice(
                            integer(&block["rows"]),
                            columns.len(),
                            &values,
                        ),
                        columns,
                        residual: DVector::from_vec(
                            block["residual"]
                                .as_array()
                                .unwrap()
                                .iter()
                                .map(scalar)
                                .collect(),
                        ),
                    }
                })
                .collect(),
            landmarks: Vec::new(),
            damping: scalar(&archive["damping"]),
        };
        let options = OptimizationOptions {
            lsqr_tolerance: archive["relative_gradient_tolerance"].as_f64().unwrap(),
            ..Default::default()
        };
        let started = std::time::Instant::now();
        let (step, iterations, converged) = solve_lsqr(&system, &options);
        let initial_gradient = system.physical_gradient_norm(None);
        let gradient = system.physical_gradient_norm(Some(&step));
        let output = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../logs/rootslam_linear_system_production.json");
        serde_json::to_writer_pretty(std::fs::File::create(output).unwrap(), &serde_json::json!({
            "iterations":iterations,"converged":converged,"gradient":gradient,
            "threshold":options.lsqr_tolerance*initial_gradient,"seconds":started.elapsed().as_secs_f64()
        })).unwrap();
        assert!(
            converged && gradient <= options.lsqr_tolerance * initial_gradient,
            "iterations={iterations}, gradient={gradient}, threshold={}",
            options.lsqr_tolerance * initial_gradient
        );
    }

    #[test]
    fn lsqr_checks_convergence_at_the_iteration_limit() {
        let system = SquareRootSystem {
            columns: 2,
            blocks: vec![RowBlock {
                columns: vec![0, 1],
                matrix: DMatrix::from_row_slice(3, 2, &[1.0_f64, 2.0, 3.0, 4.0, 5.0, 7.0]),
                residual: DVector::from_vec(vec![0.2, -0.3, 0.7]),
            }],
            landmarks: Vec::new(),
            damping: 0.1,
        };
        let options = OptimizationOptions {
            lsqr_tolerance: 1e-10,
            lsqr_max_iterations: 3,
            ..Default::default()
        };
        let (step, iterations, converged) = solve_lsqr(&system, &options);
        // Independently expand AᵀA + 0.01 I and Aᵀr for this small fixture.
        let hessian = nalgebra::Matrix2::new(35.01, 49.0, 49.0, 69.01);
        let gradient = Vector2::new(2.8, 4.1);
        let expected = Vector2::new(
            (-69.01 * 2.8 + 49.0 * 4.1) / (35.01 * 69.01 - 49.0 * 49.0),
            (49.0 * 2.8 - 35.01 * 4.1) / (35.01 * 69.01 - 49.0 * 49.0),
        );
        assert!((step.fixed_rows::<2>(0) - expected).norm() < 1e-12);
        let relative_gradient =
            (hessian * step.fixed_rows::<2>(0) + gradient).norm() / gradient.norm();
        assert!(relative_gradient <= options.lsqr_tolerance);
        assert!(
            converged,
            "iteration {iterations}: relative gradient {relative_gradient} satisfies the tolerance"
        );

        let (unfinished, _, converged) = solve_lsqr(
            &system,
            &OptimizationOptions {
                lsqr_max_iterations: 1,
                ..options
            },
        );
        assert!(
            (hessian * unfinished.fixed_rows::<2>(0) + gradient).norm()
                > options.lsqr_tolerance * gradient.norm()
        );
        assert!(!converged);
    }

    #[test]
    fn column_preconditioning_preserves_physical_damping_across_scales() {
        let columns = 24;
        let diagonal = DVector::from_fn(columns, |i, _| {
            10_f64.powf(-4.0 + 8.0 * f64::from(u32::try_from(i).unwrap()) / 23.0)
        });
        let matrix = DMatrix::from_diagonal(&diagonal);
        let residual = DVector::from_fn(columns, |i, _| {
            0.2 + f64::from(u32::try_from(i).unwrap()) * 0.1
        });
        for lambda in [1e-8_f64, 1e-2, 10.0] {
            let system = SquareRootSystem {
                columns,
                blocks: vec![RowBlock {
                    columns: (0..columns).collect(),
                    matrix: matrix.clone(),
                    residual: residual.clone(),
                }],
                landmarks: Vec::new(),
                damping: lambda.sqrt(),
            };
            let options = OptimizationOptions {
                lsqr_tolerance: 1e-12,
                lsqr_max_iterations: 32,
                ..Default::default()
            };
            let (step, iterations, converged) = solve_lsqr(&system, &options);
            assert!(converged);
            assert!(
                iterations <= 8,
                "column-normalized diagonal system: {iterations} iterations"
            );
            for i in 0..columns {
                let expected = -diagonal[i] * residual[i] / (diagonal[i] * diagonal[i] + lambda);
                assert!((step[i] - expected).abs() < 1e-10 * (1.0 + expected.abs()));
            }
            let gradient = system
                .transpose_multiply(&(system.multiply(&step) - system.rhs()))
                .norm();
            assert!(
                gradient
                    <= options.lsqr_tolerance * system.transpose_multiply(&system.rhs()).norm()
            );
        }
    }

    #[test]
    fn column_norm_scaling_handles_single_precision_extremes_and_empty_columns() {
        let system = SquareRootSystem {
            columns: 4,
            blocks: vec![RowBlock {
                columns: vec![0, 1, 3],
                matrix: DMatrix::from_row_slice(1, 3, &[1e20_f32, 1e-20, 1e-40]),
                residual: DVector::zeros(1),
            }],
            landmarks: Vec::new(),
            damping: 0.0,
        };
        let scaling = system.inverse_column_norms();
        assert!((scaling[0] / 1e-20 - 1.0).abs() < 1e-6);
        assert!((scaling[1] / 1e20 - 1.0).abs() < 1e-6);
        assert_eq!(scaling[2], 1.0);
        assert_eq!(scaling[3], 1.0);
    }

    #[test]
    fn pgo_holds_active_window_fixed_and_corrects_older_poses() {
        let (mut snapshot, _) = scene();
        snapshot.keyframes.get_mut(&0).unwrap().active = false;
        snapshot.keyframes.get_mut(&2).unwrap().active = true;
        let fixed = snapshot.keyframes[&2].t_w_b;
        let result = optimize_pose_graph(&snapshot, &OptimizationOptions::default()).unwrap();
        assert_eq!(result.poses[&2], fixed);
        assert!(result.final_cost < 1e-12);
        assert!(result.points.is_empty());
    }
}
