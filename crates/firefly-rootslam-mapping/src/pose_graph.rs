//! `RootSLAM` equations (1), (8), and (10). Measurements map body b into body a.
use firefly_basalt::utils::sophus_utils::left_jacobian_inv_so3;
use firefly_basalt::utils::sophus_utils::left_jacobian_so3;
use firefly_basalt::utils::sophus_utils::so3_exp;
use firefly_basalt::utils::sophus_utils::so3_inverse;
use firefly_basalt::utils::sophus_utils::so3_log;
use firefly_basalt::utils::sophus_utils::so3_product;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, Matrix3, SMatrix, SVector, Translation3, Vector3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeKind {
    Temporal,
    Covisibility,
    Loop,
    SpanningTree,
}
#[derive(Clone, Debug)]
pub struct PoseGraphEdge {
    pub from: i64,
    pub to: i64,
    pub measurement: Isometry3<f64>,
    pub kind: EdgeKind,
    /// Paper `S_ab`; its numerical values are not supplied in the paper.
    pub sqrt_information: SMatrix<f64, 6, 6>,
}
pub type Vector6<S> = SVector<S, 6>;
pub type Matrix6<S> = SMatrix<S, 6, 6>;

pub(crate) fn hat<S: BasaltScalar>(v: &Vector3<S>) -> Matrix3<S> {
    Matrix3::new(
        S::zero(),
        -v.z,
        v.y,
        v.z,
        S::zero(),
        -v.x,
        -v.y,
        v.x,
        S::zero(),
    )
}
pub(crate) fn product<S: BasaltScalar>(a: &Isometry3<S>, b: &Isometry3<S>) -> Isometry3<S> {
    Isometry3::from_parts(
        Translation3::from(a.translation.vector + a.rotation * b.translation.vector),
        so3_product(&a.rotation, &b.rotation),
    )
}
pub(crate) fn inverse<S: BasaltScalar>(pose: &Isometry3<S>) -> Isometry3<S> {
    let rotation = so3_inverse(&pose.rotation);
    Isometry3::from_parts(
        Translation3::from(-(rotation * pose.translation.vector)),
        rotation,
    )
}
pub(crate) fn cast_pose<S: BasaltScalar, T: BasaltScalar>(pose: &Isometry3<S>) -> Isometry3<T> {
    firefly_basalt::imu::imu_types::PoseState::new(0, *pose)
        .cast::<T>()
        .t_w_i
}
pub fn se3_exp<S: BasaltScalar>(tangent: &Vector6<S>) -> Isometry3<S> {
    let omega = tangent.fixed_rows::<3>(3).into_owned();
    let translation = left_jacobian_so3(&omega) * tangent.fixed_rows::<3>(0);
    Isometry3::from_parts(Translation3::from(translation), so3_exp(&omega))
}
pub fn se3_log<S: BasaltScalar>(pose: &Isometry3<S>) -> Vector6<S> {
    let omega = so3_log(&pose.rotation);
    let translation = left_jacobian_inv_so3(&omega) * pose.translation.vector;
    Vector6::new(
        translation.x,
        translation.y,
        translation.z,
        omega.x,
        omega.y,
        omega.z,
    )
}
pub fn adjoint<S: BasaltScalar>(pose: &Isometry3<S>) -> Matrix6<S> {
    let rotation = pose.rotation.to_rotation_matrix().into_inner();
    let mut out = Matrix6::zeros();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
    out.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
    out.fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(hat(&pose.translation.vector) * rotation));
    out
}

/// Coupled SE(3) inverse right Jacobian (not Basalt's decoupled pose Jacobian).
/// Upper-right formula follows Sophus SE3, copyright Hauke Strasdat and
/// contributors, MIT licensed. Small-angle series avoids coefficient cancellation.
pub fn right_jacobian_inverse<S: BasaltScalar>(tangent: &Vector6<S>) -> Matrix6<S> {
    let upsilon = -tangent.fixed_rows::<3>(0).into_owned();
    let omega = -tangent.fixed_rows::<3>(3).into_owned();
    let theta2 = omega.norm_squared();
    let u = hat(&upsilon);
    let w = hat(&omega);
    let half = S::from_f64_lossy(0.5);
    let q = if theta2 < S::from_f64_lossy(0.25) {
        // J_l = sum ad(x)^n/(n+1)!; this converges rapidly near zero and
        // retains translation/rotation coupling even at vanishing rotation.
        // At |omega| <= 1/2, sixteen terms bound the omitted coupling by
        // 2*|upsilon|*(1/2)^16/17!, below 1e-19*|upsilon|. Evaluating the
        // blocks directly avoids cancellation between the 1/theta^4 terms.
        let mut diagonal = Matrix3::<S>::identity();
        let mut coupling = Matrix3::<S>::zeros();
        let mut sum = Matrix3::<S>::zeros();
        for n in 1..=16 {
            let divisor = S::from_f64_lossy(f64::from(n + 1));
            coupling = (diagonal * u + coupling * w) / divisor;
            diagonal = (diagonal * w) / divisor;
            sum += coupling;
        }
        sum
    } else {
        let theta = theta2.sqrt();
        let inverse = S::one() / theta;
        let inverse2 = inverse * inverse;
        let inverse4 = inverse2 * inverse2;
        let sine = theta.sin();
        let cosine = theta.cos();
        let c1 = inverse2 - sine * inverse2 * inverse;
        let c2 = half * inverse2 + cosine * inverse4 - inverse4;
        let c3 = inverse4 + half * cosine * inverse4
            - S::from_f64_lossy(1.5) * sine * inverse * inverse4;
        let wu = w * u;
        let wuw = wu * w;
        u * half + (wu + u * w + wuw) * c1 - (u * theta2 + wuw * S::from_f64_lossy(2.0)) * c2
            + (wuw * w + w * wuw) * c3
    };
    let rotation_inverse = left_jacobian_inv_so3(&omega);
    let mut result = Matrix6::zeros();
    result
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&rotation_inverse);
    result
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&rotation_inverse);
    result
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(-rotation_inverse * q * rotation_inverse));
    result
}

/// Eq. (8)/(10), left increments on world-to-body poses.
pub fn linearize_edge<S: BasaltScalar>(
    first: &Isometry3<S>,
    second: &Isometry3<S>,
    measurement: &Isometry3<S>,
    sqrt_information: &Matrix6<S>,
) -> (Vector6<S>, Matrix6<S>, Matrix6<S>) {
    let predicted = product(first, &inverse(second));
    let error = se3_log(&product(measurement, &inverse(&predicted)));
    let right = sqrt_information * right_jacobian_inverse(&error);
    (
        sqrt_information * error,
        -right,
        right * adjoint(&predicted),
    )
}

/// `QuaternionManifold` × Euclidean translation, with body-to-world poses.
/// Ceres' quaternion tangent has half the physical SO(3) rotation angle.
fn manifold_increment(pose: &Isometry3<f64>, increment: &Vector6<f64>) -> Isometry3<f64> {
    let rotation = so3_product(
        &so3_exp(&(increment.fixed_rows::<3>(3) * 2.0)),
        &pose.rotation,
    );
    Isometry3::from_parts(
        Translation3::from(pose.translation.vector + increment.fixed_rows::<3>(0)),
        rotation,
    )
}
fn manifold_to_inverse_left(pose: &Isometry3<f64>) -> Matrix6<f64> {
    let mut mapping = Matrix6::identity();
    mapping
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(hat(&pose.translation.vector) * 2.0));
    mapping.fixed_view_mut::<3, 3>(3, 3).scale_mut(2.0);
    -adjoint(&inverse(pose)) * mapping
}

/// Equation (1), using the quaternion-manifold and additive translation chart.
#[must_use]
pub fn linearize_pose_graph_edge(
    first: &Isometry3<f64>,
    second: &Isometry3<f64>,
    measurement: &Isometry3<f64>,
) -> (Vector6<f64>, Matrix6<f64>, Matrix6<f64>) {
    let predicted = product(&inverse(first), second);
    let inverse_predicted = inverse(&predicted);
    let residual = se3_log(&product(&inverse_predicted, measurement));
    // E = P^-1 M. In the inverse-pose left charts,
    // dE_left = db - Ad(P^-1) da; J_l^-1(e) = J_r^-1(-e).
    // Equation (1) is a right relative error, whereas equation (8) is a
    // left relative error. Negating the latter does not convert between them.
    let left_inverse = right_jacobian_inverse(&-residual);
    (
        residual,
        -left_inverse * adjoint(&inverse_predicted) * manifold_to_inverse_left(first),
        left_inverse * manifold_to_inverse_left(second),
    )
}

type PoseGraphFactor = (Vector6<f64>, Vec<(usize, Matrix6<f64>)>);
struct LinearizedPoseGraph {
    accumulator: firefly_basalt::optimization::accumulator::SparseHashAccumulator<f64>,
    factors: Vec<PoseGraphFactor>,
}
fn linearize_graph(
    snapshot: &crate::map::MapSnapshot,
    poses: &std::collections::BTreeMap<i64, Isometry3<f64>>,
    order: &std::collections::BTreeMap<i64, usize>,
) -> LinearizedPoseGraph {
    use firefly_basalt::optimization::accumulator::SparseHashAccumulator;
    let mut accumulator = SparseHashAccumulator::<f64>::default();
    accumulator.reset(order.len() * 6);
    let mut factors = Vec::new();
    for edge in &snapshot.edges {
        let (Some(a), Some(b)) = (poses.get(&edge.from), poses.get(&edge.to)) else {
            continue;
        };
        let (r, ja, jb) = linearize_pose_graph_edge(a, b, &edge.measurement);
        let mut jacobians = Vec::new();
        for (time, j) in [(edge.from, ja), (edge.to, jb)] {
            if let Some(&offset) = order.get(&time) {
                accumulator.add_h(offset, offset, &(j.transpose() * j));
                accumulator.add_b(offset, &(j.transpose() * r));
                jacobians.push((offset, j));
            }
        }
        if jacobians.len() == 2 {
            let (a, ja) = &jacobians[0];
            let (b, jb) = &jacobians[1];
            accumulator.add_h(*a, *b, &(ja.transpose() * jb));
            accumulator.add_h(*b, *a, &(jb.transpose() * ja));
        }
        factors.push((r, jacobians));
    }
    accumulator.setup_solver();
    LinearizedPoseGraph {
        accumulator,
        factors,
    }
}
fn graph_cost(
    snapshot: &crate::map::MapSnapshot,
    poses: &std::collections::BTreeMap<i64, Isometry3<f64>>,
) -> f64 {
    snapshot
        .edges
        .iter()
        .filter_map(|edge| {
            Some(
                0.5 * linearize_pose_graph_edge(
                    poses.get(&edge.from)?,
                    poses.get(&edge.to)?,
                    &edge.measurement,
                )
                .0
                .norm_squared(),
            )
        })
        .sum()
}

/// Paper loop-closing PGO: body-to-world quaternion manifold, LM, direct sparse
/// Cholesky-family LDLT, active local-window keyframes fixed. Ceres itself is
/// replaced by the equivalent Rust factor assembly and sparse factorization.
/// # Errors
/// Rejects invalid optimizer damping, tolerances or iteration limits.
pub fn optimize(
    snapshot: &crate::map::MapSnapshot,
    options: &crate::global_optimization::OptimizationOptions,
) -> firefly_error::Result<crate::global_optimization::OptimizationResult> {
    use crate::global_optimization::OptimizationResult;
    use std::collections::BTreeMap;
    options.validate()?;
    let mut poses: BTreeMap<_, _> = snapshot
        .keyframes
        .iter()
        .map(|(&id, k)| (id, k.t_w_b))
        .collect();
    let any_fixed = snapshot.keyframes.values().any(|k| k.active);
    let first = snapshot.keyframes.keys().next().copied();
    let order: BTreeMap<_, _> = snapshot
        .keyframes
        .iter()
        .filter(|(id, k)| !k.active && (any_fixed || Some(**id) != first))
        .enumerate()
        .map(|(i, (&id, _))| (id, i * 6))
        .collect();
    let initial_cost = graph_cost(snapshot, &poses);
    let mut current = initial_cost;
    let mut lambda = options.initial_damping;
    let mut multiplier = 2.0;
    let mut iterations = 0;
    let mut accepted = 0;
    let mut converged = order.is_empty();
    let mut peak = 0;
    for iteration in 0..options.max_iterations {
        if converged {
            break;
        }
        iterations = iteration + 1;
        let LinearizedPoseGraph {
            accumulator,
            factors,
        } = linearize_graph(snapshot, &poses, &order);
        peak = peak
            .max((snapshot.edges.len() * 4 * 36 + order.len() * 6) * std::mem::size_of::<f64>());
        let damping = accumulator
            .h_diagonal()
            .map(|v| (lambda * v).max(options.minimum_damping));
        let increment = -accumulator.solve(Some(&damping));
        if !increment.iter().all(|v| v.is_finite()) {
            lambda *= multiplier;
            multiplier *= 2.0;
            if lambda > options.maximum_damping {
                break;
            }
            continue;
        }
        let mut candidate = poses.clone();
        for (&id, &offset) in &order {
            candidate.insert(
                id,
                manifold_increment(&poses[&id], &increment.fixed_rows::<6>(offset).into_owned()),
            );
        }
        let next = graph_cost(snapshot, &candidate);
        let mut predicted = 0.0;
        for (residual, jacobians) in &factors {
            let mut change = Vector6::zeros();
            for (offset, j) in jacobians {
                change += j * increment.fixed_rows::<6>(*offset);
            }
            predicted -= change.dot(&(residual + change * 0.5));
        }
        let decrease = current - next;
        let quality = decrease / predicted;
        if predicted > 0.0 && quality > 0.0 {
            poses = candidate;
            current = next;
            accepted += 1;
            lambda = (lambda * (1.0 / 3.0_f64).max(1.0 - (2.0 * quality - 1.0).powi(3)))
                .max(options.minimum_damping);
            multiplier = 2.0;
            converged = increment.amax() < options.step_tolerance
                || decrease <= options.relative_cost_tolerance * current.abs().max(1.0);
        } else if increment.amax() < options.step_tolerance
            && predicted.abs() < options.relative_cost_tolerance
        {
            converged = true;
        } else {
            lambda *= multiplier;
            multiplier *= 2.0;
            if lambda > options.maximum_damping {
                break;
            }
        }
    }
    Ok(OptimizationResult {
        revision: snapshot.revision,
        poses,
        points: BTreeMap::new(),
        initial_cost,
        final_cost: current,
        iterations,
        accepted_steps: accepted,
        converged,
        linear_iterations: iterations,
        peak_linear_storage_bytes: peak,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn paper_jacobians_match_full_se3_left_perturbations() {
        for angle in [0.0, 1e-7, 0.3, 2.4] {
            let a = se3_exp(&Vector6::new(0.4, -0.2, 0.1, angle, 0.1, -0.2));
            let b = se3_exp(&Vector6::new(-0.2, 0.3, 0.4, -0.1, 0.2, 0.1));
            let measurement = se3_exp(&Vector6::new(0.1, 0.0, 0.2, 0.05, -0.1, 0.1));
            let information = Matrix6::from_diagonal(&Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
            let (_, ja, jb) = linearize_edge(&a, &b, &measurement, &information);
            for axis in 0..6 {
                let mut delta = Vector6::zeros();
                delta[axis] = 1e-6;
                let da = (linearize_edge(
                    &product(&se3_exp(&delta), &a),
                    &b,
                    &measurement,
                    &information,
                )
                .0 - linearize_edge(
                    &product(&se3_exp(&-delta), &a),
                    &b,
                    &measurement,
                    &information,
                )
                .0) / 2e-6;
                let db = (linearize_edge(
                    &a,
                    &product(&se3_exp(&delta), &b),
                    &measurement,
                    &information,
                )
                .0 - linearize_edge(
                    &a,
                    &product(&se3_exp(&-delta), &b),
                    &measurement,
                    &information,
                )
                .0) / 2e-6;
                assert!(
                    (da - ja.column(axis)).norm() < 2e-7,
                    "a angle {angle} axis {axis}"
                );
                assert!(
                    (db - jb.column(axis)).norm() < 2e-7,
                    "b angle {angle} axis {axis}"
                );
            }
        }
    }
    #[test]
    fn pgo_quaternion_manifold_jacobians_match_numeric() {
        let a = se3_exp(&Vector6::new(3.0, -2.0, 1.0, 0.3, 0.1, -0.2));
        let b = se3_exp(&Vector6::new(1.0, 4.0, 2.0, -0.1, 0.2, 0.1));
        let measurement = Isometry3::translation(0.2, 0.1, 0.0);
        let (_, ja, jb) = linearize_pose_graph_edge(&a, &b, &measurement);
        for axis in 0..6 {
            let mut d = Vector6::zeros();
            d[axis] = 1e-6;
            let numeric_a =
                (linearize_pose_graph_edge(&manifold_increment(&a, &d), &b, &measurement).0
                    - linearize_pose_graph_edge(&manifold_increment(&a, &-d), &b, &measurement).0)
                    / 2e-6;
            let numeric_b =
                (linearize_pose_graph_edge(&a, &manifold_increment(&b, &d), &measurement).0
                    - linearize_pose_graph_edge(&a, &manifold_increment(&b, &-d), &measurement).0)
                    / 2e-6;
            assert!((numeric_a - ja.column(axis)).norm() < 2e-7);
            assert!((numeric_b - jb.column(axis)).norm() < 2e-7);
        }
    }
}
