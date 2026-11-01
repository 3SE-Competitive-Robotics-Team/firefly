# RootSLAM mapping

Original implementation is distributed under [BSD-3-Clause](LICENSE).
The Sophus-derived SE(3) Jacobian formula in `pose_graph.rs` retains its
[MIT notice](LICENSE-MIT).

Implementation of the persistent-map, pose-graph and rig-based square-root
global bundle adjustment described in RootSLAM (arXiv:2610.15157v1).
The authors' implementation is unpublished. This crate implements the published
equations; scheduling, solver tolerances and unspecified thresholds are explicit
implementation choices and are not claimed as an exact source reproduction.

Global optimization uses fixed camera calibration, world-to-body pose variables,
world landmarks, landmark-local orthogonal elimination, and a matrix-free
least-squares solve of the reduced square-root system. It never forms reduced
normal equations. Both f32 and f64 matrix storage and landmark QR are supported. Pose-graph
optimization holds active local-window keyframes fixed.

Global optimization defaults to f64. The f32 path remains an explicit comparison
option: on a frozen full-sequence problem, casting an independently certified
double-precision update to f32 failed the unchanged physical-gradient criterion.
Double-precision Krylov vectors alone did not resolve that returned-step rounding.
This default does not establish nonlinear convergence or reproduce the paper's
single-precision performance result.

LSQR uses column-norm right preconditioning of the complete damped operator.
The returned update is transformed back to physical coordinates and checked
against the original gradient criterion. Scaling does not change the damping
penalty or introduce normal equations. Tests include disparate column scales,
single-precision subnormals and zero columns.

`krylov_precision = "float64"` (default) keeps LSQR work vectors and matrix-vector
accumulation in double precision without duplicating the stored matrix. This is
a mixed-precision implementation when `precision = "float32"`, not a reproduction
of the paper's unspecified all-float solver configuration. `krylov_precision =
"native"` retains the matrix scalar type for the iteration as a comparison.
Both paths check the returned, rounded step using double-precision residual and
gradient accumulation against the same configured tolerance.

The global pose and landmark increments use isotropic LM damping, including
landmark damping before its QR elimination. Pose-graph residuals use the coupled
SE(3) logarithm and inverse right Jacobian in equations (8) and (10), with
translation followed by rotation in the tangent vector. Multi-camera Jacobians
include the fixed extrinsic adjoint in equation (7). Global gauge freedom is
removed by fixing the earliest body pose, an explicit implementation choice.

Loop-closing pose-graph optimization is a separate body-to-world solver for
equation (1), with additive translation and the Ceres quaternion-manifold chart,
LM, and direct sparse LDLT factorization. The Rust solver replaces Ceres;
equivalent residuals and chart derivatives are tested, but its iteration history
and performance have not been matched to Ceres. When no local frame is active,
the earliest pose anchors its gauge.
The system worker overrides only PGO's outer iteration budget using
`LoopOptions::pose_graph_max_iterations`, preserving its default of 20 separately
from the global optimizer's default of 50. Direct calls to the PGO function
continue to use the supplied `OptimizationOptions::max_iterations`.

The paper does not disclose the edge information scales, global reduced-system
solver configuration, or all termination thresholds. Information matrices are
provided by the map policy; `OptimizationOptions` exposes our LSQR, LM and
termination choices. The diagnostics' `peak_linear_storage_bytes` counts stored
numeric matrix/vector elements; it is not process peak RSS or allocator usage.
No measured speedup over the original system is claimed.

The forward matrix-vector product streams contiguous stored columns while
preserving each output's original accumulation order. The `lsqr_archive`
example's `--matvec` mode compares this traversal against the original row loop
on a frozen operator, checking bitwise equality and alternating timings for
f32/f32, f32/f64 and f64/f64 storage/workspace combinations. These timings cover
the matrix-vector product, not a complete solve or system run; the independent
physical-gradient reference keeps its original traversal.

Each global iteration freezes its currently projectable observation set. A trial
that invalidates one of those projections is rejected; an initially invalid
projection may enter a later iteration after another accepted update. This
active-observation policy is an implementation choice, not specified by the paper.

The optional `linear_system_archive` writes the first nonconverged linear solve
as a numerical JSON fixture, with column incidence, column-major matrix values,
residuals, damping, returned step and the unchanged relative gradient tolerance.
Values retain the original scalar quantization. The destination must not exist;
the archive supports independent arithmetic checks on identical solver input.

Tests check rig and pose-graph analytic Jacobians by central differences,
landmark QR plus LSQR against an independent full damped SVD solve at several
damping values, float/double optimization on identical multi-camera inputs,
and active-window fixation in pose-graph optimization. Full dataset trajectories,
runtime/memory benchmarks, and cross-implementation Ceres comparisons remain
separate system-level acceptance work.

The persistent map receives optimized local geometry before marginalization,
converts host-camera inverse-distance points to world coordinates, and maintains
observation ownership, covisibility and a spanning forest. The paper's redundancy
and map-size thresholds are provided by `MapPolicy::paper_thresholds`. The
unpublished quality score is replaced by a documented ranking of independent
observation support, total observations and age. Strong-covisibility selection,
graph contraction on culling, and chained trajectory anchors are implementation
choices. A culled keyframe removes its hosted points; its trajectory pose can
still be resolved through retained parents.

Loop detection computes the translated Basalt ORB descriptor on existing tracked
corners and queries HashBoW. Candidate islands, descriptor controls and geometric
verification thresholds are configurable implementation choices. The verifier
uses the existing Rust OpenGV generalized P3P RANSAC and nonlinear refinement,
followed by pixel reprojection checks. Descriptor and histogram caches are owned
by the loop worker; culled entries are removed from retrieval. Asynchronous loop
results are revalidated against current poses, canonical landmarks and pixels
before fusion. Optimizer results cannot overwrite newer local geometry or a
newer applied optimizer result. Pose-graph corrections transform hosted points
with their hosts; final global optimization supplies jointly updated points.

`global_problem::GlobalProblem` freezes global BA inputs and solver options in a
versioned JSON numerical archive. It retains raw f64 pose, camera, landmark,
observation and edge-information values, with float-roundtrip parsing and
exclusive file creation. It omits images, features and map lifecycle state;
restored snapshots are only for numerical replay, not resuming mapping.
`replay` changes only scalar/Krylov precision and disables archive side effects.
Callers needing an explicit iteration budget can restore, solve with their
chosen options, and use `ReplayResult::from_result` to record those actual options.
Capturing a problem during a one-iteration diagnostic run does not validate a
full optimization; comparisons must use the same explicit iteration budget.
