# firefly-rootslam-mapping

Persistent sparse mapping, loop verification, pose-graph optimization and global
bundle adjustment based on the [RootSLAM paper](https://arxiv.org/abs/2610.15157).
This crate is an independent implementation. Algorithm provenance is recorded in
the [license index](../../docs/licenses/README.md); dataset procedures and measured
results are in the [RootSLAM guide](../../apps/rootslam/README.md).

## Map constraints

The map stores four edge kinds with separate optimization and retrieval roles:

| Kind | Stored relative pose | Current role |
|---|---|---|
| `Temporal` | Odometry measurement at insertion | PGO and global BA |
| `Loop` | Geometrically verified gP3P measurement | PGO and global BA |
| `Covisibility` | Current map estimate | Retrieval islands and connectivity |
| `SpanningTree` | Current map estimate | Retrieval islands and connectivity |

The optimization edge policy is an implementation choice tracked in
[issue #131](https://github.com/3SE-Competitive-Robotics-Team/firefly/issues/131).
A constraint with zero initial residual still contributes curvature and restricts
subsequent map deformation. PGO and global BA each need a justified edge set,
measurement definition and information scale.

`EdgeInformation` supplies the square-root information `S_ab` for metre/radian
pose residuals. The default derives pixel-equivalent translation and rotation
scales from the calibrated camera response at a configurable reference depth.
The map owns this scale for temporal, loop and connectivity edges. The default
is a project calibration policy; the paper leaves its numerical values unspecified.

## Global solver

The default solver adapts
[RootBA](https://github.com/NikolausDemmel/rootba/tree/d3900037fc4bc98328e5d8d26f1c4eed922f88cc):
column scaling, landmark QR elimination, Schur-Jacobi block-preconditioned CG and
Levenberg–Marquardt acceptance. The rig shares a six-dimensional body pose across
fixed camera extrinsics and adds RootSLAM pose-graph factors.

Linearization produces reusable undamped blocks. Each LM attempt applies damping,
solves the reduced system, back-substitutes points and evaluates the trial on the
same observations. An accepted step advances the state and requests a new
linearization; a rejected step increases damping and reuses the current blocks.
Termination records costs, attempted/accepted steps, PCG iterations and convergence.
The iteration budget and the numerical convergence flag describe separate outcomes.

The default uses physical-coordinate damping and a physical-gradient check after
the PCG model test. `damping_coordinates = "scaled"` and
`linear_solver_gradient_tolerance = 0` select the upstream scaled/model-only
policy for controlled comparisons. A finite truncated PCG step goes through the
same LM acceptance test. `lsqr_reference` supplies an independent numerical path.

Householder vectors and sparse pose incidence implement the reduced operator
`Aᵀ(Ax)`. The preconditioner uses its 6×6 diagonal pose blocks; their undamped
terms are shared by damping retries. Tracks incident to at most two poses cache
their small reduced operator. Landmark work runs in parallel, with six reversible
rotations per block for damping updates.

Each optimized landmark has observations from at least two distinct camera views;
a stereo pair at one rig pose qualifies. Single-view landmarks remain attached to
their host keyframe and follow its map correction. Their observations support
later association. QR supports rank-deficient blocks through separate triangular
and damping rows.

Camera factors use each model's injective projection domain. Initial factors must
lie in that domain; trial projections outside it produce a rejected LM attempt.
Observation membership stays fixed throughout a solve. Unit camera rays make
projection thresholds angular, and Jacobians map back to metric coordinates.
Column scaling uses RootBA's Sophus square-root epsilon: 1e-5 for double and
approximately 0.0031623 for float. The gradient check uses a machine-epsilon floor.

## Numerical validation

`tests/reference/rootba_cg.cpp` generates the PCG reference from the fixed upstream
header. Compile it with C++17, the checkout's `src/` and `external/Sophus/`,
Eigen and glog; compare stdout with `tests/reference/rootba_cg.json`.
The fixture checks the step, model stopping iteration and scaling regularizer in
both precisions. Independent SVD tests cover the compressed QR operator,
preconditioner, back-substitution and damping reuse.

Frozen-problem replay compares solver choices on identical geometry and constraints.
Complete dataset runs additionally evaluate map construction, loop quality, trajectory
coverage and elapsed time. [Issue #149](https://github.com/3SE-Competitive-Robotics-Team/firefly/issues/149)
tracks the measured performance, convergence and accuracy work.

See [architecture](../../docs/architecture.md#basalt-与-rootslam-独立应用) for state
ownership and the live VIO/planner data flow.
