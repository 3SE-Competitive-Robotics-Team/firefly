# firefly-rootslam-mapping

Persistent mapping, loop detection and verification, pose-graph optimization,
and global bundle adjustment based on the RootSLAM paper. The authors'
implementation is unpublished; this crate implements the described methods
without claiming an exact source or performance reproduction.

## Optimization edge set

The pose graph holds four kinds of edges. Only the measurement-bearing ones
take part in the optimization, matching the paper, which relates keyframes
"through the odometry and through previously closed loops" (section III-C):

| Kind | Measurement | In Equation (1) and (9) |
| --- | --- | --- |
| `Temporal` | odometry relative pose at insertion | yes |
| `Loop` | gP3P estimate of a validated loop closure | yes |
| `Covisibility` | the relative pose the map holds right now | no |
| `SpanningTree` | the relative pose the map holds right now | no |

The covisibility graph and its spanning forest restate the current estimates,
so their residual is zero by construction. They keep the graph connected and
group retrieval candidates into islands; as optimization constraints they would
only add stiffness that pins the map to its present shape.

## Pose-graph residual scale

`S_ab`, the square root information matrix of Equation (9), is the scale that
brings the metre- and radian-valued pose-graph residuals onto the same unit as
the pixel-valued reprojection residuals. The paper does not publish its
values. `EdgeInformation` therefore derives them from the camera model: the
image-plane displacement of one metre of rigid body translation and of one
radian of rigid body rotation at a configurable reference depth, which for a
pinhole camera is `focal / depth` pixels per metre and `focal` pixels per
radian. The scale is owned by the persistent map, so the odometry, covisibility
and loop edges cannot drift apart from each other.

## References

Global BA defaults to the staged QR / Schur-Jacobi PCG algorithm from
[RootBA](https://github.com/NikolausDemmel/rootba/tree/d3900037fc4bc98328e5d8d26f1c4eed922f88cc).
The rig adapter fixes intrinsics, shares a six-dimensional body pose between cameras,
and appends RootSLAM pose-graph residuals. Landmark and pose columns are scaled before
damping; rejected LM steps reuse the QR factors through six reversible damping rotations.
The rig's default preserves physical-coordinate damping and verifies the original
physical-gradient accuracy after the PCG model test. RootBA's scaled damping and
model-only termination are explicit options (see the application README).
The reduced operator is applied as `Aᵀ(Ax)` without assembling the reduced Hessian;
only its 6×6 diagonal blocks are formed for the preconditioner. Landmark work is parallel.
The rig implementation retains Householder vectors and sparse pose incidence, applies
the orthogonal transforms implicitly during PCG, and constructs one pose block at a time
for the preconditioner. This is the same reduced operator without the quadratic-size
dense landmark storage. Tracks incident to at most two poses cache the small reduced
operator to avoid repeated orthogonal transforms. Both forms are tested against
independent SVD projection. The undamped preconditioner terms are reused across damping retries.
Global BA requires observations from at least two distinct camera views, matching
RootBA's landmark-block input contract. Stereo observations at one rig pose qualify.
Single-view landmarks remain in the persistent map but are omitted from the optimizer's
point updates, so map application moves them with their host keyframe. Their observations
remain available for later association. The QR primitive also supports rank-deficient
blocks, with separate triangular and damping rows.
The rig adapter preserves each camera's injective projection domain:
invalid initial factors fail linearization, and invalid trial projections reject the step.
Unlike RootBA's Snavely-camera default `ERROR` policy, merely finite projections are
insufficient for generalized cameras such as EUCM, which have a second projection branch.
The eligible observation set is fixed before optimization; trial steps never drop
observations to lower cost. Costs from runs with different eligible sets are not comparable.
Projection uses unit camera rays and maps the Jacobian back to metric coordinates.
This keeps camera optical-axis thresholds independent of landmark distance and
prevents near-camera KB4 landmarks from changing projection branches with precision.
Jacobian column scaling uses RootBA's default Sophus square-root epsilon
(1e-5 in double, approximately 0.0031623 in float), including weak or zero columns.
The gradient guard's numerical floor separately uses machine epsilon.
The retained LSQR reference uses physical-coordinate damping and is for numerical comparisons.
Algorithm attribution and the RootBA/Ceres notices are included in the license index below.

`tests/reference/rootba_cg.cpp` generates the checked-in PCG numerical oracle using
the unmodified upstream header at the revision above. It checks both the step and
the quadratic-model stopping iteration and the default scaling regularizer in float
and double. Compile with C++17, the checkout's `src/` and `external/Sophus/`
include paths, Eigen and glog, then compare stdout with
`tests/reference/rootba_cg.json`. The SVD tests separately certify the compressed
QR operator, damping retries, landmark back-substitution and preconditioner.
The solver adaptation does not imply reproduction of the paper's full system or
its dataset-group timing; map inputs and optimization budgets must be reported.

- [RootSLAM paper](https://arxiv.org/abs/2610.15157)
- [Workspace architecture](../../docs/architecture.md#basalt-与-rootslam-独立应用)
- [License index](../../docs/licenses/README.md)
