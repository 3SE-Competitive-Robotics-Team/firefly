// --- shared macros (must be defined per-file: include has its own scope) ---
#let Exp(x) = math.op("Exp") + x
#let Log(x) = math.op("Log") + x
#let boxplus(x, d) = x + math.cal("⊞") + d
#let boxminus(a, b) = a + math.cal("⊟") + b
#let xG = math.bold("x")
#let Rwi = math.bold("R") + math.attach("", b: "wi")
#let pwi = math.bold("p") + math.attach("", b: "wi")
#let vwi = math.bold("v") + math.attach("", b: "wi")
#let bg = math.bold("b") + math.attach("", b: "g")
#let ba = math.bold("b") + math.attach("", b: "a")
#let gvec = math.bold("g")
#let qq = math.bold("q")
#let xhat = "x" + "\u{0302}"
#let Phat = "P" + "\u{0302}"
#let pv = math.bold("p")
#let uv = math.bold("u")
#let tv = math.bold("t")
#let dhat = math.bold("d") + "\u{0302}"
#let phat = math.bold("p") + "\u{0302}"
// --- end shared macros ---

#heading[Local Mapping]

<sec:mapping>

The VOID map is an adaptive hierarchical probabilistic voxel map, a direct
port of the VoxelMap design #cite(<zhou2021voxelmap>) used by FAST-LIVO2
#cite(<lin2025fastlivo2>), adapted to depth-camera point clouds.

#heading(level: 2)[Map Structure]

The map is a hash table of *root voxels* of side 0.5 m, each containing an
octree that subdivides up to depth 3 (leaf half-length 6.25 cm). A node
accumulates temporary points until a per-layer threshold is reached, then
attempts to fit a plane by eigendecomposition of the point-set covariance:

$ #math.bold("C") = 1/N Σ p#math.attach("", b: "j") p#math.attach("", b: "j") p#math.attach("", b: "j")^T − #qq#qq^T, $ <eq:cov>

where #qq = 1/N Σ #pv#math.attach("", b: "j") is the plane center. If the smallest
eigenvalue satisfies λ_min < 0.01 the node is classified as a *planar voxel*
and the corresponding eigenvector becomes the plane normal #math.bold("n");
otherwise the node subdivides and the points are re-distributed to its
children. At the maximum depth, non-planar nodes stop accumulating and drop
new points. Once a plane is mature (≥ 50 points) its parameters are frozen
and new points are discarded, bounding the per-voxel memory.

The plane is stored as the tuple (#qq, #math.bold("n")) together with the
6×6 covariance #math.bold("Σ")#math.attach("", b: "nq") of the joint normal/center
estimate, propagated from the point measurement covariances #math.bold("Σ")#math.attach("", b: "pj")
by the linearized Jacobian of the eigen-decomposition:

$ #math.bold("Σ")#math.attach("", b: "nq") = Σ#math.attach("", b: "j") #math.bold("J")#math.attach("", b: "j") #math.bold("Σ")#math.attach("", b: "pj") #math.bold("J")#math.attach("", b: "j")^T, $ <eq:snq>

with #math.bold("J")#math.attach("", b: "j") the 6×3 derivative of (#math.bold("n"), #qq)
with respect to point #pv#math.attach("", b: "j") #cite(<zhou2021voxelmap>). The plane
also stores its point-distribution radius (√λ_max), used by the nearest-plane
radial test in the depth update, and the eigenvalues λ_min, λ_mid, λ_max used
for planarity classification and covariance scaling.

#heading(level: 2)[Visual Map Points and Reference Patches]

Each planar voxel that projects into the current image and passes a
gradient test seeds *visual map points* on a 30×30 pixel grid. A visual map
point stores its world position #pv#math.attach("", b: "v"), its plane normal
#math.bold("n")#math.attach("", b: "v"), and up to 30 *observations*, each consisting
of a three-level image patch pyramid (11×11 pixels per level, sampled at
scales 1, 2, 4 from the reference image), the observation pose, and the
inverse exposure time at observation time.

A new observation is appended when either 20 frames have elapsed or the
projected pixel has moved more than 40 px from the last observation
(Theorem: V-C of FAST-LIVO2). When observations exceed the budget, the
lowest-scored one is dropped. The *reference patch* is chosen by the score

$ S = (1 − ω_1) · 1/n Σ_(i) "NCC"(f, g_i) + ω_1 · c,  quad  ω_1 = ("tr"(#math.bold("Σ")#math.attach("", b: "n")))/(1 + e^("tr"(#math.bold("Σ")#math.attach("", b: "n")))), $ <eq:score>

where NCC(·,·) is the zero-mean normalized cross-correlation between patch
level 0 and the other observations, and c = #math.bold("n")̂#math.op("·")#phat
is the cosine between the plane normal and the viewing direction — the same
score as FAST-LIVO2 Eq. (12).

#heading(level: 2)[Normal Refinement]

Because the normal of a plane derived from a depth cloud is only as accurate
as the depth noise allows, VOID refines the visual-map-point normal by
photometric minimization across its observations (Section V-E of
FAST-LIVO2). The reference patch is warped into the target observation by
the affine warp of Eq. #ref(<eq:affine>), and the photometric residual

$ E(#math.bold("m")) = Σ_("pixels") (τ_i I_i(#math.bold("A")#math.attach("", b: "i")#uv#math.attach("", b: "r")) − τ_r I_r(#uv#math.attach("", b: "r")))^2 $ <eq:photoerr>

is minimized over the two free parameters #math.bold("m") ∈ ℝ² that
parameterize the plane in the reference camera frame through the change of
variables #math.bold("M") = #math.bold("B")#math.bold("m") + #math.bold("b")
(Eqs. (15)–(16) of FAST-LIVO2), recovering the refined normal as
#math.bold("n")∗ = #math.bold("M")∗ / |#math.bold("M")∗|. The minimization is a
Gauss–Newton iteration with numerical Jacobians (convergence threshold 1e-6);
it runs without blocking the main pipeline.

#heading(level: 2)[Map Maintenance]

The map is maintained inside a sliding window: when the current position is
more than 8 m from the last sliding position, all root voxels outside the
[−100, +100] root-voxel range centered on the current position are removed,
mirroring the `mapSliding` behavior of FAST-LIVO2. In the firefly
simulation the platform hovers near a fixed point, so the sliding window
never triggers and the map accumulates a dense local structure around the
hover location.
