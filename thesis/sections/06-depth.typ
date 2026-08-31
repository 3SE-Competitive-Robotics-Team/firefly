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

#heading[Depth Measurement Model]

<sec:depth>

This section derives the depth-camera adaptation of the LiDAR measurement
model of FAST-LIVO2 (its Section VI). The depth camera produces a dense
range image; we back-project it into a structured point cloud, model each
point's uncertainty in a principled depth-dependent way, and form
point-to-plane residuals whose Jacobians follow the same derivation as the
LiDAR case.

#heading(level: 2)[From LiDAR Points to Depth Points]

FAST-LIVO2 models a LiDAR point's body-frame covariance as the sum of a
range term along the beam direction and a beam-divergence/encoder term in
the tangential plane:

$ #math.bold("Σ")#math.attach("", b: "pj") = #dhat #math.op("·") σ_d^2 #dhat^T + #math.bold("A") #math.bold("Σ")_ω #math.bold("A")^T,  quad  #math.bold("A") = r ⌊#dhat ×⌋ #math.bold("N"), $ <eq:lidarcov>

where #dhat is the beam direction, σ_d the range uncertainty, ⌊·×⌋ the skew
operator, and #math.bold("N") a tangent-plane basis. A depth camera has no
beam divergence; its two dominant uncertainty sources are (i) *range*
uncertainty in the disparity domain and (ii) *angular* uncertainty from
pixel quantization. We therefore substitute

$ σ_d → σ_z(z) = sqrt((z^2 σ_("disp")/(f B))^2 + (k_("hole") z)^2),  quad  σ_ω → σ_("ang") = 1/2 arctan(1/(2 f)), $ <eq:depthnoise>

where f is the focal length, B the stereo baseline, σ_disp the disparity
standard deviation (4·depth_noise = 0.08 px in the simulator), and
k_hole = 0.02 models the range jump induced by the 5–15% invalid holes in
real depth maps. The covariance keeps the structure of Eq. #ref(<eq:lidarcov>)
with these substitutions; the resulting σ_z grows quadratically with depth,
which the filter exploits by automatically down-weighting far points.

#heading(level: 2)[Structured Cloud Back-Projection and Downsampling]

Each valid pixel (depth z > 0.05 m, z ≤ 6 m, finite) is back-projected in
the OpenGL-style depth convention and rotated into the virtual pinhole
frame:

$ #pv#math.attach("", b: "v") = #math.bold("R")#math.attach("", b: "glv") #pv#math.attach("", b: "gl"),  quad  #pv#math.attach("", b: "gl") = ((u − c_x)/f_x · z, (v − c_y)/f_y · z, −z)^T. $ <eq:backproj>

Before the update, the cloud is voxel-downsampled on a 0.5 m grid: within
each grid cell the point with the *smallest* depth uncertainty is kept
(equivalently the nearest point, since σ_z ∝ z²). This reduces a 320×240
depth frame (up to 76.8 k points) to a few hundred well-conditioned
measurements and prevents far, noisy points from dominating the plane fits.
Edge pixels whose depth jumps by more than 0.15 m from a 4-neighbor are
discarded before downsampling, rejecting the 1-px foreground bleed that
simulators and real sensors produce at depth discontinuities.

#heading(level: 2)[Point-to-Plane Residual and Jacobian]

The depth measurement model is the standard point-to-plane residual against
the voxel map (Eq. (18)–(19) of FAST-LIVO2):

$ #math.op("0") = #math.bold("n")^T (#math.bold("T")#math.attach("", b: "IG") · #math.bold("T")#math.attach("", b: "LI") · #pv#math.attach("", b: "j")^L − #qq), $ <eq:ppres>

where #math.bold("T")#math.attach("", b: "LI") is the (near-identity) depth-to-IMU
extrinsic, #qq and #math.bold("n") are the plane center and normal of the
corresponding voxel, and #pv#math.attach("", b: "j")^L is the depth point in the depth
camera frame. With the right-perturbation convention of Eq.
#ref(<eq:boxplus>), the Jacobian of the residual with respect to the error
state is

$ #math.bold("H")#math.attach("", b: "j") = [ #math.bold("A")#math.attach("", b: "j")^T, #math.bold("n")^T, #math.bold("0")#math.attach("", b: "1×13") ],  quad  #math.bold("A")#math.attach("", b: "j") = ⌊#pv#math.attach("", b: "b") ×⌋ #math.bold("R")#math.attach("", b: "wi")^T #math.bold("n"), $ <eq:pph>

because ∂(#math.bold("R")#math.attach("", b: "wi") Exp(δθ) #pv#math.attach("", b: "b"))/∂δθ = −#math.bold("R")#math.attach("", b: "wi") ⌊#pv#math.attach("", b: "b") ×⌋
and #math.bold("n")^T(−#math.bold("R")⌊#pv#math.attach("", b: "b") ×⌋) = #math.bold("A")#math.attach("", b: "j")^T.
The scalar measurement variance combines the plane uncertainty and the point
uncertainty propagated to the world frame:

$ σ_l^2 = #math.bold("J")#math.attach("", b: "nq") #math.bold("Σ")#math.attach("", b: "nq") #math.bold("J")#math.attach("", b: "nq")^T + #math.bold("n")^T (#math.bold("R")#math.bold("Σ")#math.attach("", b: "pj") #math.bold("R")^T) #math.bold("n") + 0.001, $ <eq:ppvar>

with #math.bold("J")#math.attach("", b: "nq") = [#pv − #qq, −#math.bold("n")]. The residual
and Jacobian are verified against finite differences in the unit test
`jacobian_matches_finite_difference` to a relative error below 1e-6.

#heading(level: 2)[Correspondence, Outlier Rejection, and Degeneracy Protection]

For each point the closest planar voxel is found by descending the octree.
A candidate plane must pass a *radial test* — the point-to-center distance
projected on the plane must be within 3× the plane's point-distribution
radius — and a *chi-square gate*: the residual must satisfy
|z| ≤ σ_num·√σ_l² with σ_num = 3 (matching the gate of FAST-LIVO2's
`voxel_map.cpp:737`). Among passing planes, the one with the highest
Gaussian likelihood is chosen.

Because a hovering platform views mostly a single ground plane, the
depth update can degenerate: if the mean normal is nearly parallel to the
camera axis (|cos| > 0.9) and more than 90% of the normals fall into a
single angular bin, all but one point per bin are dropped (zero-information
rows). This keeps the information matrix well-conditioned in the
near-fronto-parallel configuration where tangential position is weakly
observable from a single plane.

The full measurement batch keeps a *fixed dimension* equal to the number of
back-projected points; invalid points contribute zero information
(z = 0, #math.bold("H") = 0, R = 1e12) rather than shrinking the batch. This
keeps the sequential-update interface between the ESIKF and the measurement
models stateless and dimension-consistent.

#heading(level: 2)[Adaptive Noise and the Update Gate]

<sec:gate>

The pipeline-level update gate (Section #ref(<sec:system>)) is the practical
counterpart of the depth noise model: a freshly-initialized plane with only
the minimum 5 points can be fit with a biased normal, and the resulting
single-frame update can jump the state by ~0.5 m before the plane matures.
VOID therefore rejects any depth update whose position change exceeds 0.1 m
or whose rotation change exceeds 3° in a single frame, keeping the
propagation prior instead. The thresholds are comfortably above the
normal per-frame motion of a hovering platform (sub-0.1 m and sub-3° at
10 Hz) so legitimate updates are unaffected. The gate is a *composable
pipeline layer*: it does not modify the measurement models or the ESIKF,
and the same mechanism (with an additional velocity check of 0.5 m s⁻¹)
protects the visual update against reference-patch mismatches.
