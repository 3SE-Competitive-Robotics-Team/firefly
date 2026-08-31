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

#heading[Visual Measurement Model]

<sec:visual>

The visual measurement model follows the sparse direct alignment of
FAST-LIVO2 (its Section VII): the reference patches stored on visual map
points are warped by the current pose hypothesis and compared
photometrically with the current gray image, jointly estimating the inverse
exposure time. VOID implements the same model with a per-point aggregated
residual that keeps the sequential-update dimension fixed.

#heading(level: 2)[Visibility Query and Ray-Casting]

At each frame, VOID collects the visual map points visible from the current
camera pose. A coarse frustum test (FoV cosine) over the root voxels within
the maximum ray depth returns candidate points; each candidate is then
projected exactly and stored with its reference patch. To compensate for
the sparsity of a freshly-initialized map, a *ray-casting* step samples the
unoccupied cells of the 30×30 image grid along the center-pixel direction
from d_min = 0.5 m to d_max = 20 m and stops at the first voxel that
contains a visual point, recovering points missed by the coarse query.

#heading(level: 2)[Patch Warping and Photometric Residual]

Let #pv#math.attach("", b: "v") be a visual map point with normal #math.bold("n")#math.attach("", b: "v")
and reference observation (pose #math.bold("T")#math.attach("", b: "r"), inverse exposure
τ_r, patch pyramid I_r). Under a candidate state the point projects to
#uv = π(#math.bold("T")#math.attach("", b: "ci") #pv#math.attach("", b: "v")) with #math.bold("T")#math.attach("", b: "ci")
the camera-to-IMU extrinsic, and the reference patch is warped into the
current image by the homography induced by the plane (#qq, #math.bold("n")):

$ #math.bold("A")#math.attach("", b: "k") = #math.bold("K") (#math.bold("R")#math.attach("", b: "kr") + #tv#math.attach("", b: "kr") #math.bold("n")^T/(#math.bold("n")^T #pv#math.attach("", b: "r")) ) #math.bold("K")^{-1}, $ <eq:affine>

where #math.bold("R")#math.attach("", b: "kr"), #tv#math.attach("", b: "kr") are the relative
reference→current pose, #pv#math.attach("", b: "r") the point in the reference camera
frame, and #math.bold("K") the intrinsics. The per-pixel residual of
FAST-LIVO2 (its Eq. (22)) is

$ #math.op("0") = τ_k I_k(#uv#math.attach("", b: "i")) − τ_r I_r(#uv#math.attach("", b: "i")^′), $ <eq:photores>

where #uv#math.attach("", b: "i") is the current-frame projection and #uv#math.attach("", b: "i")^′
the warped reference pixel. Each visible point contributes a patch of
11×11 = 121 pixels; VOID aggregates them into a single scalar measurement
per point (mean residual and mean Jacobian), with per-point covariance
R = img_point_cov / n_used. This preserves the fixed-dimension interface
while remaining statistically consistent: the aggregated measurement's
information is lower than the full per-pixel model, and img_point_cov is
scaled accordingly (2e4 vs. the per-pixel 1e2 of the reference).

#heading(level: 2)[Jacobian Structure]

With the right-perturbation rotation error, the Jacobian of the photometric
residual with respect to the error state decomposes as

$ #math.bold("H")#math.attach("", b: "i") = [ #math.bold("J")#math.attach("", b: "img") #math.bold("J")#math.attach("", b: "dpi") ⌊#pv#math.attach("", b: "cam") ×⌋,  quad  −#math.bold("J")#math.attach("", b: "img") #math.bold("J")#math.attach("", b: "dpi") #math.bold("R")#math.attach("", b: "cw"),  quad  I_k(#uv#math.attach("", b: "i")),  quad  #math.bold("0") ], $ <eq:visjac>

where #math.bold("J")#math.attach("", b: "img") = τ_k ∇I_k/scale is the image gradient
(row vector) at the sampled pixel, #math.bold("J")#math.attach("", b: "dpi") =
∂#uv/∂#pv#math.attach("", b: "cam") is the projection Jacobian, ⌊#pv#math.attach("", b: "cam") ×⌋
arises from ∂#pv#math.attach("", b: "cam")/∂δθ, and I_k(#uv#math.attach("", b: "i")) is the
exposure column ∂h/∂τ. The gradient is computed as the exact derivative of
the bilinear sampler (forward differences of the interpolated image), which
avoids the discretization mismatch of central differences against the
sampling function. The analytic Jacobian is validated against finite
differences in the unit test `visual_jacobian_matches_finite_difference`
(max error < 1e-3, well within tolerance).

#heading(level: 2)[Coarse-to-Fine Pyramid Update]

The update runs over the three patch-pyramid levels from coarse to fine
(Algorithm 3). At each level the affine warps and warped reference patches
are frozen at the current state (mirroring the `retrieveFromVisualSparseMap`
step of FAST-LIVO2), the ESIKF iterates to convergence, and the state is
re-warped and updated again — the finest level performs three re-match
rounds, the others one. The inverse exposure time is part of the estimated
state during the iterations; in the simulation, exposure is fixed and the
exposure-estimation flag is disabled, forcing τ = 1.

#figure(
  block[
    #text(weight: "bold")[Algorithm 3: Pyramid direct visual update]
    #v(4pt)
    #set text(size: 8.8pt)
    #set par(leading: 0.35em)
    #par[#h(12pt)For level from coarse to fine:]
    #par[  a. Freeze affine warps and warped reference patches at current state]
    #par[  b. Run ESIKF iteration over per-point photometric residuals (Eq. #ref(<eq:photores>))]
    #par[  c. Re-warp with the updated state (finest level: 3 re-match rounds)]
  ],
  caption: [Coarse-to-fine sparse direct alignment with re-warping, mirroring
  `computeJacobianAndUpdateEKF` of FAST-LIVO2.],
)<alg:visual>

#heading(level: 2)[Outlier Rejection]

Four gates protect the visual update, mirroring FAST-LIVO2's VII-A: (i)
*viewing-angle gate* — points whose current view direction deviates more
than 80° from the plane normal (min_view_cos = 0.17) are dropped; (ii)
*depth-discontinuity gate* — a point whose camera-frame depth differs from
its 3×3 neighborhood by more than 0.5 m is treated as occluded and dropped;
(iii) *patch-error gate* — points whose total squared patch error exceeds
outlier_threshold × 121 are dropped as mismatches; and (iv) an optional
Huber kernel on the aggregated residuals (disabled in the simulation, where
δ = ∞ reduces it to least squares).
