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

#heading[Conclusion]

<sec:conclusion>

This paper presented DIVO, a direct depth-inertial-visual odometry framework
that adapts the FAST-LIVO2 algorithm family #cite(<lin2025fastlivo2>) to
depth-camera platforms. DIVO tightly couples a depth camera, a gray camera,
and an IMU through an error-state iterated Kalman filter with sequential
state updates, using an adaptive hierarchical voxel map with local planar
patches and multi-scale reference patches for sparse direct photometric
alignment. The central contributions are the depth-camera measurement-model
adaptation — replacing LiDAR beam-divergence noise with a disparity-domain
σ ∝ z² model plus pixel-quantization angular uncertainty — and a
pipeline-level update gate that rejects single-frame state jumps from
immature map structure. The entire system is implemented in Rust with an
`unsafe`-free codebase and a ROS-free, zero-copy shared-memory transport,
and is validated by 493 unit and integration tests including
finite-difference Jacobian checks below 1e-6.

On the 75 s end-to-end hovering experiment, DIVO achieves an ATE-RMS of
0.1782 m with 395 accepted depth updates and 653 accepted visual updates,
and the update gate eliminated the startup-transient position explosion
observed in the pre-gate run (6 rejections, all in the startup phase). The
paper is structured to mirror the FAST-LIVO2 paper section by section, so
that the depth-camera adaptation can be audited against the reference
implementation.

Future work includes: (i) a full quantitative ablation campaign across
multiple trajectories and sensor configurations; (ii) motion-adaptive gate
thresholds and model-based rejection criteria for aggressive flight; (iii)
rolling-shutter and auto-exposure modeling for real depth cameras; and (iv)
integration of the sliding-window map behavior into the end-to-end
evaluation on long-range trajectories.
