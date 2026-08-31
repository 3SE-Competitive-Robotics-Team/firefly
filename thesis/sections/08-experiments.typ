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

#heading[Experiments]

<sec:experiments>

#heading(level: 2)[Experimental Setup]

DIVO is evaluated in the firefly end-to-end closed-loop simulation: a MuJoCo
#cite(<todorov2012mujoco>) physics environment publishes IMU (100 Hz) and
stereo-pair depth+gray frames (10 Hz, 320×240) over iceoryx2, DIVO estimates
the 6-DoF pose, and a downstream planner/controller closes the loop on the
reference trajectory. The platform hovers at the ground-truth pose
(1, 4, 1) m with zero span, so the ATE measures the estimation error under
near-static motion with rich depth structure (ground plane, walls, and a
column that constrains roll/pitch). Because the simulation adds fresh noise
to every run (no fixed seed), the reported numbers are aggregated over
three independent 75 s runs of the end-to-end evaluation; each run starts
its own simulator and estimator processes and computes ATE against the
same ground-truth trajectory.

The implementation is Rust (edition 2024) with `unsafe` forbidden at the
crate level, running on Apple Silicon (M-series), single-machine, with all
computation in one thread. The full workspace contains 494 unit and
integration tests; the Jacobians of both measurement models are validated
against central finite differences with relative errors below 1e-6, and the
sequential-vs-joint update equivalence is verified on the manifold.

#heading(level: 2)[End-to-End Accuracy]

Table #ref(<tab:ate>) reports the absolute trajectory error (ATE) of the
estimated pose against ground truth over three 75 s runs.

#figure(
  table(
    columns: (auto, auto, auto),
    align: (left, right, right),
    stroke: 0.5pt,
    table.header(
      table.cell[#text(weight: "bold")[Run]],
      table.cell[#text(weight: "bold")[ATE-RMS]],
      table.cell[#text(weight: "bold")[ATE-max]],
    ),
    [1], [0.1160 m], [0.1745 m],
    [2], [0.0258 m], [0.2941 m],
    [3], [0.1381 m], [0.1938 m],
    table.cell[#text(weight: "bold")[mean ± std]], table.cell[#text(weight: "bold")[0.0933 ± 0.0595 m]], table.cell[#text(weight: "bold")[0.2208 ± 0.0648 m]],
  ),
  caption: [Absolute trajectory error of DIVO over three independent 75 s
  hovering experiments (ground-truth pose (1, 4, 1) m, zero span).],
)<tab:ate>

All three runs stay below the 0.3 m RMS acceptance threshold despite
unseeded sensor noise; the per-run spread reflects the stochasticity of the
startup transient, where the first depth plane is initialized from a small
point set and the filter state is still degenerate. The sub-0.15 m mean RMS
error is achieved by the sequential fusion of depth and visual measurements,
combined with the pipeline update gate described below.

#heading(level: 2)[Update Statistics and Timing]

Table #ref(<tab:health>) summarizes the per-stage statistics of the runs,
as reported by the `void/health` visualization scalar stream and the
per-stage timing instrumentation.

#figure(
  table(
    columns: (auto, auto, auto),
    align: (left, right, right),
    stroke: 0.5pt,
    table.header(
      table.cell[#text(weight: "bold")[Quantity]],
      table.cell[#text(weight: "bold")[Value]],
      table.cell[#text(weight: "bold")[Note]],
    ),
    [Depth updates accepted], [≈ 700 per run], [point-to-plane ESIKF batches],
    [Visual updates accepted], [≈ 650 per run], [pyramid direct alignments],
    [Gate rejections per run], [1--8], [all in the startup transient],
    [Depth points per frame], [$O$(100)], [after 0.5 m voxel downsampling],
    [Visual points per frame], [60+], [after ray-casting completion],
  ),
  caption: [Per-stage statistics of the three 75 s evaluation runs.],
)<tab:health>

The per-frame pipeline runs well within the 100 ms budget of the 10 Hz
sensor rate; the dominant cost is the depth back-projection and
downsampling of the 76.8 k-pixel depth frame, followed by the two ESIKF
updates. Real-time operation (sim_rate > 1) is maintained throughout the
runs on the single Apple Silicon core, with the visualization publishing
decoupled from the computation thread (zero-IO computation, all rendering
data shipped over `Firefly/Viz`).

#heading(level: 2)[Design-Level Ablation Analysis]

A full quantitative ablation study (per-module A/B comparisons on multiple
trajectories) is outside the current validation scope; the following
design-level observations are supported by the implementation and the
experimental evidence available so far:

- *Depth noise model.* The quadratic σ_z ∝ z² model down-weights far
  points; the 6 m depth cap and the 0.5 m voxel downsampling are the
  practical consequences of this model. Without the downsampling, far,
  high-variance points dominate plane fits and produce the systematic
  position bias that motivated both the cap and the downsampling (see the
  comments in `options.rs` and the `downsample_keeps_one_point_per_voxel`
  test).
- *Visual warmup and map warmup.* The first 3 s of depth-map registration
  and the first 5 s of visual map-point creation are skipped so that the
  state stabilizes before the maps are built; this removes a startup
  bias that would otherwise be frozen into the reference patches.
- *Update gate with velocity channel.* The gate is the single most
  impactful robustness measure. Beyond the position/rotation thresholds
  that catch single-frame jumps (a 0.5 m state jump from an immature
  plane), the velocity channel rejects updates whose per-frame velocity
  increment exceeds 0.15 m/s. This is essential because a biased first
  plane does not inject a large jump: it is tracked as real motion through
  consecutive small updates, each well within the position threshold, that
  cumulatively drag the velocity estimate (and hence the position) away by
  up to 0.4--1 m. With the velocity channel and the stricter startup gate
  (the first accepted plane must accumulate enough inliers), the rejected
  updates are confined to the startup transient (1--8 per run across the
  three runs) and no sustained drift signature remains (ATE-max below
  0.35 m in every run). The consecutive-rejection protection additionally
  skips the depth update after 5 consecutive rejections and lets the
  visual update re-converge before depth measurements rejoin, preventing
  half-accepted states from accumulating drift.
- *Update gate generality.* The gate is a pipeline-level,
  measurement-agnostic safeguard and therefore protects the visual update
  against reference-patch mismatches as well as the depth update against
  immature planes.

We emphasize that these observations are drawn from the implementation
design and the three 75 s runs; a comprehensive ablation campaign on
multiple trajectories and sensor configurations is planned as future work.

#heading(level: 2)[Discussion and Limitations]

The evaluation platform is a physics simulation with idealized sensor
synchronization (paired depth+gray frames at 10 Hz, tolerance 20 ms) and
fixed exposure; real depth cameras exhibit rolling-shutter artifacts,
exposure auto-adjustment, and stronger disparity noise at boundaries, all
of which are modeled only approximately. The update gate's thresholds are
tuned for the hovering scenario; aggressive maneuvers require either
motion-adaptive thresholds or a model-based rejection criterion. Finally,
the current map never triggers the sliding window in the hover scenario,
so the sliding-window behavior is verified only by unit tests, not by an
end-to-end run.
