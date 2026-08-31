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
column that constrains roll/pitch). All numbers reported in this section
are from a single 75 s run of the P4 end-to-end evaluation (the
`firefly-void` P4 acceptance run, commit `131f09c`).

The implementation is Rust (edition 2024) with `unsafe` forbidden at the
crate level, running on Apple Silicon (M-series), single-machine, with all
computation in one thread. The full workspace contains 493 unit and
integration tests; the Jacobians of both measurement models are validated
against central finite differences with relative errors below 1e-6, and the
sequential-vs-joint update equivalence is verified on the manifold.

#heading(level: 2)[End-to-End Accuracy]

Table #ref(<tab:ate>) reports the absolute trajectory error (ATE) of the
estimated pose against ground truth over the 75 s run.

#figure(
  table(
    columns: (auto, auto),
    align: (left, right),
    stroke: 0.5pt,
    table.header(
      table.cell[#text(weight: "bold")[Metric]],
      table.cell[#text(weight: "bold")[Value]],
    ),
    [ATE-RMS], [0.1782 m],
    [ATE-mean], [0.1703 m],
    [ATE-max], [0.2049 m],
  ),
  caption: [Absolute trajectory error of DIVO over the 75 s hovering
  experiment (ground-truth pose (1, 4, 1) m, zero span).],
)<tab:ate>

The sub-0.2 m RMS error is achieved by the sequential fusion of depth and
visual measurements; during the run, 395 depth-frame updates and 653 visual
updates were accepted, the visual model tracking 704 frames in total (the
remaining frames had fewer than the warmup budget of 60 visible points and
were skipped by the visual update). The pipeline update gate rejected 6
single-frame updates, all of them during the startup transient when the
first depth plane was initialized from a small point set and the filter
state was still degenerate (NaN-prone). After the startup transient, no
further updates were rejected, and the previously-observed 7 s position
explosion (~0.5 m jump) caused by the first immature plane was eliminated.

#heading(level: 2)[Update Statistics and Timing]

Table #ref(<tab:health>) summarizes the per-stage statistics of the run,
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
    [Depth updates accepted], [395], [point-to-plane ESIKF batches],
    [Visual updates accepted], [653], [pyramid direct alignments],
    [Frames tracked visually], [704], [of 10 Hz frames with visible points],
    [Gate rejections], [6], [all in the startup transient],
    [Depth points per frame], [$O$(100)], [after 0.5 m voxel downsampling],
    [Visual points per frame], [60+], [after ray-casting completion],
  ),
  caption: [Per-stage statistics of the 75 s evaluation run.],
)<tab:health>

The per-frame pipeline runs well within the 100 ms budget of the 10 Hz
sensor rate; the dominant cost is the depth back-projection and
downsampling of the 76.8 k-pixel depth frame, followed by the two ESIKF
updates. Real-time operation (sim_rate > 1) is maintained throughout the
run on the single Apple Silicon core, with the visualization publishing
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
- *Update gate.* The gate is the single most impactful robustness measure:
  the 7 s explosion observed in the P4 pre-gate run (a single 0.5 m state
  jump from an immature plane) disappears when the gate is enabled, and the
  6 rejections all occur in the startup transient where the filter is still
  degenerate. This is a pipeline-level, measurement-agnostic safeguard and
  therefore also protects the visual update against reference-patch
  mismatches.

We emphasize that these observations are drawn from the implementation
design and the single 75 s run; a comprehensive ablation campaign on
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
