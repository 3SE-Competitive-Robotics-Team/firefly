# RootSLAM

Dataset execution connects the translated original Basalt optical flow and estimator
through bounded queues, with independent image and IMU producers and state collection.
The source baseline and numerical fixtures are described in
[firefly-basalt](../../crates/firefly-basalt/README.md).

The online entry provides stereo/IMU estimation and sparse mapping; the dataset
and headless tools expose offline workflows. Dataset trajectory equivalence,
paper performance reproduction and flight acceptance require separate evidence.

Run the dataset pipeline with `cargo run --release -p firefly-rootslam --bin firefly-rootslam-dataset`.
It reads the fixed [dataset configuration](../../configs/firefly-rootslam-dataset.toml), which uses
an original Basalt checkout beside this workspace. Adjust its dataset path, start
`./target/release/mimir --save logs/rootslam.rrd` in another terminal and run
`RUST_LOG=info cargo run --release -p firefly-rootslam --bin firefly-rootslam-dataset`
from the workspace root.
Configuration includes `dataset_path`, `dataset_type`, `camera_calibration`, and
optional `vio_config`, `trajectory`, and `metrics_path`. Calibration uses the
original Basalt cereal JSON archive (`value0`). `use_double` selects the local
estimator precision; `[optimization] precision = "float64"` independently selects
global bundle-adjustment precision (`"float64"` by default). `"float32"`
together with `krylov_precision = "native"` reproduces the entirely
single-precision configuration of the paper; the default stays double precision
because the single-precision path could not be validated against a dataset in
this repository. `[loop_options]`
overrides retrieval and verification parameters. Optional `[map_policy]` supplies
all map policy fields, including the pose-graph residual scale, whose values
were not published in the paper. These parameters are defined in the mapping
crate. The persistent map owns the scale shared by its pose-graph edges.

`[optimization] krylov_precision = "native"` is the default, so the Krylov
work vectors follow the matrix precision and a single-precision run is a
complete single-precision solve. Use `"float64"` for the mixed-precision
comparison, in which single-precision matrices are paired with double-precision
LSQR accumulation; the solver warns when that combination is selected. The
matrix precision, the landmark elimination and the Krylov accumulation are
reported with every result, and the run log states which combination was used.
`[loop_options] pose_graph_max_iterations` independently limits online
loop-closing PGO to 20 outer iterations by default. `[optimization]
max_iterations` controls final global BA (50 by default); increasing its budget
does not change online PGO's iteration budget. Other optimizer settings remain
shared between these paths.

`metrics_path` writes evaluation data, including SE(3)-aligned ATE, RTE separated
by six original camera frames, and success over requested input frames. Missing
estimates remain in the denominator. When `max_frames` limits an experimental
run, the evaluation input is truncated to the same prefix. GT timestamps and
poses use the dataset reader's convention directly. `max_ground_truth_gap_ns`
defaults to 110000000 ns for GT interpolation, following the Basalt association
limit; this limit is not specified by the RootSLAM paper.

The dataset runner also accepts `marg_data` (a new cache directory containing
original binary marginalization records and the entire dataset's `gt.cereal`),
`debug_directory` (original estimator statistics and `stats_vio.ubjson` runtime/ATE
samples), and `result_path` (original
`rms_ate`, `num_frames`, `exec_time_ns` JSON). The original result uses local
Basalt states and its fixed SVD association rules; `num_frames` counts the full
dataset even when `max_frames` truncates processing. Its timer excludes input
loading, final global optimization, and result writing. `trajectory_format`
selects `"tum"` (default), `"euroc"`, or `"kitti"`. Optional `groundtruth` writes
the full GT translation stream aligned to local states in TUM format with identity
orientations, following the original export. Ground-truth exports and original
results require timestamp associations; requests without them return an error.

The additional `firefly-basalt-tools --config <path.toml>` binary connects the
original headless calibration and evaluation workflows. Select `task` as
`"calibrate"`, `"calibrate-imu"`, `"kitti-eval"`, or `"time-align"`.
Calibration inputs are `dataset_path`, `dataset_type`, `aprilgrid`, and
`result_path`; camera calibration also requires `camera_types`. IMU calibration
loads `calibration.json` from `result_path` and runs the original three-stage
optimization. KITTI evaluation takes `ground_truth`, `estimate`, and `output`;
time alignment takes `dataset_path`, `dataset_type`, and `output`, with optional
paired `calibration` and `mocap_calibration` and `output_ground_truth`.
The `"vio-sim"` task takes `calibration`, `output` (TUM trajectory), and optional
`vio_config` and `archive` (original simulation Cereal JSON). It generates and
feeds the original noisy simulation into the estimator; `use_imu` and
`use_double` default to the original simulator's enabled settings.
Optional `marg_data` creates a new marginalization cache, including
`gt_spline.cereal`. The `"mapper-sim"` and `"mapper-sim-naive"` tasks consume
`calibration` and `cache`; optional `output` requests the original aligned RMSE
result. Existing cache directories are refused rather than deleted.
The `"mapper"` task consumes an original dataset marginalization `cache`
(including `gt.cereal`) and `calibration`, with optional `vio_config`. It runs
the original detect/match/tracks/optimize/filter/optimize sequence. Optional
`trajectory` writes TUM format (`tum = false` selects EuRoC CSV), and
`result_path` writes the original JSON fields `rms_ate`, `num_frames` (retained
mapper poses), and `exec_time_ns` (the mapping stages, excluding loading and
alignment). The `"opt-flow"` task takes `calibration`,
`dataset_path`, `dataset_type`, and optional `vio_config`; it runs the original
optical-flow worker and reports frame counts, track counts, and mean track length
through the existing diagnostics channel. Producer errors and panics terminate
the input queue and join the worker before returning an error.
Pangolin GUI and live device capture are not provided by these headless entries.
Run with `RUST_LOG=info` and the workspace `mimir` recorder to capture
iteration diagnostics in RRD.

The `"global-ba"` task replays a frozen global problem captured with
`[optimization] problem_archive`. Supply `problem`, a new `output` JSON path,
`precision` (`"float32"` or `"float64"`), and `krylov_precision` (`"native"` or
`"float64"`). It restores identical poses, landmarks, observations, calibration,
and solver settings before applying the requested precision. Optional
`max_iterations` changes the outer iteration budget; `linear_system_archive`
captures the first failed linear solve for independent numerical analysis.
Replay preserves the archived linear tolerance and linear iteration budget.
The output contains optimized geometry and convergence metrics; a completed
replay does not imply convergence. Existing result files are not overwritten.
The dataset application's optional `global_result_path` exports this same result
format from its actual final global solve, without repeating optimization. Pair
it with `[optimization] problem_archive` to retain input and optimized geometry
from one run. Both archive destinations must be new files.

Export that matching problem, result and optional final trajectory to Rerun with
`uv run python apps/rootslam/export_rrd.py --problem logs/rootslam_global_problem.json --replay logs/rootslam_global_result.json --trajectory logs/rootslam_trajectory.txt --output logs/rootslam_map.rrd`.
This offline recording contains optimized landmarks, keyframes, graph edges and
convergence metadata; live `mimir` captures runtime diagnostics separately.

For the simulated closed loop, `firefly-rootslam` subscribes to Render's
`Firefly/CameraLeft` and `CameraRight` plus Sim's `Firefly/Imu`. It pairs stereo
frames within the configured timestamp tolerance and waits for IMU coverage before passing
them to Basalt. Its rig calibration comes from the shared Firefly rig geometry.
It publishes `Firefly/Odometry`, `Firefly/CorrectedOdometry`,
`Firefly/Navigation`, and posterior `Firefly/VioAttitude` for FC using
[`configs/firefly-rootslam.toml`](../../configs/firefly-rootslam.toml). Start the default
estimator with `cargo run --release -p firefly-rootslam`.
`--no-loop-closures` disables loop detection and `--no-mapping` disables the persistent
sparse map. Dense depth fusion belongs to planner's EGO map.
`--external-localization` delegates corrected odometry and navigation publishing to
localization while preserving local odometry and attitude output. The default online
navigation uses the VIO frame; sparse-map corrections are not fed back to that output.
