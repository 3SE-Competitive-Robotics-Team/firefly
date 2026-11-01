# RootSLAM

Dataset execution connects the translated original Basalt optical flow and estimator
through bounded queues, with independent image and IMU producers and state collection.
The source baseline and numerical fixtures are described in
[firefly-basalt](../../crates/firefly-basalt/README.md).

This application is under construction. A complete RootSLAM system, trajectory
equivalence, and performance reproduction have not been verified.

Run with `cargo run --release -p firefly-rootslam -- --config <path.toml>`.
The [dataset configuration](../../configs/rootslam.toml) uses an original Basalt checkout beside
this workspace. After adjusting its dataset path, start
`uv run firefly-viz --save logs/rootslam.rrd` in another terminal and run
`RUST_LOG=info cargo run --release -p firefly-rootslam`
from the workspace root.
Configuration includes `dataset_path`, `dataset_type`, `camera_calibration`, and
optional `vio_config`, `trajectory`, and `metrics_path`. Calibration uses the
original Basalt cereal JSON archive (`value0`). `use_double` selects the local
estimator precision; `[optimization] precision = "float64"` independently selects
global bundle-adjustment precision (`"float64"` by default); `"float32"` is an
explicit experimental comparison. `[loop_options]`
overrides retrieval and verification parameters. Optional `[map_policy]` supplies
all map policy fields, including edge information scaling, whose values were not
published in the paper. These parameters are defined in the mapping crate.

`[optimization] krylov_precision = "float64"` is the default for LSQR work
vectors and accumulation; matrix storage and landmark QR retain `precision`.
Use `"native"` for the corresponding native-scalar iteration. Single-precision
matrices with double-precision LSQR constitute mixed precision.
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
Run with `RUST_LOG=info` and the workspace `firefly-viz` recorder to capture
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
convergence metadata; live `firefly-viz` captures runtime diagnostics separately.

`firefly-basalt-live` provides headless T265 acquisition with the optional
`realsense` feature. Build against the shared SDK 2.51.x installation described
in [the callback bridge](../../crates/firefly-basalt-realsense/README.md), then run
`cargo run --release -p firefly-rootslam --features realsense --bin firefly-basalt-live -- --config <path.toml>`.
Select `task = "vio"` with `trajectory` for the real optical-flow and VO/VIO
pipeline, or `task = "record"` with `dataset_root` for the original stereo,
exposure, IMU and optional device-pose dataset layout. Other options and defaults
are defined by [live Options](src/live.rs); `use_imu = false` selects VO.
SIGINT/SIGTERM requests producer stop, queue drain and worker joins. Diagnostics
continue through log IPC during shutdown. Hardware reset is an explicit option.
The CLI's no-device failure and injected VO/VIO/recording lifecycle tests have
run, but T265 acquisition itself has not been tested on connected hardware.
