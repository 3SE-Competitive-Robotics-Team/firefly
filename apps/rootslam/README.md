# RootSLAM

Dataset execution connects the translated original Basalt optical flow and estimator
through bounded queues, with independent image and IMU producers and state collection.
The source baseline and numerical fixtures are described in
[firefly-basalt](../../crates/firefly-basalt/README.md).

The single application entry is `firefly-rootslam --config <path>`.
The default [configuration](../../configs/firefly-rootslam.toml) consumes IPC from
render/cameras and IMU. The [dataset example](../../configs/firefly-rootslam-dataset.toml)
selects a recorded source:

```bash
./target/release/mimir --save logs/rootslam.rrd
RUST_LOG=info ./target/release/firefly-rootslam --config configs/firefly-rootslam-dataset.toml
```

Start the recorder first in another terminal and stop it last. Paths are relative
to the process working directory; run the example from the workspace root.

- `[input]`: `type = "ipc"` (default), or `type = "dataset"` with `format`,
  `path` and `calibration`. Formats are `euroc`, `uzh`, `kitti` and `lamaria`.
- Top-level `precision = "float64"` (default) or `"float32"`: shared arithmetic
  for local VO/VIO BA, IMU preintegration, marginalization and final global BA.
- `[vio]`: common estimator options, including optional
  `vio_config` (Basalt archive), `optical_flow_detection_grid_size`, `map`
  and `loops`. Both sources use the same bias/yaw prior construction.
  Mapping and loop closure default off; the dataset examples explicitly enable both.
- `[dataset]`: optional outputs and offline execution controls described below.
  Supplying this section with IPC is an error. `vio.stats_directory` and
  `external_localization` are IPC-only.

The input boundary uses integer nanosecond timestamps, native per-camera
dimensions, sensor-to-IMU extrinsics and calibrated noise densities. Images must
match calibration; stereo indices and gyro/accelerometer timestamps must pair.
IPC retains its independent event-driven IMU thread and stationary flight
initialization. Dataset producers use bounded queues and Basalt initialization;
they drain on completion or interruption. Interrupted runs return failure.
Dataset camera times are shifted into the IMU clock using calibration
`cam_time_offset_ns` (`t_imu = t_camera + offset`). File lookup retains the original
camera times; raw IMU and ground truth keep their supplied timestamps. Ground-truth
time alignment must be verified against the dataset's evaluation protocol. The offset is applied
once before optical flow and also to the requested-frame coverage denominator.
Dataset output is the posterior trajectory at these calibrated exposure times, without the live
high-rate IMU prediction used by flight control.

### LaMAria

Use the official [ASL download tool](https://github.com/cvg/lamaria/blob/main/tools/download_lamaria.py)
with `--sequences R_01_easy --type asl`. Point `input.path` at the extracted
directory containing `aria/`, and `input.calibration` at the corresponding
official `pinhole_calibrations/*.json`. The ASL images are already undistorted;
the raw VRS fisheye calibration is incompatible. Camera sizes may differ.
`T_b_s` uses XYZW quaternions and is converted to IMU-from-camera using the
IMU's body transform. Noise, sampling rate and gravity come from that file.
EuRoC/KITTI use the original Basalt cereal calibration (`value0`).

Set `dataset.trajectory_format = "lamaria"` to export integer nanoseconds and
`tx ty tz qx qy qz qw` for the official evaluator. Controlled sequences use
MPS pseudo-ground-truth and the official evaluator fits scale. Report its score
alongside an SE(3)-only score, fitted scale and input/output coverage; a single
controlled VIO run does not establish leaderboard or full-SLAM performance.
LaMAria pGT is evaluated externally; it is never supplied as an estimator prior.

### UZH FPV

The [UZH example](../../configs/firefly-rootslam-uzh.toml) consumes the official
[Snapdragon text/image archives](https://fpv.ifi.uzh.ch/datasets/) directly.
Extract the sequence into `input.path` (containing `left_images.txt`,
`right_images.txt`, `imu.txt`, and optional `groundtruth.txt`). Point
`input.calibration` to the matching extracted calibration directory, containing
one `camchain-imucam-*.yaml` and `imu.yaml`. YAML is a native sensor import format;
application configuration remains TOML. An existing Basalt calibration JSON file
is also accepted instead of a directory.

The importer converts Kalibr pinhole/equidistant parameters to Kannala–Brandt4,
inverts `T_cam_imu`, and preserves the published continuous noise densities.
Kalibr's [time convention](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)
sets the camera-to-IMU clock offset. Stereo uses the left exposure time; the
right calibration offset must differ by at most 1 ms (93 microseconds in the
indoor-forward bundle). Per-camera exposure-time optimization is not implemented.
Only Snapdragon stereo is supported by this adapter; DAVIS event data is not.

TUM trajectory timestamps are seconds in the IMU clock, with IMU poses and XYZW
quaternions. Ground truth is scoring-only. Native `metrics_path` reports our
SE(3) ATE/coverage metrics, not the UZH leaderboard score. For the latter use
the official [RPG evaluator](https://github.com/uzh-rpg/rpg_trajectory_evaluation)
with relative errors over 40/60/80/100/120 m segments and no scale correction;
report association coverage and each segment length separately. Public-GT
sequences provide local benchmarks, not a held-out leaderboard submission.

### Paper benchmark catalog

[benchmarks.json](benchmarks.json) catalogs the 103 evaluation sequences in
[RootSLAM, Section IV](https://arxiv.org/html/2610.15157v1): 64 MSD, 11 EuRoC
and 28 TUM-VI sequences. It records paper groups, camera counts, official sources
and the scoring protocol. MSD downloads are pinned to a dataset revision with
archive sizes and SHA-256 hashes; multi-volume archives require every listed part.
The catalog is an inventory, not a download or execution record.

Use the existing dataset entry with matching public calibration. The examples are
[EuRoC](../../configs/firefly-rootslam-euroc.toml) and
[TUM-VI](../../configs/firefly-rootslam-tumvi.toml), referencing the public
calibrations in Basalt revision `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
Keep each run's outputs in a separate directory and record calibration hashes.
TUM-VI's
512×512, 16-bit EuRoC exports already contain calibrated IMU measurements,
aligned clocks and IMU-frame ground truth; do not reapply raw-data corrections.
The current EuRoC reader handles two cameras. MSD's 15 MG sequences need a
four-camera adapter before they can contribute to the paper benchmark.

Evaluate complete sequences with rigid SE(3) alignment and six-original-frame
RTE, retaining input coverage and both local VIO and final smoothed trajectories.
Paper table values are group means in centimetres; native metrics are metres.
The paper's aggregate excludes the eight TUM-VI outdoors sequences. Report
partial batches separately, and do not compare one sequence to a group mean as
evidence of reproduction. Run artifacts and completion status belong in `logs/`.

### Dataset outputs and mapping

All options in this section belong to `[dataset]`. Nested tables are
`[dataset.optimization]`, `[dataset.loop_options]` and `[dataset.map_policy]`.
Top-level `precision` selects both local and global BA arithmetic. Global BA's
matrices and Krylov vectors always use that same precision. Set it before any
TOML table, for example `precision = "float32"` before `[input]`.
`vio.use_double`, `optimization.precision` and `optimization.krylov_precision`
are rejected by the application; there are no per-stage precision overrides.
As in RootBA, scalar recurrence coefficients and objective sums retain float64.
The selected combination is recorded in results and frozen problems.
This setting covers BA arithmetic, not every numeric field in the process:
optical-flow patches remain float32; map storage, loop geometry, pose-graph
optimization and public state interfaces remain float64. The separate frozen-problem
diagnostic tool retains mixed-precision controls for numerical comparisons.

The default `linear_solver = "rootba"` uses staged landmark QR, Jacobian column
scaling, Schur-Jacobi block-preconditioned CG and RootBA's quadratic-model stopping
criterion. Damping retries reuse the linearization. `linear_solver_eta` and
`linear_solver_max_iterations` control PCG. The rig keeps its physical-coordinate
trust region (`damping_coordinates = "physical"`) and checks
`linear_solver_gradient_tolerance` before allowing the model test to stop PCG early.
This retains the previous physical-gradient accuracy criterion for early termination.
At the linear iteration cap, finite truncated steps still go through LM acceptance,
as in RootBA; a completed linear solve does not certify that criterion.
`damping_coordinates = "scaled"` and
`linear_solver_gradient_tolerance = 0` select RootBA's scaled damping and model-only
stopping policy for explicit numerical comparisons. These policies can converge
to different minima even at equal damping values. `linear_solver = "lsqr_reference"`
selects the slow independent reference with its original `lsqr_*` settings.

`loop_options.pose_graph_max_iterations` limits online loop PGO;
`optimization.max_iterations` limits final global BA. `map_policy` supplies
the complete map policy, including the shared pose-graph residual scale.
The paper does not publish that scale. Final global optimization requires
`vio.map = true`; enabling only local VIO does not run global BA.

The shipped LaMAria and UZH configurations run persistent mapping, loop detection
and PGO during input, drain the loop worker, and then run final global BA.
`trajectory` contains the final map-corrected trajectory; `local_trajectory`
retains the causal VIO posterior at the same timestamps and in the same format.
Use different output paths. Compare both against the same reference and report
verified loops, applied PGO updates and global convergence from the recording
and `global_result_path`. A successful process exit alone does not establish
solver convergence or improved accuracy. The application defaults to float64
BA; the paper's single-precision performance needs separate validation.
The IPC flight configuration continues to use local VIO and planner-owned depth.

`metrics_path` writes evaluation data, including SE(3)-aligned ATE, RTE separated
by six original camera frames, and success over requested input frames. Missing
estimates remain in the denominator. When `max_frames` limits an experimental
run, the evaluation input is truncated to the same prefix. GT timestamps and
poses use the dataset reader's convention directly. `max_ground_truth_gap_ns`
defaults to 110000000 ns for GT interpolation, following the Basalt association
limit; this limit is not specified by the RootSLAM paper.

The dataset runner also accepts `marg_data` (a new cache directory containing
original binary marginalization records and the entire dataset's `gt.cereal`),
`debug_directory` (original estimator statistics and `stats_vio.json` runtime/ATE
samples), and `result_path` (original
`rms_ate`, `num_frames`, `exec_time_ns` JSON). The original result uses local
Basalt states and its fixed SVD association rules; `num_frames` counts the full
dataset even when `max_frames` truncates processing. Its timer excludes input
loading, final global optimization, and result writing. `trajectory_format`
selects `"tum"` (default), `"euroc"`, `"kitti"`, or `"lamaria"`. Optional `groundtruth` writes
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
`[dataset.optimization] problem_archive`. Supply `problem`, a new `output` JSON path,
`precision` (`"float32"` or `"float64"`), and `krylov_precision` (`"native"` or
`"float64"`). It restores identical poses, landmarks, observations, calibration,
and solver settings before applying the requested precision. Optional
`max_iterations` changes the outer iteration budget; `linear_system_archive` (LSQR reference only)
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
