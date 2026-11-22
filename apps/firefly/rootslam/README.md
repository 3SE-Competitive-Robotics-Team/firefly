# RootSLAM

RootSLAM provides calibrated stereo/IMU odometry through one application:
`firefly-rootslam --config <path>`. IPC sources serve the flight stack; recorded
sources run public datasets through the same Basalt estimator. Persistent mapping,
loop closure and final global BA are available for dataset evaluation.

For a connected Matrix, use the [two-minute IMU check](https://3se-competitive-robotics-team.github.io/firefly/development/#验证-matrix-imu). For stereo tracking and IPC configuration, see [online operation](#online-operation-and-validation).

## Public-dataset evaluation

Run commands from the workspace root. Prepare the dataset and its matching
calibration, build the release applications, and give each sequence/precision/run
its own directory under `logs/`.

```bash
uv sync --locked --all-packages --extra test
cargo build --locked --release -p firefly-rootslam -p mimir
mkdir -p logs/tumvi_room2/float64
```

### Inputs and calibration

| Dataset | Input layout | Calibration | Example |
|---|---|---|---|
| [LaMAria](https://lamaria.inf.ethz.ch/slam_datasets) | ASL sequence containing `aria/` | Matching `pinhole_calibrations/<sequence>.json` | [LaMAria config](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/firefly-rootslam-dataset.toml) |
| [UZH FPV](https://fpv.ifi.uzh.ch/datasets/) | Snapdragon archive with `left_images.txt`, `right_images.txt`, `imu.txt` | Matching Kalibr directory or Basalt JSON | [UZH config](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/firefly-rootslam-uzh.toml) |
| [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) | Sequence containing `mav0/` | Basalt `euroc_ds_calib.json` | [EuRoC config](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/firefly-rootslam-euroc.toml) |
| [TUM-VI](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset) | Calibrated 512×512, 16-bit EuRoC export | Basalt `tumvi_512_eucm_calib.json` | [TUM-VI config](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/firefly-rootslam-tumvi.toml) |
| KITTI | Native sequence accepted by the Basalt KITTI reader | Matching Basalt cereal JSON | Select `input.format = "kitti"` |

The Basalt calibration reference is commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`. Example paths point to a sibling
Basalt checkout; set them to the actual calibration location on your machine.

Use the official [LaMAria downloader](https://github.com/cvg/lamaria/blob/main/tools/download_lamaria.py)
with `--sequences R_01_easy --type asl`. ASL images use the exported pinhole
calibration. Its XYZW body transforms are converted to IMU-from-camera; noise
densities, IMU rate and gravity come from the same file.

UZH imports `camchain-imucam-*.yaml` and `imu.yaml`, converts Kalibr
pinhole/equidistant to Kannala–Brandt4, and inverts `T_cam_imu`.
The stereo frame uses the left exposure time; the accepted difference between
camera offsets is at most 1 ms. Ground truth is used by evaluation.

TUM-VI's calibrated exports already contain aligned clocks, calibrated IMU
measurements and IMU-frame ground truth. The reader consumes these exported values
directly. EuRoC uses the archive ground truth; the original V1_01 reference has a
[documented orientation issue](https://docs.openvins.com/gs-datasets), which belongs
in comparisons involving relative pose error.

Keep downloaded inputs, download manifests, hashes and run artifacts locally.
Verify the publisher's checksum when available and record the source revision and
checksum for mirrors. Plan space for the archive, extraction and outputs; sequential
sequence pairs allow raw inputs to be released after both precision results are verified.

### Configure a run

Save the following as `logs/tumvi_room2/float64/config.toml`, adjusting the input
and calibration paths to your installation:

```toml
precision = "float64"

[input]
type = "dataset"
format = "euroc"
path = "datasets/tumvi/dataset-room2_512_16"
calibration = "../../basalt/data/tumvi_512_eucm_calib.json"

[vio]
optical_flow_detection_grid_size = 20
map = true
loops = true

[dataset]
trajectory_format = "tum"
trajectory = "logs/tumvi_room2/float64/trajectory.txt"
local_trajectory = "logs/tumvi_room2/float64/local_trajectory.txt"
metrics_path = "logs/tumvi_room2/float64/metrics.json"
global_result_path = "logs/tumvi_room2/float64/global_result.json"
global_optimization = true

[dataset.optimization]
problem_archive = "logs/tumvi_room2/float64/global_problem.json"
```

Omitting `max_frames` processes the full input. Each output file and optional
statistics/archive directory has its own fresh path, separate from input data.
The startup checks resolve path aliases and validate output ownership; writers
create new files. Existing runs remain available for comparison.

Start Mimir in one terminal. It prints the recording path, following the
[shared archive naming contract](https://3se-competitive-robotics-team.github.io/firefly/mimir/):

```bash
./target/release/mimir
```

Run the estimator in another terminal:

```bash
RUST_LOG=info ./target/release/firefly-rootslam --config logs/tumvi_room2/float64/config.toml
```

After the estimator exits, stop Mimir with Ctrl-C so it drains and closes the
recording. Check the exit status, RRD integrity, trajectory coverage and optimization
result together; substitute the directory returned by Mimir:

```bash
uv run rerun rrd verify "$run"/rrd/*.rrd
cargo xtask view "$run"
```

The dataset pipeline feeds image and IMU producers, collects posterior states,
drains the mapping worker, runs final BA when enabled, then exports results.
The local trajectory is available before final BA, supporting diagnosis of a
global-stage failure. Cancellation requests join owned workers; final optimization
already in progress completes at its existing synchronous boundary.

### Score and reproduce

| Artifact | Meaning |
|---|---|
| `local_trajectory` | Causal VIO posterior at calibrated camera times |
| `trajectory` | Final map-corrected trajectory, or local trajectory in VIO mode |
| `metrics_path` | Final SE(3) ATE RMSE in metres, six-input-frame RTE, associations and frame coverage |
| `global_result_path` | Optimized geometry, arithmetic settings, costs, iteration counts and convergence |
| `problem_archive` | Frozen calibration, poses, points, observations and solver settings for replay |
| RRD | Runtime stages, errors, mapping events and recorder completion |

Native trajectory metrics use rigid alignment and the configured GT interpolation
gap, defaulting to 110 ms. Coverage counts estimates against requested input camera
timestamps. RTE separates poses by six original input frames. Report export status,
coverage and numerical convergence as separate fields.

For LaMAria, export `trajectory_format = "lamaria"` and evaluate with the
[official evaluation tools](https://github.com/cvg/lamaria). Its challenge scores,
recall and scale-aligned controlled-data evaluation have their own protocols;
SE(3) ATE against pGT is a useful separate diagnostic.
For UZH, the [RPG evaluator](https://github.com/uzh-rpg/rpg_trajectory_evaluation)
provides the competition's 40/60/80/100/120 m relative errors. Record associations
and results by segment length alongside native ATE. Leaderboard comparisons use
the same sequences, sensors and metric.

For each run, retain the resolved configuration, commit and dirty-tree status,
release-binary hash, calibration hash, input version/checksums, tool versions,
hardware and runtime conditions. Standard commands include `git rev-parse HEAD`,
`git status --porcelain` and `shasum -a 256 <file>`. Keep these fingerprints and
scoring summaries with the local run artifacts. RRD contains runtime diagnostics.

For an FP32 pair, use a fresh directory, set top-level `precision = "float32"`,
and update every output path. This selects local BA, IMU preintegration,
marginalization and global BA arithmetic together. Map storage, loop geometry,
PGO and public state interfaces use FP64; optical flow uses FP32. Full-system
precision runs compare independently generated maps.

To isolate global solver arithmetic, replay one frozen problem with
`firefly-basalt-tools --config <replay.toml>`:

```toml
task = "global-ba"
problem = "logs/tumvi_room2/float64/global_problem.json"
output = "logs/tumvi_room2/float64/replay_result.json"
precision = "float64"
krylov_precision = "native"
```

The replay restores the archived solver settings. Version 1 archives without a
preconditioner selector retain Schur-Jacobi. Set `preconditioner = "cluster_jacobi"`
in the replay TOML to compare the current clustered method explicitly; the
selected method is included in the result. Its optional `max_iterations`
sets the diagnostic outer budget. Use a fresh output for every experiment and
compare costs, geometry, convergence and timing. Runtime stage timing and total
application duration describe separate workloads.

### Reference measurements

The M5 reference run at `ecf5cbd` used full sequences, grid size 20, mapping, loops and global bundle adjustment. All 28 runs covered the local frames. Nine global solves converged, 15 exported a trajectory at the iteration limit, and four stopped at initial linearization. Compare accuracy together with solver status, precision and calibration. Per-sequence errors, timings and run conditions are in [issue #149](https://github.com/3SE-Competitive-Robotics-Team/firefly/issues/149).

## Configuration and clocks

`input.type` selects `ipc` or `dataset`; dataset `format` selects `euroc`,
`uzh`, `kitti` or `lamaria`. `vio.vio_config` loads a Basalt configuration
archive; `optical_flow_detection_grid_size` overrides its detection cell width.
Dataset-specific options belong in `[dataset]`; IPC statistics use
`vio.stats_directory`, and optional external localization belongs to IPC.

The ingress preserves native image dimensions, integer nanosecond timestamps and
calibrated noise densities. Camera times follow
`t_imu = t_camera + cam_time_offset_ns`; file lookup retains the camera clock,
and estimation/export use the IMU clock. Raw IMU and GT retain their supplied
timestamps. Dataset output is the exposure-time posterior; live flight additionally
uses high-rate IMU prediction.

`dataset.use_imu` selects VIO (default) or VO. Mapping follows `vio.map`,
loop closure follows `vio.loops`, and final optimization follows
`dataset.global_optimization` when mapping is enabled. `dataset.loop_options`
and `dataset.map_policy` configure those modules. The global solver controls and
factor contracts are described in the [mapping crate](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-rootslam-mapping/README.md).

Optional Basalt-compatible exports are `marg_data` (binary marginalization records
and `gt.cereal`), `debug_directory` (statistics), `result_path` (local alignment
RMSE, full input frame count and estimator duration), and `groundtruth` (aligned
GT translations with identity orientations). Native `metrics_path` describes
the requested frame range and final trajectory. Formats are `tum` (seconds/XYZW),
`euroc` (nanoseconds/WXYZ), `kitti` (3×4 matrices), and `lamaria`
(integer nanoseconds/XYZW).

## Additional tools

`firefly-basalt-tools --config <path.toml>` selects a task using `task`.
Each task validates its own inputs and writes the requested outputs.

| Task | Main inputs and outputs |
|---|---|
| `kitti-eval` | `ground_truth`, `estimate`, `output` |
| `time-align` | `dataset_path`, `dataset_type`, `output`; optional paired `calibration`/`mocap_calibration`, `output_ground_truth` |
| `opt-flow` | `calibration`, `dataset_path`, `dataset_type`, optional `vio_config` |
| `mapper` | `calibration`, marginalization `cache`, optional `vio_config`, `trajectory`, `result_path` |
| `global-ba` | Frozen `problem`, new `output`, arithmetic and optional iteration budget |
| `vio-sim` | `calibration`, `output`, optional `vio_config`, `archive`, `marg_data`, `use_imu`, `use_double` |
| `mapper-sim`, `mapper-sim-naive` | `calibration`, `cache`, optional `output` |

The tools provide headless alignment, mapping and numerical workflows. Production dataset
benchmarks use the dataset application above. Original mapper cache consumers expect
the Basalt archive schema, including `gt.cereal`.

To view optimized geometry, pair a frozen problem with its matching result:

```bash
uv run python apps/firefly/rootslam/export_rrd.py --problem logs/tumvi_room2/float64/global_problem.json --replay logs/tumvi_room2/float64/global_result.json --trajectory logs/tumvi_room2/float64/trajectory.txt --output logs/tumvi_room2/float64/map.rrd
```

## Online operation and validation

The default IPC configuration provides local VIO to flight control and planner.
On the 320×240 rig, the left image carries feature IDs across time. Each stereo
pair measures disparity again for every current left track, so a feature that
loses its right observation can recover depth under the same ID. Tracking uses
the full and half-resolution images, with forward/backward and epipolar checks.
An explicit `vio.vio_config` archive selects its own tracking settings. The
frame-to-frame tracker accepts `config.optical_flow_rematch_stereo` for this
matching policy. Archives that omit it use Basalt's temporal right-eye tracking.
Optional sparse mapping maintains its own posterior map; planner owns dense depth
fusion. `--external-localization` assigns corrected-odometry and navigation
publication to the localization process. The online data flow is shown in
[architecture](https://3se-competitive-robotics-team.github.io/firefly/architecture/).

```bash
cargo test -p firefly-calibration -p firefly-basalt -p firefly-rootslam-mapping -p firefly-rootslam --all-targets
cargo clippy -p firefly-calibration -p firefly-basalt -p firefly-rootslam-mapping -p firefly-rootslam --all-targets -- -D warnings
uv run python -B -m unittest discover -s apps/firefly/rootslam/tests -p test_export_rrd.py
```

Full-workspace checks are in the
[development guide](https://3se-competitive-robotics-team.github.io/firefly/development/#测试). Observation ownership and recorder
budgets are described in [observability](https://3se-competitive-robotics-team.github.io/firefly/observability/).
