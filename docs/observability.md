# Observability

This is the single source of policy for observation registration, publishing, recording and validation.
See [architecture](architecture.md) for algorithm and coordinate responsibilities and
[development](development.md) for builds and process ownership. Defaults and limits live in source and configuration.

## Registration and change process

Every observation change must include these steps in the same review:

1. State the diagnostic question, consumer and retention requirement. Reuse existing observations.
   Remove temporary probes when their investigation finishes.
2. Use business topics for business data, `Firefly/VizScalars` for numeric diagnostics,
   `Firefly/Viz` for spatial diagnostics, `Firefly/VizVoxels` for maps, and `Firefly/Log` for human-readable events.
   Do not put per-point arrays or solver matrices in TextLog, or enlarge business payloads for a viewer.
3. Register numeric fields, order and units in [mimir/schema.rs](../apps/mimir/src/schema.rs).
   Register spatial paths and allowed payload kinds there too. Unknown paths, kinds and widths fail recording.
   Dynamic paths need bounded syntax; frame and landmark IDs must not create unbounded entity paths.
4. Update the topic and observation-group catalog below with owner, consumer, clock, frame and enabling condition.
   Wire-layout changes require synchronized Rust and Python mirrors and actual IPC interoperability tests.
   There is one canonical message schema, with no parallel version-suffixed types; rebuild and restart all peers together.
5. Gate optional work at the producer, before expensive computation or payload construction.
   All logging dispatches must honor the same `RUST_LOG` filter so `log_enabled!` is a reliable gate.
   Enable specific targets instead of global debug for hover runs.
6. Measure peak messages and bytes, bursts, producer cost, receive continuity, write throughput and queue high-water.
   Queues absorb bursts; they cannot fix sustained consumer overload. Fix accidental production and serialization overhead.
   Increasing queues, overwriting diagnostics or discarding failed samples is not an overload fix.
7. Test names, layouts, units, disabled gates, exact timestamps and values, loss detection and shutdown.
   Write and read a real RRD. Report skipped tests separately from passed tests; startup is not flight acceptance.

## Data contract

| Data | RRD representation | Time and integrity |
|---|---|---|
| Numeric diagnostics, enums and bit masks | Scalars plus named SeriesLines | Original measurement `sim_time`; preserve every row, including duplicate timestamps |
| Poses, keypoints and maps | Transform3D, Points2D, VoxelGridMap | Explicit source coordinates; display conversions never feed estimation |
| Process events | `logs/<app>` TextLog | `sim_time` if available, otherwise `wall_time` |
| Stereo images | `cameras/{left,right}/image` Image | Lossless grayscale at capture time; business camera subscriptions are latest-value |
| Keypoints | `cameras/{left,right}/points` Points2D and `track_id` | Original image timestamp, IDs and detected count retained; display coordinates add half a pixel |
| Configuration and calibration | `mimir/config`, static camera components | Only data constant for the recording is static |

Numeric state codes remain available for scoring; labels and enum meanings come from the producer definitions
in `firefly-flight`. Mimir records numeric state samples without an additional state-only stream.
Ground truth, local odometry and map odometry retain their original frames. Visual separation between
their paths is not an aligned position-error metric.

## IPC topic catalog

Rust layouts and capacities live in [firefly-pubsub](../crates/firefly-pubsub/src/lib.rs).
Python wire mirrors live in [firefly-mujoco/messages.py](../packages/firefly-mujoco/src/firefly_mujoco/messages.py).
This catalog covers all production topic constants; test topics use isolated names.

| Topic | Producers → consumers; purpose | Recording policy |
|---|---|---|
| `Firefly/VizScalars` | FC, VIO, render, localization, evaluation → mimir | Every registered numeric row; publisher sequence checked |
| `Firefly/Viz` | VIO, render, planner, localization, evaluation → mimir | Registered spatial observations; sequence checked |
| `Firefly/VizVoxels` | planner → mimir | Complete ordered map updates; missing chunks fail |
| `Firefly/Log` | Rust applications, sim, evaluation → mimir | Sequence and producer loss count checked |
| `Firefly/GroundTruth` | sim → render, mimir, evaluation | Pose and trajectory; independent scoring samples |
| `Firefly/CorrectedOdometry` | VIO/localization → mimir, evaluation | Map-frame pose and trajectory |
| `Firefly/VioAttitude` | VIO prediction → FC, mimir | Bias summary; control consumers remain independent |
| `Firefly/CameraLeft`, `Firefly/CameraRight` | render/sensors → VIO, mimir | Lossless grayscale samples when images are enabled |
| `Firefly/CameraPair` | render → image consumers | Pair notification, not separately recorded |
| `Firefly/Depth` | render/sensors → planner-owned EGO mapper | No duplicate depth display |
| `Firefly/Imu` | sim/sensors → VIO, FC | Not subscribed by the ordinary recorder |
| `Firefly/Odometry` | VIO → control, planning, evaluation | Pose diagnostics; evaluation explicitly subscribes |
| `Firefly/Navigation` | VIO/localization → optional consumers, mimir | Atomic map←odom relation; mimir records changes only and uses bounded startup matching, without duplicating the pose stream |
| `Firefly/Reference` | trajectory output → FC, evaluation | Evaluation records target pose and velocity |
| `Firefly/Control` | FC → actuators/sim, evaluation | Motor evidence and FC summaries |
| `Firefly/PlantState`, `Firefly/Airframe` | sim/airframe → FC, system consumers | Independent evaluation evidence; never VIO input |
| `Firefly/Command` | ffctl/evaluation → FC | Results and rejection reasons in event logs |
| `Firefly/Goal` | ffctl/evaluation → planner | Position and optional terminal yaw in local odom coordinates; yaw in radians, CLI in degrees |
| `Firefly/MandatoryStop` | external tools → planner | Stop command; outcome in planner state and logs |
| `Firefly/Trajectory`, `Firefly/PlannerHeartbeat`, `Firefly/TrajectoryControl`, `Firefly/PlannerStatus` | planner/FSM ↔ trajectory output/evaluation | Atomic xyz/yaw polynomials with shared piece times and a stationary endpoint; heartbeat loss finishes the committed stop; status means scheduled completion, not measured arrival |
| `Firefly/Features`, `Firefly/PoseObservation` | feature/matching → localization | Business localization input, not duplicated by mimir |

Latest-value business channels are not complete raw sensor archives. Start mimir first and stop it last.
`RUST_LOG=info,planner-motion=debug` adds one bounded proposal-summary log per motion coordination
attempt: initial state, heading decision, first sampled rejection (window, occupied or V/A/J), and
first unobserved sample as a separate coverage diagnostic. Unknown space is not a rejection under
EGO's optimistic obstacle policy. It uses `logs/planner` and the existing log budgets; it does not record per-voxel masks or
per-sample checks. Leave it disabled for ordinary operation; incomplete log recording fails validation.
Sequence checks cover the subscription interval, not data before startup or the unknown tail of a crashed producer.
Rust log queue/send loss is counted in `LogMessage.dropped_records`; the pump reports loss even without a later
business log. Known loss makes the recording fail.

## Observation groups

| Group | Owner, purpose and enabling condition |
|---|---|
| `fc/debug/*`, `fc/imu/*`, `fc/arming/*`, `fc/failsafe/*` | FC position, attitude, control, freshness and safety summaries; producer-defined cadence |
| `fc/standby_eskf/*` | Standby health, bias and variance in its documented local gravity frame |
| `fc/loop/*` | Per-control-tick timing, angular rate and allocation; only `fc::diagnostics=debug` |
| `vio/posterior/*` | Each VIO posterior pose, world velocity and IMU biases |
| `vio/prediction/*` | Shared IMU prediction pose, velocity, angular rate and posterior age |
| `vio/diagnostics/*` | Per-frame identity counts, stereo ratio, pixel residuals and optimization deltas |
| `vio/diagnostics/keypoints/{left,right}` | Camera overlays; one recorded point representation |
| `vio/diagnostics/keypoints/active_left` | Additional associated-landmark overlay, only `vio-diagnostics=debug` |
| `rootslam/*` | Stereo enqueue-to-posterior and IMU publishing timing |
| `vio/odom`, `vio/traj`, `vio/map/{pose,traj}`, `vio/gyro_bias` | Local and map poses at measurement time, derived trajectories and IMU bias summary; original frames are retained |
| `render/capture/*` | Capture-to-publish stage timing at the original capture timestamp |
| `corr/*`, `vision/debug/map_query_wall_s` | External localization gates, disagreement, correction and query timing |
| `plan/*` | Global guides, accepted executable local paths including their stopping tail, poses, obstacle planes and maps for display |
| `plan/collision` | Planner-owned depth map after obstacle inflation, in odom coordinates at the planner simulation timestamp; bounded voxel snapshots/deltas for checking collision geometry, using the existing map display cadence and budgets |
| `acceptance/*` | Scoring evidence; display simplification must preserve it and all acceptance thresholds |
| `mimir/*`, `logs/*` | Configuration, queue high-water, lifecycle, failures and process events |
| `mimir/status/{fc/tick_rate,vio/processing,vio/posterior_age,vio/landmarks,camera/publish_rate,camera/publish_latency}` | Six independent recorder-derived numeric documents from existing scalar observations; at most 2 Hz per entity in measurement time, six cached values and one constant-space publication counter, no additional IPC topic |
| `frames/{map,odom,world}` | Recorder-owned display transforms: root←map identity, temporal map←odom from Navigation, and optional constant map←world startup alignment; never estimation or control inputs |

### Spatial frames

All registered 3D payloads declare their source frame in `mimir/schema.rs`; adding a spatial entity
requires registering both its payload kind and its frame. `Transform3D.parent_frame` labels raw poses;
`CoordinateFrame` labels lines, arrows and voxels in the same row, so recursive clears cannot lose
the association. Raw positions, orientations, timestamps and acceptance evidence remain unchanged.

| Entities | Frame |
|---|---|
| `gt/{pose,traj}`, `acceptance/gt` | Simulator `world` |
| `vio/{odom,traj}`, `vio/posterior/pose_imu_world`, `vio/prediction/pose_body_odom`, `acceptance/odom` | Local estimator `odom`; posterior “world” means the estimator's local world |
| `vio/map/*`, `corr/*`, `acceptance/corrected` | Navigation `map` |
| Spatial `plan/*`, `acceptance/reference` | Local planner `odom`; independent of optional Navigation corrections |
| `cameras/{left,right}` | Extrinsics relative to the moving `tf#/vio/prediction/pose_body_odom` body frame; intrinsics and pixel overlays retain camera coordinates |

The Viewer root connects to `map`. `map←odom` is recorded at the Navigation measurement time on
alignment/revision changes. A backward timestamp/revision, changed alignment without a revision, or
changed external-map mode ends the recording as incomplete; start a new recording for a new session.

For autonomous simulation, mimir can connect `world` to `map` using one pair of truth and Navigation
poses at the same measurement nanosecond during stationary initialization near the odom origin.
Both source timestamps are converted to integer nanoseconds, matching VIO and the RRD timeline;
floating-point seconds may differ after VIO's integer-clock round-trip. Different nanoseconds do not match.
Only yaw and translation are aligned, once; roll/pitch disagreement, scale error and subsequent drift
remain visible. This display convention removes the arbitrary initial origin/heading and is not an
absolute localization accuracy claim. Matching storage is bounded in code; it does not enlarge IPC
or writer queues, discard diagnostic samples, interpolate timestamps, or fit the full trajectory.
Observed motion or a pose away from the local origin closes the startup window. Missing matches and
external-map sessions leave `world` disconnected, with a warning in `logs/mimir`; inspect it with a
separate world-frame 3D view. This is a startup heuristic: Navigation carries no session-start marker,
so starting the recorder after a return to the estimated origin can still qualify for alignment.
Start recording before producers; do not use this display anchor to score absolute drift.

Hover evidence includes truth, posterior, prediction, targets/errors, velocity, bias, posterior age,
attitude source, safety state, per-frame residuals and optimization corrections.
The fourth `vio/diagnostics/tracks` field counts associated landmark IDs; it is not the number of
whitened, robustly weighted effective constraints. Identity/initialization differences between estimators
must not be presented as equivalent accuracy evidence. Full per-point depth/weight
does not currently have a complete mainline contract; additions must
follow the registration process with bounded structured payloads.

Residuals are unweighted pixel residuals of valid projections: count, norm RMS and mean x/y residual.
Zero-count placeholders must not be interpreted as zero error. Position/velocity deltas are local
optimization after-minus-before; rotation is the corresponding rotation vector. They exclude frontend
initialization and loop synchronization. RootSLAM also reports graph and keyframe summaries.

`RUST_LOG=info,vio-gauge=debug` enables `vio/diagnostics/yaw_updates_rad` in RootSLAM.
It distinguishes a shared window rotation from a relative attitude correction: world-Z components
of `Log(R_after * R_before.inverse())` for the latest and oldest retained poses, followed by the
window mean and population standard deviation. These are rotation-vector components, not Euler yaw.
The consumer is RRD odometry diagnosis, with the image measurement `sim_time` and odom axes.
The producer gate precedes the bounded pose snapshot (at most the configured VIO window);
one four-scalar row is emitted per processed frame, bounded by the camera rate, with no extra topic
or queue capacity. Production is off by default; retain every enabled row through shutdown.

`RUST_LOG=info,vio-solver=debug` enables RootSLAM's `vio/diagnostics/costs_before`,
`costs_after`, and `solver_termination` for RRD convergence diagnosis. Costs cover the complete
active optimization problem before and after optimization: weighted robust vision, inertial,
bias random walk, and marginal prior. The prior omits its constant term and may be negative;
compare before/after within one frame, not absolute totals between different windows.
Termination contains attempted steps, rejected steps, reason (1: cost decrease tolerance,
2: step tolerance, 3: iteration budget, 4: damping limit), and last attempted step infinity norm.
Warmup frames without optimization emit none. The producer gate precedes the two extra objective
evaluations; each optimized image emits three four-scalar rows at its `sim_time`, bounded by the
camera rate, with no extra topic or queue capacity. Production is off by default; retain every
enabled row through shutdown. Costs are dimensionless and the step mixes the solver's state units.

`RUST_LOG=info,vio-input=debug` records every IMU measurement admitted by the VIO runtime as
`vio/input/gyro_rad_s` and `vio/input/accel_m_s2`, in IMU axes at the original measurement
`sim_time`. Together with the complete timestamped left/right keypoint IDs and positions, these
are inputs for backend replay; they are not predictor outputs or corrected IMU measurements.
The runtime owns the producer and gates publication before constructing either row. Production
is off by default. The simulated 1 kHz IMU adds two three-scalar rows per sample (2,000 rows/s);
no subscriber, queue capacity, or retention policy changes. A diagnostic run must verify matching
gyro/accel timestamps, the expected IMU sampling intervals, coverage of every processed image,
publisher continuity, and recording completion before treating its inputs as replayable.

Offline backend comparisons retain derived states in a separate RRD at the source image
`sim_time`: `{rust,upstream}/pose` is the IMU pose in odom, `p` and `q` preserve its
float64 position and XYZW quaternion, and `v`, `bg`, `ba` are world velocity, IMU gyroscope
bias and IMU accelerometer bias respectively. `rust/active_ids` retains current-frame IDs
associated with landmarks, with the same semantics as the online active-keypoint diagnostic. There is one row
per processed image, no live IPC producer. Record both implementations' configuration adapters;
reconstructed viewer keypoints and the initial pose carry float32 rounding and must not be
described as bit-exact reproduction of the original online run.
The unified dataset entry uses the existing RootSLAM log IPC and diagnostics registry;
it does not publish flight odometry topics. Its `sim_time` is the dataset measurement
time in seconds, without a host-time offset. Dataset camera times are converted
to the IMU clock with the calibration offset before estimation; exported trajectories
retain these integer nanoseconds (TUM serializes seconds). Raw IMU and GT retain
their supplied timestamps; record the GT alignment assumptions separately.
Benchmark evidence includes the clock offset, source/calibration fingerprints,
configuration, requested/produced frame counts, process exit and RRD completion.
Native trajectory files are algorithm outputs; runtime diagnostics still belong in RRD.
GT-based offline ablations use separate `backend_matched_raw.rrd` and
`backend_matched_ideal.rrd` recordings with the same state entities. Both retain exactly the same
track IDs, camera memberships and timestamps; only the ideal arm replaces pixels with projections
of fitted static points through GT camera poses. These are diagnostic counterfactual inputs,
never a production estimator input or flight acceptance result. Report selection counts and
keep the unmodified recording for comparison.
Offline temporal-flow reference comparisons use `flow/counts` (source points, common accepted
IDs, online-only IDs, reference-only IDs) and `flow/error_px` (median, p90, maximum location
difference on common IDs), one row per compared image pair at the target image `sim_time`.
They compare identical recorded images and source points and do not include corner detection.
Offline render counterfactuals reuse `cameras/{left,right}/image` in separate RRDs, indexed by
the original pose timestamp; transport request timestamps are only used to pair replies.
Record the renderer and scene hashes, requested/received frame counts and process exit codes.
The bounded offline backend probe may include `rust/landmarks` as a JSON TextDocument per
selected frame: landmark ID, host timestamp, inverse distance, and each retained observation's
timestamp, camera, residual and Huber weight. This has no online producer; keep the probe time
windows explicit and never enable full per-landmark payloads in production recordings.

`rootslam/frame_processing_ms` starts at stereo enqueue and ends at posterior consumption in the IMU
thread, including queue time.
`RUST_LOG=info,basalt-profile=debug` records stage events in `logs/firefly-rootslam`.
Join by frame timestamp and process-monotonic boundaries; do not add nested durations or percentiles.
`rootslam-global-profile=debug` records offline global BA stage times in milliseconds:
linearization, damping, preconditioner, linear solve including preconditioner,
back substitution and trial cost. The solver owns these events; benchmark analysis consumes
them from `logs/firefly-rootslam` on the process wall clock. There are at most six events per
LM attempt plus the initial linearization, each below 512 bytes. Timing is disabled at the
producer by default. The preconditioner is nested in the solve and must not be summed twice.
No matrices or per-landmark arrays are logged. Retain the recording with the frozen problem,
configuration and result; verify complete stage events and clean RRD shutdown after replay.
Global BA reports optimized and deferred landmark counts once at INFO level (below
512 bytes). Deferred landmarks lack two distinct camera views; they remain in the map.
Use these counts to distinguish the map size from the actual optimization problem.
`RUST_LOG=info,planner-profile=debug` records planner callbacks taking at least 10 ms in
`logs/planner`, including sensor polling, map synchronization, execution, safety and voxel display.
Each event contains the stage and elapsed wall time; nested stages must not be summed.
Production is off by default, bounded by the existing 100/20/10 Hz callback rates and 512-byte log limit.
Depth-input expiry/termination logs one warning when planner enters the latched emergency state,
including odometry measurement time, in the existing `logs/planner` stream (512-byte limit).
`rootslam-loop=debug` adds one retrieval summary per keyframe and one rejection/acceptance event
per tested loop candidate to `logs/firefly-rootslam`. Fields are frame IDs, descriptor/match/inlier
counts and rejection stage; no images or per-feature arrays. At most `1 + max_candidates` events
per detection job, bounded by the configured keyframe/image rate and the existing 512-byte log limit.
`render/capture/stages_s` records submit/readback/assembly/queue timing; `publish_s` records processing,
publication and total timing. Posterior age also includes visual processing and propagation scheduling.
Default RootSLAM runs as VIO. Planner consumes raw body odometry and depth; global map corrections
are not applied to its occupancy or trajectories. No online loop-commit observations are registered.

## Recorder architecture and operation

Mimir owns IPC subscriptions in one receive thread, copies each loan once into a bounded channel,
then releases the loan. A single writer groups scalar and typed observation rows by entity into
columnar Rerun chunks. Typed rows use Rerun's `PendingRow::many_into_chunks` so images, poses,
trajectories and metric cards do not each create a chunk per sample.
Count, byte, entity, publisher and voxel budgets are bounded in
[Options](../apps/mimir/src/config.rs); [mimir.toml](../configs/mimir.toml) contains overrides.
All batches flush by row limit or periodic deadline. Typed rows also share the `batch_bytes` memory
budget; reaching it flushes pending rows synchronously. A single oversized row is written directly
after flushing, without retaining it in the batch. Static metadata is written directly.
Repeated timestamps, order within an entity, field order and values are retained. Clear flushes
previous rows before clearing entities. Batching must not resample or discard observations.
Synchronous RRD encoding writes directly to disk without an extra unbounded SDK queue.

Voxel chunks must arrive in order and form a complete update before a map snapshot is written.
Deltas require a baseline and matching geometry. Planner display coarsening remains separate from its
planning grid. Incomplete, reordered or over-budget updates fail recording.

Images use native lossless grayscale Image components, with no external video encoder.
Keypoints retain their capture timestamps and track IDs; display pixels and principal points add 0.5
because the algorithms use integer pixel centers and Rerun uses image boundaries.
Viewer history selection must respect image timestamps; a visually adjacent point set is not proof
that it belongs to the current image. No depth display, field mesh or live static preview is recorded.
Mimir writes files; inspect them with the standard Rerun viewer.

### Default viewer layout

[configs/firefly.rbl](../configs/firefly.rbl) is the shared layout for application ID `firefly`.
Mimir embeds it as the default in each new RRD. Rerun may restore an application-specific user layout;
load the checked-in RBL explicitly to select the shared layout, or use **Reset to default blueprint**
on a new recording. Use the workspace viewer to match the recording SDK version:

```bash
cargo build --release -p mimir
./target/release/mimir --save logs/run.rrd
uv run rerun logs/run.rrd configs/firefly.rbl
```

The main area shows the map/trajectories and recorded stereo images. Small independent numeric cards
show FC control frequency (Hz), VIO processing and posterior age (ms), associated landmark IDs,
camera publication rate (FPS) and capture-to-publication latency (ms). Each card has its own entity and
view; values are not concatenated into one document. Ordinary bold text keeps the numbers compact.
The timeline cursor selects the measurement-time snapshot; a stopped recording holds its last snapshot.
Missing/nonfinite or more-than-two-second-old values display `—`.

Camera FPS counts strictly increasing `render/capture/publish_s` timestamps over at least one second
of measurement time, using the number of intervals divided by elapsed time. Duplicate/out-of-order
timestamps do not increment the counter; a gap longer than two seconds restarts the window. This is
the simulated stereo publication cadence, not wall-clock throughput, recorder receipt rate or the
inverse of VIO processing time. It works with image recording disabled; physical camera sources that
do not emit this observation display `—`. Publication latency uses `publish_s.total_s`; landmarks use
`vio/diagnostics/tracks.landmark_ids`. Other cards show the latest source value.
The display adds at most twelve small document rows per measurement second after its initial snapshot,
with constant-size state. Full source series and images remain recorded unchanged; load tests verify
queue budgets, source continuity and final recording completion with these derived cards enabled.
Tracking/control charts show track IDs, reprojection RMS and controller tracking error; the latter
is target-versus-estimated-position error, not ground-truth drift. Timing, estimator details and logs
are tabs. Thrust and other raw observations remain available in the entity tree.

In the default 3D view, green is the global guide path, coral red is the local planned trajectory
and its velocity arrows, blue is ground truth, yellow is local odometry, and purple is map-frame
navigation. The odometry and navigation histories overlap when the navigation correction is identity.

Edit the layout in the Viewer and save its blueprint back to `configs/firefly.rbl`; rebuild mimir to
embed it. There is one layout artifact and no separate Python recorder. Before accepting a layout,
check every selected entity against the observation registry and inspect a real recording screenshot.
Old recordings without `mimir/status/*` need a derived display layer for the numeric cards; changing the
layout alone cannot create missing data. Preserve the original recording when producing such a layer.

Output paths must be new. After producers stop, SIGINT/SIGTERM drains diagnostic subscriptions until empty
(including disconnected publishers), drains the owned channel, flushes all batches,
writes the full RRD footer and syncs the file. Queue overflow, sequence gaps, declared producer loss,
invalid payloads, incomplete maps and writer/close failures return errors and preserve failure evidence
where disk access remains possible. Closing does not consume a queue slot.
RRD records lifecycle and high-water statistics in `logs/mimir`. Index memory grows with chunk count;
it is separate from the bounded pending-data budget.

Raw runtime evidence belongs in `logs/*.rrd`; JSON/HTML may hold configuration, fingerprints and summaries.
Do not create parallel text recordings of runtime diagnostics. CLI help, command results and exported
artifacts may use stdout; this does not replace runtime evidence. Stderr is a diagnostic sink, not archived evidence.
The current CompactReporter writes span summaries to stderr; use RRD stage diagnostics for archived timing.

## Validation

```bash
cargo test -p mimir -p firefly-observability -p firefly-pubsub --all-targets
FIREFLY_TEST_PYTHON="$PWD/.venv/bin/python" cargo test -p firefly-pubsub --test python_interop -- --ignored
uv run pytest tests/evaluation -q
FIREFLY_RUN_TELEMETRY_LOAD=1 uv run pytest tests/system/test_mimir.py -q -s
```

The IPC load test requires release mimir and an idle workspace. It checks exact values and timestamps
after RRD readback and verifies the footer and final completion event. Repeated throughput runs use
fixed inputs and budgets; missing data fails. Queue success is evidence about this recording workload,
not flight safety or hover accuracy. Full flight tests remain governed by their source thresholds.
