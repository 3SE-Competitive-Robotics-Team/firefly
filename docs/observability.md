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
| `Firefly/Navigation` | VIO/localization → planner, FC, mimir | Atomic map←odom relation; mimir records changes only and uses bounded startup matching, without duplicating the pose stream |
| `Firefly/Reference` | trajectory output → FC, evaluation | Evaluation records target pose and velocity |
| `Firefly/Control` | FC → actuators/sim, evaluation | Motor evidence and FC summaries |
| `Firefly/PlantState`, `Firefly/Airframe` | sim/airframe → FC, system consumers | Independent evaluation evidence; never VIO input |
| `Firefly/Command` | ffctl/evaluation → FC | Results and rejection reasons in event logs |
| `Firefly/Goal` | ffctl/evaluation → planner | Planning goal, not a plotting topic |
| `Firefly/MandatoryStop` | external tools → planner | Stop command; outcome in planner state and logs |
| `Firefly/Trajectory`, `Firefly/PlannerHeartbeat`, `Firefly/TrajectoryControl`, `Firefly/PlannerStatus` | planner/FSM ↔ trajectory output/evaluation | Business lifecycle; visualization uses plan entities |
| `Firefly/Features`, `Firefly/PoseObservation` | feature/matching → localization | Business localization input, not duplicated by mimir |

Latest-value business channels are not complete raw sensor archives. Start mimir first and stop it last.
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
| `plan/*` | Paths, poses, obstacle planes and maps for display |
| `plan/collision` | Planner-owned depth map after obstacle inflation, in map coordinates at the planner simulation timestamp; bounded voxel snapshots/deltas for checking collision geometry, using the existing map display cadence and budgets |
| `acceptance/*` | Scoring evidence; display simplification must preserve it and all acceptance thresholds |
| `mimir/*`, `logs/*` | Configuration, queue high-water, lifecycle, failures and process events |
| `mimir/status/{fc/tick_rate,vio/processing,vio/posterior_age,vio/landmarks,camera/publish_rate,camera/publish_latency}` | Six independent recorder-derived numeric documents from existing scalar observations; at most 2 Hz per entity in measurement time, six cached values and one constant-space publication counter, no additional IPC topic |
| `frames/{map,odom,world}` | Recorder-owned display transforms: root←map identity, temporal map←odom from Navigation, and optional constant map←world startup alignment; never estimation or control inputs |
| `rootslam/mapping/state` | Four cumulative counters: processed detection queries, geometrically verified loops, accepted PGO transactions, rejected PGO results; 1 Hz at the current IMU measurement time, constant-size state, existing scalar IPC and diagnostic switch |
| `vio/navigation/revision` | Revision actually published by the VIO navigation owner; 10 Hz at the same IMU measurement timestamp, one scalar; absent for external-localization ownership |

Online map-correction acceptance requires both an accepted PGO transaction and a positive published
navigation revision. A verified loop, solver return, or prepared transform alone is insufficient.
Mapping source/anchor times and revisions are logged once per prepared correction; planner logs the
revision when invalidating geometry and handing off a replacement trajectory. These bounded lifecycle
events use the existing log budget. No new IPC topics, raw depth archive, or per-landmark streams are added.

### Spatial frames

All registered 3D payloads declare their source frame in `mimir/schema.rs`; adding a spatial entity
requires registering both its payload kind and its frame. `Transform3D.parent_frame` labels raw poses;
`CoordinateFrame` labels lines, arrows and voxels in the same row, so recursive clears cannot lose
the association. Raw positions, orientations, timestamps and acceptance evidence remain unchanged.

| Entities | Frame |
|---|---|
| `gt/{pose,traj}`, `acceptance/gt` | Simulator `world` |
| `vio/{odom,traj}`, `vio/posterior/pose_imu_world`, `vio/prediction/pose_body_odom`, `acceptance/odom` | Local estimator `odom`; posterior “world” means the estimator's local world |
| `vio/map/*`, `corr/*`, spatial `plan/*`, `acceptance/{corrected,reference}` | Navigation `map` |
| `cameras/{left,right}` | Extrinsics relative to the moving `tf#/vio/prediction/pose_body_odom` body frame; intrinsics and pixel overlays retain camera coordinates |

The Viewer root connects to `map`. `map←odom` is recorded at the Navigation measurement time on
alignment/revision changes. A backward timestamp/revision, changed alignment without a revision, or
changed external-map mode ends the recording as incomplete; start a new recording for a new session.

For autonomous simulation, mimir can connect `world` to `map` using one pair of truth and Navigation
poses with exactly equal source timestamps during stationary initialization near the odom origin.
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
must not be presented as equivalent accuracy evidence. Full per-point depth/weight, objective-component
costs and optimization stop reasons do not currently have a complete mainline contract; additions must
follow the registration process with bounded structured payloads.

Residuals are unweighted pixel residuals of valid projections: count, norm RMS and mean x/y residual.
Zero-count placeholders must not be interpreted as zero error. Position/velocity deltas are local
optimization after-minus-before; rotation is the corresponding rotation vector. They exclude frontend
initialization and loop synchronization. RootSLAM also reports graph and keyframe summaries.

`rootslam/frame_processing_ms` starts at stereo enqueue and ends at posterior consumption in the IMU
thread, including queue time.
`RUST_LOG=info,basalt-profile=debug` records stage events in `logs/firefly-rootslam`.
Join by frame timestamp and process-monotonic boundaries; do not add nested durations or percentiles.
`RUST_LOG=info,planner-profile=debug` records planner callbacks taking at least 10 ms in
`logs/planner`, including sensor polling, map synchronization, execution, safety and voxel display.
Each event contains the stage and elapsed wall time; nested stages must not be summed.
Production is off by default, bounded by the existing 100/20/10 Hz callback rates and 512-byte log limit.
Depth-input expiry/termination logs one warning when planner enters the latched emergency state,
including navigation time and revision, in the existing `logs/planner` stream (512-byte limit).
`rootslam-loop=debug` adds one retrieval summary per keyframe and one rejection/acceptance event
per tested loop candidate to `logs/firefly-rootslam`. Fields are frame IDs, descriptor/match/inlier
counts and rejection stage; no images or per-feature arrays. At most `1 + max_candidates` events
per detection job, bounded by the configured keyframe/image rate and the existing 512-byte log limit.
`render/capture/stages_s` records submit/readback/assembly/queue timing; `publish_s` records processing,
publication and total timing. Posterior age also includes visual processing and propagation scheduling.
Accepted RootSLAM pose-graph corrections update online navigation in the startup map gauge.
`rootslam/mapping/state` counts backend outcomes; `vio/navigation/revision` confirms publication.
Loop candidate counts alone do not prove an online navigation correction.

## Recorder architecture and operation

Mimir owns IPC subscriptions in one receive thread, copies each loan once into a bounded channel,
then releases the loan. A single writer groups scalar rows by entity into columnar Rerun chunks.
Count, byte, entity, publisher and voxel budgets are bounded in
[Options](../apps/mimir/src/config.rs); [mimir.toml](../configs/mimir.toml) contains overrides.
Scalar batches flush by row limit or periodic deadline. Repeated timestamps, order within an entity,
field order and values are retained. Clear flushes previous numeric rows before clearing entities.
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

Edit the layout in the Viewer and save its blueprint back to `configs/firefly.rbl`; rebuild mimir to
embed it. There is one layout artifact and no separate Python recorder. Before accepting a layout,
check every selected entity against the observation registry and inspect a real recording screenshot.
Old recordings without `mimir/status/*` need a derived display layer for the numeric cards; changing the
layout alone cannot create missing data. Preserve the original recording when producing such a layer.

Output paths must be new. After producers stop, SIGINT/SIGTERM drains diagnostic subscriptions until empty
(including disconnected publishers), drains the owned channel, flushes scalar batches,
writes the full RRD footer and syncs the file. Queue overflow, sequence gaps, declared producer loss,
invalid payloads, incomplete maps and writer/close failures return errors and preserve failure evidence
where disk access remains possible. Closing does not consume a queue slot.
RRD records lifecycle and high-water statistics in `logs/mimir`. Index memory grows with chunk count;
it is separate from the bounded pending-data budget.

Raw runtime evidence belongs in `logs/*.rrd`; JSON/HTML may hold configuration, fingerprints and summaries.
Do not create parallel text recordings. Stderr is a diagnostic sink, not archived evidence.
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
