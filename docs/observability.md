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
| `Firefly/Depth` | render/sensors → mapping/planner | No duplicate depth display |
| `Firefly/Imu` | sim/sensors → VIO, FC | Not subscribed by the ordinary recorder |
| `Firefly/Odometry` | VIO → control, planning, evaluation | Pose diagnostics; evaluation explicitly subscribes |
| `Firefly/Navigation` | VIO/localization → planner, FC | Business state, not a duplicate plotting topic |
| `Firefly/Reference` | trajectory output → FC, evaluation | Evaluation records target pose and velocity |
| `Firefly/Control` | FC → actuators/sim, evaluation | Motor evidence and FC summaries |
| `Firefly/PlantState`, `Firefly/Airframe` | sim/airframe → FC, system consumers | Independent evaluation evidence; never VIO input |
| `Firefly/Command` | ffctl/evaluation → FC | Results and rejection reasons in event logs |
| `Firefly/Goal` | ffctl/evaluation → planner | Planning goal, not a plotting topic |
| `Firefly/MandatoryStop` | external tools → planner | Stop command; outcome in planner state and logs |
| `Firefly/Trajectory`, `Firefly/PlannerHeartbeat`, `Firefly/TrajectoryControl`, `Firefly/PlannerStatus` | planner/FSM ↔ trajectory output/evaluation | Business lifecycle; visualization uses plan entities |
| `Firefly/OccupancyMap`, `Firefly/MapRegion` | map service ↔ consumers | Business map requests/responses; visualization uses voxel diagnostics |
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
| `vio/diagnostics/keypoints/{left,right}`, `okvis/debug/keypoints/{left,right}` | Camera overlays; one recorded point representation |
| `vio/diagnostics/keypoints/active_left` | Additional associated-landmark overlay, only `vio-diagnostics=debug` |
| `okvis/*`, `rootslam/*` | Tracking, loop commits and processing/publishing timing |
| `render/capture/*` | Capture-to-publish stage timing at the original capture timestamp |
| `corr/*`, `vision/debug/map_query_wall_s` | External localization gates, disagreement, correction and query timing |
| `plan/*` | Paths, poses, obstacle planes and maps for display |
| `acceptance/*` | Scoring evidence; display simplification must preserve it and all acceptance thresholds |
| `mimir/*`, `logs/*` | Configuration, queue high-water, lifecycle, failures and process events |

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

`okvis/frame_processing_ms` covers dequeue, extraction, matching, optimization, publication and
marginalization for successfully submitted frames; it excludes waiting before the processing thread.
`rootslam/frame_processing_ms` starts at stereo enqueue and ends at posterior consumption in the IMU
thread, so it includes queuing. These are different timing boundaries.
`RUST_LOG=info,basalt-profile=debug` records stage events in `logs/firefly-rootslam`.
Join by frame timestamp and process-monotonic boundaries; do not add nested durations or percentiles.
`render/capture/stages_s` records submit/readback/assembly/queue timing; `publish_s` records processing,
publication and total timing. Posterior age also includes visual processing and propagation scheduling.
`okvis/loop_commits` counts committed full-graph updates, not retrieval candidates.

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

```bash
cargo build --release -p mimir
./target/release/mimir --save logs/run.rrd
rerun logs/run.rrd
```

Output paths must be new. SIGINT/SIGTERM stops reception, drains the owned channel, flushes scalar batches,
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
