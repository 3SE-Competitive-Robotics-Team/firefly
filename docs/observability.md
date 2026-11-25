# Observability

Register each observation here with its producer, consumer, clock, and recording budget.
Source code and configuration define the defaults and limits. For system data flow, see
[architecture](architecture.md). For build, run, and test commands, see [development](development.md).

## Registration and change process

Every observation change must include these steps in the same review:

1. State the diagnostic question, consumer and retention requirement. Reuse existing observations.
   Remove temporary probes when their investigation finishes.
2. Use business topics for business data, `Firefly/VizScalars` for numeric diagnostics,
   `Firefly/Viz` for spatial diagnostics, `Firefly/VizVoxels` for maps, and `Firefly/Log` for human-readable events.
   Structured observations carry per-point data; TextLog carries bounded events. Business payloads describe their consumers' data needs.
3. Register numeric fields, order and units in [mimir/schema.rs](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/apps/firefly/mimir/src/schema.rs).
   Register spatial paths and allowed payload kinds there too. Unknown paths, kinds and widths fail recording.
   Dynamic paths use bounded syntax; frame and landmark identities belong in component fields.
4. Update the topic and observation-group catalog below with owner, consumer, clock, frame and enabling condition.
   Wire-layout changes require synchronized Rust and Python mirrors and actual IPC interoperability tests.
   Each message has one wire schema. Rebuild and restart all peers together when its layout changes.
5. Gate optional work at the producer, before expensive computation or payload construction.
   All logging dispatches must honor the same `RUST_LOG` filter so `log_enabled!` is a reliable gate.
   Enable specific targets instead of global debug for hover runs.
6. Measure peak messages and bytes, bursts, producer cost, receive continuity, write throughput and queue high-water.
   Queues absorb bounded bursts. Resolve sustained overload by correcting production and serialization costs
   while preserving registered observations and queue budgets.
7. Test names, layouts, units, disabled gates, exact timestamps and values, loss detection and shutdown.
   Write and read a real RRD. Report passed, skipped, and failed tests separately. Run the startup and flight acceptance checks separately.

## Data contract

| Data | RRD representation | Time and integrity |
|---|---|---|
| Numeric diagnostics, enums and bit masks | Scalars plus named SeriesLines | Original measurement `sim_time`; preserve every row, including duplicate timestamps |
| Poses, keypoints and maps | Transform3D, Points2D, VoxelGridMap | Explicit source coordinates; conversions belong to the display layer |
| Process events | `logs/<app>` TextLog | `sim_time` if available, otherwise `wall_time` |
| Camera images | `cameras/{left,right,down}/image` Image | Lossless grayscale at capture time; business camera subscriptions are latest-value |
| Keypoints | `cameras/{left,right}/points` Points2D and `track_id` | Original image timestamp, IDs and detected count retained; display coordinates add half a pixel |
| Configuration and calibration | `mimir/config`, static camera components | Only data constant for the recording is static |

Numeric state codes remain available for scoring; labels and enum meanings come from the producer definitions
in `firefly-flight`. Mimir records states in the numeric observation stream.
Ground truth, local odometry and map odometry retain their original frames. Offline comparisons state their frame alignment explicitly.

RootSLAM logs its accepted IMU count on shutdown. With `vio-receive=debug` it also reports the counter every 8192 samples.

The board USB watchdog logs a confirmed ADB rebind or a HIL timeout to the journal under
`firefly-hil-gadget`. Idle polls are silent. The supervisor archives these events with the
current boot's system stream under the existing segment and rotation limits. [Run archive](mimir.md)

## IPC topic catalog

Rust layouts and capacities live in [firefly-pubsub](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-pubsub/src/lib.rs).
Python wire mirrors live in [firefly-ipc/messages.py](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/packages/firefly-ipc/src/firefly_ipc/messages.py).
This catalog covers all production topic constants; test topics use isolated names.

| Topic | Producers → consumers; purpose | Recording policy |
|---|---|---|
| `Firefly/VizScalars` | FC, VIO, render, depth, localization, planner, traj and HIL → mimir | Ten publisher slots, including the simulated FC and pilot. Every registered numeric row; publisher sequence checked |
| `Firefly/Viz` | VIO, render, planner, localization → mimir | Registered spatial observations; sequence checked |
| `Firefly/VizVoxels` | planner → mimir | Complete ordered map updates; missing chunks fail |
| `Firefly/Log` | Rust applications, sim, flight scripts → mimir | Sequence and producer loss count checked |
| `Firefly/GroundTruth` | sim → render, mimir, tools | Pose and trajectory; independent scoring samples |
| `Firefly/CameraRig` | Render → Mimir on the host | One retained 288-byte snapshot per Render start. Left, right and down each carry resolution, pixel-center intrinsics and the body←RDF transform. Late subscribers receive the current rig. Offline Render uses `Firefly/Offline/CameraRig` alongside its offline images. Mimir stores one static baseline per camera per segment and rejects changes during a recording. |
| `Firefly/VioPrediction` | VIO → Mimir, directly or via HIL | With VIO diagnostics enabled, at most 50 samples per measurement second. Each 136-byte sample carries the measured pose and velocity, plus angular velocity and posterior age when attitude is available. One publisher, 16 receive slots, sequence checked. HIL v13 carries 129 payload bytes per sample. |
| `Firefly/CorrectedOdometry` | VIO/localization → mimir, tools | Map-frame pose and trajectory |
| `Firefly/VioAttitude` | VIO prediction → FC, mimir | Bias summary; control consumers remain independent |
| `Firefly/CameraLeft`, `Firefly/CameraRight` | render/camera → VIO, depth (left), mimir | Lossless grayscale samples when images are enabled |
| `Firefly/Activity` | Task → landing; HIL priority downlink | 32-byte owner/sequence inference lease, 5 Hz per owner, 8 receive slots and up to 8 owners/publishers; local arrival expiry ≤2s. LANDING requests inference; release/expiry suspends inference work. Phase requests are retained in `logs/flight`; actual resources in `landing/runtime`. Camera acquisition, publication and HIL image transport run continuously. |
| `Firefly/CameraDown` | render/camera → landing, mimir; OV2640 CIF 352×288, sensor id 3 | Latest-value 101400-byte grayscale input; lossless samples when images are enabled |
| `Firefly/LandingTarget` | landing → simulation pilot, daemon, mimir, HIL uplink; odom-frame body contact position | Latest-value 64-byte message; capture time, VIO session, team and validity; recorded as `landing/target` and `landing/quality` |
| `Firefly/CameraPair` | render → image consumers | Pair notification; image topics carry the recording |
| `Firefly/Depth` | YOLO depth → planner-owned EGO mapper | Latest 320×240 float32 axial metres in the left camera frame, with its capture timestamp. HIL computes on board. Offline dataset labels use a separate namespace. |
| `Firefly/Imu` | Matrix pilot or sim → RootSLAM, simulation controllers | Optional replay observations use the registered `vio/input/*` diagnostic gate |
| `Firefly/Odometry` | VIO → control, planning, flight scripts and Mimir | Mimir samples initialized local odometry at up to 10 Hz on the measurement clock for `vio/odom` and its display trail |
| `Firefly/Navigation` | VIO/localization → optional consumers, mimir | Atomic map←odom relation; mimir records changes only and uses bounded startup matching, with poses supplied by their existing observation streams |
| `Firefly/Reference` | traj → simulation pilot, flight scripts | Flight scripts observe reference goal identity |
| `Firefly/Sticks` | Stick source → Orange Pi pilot over local IPC, or simulation pilot → Mac fc via IPC/HIL | 16 bytes, queue depth 2, up to 50 Hz over USB. Four primary channels in µs. On hardware, the timestamp is in seconds on the Orange Pi monotonic clock. HIL v13 carries the same 16 bytes, little-endian. Commands expire after 100 ms. |
| `Firefly/Control` | Mac fc → simulated plant | Motor evidence and FC summaries |
| `Firefly/PlantState`, `Firefly/Airframe` | sim/airframe → FC, system consumers | Optional physical-state evidence; VIO uses camera and IMU observations |
| `Firefly/Command` | daemon, ffctl, HIL relay → simulation pilot | Three publisher slots. The simulation pilot consumes the latest request, the HIL bridge buffers up to eight. Results and rejection reasons in event logs. |
| `Firefly/FcStatus` | Simulation pilot → daemon, USB HIL and Mimir; `mode` is the native state code, `can_arm` is the FC's latest pre-arm decision (0/1). | V2 IPC type, 24 bytes at up to 10 Hz, one publisher and 16 slots. Mimir records mode and verifies sequence continuity; the existing `fc/arming/failed` observation retains the reason. HIL carries 18 payload bytes. Update all peers together. |
| `Firefly/Goal` | daemon, ffctl, HIL relay → planner | Three publisher slots. The planner consumes the latest request, the HIL bridge buffers up to four. 80-byte waypoint in local odom coordinates. STOP (0) supports optional terminal yaw in radians (CLI degrees). PASS (1) carries the next xyz position. One sample per task command. |
| `Firefly/MandatoryStop` | external tools → planner | Stop command; outcome in planner state and logs |
| `Firefly/Trajectory`, `Firefly/PlannerHeartbeat`, `Firefly/TrajectoryControl`, `Firefly/PlannerStatus` | planner → traj (trajectory, heartbeat, control), flight scripts (status) | Atomic xyz/yaw polynomials with shared piece times and a stationary endpoint; heartbeat loss finishes the committed stop; 32-byte status identifies the command and reports scheduled completion plus latched measured passage |
| `Firefly/PoseObservation` | external pose source → localization | Business localization input; localization emits registered result diagnostics |

Latest-value business channels retain the newest available sample. Complete replay uses the dedicated
registered diagnostic observations with coverage checks. Start mimir first and stop it last.
`RUST_LOG=info,planner-motion=debug` adds one bounded proposal-summary log per motion coordination
attempt: initial state, heading decision, first sampled rejection (window, occupied or V/A/J), and
the end of the first chord whose clearance neighborhood lacks recent free evidence or whose centerline is blocked. The diagnostic includes the fixed occupancy, online evidence and collision state at that endpoint.
Evidence age uses the odometry measurement time and the planner's one-second freshness limit. EGO's optimistic obstacle policy permits unknown
space. The summary uses `logs/planner` and its existing log budgets; masks and individual sample checks remain local. Leave it disabled for ordinary operation; incomplete log recording fails validation.
Sequence checks cover the subscription interval. Earlier data and the tail of a crashed producer require
independent input coverage evidence.
The traj application publishes its existing startup and odometry lifecycle events as `logs/traj`.
It uses the existing Log queue, severity filters and 512-byte message limit; sampling and IPC rates
remain 100 Hz. Mimir retains every enabled event through shutdown and validates source continuity.
Rust log queue/send loss is counted in `LogMessage.dropped_records`; the pump reports loss independently
of subsequent business events. Known loss makes the recording fail.

## Observation groups

| Group | Owner, purpose and enabling condition |
|---|---|
| `fc/debug/*`, `fc/imu/*`, `fc/arming/*`, `fc/failsafe/*` | FC position, attitude, control, freshness and safety summaries; producer-defined cadence |
| `fc/standby_eskf/*` | Standby health, bias and variance in its documented local gravity frame |
| `fc/loop/*` | Mac FC per-tick timing (tick, stick age, IMU arrival age, wall period), angular rate and allocation; only `fc::diagnostics=debug` |
| `vio/posterior/*` | Each VIO posterior pose, world velocity and IMU biases |
| `vio/prediction/*` | One VioPrediction sample supplies the pose, velocity and optional attitude diagnostics at its original measurement time |
| `vio/diagnostics/*` | Per-frame identity counts, stereo ratio, pixel residuals and optimization deltas |
| `vio/diagnostics/keypoints/{left,right}` | Camera overlays; one recorded point representation |
| `vio/diagnostics/keypoints/active_left` | Additional associated-landmark overlay, only `vio-diagnostics=debug` |
| `rootslam/*` | Stereo enqueue-to-posterior and IMU publishing timing |
| `vio/odom`, `vio/traj`, `vio/map/{pose,traj}`, `vio/gyro_bias` | Local and map poses at measurement time, derived trajectories and IMU bias summary; original frames are retained |
| `fc/debug/precision` | FC at its existing diagnostic cadence: 0 ordinary mode, 1 Align, 2 Descend, 3 Final, 4 Reacquire; one scalar; always recorded |
| `landing/target`, `landing/quality` | Native landing observations, at most 10 Hz by default, four scalars per row; always recorded. Position xyz metres + valid; session/team/reprojection pixels/confidence. Session identity remains exact in the native u64 message; scalar session is display-only. |
| `logs/landing` with `RUST_LOG=landing::gates=debug` | Optional per-inference corner/plane/position gate counts or missing capture-time odometry; at most configured 30 Hz, one bounded TextLog per inference, off by default |
| `landing/runtime` | Always-on lifecycle evidence: active (0/1), cumulative inferences, inference requested (0/1), NPU (0/1); at most 1 Hz plus transitions, four scalars per row. Dormant releases the model/session, down subscription and odometry worker; inference count stays constant. Timestamp follows the latest VIO attitude, including during cruise. |
| `landing/vision` | Optional `landing.toml diagnostics=true`: detections/valid/reprojection pixels/processing milliseconds, four scalars per inference, at most the configured 30 Hz; disabled by default |
| `render/capture/*` | Capture-to-publish stage timing at the original capture timestamp |
| `hil/link`, `hil/board` | Host USB bridge: per-wall-second traffic counts and board freshness, stamped with the latest simulated IMU time; enabled during HIL, two rows per second |
| `hil/camera/{left,right,down}` | Board camera bridge: capture timestamp, pixel checksum and dimensions for transport verification, one row per admitted image at its original capture time |
| `fc/debug/vertical_ned` | FC vertical velocity target/measurement, acceleration target and thrust target in NED, at the existing FC diagnostic cadence; retained for HIL and simulation control diagnosis |
| `corr/*` | External localization gates, disagreement and correction |
| `plan/*` | Global guides, accepted executable local paths including their stopping tail, poses, obstacle planes and maps for display |
| `depth/inference` | At most 1 Hz while publishing depth, four scalars on the image clock: cumulative published frames, preprocessing + inference + restore + IPC milliseconds for that frame, center pixel metres, A733 NPU active (0/1). No depth image telemetry. |
| `plan/collision` | Planner-owned depth map after obstacle inflation, in odom coordinates at the planner simulation timestamp; bounded voxel snapshots/deltas for checking collision geometry, using the existing map display cadence and budgets |
| `map/frame_ms` | Per accepted depth frame, including empty frames. Wall milliseconds from sampled-image enqueue to fusion start, then from fusion start through local snapshot handoff. |
| `map/update_ms` | One row per worker update after the first accepted measurement. Integration, obstacle maintenance, snapshot construction and shared-slot handoff (including release of a superseded snapshot), in wall milliseconds. Zero marks a skipped operation. Maintenance alone uses the latest accepted measurement timestamp. |
| `map/fusion_ms` | Per accepted frame, wall milliseconds in depth projection, window update, ray traversal plus probability/evidence updates, and inflation. Window time includes clearing exiting slabs. |
| `map/work` | Per accepted frame, number of valid projected endpoints, distinct voxels receiving probability votes, and exiting window voxels cleared. Empty frames can move the window. |
| `planner/loop_hz`, `planner/fsm_hz` | Completed execution callbacks and actual FSM executions per wall second; FSM executions require fresh odometry and a ready map |
| `planner/replan_submitted_hz`, `planner/replan_hz` | Worker jobs submitted and candidates actually adopted per wall second |
| `planner/replan_ms` | Per completed job: `latency_ms` from submission/snapshot through main-thread adoption/discard, solver-thread `solve_ms`, and `adopted` (0/1) |
| `planner/fsm_timing_ms` | Per execution callback: `fsm_ms` for the FSM call, `step_ms` for input/map/FSM/worker handoff/status work, and start-to-start `period_ms` |
| `traj/sample_hz`, `traj/reference_hz` | Actual trajectory sample calls with valid odometry and successful Reference publications per wall second; zero while the corresponding work is idle |
| `traj/timing_ms` | Per timer callback: `tick_ms` for inputs/decoding/sampling/publication, sampler `sample_ms`, local Reference IPC `publish_ms`, and start-to-start `period_ms` |
| `logs/flight` | Mission daemon → Mimir and offline scoring. Each accepted plan emits settings, its ground anchor and up to 16 ordered waypoint declarations. Ground and complete events use the FC status measurement timestamp. Other phase, airborne and lease events use the odometry measurement timestamp on the same timeline. Each TextLog uses the existing 512-byte limit and Log queue. |
| `logs/render` | Render publishes one capture declaration per startup at simulation epoch zero, carrying the selected scene and configured downward capture rate for image scoring. |
| `mimir/*`, `logs/*` | Configuration, queue high-water, lifecycle, failures and process events |
| `mimir/status/{fc/tick_rate,vio/processing,vio/posterior_age,vio/landmarks,camera/publish_rate,camera/publish_latency}` | Six independent recorder-derived numeric documents from existing scalar observations; at most 2 Hz per entity in measurement time, six cached values and one constant-space publication counter on existing observations |
| `frames/{map,odom,world}` | Recorder-owned display transforms: root←map identity, temporal map←odom from Navigation, and optional constant map←world startup alignment; display-layer data |

Every native mission event contains the OS boot ID (`run`), caller's `mission_id` and nullable `flight_id`. Before measured liftoff, the flight ID is null. An `airborne` event binds the mission to a positive sortie number for that run. A cancelled ground attempt retains a null flight ID. Settings declare only the accepted plan. The home anchor records the VIO session. Acceptance, airborne assignment and phase changes are recorded after the corresponding receipt commits, using the timestamp captured at the transition. Phase names use the same serialized Rust enum as the durable receipt and CLI. The verifier checks the full phase sequence against the recorded plan before computing physical metrics. It groups events by run and mission, checks airborne assignments, disarmed intervals and shared odometry. Legacy Python recordings use their original planned-flight header. Native runs accept missions on demand. Expected mission IDs and SIL/HIL backend are checked independently by the test client.

The daemon announces the boot ID on the first measured odometry after startup. Announcements from restarts of the same daemon carry the same boot ID. A recording can start after the announcement because each mission event carries the identity. An accepted 16-point plan emits 19 records at submission (settings, waypoints, home, arm). The existing Log buffer holds 256 records. USB HIL carries correlated JSON mission requests and replies, with one active host request and an 8192-byte payload limit. The board daemon executes the request through the same local socket as ffctl.

The mission verifier derives takeoff displacement, actual yaw change and heading error from the existing GT poses and FC quaternion observations using a fixed initial frame alignment. The first flight defines the odometry-to-truth transform for every flight in a recording. FC attitude-source and health rows distinguish VIO participation from estimation drift; these checks add no producers or IPC traffic.

The planner/traj performance consumers are the default Viewer Planner and Traj tabs and offline RRD checks.
These bounded observations are always produced and are independent of log severity filters.
Six frequency rows use actual monotonic elapsed time, including callback stalls, and emit at most once
per wall second. Planner adoption excludes discarded/failed jobs and synchronous stationary or emergency
trajectories. Zero adoption while waiting for a goal is normal. All nine entities are temporal Scalars
on the latest odometry `sim_time`; their values measure process wall time. `replan_thresh` is elapsed
trajectory measurement time, so a one-second threshold is not a fixed one-Hz wall-clock schedule in
accelerated or slowed simulation. Compare accepted-result cadence as well as individual result latency.

`replan_ms.latency_ms` includes snapshot, queue, solve and final main-thread decision.
`solve_ms` includes only work in the solver thread; both include OS scheduling and are not CPU-time counters.
`adopted` distinguishes a fast rejected result from an executable accepted result.
FSM core timing excludes worker execution, diagnostic production and voxel display; `step_ms` includes
sensor polling, map synchronization, worker result adoption and normal trajectory visualization, but
excludes performance publication and voxel-display handoff. `period_ms` includes intervening
safety work, display handoff, diagnostic publication, scheduler waits and preemption. The first period is zero.
Voxel scanning, coarsening, differencing and publication run on an independent display worker.
At most 10 map views per wall second are offered through one pending, unrendered view slot;
a newer view replaces pending rendering work and preserves any full-resync request.
Every produced full/delta update and chunk is retained under the existing voxel budgets.
Traj `tick_ms` excludes performance publication and log pumping. A zero FSM/sample/publish duration
means that operation was not executed in the callback; reports of computation latency use executed operations.
Reference publication timing ends when the local IPC send returns; it excludes FC consumption and USB transport.

Each callback timing row is retained (nominally 100 Hz per app), plus one row per completed worker job
(at most the existing 100-Hz execution callback cadence). Existing ScalarMessage (128 bytes),
publisher/queue limits and complete-retention checks apply. Normal one-Hz replanning adds about 26.5 kB/s
before HIL framing; the nominal 100-Hz completion bound gives 39.2 kB/s, with a maximum nine-row burst.
The existing HIL scalar path forwards these observations without additional topics or queue budgets.

The **Map** tab in the lower panel reads these four numeric entities on the depth measurement
`sim_time`. The mapping worker publishes them directly on `Firefly/VizScalars` with standard logging
settings. The voxel display worker opens only the voxel port. Each 32 ms occupancy period admits
one paired image on the first worker poll that finds it ready. Unused capacity expires at the next period. At this rate and a 500 ms
maintenance timer the nominal upper bound is 127 scalar rows/s (16,256 payload bytes/s), with at
most four rows per worker update. Counts use existing buffers and disjoint slab dimensions.
Frame update time includes maintenance performed during the same worker iteration and ends before diagnostic IPC.
The queue wait begins after depth resampling. Snapshot handoff ends on the map worker and records
neither planner consumption nor Mimir serialization. To verify rows and timestamps from the actual
worker in an indexed RRD, build release `planner` and `mimir`, then run
`uv run pytest tests/system/test_mimir.py::test_map_worker_to_recorded_map_panel -s`.
Validation checks production with log output disabled, field order/units, clock separation, result/sample/send
counts, observed queue high-water, source continuity and RRD shutdown integrity. Performance run environment
and resource snapshots use bounded `logs/performance` events on the existing Log channel (512-byte limit).

### Spatial frames

HIL forwards Navigation, spatial/voxel diagnostics, numeric diagnostics and process logs to the
host recorder. Navigation retains its atomic odometry, alignment, revision and external-map fields,
so the recorder applies the same frame construction as local simulation. Spatial and voxel payloads
retain their source entity, measurement timestamp, active array values and voxel chunk identity.
Source production gates and cadences remain unchanged; wire encoding carries active elements only.
The board uses the existing 256-message spatial and 32-message voxel IPC budgets, with at most
10 spatial publisher slots and one voxel publisher. Stick commands are drained before each diagnostic message;
a maximum-size voxel message occupies up to 196,732 payload bytes (13 USB fragments). The existing
16 KiB fragment, 512 KiB message and bounded USB queue limits apply. Sequence validation covers
each source publisher; the host assigns one continuous sequence per forwarded topic.
Log trace IDs and producer loss counts survive transport. A source gap or known producer loss
fails the run. HIL traffic summaries
use simulation timestamps; heartbeat deadlines use local monotonic wall time. Retain the complete
RRD through shutdown and verify transport, posterior coverage and hover limits independently.
For roundtrip acceptance, the board forwards `Firefly/Reference` and `Firefly/PlannerStatus` on
the existing bounded USB uplink. The board HIL motor process owns their latest-value subscriptions and checks for new stick commands
before every telemetry write. Their producer rates remain the planner/traj applications' existing
cadence (at most 100 Hz each); the two payloads add at most 29.6 kB/s including frame headers.
Flight scripts observe original measurement and goal timestamps, alignment fields and completion
values. Native completion and measured arrival are distinct facts. Command events use logs/flight,
with at most one 512-byte message per command or transition on the existing Log queue budget.

All registered 3D payloads declare their source frame in `mimir/schema.rs`; adding a spatial entity
requires registering both its payload kind and its frame. `Transform3D.parent_frame` labels raw poses;
`CoordinateFrame` labels lines, arrows and voxels in the same row, preserving their association across recursive clears. Raw positions, orientations, timestamps remain unchanged.

| Entities | Frame |
|---|---|
| `gt/{pose,traj}` | Simulator `world` |
| `vio/{odom,traj}`, `vio/posterior/pose_imu_world`, `vio/prediction/pose_body_odom` | Local estimator `odom`; posterior “world” means the estimator's local world |
| `vio/map/*`, `corr/*` | Navigation `map` |
| Spatial `plan/*` | Local planner `odom`; independent of optional Navigation corrections |
| `cameras/{left,right}` | Extrinsics relative to the moving `tf#/vio/prediction/pose_body_odom` body frame; intrinsics and pixel overlays retain camera coordinates |

The Viewer root connects to `map`. `map←odom` is recorded at the Navigation measurement time on
alignment/revision changes. A backward timestamp/revision, an alignment change at the same revision, or
changed external-map mode ends the recording as incomplete; start a new recording for a new session.

For autonomous simulation, mimir can connect `world` to `map` using one pair of truth and Navigation
poses at the same measurement nanosecond during stationary initialization near the odom origin.
Both source timestamps are converted to integer nanoseconds, matching VIO and the RRD timeline;
floating-point seconds may differ after VIO's integer-clock round-trip. Pairing uses exact integer nanoseconds.
Only yaw and translation are aligned, once; roll/pitch disagreement, scale error and subsequent drift
remain visible. This display convention aligns the arbitrary initial origin/heading; absolute localization
uses independent evaluation. Matching uses bounded storage and exact timestamps, preserves every diagnostic
sample, and leaves the existing IPC and writer queue budgets unchanged.
Observed motion or a pose away from the local origin closes the startup window. Missing matches and
external-map sessions leave `world` disconnected, with a warning in `logs/mimir`; inspect it with a
separate world-frame 3D view. This startup heuristic identifies a session from pose and motion:
starting the recorder after a return to the estimated origin can still qualify for alignment.
Start recording before producers. Score absolute drift from the original trajectories and evaluation alignment.

Hover evidence includes truth, posterior, prediction, targets/errors, velocity, bias, posterior age,
attitude source, safety state, per-frame residuals and optimization corrections.
The fourth `vio/diagnostics/tracks` field counts associated landmark IDs. Effective constraint counts need
separate whitening and robust-weight semantics. Comparisons state each estimator's identity and initialization
rules. Per-point depth/weight additions use the registration process and bounded structured payloads.

Residuals are unweighted pixel residuals of valid projections: count, norm RMS and mean x/y residual.
A zero count represents an unavailable residual statistic. Position/velocity deltas are local
optimization after-minus-before; rotation is the corresponding rotation vector. They exclude frontend
initialization and loop synchronization. RootSLAM also reports graph and keyframe summaries.

`RUST_LOG=info,vio-gauge=debug` enables `vio/diagnostics/yaw_updates_rad` in RootSLAM.
It distinguishes a shared window rotation from a relative attitude correction: world-Z components
of `Log(R_after * R_before.inverse())` for the latest and oldest retained poses, followed by the
window mean and population standard deviation. All four values use rotation-vector components.
The consumer is RRD odometry diagnosis, with the image measurement `sim_time` and odom axes.
The producer gate precedes the bounded pose snapshot (at most the configured VIO window);
one four-scalar row is emitted per processed frame, bounded by the camera rate, through the existing topic
and queue budget. Production is off by default; retain every enabled row through shutdown.

`RUST_LOG=info,vio-solver=debug` enables RootSLAM's `vio/diagnostics/costs_before`,
`costs_after`, and `solver_termination` for RRD convergence diagnosis. Costs cover the complete
active optimization problem before and after optimization: weighted robust vision, inertial,
bias random walk, and marginal prior. The prior omits its constant term and may be negative;
compare before/after within one frame using that frame's prior.
Termination contains attempted steps, rejected steps, reason (1: cost decrease tolerance,
2: step tolerance, 3: iteration budget, 4: damping limit), and last attempted step infinity norm.
Emission begins with the first optimized frame. The producer gate precedes the two extra objective
evaluations; each optimized image emits three four-scalar rows at its `sim_time`, bounded by the
camera rate, through the existing topic and queue budget. Production is off by default; retain every
enabled row through shutdown. Costs are dimensionless and the step mixes the solver's state units.

`RUST_LOG=info,vio-input=debug` records every IMU measurement admitted by the VIO runtime as
`vio/input/gyro_rad_s` and `vio/input/accel_m_s2`, in IMU axes at the original measurement
`sim_time`. Together with the complete timestamped left/right keypoint IDs and positions, these
are raw inputs for backend replay; predictor and corrected measurements retain their separate entities.
The runtime owns the producer and gates publication before constructing either row. Production
is off by default. The simulated 1 kHz IMU adds two three-scalar rows per sample (2,000 rows/s);
the existing subscriber, queue and retention policies apply. A diagnostic run must verify matching
gyro/accel timestamps, the expected IMU sampling intervals, coverage of every processed image,
publisher continuity, and recording completion before treating its inputs as replayable.

Offline backend comparisons retain derived states in a separate RRD at the source image
`sim_time`: `{rust,upstream}/pose` is the IMU pose in odom, `p` and `q` preserve its
float64 position and XYZW quaternion, and `v`, `bg`, `ba` are world velocity, IMU gyroscope
bias and IMU accelerometer bias respectively. `rust/active_ids` retains current-frame IDs
associated with landmarks, with the same semantics as the online active-keypoint diagnostic. There is one row
per processed image, produced by the offline comparison. Record both implementations' configuration adapters;
reconstructed viewer keypoints and the initial pose retain their float32 precision. Report that precision
when comparing with the original online run.
The unified dataset entry publishes through the RootSLAM log IPC and diagnostics registry.
Flight odometry belongs to the live input path. Dataset `sim_time` is the supplied measurement
time in seconds. Dataset camera times are converted
to the IMU clock with the calibration offset before estimation; exported trajectories
retain these integer nanoseconds (TUM serializes seconds). Raw IMU and GT retain
their supplied timestamps; record the GT alignment assumptions separately.
Benchmark evidence includes the clock offset, source/calibration fingerprints,
configuration, requested/produced frame counts, process exit and RRD completion.
Native trajectory files are algorithm outputs; runtime diagnostics still belong in RRD.
GT-based offline ablations use separate `backend_matched_raw.rrd` and
`backend_matched_ideal.rrd` recordings with the same state entities. Both retain exactly the same
track IDs, camera memberships and timestamps; only the ideal arm replaces pixels with projections
of fitted static points through GT camera poses. These are diagnostic counterfactual inputs evaluated in the offline comparison. Report selection counts and
keep the unmodified recording for comparison.
Offline temporal-flow reference comparisons use `flow/counts` (source points, common accepted
IDs, online-only IDs, reference-only IDs) and `flow/error_px` (median, p90, maximum location
difference on common IDs), one row per compared image pair at the target image `sim_time`.
They compare temporal tracking from identical recorded images and source points; corner detection has its own validation.
Offline render counterfactuals reuse `cameras/{left,right}/image` in separate RRDs, indexed by
the original pose timestamp; transport request timestamps are only used to pair replies.
Record the renderer and scene hashes, requested/received frame counts and process exit codes.
The bounded offline backend probe may include `rust/landmarks` as a JSON TextDocument per
selected frame: landmark ID, host timestamp, inverse distance, and each retained observation's
timestamp, camera, residual and Huber weight. The offline probe records explicitly selected time windows;
production uses the bounded summary observations.

`rootslam/frame_processing_ms` starts at stereo enqueue and ends at posterior consumption in the IMU
thread, including queue time.
`RUST_LOG=info,basalt-profile=debug` records stage events in `logs/firefly-rootslam`.
Join by frame timestamp and process-monotonic boundaries; aggregate disjoint durations and compute percentiles from samples.
`rootslam-global-profile=debug` records offline global BA stage times in milliseconds:
linearization, damping, preconditioner, linear solve including preconditioner,
back substitution and trial cost. The solver owns these events; benchmark analysis consumes
them from `logs/firefly-rootslam` on the process wall clock. There are at most six events per
LM attempt plus the initial linearization, each below 512 bytes. Timing is disabled at the
producer by default. The solve time includes the preconditioner, so their durations are reported as
nested stages. The events contain timing metadata. Retain the recording with the frozen problem,
configuration and result; verify complete stage events and clean RRD shutdown after replay.
Global BA reports optimized and deferred landmark counts once at INFO level (below
512 bytes). Deferred landmarks lack two distinct camera views; they remain in the map.
Use these counts to distinguish the map size from the actual optimization problem.
`RUST_LOG=info,planner-profile=debug` records planner callbacks taking at least 10 ms in
`logs/planner`, including sensor polling, map synchronization, execution, safety, display handoff
(`display-submit`) and the independent display worker (`display`).
Each event contains the stage and elapsed wall time; aggregation accounts for nested stages once.
Production is off by default, bounded by the existing 100/20/10 Hz callback rates and 512-byte log limit.
Depth-input expiry/termination logs one warning when planner enters the latched emergency state,
including odometry measurement time, in the existing `logs/planner` stream (512-byte limit).
`rootslam-loop=debug` adds one retrieval summary per keyframe and one rejection/acceptance event
per tested loop candidate to `logs/firefly-rootslam`. Fields are frame IDs, descriptor/match/inlier
counts and rejection stage in a bounded summary. At most `1 + max_candidates` events
per detection job, bounded by the configured keyframe/image rate and the existing 512-byte log limit.
`render/capture/stages_s` records submit/readback/assembly/queue timing; `publish_s` records processing,
publication and total timing. Posterior age also includes visual processing and propagation scheduling.
Default RootSLAM runs as VIO. Planner maintains occupancy and trajectories from raw body odometry
and depth. Global map optimization is evaluated through the dataset observations.

## Matrix flight diagnostics

During a real Matrix USB session, pilot publishes twelve scalar groups per flight snapshot
on `Firefly/VizScalars`. Mimir records them on the IMU measurement timeline (`sim_time`).
Before the first IMU anchor, pilot uses USB arrival time. Start the onboard recorder
before the flight with `systemctl start firefly-mimir.service`.

| Entity under `matrix/` | Recorded fields |
|---|---|
| `link` | Slot since handshake and raw 32-bit FC capture time in µs. A skipped slot identifies missing coverage. |
| `attitude` | Betaflight roll, pitch and yaw in degrees. |
| `control/state`, `control/sticks`, `control/motors` | Armed, RX signal, companion applied, mode mask. Effective primary channels in µs and four MSP motor commands. |
| `faults/state`, `faults/last_disarm`, `faults/last_failsafe` | Arming-disable mask and current failsafe phase. Latest disarm reason and failsafe entry phase, each paired with its latched FC time in µs. |
| `power/state`, `power/meter`, `power/extrema` | Betaflight battery state, cell count, voltage (V), current (A), charge (mAh), minimum sampled ADC voltage and peak sampled ADC current since the previous transmitted snapshot. |
| `temperature` | Cached MCU die temperature (°C), refreshed by Betaflight at 1 Hz. |

The firmware sends one 68-byte snapshot per 200 ms slot (340 B/s nominal).
It drains incoming commands first, reserves eight IMU frames of USB TX space and
skips congested slots. ADC extrema survive these skipped slots. The USB thread in
pilot offers whole snapshots to a two-slot queue with `try_send`. A separate thread
publishes at most 60 scalar messages/s (7,680 B/s of IPC payload) at nominal rate.
Sensor availability bits become NaN for unavailable measurements. Flight controller
event times and capture times use the same wrapping µs counter. The slot resets
on a new handshake. Mimir preserves the groups and gaps in each recorded run.

## Recorder architecture and operation

Mimir prints the absolute run directory on stdout and writes numbered files under `rrd/`. The [Mimir recording contract](mimir.md) defines the layout, storage limits and retention rules for all callers.

Mimir owns IPC subscriptions in one receive thread, copies active fields into owned events for the bounded writer channel,
then releases each loan. Geometry events carry only the selected shape and voxel events carry the
indices declared by the chunk. Queue accounting includes each event and its owned buffers. A single writer groups scalar and typed observation rows by entity into
columnar Rerun chunks. Typed rows use Rerun's `PendingRow::many_into_chunks` to group images, poses,
trajectories and metric cards.
Count, byte, entity, publisher and voxel budgets are bounded in
[Options](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/apps/firefly/mimir/src/config.rs); [mimir.toml](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/mimir.toml) contains overrides.
All batches flush at the row limit or the configured deadline (500 ms in the installed configuration). Typed rows also share the `batch_bytes` memory
budget; reaching it flushes pending rows synchronously. A single oversized row is written directly
after flushing and releases its temporary storage immediately. Static metadata is written directly.
Repeated timestamps, order within an entity, field order and values are retained. Clear flushes
previous rows before clearing entities. Batching preserves every source observation.
Synchronous RRD encoding writes directly to disk.

Voxel chunks must arrive in order and form a complete update before a map snapshot is written.
Deltas require a baseline and matching geometry. Planner display coarsening remains separate from its
planning grid. Incomplete, reordered or over-budget updates fail recording.

Images use native lossless grayscale Image components.
Keypoints retain their capture timestamps and track IDs; display pixels and principal points add 0.5
because the algorithms use integer pixel centers and Rerun uses image boundaries.
Viewer history selection pairs point sets and images by measurement timestamp.
The recording contains stereo images and registered spatial observations; inspect the output file
with the standard Rerun viewer.

### Default viewer layout

Open a run with `cargo xtask view "$run"`. Mimir embeds the [shared blueprint](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/firefly.rbl) in each new segment. If Rerun opens a cached layout, select **Reset to default blueprint** or load the RBL explicitly.

The map shows ground truth in blue, local odometry in yellow, map-frame navigation in purple, the guide path in green and the local plan in coral. The Down and Stereo views show images at capture time. Landing, Planner and Traj plots use the same measurement timeline. Controller tracking error compares the target to the estimated position. Ground-truth drift requires frame alignment.

Numeric cards show the latest value at the cursor. Missing, nonfinite or more-than-two-second-old measurements display `—`. Camera FPS uses strictly increasing `render/capture/publish_s` timestamps over at least one second. The counter ignores repeated or out-of-order timestamps and restarts after a two-second gap. Publication latency comes from `publish_s.total_s` and the landmark count from `vio/diagnostics/tracks.landmark_ids`. The cards add at most twelve document rows per measurement second after initialization.

To change the shared layout, edit the [blueprint generator](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/scripts/build_blueprint.py), run `uv run python scripts/build_blueprint.py` and rebuild Mimir. Check the selected entities against the registry and inspect a real recording. For older recordings without `mimir/status/*`, the viewer can use a derived display layer while preserving the source.

### Recorder shutdown

Output paths must be new. After producers stop, SIGINT/SIGTERM drains diagnostic subscriptions until empty
(including disconnected publishers), drains the owned channel, flushes all batches,
writes the full RRD footer and syncs the file. Queue overflow, sequence gaps, declared producer loss,
invalid payloads, incomplete maps and writer/close failures return errors and preserve failure evidence
where disk access remains possible. Closing uses an independent control path.
RRD records lifecycle and high-water statistics in `logs/mimir`. Index memory grows with chunk count;
it is separate from the bounded pending-data budget.

For startup failures, inspect process output in `text/` and service or kernel events in `system/`. The run manifest lists gaps and evicted ranges for each source.

## Validation

```bash
cargo test -p mimir -p firefly-observability -p firefly-pubsub --all-targets
cargo test -p firefly-pubsub --test python_interop -- --ignored
uv run pytest packages/firefly-workspace/tests tests/test_observability_contract.py -q
uv run pytest tests/system/test_mimir.py -q -s
```

The IPC load test requires release mimir and an idle workspace. It checks exact values and timestamps
after RRD readback and verifies the footer and final completion event. Repeated throughput runs use
fixed inputs and budgets; missing data fails. Recording integrity, flight safety and hover accuracy each
have their own checks. Full flight tests use the thresholds defined by their sources.
