"""Grade native mission states, continuous images and physical flight evidence."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field
from operator import itemgetter
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .config import CUBOID_ROOF
from .recording import Evidence, read_recording


@dataclass
class Report:
    failures: list[str] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)

    def check(self, condition, message):
        if not condition:
            self.failures.append(message)

    def result(self):
        return {"passed": not self.failures, "failures": self.failures, "metrics": self.metrics}


@dataclass(frozen=True)
class PhaseEvent:
    timestamp: float
    name: str
    goal: tuple[float, float, float] | None
    index: int | None

    @classmethod
    def from_row(cls, row):
        stamp, payload = row
        goal = payload.get("goal")
        return cls(
            stamp, payload["name"], tuple(goal) if goal is not None else None, payload.get("index")
        )


@dataclass(frozen=True)
class WaypointLeg:
    index: int
    travel: PhaseEvent
    hover: PhaseEvent | None
    following: PhaseEvent


@dataclass(frozen=True)
class Timeline:
    settings: dict
    home: dict
    start: float
    phases: dict[str, PhaseEvent]
    legs: tuple[WaypointLeg, ...]

    @property
    def end(self):
        return self.phases["complete"].timestamp


def read_timeline(data, report):
    check = report.check
    settings_rows = [row for _, row in data.events if row["kind"] == "settings"]
    check(data.recording_complete, "Mimir did not confirm complete recording integrity")
    homes = [(stamp, row) for stamp, row in data.events if row["kind"] == "home"]
    check(
        len(settings_rows) == 1 and len(homes) == 1,
        "one settings event and one takeoff-frame anchor required",
    )
    check(not any(row["kind"] == "failure" for _, row in data.events), "mission reported a failure")
    if not settings_rows or not homes:
        return None
    settings = settings_rows[0]
    check(settings.get("mission") in ("hover", "roundtrip"), "unknown mission kind")
    if report.failures:
        return None
    roundtrip = settings["mission"] == "roundtrip"
    count = settings.get("waypoint_count")
    check(
        type(count) is int and (1 <= count <= 16 if roundtrip else count == 0),
        "invalid waypoint count",
    )
    if report.failures:
        return None
    planned = [(t, row) for t, row in data.events if row["kind"] == "waypoint"]
    arrivals = [row.get("arrival", "stop") for _, row in planned]
    check(all(kind in ("stop", "pass") for kind in arrivals), "invalid waypoint arrival mode")
    expected = ["takeoff"]
    if roundtrip:
        expected.extend(phase for kind in arrivals for phase in
                        (("waypoint", "waypoint_hover") if kind == "stop" else ("waypoint",)))
        expected.append("return")
    if settings["precision_land"]:
        expected.append("acquire")
    expected.extend(("land", "complete"))
    phases = [(stamp, row) for stamp, row in data.events if row["kind"] == "phase"]
    check(
        [row["name"] for _, row in phases] == expected,
        "mission phase sequence is incomplete or out of order",
    )
    if len(phases) != len(expected) or [row["name"] for _, row in phases] != expected:
        return None
    phase = {row["name"]: (stamp, row) for stamp, row in phases}
    start, home = homes[0]
    legs = []
    if roundtrip:
        check(
            [row.get("index") for _, row in planned] == list(range(count)),
            "ordered waypoint declarations are missing or duplicated",
        )
        if len(planned) != count:
            return None
        cursor = 1
        for index, arrival in enumerate(arrivals):
            travel = phases[cursor]
            cursor += 1
            hover = None
            if arrival == "stop":
                hover = phases[cursor]
                cursor += 1
            following = phases[cursor]
            legs.append(WaypointLeg(index, PhaseEvent.from_row(travel),
                                    PhaseEvent.from_row(hover) if hover else None,
                                    PhaseEvent.from_row(following)))
            offset = planned[index][1].get("offset", [])
            if len(offset) == 2:  # Planar declarations in older recordings inherit takeoff height.
                offset = [*offset, settings["altitude"]]
            ceiling = CUBOID_ROOF if settings.get("shape") == "cuboid" else settings["altitude"]
            check(
                len(offset) == 3 and np.isfinite(offset).all() and 0.5 <= offset[2] <= ceiling,
                "invalid declared waypoint offset",
            )
            if len(offset) != 3:
                return None
            goal = np.array(home["position"]) + offset
            check(
                planned[index][0] <= start
                and travel[1].get("index") == index
                and np.array_equal(travel[1].get("goal"), goal)
                and (hover is None or (hover[1].get("index") == index
                                        and np.array_equal(hover[1].get("goal"), goal))),
                f"waypoint {index}: phase identity/goal differs from the declared route",
            )
        return_goal = np.array(home["position"]) + [
            *settings["return_offset"],
            settings["altitude"],
        ]
        check(
            np.array_equal(phase["return"][1].get("goal"), return_goal),
            "return goal differs from the takeoff-frame approach point",
        )

    check(
        all(phases[i + 1][0] > phases[i][0] for i in range(len(phases) - 1)),
        "phase timestamps must increase",
    )

    if report.failures:
        return None
    return Timeline(
        settings,
        home,
        start,
        {name: PhaseEvent.from_row(row) for name, row in phase.items()},
        tuple(legs),
    )


def check_images(data, timeline, report):
    settings, start, end = timeline.settings, timeline.start, timeline.end
    check, metrics = report.check, report.metrics
    images = [row for row in data.down_frames if start <= row[0] <= end]
    check(len(images) >= 3, "down-camera images were not recorded; enable mimir.images")
    check(
        all(row[1:] == (352, 288, 352 * 288) for row in images),
        "recorded down-camera images must be lossless CIF grayscale",
    )
    if images:
        image_times = [row[0] for row in images]
        metrics["down_frames"] = len(images)
        gaps = np.diff(image_times)
        metrics["max_down_gap_s"] = float(max(gaps, default=0.0))
        metrics["down_fps"] = (
            (len(image_times) - 1) / (image_times[-1] - image_times[0])
            if len(image_times) > 1
            else 0.0
        )
        check(
            image_times[0] <= start + 0.5
            and image_times[-1] >= end - 0.5
            and metrics["max_down_gap_s"] <= 0.5,
            "continuous down-camera recording does not cover the mission",
        )
        rate = settings.get("down_rate_hz")
        check(rate in (15, 30), "invalid requested down-camera rate")
        if rate not in (15, 30):
            return
        check(
            min(np.diff(image_times), default=1.0) >= 1.0 / rate - 0.001000001,
            "down-camera exceeds the requested capture rate",
        )
        # Intent comes from the shared resource API; actual work comes from native counters/images.
        intervals = []
        enabled, since = False, start
        requests = [
            (t, row["enabled"])
            for t, row in data.events
            if row["kind"] == "activity" and start <= t <= end
        ]
        check(bool(requests), "resource lifecycle requests are missing")
        for t, requested in requests:
            if requested != enabled:
                if t > since:
                    intervals.append((since, t, enabled))
                since, enabled = t, requested
        if since < end:
            intervals.append((since, end, enabled))
        windows = [(begin, finish) for begin, finish, active in intervals if active]
        check(len(windows) >= 2, "takeoff and landing require separate activations")
        check(
            len(image_times) - 1 <= (image_times[-1] - image_times[0] + 0.002) * rate,
            "down-camera long-term rate exceeds the configured rate",
        )
        runtime = data.scalars.get("/landing/runtime", [])
        check(bool(runtime), "landing lifecycle evidence is missing")
        check(
            all(row[3] == float(settings.get("transport") == "hil") for _, row in runtime),
            "landing backend does not match SIL/HIL execution",
        )
        for begin, finish in windows:
            check(
                any(row[0] == 1 for t, row in runtime if begin <= t <= finish),
                "landing pipeline was not active during a requested window",
            )
        check(
            any(row[0] == 0 for t, row in runtime if end <= t <= end + 0.5),
            "landing resources were not released at mission completion",
        )
        for begin, finish, active in intervals:
            if active or finish - begin <= 0.5:
                continue
            begin += 0.5  # Drain the bounded in-flight capture/inference work.
            idle = [(t, row) for t, row in runtime if begin <= t < finish]
            minimum = 3 if finish - begin >= 3 else int(finish - begin >= 1)
            check(
                len(idle) >= minimum and all(row[0] == 0 and row[2] == 0 for _, row in idle),
                "landing pipeline did not suspend during idle time",
            )
            check(
                len({row[1] for _, row in idle}) <= 1,
                "landing inference continued during idle time",
            )
        metrics["down_rate_limit_hz"] = rate
        counts = [row[1] for t, row in runtime if start <= t <= end + .5]
        metrics["landing_inferences"] = max(counts, default=0.) - min(counts, default=0.)


def evaluate_flight(data: Evidence, *, frame=None, first=True):
    report = Report()
    timeline = read_timeline(data, report)
    if timeline is None:
        return report.result()
    if timeline.settings["precision_land"]:
        check_images(data, timeline, report)
    check, metrics, result = report.check, report.metrics, report.result
    settings, home = timeline.settings, timeline.home
    start, end, phase, legs = timeline.start, timeline.end, timeline.phases, timeline.legs
    metrics.update(home_s=start, complete_s=end)
    roundtrip = settings["mission"] == "roundtrip"
    poses = [row for row in data.poses if start - 0.05 <= row[0] <= end + 0.05]
    modes = [
        (t, int(row[0])) for t, row in data.scalars.get("/fc/debug/state", []) if start <= t <= end
    ]
    check(len(poses) > 10 and len(modes) > 5, "ground truth or FC state evidence is missing")
    if len(poses) <= 10 or len(modes) <= 5:
        return result()
    times = np.array([row[0] for row in poses])
    positions = np.array([row[1] for row in poses])
    check(np.isfinite(positions).all() and np.isfinite(times).all(), "nonfinite ground truth")
    check(
        np.all(np.diff(times) > 0) and np.max(np.diff(times)) <= 0.1,
        "ground truth has duplicate times or coverage gaps",
    )
    check(
        times[0] <= start and times[-1] >= end - 0.05, "ground truth must cover the whole mission"
    )
    check(np.max(np.diff([t for t, _ in modes])) <= 0.35, "FC state evidence has coverage gaps")
    check(all(mode != 6 for _, mode in modes), "FC entered attitude fallback")
    if report.failures:
        return result()
    home_world = np.array([np.interp(start, times, positions[:, i]) for i in range(3)])
    gt_rotations = Slerp(times, Rotation.from_quat([row[2] for row in poses]))
    gt_rotation = gt_rotations([start])[0]
    if frame is None:
        alignment = gt_rotation * Rotation.from_quat(home["quaternion"]).inv()
        frame = (alignment, home_world - alignment.apply(home["position"]))
    alignment, origin = frame

    def world_goal(point):
        return origin + alignment.apply(point)

    def rows_between(name, stop):
        return [
            (t, row)
            for t, row in data.scalars.get(name, [])
            if phase[stop[0]].timestamp <= t <= phase[stop[1]].timestamp
        ]

    def phase_modes(begin, finish):
        return [
            value for t, value in modes if phase[begin].timestamp <= t <= phase[finish].timestamp
        ]

    approach = "acquire" if settings["precision_land"] else "land"
    flight_start = legs[0].travel.timestamp if roundtrip else phase[approach].timestamp
    check(
        any(mode == 2 for t, mode in modes if phase["takeoff"].timestamp <= t <= flight_start),
        "TAKEOFF state not observed",
    )
    from firefly_mujoco.scene import resolve

    pad = np.array(resolve(settings.get("scene")).hangar(settings["team"]))
    check(
        np.linalg.norm(home_world[:2] - pad[:2]) <= (0.03 if first else 0.12)
        and 0.015 <= home_world[2] - pad[2] <= 0.03,
        "mission did not start on the configured team pad",
    )
    height = positions[:, 2] - home_world[2]
    takeoff = height[times <= flight_start]
    check(takeoff.size and takeoff.max() >= settings["altitude"] - 0.2, "takeoff height was not reached")
    route_heights = [leg.travel.goal[2] - home["position"][2] for leg in legs]
    max_height = max([settings["altitude"], *route_heights])
    check(height.max() <= max_height + 0.4, "flight exceeded the altitude envelope")
    if max_height > settings["altitude"] + 1e-6:
        check(takeoff.size and takeoff.max() <= settings["altitude"] + 0.4, "takeoff exceeded the altitude envelope")
    metrics["max_height_m"] = float(height.max())
    takeoff_begin = next((t for t, mode in modes if mode == 2), None)
    if takeoff_begin is not None:
        metrics["takeoff_s"] = takeoff_begin
    takeoff_end = next(
        (t for t, mode in modes if takeoff_begin is not None and t > takeoff_begin and mode == 3),
        None,
    )
    if takeoff_begin is not None and takeoff_end is not None:
        selected = (times >= takeoff_begin) & (times <= takeoff_end)
        wrap = lambda angles: (angles + np.pi) % (2 * np.pi) - np.pi
        if selected.any():
            yaw = gt_rotations(times[selected]).as_euler("xyz")[:, 2]
            metrics["takeoff_actual_yaw_change_deg"] = float(
                np.abs(wrap(yaw - gt_rotation.as_euler("xyz")[2])).max() * 180 / np.pi
            )
            metrics["takeoff_xy_displacement_m"] = float(
                np.linalg.norm(positions[selected, :2] - home_world[:2], axis=1).max()
            )
        sources = [
            int(row[0])
            for t, row in data.scalars.get("/fc/imu/source", [])
            if takeoff_begin <= t <= takeoff_end
        ]
        if sources:
            metrics["takeoff_attitude_sources"] = sorted(set(sources))
            check(
                all(source == 1 for source in sources),
                "takeoff did not continuously use VIO attitude",
            )
        health = [
            row
            for t, row in data.scalars.get("/fc/imu/health", [])
            if takeoff_begin <= t <= takeoff_end
        ]
        if health:
            metrics["takeoff_posterior_max_age_s"] = max(row[2] for row in health)
            check(
                all(row[0] == row[1] == 1 for row in health),
                "takeoff VIO attitude or posterior became unavailable",
            )
        attitude = [
            (t, row)
            for t, row in data.scalars.get("/fc/imu/quaternion_xyzw", [])
            if takeoff_begin <= t <= takeoff_end
        ]
        if attitude:
            stamps, quaternions = zip(*attitude)
            estimated = alignment * Rotation.from_quat(quaternions)
            error = wrap(
                estimated.as_euler("xyz")[:, 2] - gt_rotations(stamps).as_euler("xyz")[:, 2]
            )
            metrics["takeoff_heading_error_deg"] = float(np.abs(error).max() * 180 / np.pi)
    velocities = np.gradient(positions, times, axis=0)
    if roundtrip:
        check(4 in phase_modes("return", approach), "return leg requires native TRACK state")
        metrics["waypoints"] = []
        for leg in legs:
            index = leg.index
            travel, hold, following = leg.travel, leg.hover, leg.following
            label = f"waypoint {index}"
            if hold is None:
                # The switch happens on entry to the radius. Find the physical crossing
                # around that instant, including the beginning of the outgoing leg.
                window = (times >= max(travel.timestamp, following.timestamp - 0.5)) & (
                    times <= following.timestamp + 0.5)
                check(window.any(), f"{label}: missing ground truth during passage")
                if not window.any():
                    continue
                samples = np.flatnonzero(window)
                distances = np.linalg.norm(positions[window] - world_goal(travel.goal), axis=1)
                nearest = int(distances.argmin())
                error = float(distances[nearest])
                speed = float(np.linalg.norm(velocities[samples[nearest], :2]))
                near_modes = [mode for t, mode in modes if times[samples[0]] <= t <= times[samples[-1]]]
                check(bool(near_modes) and all(mode == 4 for mode in near_modes),
                      f"{label}: continuous TRACK required through the waypoint")
                check(error <= 0.35 and speed > 0.25, f"{label}: physical fly-through missed or stopped")
                metrics["waypoints"].append({"index": index, "arrival": "pass",
                                             "closest_error_m": error, "pass_speed_mps": speed})
                continue
            check(
                any(mode == 4 for t, mode in modes if travel.timestamp <= t < hold.timestamp),
                f"{label}: native TRACK state missing",
            )
            check(
                hold.timestamp - travel.timestamp > 0
                and following.timestamp - hold.timestamp >= settings["hover_seconds"] - 0.05,
                f"{label}: hover duration too short",
            )
            holding_modes = [mode for t, mode in modes if hold.timestamp <= t < following.timestamp]
            check(
                bool(holding_modes) and all(mode == 3 for mode in holding_modes),
                f"{label}: native HOLD state missing during hover",
            )
            hover = (times >= hold.timestamp) & (times <= following.timestamp)
            if hover.any():
                error = np.linalg.norm(positions[hover] - world_goal(travel.goal), axis=1)
                speed = float(np.linalg.norm(velocities[hover], axis=1).max())
                metrics["waypoints"].append(
                    {
                        "index": index,
                        "arrival": "stop",
                        "max_error_m": float(error.max()),
                        "max_speed_mps": speed,
                        "hover_seconds": following.timestamp - hold.timestamp,
                    }
                )
                check(error.max() <= 0.3, f"{label}: physical waypoint hover error exceeds 0.3m")
                check(speed <= 0.25, f"{label}: physical waypoint hover did not settle")
            else:
                check(False, f"{label}: hover ground truth is missing")
        metrics["waypoint_max_error_m"] = max(
            (row.get("max_error_m", row.get("closest_error_m")) for row in metrics["waypoints"]), default=0.0
        )
        acquired_position = np.array(
            [np.interp(phase[approach].timestamp, times, positions[:, i]) for i in range(3)]
        )
        return_error = acquired_position - world_goal(phase["return"].goal)
        metrics["return_xy_error_m"] = float(np.linalg.norm(return_error[:2]))
        metrics["return_height_error_m"] = float(abs(return_error[2]))
        check(metrics["return_xy_error_m"] <= 0.3, "physical return horizontal error exceeds 0.3m")
        check(metrics["return_height_error_m"] <= 0.3, "physical return height error exceeds 0.3m")
        check(
            np.linalg.norm(acquired_position[:2] - pad[:2]) <= 0.45,
            "return did not reach the pad neighbourhood",
        )
        previous_height = settings["altitude"]
        for travel, following in [
            *((leg.travel, leg.following) for leg in legs), (phase["return"], phase[approach])
        ]:
            target_height = travel.goal[2] - home["position"][2]
            cruise = (times >= travel.timestamp) & (times <= following.timestamp)
            check(cruise.any() and height[cruise].min() >= min(previous_height, target_height) - 0.4,
                  "premature descent during waypoint flight")
            previous_height = target_height
    if settings["precision_land"]:
        check_precision(rows_between, phase, home, settings, check)
    landed_modes = [(t, mode) for t, mode in modes if t >= phase["land"].timestamp]
    check(
        bool(landed_modes)
        and any(mode == 5 for _, mode in landed_modes)
        and landed_modes[-1][1] == 0,
        "LAND must finish in DISARMED",
    )
    land_start = next((i for i, (_, mode) in enumerate(landed_modes) if mode == 5), None)
    if land_start is not None:
        check(
            all(mode in (0, 5) for _, mode in landed_modes[land_start:]),
            "FC left LAND before completion",
        )
    disarmed = next((t for t, mode in landed_modes if mode == 0), None)
    if disarmed is not None:
        metrics["disarmed_s"] = disarmed
        check(
            all(mode == 0 for t, mode in landed_modes if t >= disarmed), "FC rearmed after landing"
        )
        disarm_height = np.interp(disarmed, times, positions[:, 2]) - home_world[2]
        check(abs(disarm_height) <= 0.04, "disarmed before physical ground contact")
        disarm_speed = np.linalg.norm(
            [np.interp(disarmed, times, velocities[:, i]) for i in range(3)]
        )
        check(disarm_speed <= 0.15, "disarmed while physically moving")
        check(end - disarmed >= 0.5, "post-disarm settling evidence too short")
    tail = times >= end - 0.5
    metrics["landing_xy_error_m"] = float(
        np.linalg.norm(positions[tail, :2] - pad[:2], axis=1).max()
    )
    metrics["landing_height_error_m"] = float(np.abs(positions[tail, 2] - home_world[2]).max())
    metrics["landing_max_speed_mps"] = float(np.linalg.norm(velocities[tail], axis=1).max())
    if settings["precision_land"]:
        check(metrics["landing_xy_error_m"] <= 0.12, "physical landing error exceeds 0.12m")
    check(
        metrics["landing_height_error_m"] <= 0.02,
        "physical landing is not on the original pad surface",
    )
    check(metrics["landing_max_speed_mps"] <= 0.05, "vehicle is not stationary on the pad")
    return result()


def check_precision(rows_between, phase, home, settings, check):
    acquisition = [
        (t, row)
        for t, row in rows_between("/landing/target", ("acquire", "land"))
        if t > phase["acquire"].timestamp and row[3] == 1
    ]
    quality = dict(rows_between("/landing/quality", ("acquire", "land")))
    valid = [
        (t, row)
        for t, row in acquisition
        if t in quality
        and quality[t][0] == float(home["session"])
        and quality[t][1] == int(settings["team"] == "blue")
        and 0 <= quality[t][2] <= 2.0
        and 0.5 <= quality[t][3] <= 1.0
    ]
    check(
        len({t for t, _ in valid}) >= 3
        and round((valid[-1][0] - valid[0][0]) * 1e9) >= 150_000_000,
        "three fresh, same-session/team visual observations over 0.15s required after return",
    )
    check(
        bool(valid) and 0 <= phase["land"].timestamp - valid[-1][0] <= 0.25,
        "landing began with a stale visual target",
    )
    precision = [
        int(row[0]) for _, row in rows_between("/fc/debug/precision", ("land", "complete"))
    ]
    check(
        all(code in precision for code in (1, 2, 3))
        and precision.index(1) < precision.index(2) < precision.index(3),
        "ordered Align, Descend and Final precision stages required",
    )


def _window(rows, begin, end):
    stamp = itemgetter(0)
    return rows[bisect_left(rows, begin, key=stamp):bisect_right(rows, end, key=stamp)]


def _initial_frame(poses, home):
    """Align odometry once at the first ground anchor using bracketing truth samples."""
    stamp, anchor = home
    samples = _window(poses, stamp - .1, stamp + .1)
    if len(samples) < 2 or not samples[0][0] <= stamp <= samples[-1][0]:
        return None
    times, xyz, quats = zip(*samples)
    if np.any(np.diff(times) <= 0):
        return None
    rotation = Slerp(times, Rotation.from_quat(quats))(stamp) * Rotation.from_quat(anchor["quaternion"]).inv()
    position = [np.interp(stamp, times, axis) for axis in zip(*xyz)]
    return rotation, position - rotation.apply(anchor["position"])


def evaluate(data: Evidence):
    """Grade each flight in one fixed odometry frame; older recordings contain one flight."""
    headers = [row for _, row in data.events if row["kind"] == "system_run"]
    if not headers and not any("flight_id" in row for _, row in data.events):
        return evaluate_flight(data)
    report = Report()
    count = headers[0].get("planned_flights") if headers else None
    if len(headers) != 1 or type(count) is not int or count < 1:
        report.check(False, "invalid system run declaration")
        return report.result()
    groups = {}
    last_id = 0
    for stamp, row in data.events:
        if row["kind"] in ("system_run", "source", "provenance"):
            continue
        fid = row.get("flight_id")
        if type(fid) is not int or not 1 <= fid <= count:
            report.check(False, "missing or invalid flight identity")
            continue
        report.check(fid >= last_id, "flight events overlap or run out of order")
        last_id = fid
        groups.setdefault(fid, []).append((stamp, row))
    report.check(list(groups) == list(range(1, len(groups) + 1)) and len(groups) == count,
                 "run did not execute all declared flights in order")
    previous_end = None
    session = frame = None
    flights = []
    for fid, events in groups.items():
        settings = next((row for _, row in events if row["kind"] == "settings"), {})
        home = next(((t, row) for t, row in events if row["kind"] == "home"), None)
        end = next((t for t, row in events if row.get("name") == "complete"), events[-1][0])
        start = home[0] if home else events[0][0]
        next_events = groups.get(fid + 1)
        release_end = min(end + .5, next_events[0][0]) if next_events else end + .5
        poses = _window(data.poses, start - .05, end + .05)
        part = Evidence(events=events, poses=poses, recording_complete=data.recording_complete,
                        down_frames=_window(data.down_frames, start, end),
                        scalars={k: _window(rows, start, release_end) for k, rows in data.scalars.items()})
        boundary = Report()
        if home:
            _, anchor = home
            if fid == 1:
                session = anchor["session"]
                frame = _initial_frame(poses, home)
            boundary.check(frame is not None and anchor["session"] == session,
                           "shared VIO session or initial odometry alignment is missing")
        if previous_end is not None:
            idle = _window(data.scalars.get("/fc/debug/state", []), previous_end - .1, start)
            coverage = [previous_end - .1, *(t for t, _ in idle), start]
            boundary.check(start > previous_end and bool(idle) and max(np.diff(coverage)) <= .35
                           and all(v[0] == 0 for _, v in idle),
                           "next flight requires a disarmed ground interval")
        result = evaluate_flight(part, frame=frame, first=fid == 1)
        result["failures"] += boundary.failures
        result["passed"] = not result["failures"]
        flights.append(dict(flight_id=fid, shape=settings.get("shape"), **result))
        report.failures.extend(f"flight {fid}: {message}" for message in result["failures"])
        previous_end = end
    if count == 1 and flights:
        report.metrics.update(flights[0]["metrics"])
    report.metrics.update(planned_flights=count, vio_session=session, flights=flights)
    return report.result()


def verify_recording(path):
    return evaluate(read_recording(path))
