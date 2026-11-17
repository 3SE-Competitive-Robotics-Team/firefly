"""Independently grade a closed SIL/HIL RRD; ground truth is used only here."""
import argparse
from dataclasses import dataclass, field
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass
class Evidence:
    events: list = field(default_factory=list)
    poses: list = field(default_factory=list)
    scalars: dict = field(default_factory=dict)
    recording_complete: bool = False
    down_frames: list = field(default_factory=list)


def read_recording(path):
    from rerun.chunk import RrdReader

    reader = RrdReader(path)
    [recording] = reader.recordings()
    # Indexed access requires a completed footer; live/truncated runs cannot pass.
    reader.store(store=recording).schema()
    data = Evidence()
    scalar_paths = {"/fc/debug/state", "/fc/debug/precision", "/landing/target", "/landing/quality", "/landing/runtime",
                    "/fc/imu/source", "/fc/imu/health", "/fc/imu/quaternion_xyzw"}
    for chunk in reader.stream(store=recording):
        entity = str(chunk.entity_path)
        if entity not in scalar_paths | {"/gt/pose", "/logs/flight", "/logs/mimir", "/cameras/down/image"}:
            continue
        batch = chunk.to_record_batch()
        if entity == "/logs/mimir" and "TextLog:text" in batch.schema.names:
            data.recording_complete |= any(text == "recording complete"
                for row in batch.column("TextLog:text").to_pylist() for text in row or [])
        if "sim_time" not in batch.schema.names:
            continue
        stamps = [value / 1e9 for value in batch.column("sim_time").cast("int64").to_pylist()]
        if entity == "/cameras/down/image" and "Image:buffer" in batch.schema.names:
            for stamp, image, formats in zip(stamps, batch.column("Image:buffer"), batch.column("Image:format").to_pylist()):
                if image.is_valid and len(image) == 1 and formats:
                    data.down_frames.append((stamp, formats[0]["width"], formats[0]["height"], len(image[0])))
        if entity == "/gt/pose" and "Transform3D:translation" in batch.schema.names:
            for stamp, position, quaternion in zip(stamps, batch.column("Transform3D:translation").to_pylist(),
                                                   batch.column("Transform3D:quaternion").to_pylist()):
                if position and quaternion:
                    data.poses.append((stamp, position[0], quaternion[0]))
        if entity in scalar_paths and "Scalars:scalars" in batch.schema.names:
            data.scalars.setdefault(entity, []).extend((stamp, row) for stamp, row in
                zip(stamps, batch.column("Scalars:scalars").to_pylist()) if row)
        if "TextLog:text" in batch.schema.names:
            for stamp, row in zip(stamps, batch.column("TextLog:text").to_pylist()):
                for text in row or []:
                    if entity == "/logs/flight" and text.startswith("mission {"):
                        data.events.append((stamp, json.loads(text.removeprefix("mission "))))
    data.events.sort(key=lambda row: row[0])
    data.poses.sort(key=lambda row: row[0])
    for values in data.scalars.values():
        values.sort(key=lambda row: row[0])
    data.down_frames.sort()
    return data


def evaluate(data):
    """Check state sequence, fresh visual reacquisition and physical flight/landing."""
    failures, metrics = [], {}

    def check(condition, message):
        if not condition:
            failures.append(message)

    def result():
        return {"passed": not failures, "failures": failures, "metrics": metrics}

    settings_rows = [row for _, row in data.events if row["kind"] == "settings"]
    check(data.recording_complete, "Mimir did not confirm complete recording integrity")
    homes = [(stamp, row) for stamp, row in data.events if row["kind"] == "home"]
    check(len(settings_rows) == 1 and len(homes) == 1, "one settings event and one takeoff-frame anchor required")
    check(not any(row["kind"] == "failure" for _, row in data.events), "mission reported a failure")
    if not settings_rows or not homes:
        return result()
    settings = settings_rows[0]
    roundtrip = settings["mission"] == "roundtrip"
    count = settings.get("waypoint_count")
    if count is not None:
        check(type(count) is int and (1 <= count <= 16 if roundtrip else count == 0),
              "invalid waypoint count")
        if failures:
            return result()
    expected = (["takeoff"] + ["waypoint", "waypoint_hover"] * count + ["return", "acquire", "land", "complete"]
                if roundtrip and count is not None else
                ["takeoff", "outbound", "outbound_hover", "return", "acquire", "land", "complete"]
                if roundtrip else ["takeoff", "acquire", "land", "complete"])
    phases = [(stamp, row) for stamp, row in data.events if row["kind"] == "phase"]
    check([row["name"] for _, row in phases] == expected, "mission phase sequence is incomplete or out of order")
    check(settings["precision_land"], "precision landing must be selected")
    if len(phases) != len(expected) or [row["name"] for _, row in phases] != expected:
        return result()
    phase = {row["name"]: (stamp, row) for stamp, row in phases}
    start, home = homes[0]
    end = phase["complete"][0]
    legs = []
    if roundtrip:
        if count is None:
            legs = [(phase["outbound"], phase["outbound_hover"], phase["return"])]
        else:
            planned = [(t, row) for t, row in data.events if row["kind"] == "waypoint"]
            check([row.get("index") for _, row in planned] == list(range(count)),
                  "ordered waypoint declarations are missing or duplicated")
            if len(planned) != count:
                return result()
            for index in range(count):
                travel, hover, following = phases[1 + 2*index:4 + 2*index]
                legs.append((travel, hover, following))
                offset = planned[index][1].get("offset", [])
                check(len(offset) == 2 and np.isfinite(offset).all(), "invalid declared waypoint offset")
                if len(offset) != 2:
                    return result()
                goal = np.array(home["position"]) + [offset[0], offset[1], settings["altitude"]]
                check(planned[index][0] <= start and travel[1].get("index") == hover[1].get("index") == index
                      and np.array_equal(travel[1].get("goal"), goal)
                      and np.array_equal(hover[1].get("goal"), goal),
                      f"waypoint {index}: phase identity/goal differs from the declared route")
            return_goal = np.array(home["position"]) + [*settings["return_offset"], settings["altitude"]]
            check(np.array_equal(phase["return"][1].get("goal"), return_goal),
                  "return goal differs from the takeoff-frame approach point")
    images = [row for row in data.down_frames if start <= row[0] <= end]
    check(len(images) >= 3, "down-camera images were not recorded; enable mimir.images")
    check(all(row[1:] == (352, 288, 352 * 288) for row in images), "recorded down-camera images must be lossless CIF grayscale")
    if images:
        image_times = [row[0] for row in images]
        metrics["down_frames"] = len(images)
        gaps = np.diff(image_times)
        metrics["max_down_gap_s"] = float(max(gaps, default=0.))
        metrics["down_fps"] = (len(image_times) - 1) / (image_times[-1] - image_times[0]) if len(image_times) > 1 else 0.
        check(image_times[0] <= start + .5 and image_times[-1] >= end - .5
              and metrics["max_down_gap_s"] <= .5,
              "continuous down-camera recording does not cover the mission")
        rate = settings.get("down_rate_hz")
        if rate is not None:
            check(rate in (15, 30), "invalid requested down-camera rate")
            check(min(np.diff(image_times), default=1.) >= 1. / rate - .001000001,
                  "down-camera exceeds the requested capture rate")
            # Intent comes from the shared resource API; actual work comes from native counters/images.
            intervals = []
            enabled, since = False, start
            requests = [(t, row["enabled"]) for t, row in data.events
                        if row["kind"] == "activity" and start <= t <= end]
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
            check(len(image_times) - 1 <= (image_times[-1] - image_times[0] + .002) * rate,
                  "down-camera long-term rate exceeds the configured rate")
            runtime = data.scalars.get("/landing/runtime", [])
            check(bool(runtime), "landing lifecycle evidence is missing")
            check(all(row[3] == float(settings.get("transport") == "hil") for _, row in runtime),
                  "landing backend does not match SIL/HIL execution")
            for begin, finish in windows:
                check(any(row[0] == 1 for t, row in runtime if begin <= t <= finish),
                      "landing pipeline was not active during a requested window")
            check(any(row[0] == 0 for t, row in runtime if end <= t <= end + .5),
                  "landing resources were not released at mission completion")
            for begin, finish, active in intervals:
                if active or finish - begin <= .5:
                    continue
                begin += .5  # Drain the bounded in-flight capture/inference work.
                idle = [(t, row) for t, row in runtime if begin <= t < finish]
                minimum = 3 if finish - begin >= 3 else int(finish - begin >= 1)
                check(len(idle) >= minimum and all(row[0] == 0 and row[2] == 0 for _, row in idle),
                      "landing pipeline did not suspend during idle time")
                check(len({row[1] for _, row in idle}) <= 1,
                      "landing inference continued during idle time")
            metrics["down_rate_limit_hz"] = rate
            metrics["landing_inferences"] = max((row[1] for _, row in runtime), default=0.)
    check(all(phases[i + 1][0] > phases[i][0] for i in range(len(phases) - 1)), "phase timestamps must increase")
    poses = [row for row in data.poses if start - .05 <= row[0] <= end + .05]
    modes = [(t, int(row[0])) for t, row in data.scalars.get("/fc/debug/state", []) if start <= t <= end]
    check(len(poses) > 10 and len(modes) > 5, "ground truth or FC state evidence is missing")
    if len(poses) <= 10 or len(modes) <= 5:
        return result()
    times = np.array([row[0] for row in poses])
    positions = np.array([row[1] for row in poses])
    check(np.isfinite(positions).all() and np.isfinite(times).all(), "nonfinite ground truth")
    check(np.all(np.diff(times) > 0) and np.max(np.diff(times)) <= .1, "ground truth has duplicate times or coverage gaps")
    check(times[0] <= start and times[-1] >= end - .05, "ground truth must cover the whole mission")
    check(np.max(np.diff([t for t, _ in modes])) <= .35, "FC state evidence has coverage gaps")
    check(all(mode != 6 for _, mode in modes), "FC entered attitude fallback")
    if failures:
        return result()
    home_world = np.array([np.interp(start, times, positions[:, i]) for i in range(3)])
    gt_rotations = Slerp(times, Rotation.from_quat([row[2] for row in poses]))
    gt_rotation = gt_rotations([start])[0]
    alignment = gt_rotation * Rotation.from_quat(home["quaternion"]).inv()

    def world_goal(point):
        return home_world + alignment.apply(np.array(point) - np.array(home["position"]))

    def rows_between(name, stop):
        return [(t, row) for t, row in data.scalars.get(name, []) if phase[stop[0]][0] <= t <= phase[stop[1]][0]]

    def phase_modes(begin, finish):
        return [value for t, value in modes if phase[begin][0] <= t <= phase[finish][0]]

    flight_start = legs[0][0][0] if roundtrip else phase["acquire"][0]
    check(any(mode == 2 for t, mode in modes if phase["takeoff"][0] <= t <= flight_start), "TAKEOFF state not observed")
    from firefly_mujoco.scene import resolve
    pad = np.array(resolve(settings.get("scene")).hangar(settings["team"]))
    check(np.linalg.norm(home_world[:2] - pad[:2]) <= .03 and .015 <= home_world[2] - pad[2] <= .03,
          "mission did not start on the configured team pad")
    height = positions[:, 2] - home_world[2]
    check(height.max() >= settings["altitude"] - .2, "takeoff height was not reached")
    check(height.max() <= settings["altitude"] + .4, "flight exceeded the altitude envelope")
    metrics["max_height_m"] = float(height.max())
    takeoff_begin = next((t for t, mode in modes if mode == 2), None)
    takeoff_end = next((t for t, mode in modes if takeoff_begin is not None and t > takeoff_begin and mode == 3), None)
    if takeoff_begin is not None and takeoff_end is not None:
        selected = (times >= takeoff_begin) & (times <= takeoff_end)
        wrap = lambda angles: (angles + np.pi) % (2*np.pi) - np.pi
        if selected.any():
            yaw = gt_rotations(times[selected]).as_euler("xyz")[:, 2]
            metrics["takeoff_actual_yaw_change_deg"] = float(np.abs(wrap(yaw - gt_rotation.as_euler("xyz")[2])).max() * 180 / np.pi)
            metrics["takeoff_xy_displacement_m"] = float(np.linalg.norm(positions[selected, :2] - home_world[:2], axis=1).max())
        sources = [int(row[0]) for t, row in data.scalars.get("/fc/imu/source", []) if takeoff_begin <= t <= takeoff_end]
        if sources:
            metrics["takeoff_attitude_sources"] = sorted(set(sources))
            check(all(source == 1 for source in sources), "takeoff did not continuously use VIO attitude")
        health = [row for t, row in data.scalars.get("/fc/imu/health", []) if takeoff_begin <= t <= takeoff_end]
        if health:
            metrics["takeoff_posterior_max_age_s"] = max(row[2] for row in health)
            check(all(row[0] == row[1] == 1 for row in health), "takeoff VIO attitude or posterior became unavailable")
        attitude = [(t, row) for t, row in data.scalars.get("/fc/imu/quaternion_xyzw", []) if takeoff_begin <= t <= takeoff_end]
        if attitude:
            stamps, quaternions = zip(*attitude)
            estimated = alignment * Rotation.from_quat(quaternions)
            error = wrap(estimated.as_euler("xyz")[:, 2] - gt_rotations(stamps).as_euler("xyz")[:, 2])
            metrics["takeoff_heading_error_deg"] = float(np.abs(error).max() * 180 / np.pi)
    velocities = np.gradient(positions, times, axis=0)
    if roundtrip:
        check(4 in phase_modes("return", "acquire"), "return leg requires native TRACK state")
        metrics["waypoints"] = []
        for index, (travel, hold, following) in enumerate(legs):
            label = f"waypoint {index}"
            check(any(mode == 4 for t, mode in modes if travel[0] <= t < hold[0]),
                  f"{label}: native TRACK state missing")
            check(hold[0] - travel[0] > 0 and following[0] - hold[0] >= settings["hover_seconds"] - .05,
                  f"{label}: hover duration too short")
            holding_modes = [mode for t, mode in modes if hold[0] <= t < following[0]]
            check(bool(holding_modes) and all(mode == 3 for mode in holding_modes),
                  f"{label}: native HOLD state missing during hover")
            hover = (times >= hold[0]) & (times <= following[0])
            if hover.any():
                error = np.linalg.norm(positions[hover] - world_goal(travel[1]["goal"]), axis=1)
                speed = float(np.linalg.norm(velocities[hover], axis=1).max())
                metrics["waypoints"].append({"index": index, "max_error_m": float(error.max()),
                                             "max_speed_mps": speed, "hover_seconds": following[0] - hold[0]})
                check(error.max() <= .3, f"{label}: physical waypoint hover error exceeds 0.3m")
                check(speed <= .25, f"{label}: physical waypoint hover did not settle")
            else:
                check(False, f"{label}: hover ground truth is missing")
        metrics["waypoint_max_error_m"] = max((row["max_error_m"] for row in metrics["waypoints"]), default=0.)
        acquired_position = np.array([np.interp(phase["acquire"][0], times, positions[:, i]) for i in range(3)])
        return_error = acquired_position - world_goal(phase["return"][1]["goal"])
        metrics["return_xy_error_m"] = float(np.linalg.norm(return_error[:2]))
        metrics["return_height_error_m"] = float(abs(return_error[2]))
        check(metrics["return_xy_error_m"] <= .3, "physical return horizontal error exceeds 0.3m")
        check(metrics["return_height_error_m"] <= .3, "physical return height error exceeds 0.3m")
        check(np.linalg.norm(acquired_position[:2] - pad[:2]) <= .45, "return did not reach the pad neighbourhood")
        cruise = (times >= flight_start) & (times <= phase["acquire"][0])
        check(height[cruise].min() >= settings["altitude"] - .4, "premature descent during waypoint flight")
    acquisition = [(t, row) for t, row in rows_between("/landing/target", ("acquire", "land"))
                   if t > phase["acquire"][0] and row[3] == 1]
    quality = dict(rows_between("/landing/quality", ("acquire", "land")))
    valid = [(t, row) for t, row in acquisition if t in quality and quality[t][0] == float(home["session"])
             and quality[t][1] == int(settings["team"] == "blue") and 0 <= quality[t][2] <= 2.
             and .5 <= quality[t][3] <= 1.]
    check(len({t for t, _ in valid}) >= 3 and round((valid[-1][0] - valid[0][0]) * 1e9) >= 150_000_000,
          "three fresh, same-session/team visual observations over 0.15s required after return")
    check(bool(valid) and 0 <= phase["land"][0] - valid[-1][0] <= .25, "landing began with a stale visual target")
    precision = [int(row[0]) for _, row in rows_between("/fc/debug/precision", ("land", "complete"))]
    check(all(code in precision for code in (1, 2, 3))
          and precision.index(1) < precision.index(2) < precision.index(3),
          "ordered Align, Descend and Final precision stages required")
    landed_modes = [(t, mode) for t, mode in modes if t >= phase["land"][0]]
    check(bool(landed_modes) and any(mode == 5 for _, mode in landed_modes) and landed_modes[-1][1] == 0, "LAND must finish in DISARMED")
    land_start = next((i for i, (_, mode) in enumerate(landed_modes) if mode == 5), None)
    if land_start is not None:
        check(all(mode in (0, 5) for _, mode in landed_modes[land_start:]), "FC left LAND before completion")
    disarmed = next((t for t, mode in landed_modes if mode == 0), None)
    if disarmed is not None:
        check(all(mode == 0 for t, mode in landed_modes if t >= disarmed), "FC rearmed after landing")
        disarm_height = np.interp(disarmed, times, positions[:, 2]) - home_world[2]
        check(abs(disarm_height) <= .04, "disarmed before physical ground contact")
        disarm_speed = np.linalg.norm([np.interp(disarmed, times, velocities[:, i]) for i in range(3)])
        check(disarm_speed <= .15, "disarmed while physically moving")
        check(end - disarmed >= .5, "post-disarm settling evidence too short")
    tail = times >= end - .5
    metrics["landing_xy_error_m"] = float(np.linalg.norm(positions[tail, :2] - pad[:2], axis=1).max())
    metrics["landing_height_error_m"] = float(np.abs(positions[tail, 2] - home_world[2]).max())
    metrics["landing_max_speed_mps"] = float(np.linalg.norm(velocities[tail], axis=1).max())
    check(metrics["landing_xy_error_m"] <= .12, "physical landing error exceeds 0.12m")
    check(metrics["landing_height_error_m"] <= .02, "physical landing is not on the original pad surface")
    check(metrics["landing_max_speed_mps"] <= .05, "vehicle is not stationary on the pad")
    return result()


def verify_recording(path):
    return evaluate(read_recording(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    args = parser.parse_args()
    try:
        report = verify_recording(args.recording)
    except Exception as error:
        report = {"passed": False, "failures": [f"invalid recording: {error}"], "metrics": {}}
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
