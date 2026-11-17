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


def read_recording(path):
    from rerun.chunk import RrdReader

    reader = RrdReader(path)
    [recording] = reader.recordings()
    # Indexed access requires a completed footer; live/truncated runs cannot pass.
    reader.store(store=recording).schema()
    data = Evidence()
    scalar_paths = {"/fc/debug/state", "/fc/debug/precision", "/landing/target", "/landing/quality"}
    for chunk in reader.stream(store=recording):
        entity = str(chunk.entity_path)
        if entity not in scalar_paths | {"/gt/pose", "/logs/flight", "/logs/mimir"}:
            continue
        batch = chunk.to_record_batch()
        if entity == "/logs/mimir" and "TextLog:text" in batch.schema.names:
            data.recording_complete |= any(text == "recording complete"
                for row in batch.column("TextLog:text").to_pylist() for text in row or [])
        if "sim_time" not in batch.schema.names:
            continue
        stamps = [value / 1e9 for value in batch.column("sim_time").cast("int64").to_pylist()]
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
    expected = ["takeoff", "outbound", "outbound_hover", "return", "acquire", "land", "complete"] if roundtrip else ["takeoff", "acquire", "land", "complete"]
    phases = [(stamp, row) for stamp, row in data.events if row["kind"] == "phase"]
    check([row["name"] for _, row in phases] == expected, "mission phase sequence is incomplete or out of order")
    check(settings["precision_land"], "precision landing must be selected")
    if len(phases) != len(expected) or [row["name"] for _, row in phases] != expected:
        return result()
    phase = {row["name"]: (stamp, row) for stamp, row in phases}
    start, home = homes[0]
    end = phase["complete"][0]
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
    gt_rotation = Slerp(times, Rotation.from_quat([row[2] for row in poses]))([start])[0]
    alignment = gt_rotation * Rotation.from_quat(home["quaternion"]).inv()

    def world_goal(point):
        return home_world + alignment.apply(np.array(point) - np.array(home["position"]))

    def rows_between(name, stop):
        return [(t, row) for t, row in data.scalars.get(name, []) if phase[stop[0]][0] <= t <= phase[stop[1]][0]]

    def phase_modes(begin, finish):
        return [value for t, value in modes if phase[begin][0] <= t <= phase[finish][0]]

    check(2 in phase_modes("takeoff", "outbound" if roundtrip else "acquire"), "TAKEOFF state not observed")
    from firefly_mujoco.scene import resolve
    pad = np.array(resolve(settings.get("scene")).hangar(settings["team"]))
    check(np.linalg.norm(home_world[:2] - pad[:2]) <= .03 and .015 <= home_world[2] - pad[2] <= .03,
          "mission did not start on the configured team pad")
    height = positions[:, 2] - home_world[2]
    check(height.max() >= settings["altitude"] - .2, "takeoff height was not reached")
    check(height.max() <= settings["altitude"] + .4, "flight exceeded the altitude envelope")
    metrics["max_height_m"] = float(height.max())
    velocities = np.gradient(positions, times, axis=0)
    if roundtrip:
        check(4 in phase_modes("outbound", "outbound_hover") and 4 in phase_modes("return", "acquire"),
              "both legs require native TRACK state")
        hover_start, hover_end = phase["outbound_hover"][0], phase["return"][0]
        check(hover_end - hover_start >= settings["hover_seconds"] - .05, "waypoint hover duration too short")
        hover = (times >= hover_start) & (times <= hover_end)
        if hover.any():
            error = np.linalg.norm(positions[hover] - world_goal(phase["outbound"][1]["goal"]), axis=1)
            metrics["waypoint_max_error_m"] = float(error.max())
            check(error.max() <= .3, "physical waypoint hover error exceeds 0.3m")
            check(np.linalg.norm(velocities[hover], axis=1).max() <= .25, "physical waypoint hover did not settle")
        else:
            check(False, "waypoint hover ground truth is missing")
        acquired_position = np.array([np.interp(phase["acquire"][0], times, positions[:, i]) for i in range(3)])
        return_error = acquired_position - world_goal(phase["return"][1]["goal"])
        metrics["return_xy_error_m"] = float(np.linalg.norm(return_error[:2]))
        metrics["return_height_error_m"] = float(abs(return_error[2]))
        check(metrics["return_xy_error_m"] <= .3, "physical return horizontal error exceeds 0.3m")
        check(metrics["return_height_error_m"] <= .3, "physical return height error exceeds 0.3m")
        check(np.linalg.norm(acquired_position[:2] - pad[:2]) <= .45, "return did not reach the pad neighbourhood")
        cruise = (times >= phase["outbound"][0]) & (times <= phase["acquire"][0])
        check(height[cruise].min() >= settings["altitude"] - .4, "premature descent during waypoint flight")
    acquisition = [(t, row) for t, row in rows_between("/landing/target", ("acquire", "land"))
                   if t > phase["acquire"][0] and row[3] == 1]
    quality = dict(rows_between("/landing/quality", ("acquire", "land")))
    valid = [(t, row) for t, row in acquisition if t in quality and quality[t][0] == float(home["session"])
             and quality[t][1] == int(settings["team"] == "blue") and 0 <= quality[t][2] <= 2.
             and .5 <= quality[t][3] <= 1.]
    check(len({t for t, _ in valid}) >= 3 and valid[-1][0] - valid[0][0] >= .15,
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
