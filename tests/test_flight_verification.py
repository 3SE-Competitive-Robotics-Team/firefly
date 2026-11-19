"""Unit coverage of mission gates and independent physical scoring."""
import copy
from pathlib import Path
import sys
import tomllib

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from flight_mission.recording import Evidence
from flight_mission.verification import evaluate, evaluate_native
from firefly_mujoco.scene import resolve


@pytest.mark.parametrize("team", ["red", "blue"])
def test_physical_roundtrip_passes_for_either_team(team):
    result = evaluate(successful_evidence(team))
    assert result["passed"], result


@pytest.mark.parametrize("fault", [None, "session", "intervening_arm", "shifted_odom", "missing"])
def test_flights_keep_one_frame_and_independent_grades(fault):
    data = successful_evidence()
    second = copy.deepcopy(data)
    second.scalars["/fc/debug/state"][0] = (1., [0])
    shift = 19.
    data.events = [(0., {"kind": "system_run", "planned_flights": 2})] + [
        (t, dict(row, flight_id=1)) for t, row in data.events]
    if fault == "shifted_odom":
        # A one-metre VIO translation jump moves every declared goal with the home.
        for _, row in second.events:
            if row["kind"] == "home":
                row["position"][0] += 1.
            if "goal" in row:
                row["goal"] = [row["goal"][0] + 1., *row["goal"][1:]]
    if fault == "session":
        next(row for _, row in second.events if row["kind"] == "home")["session"] += 1
    if fault != "missing":
        data.events.extend((t + shift, dict(row, flight_id=2)) for t, row in second.events)
    data.poses.extend((t + shift, p, q) for t, p, q in second.poses)
    data.down_frames.extend((t + shift, *dims) for t, *dims in second.down_frames)
    states = data.scalars["/fc/debug/state"]
    first_end = next(t for t, row in data.events if row.get("name") == "complete")
    states.extend((float(t), [1 if fault == "intervening_arm" else 0])
                  for t in np.arange(first_end + .05, shift + 1., .1))
    for key, rows in second.scalars.items():
        data.scalars[key].extend((t + shift, v) for t, v in rows)
    report = evaluate(data)
    assert report["metrics"]["flights"][0]["passed"], report
    assert report["passed"] == (fault is None), report
    if fault == "shifted_odom":
        assert any("physical waypoint hover error" in s for s in report["failures"])


@pytest.mark.parametrize("last, passed", [(12.934, True), (12.933999, False)])
def test_visual_span_uses_rrd_nanosecond_precision(last, passed):
    data = successful_evidence()
    stamps = (12.784, 12.867, last)
    data.scalars["/landing/target"] = [(t, [0., 0., 0., 1.]) for t in stamps]
    data.scalars["/landing/quality"] = [(t, [7., 0., .2, .9]) for t in stamps]
    assert evaluate(data)["passed"] == passed


@pytest.mark.parametrize("fault", ["cruise_gap", "cruise_compute", "missing_runtime", "rate", "invalid_rate", "missing_rate", "rate_drift", "not_released", "cpu_hil"])
def test_continuous_video_and_on_demand_inference_are_independently_scored(fault):
    data = successful_evidence()
    runtime = data.scalars["/landing/runtime"]
    if fault == "cruise_gap":
        data.down_frames = [row for row in data.down_frames if not 4. < row[0] < 6.]
    elif fault == "cruise_compute":
        runtime[5][1][1] += 1
    elif fault == "missing_runtime":
        data.scalars.pop("/landing/runtime")
    elif fault == "rate":
        data.down_frames.insert(1, (1.11, 352, 288, 352 * 288))
    elif fault == "invalid_rate":
        data.events[0][1]["down_rate_hz"] = 0
    elif fault == "missing_rate":
        data.events[0][1].pop("down_rate_hz")
    elif fault == "rate_drift":
        data.down_frames = [(float(t), 352, 288, 352 * 288) for t in np.arange(1.1, 19., 1/15.1)]
    elif fault == "not_released":
        runtime[-1][1][0] = 1.
    elif fault == "cpu_hil":
        data.events[0][1]["transport"] = "hil"
    report = evaluate(data)
    assert not report["passed"], report


def test_scoring_uses_initial_full_coordinate_alignment():
    data = successful_evidence()
    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_euler("z", 90, degrees=True)
    center = np.array(resolve().hangar("red")) + [0., 0., .021]
    data.poses = [(t, (center + rotation.apply(np.array(p) - center)).tolist(), rotation.as_quat().tolist())
                  for t, p, _ in data.poses]
    assert evaluate(data)["passed"]


@pytest.mark.parametrize("fault", [None, "standby", "stale"])
def test_takeoff_audit_distinguishes_vio_participation_from_heading_drift(fault):
    from scipy.spatial.transform import Rotation
    data = successful_evidence()
    data.poses = [(t, p, Rotation.from_euler("z", min(max(t-1.5, 0.), 1.)*.1).as_quat().tolist())
                  for t, p, _ in data.poses]
    stamps = np.arange(1., 19., .1)
    data.scalars["/fc/imu/source"] = [(float(t), [2. if fault == "standby" else 1.]) for t in stamps]
    data.scalars["/fc/imu/health"] = [(float(t), [1., 0. if fault == "stale" else 1., .05]) for t in stamps]
    data.scalars["/fc/imu/quaternion_xyzw"] = [(float(t), [0., 0., 0., 1.]) for t in stamps]
    report = evaluate(data)
    assert report["passed"] == (fault is None), report
    assert report["metrics"]["takeoff_actual_yaw_change_deg"] == pytest.approx(np.degrees(.1))
    assert report["metrics"]["takeoff_heading_error_deg"] == pytest.approx(np.degrees(.1))
    assert report["metrics"]["takeoff_posterior_max_age_s"] == .05


@pytest.mark.parametrize("fault", ["no_truth", "never_left", "stale_vision", "wrong_team", "wrong_height", "early_disarm", "no_return", "truth_gap", "wrong_xy", "short_hover", "wrong_precision_order", "left_land", "recorder_loss", "no_down", "down_gap", "down_size"])
def test_command_completion_cannot_hide_physical_or_protocol_failure(fault):
    data = successful_evidence()
    if fault == "no_truth":
        data.poses = []
    elif fault == "never_left":
        x = resolve().hangar("red")[0]
        data.poses = [(t, [x, p[1], p[2]], q) for t, p, q in data.poses]
    elif fault == "stale_vision":
        data.scalars["/landing/target"] = [(t - 2, row) for t, row in data.scalars["/landing/target"]]
    elif fault == "wrong_team":
        for _, row in data.scalars["/landing/quality"]:
            row[1] = 1.
    elif fault == "wrong_height":
        data.poses = [(t, [p[0], p[1], p[2] + (.15 if t >= 17.9 else 0.)], q) for t, p, q in data.poses]
    elif fault == "early_disarm":
        data.scalars["/fc/debug/state"] = [(t, [0] if t >= 13.5 else row) for t, row in data.scalars["/fc/debug/state"]]
    elif fault == "no_return":
        data.events = [(t, row) for t, row in data.events if row.get("name") != "return"]
    elif fault == "truth_gap":
        data.poses = [row for row in data.poses if not 10 < row[0] < 11]
    elif fault == "wrong_xy":
        data.poses = [(t, [p[0] + (.2 if t >= 17.9 else 0.), p[1], p[2]], q) for t, p, q in data.poses]
    elif fault == "short_hover":
        data.events[0][1]["hover_seconds"] = 5.
    elif fault == "wrong_precision_order":
        data.scalars["/fc/debug/precision"] = [(t, [{1: 3, 3: 1}.get(row[0], row[0])])
                                                 for t, row in data.scalars["/fc/debug/precision"]]
    elif fault == "left_land":
        data.scalars["/fc/debug/state"] = [(t, [3] if 15 < t < 16 else row) for t, row in data.scalars["/fc/debug/state"]]
    elif fault == "recorder_loss":
        data.recording_complete = False
    elif fault == "no_down":
        data.down_frames = []
    elif fault == "down_gap":
        data.down_frames = [row for row in data.down_frames if not 10 < row[0] < 11]
    elif fault == "down_size":
        data.down_frames = [(t, 320, 240, size) for t, _, _, size in data.down_frames]
    assert not evaluate(data)["passed"], fault




def test_mission_sensor_profile_preserves_imu_and_sets_camera_budget():
    from firefly_sim.main import decode_config
    path = Path(__file__).resolve().parents[1] / "configs/sim-mission.toml"
    physics, imu, pose, _, _ = decode_config(tomllib.loads(path.read_text()))
    assert physics == imu == .001
    assert pose == pytest.approx(1 / 30)


def route(shape, team="red"):
    config = Path(__file__).resolve().parents[1] / f"configs/mission-{shape}.toml"
    points = tomllib.loads(config.read_text())["waypoints"]
    sign = 1 if team == "red" else -1
    return tuple((sign * p["offset"][0], sign * p["offset"][1]) for p in points)


def test_cuboid_covers_twelve_edges_at_two_flight_levels():
    configs = Path(__file__).resolve().parents[1] / "configs"
    points = tomllib.loads((configs / "mission-cuboid.toml").read_text())["waypoints"]
    assert all(p["arrival"] == "stop" for p in points)
    red = np.array([p["offset"] for p in points])
    assert red[0] == pytest.approx([3., 0., 3.3])
    assert np.ptp(red, axis=0) == pytest.approx([3., 2., 2.])
    assert np.unique(red, axis=0).shape == (8, 3)
    assert np.all(np.count_nonzero(np.diff(red, axis=0), axis=1) == 1)
    assert len({frozenset((tuple(a), tuple(b))) for a, b in zip(red, red[1:])}) == 12
    desktop = tomllib.loads((configs / "planner-cuboid-sil.toml").read_text())
    assert desktop["virtual_ceiling"] >= red[:, 2].max() + desktop["obstacle_clearance"]


def test_multilevel_waypoints_are_scored_at_their_declared_height():
    data = successful_evidence(points=((2., 0., 1.), (2., 0., .5)))
    assert evaluate(data)["passed"]
    second = next(row for _, row in data.events if row["kind"] == "waypoint" and row["index"] == 1)
    second["offset"] = (2., 0., 1.)
    assert not evaluate(data)["passed"]


@pytest.mark.parametrize("fault", [None, "missing_header", "missing_liftoff", "changed_run", "old_names", "missing_hold"])
def test_native_events_bind_the_flight_only_after_measured_liftoff(fault):
    data = successful_evidence()
    settings = data.events[0][1]
    data.capture = {"down_rate_hz": settings.pop("down_rate_hz"), "scene": "rmuc2027@1.0.0"}
    events = [(0., {"kind": "system_run", "run": "boot"})]
    observed = [(1.02, {"kind": "phase", "name": "arm"}), (1.5, {"kind": "airborne"}),
                (2.95, {"kind": "phase", "name": "hover"}),
                (5.9, {"kind": "phase", "name": "hold", "index": 0}),
                (11.8, {"kind": "phase", "name": "hold", "index": 1}),
                (11.9, {"kind": "phase", "name": "dwell", "index": 1}),
                (18.9, {"kind": "phase", "name": "ground"})]
    for stamp, row in sorted(data.events + observed, key=lambda pair: pair[0]):
        name = row.get("name")
        if name in ("waypoint", "return"):
            if name == "return":
                row["index"] = 1
            row["name"] = "navigate" if fault != "old_names" else name
        elif name == "waypoint_hover":
            row["name"] = "dwell"
        if fault == "missing_liftoff" and row["kind"] == "airborne":
            continue
        if fault == "missing_hold" and name == "hold":
            continue
        events.append((stamp, dict(row, run="changed" if fault == "changed_run" and stamp > 6 else "boot",
                                   mission_id="inspection", flight_id=1 if stamp >= 1.5 else None)))
    data.events = events[1:] if fault == "missing_header" else events
    result = evaluate(data)
    assert result["passed"] == (fault is None), result
    data.poses.clear()
    data.capture.clear()
    board = evaluate_native(data, board=True)
    assert board["passed"] == (fault is None), board
    if fault is None:
        assert not board["metrics"]["flights"][0]["ground_truth_available"]
        assert not evaluate(data)["passed"]
        data.scalars["/fc/debug/state"].clear()
        assert not evaluate_native(data, board=True)["passed"]


def test_restart_is_the_same_run_and_ground_cancellation_has_no_flight_number():
    header = {"kind": "system_run", "run": "boot"}
    data = Evidence(events=[(0., header), (1., header)], recording_complete=True)
    for i, name in enumerate(["arm", "recover", "interrupted"]):
        data.events.append((i+2., dict(kind="phase", name=name, run="boot", mission_id="cancelled", flight_id=None)))
    result = evaluate(data)
    assert result["passed"] and result["metrics"]["flights"] == []
    assert result["metrics"]["ground_attempts"] == ["cancelled"]


def test_cuboid_may_climb_above_takeoff_only_after_entering_the_route():
    data = successful_evidence(points=((3., 0., 3.3), (3., 0., 1.3)))
    data.events[0][1]["shape"] = "cuboid"
    assert evaluate(data)["passed"]
    _, pos, _ = next(row for row in data.poses if 2.5 < row[0] < 3.)
    pos[2] += 1.
    assert "takeoff exceeded the altitude envelope" in evaluate(data)["failures"]




def successful_evidence(team="red", *, points=((2., 0.),)):
    data = Evidence(recording_complete=True)
    data.events = [
        (0., {"kind": "settings", "mission": "roundtrip", "precision_land": True,
              "team": team, "altitude": 1., "hover_seconds": 3., "waypoint_count": len(points),
              "return_offset": [.12, -.08], "down_rate_hz": 15, "transport": "sil"}),
        (1., {"kind": "home", "position": [0., 0., 0.], "quaternion": [0., 0., 0., 1.], "session": 7}),
        (1.1, {"kind": "phase", "name": "takeoff"}),
    ]
    knots = [(0., [0., 0., 0.]), (1., [0., 0., 0.]), (3., [0., 0., 1.])]
    tracks = []
    for index, point in enumerate(points):
        start = 3. + 6.*index
        goal = [*point, 1.] if len(point) == 2 else list(point)
        data.events += [
            (.5, {"kind": "waypoint", "index": index, "offset": point}),
            (start, {"kind": "phase", "name": "waypoint", "index": index, "goal": goal}),
            (start + 3., {"kind": "phase", "name": "waypoint_hover", "index": index, "goal": goal}),
        ]
        knots += [(start + 3., goal), (start + 6., goal)]
        tracks.append((start, start + 3.))
    returning = 3. + 6.*len(points)
    acquire, land, contact, end = returning + 3., returning + 4., returning + 9., returning + 10.
    data.events += [(returning, {"kind": "phase", "name": "return", "goal": [.12, -.08, 1.]}),
                    (acquire, {"kind": "phase", "name": "acquire"}),
                    (land, {"kind": "phase", "name": "land"}),
                    (end, {"kind": "phase", "name": "complete"})]
    data.events.sort(key=lambda row: row[0])
    knots += [(acquire, [.12, -.08, 1.]), (land, [.12, -.08, 1.]),
              (contact, [0., 0., 0.]), (end + 1., [0., 0., 0.])]
    pad = np.array(resolve().hangar(team)) + [0., 0., .021]
    knot_times = [t for t, _ in knots]
    for t in np.arange(.96, end + .05, .02):
        i = min(np.searchsorted(knot_times, t, side="right") - 1, len(knots) - 2)
        f = (t - knots[i][0]) / (knots[i+1][0] - knots[i][0])
        f = f*f*(3 - 2*f)
        position = pad + np.array(knots[i][1])*(1-f) + np.array(knots[i+1][1])*f
        data.poses.append((float(t), position.tolist(), [0., 0., 0., 1.]))
    data.down_frames = [(float(t), 352, 288, 352 * 288) for t in np.arange(1.1, end, 1/15)]
    data.events += [(t, {"kind": "activity", "enabled": enabled})
                    for t, enabled in [(1.1, True), (3., False), (acquire, True), (end, False)]]
    data.events.sort(key=lambda row: row[0])
    runtime = [(1.1, [1., 0., 1., 0.]), (2., [1., 10., 1., 0.])]
    runtime += [(float(t), [0., 20., 0., 0.]) for t in np.arange(3., acquire)]
    runtime += [(acquire, [1., 20., 1., 0.]), (land+3., [1., 70., 1., 0.]), (end, [0., 100., 0., 0.])]
    data.scalars["/landing/runtime"] = runtime
    tracks.append((returning, acquire))
    for t in np.arange(1., end + .01, .1):
        mode = (0 if t >= contact else 5 if t >= land else 4 if any(a <= t < b for a, b in tracks)
                else 3 if t >= 2.9 else 2 if t >= 1.5 else 1)
        data.scalars.setdefault("/fc/debug/state", []).append((float(t), [mode]))
        precision = 0 if t >= contact or t < land else 3 if t >= land+3. else 2 if t >= land+.5 else 1
        data.scalars.setdefault("/fc/debug/precision", []).append((float(t), [precision]))
    data.scalars["/landing/target"] = [(t, [0., 0., 0., 1.]) for t in (land-.4, land-.2, land-.05)]
    data.scalars["/landing/quality"] = [(t, [7., float(team == "blue"), .2, .9]) for t in (land-.4, land-.2, land-.05)]
    return data


@pytest.mark.parametrize("shape, count", [("square", 5), ("triangle", 4)])
@pytest.mark.parametrize("team", ["red", "blue"])
def test_each_polygon_waypoint_is_physically_scored(shape, count, team):
    points = route(shape, team)
    report = evaluate(successful_evidence(team, points=points))
    assert report["passed"], report
    assert [row["index"] for row in report["metrics"]["waypoints"]] == list(range(count))


@pytest.mark.parametrize("fault", ["none", "stationary", "miss", "hold"])
def test_fly_through_requires_measured_speed_proximity_and_track(fault):
    data = successful_evidence(points=((4., 0.), (8., 0.), (4., 0.)))
    next(row for _, row in data.events if row["kind"] == "waypoint")["arrival"] = "pass"
    data.events = [(t, row) for t, row in data.events
                   if not (row.get("name") == "waypoint_hover" and row.get("index") == 0)]
    pad = np.array(resolve().hangar("red")) + [0., 0., .021]
    for i, (t, p, q) in enumerate(data.poses):
        if 8. <= t <= 10.:
            data.poses[i] = (t, (pad + [4. + (0. if fault == "stationary" else t - 9.),
                                          1. if fault == "miss" else 0., 1.]).tolist(), q)
    data.scalars["/fc/debug/state"] = [
        (t, [3 if fault == "hold" else 4]) if 8. <= t <= 10. else (t, row)
        for t, row in data.scalars["/fc/debug/state"]
    ]
    result = evaluate(data)
    assert result["passed"] == (fault == "none"), result


@pytest.mark.parametrize("fault", ["skipped", "identity", "wrong_goal", "wrong_return", "missing_intent", "short_hold", "no_track", "no_hold", "missed_middle", "count", "missing_count"])
def test_polygon_cannot_skip_or_miss_an_intermediate_waypoint(fault):
    data = successful_evidence(points=route("square"))
    if fault == "skipped":
        data.events = [(t, row) for t, row in data.events if not (row.get("name") == "waypoint" and row.get("index") == 1)]
    elif fault == "identity":
        next(row for _, row in data.events if row.get("name") == "waypoint_hover")["index"] = 3
    elif fault == "wrong_goal":
        leg = next(row for _, row in data.events if row.get("name") == "waypoint")
        leg["goal"] = [leg["goal"][0] + 1., *leg["goal"][1:]]
    elif fault == "wrong_return":
        next(row for _, row in data.events if row.get("name") == "return")["goal"] = [2., 0., 1.]
    elif fault == "missing_intent":
        data.events = [(t, row) for t, row in data.events if not (row["kind"] == "waypoint" and row["index"] == 1)]
    elif fault == "short_hold":
        data.events = [(t + 1. if row.get("name") == "waypoint_hover" and row.get("index") == 1 else t, row)
                       for t, row in data.events]
    elif fault in {"no_track", "no_hold"}:
        begin, end, replacement = (9., 12., 3) if fault == "no_track" else (12., 15., 4)
        data.scalars["/fc/debug/state"] = [(t, [replacement] if begin <= t < end else row)
                                          for t, row in data.scalars["/fc/debug/state"]]
    elif fault == "missed_middle":
        data.poses = [(t, [p[0]+1., p[1], p[2]] if 12. <= t <= 15. else p, q) for t, p, q in data.poses]
    elif fault == "count":
        data.events[0][1]["waypoint_count"] = 17
    elif fault == "missing_count":
        data.events[0][1].pop("waypoint_count")
    report = evaluate(data)
    assert not report["passed"], report
