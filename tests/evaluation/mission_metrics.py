"""从验收 RRD 读回原始证据；地图误差不对齐，局部 VIO 只作固定尺度对齐。"""
import numpy as np
import pyarrow as pa
from rerun.chunk import RrdReader

from metrics import compute_metrics, interp_linear


def read_evidence(path):
    reader = RrdReader(path)
    rows = {}
    for store in reader.recordings():
        for chunk in reader.stream(store=store):
            entity = chunk.entity_path.lstrip("/")
            if not entity.startswith("acceptance/") or chunk.is_static:
                continue
            batch = chunk.to_record_batch()
            if "sim_time" not in batch.schema.names:
                continue
            times = batch.column("sim_time").cast(pa.int64()).to_pylist()
            if "Transform3D:translation" in batch.schema.names:
                positions = batch.column("Transform3D:translation").to_pylist()
                quats = batch.column("Transform3D:quaternion").to_pylist()
                values = [None if p is None or q is None else [*p[0], *q[0]] for p, q in zip(positions, quats)]
            elif "Scalars:scalars" in batch.schema.names:
                values = batch.column("Scalars:scalars").to_pylist()
            else:
                continue
            rows.setdefault(entity.removeprefix("acceptance/"), []).extend(
                (t / 1e9, *v) for t, v in zip(times, values) if t is not None and v is not None)
    result = {}
    for entity, values in rows.items():
        data = np.array(sorted(values), dtype=float)
        if not np.isfinite(data).all():
            raise ValueError(f"non-finite recorded evidence: {entity}")
        # 重复时间戳只允许内容完全相同；重复不增加覆盖率。
        unique = []
        for row in data:
            if unique and row[0] == unique[-1][0]:
                if not np.array_equal(row, unique[-1]):
                    raise ValueError(f"conflicting evidence at one timestamp: {entity}")
            else:
                unique.append(row)
        result[entity] = np.asarray(unique)
    return result


def position_error(truth, estimate, start, end):
    if len(truth) < 2 or len(estimate) < 2:
        raise ValueError("insufficient position evidence")
    times = truth[:, 0]
    mask = (times >= max(start, estimate[0, 0])) & (times <= min(end, estimate[-1, 0]))
    samples = truth[mask]
    if len(samples) < 10:
        raise ValueError("insufficient overlapping position evidence")
    if np.max(np.diff(samples[:, 0])) > 0.5:
        raise ValueError("truth evidence has a gap longer than 0.5s")
    intervals = np.diff(estimate[:, 0])
    window = (estimate[:-1, 0] <= end) & (estimate[1:, 0] >= start)
    if not window.any() or max(intervals[window]) > 0.5:
        raise ValueError("position evidence has a gap longer than 0.5s")
    span = min(end, times[-1]) - max(start, times[0])
    coverage = (samples[-1, 0] - samples[0, 0]) / max(span, 1e-9)
    if coverage < 0.9:
        raise ValueError(f"position evidence coverage too low: {coverage:.3f}")
    actual = interp_linear(samples[:, 0], estimate[:, 0], estimate[:, 1:4])
    norm = np.linalg.norm(actual - samples[:, 1:4], axis=1)
    return {"samples": len(samples), "start_sim_s": float(samples[0, 0]), "end_sim_s": float(samples[-1, 0]),
            "coverage": float(coverage), "rmse_m": float(np.sqrt(np.mean(norm**2))),
            "max_m": float(norm.max()), "p95_m": float(np.quantile(norm, .95))}


def yaw_error(truth, estimate):
    def yaw(rows):
        x, y, z, w = rows[:, 4:8].T
        return np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y*y + z*z)))
    times = estimate[:, 0]
    mask = (times >= truth[0, 0]) & (times <= truth[-1, 0])
    delta = yaw(estimate)[mask] - np.interp(times[mask], truth[:, 0], yaw(truth))
    if not len(delta):
        raise ValueError("no overlapping yaw evidence")
    wrapped = np.arctan2(np.sin(delta), np.cos(delta))
    return float(np.rad2deg(np.sqrt(np.mean(wrapped**2))))


def settled_truth(data, end, seconds, goal, tolerance, speed):
    """独立评分连续真值窗口；不允许短窗口、断流或端点外推掩盖到达误差。"""
    start = end - seconds
    samples = {}
    for key in ["gt", "gt_velocity"]:
        rows = data[key]
        window = rows[(rows[:, 0] >= start - 1e-6) & (rows[:, 0] <= end + 1e-6)]
        if (len(window) < 2 or window[0, 0] > start + .11
                or window[-1, 0] < end - .11 or np.diff(window[:, 0]).max() > .2):
            raise ValueError(f"incomplete settled truth window: {key}")
        samples[key] = window
    errors = np.abs(samples["gt"][:, 1:4] - goal)
    distances = np.linalg.norm(errors, axis=1)
    velocities = np.linalg.norm(samples["gt_velocity"][:, 1:4], axis=1)
    passed = (np.all(distances < tolerance) if np.isscalar(tolerance)
              else (np.all(np.linalg.norm(errors[:, :2], axis=1) < tolerance[0])
                    and np.all(errors[:, 2] < tolerance[2]))) and np.all(velocities < speed)
    return bool(passed), {"alignment": "none", "window_seconds": seconds,
                         "max_position_error_m": float(distances.max()),
                         "max_speed_mps": float(velocities.max())}


def score(case, options):
    data = read_evidence(case["recording"])
    stages = case["stages"]
    def check(name, action):
        try:
            passed, details = action()
            stages[name] = {"status": "passed" if passed else "failed", "metrics": details}
        except (ValueError, KeyError, IndexError) as error:
            stages[name] = {"status": "failed", "reason": str(error)}
    def collision():
        physics = data["physics"]
        if len(physics) < 10:
            raise ValueError("missing physics coverage")
        counts = physics[-1, 1:]
        live = case.get("last_physics_counters")
        if live is None or counts[0] < live[0]:
            raise ValueError("recording missing final observed physics counters")
        if np.any(np.diff(physics[:, 1]) <= 0):
            raise ValueError("physics counter not strictly increasing")
        return counts[2] == 0 and counts[3] == 0, {
            "physics_steps": int(counts[0]), "contact_steps": int(counts[1]),
            "unexpected_contact_steps": int(counts[2]), "reset_attempts": int(counts[3]),
            "observed_until_sim_s": float(physics[-1, 0]),
            "mission_landing_completed": stages.get("landing", {}).get("status") == "passed",
        }
    check("collision", collision)
    def vio():
        gt, odom = data["gt"], data["odom"]
        overlapping = gt[(gt[:, 0] >= odom[0, 0]) & (gt[:, 0] <= odom[-1, 0])]
        if len(overlapping) < 10 or len(odom) < 10:
            raise ValueError("insufficient VIO overlap")
        if max(np.diff(overlapping[:, 0]).max(), np.diff(odom[:, 0]).max()) > 0.5:
            raise ValueError("VIO scoring evidence has a gap longer than 0.5s")
        result = compute_metrics(gt[:, 0], gt[:, 1:4], odom[:, 0], odom[:, 1:4], float(gt[-1, 0] - gt[0, 0]))
        return (result["ate_rmse"] <= options.vio_ate_rmse_m
                and result["rpe_rmse_1s"] is not None and result["rpe_rmse_1s"] <= options.vio_rpe_rmse_m), result
    check("vio_accuracy", vio)
    for name, seconds, speed in [("hover", options.hover_seconds, options.hover_speed_mps),
                                  ("landing", 1., .1)]:
        phase = stages.get(name, {})
        if phase.get("status") == "passed":
            goal = data["gt"][0, 1:4].copy()
            if name == "hover":
                goal[2] += options.altitude_m
                tolerance = np.array([options.hover_xy_error_m] * 2 + [options.hover_height_error_m])
            else:
                tolerance = np.array([options.hover_xy_error_m] * 2 + [.08])
            check(name + "_accuracy", lambda: settled_truth(data, phase["end_sim_s"], seconds, goal, tolerance, speed))
    if stages.get("failure_hold", {}).get("status") == "passed":
        def hold_accuracy():
            truth = data["gt"]
            start = stages["reference_loss"]["start_sim_s"]
            if not truth[0, 0] <= start <= truth[-1, 0]:
                raise ValueError("missing fault origin truth")
            goal = interp_linear(np.array([start]), truth[:, 0], truth[:, 1:4])[0]
            return settled_truth(data, stages["failure_hold"]["end_sim_s"], 2., goal, .5, float("inf"))
        check("failure_hold_accuracy", hold_accuracy)
    if stages.get("terminal_hold", {}).get("status") == "passed":
        check("terminal_hold_accuracy", lambda: settled_truth(
            data, stages["terminal_hold"]["end_sim_s"], options.hover_seconds,
            np.asarray(options.waypoints[-1]), options.goal_tolerance_m, options.goal_speed_mps))
    if case["case"] != "estimator_loss":
        def corrected():
            gt, estimate = data["gt"], data["corrected"]
            result = position_error(gt, estimate, estimate[0, 0], estimate[-1, 0])
            result.update(alignment="none", yaw_rmse_deg=yaw_error(gt, estimate))
            return result["rmse_m"] <= options.map_ate_rmse_m and result["yaw_rmse_deg"] <= options.map_yaw_rmse_deg, result
        check("map_accuracy", corrected)
    if case["case"] == "nominal":
        def arrivals():
            results = []
            for arrival in case.get("waypoint_arrivals", []):
                passed, detail = settled_truth(data, arrival["arrived_sim_s"], 1.,
                    np.asarray(arrival["goal_map_m"]), options.goal_tolerance_m, options.goal_speed_mps)
                results.append({"passed": passed, **detail})
            return len(results) == len(options.waypoints) and all(r["passed"] for r in results), {"arrivals": results}
        check("waypoint_accuracy", arrivals)
        def tracking():
            phase = stages["tracking"]
            if "start_sim_s" not in phase:
                raise ValueError("tracking phase not executed")
            result = position_error(data["gt"], data["reference"], phase["start_sim_s"], phase["end_sim_s"])
            result["alignment"] = "none"
            return result["rmse_m"] <= options.tracking_rmse_m and result["max_m"] <= options.tracking_max_m, result
        if stages["tracking"]["status"] == "blocked":
            stages["tracking_error"] = {"status": "blocked", "reason": "tracking phase not executed"}
        else:
            check("tracking_error", tracking)
    truth = data.get("gt", np.empty((0, 8)))
    frontend = data.get("loop_frontend", np.empty((0, 5)))
    backend = data.get("loop_backend", np.empty((0, 4)))
    max_candidates = int(frontend[:, 2].max()) if len(frontend) else None
    accepted_loops = int(backend[:, 1].max()) if len(backend) else 0 if len(frontend) else None
    case["mission_summary"] = {
        "waypoint_arrivals": case.get("waypoint_arrivals", []),
        "waypoints_required": len(options.waypoints) if case["case"] == "nominal" else 0,
        "estimated_waypoints_completed": len(case.get("waypoint_arrivals", [])) == len(options.waypoints) if case["case"] == "nominal" else None,
        "all_waypoints_reached": stages.get("waypoint_accuracy", {}).get("status") == "passed" if case["case"] == "nominal" else None,
        "max_horizontal_distance_from_start_m": float(np.linalg.norm(truth[:, 1:3] - truth[0, 1:3], axis=1).max()) if len(truth) else None,
        "horizontal_path_length_m": float(np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1).sum()) if len(truth) else None,
        "last_horizontal_distance_from_start_m": float(np.linalg.norm(truth[-1, 1:3] - truth[0, 1:3])) if len(truth) else None,
        "online_keyframes": int(frontend[:, 1].max()) if len(frontend) else None,
        "maximum_loop_candidates": max_candidates,
        "accepted_online_loops": accepted_loops,
        "loop_evidence": "accepted" if accepted_loops else "no_accepted_constraint_observed" if len(frontend) else "missing",
        "measurement_source": "RRD; ground truth only for route scoring; no post-hoc map alignment",
    }
    case["evidence_entities"] = {entity: {"samples": len(rows), "first_sim_s": float(rows[0, 0]),
                                        "last_sim_s": float(rows[-1, 0])} for entity, rows in data.items()}
    case["status"] = "passed" if all(value["status"] == "passed" for value in stages.values()) else "failed"
    return case
