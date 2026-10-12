"""外力恢复评分：脉冲执行、真实偏离与原目标恢复必须分别成立。"""
from dataclasses import dataclass

import numpy as np

from .hover_metrics import HoverLimits, attitude_health, covered_window
from .mission_metrics import read_evidence, settled_truth, startup_truth


@dataclass(frozen=True)
class DisturbanceLimits:
    """固定验收边界；随机种子、力（N）、时间（s）、位移（m）。"""
    seed: int = 20261012
    min_force_n: float = 1.0
    max_force_n: float = 1.2
    pulse_seconds: float = .5
    recovery_seconds: float = 10.
    min_displacement_m: float = .3
    max_excursion_m: float = 1.5
    max_height_error_m: float = .3
    rounds: int = 10


def disturbance_checks(data, stages, options, limits, force, hover_limits=HoverLimits()):
    """真值仅用于在线否决与离线评分，不输出控制量或调整目标。"""
    pulse_phase = stages["disturbance"]
    recovery = stages["sustained_hover"]
    if pulse_phase["status"] != "passed" or recovery["status"] != "passed":
        raise ValueError("disturbance/recovery observation did not complete")
    recovery_start, recovery_end = recovery["start_sim_s"], recovery["end_sim_s"]
    if (not np.isfinite([recovery_start, recovery_end]).all()
            or recovery_end - recovery_start < hover_limits.seconds - 1e-6):
        raise ValueError("recovery hover did not complete")
    rows = data["disturbance_force"]
    if rows.ndim != 2 or rows.shape[1] != 5 or len(rows) < 3 or not np.isfinite(rows).all():
        raise ValueError("missing force application evidence")
    applied, cleared = rows[:-1], rows[-1]
    dt = applied[:, 4]
    if (not np.all(dt > 0) or not np.allclose(applied[:, 1:4], force, rtol=0, atol=1e-6)
            or not np.all(cleared[1:] == 0)
            or not np.allclose(np.diff(rows[:, 0]), dt, rtol=0, atol=1e-6)):
        raise ValueError("force pulse interrupted, repeated, mismatched or not cleared")
    duration = float(dt.sum())
    start, end = float(rows[0, 0]), float(cleared[0])
    if (not pulse_phase["start_sim_s"] - .1 <= start <= pulse_phase["start_sim_s"] + .3
            or not limits.pulse_seconds - 1e-6 <= duration < limits.pulse_seconds + float(dt.max()) + 1e-6
            or not end <= recovery["start_sim_s"] <= end + limits.recovery_seconds + .3):
        raise ValueError("force or recovery timing outside fixed budget")

    goal = np.array([0., 0., options.altitude_m])
    tolerance = np.array([options.hover_xy_error_m] * 2 + [options.hover_height_error_m])
    baseline = stages["hover"]
    baseline_end = baseline["end_sim_s"]
    if (baseline["status"] != "passed" or not np.isfinite(baseline_end)
            or not baseline_end <= pulse_phase["start_sim_s"]):
        raise ValueError("missing or invalid pre-disturbance hover")
    before, before_metrics = settled_truth(data, baseline_end, options.hover_seconds,
                                           goal, tolerance, options.hover_speed_mps)

    disturbed = covered_window(data["gt"], start, recovery["start_sim_s"], .2)
    displacement = np.linalg.norm(disturbed[:, 1:3] - goal[:2], axis=1)
    peak = float(displacement.max())
    whole = covered_window(data["gt"], start, recovery["end_sim_s"], .2)
    excursion = float(np.linalg.norm(whole[:, 1:3] - goal[:2], axis=1).max())
    height_error = float(np.abs(whole[:, 3] - goal[2]).max())
    modes = covered_window(data["fc_state"], pulse_phase["start_sim_s"], recovery_end, .2)
    covered_window(data["gt"], recovery_start, recovery_end, .2)
    covered_window(data["gt_velocity"], recovery_start, recovery_end, .2)
    recovered = settled_truth(data, recovery_end, recovery_end - recovery_start,
                              goal, tolerance, options.hover_speed_mps)
    health = attitude_health(data, recovery_start, recovery_end, hover_limits)
    return {
        "sustained_hover_accuracy": recovered,
        "attitude_health": health,
        "hold_mode_continuity": (bool(np.all(modes[:, 1] == 3)), {"samples": len(modes)}),
        "pre_disturbance_hover": (before, before_metrics),
        "force_application": (True, {"duration_s": duration, "samples": len(applied),
                                     "impulse_world_ns": (applied[:, 1:4] * dt[:, None]).sum(axis=0).tolist()}),
        "physical_displacement": (peak >= limits.min_displacement_m, {"peak_horizontal_displacement_m": peak}),
        "disturbance_containment": (excursion <= limits.max_excursion_m and height_error <= limits.max_height_error_m,
                                    {"max_horizontal_excursion_m": excursion, "max_height_error_m": height_error}),
    }


def round_checks(data, stages, options, limits, force, index, hover_limits=HoverLimits()):
    """评分一轮同源证据；index 从 1 开始，返回检查结果而非控制指令。

    调用方必须在所有检查通过后才开始下一轮；缺失证据抛出异常，也必须否决继续。
    """
    if not isinstance(index, int) or not 1 <= index <= limits.rounds:
        raise ValueError("disturbance round index outside configured range")
    baseline = "hover" if index == 1 else f"recovery_{index - 1:02d}"
    return disturbance_checks(
        {**data, "disturbance_force": data[f"disturbance_force/{index:02d}"]},
        {"hover": stages[baseline], "disturbance": stages[f"disturbance_{index:02d}"],
         "sustained_hover": stages[f"recovery_{index:02d}"]},
        options, limits, force, hover_limits)


def score_disturbance(case, options, limits, forces, hover_limits=HoverLimits(), *,
                      evidence_reader=read_evidence, frame_transform=startup_truth):
    """逐轮离线评分；首个失败后全部阻塞，已有失败不得覆盖为通过。"""
    stages = case["stages"]
    already_failed = (case.get("status") == "failed"
                      or any(stage["status"] != "passed" for stage in stages.values()))
    try:
        forces = np.asarray(forces, dtype=float)
        if limits.rounds != 10 or forces.shape != (10, 3) or not np.isfinite(forces).all():
            raise ValueError("disturbance acceptance requires ten finite force vectors")
        data, case["evaluation_frame"] = frame_transform(evidence_reader(case["recording"]))
    except (OSError, TypeError, ValueError, KeyError, IndexError) as error:
        stages["disturbance_evidence"] = {"status": "failed", "reason": str(error)}
        case["status"] = "failed"
        return case

    blocked = False
    for index, force in enumerate(forces, 1):
        name = f"round_{index:02d}"
        previous = stages.get(name)
        if blocked:
            if previous is None or previous["status"] == "passed":
                stages[name] = {"status": "blocked", "reason": "preceding round did not pass"}
            continue
        assessment = stages.get(f"assess_{index:02d}")
        if assessment is not None and assessment["status"] in {"failed", "blocked"}:
            stages[name] = {"status": "failed", "reason": f"assess_{index:02d} did not pass"}
            blocked = True
            continue
        if previous is not None and previous["status"] != "passed":
            blocked = True
            continue
        try:
            checks = round_checks(data, stages, options, limits, force, index, hover_limits)
            passed = all(passed for passed, _ in checks.values())
            stages[name] = {
                "status": "passed" if passed else "failed",
                "checks": {key: {"status": "passed" if ok else "failed", "metrics": metrics}
                           for key, (ok, metrics) in checks.items()},
            }
            blocked = not passed
        except (TypeError, ValueError, KeyError, IndexError) as error:
            stages[name] = {"status": "failed", "reason": str(error)}
            blocked = True
    if already_failed or blocked or any(stage["status"] != "passed" for stage in stages.values()):
        case["status"] = "failed"
    return case
