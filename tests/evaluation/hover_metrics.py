"""持续悬停独立评分：真值只评分，窗口固定，缺失证据必须失败。"""
from dataclasses import dataclass

import numpy as np

from mission_metrics import landing_truth, read_evidence, settled_truth, startup_truth


@dataclass(frozen=True)
class HoverLimits:
    seconds: float = 20.0
    # 10Hz 视觉外援的测量年龄预算；只用于验收，不改变飞控的创新门。
    aid_age_s: float = 0.5
    tilt_error_deg: float = 2.0


def covered_window(rows, start, end, max_gap):
    """首尾各允许一个采样间隔；断流、短录制、非有限值不得参与通过判定。"""
    if rows.ndim != 2 or not np.isfinite(rows).all() or len(rows) < 2:
        raise ValueError("invalid hover evidence")
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError("nonmonotonic hover evidence")
    window = rows[(rows[:, 0] >= start - 1e-6) & (rows[:, 0] <= end + 1e-6)]
    if (len(window) < 2 or window[0, 0] > start + max_gap
            or window[-1, 0] < end - max_gap or np.max(np.diff(window[:, 0])) > max_gap + 1e-6):
        raise ValueError("incomplete hover evidence")
    return window


def body_gravity(quaternions):
    """Hamilton body→world 的 Rᵀ ez；不依赖局部航向规范。"""
    if np.any(np.abs(np.linalg.norm(quaternions, axis=1) - 1.) > 1e-3):
        raise ValueError("nonunit attitude evidence")
    x, y, z, w = quaternions.T
    return np.column_stack((2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)))


def attitude_health(data, start, end, limits):
    health = covered_window(data["imu_health"], start, end, 1.2)
    attitude = covered_window(data["imu_attitude"], start, end, .2)
    truth = covered_window(data["gt"], start, end, .2)
    # 仅比较覆盖范围内的同时刻姿态；不能把真值端点外推到缺测时刻。
    attitude = attitude[(attitude[:, 0] >= truth[0, 0]) & (attitude[:, 0] <= truth[-1, 0])]
    attitude = covered_window(attitude, start, end, .2)
    gravity = body_gravity(truth[:, 4:8])
    actual = np.column_stack([np.interp(attitude[:, 0], truth[:, 0], gravity[:, i]) for i in range(3)])
    actual /= np.linalg.norm(actual, axis=1)[:, None]
    estimated = body_gravity(attitude[:, 1:5])
    angles = np.rad2deg(np.arccos(np.clip(np.sum(actual * estimated, axis=1), -1., 1.)))
    max_age, max_tilt = float(health[:, 3].max()), float(angles.max())
    passed = (np.all(health[:, 1:3] == 1.) and np.all(health[:, 3] >= 0.)
              and max_age <= limits.aid_age_s and max_tilt <= limits.tilt_error_deg)
    return bool(passed), {"max_aid_age_s": max_age, "max_tilt_error_deg": max_tilt,
                          "health_samples": len(health), "attitude_samples": len(attitude)}


def score_hover(case, options, limits=HoverLimits(), *, phase_name="sustained_hover"):
    """完整 20 秒真值悬停、姿态链路、落地、物理安全和退出必须同时通过。"""
    stages = case["stages"]

    def check(name, action):

        try:
            passed, metrics = action()
            stages[name] = {"status": "passed" if passed else "failed", "metrics": metrics}
        except (ValueError, KeyError, IndexError) as error:
            stages[name] = {"status": "failed", "reason": str(error)}

    try:
        data, case["evaluation_frame"] = startup_truth(read_evidence(case["recording"]))
        phase = stages[phase_name]
        start, end = phase["start_sim_s"], phase["end_sim_s"]
        if phase["status"] != "passed" or end - start < limits.seconds - 1e-6:
            raise ValueError("sustained hover did not complete")
    except (OSError, ValueError, KeyError, IndexError) as error:
        stages["hover_evidence"] = {"status": "failed", "reason": str(error)}
        case["status"] = "failed"
        return case
    tolerance = np.array([options.hover_xy_error_m] * 2 + [options.hover_height_error_m])
    check("sustained_hover_accuracy", lambda: settled_truth(
        data, end, end - start, np.array([0., 0., options.altitude_m]), tolerance, options.hover_speed_mps))
    check("attitude_health", lambda: attitude_health(data, start, end, limits))
    check("landing_accuracy", lambda: landing_truth(data, stages["landing"], options))

    def physics():
        rows = data["physics"]
        covered_window(rows, data["gt"][0, 0], stages["landing"]["end_sim_s"], .3)
        live = np.asarray(case["last_physics_counters"][:4])
        if np.any(np.diff(rows[:, 1]) <= 0) or rows[-1, 1] < live[0]:
            raise ValueError("incomplete physics counters")
        return bool(np.all(rows[:, 3:5] == 0) and np.all(live[2:4] == 0)), {
            "unexpected_contact_steps": float(rows[:, 3].max()), "reset_attempts": float(rows[:, 4].max())}
    check("collision", physics)
    required = {"initialization", "takeoff", "hover", phase_name, "landing", "shutdown",
                "sustained_hover_accuracy", "attitude_health", "landing_accuracy", "collision"}
    case["status"] = "passed" if (required <= stages.keys() and not case.get("error")
        and case.get("remaining_pids") == [] and all(s["status"] == "passed" for s in stages.values())) else "failed"
    return case
