"""悬停门控反例：估计自洽、末尾恢复和缺测均不能掩盖真实漂移。"""

import numpy as np
import pytest

from firefly_evaluation import hover_metrics
from firefly_evaluation.hover_metrics import HoverLimits, attitude_health, score_hover

from firefly_evaluation.mission import Options


@pytest.fixture
def evidence(monkeypatch):
    times = np.arange(301) / 10
    truth = np.column_stack((times, np.zeros((len(times), 6)), np.ones(len(times))))
    truth[:, 3] = np.where((times >= 3.) & (times < 28.), 1., 0.)
    health_times = np.arange(31.)
    data = {"gt": truth, "gt_velocity": np.column_stack((times, np.zeros((len(times), 3)))),
            "imu_attitude": np.column_stack((times, np.zeros((len(times), 3)), np.ones(len(times)))),
            "imu_health": np.column_stack((health_times, np.ones((31, 2)), np.full(31, .2))),
            "physics": np.column_stack((times, times * 200 + 1, np.zeros((len(times), 3))))}
    stages = {name: {"status": "passed"} for name in ["initialization", "takeoff", "hover", "shutdown"]}
    stages["sustained_hover"] = {"status": "passed", "start_sim_s": 5., "end_sim_s": 25.}
    stages["landing"] = {"status": "passed", "end_sim_s": 30.,
                         "metrics": {"disarmed_sim_s": 28., "rest_start_sim_s": 29.}}
    case = {"recording": "unused", "stages": stages, "remaining_pids": [],
            "last_physics_counters": data["physics"][-1, 1:].tolist()}
    monkeypatch.setattr(hover_metrics, "read_evidence", lambda _: data)
    return case, data


def test_complete_hover_and_independent_yaw_gauge_pass(evidence):
    case, data = evidence
    data["imu_attitude"][:, 3:5] = [np.sin(.5), np.cos(.5)]
    assert score_hover(case, Options())["status"] == "passed"
    metrics = case["stages"]["attitude_health"]["metrics"]
    assert metrics["max_tilt_error_deg"] == pytest.approx(0.)
    assert metrics["max_aid_age_s"] == pytest.approx(.2)


@pytest.mark.parametrize("fault", ["drift_then_return", "speed_spike", "aid_starvation", "tilt",
                                   "attitude_gap", "health_missing", "short_window", "landing_offset",
                                   "collision", "reset", "physics_truncated", "remaining_process", "shutdown"])
def test_hover_gate_rejects_each_failure(evidence, fault):
    case, data = evidence
    if fault == "drift_then_return":
        data["gt"][80:120, 1] = .6
    elif fault == "speed_spike":
        data["gt_velocity"][100, 1] = .3
    elif fault == "aid_starvation":
        data["imu_health"][10, 3] = 16.
    elif fault == "tilt":
        data["imu_attitude"][100, 1:5] = [0., np.sin(np.deg2rad(5.) / 2), 0., np.cos(np.deg2rad(5.) / 2)]
    elif fault == "attitude_gap":
        data["imu_attitude"] = np.delete(data["imu_attitude"], np.arange(100, 150), axis=0)
    elif fault == "health_missing":
        del data["imu_health"]
    elif fault == "short_window":
        case["stages"]["sustained_hover"]["start_sim_s"] = 6.
    elif fault == "landing_offset":
        data["gt"][290:, 1] = .6
    elif fault in {"collision", "reset"}:
        data["physics"][100:, 3 if fault == "collision" else 4] = 1.
    elif fault == "physics_truncated":
        data["physics"] = data["physics"][:-1]
    elif fault == "remaining_process":
        case["remaining_pids"] = [123]
    else:
        case["stages"]["shutdown"]["status"] = "failed"
    assert score_hover(case, Options())["status"] == "failed"


def test_tilt_error_matches_analytic_pitch(evidence):
    _, data = evidence
    angle = np.deg2rad(1.5)
    data["imu_attitude"][:, 1:5] = [0., np.sin(angle / 2), 0., np.cos(angle / 2)]
    passed, metrics = attitude_health(data, 5., 25., HoverLimits())
    assert passed and metrics["max_tilt_error_deg"] == pytest.approx(1.5)


def test_touchdown_transient_is_separate_from_rest_and_collision(evidence):
    case, data = evidence
    # 固定等待期的 0.4m/s 触地过程不属于静止窗口；全过程碰撞记录仍必须通过。
    data["gt_velocity"][281, 3] = -.4
    assert score_hover(case, Options())["status"] == "passed"
    data["physics"][281:, 3] = 1.
    assert score_hover(case, Options())["status"] == "failed"


def test_speed_after_settling_still_fails(evidence):
    case, data = evidence
    data["gt_velocity"][295, 3] = -.4
    assert score_hover(case, Options())["status"] == "failed"
    assert case["stages"]["landing_accuracy"]["status"] == "failed"


@pytest.mark.parametrize("end", [29., 31.])
def test_rest_window_cannot_include_touchdown_or_wait_until_pass(evidence, end):
    case, _ = evidence
    case["stages"]["landing"]["end_sim_s"] = end
    assert score_hover(case, Options())["status"] == "failed"


def test_landing_observes_fixed_window_without_truth(tmp_path, monkeypatch):
    from firefly_evaluation.mission import Mission
    mission = Mission(tmp_path, "imu_hover", Options())
    mission.t, mission.mode = 10., 5
    monkeypatch.setattr(mission, "command", lambda *args: None)
    waits = []
    def wait(predicate, **kwargs):
        waits.append(kwargs)
        assert not predicate()
        if len(waits) == 1:
            mission.t, mission.mode = 13., 0
        else:
            mission.t = 14.5
            assert not predicate()
            mission.t = 15.
        assert predicate()
    monkeypatch.setattr(mission, "wait", wait)
    result = mission.landing()
    assert result["disarmed_sim_s"] == 13.
    assert result["rest_start_sim_s"] == 14.
    assert mission.t == 15. and waits[1]["sim"] == 17.
