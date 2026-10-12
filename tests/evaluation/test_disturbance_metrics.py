"""外力验收评分的合成证据契约；不启动飞行进程。"""
from firefly_evaluation.mission import Options

import numpy as np
import pytest

from firefly_evaluation import hover_metrics
from firefly_evaluation.disturbance_metrics import DisturbanceLimits, disturbance_checks, round_checks, score_disturbance
from firefly_evaluation.hover_metrics import HoverLimits


@pytest.fixture
def evidence():
    times = np.arange(351) / 10.
    gt = np.zeros((len(times), 8))
    gt[:, 0], gt[:, 3], gt[:, 7] = times, 1., 1.
    gt[:, 1] = np.interp(times, [0., 4., 5., 6., 35.], [0., 0., .4, 0., 0.])
    velocity = np.column_stack((times, np.zeros((len(times), 3))))
    force = np.array([1.1, 0., 0.])
    pulse = np.array([[4. + i / 10., *force, .1] for i in range(5)] + [[4.5, 0., 0., 0., 0.]])
    data = {"gt": gt, "gt_velocity": velocity, "disturbance_force": pulse,
            "fc_state": np.column_stack((times, np.full(len(times), 3.))),
            "imu_attitude": np.column_stack((times, gt[:, 4:8])),
            "imu_health": np.column_stack((times, np.ones((len(times), 2)), np.full(len(times), .1)))}
    stages = {"hover": {"status": "passed", "end_sim_s": 4.},
              "disturbance": {"status": "passed", "start_sim_s": 4.},
              "sustained_hover": {"status": "passed", "start_sim_s": 14.5, "end_sim_s": 34.5}}
    options = Options(altitude_m=1., hover_xy_error_m=.2, hover_height_error_m=.15,
                              hover_seconds=3., hover_speed_mps=.2)
    return data, stages, options, DisturbanceLimits(), force


def test_force_impulse_and_real_departure(evidence):
    checks = disturbance_checks(*evidence)
    assert all(passed for passed, _ in checks.values())
    metrics = checks["force_application"][1]
    assert metrics["duration_s"] == pytest.approx(.5)
    assert metrics["impulse_world_ns"] == pytest.approx([.55, 0., 0.])
    assert checks["physical_displacement"][1]["peak_horizontal_displacement_m"] == pytest.approx(.4)


def test_motion_inside_target_tolerance_is_not_recovery(evidence):
    gt = evidence[0]["gt"]
    gt[:, 1] = np.interp(gt[:, 0], [0., 4., 5., 6., 35.], [-.19, -.19, .12, 0., 0.])
    assert not disturbance_checks(*evidence)["physical_displacement"][0]


@pytest.mark.parametrize("fault", ["missing", "gap", "uncleared", "wrong_force", "late_recovery", "truth_gap"])
def test_invalid_evidence_rejected(evidence, fault):
    data, stages, _, _, _ = evidence
    if fault == "missing":
        del data["disturbance_force"]
    elif fault == "gap":
        data["disturbance_force"] = np.delete(data["disturbance_force"], 2, axis=0)
    elif fault == "uncleared":
        data["disturbance_force"][-1, 1] = 1.1
    elif fault == "wrong_force":
        data["disturbance_force"][2, 1] = .5
    elif fault == "late_recovery":
        stages["sustained_hover"]["start_sim_s"] = 16.
    else:
        data["gt"] = data["gt"][(data["gt"][:, 0] < 7.) | (data["gt"][:, 0] > 8.)]
    with pytest.raises((ValueError, KeyError)):
        disturbance_checks(*evidence)


@pytest.mark.parametrize("fault", ["mode", "height", "excursion", "baseline"])
def test_safety_and_initial_hover_are_required(evidence, fault):
    data = evidence[0]
    if fault == "mode":
        data["fc_state"][50, 1] = 6.
        key = "hold_mode_continuity"
    elif fault == "height":
        data["gt"][50, 3] = 1.4
        key = "disturbance_containment"
    elif fault == "excursion":
        data["gt"][50, 1] = 1.6
        key = "disturbance_containment"
    else:
        data["gt"][20, 1] = .25
        key = "pre_disturbance_hover"
    assert not disturbance_checks(*evidence)[key][0]


@pytest.mark.parametrize("fault", ["position", "speed", "aid", "attitude", "short", "missing", "gap"])
def test_complete_recovery_is_required(evidence, fault):
    data, stages, *_ = evidence
    key = "sustained_hover_accuracy"
    if fault == "position":
        data["gt"][200, 1] = .25
    elif fault == "speed":
        data["gt_velocity"][200, 1] = .3
    elif fault == "aid":
        data["imu_health"][200, 3] = .6
        key = "attitude_health"
    elif fault == "attitude":
        data["imu_attitude"][200, 1:5] = [np.sin(np.deg2rad(2)), 0., 0., np.cos(np.deg2rad(2))]
        key = "attitude_health"
    elif fault == "short":
        stages["sustained_hover"]["end_sim_s"] = 34.
    elif fault == "missing":
        del data["imu_health"]
    else:
        data["gt_velocity"] = data["gt_velocity"][data["gt_velocity"][:, 0] < 30.]
    if fault in {"short", "missing", "gap"}:
        with pytest.raises((ValueError, KeyError)):
            disturbance_checks(*evidence)
    else:
        assert not disturbance_checks(*evidence)[key][0]


@pytest.fixture
def ten_rounds(evidence):
    _, _, options, limits, force = evidence
    times = np.arange(3091) / 10.
    gt = np.zeros((len(times), 8))
    gt[:, 0], gt[:, 3], gt[:, 7] = times, 1., 1.
    data = {"gt": gt, "gt_velocity": np.column_stack((times, np.zeros((len(times), 3)))),
            "fc_state": np.column_stack((times, np.full(len(times), 3.))),
            "imu_attitude": np.column_stack((times, gt[:, 4:8])),
            "imu_health": np.column_stack((times, np.ones((len(times), 2)), np.full(len(times), .1)))}
    stages = {"hover": {"status": "passed", "end_sim_s": 4.}}
    for index in range(1, 11):
        start = 4. + (index - 1) * 30.5
        gt[:, 1] += np.interp(times, [start, start + 1., start + 2.], [0., .4, 0.])
        data[f"disturbance_force/{index:02d}"] = np.array(
            [[start + i / 10., *force, .1] for i in range(5)] + [[start + .5, 0., 0., 0., 0.]])
        stages[f"disturbance_{index:02d}"] = {"status": "passed", "start_sim_s": start}
        stages[f"recovery_{index:02d}"] = {
            "status": "passed", "start_sim_s": start + 10.5, "end_sim_s": start + 30.5}
    return data, stages, options, limits, np.tile(force, (10, 1))


def score_synthetic(evidence, status="passed"):
    data, stages, options, limits, forces = evidence
    case = {"status": status, "stages": stages, "recording": "synthetic.rrd"}
    return score_disturbance(case, options, limits, forces, evidence_reader=lambda _: data,
                             frame_transform=lambda rows: (rows, {"synthetic": True}))


@pytest.mark.parametrize("with_assessments", [False, True])
def test_ten_rounds_pass(ten_rounds, with_assessments):
    data, stages, options, limits, forces = ten_rounds
    if with_assessments:
        for index in range(1, 11):
            stages[f"assess_{index:02d}"] = {"status": "passed"}
    assert limits.rounds == 10
    for index, force in enumerate(forces, 1):
        assert all(ok for ok, _ in round_checks(data, stages, options, limits, force, index).values())
    result = score_synthetic(ten_rounds)
    assert result["status"] == "passed"
    assert all(result["stages"][f"round_{i:02d}"]["status"] == "passed" for i in range(1, 11))


@pytest.mark.parametrize("fault", ["unrecovered", "missing_force", "missing_phase", "short", "blocked", "failed"])
def test_round_failure_blocks_all_following_rounds(ten_rounds, fault):
    data, stages, *_ = ten_rounds
    recovery = stages["recovery_04"]
    if fault == "unrecovered":
        mask = (data["gt"][:, 0] >= recovery["start_sim_s"]) & (data["gt"][:, 0] <= recovery["end_sim_s"])
        data["gt"][mask, 1] = .25
    elif fault == "missing_force":
        del data["disturbance_force/04"]
    elif fault == "missing_phase":
        del stages["recovery_04"]
    elif fault == "short":
        recovery["end_sim_s"] -= .5
    else:
        recovery["status"] = fault
    result = score_synthetic(ten_rounds)
    assert result["status"] == "failed"
    assert all(stages[f"round_{i:02d}"]["status"] == "passed" for i in range(1, 4))
    assert stages["round_04"]["status"] == "failed"
    assert all(stages[f"round_{i:02d}"]["status"] == "blocked" for i in range(5, 11))
    if fault in {"blocked", "failed"}:
        assert stages["recovery_04"]["status"] == fault


def test_round_baseline_uses_previous_recovery(ten_rounds):
    data, stages, options, limits, forces = ten_rounds
    data["gt"][data["gt"][:, 0] == 33., 1] = .25
    checks = round_checks(data, stages, options, limits, forces[1], 2)
    assert not checks["pre_disturbance_hover"][0]


@pytest.mark.parametrize("index", [0, 11, -1])
def test_round_index_is_one_based(ten_rounds, index):
    data, stages, options, limits, forces = ten_rounds
    with pytest.raises(ValueError):
        round_checks(data, stages, options, limits, forces[0], index)


def test_failed_case_is_not_promoted(ten_rounds):
    assert score_synthetic(ten_rounds, status="failed")["status"] == "failed"


@pytest.mark.parametrize("status", ["failed", "blocked"])
def test_existing_round_verdict_is_not_promoted(ten_rounds, status):
    stages = ten_rounds[1]
    stages["round_04"] = {"status": status, "reason": "online verdict"}
    score_synthetic(ten_rounds)
    assert stages["round_04"]["status"] == status
    assert stages["round_05"]["status"] == "blocked"


@pytest.mark.parametrize("status", ["failed", "blocked"])
@pytest.mark.parametrize("previous_status", [None, "passed", "failed", "blocked"])
def test_online_assessment_vetoes_offline_round(ten_rounds, status, previous_status):
    stages = ten_rounds[1]
    for index in range(1, 11):
        stages[f"assess_{index:02d}"] = {"status": "passed"}
    assessment = {"status": status, "reason": "online evidence did not pass"}
    stages["assess_04"] = assessment.copy()
    if previous_status is not None:
        stages["round_04"] = {"status": previous_status}
    for index in range(5, 11):
        stages[f"round_{index:02d}"] = {"status": "passed"}
    result = score_synthetic(ten_rounds)
    assert result["status"] == "failed"
    assert stages["assess_04"] == assessment
    assert all(stages[f"round_{i:02d}"]["status"] == "passed" for i in range(1, 4))
    assert stages["round_04"]["status"] == "failed"
    assert "assess_04" in stages["round_04"]["reason"]
    assert all(stages[f"round_{i:02d}"]["status"] == "blocked" for i in range(5, 11))


@pytest.mark.parametrize("count", [0, 9, 11])
def test_exactly_ten_forces_required(ten_rounds, count):
    data, stages, options, limits, _ = ten_rounds
    result = score_synthetic((data, stages, options, limits, np.zeros((count, 3))))
    assert result["status"] == "failed"
    assert stages["disturbance_evidence"]["status"] == "failed"


def test_custom_hover_limits_are_forwarded(ten_rounds):
    data, stages, options, limits, forces = ten_rounds
    checks = round_checks(data, stages, options, limits, forces[0], 1, HoverLimits(aid_age_s=.05))
    assert not checks["attitude_health"][0]


@pytest.mark.parametrize("phase_name", ["sustained_hover", "recovery_10"])
def test_hover_global_checks_support_named_phase(evidence, phase_name):
    data, stages, options, _, _ = evidence
    phase = stages.pop("sustained_hover")
    stages[phase_name] = phase
    for name in ("initialization", "takeoff", "shutdown"):
        stages[name] = {"status": "passed"}
    stages["landing"] = {"status": "passed", "end_sim_s": 35.}
    times = data["gt"][:, 0]
    steps = np.arange(1, len(times) + 1)
    data["physics"] = np.column_stack((times, steps, np.zeros((len(times), 3))))
    case = {"status": "passed", "stages": stages, "recording": "synthetic.rrd",
            "last_physics_counters": [int(steps[-1]), 0, 0, 0], "remaining_pids": []}
    result = hover_metrics.score_hover(
        case, options, phase_name=phase_name, evidence_reader=lambda _: data,
        frame_transform=lambda rows: (rows, {}), landing_check=lambda *_: (True, {}),
    )
    assert result["status"] == "passed"
    for name in ("sustained_hover_accuracy", "attitude_health", "landing_accuracy", "collision"):
        assert stages[name]["status"] == "passed"
