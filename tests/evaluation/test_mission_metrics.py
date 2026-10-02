"""任务评分独立解析反例：禁止地图误差对齐、跨数据断流评分或放行高速着陆接触。"""
from pathlib import Path
import sys

import numpy as np
import pytest

from mission_metrics import position_error, yaw_error

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "system"))
from mission import Options
from mission_sim import permitted_pad_contact
from firefly_mujoco import drone_pad


def poses():
    time = np.arange(0., 3.01, .1)
    rows = np.zeros((len(time), 8))
    rows[:, 0] = time
    rows[:, 1] = time
    rows[:, 7] = 1.
    return rows


def test_map_tracking_error_preserves_translation_and_scale_error():
    gt = poses()
    estimate = gt.copy()
    estimate[:, 2] += .6
    result = position_error(gt, estimate, 0., 3.)
    assert result["rmse_m"] == pytest.approx(.6)
    assert result["max_m"] == pytest.approx(.6)
    assert result["coverage"] == pytest.approx(1.)


def test_map_yaw_detects_opposite_heading_and_wrap():
    gt, estimate = poses(), poses()
    estimate[:, 6:8] = [1., 0.]
    assert yaw_error(gt, estimate) == pytest.approx(180.)
    a, b = np.deg2rad(179.) / 2, np.deg2rad(-179.) / 2
    gt[:, 6:8] = [np.sin(a), np.cos(a)]
    estimate[:, 6:8] = [np.sin(b), np.cos(b)]
    assert yaw_error(gt, estimate) == pytest.approx(2.)


def test_missing_evidence_cannot_pass_by_extrapolating():
    gt = poses()
    with pytest.raises(ValueError, match="coverage"):
        position_error(gt, gt[:15], 0., 3.)
    missing = np.concatenate([gt[:5], gt[20:]])
    with pytest.raises(ValueError, match="gap"):
        position_error(gt, missing, 0., 3.)
    with pytest.raises(ValueError, match="gap"):
        position_error(missing, gt, 0., 3.)


def test_contact_exception_only_covers_low_speed_pad_touchdown():
    pad = np.asarray(drone_pad())
    normal = [0., 0., 1.]
    assert permitted_pad_contact(pad, np.zeros(3), normal)
    assert not permitted_pad_contact(pad, np.array([0., 0., -2.]), normal)
    assert not permitted_pad_contact(pad + [0., 0., 1.], np.zeros(3), normal)
    assert not permitted_pad_contact(pad + [1., 0., 0.], np.zeros(3), normal)
    assert not permitted_pad_contact(pad, np.zeros(3), [1., 0., 0.])


def test_invalid_thresholds_and_missing_mission_are_rejected():
    Options().validate()
    with pytest.raises(ValueError):
        Options(vio_ate_rmse_m=float("nan")).validate()
    with pytest.raises(ValueError):
        Options(waypoints=[]).validate()


def test_preflight_failure_returns_nonzero_and_keeps_report(tmp_path, monkeypatch):
    import importlib.util
    import json
    path = Path(__file__).resolve().parents[2] / "scripts/accept_rmuc.py"
    spec = importlib.util.spec_from_file_location("acceptance_cli", path)
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    monkeypatch.setattr(cli, "ROOT", tmp_path)
    monkeypatch.setattr(cli, "active_processes", lambda: [])
    assert cli.main(["--config", str(tmp_path / "missing.toml")]) == 1
    reports = list((tmp_path / "logs/acceptance").glob("*/report.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text())
    assert report["status"] == "failed" and report["cases"] == []
    assert report["error"] and reports[0].with_suffix(".html").is_file()


def test_repeated_failures_keep_four_distinct_recordings(tmp_path, monkeypatch):
    import importlib.util
    import json
    path = Path(__file__).resolve().parents[2] / "scripts/accept_rmuc.py"
    spec = importlib.util.spec_from_file_location("acceptance_cli_repeat", path)
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    monkeypatch.setattr(cli, "ROOT", tmp_path)
    monkeypatch.setattr(cli, "active_processes", lambda: [])
    monkeypatch.setattr(cli, "provenance", lambda: {})
    monkeypatch.setattr(cli, "score", lambda case, options: case)

    class FailedMission:
        def __init__(self, directory, name, options):
            self.recording = directory / f"{name}.rrd"
            self.name = name

        def run(self):
            self.recording.write_bytes(b"unit test evidence placeholder")
            return {"case": self.name, "status": "failed", "recording": str(self.recording),
                    "remaining_pids": [], "stages": {"tracking": {"status": "failed"}}}

    monkeypatch.setattr(cli, "Mission", FailedMission)
    config = tmp_path / "acceptance.toml"
    config.write_text("")
    assert cli.main(["--config", str(config), "--case", "nominal", "--repeat", "4"]) == 1
    report_path, = (tmp_path / "logs/acceptance").glob("*/report.json")
    report = json.loads(report_path.read_text())
    assert report["status"] == "failed"
    assert [case["attempt"] for case in report["cases"]] == [1, 2, 3, 4]
    assert len({case["recording"] for case in report["cases"]}) == 4
    html = report_path.with_suffix(".html").read_text()
    for attempt in range(1, 5):
        relative = f"attempt_{attempt:02d}/nominal.rrd"
        assert (report_path.parent / relative).is_file()
        assert f"href='{relative}'" in html


def test_roundtrip_distance_summary_does_not_override_failed_acceptance(monkeypatch):
    import mission_metrics
    truth = np.array([[0., -13., 0., 2.2, 0., 0., 0., 1.],
                      [10., -3., 0., 2.2, 0., 0., 0., 1.],
                      [20., -13., 0., 2.2, 0., 0., 0., 1.]])
    monkeypatch.setattr(mission_metrics, "read_evidence", lambda path: {
        "gt": truth, "loop_frontend": np.array([[20., 12., 3., 0., 0.]])})
    case = {"case": "nominal", "recording": "unused", "waypoint_arrivals": [{}, {}],
            "stages": {"tracking": {"status": "blocked"}}}
    mission_metrics.score(case, Options())
    summary = case["mission_summary"]
    assert summary["max_horizontal_distance_from_start_m"] == pytest.approx(10.)
    assert summary["horizontal_path_length_m"] == pytest.approx(20.)
    assert summary["last_horizontal_distance_from_start_m"] == pytest.approx(0.)
    assert summary["estimated_waypoints_completed"]
    assert not summary["all_waypoints_reached"]
    assert summary["online_keyframes"] == 12
    assert summary["maximum_loop_candidates"] == 3
    assert summary["loop_evidence"] == "no_accepted_constraint_observed"
    assert case["status"] == "failed"


def test_estimated_arrival_does_not_require_truth_and_false_arrival_fails_scoring():
    import time
    from mission import Mission
    from mission_metrics import settled_truth
    mission = Mission.__new__(Mission)
    mission.t = 3.
    goal = np.array([1., 0., 2.])
    mission.latest = {"corrected": (3., goal.copy(), np.zeros(3), None, True, time.monotonic())}
    assert mission.estimated_near("corrected", goal, .35, .3)
    gt = poses()
    gt[:, 1:4] = goal + [1., 0., 0.]
    velocity = np.column_stack([gt[:, 0], np.zeros((len(gt), 3))])
    passed, result = settled_truth({"gt": gt, "gt_velocity": velocity}, 3., 1., goal, .35, .3)
    assert not passed and result["max_position_error_m"] == pytest.approx(1.)
    gt[:, 1:4] = goal
    assert settled_truth({"gt": gt, "gt_velocity": velocity}, 3., 1., goal, .35, .3)[0]
    with pytest.raises(ValueError, match="incomplete"):
        settled_truth({"gt": gt[:-5], "gt_velocity": velocity}, 3., 1., goal, .35, .3)
    mission.t = 3.5
    assert not mission.estimated_near("corrected", goal, .35, .3)
