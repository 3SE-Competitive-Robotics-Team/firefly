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
