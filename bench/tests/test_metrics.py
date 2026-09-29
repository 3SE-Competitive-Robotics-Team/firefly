"""局部里程计评测的坐标对齐与尺度约束。"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bench_vio import compute_metrics


def trajectory():
    t = np.linspace(0.0, 3.0, 31)
    p = np.column_stack((t, np.sin(t), 0.2 * t**2))
    return t, p


def test_local_origin_and_yaw_are_evaluation_only():
    t, truth = trajectory()
    yaw = 0.8
    r = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                  [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    estimate = (truth - np.array([20.0, -3.0, 2.0])) @ r
    original = estimate.copy()
    metrics = compute_metrics(t, truth, t, estimate, 3.0)
    assert metrics["ate_rmse"] < 1e-10
    assert metrics["rpe_rmse_1s"] < 1e-10
    assert metrics["raw_ate_rmse"] > 10.0
    assert metrics["alignment"] == "yaw_translation"
    np.testing.assert_array_equal(estimate, original)


def test_alignment_cannot_hide_scale_error():
    t, truth = trajectory()
    metrics = compute_metrics(t, truth, t, truth * 2.0, 3.0)
    assert metrics["ate_rmse"] > 0.5
    assert metrics["rpe_rmse_1s"] > 0.5


def test_absolute_evaluation_keeps_offset():
    t, truth = trajectory()
    metrics = compute_metrics(t, truth, t, truth + [2.0, 0.0, 0.0], 3.0, alignment="none")
    assert metrics["ate_rmse"] == pytest.approx(2.0)
