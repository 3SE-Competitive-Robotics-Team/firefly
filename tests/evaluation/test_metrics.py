"""局部里程计评测的坐标对齐与尺度约束。"""
import numpy as np
import pytest

from metrics import compute_metrics


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


@pytest.mark.parametrize("hz", [10, 20, 37])
def test_one_second_rpe_uses_time_not_frame_count(hz):
    t = np.linspace(0.0, 3.0, 3 * hz + 1)
    truth = np.column_stack((t, np.zeros_like(t), np.zeros_like(t)))
    estimate = truth.copy()
    estimate[:, 0] += 0.2 * t
    result = compute_metrics(t, truth, t, estimate, 3.0, alignment="none")
    assert result["rpe_rmse_1s"] == pytest.approx(0.2, abs=1e-12)


def test_irregular_samples_and_distinct_sensor_clocks():
    gt = np.array([0., .03, .1, .23, .34, .5, .71, .9, 1.1, 1.45, 1.6, 2., 2.3, 2.7, 3.])
    odom = np.linspace(0., 3., 61)
    truth = np.column_stack((gt, 0 * gt, 0 * gt))
    estimate = np.column_stack((1.2 * odom, 0 * odom, 0 * odom))
    result = compute_metrics(gt, truth, odom, estimate, 3., alignment="none")
    assert result["rpe_rmse_1s"] == pytest.approx(.2, abs=1e-12)


def test_short_recording_does_not_report_zero_rpe():
    t = np.linspace(0., .5, 20)
    p = np.column_stack((t, 0 * t, 0 * t))
    result = compute_metrics(t, p, t, p, .5)
    assert result["rpe_num_pairs_1s"] == 0
    assert result["rpe_rmse_1s"] is None


@pytest.mark.parametrize("fault", ["duplicate", "reverse", "nan", "shape"])
def test_invalid_measurements_are_rejected(fault):
    t, p = trajectory()
    if fault == "duplicate":
        t[5] = t[4]
    elif fault == "reverse":
        t = t[::-1]
    elif fault == "nan":
        p[5, 1] = np.nan
    else:
        p = p[:, :2]
    with pytest.raises(ValueError):
        compute_metrics(t, p, t, p, 3.)
