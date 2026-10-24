"""发布节拍网格测试：`advance_grid` 正常推进与追赶计数。"""

from __future__ import annotations

import pytest

from firefly_sim.main import advance_grid, decode_config


def test_steady_advance_skips_none():
    """正常节拍：恰好推进一格，跳过 0。"""
    nxt, skipped = advance_grid(1.00, 1.005, 0.01)
    assert abs(nxt - 1.01) < 1e-12
    assert skipped == 0


def test_hitch_counts_skipped_slots():
    """200ms 卡顿：跳过 20 格并如实计数（物理不可倒带，调用方告警）。"""
    nxt, skipped = advance_grid(10.00, 10.20, 0.01)
    assert skipped == 20
    assert abs(nxt - 10.21) < 1e-9


def test_exact_grid_boundary_advances_one():
    """恰落格点：只推进一格（`1e-12` 容差吸收浮点误差，不误报）。"""
    nxt, skipped = advance_grid(5.00, 5.00, 0.1)
    assert skipped == 0
    assert abs(nxt - 5.1) < 1e-12


def test_default_pose_rate_is_30hz_with_1khz_physics_and_imu():
    physics, imu, pose, _ = decode_config({"start": [0., 0., 0.]})
    assert physics == pytest.approx(0.001)
    assert imu == pytest.approx(0.001)
    assert pose == pytest.approx(1. / 30.)


def test_30hz_pose_grid_uses_actual_physics_timestamps_without_skipping():
    next_pose = 0.
    stamps = []
    for step in range(1001):
        stamp = step / 1000.
        if stamp + 1e-12 >= next_pose:
            stamps.append(step)
            next_pose, skipped = advance_grid(next_pose, stamp, 1. / 30.)
            assert skipped == 0
    assert len(stamps) == 31
    assert stamps[:4] == [0, 34, 67, 100]
    assert set(b - a for a, b in zip(stamps, stamps[1:])) == {33, 34}


@pytest.mark.parametrize("rates", [{"imu": 30.}, {"pose": 1001.}])
def test_invalid_sensor_rates_are_rejected(rates):
    with pytest.raises(ValueError):
        decode_config({"rates": rates, "start": [0., 0., 0.]})
