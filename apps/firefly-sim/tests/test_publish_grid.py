"""发布节拍网格测试：`advance_grid` 正常推进与追赶计数。"""

from __future__ import annotations

from firefly_sim.main import advance_grid


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
