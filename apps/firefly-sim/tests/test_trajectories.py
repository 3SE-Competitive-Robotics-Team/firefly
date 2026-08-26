"""轨迹生成器不变量测试：注册表完整性、周期连续性、解析速度正确性。"""

from __future__ import annotations

import numpy as np
import pytest

from firefly_sim.trajectories import TRAJECTORIES, get_trajectory


def test_registry_has_four_lissajous_instances():
    lissajous = [n for n in TRAJECTORIES if n.startswith("lissajous_")]
    assert len(lissajous) == 4


def test_get_trajectory_unknown_name_lists_available():
    with pytest.raises(ValueError, match="lissajous_classic"):
        get_trajectory("nope")


@pytest.mark.parametrize("name", sorted(TRAJECTORIES))
def test_period_continuity(name: str):
    """ref(t) 在周期边界连续（pos/vel 首尾一致）。"""
    traj = TRAJECTORIES[name]
    p0, v0 = traj.ref(0.0)
    p1, v1 = traj.ref(traj.period)
    assert np.allclose(p0, p1, atol=1e-12)
    assert np.allclose(v0, v1, atol=1e-12)


@pytest.mark.parametrize("name", sorted(TRAJECTORIES))
def test_velocity_matches_numerical_derivative(name: str):
    traj = TRAJECTORIES[name]
    dt = 1e-5
    for t in (0.3, traj.period / 4.0, 1.7, traj.period * 0.9):
        pos_a, _ = traj.ref(t)
        pos_b, _ = traj.ref(t + dt)
        _, vel = traj.ref(t)
        assert np.allclose((pos_b - pos_a) / dt, vel, atol=1e-4)


@pytest.mark.parametrize("name", sorted(TRAJECTORIES))
def test_altitude_stays_above_ground(name: str):
    """z 参考全程 > 0（触地会触发物理失稳守卫，毒化下游 VIO）。"""
    traj = TRAJECTORIES[name]
    ts = np.linspace(0.0, traj.period, 2001)
    z_min = min(float(traj.ref(float(t))[0][2]) for t in ts)
    assert z_min > 0.05
