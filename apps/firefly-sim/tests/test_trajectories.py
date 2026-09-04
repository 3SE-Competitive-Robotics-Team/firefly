"""轨迹生成器不变量测试：注册表完整性、周期连续性、解析速度正确性、偏航连续性。"""

from __future__ import annotations

import numpy as np
import pytest

from firefly_sim.trajectories import TRAJECTORIES, get_trajectory, trajectory_yaw


def test_registry_has_outback_instances():
    names = sorted(TRAJECTORIES)
    assert names == ["outback_base", "outback_fast", "outback_low"]


def test_get_trajectory_unknown_name_lists_available():
    with pytest.raises(ValueError, match="outback_base"):
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


@pytest.mark.parametrize("name", sorted(TRAJECTORIES))
def test_yaw_continuity_mod_2pi(name: str):
    """偏航角在周期边界模 2π 一致（卷绕允许，PD 侧折叠误差）。"""
    traj = TRAJECTORIES[name]
    y0, _ = trajectory_yaw(traj, 0.0)
    y1, _ = trajectory_yaw(traj, traj.period)
    assert abs((y1 - y0 + np.pi) % (2.0 * np.pi) - np.pi) < 1e-9


@pytest.mark.parametrize("name", sorted(TRAJECTORIES))
def test_yaw_rate_matches_numerical_derivative(name: str):
    """偏航角速度 = 航向角数值导数（unwrap 后比较）。

    纯悬停段（速度为零且角速度为零）跳过：分段常数航向的数值导数在
    切换点为脉冲，解析值为 0，两者口径不同。原地掉头段（速度为零、
    角速度非零）正常校验。
    """
    traj = TRAJECTORIES[name]
    dt = 1e-4
    for t in np.linspace(0.5, traj.period - 0.5, 60):
        _, vel = traj.ref(float(t))
        y0, rate = trajectory_yaw(traj, float(t))
        if float(np.linalg.norm(vel)) < 0.05 and abs(rate) < 0.05:
            continue
        y1, _ = trajectory_yaw(traj, float(t) + dt)
        num = (y1 - y0) / dt
        assert abs(num - rate) < 2e-2, (name, t, num, rate)
