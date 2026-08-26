"""脚本化参考轨迹生成器（`--script` 模式，VIO bench 的激励源）。

层级：轨迹生成器（如 [`LissajousTrajectory`]）→ 具名实例（注册于
[`TRAJECTORIES`]）。新增曲线类型时实现 [`Trajectory.ref`] 并把实例加入
注册表即可被 bench 套件（`bench/bench_suite.py`）自动枚举。

不变量：所有实例的 pos/vel 必须是周期连续函数——仿真按连续时间求值，
周期边界（含潜在 t 回卷）不得产生参考跳变（跳变会导致 PD 发散 →
MuJoCo QACC NaN）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class Trajectory(Protocol):
    """参考轨迹协议：`ref(t)` 返回位置/速度（m、m/s），t 为仿真时间（s）。"""

    name: str
    #: 周期（s）；ref(t) == ref(t + period)
    period: float

    def ref(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """返回 t 时刻参考位置与速度。"""
        ...


@dataclass(frozen=True)
class LissajousTrajectory:
    """李萨如曲线生成器：三轴取基频整倍频正弦，中心 + 振幅参数化。

    `freq` 为整数倍频比，保证曲线在 `period` 内闭合（周期连续性不变量）；
    速度为解析导数。
    """

    name: str
    #: 曲线中心（m）
    center: tuple[float, float, float]
    #: 三轴振幅（m）
    amplitude: tuple[float, float, float]
    #: 三轴频率倍数（相对基频 1/period 的整数倍）
    freq: tuple[int, int, int]
    #: 闭合周期（s）
    period: float

    def ref(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        w = 2.0 * np.pi * np.asarray(self.freq, dtype=float) / self.period
        phase = w * t
        pos = np.asarray(self.center) + np.asarray(self.amplitude) * np.sin(phase)
        vel = np.asarray(self.amplitude) * w * np.cos(phase)
        return pos, vel


#: 具名实例注册表。振幅受两界约束：z 下限 > 0（避免触地）、峰值速度/
#: 加速度控制在 PD 可跟踪范围（超过则物理发散：QACC NaN）。
TRAJECTORIES: dict[str, Trajectory] = {
    traj.name: traj
    for traj in (
        # 原 _scripted_ref 硬编码曲线，bench 历史基线用
        LissajousTrajectory(
            name="lissajous_classic",
            center=(1.0, 4.0, 1.0),
            amplitude=(3.0, 1.0, 0.5),
            freq=(1, 2, 3),
            period=20.0,
        ),
        LissajousTrajectory(
            name="lissajous_wide",
            center=(1.0, 4.0, 1.5),
            amplitude=(3.5, 2.0, 0.6),
            freq=(1, 2, 1),
            period=26.0,
        ),
        LissajousTrajectory(
            name="lissajous_tight",
            center=(1.0, 4.0, 1.0),
            amplitude=(1.5, 1.5, 0.8),
            freq=(2, 3, 1),
            period=16.0,
        ),
        LissajousTrajectory(
            name="lissajous_vertical",
            center=(1.0, 4.0, 1.5),
            amplitude=(1.0, 1.0, 1.2),
            freq=(1, 2, 2),
            period=18.0,
        ),
    )
}


def get_trajectory(name: str) -> Trajectory:
    """按名取实例；未知名字报错并列出可用项。"""
    try:
        return TRAJECTORIES[name]
    except KeyError:
        available = ", ".join(sorted(TRAJECTORIES))
        raise ValueError(f"未知轨迹实例 {name!r}（可用：{available}）") from None
