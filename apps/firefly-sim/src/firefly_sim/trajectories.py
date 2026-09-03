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


def _smoothstep(u: float) -> float:
    """C² 平滑阶跃 u∈[0,1]→[0,1]（端点一/二阶导为 0），越界夹紧。"""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    return u * u * u * (6.0 * u * u - 15.0 * u + 10.0)


def _smoothstep_dot(u: float, dt: float) -> float:
    """`_smoothstep` 对 t 的导数（u=(t-t0)/dt → s'(u)/dt）；u 越界为 0。"""
    if u <= 0.0 or u >= 1.0:
        return 0.0
    return 30.0 * u * u * (1.0 - u) * (1.0 - u) / dt


@dataclass(frozen=True)
class StraightForwardTrajectory:
    """直线往返前飞：pos = start + p(t)·delta，p 为 C² 折返标量。

    p 剖面 = 起飞悬停 → smoothstep 升 → 远端悬停 → smoothstep 降 → 起点
    悬停至周期末。升/降段端点速度与加速度为 0，悬停段拼接连续，故 pos/vel
    全时域 C² 且 ref(0)=ref(period)=start（周期闭合不变量）。delta 允许
    斜向位移（z 分量 > 0 时起飞段同步缓升，无高度阶跃）。峰值速度 =
    |delta|·1.875/leg_时长，由参数保证 PD 可跟踪。
    """

    name: str
    #: 起点（m）
    start: tuple[float, float, float]
    #: 前飞位移（m）：只取 delta 正方向，返程沿原路退回
    delta: tuple[float, float, float]
    #: 剖面关键时刻（s）：起飞 / 到远端 / 返程出发 / 回到起点，此后悬停至 period
    t_go: float
    t_arrive: float
    t_return: float
    t_home: float
    period: float

    def _p(self, t: float) -> float:
        """折返标量 p(t)∈[0,1]：0 = 起点悬停，1 = 远端悬停。"""
        if t <= self.t_go or t >= self.t_home:
            return 0.0
        if t <= self.t_arrive:
            return _smoothstep((t - self.t_go) / (self.t_arrive - self.t_go))
        if t <= self.t_return:
            return 1.0
        return 1.0 - _smoothstep((t - self.t_return) / (self.t_home - self.t_return))

    def _dp(self, t: float) -> float:
        """p(t) 的时间导数（m/s 的标量系数）。"""
        if t <= self.t_go or t >= self.t_home:
            return 0.0
        if t <= self.t_arrive:
            return _smoothstep_dot(
                (t - self.t_go) / (self.t_arrive - self.t_go), self.t_arrive - self.t_go
            )
        if t <= self.t_return:
            return 0.0
        return -_smoothstep_dot(
            (t - self.t_return) / (self.t_home - self.t_return), self.t_home - self.t_return
        )

    def ref(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        p, dp = self._p(t), self._dp(t)
        d = np.asarray(self.delta, dtype=float)
        pos = np.asarray(self.start, dtype=float) + p * d
        vel = dp * d
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
        # 直线前飞（P14 实验）：验证 P13 预测——横向约束全程在线（x∈[1,4.2]
        # 巡航时箱前脸距 0.6~4.8m < 深度量程 6m）时定位不再漂。远端停在箱阵
        # 前 x=4.2（净距 0.55m），y 恒 4 避开三根中线立柱（x=9/12/16）。
        StraightForwardTrajectory(
            name="straight_forward",
            start=(1.0, 4.0, 1.0),
            delta=(3.2, 0.0, 0.4),
            t_go=3.0,
            t_arrive=11.0,
            t_return=39.0,
            t_home=47.0,
            period=60.0,
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
