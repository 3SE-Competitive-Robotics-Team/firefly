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


@dataclass(frozen=True)
class ForwardSweepTrajectory:
    """前飞扫掠：基线往返 + 空间相位横向/垂向调制。

    pos = start + p·delta + [0, Ay·sin(2π·ky·p), z_mod]，p 为往返标量
    （剖面同 [`StraightForwardTrajectory`]，悬停段 p 恒定→调制冻结，
    起终点 p=0→调制为 0）。z_mod：spiral=False 时 Az·sin(2π·kz·p)；
    True 时 Az·(1-cos(2π·ky·p))（y-z 平面绕前进轴螺旋）。vel 对 p
    解析求导 × dp/dt，全时域 C² 且周期闭合。

    横向加速度预算 Ay·(2π·ky)²·(dp/dt)² ≤ ~2m/s²（PD 跟踪界）——高频
    调制配慢腿（leg ≥ 12s）或小幅值，见各实例注释。
    """

    name: str
    #: 起点（m）
    start: tuple[float, float, float]
    #: 前飞端点偏移（m）
    delta: tuple[float, float, float]
    #: 横向调制幅值/周期数（空间相位）
    y_amp: float = 0.0
    y_cycles: int = 0
    #: 垂向调制幅值/周期数（spiral=True 时 y_cycles 共用，Az 为螺旋半径 z 分量）
    z_amp: float = 0.0
    z_cycles: int = 0
    #: True = y-z 螺旋（绕前进轴），False = y/z 独立正弦
    spiral: bool = False
    #: 剖面关键时刻（s）：起飞 / 到远端 / 返程出发 / 回到起点，此后悬停至 period
    t_go: float = 2.0
    t_arrive: float = 10.0
    t_return: float = 28.0
    t_home: float = 36.0
    period: float = 40.0

    def _p(self, t: float) -> float:
        """折返标量 p(t)∈[0,1]（同 StraightForward 剖面）。"""
        if t <= self.t_go or t >= self.t_home:
            return 0.0
        if t <= self.t_arrive:
            return _smoothstep((t - self.t_go) / (self.t_arrive - self.t_go))
        if t <= self.t_return:
            return 1.0
        return 1.0 - _smoothstep((t - self.t_return) / (self.t_home - self.t_return))

    def _dp(self, t: float) -> float:
        """p(t) 的时间导数。"""
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
        base = np.asarray(self.start, dtype=float) + p * np.asarray(self.delta, dtype=float)
        if self.y_cycles > 0:
            ph = 2.0 * np.pi * self.y_cycles * p
            base[1] += self.y_amp * np.sin(ph)
            dy = self.y_amp * 2.0 * np.pi * self.y_cycles * np.cos(ph)
        else:
            dy = 0.0
        if self.spiral and self.y_cycles > 0:
            ph = 2.0 * np.pi * self.y_cycles * p
            base[2] += self.z_amp * (1.0 - np.cos(ph))
            dz = self.z_amp * 2.0 * np.pi * self.y_cycles * np.sin(ph)
        elif self.z_cycles > 0:
            phz = 2.0 * np.pi * self.z_cycles * p
            base[2] += self.z_amp * np.sin(phz)
            dz = self.z_amp * 2.0 * np.pi * self.z_cycles * np.cos(phz)
        else:
            dz = 0.0
        vel = dp * np.asarray(self.delta, dtype=float) + dp * np.array([0.0, dy, dz])
        return base, vel


@dataclass(frozen=True)
class WaypointChainTrajectory:
    """航点链：相邻航点间 smoothstep 插值（同位置重复航点 = 悬停）。

    航点 (t, pos) 严格递增，首航点 t=0、末航点 t=period 且位置同为起点；
    段内端点速度为 0，全时域 C² 且周期闭合。急停/多圈等非对称剖面用。
    """

    name: str
    #: 航点时刻（s）
    times: tuple[float, ...]
    #: 航点位置（m）
    points: tuple[tuple[float, float, float], ...]
    period: float

    def ref(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        ts = self.times
        ps = [np.asarray(p, dtype=float) for p in self.points]
        if t <= ts[0]:
            return ps[0].copy(), np.zeros(3)
        for i in range(1, len(ts)):
            if t <= ts[i]:
                dt = ts[i] - ts[i - 1]
                u = (t - ts[i - 1]) / dt
                pos = ps[i - 1] + (ps[i] - ps[i - 1]) * _smoothstep(u)
                vel = (ps[i] - ps[i - 1]) * _smoothstep_dot(u, dt)
                return pos, vel
        return ps[-1].copy(), np.zeros(3)


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
        # 前飞鲁棒性边界 10 变体（横向锚在线是否对一切前飞机动成立）：
        # 包络：x≤4.2（箱区净距≥0.55m）、y∈[2.8,5.2]（侧柱净距≥0.95m）、
        # z≥0.6。偏航控制不存在（PD 纯位置+水平回正），sway/diag 为位置
        # 等效版（横向视点扫掠/斜向观测），真偏航需扩展 env 姿态控制。
        # 时长统一 period=40s，bench --duration 40 覆盖完整闭环（含返程，
        # 修正 straight_forward 用 34s 只测出程的截断弱点）。
        # 横向 S 形单周期：Ay·(2π)²·dp²≈1.9m/s²（8s 腿，PD 界内）。
        ForwardSweepTrajectory(
            name="ff_cross",
            start=(1.0, 4.0, 1.0),
            delta=(3.2, 0.0, 0.4),
            y_amp=0.9,
            y_cycles=1,
        ),
        # 绕前进轴螺旋 2 圈：半径 0.3，12s 慢腿（横向≈1.2m/s²）。
        ForwardSweepTrajectory(
            name="ff_spiral",
            start=(1.0, 4.0, 0.9),
            delta=(3.2, 0.0, 0.2),
            y_amp=0.3,
            y_cycles=2,
            z_amp=0.3,
            spiral=True,
            t_arrive=14.0,
            t_return=24.0,
        ),
        # 正弦偏航位置等效：小幅高频横向扫掠（视点方向快速变化，
        # 考前端 bearing rate），12s 慢腿（≈1.9m/s²）。
        ForwardSweepTrajectory(
            name="ff_sway",
            start=(1.0, 4.0, 1.0),
            delta=(3.2, 0.0, 0.4),
            y_amp=0.12,
            y_cycles=4,
            t_arrive=14.0,
            t_return=24.0,
        ),
        # 之字 3 折：A=0.25、12s 慢腿（≈2.2m/s²，预算上限）。
        ForwardSweepTrajectory(
            name="ff_zigzag",
            start=(1.0, 4.0, 1.0),
            delta=(3.2, 0.0, 0.4),
            y_amp=0.25,
            y_cycles=3,
            t_arrive=14.0,
            t_return=24.0,
        ),
        # 爬升：z 0.8→1.8（箱顶 1.7m 高度层切换，深度近距↔远距）。
        ForwardSweepTrajectory(
            name="ff_climb",
            start=(1.0, 4.0, 0.8),
            delta=(3.2, 0.0, 1.0),
        ),
        # 高速：4s 腿，峰值 1.5m/s、加速度≈1.2m/s²。
        ForwardSweepTrajectory(
            name="ff_fast",
            start=(1.0, 4.0, 1.0),
            delta=(3.2, 0.0, 0.4),
            t_arrive=6.0,
            t_return=26.0,
            t_home=30.0,
        ),
        # 低速贴地：z=0.6 恒高，10s 慢腿（峰值 0.6m/s，地面大光流）。
        ForwardSweepTrajectory(
            name="ff_low",
            start=(1.0, 4.0, 0.6),
            delta=(3.2, 0.0, 0.1),
            t_arrive=12.0,
            t_return=24.0,
            t_home=34.0,
        ),
        # 大偏航斜飞位置等效：斜向 20°（y 4→5.2，侧视箱面夹角变化）。
        ForwardSweepTrajectory(
            name="ff_diag",
            start=(1.0, 4.0, 1.0),
            delta=(3.2, 1.2, 0.4),
        ),
        # 急停重启：中途 p=0.5 悬停 5s（零速 IMU 零偏 + 前端丢失重捕）。
        WaypointChainTrajectory(
            name="ff_estop",
            times=(0.0, 2.0, 8.0, 13.0, 18.0, 28.0, 36.0, 40.0),
            points=(
                (1.0, 4.0, 1.0),
                (1.0, 4.0, 1.0),
                (2.6, 4.0, 1.2),
                (2.6, 4.0, 1.2),
                (4.2, 4.0, 1.4),
                (4.2, 4.0, 1.4),
                (1.0, 4.0, 1.0),
                (1.0, 4.0, 1.0),
            ),
            period=40.0,
        ),
        # 往返多圈：2 整圈（同几何重复，检验漂移累积 vs 闭环抵消）。
        WaypointChainTrajectory(
            name="ff_laps",
            times=(0.0, 2.0, 7.0, 9.0, 14.0, 16.0, 21.0, 23.0, 28.0, 40.0),
            points=(
                (1.0, 4.0, 1.0),
                (1.0, 4.0, 1.0),
                (4.2, 4.0, 1.4),
                (4.2, 4.0, 1.4),
                (1.0, 4.0, 1.0),
                (1.0, 4.0, 1.0),
                (4.2, 4.0, 1.4),
                (4.2, 4.0, 1.4),
                (1.0, 4.0, 1.0),
                (1.0, 4.0, 1.0),
            ),
            period=40.0,
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
