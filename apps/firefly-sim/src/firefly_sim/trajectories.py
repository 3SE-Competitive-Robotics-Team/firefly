"""脚本化参考轨迹生成器（`--script` 模式，VIO bench 的激励源）。

层级：轨迹生成器（如 [`UTurnTrajectory`]）→ 具名实例（注册于
[`TRAJECTORIES`]）。新增曲线类型时实现 [`Trajectory.ref`]（位置/速度）
与 [`Trajectory.yaw`]（偏航角/角速度）并把实例加入注册表，
即被 bench 套件（`bench/bench_suite.py`）自动枚举。

不变量：所有实例的 pos/vel 必须是周期连续函数——仿真按连续时间求值，
周期边界（含潜在 t 回卷）不得产生参考跳变（跳变会导致 PD 发散 →
MuJoCo QACC NaN）。偏航角允许卷绕（多圈连续，如 -2π），PD 侧对误差
折叠到 [-π, π]，故卷绕不破坏连续性。
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

    def yaw(self, t: float) -> tuple[float, float]:
        """返回 t 时刻参考偏航角与角速度（rad、rad/s，机体 x 轴相对世界 x 轴）。

        缺省面朝 +x（悬停/无偏航机动）；掉头轨迹覆盖为切向航向。
        """
        ...


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
class UTurnTrajectory:
    """底线往返掉头：起点悬停 → 直线去程（机头朝前）→ 末端原地偏航掉头
    （位置不动，航向 0→-π 平滑旋转，无突变）→ 直线返程（机头朝前，
    非倒飞）→ 起点原地偏航转回（-π→-2π）→ 悬停至周期末。

    位置段 smoothstep 拼接（端点速度为 0），偏航段 smoothstep 旋转
    （端点角速度为 0），全时域 C² 且 ref(0)=ref(period)=start、
    yaw 模 2π 闭合（周期闭合不变量）。去/返程同线（y 恒定）。
    峰值速度 = 段长·1.875/段时长，峰值偏航率 = π·1.875/掉头时长，
    由参数保证 PD 可跟踪。
    """

    name: str
    #: 起点（m，西底线中点）
    start: tuple[float, float, float]
    #: 去程终点 x（m，东底线掉头点）
    x_far: float
    #: 巡航高度（m）
    cruise_z: float
    #: 剖面关键时刻（s）：起飞 / 到远端 / 掉头完 / 回到起点 /
    #: 转回完，此后悬停至 period
    t_go: float
    t_arrive: float
    t_turned: float
    t_back: float
    t_turned2: float
    period: float

    def _pos(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """位置/速度分段：悬停-直线-悬停转-直线-悬停转-悬停。"""
        s = np.array([float(self.start[0]), float(self.start[1]), self.cruise_z])
        b = np.array([self.x_far, float(self.start[1]), self.cruise_z])
        z = np.zeros(3)
        if t < self.t_go:
            return s.copy(), z
        if t < self.t_arrive:
            return self._straight(t, self.t_go, self.t_arrive, s, b)
        if t < self.t_turned:
            return b.copy(), z
        if t < self.t_back:
            return self._straight(t, self.t_turned, self.t_back, b, s)
        return s.copy(), z

    def _straight(
        self, t: float, t0: float, t1: float, a: np.ndarray, b: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """a→b smoothstep 直线段位置/速度。"""
        u = (t - t0) / (t1 - t0)
        pos = a + (b - a) * _smoothstep(u)
        vel = (b - a) * _smoothstep_dot(u, t1 - t0)
        return pos, vel

    def ref(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        return self._pos(t)

    def yaw(self, t: float) -> tuple[float, float]:
        if t < self.t_arrive:
            return 0.0, 0.0
        if t < self.t_turned:
            u = (t - self.t_arrive) / (self.t_turned - self.t_arrive)
            return -np.pi * _smoothstep(u), -np.pi * _smoothstep_dot(u, self.t_turned - self.t_arrive)
        if t < self.t_back:
            return -np.pi, 0.0
        if t < self.t_turned2:
            u = (t - self.t_back) / (self.t_turned2 - self.t_back)
            return -np.pi - np.pi * _smoothstep(u), -np.pi * _smoothstep_dot(
                u, self.t_turned2 - self.t_back
            )
        return -2.0 * np.pi, 0.0


def trajectory_yaw(traj: Trajectory, t: float) -> tuple[float, float]:
    """取轨迹偏航参考；未实现 `yaw` 的旧实例回落面朝 +x（悬停）。"""
    yaw_fn = getattr(traj, "yaw", None)
    if yaw_fn is None:
        return 0.0, 0.0
    return yaw_fn(t)


#: 具名实例注册表。峰值速度/偏航率控制在 PD 可跟踪范围（超过则物理发散：
#: QACC NaN）。去/返程 29m 同线 y=10。
TRAJECTORIES: dict[str, Trajectory] = {
    traj.name: traj
    for traj in (
        # 基线：1.5m/s 巡航（去 36s 峰 1.51；掉头 6s 峰 0.98rad/s；
        # 返 36s 峰 1.51；转回 6s），周期 92s
        UTurnTrajectory(
            name="outback_base",
            start=(1.0, 10.0, 1.0),
            x_far=30.0,
            cruise_z=1.2,
            t_go=3.0,
            t_arrive=39.0,
            t_turned=45.0,
            t_back=81.0,
            t_turned2=87.0,
            period=92.0,
        ),
        # 高速：1.8m/s 巡航（去 30s 峰 1.81；掉头 5s 峰 1.18rad/s；
        # 返 30s 峰 1.81；转回 5s），周期 78s
        UTurnTrajectory(
            name="outback_fast",
            start=(1.0, 10.0, 1.0),
            x_far=30.0,
            cruise_z=1.2,
            t_go=3.0,
            t_arrive=33.0,
            t_turned=38.0,
            t_back=68.0,
            t_turned2=73.0,
            period=78.0,
        ),
        # 低空：1.2m/s 巡航、z=0.7 贴地（地面大光流；去 45s 峰 1.21；
        # 掉头 6s 峰 0.98rad/s；返 45s 峰 1.21；转回 6s），周期 110s
        UTurnTrajectory(
            name="outback_low",
            start=(1.0, 10.0, 0.7),
            x_far=30.0,
            cruise_z=0.7,
            t_go=3.0,
            t_arrive=48.0,
            t_turned=54.0,
            t_back=99.0,
            t_turned2=105.0,
            period=110.0,
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
