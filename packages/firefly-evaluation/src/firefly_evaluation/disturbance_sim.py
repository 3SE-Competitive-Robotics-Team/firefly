"""RMUC 外力验收夹具：世界系质心力脉冲，仅供测试，不修改状态或飞控目标。"""
import ctypes

import numpy as np
from firefly_sim.main import main as run_sim

from .mission_io import service
from .mission_sim import ObservedEnv

TOPIC = "Firefly/Test/HoverDisturbance"


class ForcePulse(ctypes.Structure):
    """顺序脉冲：轮次从 1 开始，发送时刻（仿真秒）、世界系力（N）、持续时间（秒）。"""
    _fields_ = [("timestamp", ctypes.c_double), ("force", ctypes.c_double * 3),
                ("seconds", ctypes.c_double), ("index", ctypes.c_uint64)]

    @staticmethod
    def type_name():
        return "FireflyTestHoverForcePulse"


class DisturbedEnv(ObservedEnv):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pulses = service(self.node, TOPIC, ForcePulse).subscriber_builder().create()
        self.force = np.zeros(3)
        self.remaining_steps = 0
        self.index = 0
        self.clear_pending = False

    def step(self):
        while (sample := self.pulses.receive()) is not None:
            pulse = sample.payload().contents
            force = np.array(pulse.force)
            if (pulse.index != self.index + 1 or self.remaining_steps or self.clear_pending
                    or not np.isfinite([pulse.timestamp, pulse.seconds, *force]).all()
                    or not -.1 <= self.time - pulse.timestamp <= .3
                    or not 0 < pulse.seconds <= 1. or not 0 < np.linalg.norm(force) <= 2.
                    or force[2] != 0.):
                raise RuntimeError("invalid, stale, overlapping or out-of-order test force pulse")
            self.index = int(pulse.index)
            self.force = force
            self.remaining_steps = int(np.ceil(pulse.seconds / self.model.opt.timestep - 1e-9))
            self.recorder.event(f"test force pulse {self.index} accepted: world-frame force at body COM", self.time)
        dt = float(self.model.opt.timestep)
        wrench = self.data.xfrc_applied[self._drone_id]
        wrench[:] = 0.
        if self.remaining_steps:
            # MuJoCo xfrc_applied 为世界系质心 wrench；前三维是力，后三维是力矩。
            wrench[:3] = self.force
            self.recorder.scalars(f"acceptance/disturbance_force/{self.index:02d}", self.time, [*self.force, dt])
            self.remaining_steps -= 1
            self.clear_pending = True
        elif self.clear_pending:
            self.recorder.scalars(f"acceptance/disturbance_force/{self.index:02d}", self.time, [0., 0., 0., 0.])
            self.clear_pending = False
        super().step()


def main(*, runner=run_sim):
    runner(["--no-trace"], env_factory=DisturbedEnv, publish_plant_state=False)


if __name__ == "__main__":
    main()
