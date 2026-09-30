"""只读物理观测夹具：每步累计接触，固定 IMU 噪声种子，禁止静默物理重置。"""
import importlib
import os
import sys

import iceoryx2 as iox2
import numpy as np
from firefly_mujoco import DroneEnv, drone_pad

from mission_io import Recorder

CONTACT_POLICY = {"pad_radius_m": 0.5, "height_tolerance_m": 0.06,
                  "speed_limit_mps": 0.5, "min_vertical_normal_cosine": 0.9}


def permitted_pad_contact(position, velocity, normal):
    """停机坪附近低速竖直接触可用于起降；其他接触计为碰撞。"""
    origin = np.asarray(drone_pad())
    return (np.linalg.norm(position[:2] - origin[:2]) < CONTACT_POLICY["pad_radius_m"]
            and abs(position[2] - origin[2]) < CONTACT_POLICY["height_tolerance_m"]
            and np.linalg.norm(velocity) < CONTACT_POLICY["speed_limit_mps"]
            and abs(normal[2]) >= CONTACT_POLICY["min_vertical_normal_cosine"])


class ObservedEnv(DroneEnv):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self.recorder = Recorder(self.node)
        self.steps = self.contact_steps = self.collisions = self.resets = 0
        self.last_record = -1.
        self.initial_reset = False
        self.max_penetration = 0.

    def reset(self, *args, **kwargs):
        if self.initial_reset:
            self.resets += 1
            self.record()
            raise RuntimeError("acceptance rejects implicit physics reset")
        super().reset(*args, **kwargs)
        self.initial_reset = True

    def step(self):
        super().step()
        self.steps += 1
        finite = np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
        if not finite:
            self.resets += 1
            self.record()
            raise RuntimeError("non-finite physics state")
        contacts = [self.data.contact[i] for i in range(self.data.ncon)
                    if self._drone_id in self.model.geom_bodyid[self.data.contact[i].geom]]
        if contacts:
            self.contact_steps += 1
            self.max_penetration = max(self.max_penetration, max(-float(c.dist) for c in contacts))
            pos, vel, _, _ = self.state()
            if not all(permitted_pad_contact(pos, vel, c.frame[:3]) for c in contacts):
                self.collisions += 1
                if self.collisions == 1:
                    self.recorder.event(f"unexpected contact position={pos.tolist()} velocity={vel.tolist()}", self.time, True)
        if self.time - self.last_record >= 0.05 - 1e-9 or self.collisions:
            self.record()

    def record(self):
        self.recorder.scalars("acceptance/physics", self.time,
                              [self.steps, self.contact_steps, self.collisions, self.resets])
        self.recorder.scalars("acceptance/penetration", self.time, [self.max_penetration])
        self.last_record = self.time


def main():
    np.random.seed(int(os.environ["FIREFLY_MISSION_SEED"]))
    sim = importlib.import_module("firefly_sim.main")
    sim.DroneEnv = ObservedEnv
    sim._publish_plant_state = lambda *args: None
    sys.argv = ["firefly-sim", "--no-trace"]
    sim.main()


if __name__ == "__main__":
    main()
