"""初始化必须等待飞控实际解锁检查，不只等待 VIO 姿态。"""
import time

import numpy as np
import pytest

from firefly_evaluation.mission import Mission, Options


@pytest.mark.parametrize("arming", [None, "failed", "stale_sim", "stale_wall", "future"])
def test_initialization_waits_for_fresh_arming_check(tmp_path, arming):
    class StartupMission(Mission):
        pumps = 0

        def pump(self, guard=True):
            self.pumps += 1
            assert self.pumps <= 2, "initialization ignored the ready arming check"
            self.t = 2. + self.pumps * .1
            now = time.monotonic()
            self.latest["odom"] = (self.t, np.zeros(3), np.zeros(3), [0., 0., 0., 1.], True, now)
            self.counts["odom"] = 30
            self.zero_controls = 100
            self.attitude_health = (self.t, True, True, now)
            self.arming_health = (self.t, True, now)
            if self.pumps == 1:
                self.arming_health = {
                    None: None,
                    "failed": (self.t, False, now),
                    "stale_sim": (self.t - .4, True, now),
                    "stale_wall": (self.t, True, now - .6),
                    "future": (self.t + .1, True, now),
                }[arming]

    mission = StartupMission(tmp_path, "startup", Options(), sleep=lambda _: None)
    metrics = mission.initialized()
    assert mission.pumps == 2
    assert metrics["ready_sim_s"] == 2.2
