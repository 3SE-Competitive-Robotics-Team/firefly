"""航段交接以目标身份和正常结束快照为准；到点仍由实测判定。"""
import time
from unittest.mock import create_autospec

import numpy as np
import pytest

from firefly_evaluation.mission import Mission, MissionFailure, Options
from firefly_evaluation.mission_io import Recorder


def ready(mission, goal=100.):
    mission.t = 10.
    mission.planner_status = (10., goal, False, time.monotonic())
    mission.reference_goal = goal
    mission.latest["reference"] = (10., np.zeros(3), np.zeros(3), None, True, time.monotonic())


@pytest.mark.parametrize("invalid", ["old_goal", "old_reference", "finished", "stale_status", "stale_reference"])
def test_new_leg_rejects_unrelated_or_stale_reference(tmp_path, invalid):
    mission = Mission(tmp_path, "nominal", Options())
    ready(mission)
    assert mission.leg_reference_ready(100.)
    if invalid == "old_goal":
        ready(mission, 99.)
    elif invalid == "old_reference":
        mission.reference_goal = 99.
    elif invalid == "finished":
        mission.planner_status = (10., 100., True, time.monotonic())
    elif invalid == "stale_status":
        mission.planner_status = (9., 100., False, time.monotonic())
    else:
        mission.latest["reference"] = (9., np.zeros(3), np.zeros(3), None, True, time.monotonic())
    assert not mission.leg_reference_ready(100.)


class TrackingMission(Mission):
    def __init__(self, directory, interruption=None):
        super().__init__(directory, "nominal", Options(), sleep=lambda _: None)
        self.commands = []
        self.recorder = create_autospec(Recorder, instance=True, spec_set=True)
        self.steps = 0
        self.active = False
        self.holding = False
        self.interruption = interruption

    def command(self, *args):
        self.commands.append(args)
        if args[:2] == ("planner", "goal"):
            ready(self, time.time())
            self.goal = np.asarray(args[2:])
            self.steps = 0
            self.active = self.holding = False
        elif args == ("fc", "track"):
            self.mode = 4
            self.active = True
        elif args == ("fc", "hold"):
            assert self.steps == 3, "Hold must follow normal completion, not early arrival"
            self.mode = 3
            self.holding = True

    def pump(self, guard=True):
        self.t += .1
        self.steps += int(self.active)
        assert self.steps < 30, "leg failed to complete"
        stamp = self.planner_status[1]
        self.planner_status = (self.t, stamp, self.steps >= 3, time.monotonic())
        self.latest["reference"] = (self.t, np.zeros(3), np.zeros(3), None, True, time.monotonic())
        # 接近目标不代表轨迹结束；轨迹结束也不替代实际到点。
        position = self.goal + (1. if 3 <= self.steps < 6 else 0.)
        self.latest["corrected"] = (self.t, position, np.zeros(3), None, True, time.monotonic())
        if self.steps == 3:
            if self.interruption == "hold":
                self.mode = 3
            elif self.interruption == "replaced":
                self.planner_status = (self.t, stamp + 1., True, time.monotonic())
            elif self.interruption == "stale":
                self.planner_status = (self.t - 1., stamp, True, time.monotonic())


def test_each_leg_tracks_new_reference_and_holds_on_normal_completion(tmp_path):
    mission = TrackingMission(tmp_path)
    assert len(mission.tracking()["arrivals"]) == 2
    assert mission.steps >= 16, "actual arrival must remain stable for one second"
    assert [cmd[:2] for cmd in mission.commands] == [
        ("planner", "goal"), ("fc", "track"), ("fc", "hold"),
        ("planner", "goal"), ("fc", "track"), ("fc", "hold"),
    ]


@pytest.mark.parametrize("interruption", ["hold", "replaced", "stale"])
def test_abnormal_interruption_is_not_normal_completion(tmp_path, interruption):
    mission = TrackingMission(tmp_path, interruption)
    with pytest.raises(MissionFailure):
        mission.tracking()
    assert ("fc", "hold") not in mission.commands
