"""外力任务的阶段执行与失败中止；不启动仿真进程。"""
from unittest.mock import create_autospec
from firefly_evaluation.mission_io import Recorder

import pytest

from firefly_evaluation import disturbance
from firefly_evaluation.disturbance import DisturbanceMission
from firefly_evaluation.disturbance_metrics import DisturbanceLimits
from firefly_evaluation.hover_metrics import HoverLimits
from firefly_evaluation.mission import MissionFailure, Options


@pytest.mark.parametrize("failed_round", [None, 1, 4, 10])
def test_disturbance_sequence_is_fail_fast(tmp_path, failed_round):
    """复用实际阶段执行器，替换进程与证据源，验证失败后不会再发力。"""
    events = []

    class FakeMission(DisturbanceMission):
        def start(self, name, command):
            pass

        def __init__(self, *args, **kwargs):
            super().__init__(*args, sleep=lambda _: None, **kwargs)

        def connect(self):
            self.recorder = create_autospec(Recorder, instance=True, spec_set=True)

        def send_force(self, pulse):
            events.append(("force", int(pulse.index)))

        def initialized(self):
            pass

        def takeoff(self):
            self.mode = 3

        def hover(self):
            pass

        def hover_ok(self):
            return True

        def wait(self, predicate, *, sim, **kwargs):
            self.t += sim
            assert predicate()

        def assess_round(self, options, limits, hover_limits, force, index):
            events.append(("assess", index))
            if index == failed_round:
                raise MissionFailure("synthetic recovery failure")

        def landing(self):
            events.append(("landing", None))

        def shutdown(self):
            events.append(("shutdown", None))
            self.result["stages"]["shutdown"] = {"status": "passed"}
            self.result["remaining_pids"] = []

        def run_phases(self, phases, applications):
            return self._run_phases(phases, applications)

    result, _ = disturbance.run_disturbance(
        tmp_path / "flight", Options(hover_xy_error_m=.2), DisturbanceLimits(), HoverLimits(),
        mission_factory=FakeMission, hover_scorer=lambda *args, **kwargs: None,
        disturbance_scorer=lambda *args, **kwargs: None,
    )
    count = failed_round or 10
    expected = [(kind, index) for index in range(1, count + 1) for kind in ("force", "assess")]
    if failed_round is None:
        expected.append(("landing", None))
    expected.append(("shutdown", None))
    assert events == expected
    assert result["status"] == ("failed" if failed_round else "passed")
    if failed_round:
        assert result["stages"][f"assess_{failed_round:02d}"]["status"] == "failed"
        assert all(result["stages"][f"disturbance_{index:02d}"]["status"] == "blocked"
                   for index in range(failed_round + 1, 11))
