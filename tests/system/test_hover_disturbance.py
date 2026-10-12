"""RMUC 起始点起飞、十轮随机水平外力、逐轮恢复判定与降落验收。

uv run pytest tests/system/test_hover_disturbance.py -q
需要图形会话、RMUC 资产和 release vio/fc/render/ffctl；证据只写 logs 下 RRD。
每轮评分通过才允许下一轮，失败立即中止；随机种子固定以复现扰动。
不重发 Hold，不给飞控发送真值或恢复目标。
"""
from dataclasses import asdict
from functools import partial
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
from firefly_viz.messages import VizMessage

from disturbance_metrics import DisturbanceLimits, round_checks, score_disturbance
from disturbance_sim import ForcePulse, TOPIC
from hover_metrics import HoverLimits, score_hover
from mission import Mission, MissionFailure, Options, ROOT
from mission_io import service
from mission_metrics import startup_truth


class DisturbanceMission(Mission):
    """同源 Viz 证据仅用于验收否决；不修改飞控目标、状态或估计器输入。"""
    evidence_aliases = {"fc/imu/health": "imu_health", "fc/imu/quaternion_xyzw": "imu_attitude",
                        "fc/debug/state": "fc_state"}
    required_evidence = {"gt", "gt_velocity", "imu_health", "imu_attitude", "fc_state"}

    def start(self, name, command):
        if name == "sim":
            command = [sys.executable, str(ROOT / "tests/system/disturbance_sim.py")]
        super().start(name, command)

    def connect(self):
        super().connect()
        self.force_publisher = service(self.node, TOPIC, ForcePulse).publisher_builder().create()
        self.evidence_sub = service(self.node, "Firefly/Viz", VizMessage, 256, 8).subscriber_builder().buffer_size(256).create()
        self.evidence = {}

    def pump(self, guard=True):
        super().pump(guard)
        while (sample := self.evidence_sub.receive()) is not None:
            msg = sample.payload().contents
            entity = bytes(msg.entity[:msg.entity_len]).decode()
            key = self.evidence_aliases.get(entity, entity.removeprefix("acceptance/"))
            if key not in self.required_evidence and not key.startswith("disturbance_force/"):
                continue
            if msg.kind == 1 and key == "gt":
                values = [*msg.xyz, *msg.quat_xyzw]
            elif msg.kind == 4 and key != "gt":
                values = list(msg.scalars[:msg.scalar_count])
            else:
                raise MissionFailure(f"unexpected live evidence kind: {entity}")
            row = np.array([msg.timestamp, *values], dtype=float)
            if not np.isfinite(row).all():
                raise MissionFailure(f"non-finite live evidence: {entity}")
            rows = self.evidence.setdefault(key, [])
            if rows and row[0] <= rows[-1][0]:
                if np.array_equal(row, rows[-1]):
                    continue
                raise MissionFailure(f"nonmonotonic/conflicting live evidence: {entity}")
            rows.append(row)

    def require_hover(self, reason):
        """否决时记录组成门控的各项数值，阈值仍由 Options 统一定义。"""
        if self.mode == 3 and self.hover_ok():
            return
        self.recorder.event(reason, self.t, error=True)
        state = self.latest.get("odom")
        self.recorder.scalars("acceptance/hover_failure/state", self.t,
                              [self.mode, state is not None, self.control_origin is not None])
        if state is not None:
            self.recorder.scalars("acceptance/hover_failure/freshness", self.t,
                                  [self.t - state[0], time.monotonic() - state[5], state[4]])
            if self.control_origin is not None:
                self.recorder.scalars("acceptance/hover_failure/error", self.t,
                                      [float(np.linalg.norm(state[1][:2] - self.control_origin[:2])),
                                       abs(state[1][2] - self.control_origin[2] - self.options.altitude_m),
                                       float(np.linalg.norm(state[2]))])
        raise MissionFailure(reason)

    def assess_round(self, options, limits, hover_limits, force, index):
        """独立真值评分只可中止验收；下一轮仍须通过新鲜里程计的悬停检查。"""
        end = self.result["stages"][f"recovery_{index:02d}"]["end_sim_s"]

        def evidence_ready():
            self.require_hover("hover lost while waiting for round evidence")
            return all(key in self.evidence and self.evidence[key][-1][0] >= end
                       for key in self.required_evidence)

        self.wait(evidence_ready, sim=2., wall=10.)
        data, _ = startup_truth({key: np.asarray(rows) for key, rows in self.evidence.items()})
        checks = round_checks(data, self.result["stages"], options, limits, force, index, hover_limits)
        failed = [name for name, (passed, _) in checks.items() if not passed]
        if failed:
            raise MissionFailure(f"disturbance round {index} failed: {', '.join(failed)}")
        return {name: metrics for name, (_, metrics) in checks.items()}


def test_hover_disturbance(flight_directory, flight_report):
    flight_directory.mkdir(parents=True)
    # 恢复半径必须小于要求的真实偏离量，不能把扰动全程都在容差内记作恢复。
    options = Options(hover_xy_error_m=.2)
    options.validate()
    limits, hover_limits = DisturbanceLimits(), HoverLimits()
    rng = np.random.default_rng(limits.seed)
    forces = []
    for _ in range(limits.rounds):
        angle = rng.uniform(-np.pi, np.pi)
        magnitude = rng.uniform(limits.min_force_n, limits.max_force_n)
        forces.append([magnitude * np.cos(angle), magnitude * np.sin(angle), 0.])
    forces = np.asarray(forces)
    mission = DisturbanceMission(flight_directory, "hover_disturbance", options, vio_only=True)

    def disturb(index, force):
        if index > 1 and mission.result["stages"][f"assess_{index - 1:02d}"]["status"] != "passed":
            raise MissionFailure("previous disturbance round has not passed")
        if mission.mode != 3 or not mission.hover_ok():
            raise MissionFailure("force injection requires fresh hover odometry and FC Hold")
        pulse = ForcePulse()
        pulse.timestamp, pulse.seconds, pulse.index = mission.t, limits.pulse_seconds, index
        pulse.force[:] = force
        mission.force_publisher.loan_uninit().write_payload(pulse).send()
        mission.recorder.event(f"force request round={index}/{limits.rounds} seed={limits.seed} world_N={force.tolist()} duration_s={limits.pulse_seconds}", mission.t)
        end = mission.t + limits.pulse_seconds + limits.recovery_seconds

        def observing():
            if mission.mode != 3:
                raise MissionFailure("FC left Hold during disturbance recovery")
            return mission.t >= end

        mission.wait(observing, sim=limits.pulse_seconds + limits.recovery_seconds + .3)
        return {"round": index, "recovery_budget_s": limits.recovery_seconds}

    def sustained_hover():
        end = mission.t + hover_limits.seconds

        def observing():
            mission.require_hover("fresh estimated hover not maintained after recovery deadline")
            return mission.t >= end

        mission.wait(observing, sim=hover_limits.seconds + .3)
        return {"seconds": hover_limits.seconds}

    phases = [("initialization", mission.initialized), ("takeoff", mission.takeoff), ("hover", mission.hover)]
    for index, force in enumerate(forces, 1):
        phases.extend([
            (f"disturbance_{index:02d}", partial(disturb, index, force)),
            (f"recovery_{index:02d}", sustained_hover),
            (f"assess_{index:02d}", partial(mission.assess_round, options, limits, hover_limits, force, index)),
        ])
    phases.append(("landing", mission.landing))
    result = mission.run_phases(phases, ("render", "vio", "fc"))
    score_hover(result, options, hover_limits, phase_name=f"recovery_{limits.rounds:02d}")
    score_disturbance(result, options, limits, forces, hover_limits)
    flight_report(result, options, disturbance_limits=asdict(limits),
                  hover_limits=asdict(hover_limits), forces_world_n=forces.tolist())


@pytest.mark.parametrize("failed_round", [None, 1, 4, 10])
def test_disturbance_sequence_is_fail_fast(tmp_path, monkeypatch, failed_round):
    """复用实际阶段执行器，替换进程与证据源，验证失败后不会再发力。"""
    import mission as mission_module

    events, reports = [], []

    class FakeMission(DisturbanceMission):
        def start(self, name, command):
            pass

        def connect(self):
            self.recorder = SimpleNamespace(event=lambda *args: None)

            def write(pulse):
                index = int(pulse.index)
                return SimpleNamespace(send=lambda: events.append(("force", index)))

            self.force_publisher = SimpleNamespace(loan_uninit=lambda: SimpleNamespace(write_payload=write))

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

    module = sys.modules[__name__]
    monkeypatch.setattr(module, "DisturbanceMission", FakeMission)
    monkeypatch.setattr(module, "score_hover", lambda *args, **kwargs: None)
    monkeypatch.setattr(module, "score_disturbance", lambda *args, **kwargs: None)
    monkeypatch.setattr(mission_module.time, "sleep", lambda _: None)
    test_hover_disturbance(tmp_path / "flight", lambda result, *args, **kwargs: reports.append(result))
    count = failed_round or 10
    expected = [(kind, index) for index in range(1, count + 1) for kind in ("force", "assess")]
    if failed_round is None:
        expected.append(("landing", None))
    expected.append(("shutdown", None))
    assert events == expected
    assert reports[0]["status"] == ("failed" if failed_round else "passed")
    if failed_round:
        assert reports[0]["stages"][f"assess_{failed_round:02d}"]["status"] == "failed"
        assert all(reports[0]["stages"][f"disturbance_{index:02d}"]["status"] == "blocked"
                   for index in range(failed_round + 1, 11))
