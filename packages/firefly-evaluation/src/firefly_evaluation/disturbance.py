"""水平外力任务编排与逐轮独立评分。"""
from functools import partial
import sys
import time

import numpy as np
from firefly_viz.messages import ScalarMessage, VizMessage

from .disturbance_metrics import round_checks, score_disturbance
from .disturbance_sim import ForcePulse, TOPIC
from .hover_metrics import score_hover
from .mission import Mission, MissionFailure
from .mission_io import service
from .mission_metrics import startup_truth


class DisturbanceMission(Mission):
    """同源 Viz 证据仅用于验收否决；不修改飞控目标、状态或估计器输入。"""
    evidence_aliases = {"fc/imu/health": "imu_health", "fc/imu/quaternion_xyzw": "imu_attitude",
                        "fc/debug/state": "fc_state", "fc/imu/source": "attitude_source"}
    required_evidence = {"gt", "gt_velocity", "imu_health", "imu_attitude", "fc_state", "attitude_source"}

    def start(self, name, command):
        if name == "sim":
            command = [sys.executable, "-m", "firefly_evaluation.disturbance_sim"]
        super().start(name, command)

    def connect(self):
        super().connect()
        self.force_publisher = service(self.node, TOPIC, ForcePulse).publisher_builder().create()
        self.evidence_subs = [service(self.node, topic, cls, 256, 8).subscriber_builder().buffer_size(256).create()
                              for topic, cls in [("Firefly/Viz", VizMessage), ("Firefly/VizScalars", ScalarMessage)]]
        self.evidence = {}

    def send_force(self, pulse: ForcePulse) -> None:
        """发送一条世界系力脉冲；顺序和有效性由接收端校验。"""
        self.force_publisher.loan_uninit().write_payload(pulse).send()

    def pump(self, guard=True):
        super().pump(guard)
        for subscriber in self.evidence_subs:
            while (sample := subscriber.receive()) is not None:
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


def run_disturbance(directory, options, limits, hover_limits, *, workspace=None,
                    mission_factory: type[DisturbanceMission] = DisturbanceMission,
                    hover_scorer=score_hover, disturbance_scorer=score_disturbance):
    """执行固定种子的外力序列；任何轮次失败均阻止下一次注入，返回评分与外力数组。"""
    options.validate()
    rng = np.random.default_rng(limits.seed)
    forces = []
    for _ in range(limits.rounds):
        angle = rng.uniform(-np.pi, np.pi)
        magnitude = rng.uniform(limits.min_force_n, limits.max_force_n)
        forces.append([magnitude * np.cos(angle), magnitude * np.sin(angle), 0.])
    forces = np.asarray(forces)
    mission = mission_factory(directory, "hover_disturbance", options, vio_only=True, workspace=workspace)

    def disturb(index, force):
        if index > 1 and mission.result["stages"][f"assess_{index - 1:02d}"]["status"] != "passed":
            raise MissionFailure("previous disturbance round has not passed")
        if mission.mode != 3 or not mission.hover_ok():
            raise MissionFailure("force injection requires fresh hover odometry and FC Hold")
        pulse = ForcePulse()
        pulse.timestamp, pulse.seconds, pulse.index = mission.t, limits.pulse_seconds, index
        pulse.force[:] = force
        mission.send_force(pulse)
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
    result = mission.run_phases(phases, ("render", "firefly-okvis-x", "fc"))
    hover_scorer(result, options, hover_limits, phase_name=f"recovery_{limits.rounds:02d}")
    disturbance_scorer(result, options, limits, forces, hover_limits)
    return result, forces
