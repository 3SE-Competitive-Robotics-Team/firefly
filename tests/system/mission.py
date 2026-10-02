"""真实进程任务验收；预设命令，真值只评分，失败不调整门槛。"""
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
import logging
import os
import signal
import subprocess
import sys
import threading
import time

import iceoryx2 as iox2
import numpy as np
from firefly_mujoco import ControlMessage, OdomMessage
from firefly_viz.messages import VizMessage

from mission_io import Recorder, ReferenceMessage, pose_values, service

ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Options:
    seed: int = 20260930
    altitude_m: float = 1.0
    hover_seconds: float = 3.0
    hover_height_error_m: float = 0.15
    hover_xy_error_m: float = 0.5
    hover_speed_mps: float = 0.2
    goal_tolerance_m: float = 0.35
    goal_speed_mps: float = 0.3
    tracking_rmse_m: float = 0.35
    tracking_max_m: float = 0.8
    vio_ate_rmse_m: float = 0.2
    vio_rpe_rmse_m: float = 0.15
    map_ate_rmse_m: float = 0.25
    map_yaw_rmse_deg: float = 15.0
    reference_loss_wall_s: float = 2.5
    estimator_fallback_wall_s: float = 0.8
    stage_wall_s: float = 90.0
    waypoint_sim_s: float = 25.0
    # 地图系米；飞往台阶上空，再返回起飞点上方。
    waypoints: tuple = ((-11.0, 0.0, 2.0), (-13.0, 0.0, 1.405))

    def validate(self):
        for key, value in asdict(self).items():
            if key not in {"seed", "waypoints"} and (not np.isfinite(value) or value <= 0):
                raise ValueError(f"invalid acceptance threshold: {key}")
        points = np.asarray(self.waypoints)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError("mission requires at least two finite map waypoints")


class MissionFailure(RuntimeError):
    pass


class Mission:
    def __init__(self, directory, case, options):
        self.options, self.case = options, case
        self.recording = directory / f"{case}.rrd"
        self.result = {"case": case, "status": "running", "recording": str(self.recording), "stages": {}}
        self.processes = {}
        self.expected_stops = set()
        self.latest = {}
        self.last_logged = {}
        self.counts = {}
        self.t = 0.
        self.mode = None
        self.physics = None
        self.physics_time = None
        self.zero_controls = 0
        self.nonzero_controls = 0
        self.last_motors = [0.] * 4
        self.sequence_errors = 0
        self.recorder = None
        self.origin = None
        self.visual_updates = 0

    def start(self, name, command):
        env = {**os.environ, "RUST_LOG": "info", "FIREFLY_MISSION_SEED": str(self.options.seed), "PYTHONDONTWRITEBYTECODE": "1"}
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        tail = deque(maxlen=12)
        def drain():
            for line in process.stdout:
                tail.append(line.rstrip())
        threading.Thread(target=drain, daemon=True).start()
        self.processes[name] = process, tail
        if self.recorder:
            self.recorder.event(f"process start name={name} pid={process.pid} argv={command}", self.t)

    def stop(self, name):
        process, _ = self.processes[name]
        self.expected_stops.add(name)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
        return process.returncode

    def command(self, *args):
        self.recorder.event(f"command ffctl {' '.join(map(str, args))}", self.t)
        completed = subprocess.run([str(ROOT / "target/release/ffctl"), *map(str, args)], cwd=ROOT,
                                   capture_output=True, text=True, timeout=5)
        if completed.returncode:
            raise MissionFailure(f"ffctl failed: {completed.stderr[-500:]}")

    def connect(self):
        iox2.set_log_level(iox2.LogLevel.Error)
        self.node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self.recorder = Recorder(self.node)
        self.subs = {}
        for name, topic, cls, capacity, publishers in [
            ("gt", "GroundTruth", OdomMessage, None, None),
            ("odom", "Odometry", OdomMessage, None, None),
            ("corrected", "CorrectedOdometry", OdomMessage, None, None),
            ("reference", "Reference", ReferenceMessage, None, None),
            ("control", "Control", ControlMessage, 32, None),
            ("viz", "Viz", VizMessage, 256, 8),
        ]:
            builder = service(self.node, "Firefly/" + topic, cls, capacity, publishers).subscriber_builder()
            self.subs[name] = builder.buffer_size(capacity or 2).create()

    def pump(self, guard=True):
        for name, (process, tail) in self.processes.items():
            if name not in self.expected_stops and process.poll() is not None:
                diagnostic = " | ".join(tail)
                self.recorder.event(f"unexpected process exit {name} code={process.returncode} {diagnostic}", self.t, True)
                raise MissionFailure(f"process {name} exited {process.returncode}: {diagnostic[-400:]}")
        for name, sub in self.subs.items():
            while (sample := sub.receive()) is not None:
                msg = sample.payload().contents
                if name == "viz":
                    entity = bytes(msg.entity[:msg.entity_len]).decode(errors="replace")
                    if entity == "fc/debug/state":
                        self.mode = int(msg.scalars[0])
                    elif entity == "corr/debug/gate" and msg.scalar_count == 4 and msg.scalars[3] == 1:
                        self.visual_updates += 1
                        self.recorder.scalars("acceptance/visual_update", msg.timestamp, list(msg.scalars[:4]))
                    elif entity == "acceptance/physics":
                        self.physics = list(msg.scalars)
                        self.physics_time = msg.timestamp
                    continue
                if name == "control":
                    values = list(msg.thrust)
                    self.last_motors = values
                    if not np.isfinite(values).all() or min(values) < 0:
                        raise MissionFailure("invalid motor output")
                    self.zero_controls = self.zero_controls + 1 if max(values) == 0 else 0
                    self.nonzero_controls += int(max(values) > 0)
                    stamp = msg.state_time
                    if stamp - self.last_logged.get(name, -1.) >= 0.05 - 1e-9:
                        self.recorder.scalars("acceptance/motors", stamp, values)
                        self.last_logged[name] = stamp
                    continue
                stamp = float(msg.timestamp)
                pos, vel, quat = pose_values(msg)
                if not np.isfinite([stamp, *pos, *vel, *quat]).all():
                    raise MissionFailure(f"non-finite {name} sample")
                previous = self.latest.get(name)
                if previous and stamp < previous[0]:
                    self.sequence_errors += 1
                    raise MissionFailure(f"{name} timestamp regressed")
                if previous and stamp == previous[0]:
                    continue
                initialized = getattr(msg, "is_initialized", True)
                self.latest[name] = (stamp, np.array(pos), np.array(vel), quat, initialized, time.monotonic())
                self.counts[name] = self.counts.get(name, 0) + 1
                if name == "gt":
                    self.t = stamp
                    if self.origin is None:
                        self.origin = np.array(pos)
                if stamp - self.last_logged.get(name, -1.) >= 0.099 - 1e-9:
                    self.recorder.pose("acceptance/" + name, stamp, pos, quat)
                    self.recorder.scalars("acceptance/" + name + "_velocity", stamp, vel)
                    self.last_logged[name] = stamp
        if guard and self.physics and (self.physics[2] > 0 or self.physics[3] > 0):
            raise MissionFailure(f"physics collision/reset counters={self.physics}")
        if guard and self.physics_time is not None and self.t - self.physics_time > 0.3:
            raise MissionFailure("physics evidence stream stale")
        if guard and "gt" in self.latest:
            pos = self.latest["gt"][1]
            if not (-14.5 < pos[0] < 14.5 and -7.5 < pos[1] < 7.5 and 0.2 < pos[2] < 3.0):
                raise MissionFailure(f"mission containment exceeded: {pos.tolist()}")

    def wait(self, predicate, *, stable=0., wall=None, sim=None, guard=True):
        deadline = time.monotonic() + (wall or self.options.stage_wall_s)
        start, held, previous = self.t, None, None
        while time.monotonic() < deadline:
            self.pump(guard)
            if sim is not None and self.t - start > sim:
                raise MissionFailure(f"measurement-time deadline exceeded ({sim}s)")
            if previous is not None and self.t - previous > 0.3:
                held = None
            if predicate():
                held = self.t if held is None else held
                if self.t - held >= stable:
                    return
            else:
                held = None
            previous = self.t
            time.sleep(0.01)
        raise MissionFailure("wall-clock deadline exceeded")

    def phase(self, name, action):
        stage = {"status": "running", "start_sim_s": self.t}
        self.result["stages"][name] = stage
        start = time.monotonic()
        self.recorder.event(f"phase begin {name}", self.t)
        logging.getLogger("acceptance").info("%s/%s begin t=%.3f", self.case, name, self.t)
        try:
            metrics = action()
            stage.update(status="passed", metrics=metrics or {})
        except BaseException as error:
            stage.update(status="failed", reason=str(error) or type(error).__name__)
            raise
        finally:
            stage.update(end_sim_s=self.t, wall_seconds=time.monotonic() - start)
            self.recorder.event(f"phase end {name} status={stage['status']} {stage.get('reason', '')}", self.t,
                                stage["status"] == "failed")
            logging.getLogger("acceptance").info("%s/%s %s t=%.3f %s", self.case, name, stage["status"], self.t, stage.get("reason", ""))

    def initialized(self):
        def ready():
            odom = self.latest.get("odom")
            return (odom is not None and odom[4] and self.counts["odom"] >= 20
                    and self.zero_controls >= 100 and self.physics is not None)
        self.wait(ready)
        odom = self.latest["odom"]
        if odom[0] < 1.0 or np.linalg.norm(odom[1]) > 0.1 or self.nonzero_controls:
            raise MissionFailure("initialization window, local origin or disarmed motor contract failed")
        return {"ready_sim_s": odom[0], "local_origin_m": odom[1].tolist(), "zero_controls": self.zero_controls}

    def takeoff(self):
        self.command("fc", "arm")
        self.wait(lambda: self.mode == 1, wall=5.)
        self.command("fc", "takeoff", self.options.altitude_m)
        self.wait(lambda: self.mode == 3 and self.hover_ok(), sim=20.)
        return {"target_relative_height_m": self.options.altitude_m}

    def hover_ok(self):
        _, pos, vel, *_ = self.latest["gt"]
        return (abs(pos[2] - self.origin[2] - self.options.altitude_m) < self.options.hover_height_error_m
                and np.linalg.norm(pos[:2] - self.origin[:2]) < self.options.hover_xy_error_m
                and np.linalg.norm(vel) < self.options.hover_speed_mps)

    def hover(self):
        self.wait(self.hover_ok, stable=self.options.hover_seconds, sim=15.)
        return {"stable_seconds": self.options.hover_seconds}

    def map_ready(self):
        self.start("aliked", [str(ROOT / "target/release/aliked")])
        self.start("lightglue", [str(ROOT / "target/release/lightglue"), "--map", str(ROOT / "apps/planner/maps/rmuc2026.ffvmap")])
        self.wait(lambda: "corrected" in self.latest and self.latest["corrected"][4], wall=30.)
        self.wait(lambda: self.visual_updates >= 2, wall=60.)
        self.start("planner", [str(ROOT / "target/release/planner"), "--goal", *map(str, self.options.waypoints[0])])
        self.wait(lambda: "reference" in self.latest, wall=30.)
        return {"source": "VIO + ALIKED-N16 + LightGlue + PnP + localization", "accepted_visual_updates": self.visual_updates, "online_loop_closure": "not_implemented"}

    def tracking(self):
        self.command("fc", "track")
        self.wait(lambda: self.mode == 4, wall=5.)
        arrivals = []
        for waypoint in self.options.waypoints:
            self.command("planner", "goal", *waypoint)
            goal = np.asarray(waypoint)
            self.wait(lambda: np.linalg.norm(self.latest["gt"][1] - goal) < self.options.goal_tolerance_m
                      and np.linalg.norm(self.latest["gt"][2]) < self.options.goal_speed_mps,
                      stable=1., sim=self.options.waypoint_sim_s)
            arrivals.append({"goal_map_m": list(waypoint), "arrived_sim_s": self.t})
        self.command("fc", "hold")
        return {"arrivals": arrivals}

    def landing(self):
        self.command("fc", "land")
        self.wait(lambda: self.mode == 0 and abs(self.latest["gt"][1][2] - self.origin[2]) < 0.08
                  and np.linalg.norm(self.latest["gt"][2]) < 0.1, stable=1., sim=20.)
        return {"landed_map_m": self.latest["gt"][1].tolist()}

    def reference_loss(self):
        self.command("fc", "track")
        self.wait(lambda: self.mode == 4, wall=5.)
        origin = self.latest["gt"][1].copy()
        self.fault_origin = origin
        start = time.monotonic()
        self.recorder.event("fault inject: stop planner with SIGINT", self.t)
        self.stop("planner")
        self.wait(lambda: self.mode == 3, wall=max(0.01, self.options.reference_loss_wall_s - (time.monotonic() - start)))
        latency = time.monotonic() - start
        if latency > self.options.reference_loss_wall_s:
            raise MissionFailure(f"reference loss response too slow: {latency}s")
        return {"hold_latency_wall_s": latency, "max_allowed_latency_wall_s": self.options.reference_loss_wall_s,
                "fault_origin_map_m": origin.tolist(), "hold_position_map_m": self.latest["gt"][1].tolist()}

    def failure_hold(self):
        self.wait(lambda: self.mode == 3 and np.linalg.norm(self.latest["gt"][1] - self.fault_origin) < 0.5,
                  stable=2., sim=5.)
        return {"hold_radius_m": 0.5, "stable_seconds": 2.0}

    def estimator_fallback(self):
        self.recorder.event("fault inject: stop VIO with SIGINT", self.t)
        start = time.monotonic()
        self.stop("vio")
        self.wait(lambda: self.mode == 6 and min(self.last_motors) > 0., wall=max(0.01, self.options.estimator_fallback_wall_s - (time.monotonic() - start)))
        latency = time.monotonic() - start
        if latency > self.options.estimator_fallback_wall_s:
            raise MissionFailure(f"estimator fallback too slow: {latency}s")
        return {"attitude_fallback_latency_wall_s": latency, "flight_mode": self.mode}

    def degraded_support(self):
        self.wait(lambda: self.mode == 6 and min(self.last_motors) > 0., stable=3., sim=5.)
        return {"observed_seconds": 3., "capability": "IMU attitude support with nominal hover thrust; no position or altitude guarantee"}

    def shutdown(self):
        failures = []
        results = {}
        # 先停止物理，保留日志写入器直到所有计算进程都退出。
        order = ["sim", "planner", "lightglue", "aliked", "localization", "fc", "vio", "render"]
        for name in order:
            if name not in self.processes:
                continue
            start = time.monotonic()
            try:
                results[name] = {"exit_code": self.stop(name), "wall_seconds": time.monotonic() - start}
                if results[name]["exit_code"] != 0:
                    failures.append(f"{name} nonzero exit")
            except subprocess.TimeoutExpired:
                failures.append(f"{name} did not exit, pid={self.processes[name][0].pid}")
        if self.recorder:
            self.recorder.event(f"shutdown {results} failures={failures}", self.t, bool(failures))
            time.sleep(0.4)
        if "viz" in self.processes:
            try:
                results["viz"] = {"exit_code": self.stop("viz")}
                if results["viz"]["exit_code"] != 0:
                    failures.append("viz nonzero exit")
            except subprocess.TimeoutExpired:
                failures.append(f"viz did not exit, pid={self.processes['viz'][0].pid}")
        self.result["stages"]["shutdown"] = {"status": "failed" if failures else "passed", "processes": results, "failures": failures}
        self.result["live_counts"] = self.counts
        self.result["last_physics_counters"] = self.physics
        self.result["remaining_pids"] = [p.pid for p, _ in self.processes.values() if p.poll() is None]

    def run(self):
        phases = [("initialization", self.initialized), ("takeoff", self.takeoff), ("hover", self.hover)]
        if self.case != "estimator_loss":
            phases.append(("map_alignment", self.map_ready))
        if self.case == "nominal":
            phases.extend([("tracking", self.tracking), ("landing", self.landing)])
        elif self.case == "reference_loss":
            phases.extend([("reference_loss", self.reference_loss), ("failure_hold", self.failure_hold), ("landing", self.landing)])
        else:
            phases.extend([("estimator_fallback", self.estimator_fallback), ("degraded_support", self.degraded_support)])
        self.result["stages"] = {name: {"status": "blocked", "reason": "preceding phase not completed"} for name, _ in phases}
        try:
            self.start("viz", [sys.executable, "-m", "firefly_viz.main", "--save", str(self.recording)])
            time.sleep(2.)
            self.connect()
            for name in ["render", "vio", "fc", "localization"]:
                self.start(name, [str(ROOT / f"target/release/{name}")])
            self.start("sim", [sys.executable, str(ROOT / "tests/system/mission_sim.py")])
            for name, action in phases:
                self.phase(name, action)
        except KeyboardInterrupt:
            self.result.update(error="interrupted", interrupted=True)
        except Exception as error:
            self.result["error"] = str(error)
            if self.recorder:
                self.recorder.event(str(error), self.t, True)
        finally:
            self.shutdown()
        if self.case == "estimator_loss":
            self.result["stages"]["failure_terminal"] = {"status": "not_supported", "reason": "No independent position/altitude estimate: IMU attitude support cannot guarantee hover or safe landing"}
        self.result["status"] = "passed" if all(s["status"] == "passed" for s in self.result["stages"].values()) else "failed"
        return self.result
