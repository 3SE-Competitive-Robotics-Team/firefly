"""Own native processes, IPC inputs and the mission lifecycle on one thread."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time

import iceoryx2 as iox2
from firefly_mujoco import (
    LandingTargetMessage,
    LogMessage,
    OdomMessage,
    PlannerStatusMessage,
    ReferenceMessage,
    TraceContext,
)
from firefly_mujoco.messages import AttitudeMessage, ScalarMessage
from firefly_mujoco.activity import ActivityLease, LANDING
from firefly_mujoco.scene import load_scene_name
from firefly_workspace import digest, load_toml
from firefly_workspace.process import ManagedProcess, isolated_run
from .config import MissionOptions, Phase, route_points
from .gates import Arrival, VisualAcquisition


class FlightRunner:
    def __init__(self, options: MissionOptions, root: Path, team: str):
        self.options, self.root, self.team = (options, root, team)
        self.waypoints = route_points(options, team)
        self.down_rate = (
            load_toml(root / "configs/render.toml").get("capture", {}).get("down_hz", 15)
        )
        self.processes, self.latest, self.received, self.subs = ({}, {}, {}, {})
        self.mode, self.stamp, self.sequence, self.logs = (None, 0.0, 0, None)
        self.armed_session, self.activity, self.node, self.recording = (None, None, None, None)
        self.phase_name = Phase.STARTUP

    def _start(self, name, command):
        child = ManagedProcess(
            command,
            cwd=self.root,
            env={
                **os.environ,
                "RUST_LOG": os.environ.get("RUST_LOG", "info"),
                "FIREFLY_MISSION_SEED": str(self.options.seed),
            },
        )
        self.processes[name] = child
        return child

    def _event(self, text):
        print(text, flush=True)
        if self.logs is None or self.processes["mimir"].process.poll() is not None:
            return
        data = text.encode()
        if len(data) > 512:
            raise ValueError("flight event exceeds the registered TextLog payload")
        msg = LogMessage(sim_time=self.stamp, log_level=3, sequence=self.sequence)
        self.sequence += 1
        msg.wall_secs, msg.wall_nanos = divmod(time.time_ns(), 10**9)
        msg.tag[:6], msg.tag_len = (b"flight", 6)
        msg.text[: len(data)], msg.text_len = (data, len(data))
        self.logs.loan_uninit().write_payload(msg).send()

    def _mission_event(self, kind, **fields):
        self._event("mission " + json.dumps({"kind": kind, **fields}, separators=(",", ":")))

    def _request_activity(self, enabled):
        if self.activity is not None:
            self.activity.set(LANDING if enabled else 0, self.stamp)
            self._mission_event("activity", enabled=enabled)

    def _phase(self, name, **fields):
        self.phase_name = self.phase_name.advance(Phase(name))
        self._mission_event("phase", name=name, **fields)
        self._request_activity(name in {"takeoff", "acquire", "land"})

    def _pump(self, landing=False):
        if self.activity is not None:
            self.activity.pump(self.stamp)
        for name, child in self.processes.items():
            if child.process.poll() is not None and (not (landing and name == "mimir")):
                raise RuntimeError(
                    f"{name} exited {child.process.returncode}: {' | '.join(child.tail)}"
                )
        for name, sub in self.subs.items():
            while (sample := sub.receive()) is not None:
                payload = sample.payload().contents
                msg = type(payload).from_buffer_copy(payload)
                self.latest[name], self.received[name] = (msg, time.monotonic())
                if name == "odom":
                    self.stamp = max(self.stamp, msg.timestamp)
                elif name == "scalars" and bytes(msg.entity[: msg.entity_len]) == b"fc/debug/state":
                    self.mode = int(msg.scalars[0])
                    self.received["mode"] = time.monotonic()
                elif (
                    name == "attitude"
                    and self.armed_session is not None
                    and (msg.session != self.armed_session)
                ):
                    raise RuntimeError("VIO session changed during mission")
        time.sleep(0.01)

    def _wait(self, predicate, wall=30.0, *, modes=None, landing=False):
        deadline = time.monotonic() + wall
        while time.monotonic() < deadline:
            self._pump(landing)
            if modes is not None:
                if self.mode not in modes:
                    raise RuntimeError(
                        f"{self.phase_name}: unexpected FC mode {self.mode}, expected {sorted(modes)}"
                    )
                for topic in ("odom", "mode", "attitude"):
                    if time.monotonic() - self.received.get(topic, 0.0) > 1.0:
                        raise RuntimeError(f"{self.phase_name}: stale {topic}")
            if predicate():
                return
        raise TimeoutError(f"{self.phase_name}: completion deadline exceeded ({wall:g}s)")

    def _command(self, *values):
        self._event("command ffctl " + " ".join(map(str, values)))
        result = subprocess.run(
            [str(self.root / "target/release/ffctl"), *map(str, values)],
            cwd=self.root,
            capture_output=True,
            text=True,
            check=False,
            timeout=5.0,
        )
        if result.returncode:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(
                f"ffctl {values[0]} {values[1]} failed ({result.returncode}): {detail[:240]}"
            )
        self._event(result.stdout.strip())

    def _hold(self, seconds, *, modes=(3,)):
        end = self.stamp + seconds
        self._wait(lambda: self.stamp >= end, wall=seconds * 2 + 30.0, modes=set(modes))

    def _navigate(self, name, goal, **fields):
        self._phase(name, goal=goal, **fields)
        issued = time.time()
        self._command("planner", "goal", "--", *goal)
        self._wait(
            lambda: (
                "planner" in self.latest
                and "reference" in self.latest
                and (self.latest["planner"].goal_timestamp >= issued)
                and (
                    self.latest["reference"].goal_timestamp == self.latest["planner"].goal_timestamp
                )
                and (0 <= self.stamp - self.latest["reference"].timestamp <= 0.25)
            ),
            modes={3},
        )
        identity = self.latest["planner"].goal_timestamp
        self._command("fc", "track")
        self._wait(lambda: self.mode == 4, modes={3, 4})
        arrival = Arrival(tuple(goal), identity)
        self._wait(
            lambda: arrival.observe(
                self.latest.get("planner"), self.latest.get("odom"), self.stamp
            ),
            wall=100.0,
            modes={4},
        )
        self._command("fc", "hold")
        self._wait(lambda: self.mode == 3, modes={3, 4})

    def _startup(self):
        if self.options.hil:
            subprocess.run(
                [str(self.root / "target/release/hil"), "--check"], cwd=self.root, check=True
            )
        self.recording = self._start(
            "mimir", [str(self.root / "target/release/mimir")]
        ).wait_recording()
        time.sleep(2.0)
        iox2.set_log_level(iox2.LogLevel.Error)
        self.node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)

        def service(topic, payload, capacity, publishers=None):
            builder = (
                self.node.service_builder(iox2.ServiceName.new("Firefly/" + topic))
                .publish_subscribe(payload)
                .user_header(TraceContext)
                .subscriber_max_buffer_size(capacity)
            )
            if publishers is not None:
                builder = builder.max_publishers(publishers)
            return builder.open_or_create()

        self.logs = service("Log", LogMessage, 256, 10).publisher_builder().create()
        if self.options.precision_land:
            self.activity = ActivityLease(self.node)
        self._event(f"flight recording={self.recording}")
        revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=self.root, text=True
        ).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=self.root))
        self._mission_event("source", revision=revision, dirty=dirty)
        for name, topic, payload, capacity, publishers in [
            ("landing", "LandingTarget", LandingTargetMessage, 2, None),
            ("odom", "Odometry", OdomMessage, 2, None),
            ("attitude", "VioAttitude", AttitudeMessage, 2, None),
            ("planner", "PlannerStatus", PlannerStatusMessage, 2, None),
            ("reference", "Reference", ReferenceMessage, 2, None),
            ("scalars", "VizScalars", ScalarMessage, 1024, 8),
        ]:
            self.subs[name] = (
                service(topic, payload, capacity, publishers)
                .subscriber_builder()
                .buffer_size(capacity)
                .create()
            )
        self._mission_event(
            "settings",
            mission=self.options.mission,
            precision_land=self.options.precision_land,
            team=self.team,
            scene=load_scene_name(),
            waypoint_count=len(self.waypoints),
            shape=self.options.shape,
            altitude=self.options.altitude,
            return_offset=self.options.return_offset,
            hover_seconds=self.options.hover_seconds,
            down_rate_hz=self.down_rate,
            seed=self.options.seed,
            transport="hil" if self.options.hil else "sil",
        )
        for index, offset in enumerate(self.waypoints):
            self._mission_event("waypoint", index=index, offset=offset)
        binaries = ["render", "mimir", "ffctl"] + (
            ["hil"] if self.options.hil else ["fc", "firefly-rootslam"]
        )
        sim_config = "sim-mission" if self.options.mission == "roundtrip" else "sim"
        configs = ["game", "scene", sim_config, "render", "mimir"]
        if self.options.hil:
            configs += ["hil", "processes-hil", "fc-hil", "rootslam-hil"]
        else:
            configs += ["fc", "firefly-rootslam"]
        if self.options.mission == "roundtrip":
            binaries += [] if self.options.hil else ["planner", "traj"]
            configs += ["planner", "traj"]
        if self.options.precision_land:
            binaries += [] if self.options.hil else ["landing"]
            configs += ["landing"]
        paths = [self.root / "target/release" / name for name in binaries]
        paths += [self.root / "configs" / f"{name}.toml" for name in configs]
        paths += [
            self.root / "scripts/flight.py",
            self.root / "scripts/verify_flight.py",
            self.root / "apps/firefly-sim/src/firefly_sim/main.py",
            self.root / "packages/firefly-mujoco/src/firefly_mujoco/scene.py",
            self.root / "packages/firefly-mujoco/src/firefly_mujoco/env.py",
            self.root / "packages/firefly-mujoco/src/firefly_mujoco/activity.py",
            self.root / "packages/firefly-mujoco/src/firefly_mujoco/messages.py",
        ]
        if self.options.precision_land:
            paths += [self.root / "models/h-marker-yolo26n-cif-v1/manifest.json"]
        paths += sorted(Path(__file__).parent.glob("*.py"))
        for path in paths:
            if path.suffix == ".toml" and (not path.exists()):
                self._mission_event(
                    "provenance", path=str(path.relative_to(self.root)), defaults=True
                )
            else:
                self._mission_event(
                    "provenance", path=str(path.relative_to(self.root)), sha256=digest(path)
                )
        self._start(
            "sim",
            [
                sys.executable,
                "-c",
                f"from firefly_sim.main import main; main(['--no-trace', '--config', 'configs/{sim_config}.toml'], publish_plant_state=False)",
            ],
        )
        self._start("render", [str(self.root / "target/release/render")])
        if self.options.hil:
            self._start("hil", [str(self.root / "target/release/hil")])
        else:
            self._start(
                "vio",
                [
                    str(self.root / "target/release/firefly-rootslam"),
                    "--no-loop-closures",
                    "--no-mapping",
                ],
            )
            self._start("fc", [str(self.root / "target/release/fc")])
            if self.options.precision_land:
                self._start("landing", [str(self.root / "target/release/landing"), "--simulation"])
            if self.options.mission == "roundtrip":
                self._start("traj", [str(self.root / "target/release/traj")])
                self._start("planner", [str(self.root / "target/release/planner")])

    def _execute(self):
        self._wait(
            lambda: (
                "odom" in self.latest
                and self.latest["odom"].is_initialized
                and ("attitude" in self.latest)
                and (self.mode == 0)
                and (self.stamp >= 4.0)
            ),
            wall=40.0,
        )
        self.armed_session = self.latest["attitude"].session
        pose = self.latest["odom"]
        home = [pose.position_x, pose.position_y, pose.position_z]
        quaternion = [pose.quat_x, pose.quat_y, pose.quat_z, pose.quat_w]
        self._mission_event(
            "home", position=home, quaternion=quaternion, session=self.armed_session
        )
        self._phase("takeoff")
        self._command("fc", "arm")
        self._wait(lambda: self.mode == 1, modes={0, 1})
        self._command("fc", "takeoff", self.options.altitude)
        self._wait(lambda: self.mode == 3, modes={1, 2, 3})
        if self.options.mission == "hover":
            self._request_activity(False)
        self._hold(1.0 if self.options.mission == "roundtrip" else self.options.hover_seconds)
        if self.options.mission == "roundtrip":
            nearby = [
                home[0] + self.options.return_offset[0],
                home[1] + self.options.return_offset[1],
                home[2] + self.options.altitude,
            ]
            for index, offset in enumerate(self.waypoints):
                waypoint = [
                    home[0] + offset[0],
                    home[1] + offset[1],
                    home[2] + self.options.altitude,
                ]
                self._navigate("waypoint", waypoint, index=index)
                self._phase("waypoint_hover", goal=waypoint, index=index)
                self._hold(self.options.hover_seconds)
            self._navigate("return", nearby)
        if self.options.precision_land:
            self._phase("acquire")
            acquisition = VisualAcquisition(
                after=self.stamp, session=self.armed_session, team=int(self.team == "blue")
            )
            self._wait(
                lambda: acquisition.observe(
                    self.latest.get("landing"),
                    self.stamp,
                    self.received.get("landing", 0.0),
                    time.monotonic(),
                ),
                wall=self.options.acquire_timeout,
                modes={3},
            )
            self._phase("land", target_time=acquisition.target_time)
            self._command("fc", "land", "--precision")
            self._wait(lambda: self.mode == 5, modes={3, 5})
            self._wait(lambda: self.mode == 0, wall=45.0, modes={0, 5})
            self._hold(0.75, modes=(0,))
        else:
            self._phase("land")
            self._command("fc", "land")
            self._wait(lambda: self.mode == 5, modes={3, 5})
            self._wait(lambda: self.mode == 0, wall=45.0, modes={0, 5})
            self._hold(0.75, modes=(0,))
        self._phase("complete")
        if self.activity is not None:
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                self._pump()

    def _close(self, error: str | None) -> list[str]:
        cleanup = []
        try:
            if "odom" in self.latest and self.mode != 0:
                if error and self.options.precision_land:
                    self._command("fc", "hold")
                    self._event("abort: HOLD requested before simulation shutdown")
                else:
                    self._command("fc", "land")
                    self._wait(lambda: self.mode == 0, wall=40.0, landing=True)
                    self._event("native FC landed and DISARMED")
        except Exception as failure:
            cleanup.append(f"landing: {failure}")
        if self.activity is not None:
            try:
                self.activity.set(0, self.stamp)
            except Exception as failure:
                cleanup.append(f"inference lease: {failure}")
        for name, child in reversed(self.processes.items()):
            try:
                code = child.stop()
                if code:
                    cleanup.append(f"{name}: exit {code}")
            except Exception as failure:
                cleanup.append(f"{name}: {failure}")
                try:
                    if child.process.poll() is None:
                        child.process.kill()
                        child.process.wait(timeout=5.0)
                        child.reader.join(timeout=5.0)
                except Exception as forced_failure:
                    cleanup.append(f"{name} forced shutdown: {forced_failure}")
        return cleanup

    def run(self) -> int:
        error, cleanup, verification, completed = (None, [], None, False)
        with isolated_run(self.root):
            try:
                self._startup()
                self._execute()
                completed = True
            except (Exception, KeyboardInterrupt) as failure:
                error = str(failure) or type(failure).__name__
                try:
                    self._mission_event("failure", phase=self.phase_name, reason=error[:64])
                except Exception as logging_failure:
                    error += f"; failure recording: {logging_failure}"
            finally:
                cleanup = self._close(error)
            if self.recording is not None and self.options.precision_land:
                from .verification import verify_recording

                try:
                    verification = verify_recording(self.recording)
                    if not verification["passed"]:
                        error = error or "closed-loop verification failed"
                except Exception as failure:
                    verification = {
                        "passed": False,
                        "failures": [f"recording verification: {failure}"],
                        "metrics": {},
                    }
                    error = error or "closed-loop verification failed"
            print(
                json.dumps(
                    {
                        "commands_completed": completed,
                        "error": error,
                        "cleanup_errors": cleanup,
                        "recording": str(self.recording),
                        "verification": verification,
                    }
                )
            )
            return 1 if error or cleanup else 0
