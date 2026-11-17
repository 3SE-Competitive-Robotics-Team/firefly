"""Run native SIL/USB HIL missions; Mimir owns all runtime evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import iceoryx2 as iox2
from firefly_mujoco import LandingTargetMessage, LogMessage, OdomMessage, PlannerStatusMessage, ReferenceMessage, TraceContext
from firefly_mujoco.messages import AttitudeMessage, ScalarMessage
from firefly_mujoco.activity import ActivityLease, LANDING
from firefly_mujoco.scene import load_scene_name
from firefly_workspace import digest, workspace_root
from firefly_workspace.game import load_team
from firefly_workspace.process import ManagedProcess, isolated_run


def parse_args(argv=None, *, hil=None, mission=None):
    parser = argparse.ArgumentParser(description=__doc__)
    if hil is None:
        parser.add_argument("--hil", action="store_true")
    else:
        parser.set_defaults(hil=hil)
    if mission is None:
        parser.add_argument("--mission", choices=["hover", "roundtrip"], default="roundtrip")
    else:
        parser.set_defaults(mission=mission)
    parser.add_argument("--hover-seconds", type=float, help="hover duration (hover: 20s; roundtrip waypoint: 3s)")
    route = parser.add_mutually_exclusive_group()
    route.add_argument("--goal", type=float, nargs=2, metavar=("DX", "DY"),
                       help="single waypoint offset in odom metres (red: 4,0; blue: -4,0)")
    route.add_argument("--shape", choices=("square", "triangle"), help="closed polygon one side-length ahead of takeoff")
    route.add_argument("--waypoint", type=float, nargs=2, action="append", metavar=("DX", "DY"),
                       help="repeat for ordered offsets from takeoff in odom metres")
    parser.add_argument("--side", type=float, help="square/triangle side length in metres (default: 2)")
    parser.add_argument("--altitude", type=float, help="height above takeoff (hover: 1m; roundtrip: 2m)")
    parser.add_argument("--return-offset", type=float, nargs=2, default=[.12, -.08], metavar=("DX", "DY"),
                        help="return near takeoff before visual alignment, in odom metres")
    parser.add_argument("--precision-land", action="store_true", help="finish with fresh down-camera acquisition and precision landing")
    parser.add_argument("--acquire-timeout", type=float, default=12., help="visual reacquisition deadline, in wall seconds")
    parser.add_argument("--down-rate", type=int, choices=(15, 30), default=15, help="active down-camera FPS")
    parser.add_argument("--seed", type=int, default=20260930)
    args = parser.parse_args(argv)
    if args.hover_seconds is None:
        args.hover_seconds = 20. if args.mission == "hover" else 3.
    if args.altitude is None:
        args.altitude = 1. if args.mission == "hover" else 2.
    if args.side is not None and args.shape is None:
        parser.error("--side requires --shape")
    if args.mission == "hover" and (args.goal is not None or args.shape or args.waypoint):
        parser.error("waypoints and shapes require --mission roundtrip")
    if args.shape and args.side is None:
        args.side = 2.
    values = [*(args.goal or []), *(value for point in args.waypoint or [] for value in point),
              *([] if args.side is None else [args.side]),
              *args.return_offset, args.altitude, args.hover_seconds, args.acquire_timeout]
    if not all(math.isfinite(value) for value in values):
        parser.error("mission parameters must be finite")
    if not 0 < args.hover_seconds <= 600 or not .5 <= args.altitude <= 2.:
        parser.error("hover duration must be in (0,600] and altitude in [0.5,2] metres")
    if not 1 <= args.acquire_timeout <= 60 or math.hypot(*args.return_offset) > .3:
        parser.error("acquisition timeout must be in [1,60] seconds and return offset within 0.3m")
    if args.mission == "roundtrip" and args.goal is not None and not 1. <= math.hypot(*args.goal) <= 10.:
        parser.error("roundtrip waypoint distance must be in [1,10] metres")
    if args.shape and not 1. <= args.side <= 4.:
        parser.error("shape side length must be in [1,4] metres")
    if args.waypoint:
        if len(args.waypoint) > 16 or not 1. <= math.hypot(*args.waypoint[0]) <= 10.:
            parser.error("provide 1..16 waypoints; the first must be 1..10 metres from takeoff")
        previous = [0., 0.]
        for point in args.waypoint:
            if math.hypot(*point) > 10. or not .5 <= math.dist(previous, point) <= 10.:
                parser.error("waypoints must stay within 10m of takeoff, with consecutive legs in [0.5,10]m")
            previous = point
    return args


def default_goal(team):
    """RMUC demonstration routes head towards the central field from either hangar."""
    return [4., 0.] if team == "red" else [-4., 0.]


def route_points(args, team):
    """Ordered offsets in initial odom axes; polygons close at their field-side entry."""
    if args.mission == "hover":
        return []
    if args.waypoint:
        return args.waypoint
    if args.shape:
        side = args.side
        points = ([[side, 0.], [2*side, 0.], [2*side, side], [side, side], [side, 0.]] if args.shape == "square"
                  else [[side, 0.], [2*side, 0.], [1.5*side, math.sqrt(3.) * side / 2.], [side, 0.]])
        direction = 1. if team == "red" else -1.
        return [[direction * x, direction * y] for x, y in points]
    return [args.goal if args.goal is not None else default_goal(team)]


def at_goal(message, goal):
    position = [message.position_x, message.position_y, message.position_z]
    velocity = [message.velocity_x, message.velocity_y, message.velocity_z]
    return math.dist(position, goal) <= .15 and math.hypot(*velocity) <= .12


def fresh_target(message, stamp, *, after, session, team):
    return (message.valid == 1 and message.session == session and message.team == team
            and after < message.timestamp <= stamp and stamp - message.timestamp <= .25
            and all(math.isfinite(value) for value in message.position)
            and math.isfinite(message.confidence) and .5 <= message.confidence <= 1.
            and math.isfinite(message.reprojection_px) and 0 <= message.reprojection_px <= 2.)


def main(argv=None, *, hil=None, mission=None):
    args = parse_args(argv, hil=hil, mission=mission)
    root = workspace_root(Path(__file__))
    team = load_team()
    waypoints = route_points(args, team)
    processes, latest, received, subs = {}, {}, {}, {}
    mode, stamp, sequence, logs = None, 0., 0, None
    armed_session = None
    activity = None
    phase_name = "startup"

    def start(name, command):
        child = ManagedProcess(command, cwd=root, env={**os.environ,
            "RUST_LOG": os.environ.get("RUST_LOG", "info"), "FIREFLY_MISSION_SEED": str(args.seed)})
        processes[name] = child
        return child

    def event(text):
        nonlocal sequence
        print(text, flush=True)
        if logs is None or processes["mimir"].process.poll() is not None:
            return
        data = text.encode()
        if len(data) > 512:
            raise ValueError("flight event exceeds the registered TextLog payload")
        msg = LogMessage(sim_time=stamp, log_level=3, sequence=sequence)
        sequence += 1
        msg.wall_secs, msg.wall_nanos = divmod(time.time_ns(), 10**9)
        msg.tag[:6], msg.tag_len = b"flight", 6
        msg.text[:len(data)], msg.text_len = data, len(data)
        logs.loan_uninit().write_payload(msg).send()

    def mission_event(kind, **fields):
        event("mission " + json.dumps({"kind": kind, **fields}, separators=(",", ":")))

    def request_activity(enabled):
        if activity is not None:
            activity.set(LANDING if enabled else 0, stamp)
            mission_event("activity", enabled=enabled, rate_hz=args.down_rate)

    def phase(name, **fields):
        nonlocal phase_name
        phase_name = name
        mission_event("phase", name=name, **fields)
        request_activity(name in {"takeoff", "acquire", "land"})

    def pump(landing=False):
        nonlocal mode, stamp
        if activity is not None:
            activity.pump(stamp)
        for name, child in processes.items():
            if child.process.poll() is not None and not (landing and name == "mimir"):
                raise RuntimeError(f"{name} exited {child.process.returncode}: {' | '.join(child.tail)}")
        for name, sub in subs.items():
            while (sample := sub.receive()) is not None:
                payload = sample.payload().contents
                msg = type(payload).from_buffer_copy(payload)
                latest[name], received[name] = msg, time.monotonic()
                if name == "odom":
                    stamp = max(stamp, msg.timestamp)
                elif name == "scalars" and bytes(msg.entity[:msg.entity_len]) == b"fc/debug/state":
                    mode = int(msg.scalars[0])
                    received["mode"] = time.monotonic()
                elif name == "attitude" and armed_session is not None and msg.session != armed_session:
                    raise RuntimeError("VIO session changed during mission")
        time.sleep(.01)

    def wait(predicate, wall=30., *, modes=None, landing=False):
        deadline = time.monotonic() + wall
        while time.monotonic() < deadline:
            pump(landing)
            if modes is not None:
                if mode not in modes:
                    raise RuntimeError(f"{phase_name}: unexpected FC mode {mode}, expected {sorted(modes)}")
                for topic in ("odom", "mode", "attitude"):
                    if time.monotonic() - received.get(topic, 0.) > 1.:
                        raise RuntimeError(f"{phase_name}: stale {topic}")
            if predicate():
                return
        raise TimeoutError(f"{phase_name}: completion deadline exceeded ({wall:g}s)")

    def command(*values):
        event("command ffctl " + " ".join(map(str, values)))
        result = subprocess.run([str(root / "target/release/ffctl"), *map(str, values)],
                                cwd=root, capture_output=True, text=True, check=False, timeout=5.)
        if result.returncode:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(f"ffctl {values[0]} {values[1]} failed ({result.returncode}): {detail[:240]}")
        event(result.stdout.strip())

    def hold(seconds, *, modes=(3,)):
        end = stamp + seconds
        wait(lambda: stamp >= end, wall=seconds * 2 + 30., modes=set(modes))

    def navigate(name, goal, **fields):
        phase(name, goal=goal, **fields)
        issued = time.time()
        command("planner", "goal", "--", *goal)
        wait(lambda: "planner" in latest and "reference" in latest
             and latest["planner"].goal_timestamp >= issued
             and latest["reference"].goal_timestamp == latest["planner"].goal_timestamp
             and 0 <= stamp - latest["reference"].timestamp <= .25, modes={3})
        identity = latest["planner"].goal_timestamp
        command("fc", "track")
        wait(lambda: mode == 4, modes={3, 4})
        settled_since = None

        def finished():
            nonlocal settled_since
            planner = latest["planner"]
            arrived = (planner.goal_timestamp == identity and bool(planner.finished)
                       and 0 <= stamp - planner.timestamp <= .5
                       and at_goal(latest["odom"], goal))
            settled_since = (stamp if settled_since is None else settled_since) if arrived else None
            return settled_since is not None and stamp - settled_since >= .5

        wait(finished, wall=100., modes={4})
        command("fc", "hold")
        wait(lambda: mode == 3, modes={3, 4})

    with isolated_run(root):
        error, cleanup, recording, verification = None, [], None, None
        completed = False
        try:
            if args.hil:
                subprocess.run([str(root / "target/release/hil"), "--check"], cwd=root, check=True)
            recording = start("mimir", [str(root / "target/release/mimir")]).wait_recording()
            time.sleep(2.)
            iox2.set_log_level(iox2.LogLevel.Error)
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)

            def service(topic, payload, capacity, publishers=None):
                builder = node.service_builder(iox2.ServiceName.new("Firefly/" + topic)).publish_subscribe(payload).user_header(TraceContext).subscriber_max_buffer_size(capacity)
                if publishers is not None:
                    builder = builder.max_publishers(publishers)
                return builder.open_or_create()

            logs = service("Log", LogMessage, 256, 10).publisher_builder().create()
            if args.precision_land:
                activity = ActivityLease(node, rate_hz=args.down_rate)
            event(f"flight recording={recording}")
            revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
            dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=root))
            mission_event("source", revision=revision, dirty=dirty)
            for name, topic, payload, capacity, publishers in [
                ("landing", "LandingTarget", LandingTargetMessage, 2, None),
                ("odom", "Odometry", OdomMessage, 2, None),
                ("attitude", "VioAttitude", AttitudeMessage, 2, None),
                ("planner", "PlannerStatus", PlannerStatusMessage, 2, None),
                ("reference", "Reference", ReferenceMessage, 2, None),
                ("scalars", "VizScalars", ScalarMessage, 1024, 8),
            ]:
                subs[name] = service(topic, payload, capacity, publishers).subscriber_builder().buffer_size(capacity).create()
            mission_event("settings", mission=args.mission, precision_land=args.precision_land,
                          team=team, scene=load_scene_name(), waypoint_count=len(waypoints),
                          shape=args.shape, altitude=args.altitude, return_offset=args.return_offset,
                          hover_seconds=args.hover_seconds, down_rate_hz=args.down_rate, seed=args.seed, transport="hil" if args.hil else "sil")
            for index, offset in enumerate(waypoints):
                mission_event("waypoint", index=index, offset=offset)
            binaries = ["render", "mimir", "ffctl"] + (["hil"] if args.hil else ["fc", "firefly-rootslam"])
            sim_config = "sim-mission" if args.mission == "roundtrip" else "sim"
            configs = ["game", "scene", sim_config, "render", "mimir"]
            if args.hil:
                configs += ["hil", "processes-hil", "fc-hil", "rootslam-hil"]
            else:
                configs += ["fc", "firefly-rootslam"]
            if args.mission == "roundtrip":
                binaries += [] if args.hil else ["planner", "traj"]
                configs += ["planner", "traj"]
            if args.precision_land:
                binaries += [] if args.hil else ["landing"]
                configs += ["landing"]
            paths = [root / "target/release" / name for name in binaries]
            paths += [root / "configs" / f"{name}.toml" for name in configs]
            paths += [Path(__file__).resolve(), root / "scripts/verify_flight.py",
                      root / "apps/firefly-sim/src/firefly_sim/main.py",
                      root / "packages/firefly-mujoco/src/firefly_mujoco/scene.py",
                      root / "packages/firefly-mujoco/src/firefly_mujoco/env.py",
                      root / "packages/firefly-mujoco/src/firefly_mujoco/activity.py",
                      root / "packages/firefly-mujoco/src/firefly_mujoco/messages.py"]
            if args.precision_land:
                paths += [root / "models/h-marker-yolo26n-cif-v1/manifest.json"]
            for path in paths:
                if path.suffix == ".toml" and not path.exists():
                    mission_event("provenance", path=str(path.relative_to(root)), defaults=True)
                else:
                    mission_event("provenance", path=str(path.relative_to(root)), sha256=digest(path))
            start("sim", [sys.executable, "-c",
                f"from firefly_sim.main import main; main(['--no-trace', '--config', 'configs/{sim_config}.toml'], publish_plant_state=False)"])
            start("render", [str(root / "target/release/render")])
            if args.hil:
                start("hil", [str(root / "target/release/hil")])
            else:
                start("vio", [str(root / "target/release/firefly-rootslam"), "--no-loop-closures", "--no-mapping"])
                start("fc", [str(root / "target/release/fc")])
                if args.precision_land:
                    start("landing", [str(root / "target/release/landing"), "--simulation"])
                if args.mission == "roundtrip":
                    start("traj", [str(root / "target/release/traj")])
                    start("planner", [str(root / "target/release/planner")])
            wait(lambda: "odom" in latest and latest["odom"].is_initialized
                 and "attitude" in latest and mode == 0 and stamp >= 4., wall=40.)
            armed_session = latest["attitude"].session
            pose = latest["odom"]
            home = [pose.position_x, pose.position_y, pose.position_z]
            quaternion = [pose.quat_x, pose.quat_y, pose.quat_z, pose.quat_w]
            mission_event("home", position=home, quaternion=quaternion, session=armed_session)
            phase("takeoff")
            command("fc", "arm")
            wait(lambda: mode == 1, modes={0, 1})
            command("fc", "takeoff", args.altitude)
            wait(lambda: mode == 3, modes={1, 2, 3})
            if args.mission == "hover":
                request_activity(False)
            hold(1. if args.mission == "roundtrip" else args.hover_seconds)
            if args.mission == "roundtrip":
                nearby = [home[0] + args.return_offset[0], home[1] + args.return_offset[1], home[2] + args.altitude]
                for index, offset in enumerate(waypoints):
                    waypoint = [home[0] + offset[0], home[1] + offset[1], home[2] + args.altitude]
                    navigate("waypoint", waypoint, index=index)
                    phase("waypoint_hover", goal=waypoint, index=index)
                    hold(args.hover_seconds)
                navigate("return", nearby)
            if args.precision_land:
                phase("acquire")
                acquire_stamp, observations, last_target = stamp, [], None

                def acquired():
                    nonlocal last_target
                    message = latest.get("landing")
                    if message is None or not fresh_target(message, stamp, after=acquire_stamp,
                            session=armed_session, team=int(team == "blue")):
                        observations.clear()
                        return False
                    if time.monotonic() - received["landing"] > .25:
                        observations.clear()
                        return False
                    if message.timestamp != last_target:
                        observations.append(message.timestamp)
                        last_target = message.timestamp
                    return len(observations) >= 3 and observations[-1] - observations[0] >= .15

                wait(acquired, wall=args.acquire_timeout, modes={3})
                phase("land", target_time=observations[-1])
                command("fc", "land", "--precision")
                wait(lambda: mode == 5, modes={3, 5})
                wait(lambda: mode == 0, wall=45., modes={0, 5})
                hold(.75, modes=(0,))
            else:
                phase("land")
                command("fc", "land")
                wait(lambda: mode == 5, modes={3, 5})
                wait(lambda: mode == 0, wall=45., modes={0, 5})
                hold(.75, modes=(0,))
            phase("complete")
            if activity is not None:
                # Let both native and USB receivers observe release before stopping their processes.
                deadline = time.monotonic() + .3
                while time.monotonic() < deadline:
                    pump()
            completed = True
        except (Exception, KeyboardInterrupt) as failure:
            error = str(failure) or type(failure).__name__
            mission_event("failure", phase=phase_name, reason=error[:64])
        finally:
            try:
                if "odom" in latest and mode != 0:
                    if error and args.precision_land:
                        command("fc", "hold")
                        event("abort: HOLD requested before simulation shutdown")
                    else:
                        command("fc", "land")
                        wait(lambda: mode == 0, wall=40., landing=True)
                        event("native FC landed and DISARMED")
            except Exception as failure:
                cleanup.append(f"landing: {failure}")
            if activity is not None:
                activity.set(0, stamp)
            for name, child in reversed(processes.items()):
                try:
                    code = child.stop()
                    if code:
                        cleanup.append(f"{name}: exit {code}")
                except Exception as failure:
                    cleanup.append(f"{name}: {failure}")
                    if child.process.poll() is None:
                        child.process.kill()
                        child.process.wait(timeout=5.)
                        child.reader.join(timeout=5.)
        if recording is not None and args.precision_land:
            from verify_flight import verify_recording
            try:
                verification = verify_recording(recording)
                if not verification["passed"]:
                    error = error or "closed-loop verification failed"
            except Exception as failure:
                verification = {"passed": False, "failures": [f"recording verification: {failure}"], "metrics": {}}
                error = error or "closed-loop verification failed"
        print(json.dumps({"commands_completed": completed, "error": error,
                          "cleanup_errors": cleanup, "recording": str(recording), "verification": verification}))
        return 1 if error or cleanup else 0


if __name__ == "__main__":
    raise SystemExit(main())
