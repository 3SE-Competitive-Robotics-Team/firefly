"""Release flight command script; Mimir records native state and completion events."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import iceoryx2 as iox2
from firefly_mujoco import LandingTargetMessage, LogMessage, OdomMessage, PlannerStatusMessage, ReferenceMessage, TraceContext
from firefly_mujoco.messages import ScalarMessage
from firefly_workspace import workspace_root
from firefly_workspace.process import ManagedProcess, isolated_run


def main(argv=None, *, hil=None, mission=None):
    parser = argparse.ArgumentParser(description=__doc__)
    if hil is None:
        parser.add_argument("--hil", action="store_true")
    else:
        parser.set_defaults(hil=hil)
    if mission is None:
        parser.add_argument("--mission", choices=["hover", "roundtrip"], default="roundtrip")
    else:
        parser.set_defaults(mission=mission)
    if mission != "roundtrip":
        parser.add_argument("--hover-seconds", type=float, help="hover duration in seconds (default: 20)")
    else:
        parser.set_defaults(hover_seconds=None)
    parser.add_argument("--precision-land", action="store_true", help="finish hover with down-camera precision landing")
    parser.add_argument("--seed", type=int, default=20260930)
    args = parser.parse_args(argv)
    if args.precision_land and args.mission != "hover":
        parser.error("--precision-land requires --mission hover")
    if args.hover_seconds is not None and args.mission != "hover":
        parser.error("--hover-seconds requires --mission hover")
    if args.hover_seconds is None:
        args.hover_seconds = 20.
    if not 0 < args.hover_seconds <= 600:
        parser.error("--hover-seconds must be in (0, 600]")
    root = workspace_root(Path(__file__))
    processes, latest, subs = {}, {}, {}
    mode, stamp, sequence, logs = None, 0., 0, None

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
        msg = LogMessage(sim_time=stamp, log_level=3, sequence=sequence)
        sequence += 1
        msg.wall_secs, msg.wall_nanos = divmod(time.time_ns(), 10**9)
        data = text.encode()[:512].decode(errors="ignore").encode()
        msg.tag[:6], msg.tag_len = b"flight", 6
        msg.text[:len(data)], msg.text_len = data, len(data)
        logs.loan_uninit().write_payload(msg).send()

    def pump(landing=False):
        nonlocal mode, stamp
        for name, child in processes.items():
            if child.process.poll() is not None and not (landing and name == "mimir"):
                raise RuntimeError(f"{name} exited {child.process.returncode}: {' | '.join(child.tail)}")
        for name, sub in subs.items():
            while (sample := sub.receive()) is not None:
                payload = sample.payload().contents
                msg = type(payload).from_buffer_copy(payload)
                latest[name] = msg
                if name == "odom":
                    stamp = max(stamp, msg.timestamp)
                elif name == "scalars" and bytes(msg.entity[:msg.entity_len]) == b"fc/debug/state":
                    mode = int(msg.scalars[0])
        time.sleep(.01)

    def wait(predicate, wall=30., landing=False):
        deadline = time.monotonic()+wall
        while time.monotonic() < deadline:
            pump(landing)
            if predicate():
                return
        raise TimeoutError("flight command/completion deadline exceeded")

    def command(*values):
        event("command ffctl "+" ".join(map(str, values)))
        result = subprocess.run([str(root/"target/release/ffctl"), *map(str, values)],
                                cwd=root, capture_output=True, text=True, check=True, timeout=5.)
        event(result.stdout.strip())

    def hold(seconds):
        end = stamp+seconds
        wait(lambda: stamp >= end, wall=seconds*2+30.)

    with isolated_run(root):
        error, cleanup, recording = None, [], None
        try:
            if args.hil:
                subprocess.run([str(root/"target/release/hil"), "--check"], cwd=root, check=True)
            recording = start("mimir", [str(root/"target/release/mimir")]).wait_recording()
            time.sleep(2.)
            iox2.set_log_level(iox2.LogLevel.Error)
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            def service(topic, payload, capacity, publishers=None):
                builder = node.service_builder(iox2.ServiceName.new("Firefly/"+topic)).publish_subscribe(payload).user_header(TraceContext).subscriber_max_buffer_size(capacity)
                if publishers is not None:
                    builder = builder.max_publishers(publishers)
                return builder.open_or_create()
            logs = service("Log", LogMessage, 256, 10).publisher_builder().create()
            for name, topic, payload, capacity, publishers in [
                ("landing", "LandingTarget", LandingTargetMessage, 2, None),
                ("odom", "Odometry", OdomMessage, 2, None),
                ("planner", "PlannerStatus", PlannerStatusMessage, 2, None),
                ("reference", "Reference", ReferenceMessage, 2, None),
                ("scalars", "VizScalars", ScalarMessage, 1024, 8),
            ]:
                subs[name] = service(topic, payload, capacity, publishers).subscriber_builder().buffer_size(capacity).create()
            event(f"flight transport={'hil' if args.hil else 'sil'} mission={args.mission} seed={args.seed} recording={recording}")
            start("sim", [sys.executable, "-c", "from firefly_sim.main import main; main(['--no-trace'], publish_plant_state=False)"])
            start("render", [str(root/"target/release/render")])
            if args.hil:
                start("hil", [str(root/"target/release/hil")])
            else:
                start("vio", [str(root/"target/release/firefly-rootslam"), "--no-loop-closures", "--no-mapping"])
                start("fc", [str(root/"target/release/fc")])
                if args.precision_land:
                    start("landing", [str(root/"target/release/landing"), "--simulation"])
                if args.mission == "roundtrip":
                    start("traj", [str(root/"target/release/traj")])
                    start("planner", [str(root/"target/release/planner")])
            wait(lambda: "odom" in latest and latest["odom"].is_initialized and stamp >= 4., wall=40.)
            command("fc", "arm")
            wait(lambda: mode == 1)
            command("fc", "takeoff", 1.)
            wait(lambda: mode == 3)
            hold(args.hover_seconds if args.mission == "hover" else 3.)
            if args.mission == "roundtrip":
                for goal in ([22., 0., 1.595], [0., 0., 1.]):
                    issued = time.time()
                    command("planner", "goal", *goal)
                    wait(lambda: "planner" in latest and "reference" in latest
                         and latest["planner"].goal_timestamp >= issued
                         and latest["reference"].goal_timestamp == latest["planner"].goal_timestamp)
                    identity = latest["planner"].goal_timestamp
                    command("fc", "track")
                    wait(lambda: mode == 4)
                    def finished():
                        if mode != 4:
                            raise RuntimeError(f"native FC left TRACK: mode={mode}")
                        return latest["planner"].goal_timestamp == identity and bool(latest["planner"].finished)
                    wait(finished, wall=100.)
                    pose = latest["odom"]
                    event(f"trajectory completed goal={goal} odom={[pose.position_x, pose.position_y, pose.position_z]}")
                    command("fc", "hold")
                    wait(lambda: mode == 3)
                    hold(2.)
            if args.precision_land:
                wait(lambda: "landing" in latest and latest["landing"].valid == 1
                     and 0 <= stamp-latest["landing"].timestamp < .25)
                command("fc", "land", "--precision")
                wait(lambda: mode == 5)
                wait(lambda: mode == 0, wall=45.)
                event("precision landing completed and DISARMED")
            event("flight commands completed")
        except (Exception, KeyboardInterrupt) as failure:
            error = str(failure) or type(failure).__name__
            event("flight interrupted: "+error)
        finally:
            try:
                if "odom" in latest and mode != 0:
                    command("fc", "land")
                    wait(lambda: mode == 0, wall=40., landing=True)
                    event("native FC landed and DISARMED")
            except Exception as failure:
                cleanup.append(f"landing: {failure}")
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
        print(json.dumps({"commands_completed": error is None, "error": error,
                          "cleanup_errors": cleanup, "recording": str(recording)}))
        return 1 if error or cleanup else 0


if __name__ == "__main__":
    raise SystemExit(main())
