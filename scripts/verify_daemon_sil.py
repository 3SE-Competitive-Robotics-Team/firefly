"""Exercise independent onboard clients against one continuous native SIL stack."""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path

from firefly_workspace import load_toml
from firefly_workspace.process import ManagedProcess, isolated_run
from flight_mission.recording import read_recording
from flight_mission.verification import evaluate

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "target/release"
SHAPES = ("square", "cuboid")


def grade(path):
    data = read_recording(path)
    # The test case supplies the expected mission count and scene configuration.
    # The daemon supplies all measured phase boundaries, identities and anchors.
    data.events = [
        (stamp, event)
        for stamp, event in data.events
        if event.get("mission_id") != "restart-test"
    ]
    for _, event in data.events:
        shape = event.get("mission_id")
        if shape not in SHAPES:
            raise ValueError(f"unexpected mission identity: {shape}")
        event["flight_id"] = SHAPES.index(shape) + 1
        if event["kind"] == "settings":
            event.update(
                shape=shape,
                down_rate_hz=load_toml(ROOT / "configs/render.toml")["capture"][
                    "down_hz"
                ],
            )
    data.events.insert(0, (0.0, {"kind": "system_run", "planned_flights": len(SHAPES)}))
    return evaluate(data)


def exercise(recording_started):
    with isolated_run(ROOT), ExitStack() as stack:
        prefix = Path(
            stack.enter_context(
                tempfile.TemporaryDirectory(dir=ROOT / "target", prefix="sil-")
            )
        )
        (prefix / "bin").mkdir()
        (prefix / "configs").symlink_to(ROOT / "configs", target_is_directory=True)
        for name in ("firefly-daemon", "ffctl"):
            shutil.copy2(RELEASE / name, prefix / "bin" / name)

        def start(*args):
            child = ManagedProcess(args, cwd=ROOT, env=os.environ.copy())
            stack.callback(child.stop)
            return child

        def cli(*args, check=True):
            p = subprocess.run(
                [prefix / "bin/ffctl", "mission", *args],
                capture_output=True,
                text=True,
                timeout=8,
                check=False,
            )
            if check and p.returncode:
                raise RuntimeError(f"ffctl {args}: {p.stderr}")
            return json.loads(p.stdout) if not p.returncode else None

        daemon = start(prefix / "bin/firefly-daemon")

        def wait(read, predicate, seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if daemon.process.poll() is not None:
                    raise RuntimeError("daemon exited: " + " | ".join(daemon.tail))
                value = read()
                if value is not None and predicate(value):
                    return value
                time.sleep(0.25)
            raise TimeoutError(f"last response: {value}")

        cold = wait(lambda: cli("status", check=False), lambda _: True, 10)
        assert not cold["ready"]
        assert (
            cli("submit", "square", ROOT / "configs/mission-square.toml", check=False)
            is None
        )
        assert (
            subprocess.run(
                [prefix / "bin/firefly-daemon"],
                capture_output=True,
                timeout=5,
                check=False,
            ).returncode
            != 0
        )
        path = start(RELEASE / "mimir").wait_recording()
        recording_started(path)
        print(path, flush=True)
        start(
            sys.executable,
            "-c",
            "from firefly_sim.main import main; main(['--no-trace', '--config', 'configs/sim-mission.toml'], publish_plant_state=False, seed=42)",
        )
        start(RELEASE / "render")
        start(RELEASE / "firefly-rootslam", "--no-loop-closures", "--no-mapping")
        start(RELEASE / "fc")
        start(RELEASE / "landing", "--simulation")
        start(RELEASE / "traj")
        start(RELEASE / "planner", "--config", ROOT / "configs/planner-cuboid-sil.toml")
        wait(lambda: cli("status"), lambda s: s["ready"], 60)
        time.sleep(4)
        session = None
        for shape in SHAPES:
            plan = ROOT / f"configs/mission-{shape}.toml"
            receipt = cli("submit", shape, plan)
            assert receipt["run"] == cold["run"]
            assert session is None or receipt["session"] == session
            session = receipt["session"]
            time.sleep(2)
            assert cli("submit", shape, plan)["session"] == session
            end = wait(
                lambda mission_id=shape: cli("status", mission_id),
                lambda s: s["phase"] in ("recover", "interrupted", "complete"),
                600,
            )
            print(json.dumps(end), flush=True)
            if end["phase"] != "complete":
                raise RuntimeError(str(end))
            assert end["flight_number"] == SHAPES.index(shape) + 1
            time.sleep(1)
        # A fresh daemon must still see the same run and both completed receipts.
        daemon.process.kill()
        daemon.process.wait(timeout=5)
        daemon = start(prefix / "bin/firefly-daemon")
        after = wait(
            lambda: cli("status", check=False), lambda s: s["active"] is None, 15
        )
        assert after["run"] == cold["run"] and after["session"] == session
        assert all(
            cli("submit", s, ROOT / f"configs/mission-{s}.toml")["phase"] == "complete"
            for s in SHAPES
        )
        # Kill the mission owner during hover. FC and VIO continue running.
        wait(lambda: cli("status"), lambda s: s["ready"], 30)
        hover = prefix / "hover.toml"
        hover.write_text("altitude=0.8\nhover_seconds=15\nprecision_land=false\n")
        cli("submit", "restart-test", hover)
        wait(lambda: cli("status", "restart-test"), lambda s: s["phase"] == "hover", 40)
        daemon.process.kill()
        daemon.process.wait(timeout=5)
        daemon = start(prefix / "bin/firefly-daemon")
        result = wait(
            lambda: cli("status", "restart-test", check=False),
            lambda s: s["phase"] in ("interrupted", "needs_operator"),
            60,
        )
        assert result["flight_number"] == 3 and result["session"] == session
        assert result["reason"] == "daemon restarted during mission"
        assert cli("submit", "restart-test", hover)["phase"] == result["phase"]
        if result["phase"] == "needs_operator":
            assert not cli("status")["ready"] and result["attention"]
            assert cli("submit", "another-launch", hover, check=False) is None
        print(json.dumps(result), flush=True)
    return path, result["phase"] == "interrupted"


def main():
    paths = []
    failure = None
    recovered = None
    if len(sys.argv) > 1:
        paths.append(Path(sys.argv[1]))
    else:
        try:
            _, recovered = exercise(paths.append)
        except (TimeoutError, RuntimeError, AssertionError) as error:
            if not paths:
                raise
            failure = f"{type(error).__name__}: {error}"
    result = grade(paths[0])
    if failure:
        result["failures"].append(failure)
        result["passed"] = False
    if recovered is not None:
        result["recovery_disarmed"] = recovered
        if not recovered:
            result["failures"].append(
                "Airborne daemon restart required operator intervention"
            )
            result["passed"] = False
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
