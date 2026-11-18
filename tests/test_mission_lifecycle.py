"""Mission phase, resource cleanup and bounded arrival/vision gates."""

from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from flight_mission.config import Phase, parse_args
from flight_mission.gates import Arrival, VisualAcquisition
from flight_mission.runtime import FlightRunner
from firefly_ipc.messages import FcStatusMessage, FlightMode, LandingTargetMessage, OdomMessage


def test_options_are_immutable_and_route_offsets_own_their_values():
    options = parse_args(["--waypoint", "2", "0", "--waypoint", "3", "1"])
    assert options.waypoint == ((2.0, 0.0), (3.0, 1.0))
    with pytest.raises(FrozenInstanceError):
        options.altitude = 1.0


def test_phase_sequence_covers_waypoints_and_rejects_invalid_reentry():
    phase = Phase.STARTUP
    for following in (
        Phase.TAKEOFF,
        Phase.WAYPOINT,
        Phase.WAYPOINT_HOVER,
        Phase.WAYPOINT,
        Phase.WAYPOINT_HOVER,
        Phase.RETURN,
        Phase.ACQUIRE,
        Phase.LAND,
        Phase.COMPLETE,
    ):
        phase = phase.advance(following)
    for following in (Phase.TAKEOFF, Phase.LAND, Phase.COMPLETE):
        with pytest.raises(ValueError):
            phase.advance(following)
    with pytest.raises(ValueError):
        Phase.WAYPOINT.advance(Phase.RETURN)


def test_arrival_identity_and_motion_restart_the_settling_window():
    arrival = Arrival((2.0, 0.0, 1.0), 7.0)
    pose = OdomMessage(position_x=2.0, position_z=1.0, is_initialized=True)
    planner = SimpleNamespace(goal_timestamp=7.0, timestamp=1.0, finished=True)
    assert not arrival.observe(planner, pose, 1.0)
    planner.timestamp = 1.6
    planner.goal_timestamp = 8.0
    assert not arrival.observe(planner, pose, 1.6)
    planner.goal_timestamp = 7.0
    assert not arrival.observe(planner, pose, 1.6)
    pose.velocity_x = 0.5
    planner.timestamp = 1.9
    assert not arrival.observe(planner, pose, 1.9)
    pose.velocity_x = 0.0
    planner.timestamp = 2.0
    assert not arrival.observe(planner, pose, 2.0)
    planner.timestamp = 2.5
    assert arrival.observe(planner, pose, 2.5)


def test_visual_confirmation_deduplicates_and_recovers_after_stale_input():
    acquisition = VisualAcquisition(after=1.0, session=7, team=0)

    def target(t):
        return LandingTargetMessage(
            timestamp=t, session=7, team=0, valid=1, confidence=0.9, reprojection_px=0.2
        )

    for _ in range(100):
        assert not acquisition.observe(target(1.1), 1.1, 3.0, 3.0)
    assert not acquisition.observe(target(1.2), 1.2, 3.1, 3.1)
    assert not acquisition.observe(target(1.3), 1.3, 3.1, 3.5)
    assert acquisition.target_time is None
    assert not acquisition.observe(target(1.4), 1.4, 4.0, 4.0)
    assert not acquisition.observe(target(1.5), 1.5, 4.1, 4.1)
    assert acquisition.observe(target(1.55), 1.55, 4.2, 4.2)
    assert acquisition.target_time == 1.55
    assert not acquisition.observe(target(1.54), 1.55, 4.3, 4.3)
    assert acquisition.target_time is None


def test_mode_wait_uses_status_arrival_time(monkeypatch):
    from ctypes import pointer

    runner = FlightRunner(parse_args([]), ROOT, "red")
    now = 10.
    monkeypatch.setattr("flight_mission.runtime.time.monotonic", lambda: now)
    monkeypatch.setattr("flight_mission.runtime.time.sleep", lambda _: None)
    sample = SimpleNamespace(payload=lambda: pointer(FcStatusMessage(4., 0, FlightMode.HOLD)))
    samples = iter([sample])
    runner.subs["mode"] = SimpleNamespace(receive=lambda: next(samples, None))
    runner.received.update(odom=now, attitude=now)
    runner._wait(lambda: True, modes={FlightMode.HOLD})
    assert runner.mode is FlightMode.HOLD
    now += 1.1
    runner.received.update(odom=now, attitude=now)
    with pytest.raises(RuntimeError, match="stale mode"):
        runner._wait(lambda: True, modes={FlightMode.HOLD})


def test_runner_starts_the_requested_command_and_cleans_every_child(monkeypatch):
    commands, stopped = [], []

    class Child:
        def __init__(self, command, **kwargs):
            self.command = command
            commands.append(command)

        def stop(self):
            stopped.append(commands.index(self.command))
            return 0

    monkeypatch.setattr("flight_mission.runtime.ManagedProcess", Child)
    runner = FlightRunner(parse_args([]), ROOT, "red")
    runner._start("first", ["one"])
    runner._start("second", ["two"])
    assert runner._close(None) == []
    assert commands == [["one"], ["two"]] and stopped == [1, 0]


def test_cleanup_reports_a_failure_and_still_releases_other_children():
    released = []

    class Child:
        def __init__(self, name):
            self.name = name
            self.process = SimpleNamespace(poll=lambda: 1)

        def stop(self):
            released.append(self.name)
            if self.name == "second":
                raise RuntimeError("close failed")
            return 0

    runner = FlightRunner(parse_args([]), ROOT, "red")
    runner.processes = {name: Child(name) for name in ("first", "second")}
    assert runner._close(None) == ["second: close failed"]
    assert released == ["second", "first"]


def test_cleanup_releases_children_after_lease_and_forced_shutdown_failures():
    released = []

    def fail():
        raise RuntimeError("unavailable")

    runner = FlightRunner(parse_args([]), ROOT, "red")
    runner.activity = SimpleNamespace(set=lambda *_: fail())
    runner.processes = {
        "first": SimpleNamespace(stop=lambda: released.append("first") or 0),
        "second": SimpleNamespace(stop=fail, process=SimpleNamespace(poll=lambda: None, kill=fail)),
    }
    assert runner._close(None) == [
        "inference lease: unavailable",
        "second: unavailable",
        "second forced shutdown: unavailable",
    ]
    assert released == ["first"]


def test_cancelled_run_reports_failure_and_closes_resources(monkeypatch, capsys):
    from contextlib import nullcontext
    import json

    runner = FlightRunner(parse_args([]), ROOT, "red")
    released = []

    def cancel():
        raise KeyboardInterrupt

    def unavailable(*_, **__):
        raise RuntimeError("recording unavailable")

    monkeypatch.setattr("flight_mission.runtime.isolated_run", lambda _: nullcontext())
    monkeypatch.setattr(runner, "_startup", lambda: None)
    monkeypatch.setattr(runner, "_execute", cancel)
    monkeypatch.setattr(runner, "_mission_event", unavailable)
    runner.processes = {"child": SimpleNamespace(stop=lambda: released.append(True) or 0)}
    assert runner.run() == 1
    report = json.loads(capsys.readouterr().out)
    assert report["error"] == "KeyboardInterrupt; failure recording: recording unavailable"
    assert not report["commands_completed"] and report["cleanup_errors"] == []
    assert released == [True]
