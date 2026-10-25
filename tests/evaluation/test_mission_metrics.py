"""任务评分独立解析反例：禁止地图误差对齐、跨数据断流评分或放行高速着陆接触。"""
import sys

import numpy as np
import pytest

from firefly_evaluation.mission_metrics import position_error, yaw_error

from firefly_evaluation.mission import Mission, Options
from firefly_evaluation.mission_io import Recorder
from unittest.mock import create_autospec
from firefly_evaluation.mission_sim import permitted_pad_contact
from firefly_mujoco import drone_pad


def test_command_keeps_observation_pump_running(tmp_path):
    binary = tmp_path / "target/release/ffctl"
    binary.parent.mkdir(parents=True)
    gate = tmp_path / "observation_received"
    binary.write_text(f"#!{sys.executable}\nimport pathlib,time\nwhile not pathlib.Path({str(gate)!r}).exists(): time.sleep(.01)\n")
    binary.chmod(0o755)
    pumps = []
    def pump():
        pumps.append(True)
        gate.touch()
    class PumpMission(Mission):
        def pump(self, guard=True):
            pump()

    subject = PumpMission(tmp_path, "nominal", Options())
    subject.root = tmp_path
    subject.recorder = create_autospec(Recorder, instance=True, spec_set=True)
    subject.command("fc", "hold")
    assert pumps and gate.exists()


def poses():
    time = np.arange(0., 3.01, .1)
    rows = np.zeros((len(time), 8))
    rows[:, 0] = time
    rows[:, 1] = time
    rows[:, 7] = 1.
    return rows


def test_map_tracking_error_preserves_translation_and_scale_error():
    gt = poses()
    estimate = gt.copy()
    estimate[:, 2] += .6
    result = position_error(gt, estimate, 0., 3.)
    assert result["rmse_m"] == pytest.approx(.6)
    assert result["max_m"] == pytest.approx(.6)
    assert result["coverage"] == pytest.approx(1.)


def test_map_yaw_detects_opposite_heading_and_wrap():
    gt, estimate = poses(), poses()
    estimate[:, 6:8] = [1., 0.]
    assert yaw_error(gt, estimate) == pytest.approx(180.)
    a, b = np.deg2rad(179.) / 2, np.deg2rad(-179.) / 2
    gt[:, 6:8] = [np.sin(a), np.cos(a)]
    estimate[:, 6:8] = [np.sin(b), np.cos(b)]
    assert yaw_error(gt, estimate) == pytest.approx(2.)


def test_missing_evidence_cannot_pass_by_extrapolating():
    gt = poses()
    with pytest.raises(ValueError, match="coverage"):
        position_error(gt, gt[:15], 0., 3.)
    missing = np.concatenate([gt[:5], gt[20:]])
    with pytest.raises(ValueError, match="gap"):
        position_error(gt, missing, 0., 3.)
    with pytest.raises(ValueError, match="gap"):
        position_error(missing, gt, 0., 3.)


def test_contact_exception_only_covers_low_speed_pad_touchdown():
    pad = np.asarray(drone_pad())
    normal = [0., 0., 1.]
    assert permitted_pad_contact(pad, np.zeros(3), normal)
    assert not permitted_pad_contact(pad, np.array([0., 0., -2.]), normal)
    assert not permitted_pad_contact(pad + [0., 0., 1.], np.zeros(3), normal)
    assert not permitted_pad_contact(pad + [1., 0., 0.], np.zeros(3), normal)
    assert not permitted_pad_contact(pad, np.zeros(3), [1., 0., 0.])


def test_invalid_thresholds_and_missing_mission_are_rejected():
    Options().validate()
    Options(waypoints=((9., 0., 2.),), finish_action="hold").validate()
    with pytest.raises(ValueError):
        Options(vio_ate_rmse_m=float("nan")).validate()
    with pytest.raises(ValueError):
        Options(waypoints=[]).validate()
    with pytest.raises(ValueError):
        Options(finish_action="unknown").validate()


def test_terminal_hold_requires_independent_truth_accuracy():
    from firefly_evaluation import mission_metrics
    truth = poses()
    truth[:, 1:4] = [8.4, 0., 2.]
    velocity = np.column_stack([truth[:, 0], np.zeros((len(truth), 3))])
    evidence = {"gt": truth, "gt_velocity": velocity}
    case = {"case": "nominal", "recording": "unused", "stages": {
        "tracking": {"status": "blocked"},
        "terminal_hold": {"status": "passed", "end_sim_s": 3.}}}
    mission_metrics.score(case, Options(waypoints=((9., 0., 2.),), finish_action="hold"),
                          evidence_reader=lambda _: evidence)
    assert case["stages"]["terminal_hold_accuracy"]["status"] == "failed"
    assert case["stages"]["terminal_hold_accuracy"]["metrics"]["max_position_error_m"] == pytest.approx(.6)


@pytest.mark.parametrize("contents", ["altitude_m = [", "altitude_m = -1", "unknown = 1"],
                         ids=["invalid_toml", "invalid_threshold", "unknown_option"])
def test_preflight_failure_returns_nonzero_and_keeps_report(tmp_path, contents):
    import json
    from firefly_evaluation import cli
    config = tmp_path / "invalid.toml"
    config.write_text(contents)
    resource_calls = []
    assert cli.main(["--config", str(config)], resolve_workspace=lambda _: tmp_path,
                    check_idle=lambda _: resource_calls.append("check_idle"),
                    collect_provenance=lambda _: resource_calls.append("provenance")) == 1
    assert resource_calls == []
    reports = list((tmp_path / "logs/acceptance").glob("*/report.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text())
    assert report["status"] == "failed" and report["cases"] == []
    assert report["error"] and reports[0].with_suffix(".html").is_file()


def test_repeated_failures_keep_four_distinct_recordings(tmp_path):
    import json
    from firefly_evaluation import cli

    class FailedMission:
        def __init__(self, directory, name, options, *, workspace):
            self.recording = directory / f"{name}.rrd"
            self.name = name

        def run(self):
            self.recording.write_bytes(b"unit test evidence placeholder")
            return {"case": self.name, "status": "failed", "recording": str(self.recording),
                    "remaining_pids": [], "stages": {"tracking": {"status": "failed"}}}

    config = tmp_path / "acceptance.toml"
    config.write_text("")
    assert cli.main(["--config", str(config), "--case", "nominal", "--repeat", "4"],
                    resolve_workspace=lambda _: tmp_path, check_idle=lambda _: None,
                    collect_provenance=lambda _: {}, mission_factory=FailedMission,
                    score_case=lambda case, options: case) == 1
    report_path, = (tmp_path / "logs/acceptance").glob("*/report.json")
    report = json.loads(report_path.read_text())
    assert report["status"] == "failed"
    assert [case["attempt"] for case in report["cases"]] == [1, 2, 3, 4]
    assert len({case["recording"] for case in report["cases"]}) == 4
    html = report_path.with_suffix(".html").read_text()
    for attempt in range(1, 5):
        relative = f"attempt_{attempt:02d}/nominal.rrd"
        assert (report_path.parent / relative).is_file()
        assert f"href='{relative}'" in html


def test_roundtrip_distance_summary_does_not_override_failed_acceptance():
    from firefly_evaluation import mission_metrics
    truth = np.array([[0., -13., 0., 2.2, 0., 0., 0., 1.],
                      [10., -3., 0., 2.2, 0., 0., 0., 1.],
                      [20., -13., 0., 2.2, 0., 0., 0., 1.]])
    evidence = {"gt": truth, "loop_backend": np.array([[20., 0.]])}
    case = {"case": "nominal", "recording": "unused", "waypoint_arrivals": [{}, {}],
            "stages": {"tracking": {"status": "blocked"}}}
    mission_metrics.score(case, Options(), evidence_reader=lambda _: evidence)
    summary = case["mission_summary"]
    assert summary["max_horizontal_distance_from_start_m"] == pytest.approx(10.)
    assert summary["horizontal_path_length_m"] == pytest.approx(20.)
    assert summary["last_horizontal_distance_from_start_m"] == pytest.approx(0.)
    assert summary["estimated_waypoints_completed"]
    assert not summary["all_waypoints_reached"]
    assert summary["committed_full_graph_updates"] == 0
    assert summary["loop_evidence"] == "no_committed_update_observed"
    assert case["status"] == "failed"


def test_estimated_arrival_does_not_require_truth_and_false_arrival_fails_scoring():
    import time
    from firefly_evaluation.mission import Mission
    from firefly_evaluation.mission_metrics import settled_truth
    mission = Mission.__new__(Mission)
    mission.t = 3.
    goal = np.array([1., 0., 2.])
    mission.latest = {"corrected": (3., goal.copy(), np.zeros(3), None, True, time.monotonic())}
    assert mission.estimated_near("corrected", goal, .35, .3)
    gt = poses()
    gt[:, 1:4] = goal + [1., 0., 0.]
    velocity = np.column_stack([gt[:, 0], np.zeros((len(gt), 3))])
    passed, result = settled_truth({"gt": gt, "gt_velocity": velocity}, 3., 1., goal, .35, .3)
    assert not passed and result["max_position_error_m"] == pytest.approx(1.)
    gt[:, 1:4] = goal
    assert settled_truth({"gt": gt, "gt_velocity": velocity}, 3., 1., goal, .35, .3)[0]
    with pytest.raises(ValueError, match="incomplete"):
        settled_truth({"gt": gt[:-5], "gt_velocity": velocity}, 3., 1., goal, .35, .3)
    mission.t = 3.5
    assert not mission.estimated_near("corrected", goal, .35, .3)


def test_vio_only_map_stage_starts_planner_and_output_and_labels_fixed_prior(tmp_path):
    from firefly_evaluation.mission import Mission
    launched = []

    class MapMission(Mission):
        def start(self, name, command):
            launched.append((name, command))

        def wait(self, predicate, **kwargs):
            assert predicate(), "not ready"

    mission = MapMission(tmp_path, "nominal", Options(), vio_only=True)
    mission.latest["corrected"] = (2., None, None, None, True)
    mission.latest["reference"] = None
    result = mission.map_ready()
    assert [name for name, _ in launched] == ["traj-server", "planner"]
    assert "--traj-server" in launched[0][1]
    assert "--vio-only" not in launched[1][1]
    assert result["accepted_visual_updates_at_start"] == 0
    assert result["visual_frontend"] == "disabled"
    assert mission.result["localization_mode"] == "vio_only"


def test_default_map_stage_uses_running_visual_frontend(tmp_path):
    from firefly_evaluation.mission import Mission
    launched = []

    class MapMission(Mission):
        def start(self, name, command):
            launched.append((name, command))

        def wait(self, predicate, **kwargs):
            assert predicate(), "not ready"

    mission = MapMission(tmp_path, "nominal", Options())
    mission.latest["corrected"] = (2., None, None, None, True)
    mission.latest["reference"] = None
    mission.visual_updates = 0
    result = mission.map_ready()
    assert [name for name, _ in launched] == ["traj-server", "planner"]
    assert all("--vio-only" not in command and "--map" not in command for _, command in launched)
    assert result["accepted_visual_updates_at_start"] == 0
    assert result["visual_frontend"] == "enabled"


@pytest.mark.parametrize(("vio_only", "prior_maps"), [(False, False), (False, True), (True, False)])
def test_online_frontend_runs_during_takeoff_and_initial_hover(tmp_path, vio_only, prior_maps):
    from firefly_evaluation.mission import Mission
    class PhaseMission(Mission):
        def run_phases(self, phases, applications):
            assert [name for name, _ in phases][:3] == ["initialization", "takeoff", "hover"]
            assert "firefly-okvis-x" in applications
            assert "slam" not in applications
            assert ("localization" in applications) == prior_maps
            assert ("aliked" in applications) == prior_maps
            assert ("lightglue" in applications) == prior_maps
            return self.result

    PhaseMission(tmp_path, "nominal", Options(), vio_only=vio_only, prior_maps=prior_maps).run()


@pytest.mark.parametrize("observations,passed", [(0, False), (1, False), (2, True)])
def test_visual_evidence_is_scored_independently_of_navigation(observations, passed):
    from firefly_evaluation import mission_metrics
    evidence = {"gt": poses(), "visual_update": np.array(
        [[float(i + 1), 0., 3., .1, 1.] for i in range(observations)]).reshape(-1, 5)}
    if not observations:
        evidence.pop("visual_update")
    case = {"case": "nominal", "localization_mode": "visual", "recording": "unused",
            "stages": {"tracking": {"status": "blocked"}}}
    mission_metrics.score(case, Options(), evidence_reader=lambda _: evidence)
    assert (case["stages"]["visual_localization"]["status"] == "passed") == passed
    assert case["stages"]["visual_localization"]["metrics"]["accepted_map_observations"] == observations
    case["localization_mode"] = "vio_only"
    case["stages"] = {"tracking": {"status": "blocked"}}
    mission_metrics.score(case, Options(), evidence_reader=lambda _: evidence)
    assert "visual_localization" not in case["stages"]
    assert case["mission_summary"]["loop_evidence"] == "disabled_vio_only"


def test_session_scoring_uses_only_initial_truth_gauge_and_preserves_drift():
    from firefly_evaluation.mission_metrics import startup_truth
    truth = poses()
    truth[:, 1:4] = np.column_stack((np.full(len(truth), 10.), truth[:, 0] + 20., np.full(len(truth), 3.)))
    truth[:, 6:8] = np.sqrt(.5)
    original = truth.copy()
    data, frame = startup_truth({"gt": truth})
    expected = poses()
    assert np.allclose(data["gt"], expected)
    assert np.array_equal(truth, original)
    drifted = expected.copy()
    drifted[:, 2] += .6
    assert position_error(data["gt"], drifted, 0., 3.)["rmse_m"] == pytest.approx(.6)
    assert not frame["trajectory_fitting"] and not frame["feedback_to_algorithms"]


def test_online_slam_requires_loop_constraint_even_without_prior_map():
    from firefly_evaluation import mission_metrics
    evidence = {"gt": poses()}
    case = {"case": "nominal", "localization_mode": "online_slam", "recording": "unused",
            "stages": {"tracking": {"status": "blocked"}}}
    mission_metrics.score(case, Options(), evidence_reader=lambda _: evidence)
    assert case["stages"]["online_loop"]["status"] == "failed"
    assert "visual_localization" not in case["stages"]


@pytest.mark.parametrize("updates, expected", [(0., "failed"), (1., "passed"), (3., "passed")])
def test_online_loop_requires_a_committed_full_graph_update(updates, expected):
    from firefly_evaluation import mission_metrics
    case = {"case": "nominal", "localization_mode": "online_slam", "recording": "unused",
            "waypoint_arrivals": [], "stages": {"tracking": {"status": "blocked"}}}
    evidence = {"loop_backend": np.array([[1., 0.], [2., updates]])}
    mission_metrics.score(case, Options(), evidence_reader=lambda _: evidence)
    assert case["stages"]["online_loop"]["status"] == expected
    assert case["stages"]["online_loop"]["metrics"]["committed_full_graph_updates"] == updates
    # A loop commit alone cannot pass route, landing and lifecycle acceptance.
    assert case["status"] == "failed"
