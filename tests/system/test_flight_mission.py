"""Real planner/traj/VIO/INT8 flight and independent RRD scoring; opt-in GPU test."""
import json
import os
from pathlib import Path
import time

import pytest
from firefly_workspace.process import ManagedProcess


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("route, points", [
    ("square", 5),
    ("triangle", 4),
    ("cuboid", 16),
])
def test_roundtrip_reacquires_pad_and_physically_lands(route, points):
    reports = []

    def observe(line):
        if line.startswith("{") and '"commands_completed":' in line:
            reports.append(json.loads(line))

    process = ManagedProcess(
        ["cargo", "xtask", "sil", f"configs/mission/{route}.toml"],
        cwd=ROOT, env={**os.environ, "RUST_LOG": "info"}, on_line=observe,
    )
    try:
        deadline = time.monotonic() + 300.
        while process.process.poll() is None and time.monotonic() < deadline:
            time.sleep(.1)
        assert process.process.poll() is not None, "mission exceeded the system-test deadline"
    finally:
        code = process.stop(45.)
    assert code == 0, "\n".join(process.tail)
    assert len(reports) == 1
    report = reports[0]
    assert report["commands_completed"] and not report["cleanup_errors"], report
    assert report["verification"]["passed"], report
    assert report["verification"]["metrics"]["down_frames"] > 30
    assert report["verification"]["metrics"]["max_down_gap_s"] <= .5
    assert report["verification"]["metrics"]["down_fps"] >= 10.
    assert len(report["verification"]["metrics"]["waypoints"]) == points
    if route == "square":
        visits = report["verification"]["metrics"]["waypoints"]
        assert [v["arrival"] for v in visits] == ["pass"] * 4 + ["stop"]
        assert visits[0]["pass_speed_mps"] > 0.25
        assert visits[-1]["hover_seconds"] >= .95
    from firefly_workspace.recording import recording_files
    from rerun.chunk import RrdReader
    with recording_files(report["recording"]) as files:
        for file in files:
            reader = RrdReader(file)
            [blueprint] = reader.blueprints()
            assert any("ViewBlueprint:space_origin" in (batch := chunk.to_record_batch()).schema.names
                       and ["/cameras/down"] in batch.column("ViewBlueprint:space_origin").to_pylist()
                       for chunk in reader.stream(store=blueprint)), "embedded blueprint must expose the down camera"
