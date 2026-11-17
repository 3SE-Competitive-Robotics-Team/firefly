"""Real planner/traj/VIO/INT8 flight and independent RRD scoring; opt-in GPU test."""
import json
import os
from pathlib import Path
import time

import pytest
from firefly_workspace.process import ManagedProcess


ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(
    os.environ.get("FIREFLY_RUN_PRECISION_MISSION") != "1",
    reason="requires idle host, native GPU and quantized landing model",
)


def test_roundtrip_reacquires_pad_and_physically_lands():
    reports = []

    def observe(line):
        if line.startswith('{"commands_completed":'):
            reports.append(json.loads(line))

    process = ManagedProcess(
        [str(ROOT / ".venv/bin/python"), "scripts/flight.py", "--mission", "roundtrip", "--precision-land"],
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
    assert Path(report["recording"]).is_file()
