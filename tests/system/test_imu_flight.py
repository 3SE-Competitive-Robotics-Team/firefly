"""独立姿态链路的 release 起降与视觉失联验收；原始证据只写 RRD。

FIREFLY_RUN_IMU_FLIGHT=1 uv run --all-packages --extra test pytest tests/system/test_imu_flight.py -q
需要图形会话、RMUC 资产和 release vio/fc/render/ffctl；运行前须停止其他闭环进程。
"""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
import subprocess

import pytest

from mission import Mission, Options, ROOT

pytestmark = pytest.mark.skipif(
    os.environ.get("FIREFLY_RUN_IMU_FLIGHT") != "1",
    reason="requires FIREFLY_RUN_IMU_FLIGHT=1, RMUC assets and graphics session",
)


@pytest.mark.parametrize("case", ["imu_landing", "imu_vio_loss"])
def test_imu_flight(case):
    directory = ROOT / "logs" / f"{case}_{datetime.now(timezone.utc):%Y%m%dT%H%M%S%fZ}"
    directory.mkdir(parents=True)
    options = Options()
    options.validate()
    mission = Mission(directory, case, options)
    phases = [("initialization", mission.initialized), ("takeoff", mission.takeoff), ("hover", mission.hover)]
    phases.extend([("landing", mission.landing)] if case == "imu_landing" else [
        ("estimator_fallback", mission.estimator_fallback), ("degraded_support", mission.degraded_support),
    ])
    files = [ROOT / f"target/release/{name}" for name in ("render", "vio", "fc", "ffctl")]
    files.extend(sorted((ROOT / "configs").glob("*.toml")))
    for folder in ("apps/fc", "apps/vio", "crates/firefly-imu", "crates/firefly-pubsub", "crates/firefly-vio"):
        files.extend(sorted((ROOT / folder).rglob("*.rs")))
    provenance = {
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "working_tree_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "diff", "HEAD"], cwd=ROOT)).hexdigest(),
        "files_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        "options": asdict(options),
    }
    result = mission.run_phases(phases, ("render", "vio", "fc"))
    result["provenance"] = provenance
    if case == "imu_vio_loss":
        result["capability_limit"] = "Attitude support only; no guaranteed position hold, altitude hold or landing."
    (directory / "report.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    if result.get("interrupted"):
        raise KeyboardInterrupt
    assert result["status"] == "passed", f"see {directory / 'report.json'}"
