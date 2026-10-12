"""真实飞行测试的运行条件、证据目录和进程退出契约。"""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT / "scripts", ROOT / "tests/evaluation"):
    sys.path.insert(0, str(directory))

from process_lifecycle import require_idle


@pytest.fixture(scope="session", autouse=True)
def build_release_apps():
    """Build every Rust app once before system tests can launch release binaries."""
    subprocess.run(
        ["cargo", "xtask", "build-all-release"],
        cwd=ROOT,
        check=True,
    )


@pytest.fixture
def flight_environment(request):
    """显式选择文件或节点才启动飞行；全仓收集不要求图形环境。"""
    explicitly_selected = any(
        (request.config.invocation_params.dir / arg.split("::", 1)[0]).resolve() == request.path.resolve()
        for arg in request.config.args
    )
    enabled = (request.path.name == "test_imu_flight.py"
               and os.environ.get("FIREFLY_RUN_IMU_FLIGHT") == "1")
    if not explicitly_selected and not enabled:
        pytest.skip(f"run uv run pytest {request.path.relative_to(ROOT)}")
    require_idle(ROOT)
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        pytest.fail("flight test requires a graphical session (DISPLAY or WAYLAND_DISPLAY)")
    missing = [name for name in ("render", "vio", "fc", "ffctl")
               if not (ROOT / f"target/release/{name}").is_file()]
    if missing:
        pytest.fail(f"missing release binaries: {missing}; run cargo build --release -p render -p vio -p fc -p ffctl")
    for asset in ("field.glb", "rmuc2026_collision.json"):
        if not (ROOT / "models/rmuc2026" / asset).is_file():
            pytest.fail(f"missing RMUC asset: {asset}; see docs/development.md#run")
    try:
        yield
    finally:
        require_idle(ROOT)


@pytest.fixture
def flight_directory(request, flight_environment):
    """每次用例独占的 logs 路径；由报告写入方创建目录。"""
    name = request.node.name.replace("[", "_").replace("]", "")
    return ROOT / "logs" / f"{name}_{datetime.now(timezone.utc):%Y%m%dT%H%M%S%fZ}"


@pytest.fixture
def flight_report(flight_directory):
    """运行前冻结指纹，退出后保存评分并使失败成为 pytest 失败。"""
    names = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=ROOT,
    ).decode().split("\0")
    files = {ROOT / name for name in names if name and (ROOT / name).is_file()}
    files.update(ROOT / f"target/release/{name}" for name in ("render", "vio", "fc", "ffctl"))
    files.update(ROOT / "models/rmuc2026" / name for name in ("field.glb", "rmuc2026_collision.json"))
    hashes = {}
    for path in sorted(files):
        with path.open("rb") as stream:
            hashes[str(path.relative_to(ROOT))] = hashlib.file_digest(stream, "sha256").hexdigest()
    provenance = {
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "working_tree_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "diff", "HEAD"], cwd=ROOT)).hexdigest(),
        "files_sha256": hashes,
    }

    def finish(result, options, **metadata):
        result["provenance"] = {**provenance, "options": asdict(options), **metadata}
        path = flight_directory / "report.json"
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        if result.get("interrupted"):
            raise KeyboardInterrupt
        assert result["status"] == "passed", f"see {path}"

    return finish
