"""真实飞行测试的运行条件、证据目录和进程退出契约。"""
import os
from pathlib import Path
import subprocess
import sys

import pytest

from firefly_evaluation.process_lifecycle import require_idle

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session", autouse=True)
def build_release_apps(request):
    """Build release apps when the system-test suite is explicitly selected."""
    system_tests = (ROOT / "tests/system").resolve()
    invocation_dir = Path(request.config.invocation_params.dir)
    selected_system_tests = any(
        (invocation_dir / str(arg).split("::", 1)[0]).resolve() == system_tests
        or system_tests in (invocation_dir / str(arg).split("::", 1)[0]).resolve().parents
        for arg in request.config.args
        if not str(arg).startswith("-")
    )
    if not selected_system_tests:
        return

    if all(item.path.name == "test_mimir.py" for item in request.session.items):
        subprocess.run(["cargo", "build", "--release", "-p", "mimir"], cwd=ROOT, check=True)
        return

    subprocess.run(
        ["cargo", "xtask", "release"],
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
    # macOS 的图形会话不导出 DISPLAY/WAYLAND_DISPLAY；渲染本身无窗口。
    graphical_session = (sys.platform == "darwin"
                         or os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not graphical_session:
        pytest.fail("flight test requires a graphical session (DISPLAY or WAYLAND_DISPLAY)")
    vio_process = "firefly-rootslam"
    missing = [name for name in ("render", vio_process, "fc", "ffctl")
               if not (ROOT / f"target/release/{name}").is_file()]
    if missing:
        pytest.fail(f"missing release binaries: {missing}; run cargo xtask release")
    from firefly_mujoco import load_scene_name, resolve_scene

    scene = resolve_scene(load_scene_name())
    for asset in (scene.visual, scene.collision):
        if not asset.is_file():
            pytest.fail(f"missing RMUC asset: {asset}; see docs/development.md#run")
    try:
        yield
    finally:
        require_idle(ROOT)
