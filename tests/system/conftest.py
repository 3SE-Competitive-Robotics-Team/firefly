"""真实飞行测试的运行条件、证据目录和进程退出契约。"""
from pathlib import Path
import subprocess

import pytest


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
