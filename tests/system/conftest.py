"""真实飞行测试的运行条件、证据目录和进程退出契约。"""
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session", autouse=True)
def build_release_apps(request):
    """Build release apps when the system-test suite is explicitly selected."""
    suites = {item.path.name for item in request.session.items}
    if suites <= {"test_mimir.py", "test_matrix_pilot.py"}:
        packages = ["-p", "pilot"] if "test_matrix_pilot.py" in suites else []
        subprocess.run(["cargo", "build", "--release", "-p", "mimir", *packages], cwd=ROOT, check=True)
        return

    subprocess.run(
        ["cargo", "xtask", "release"],
        cwd=ROOT,
        check=True,
    )
