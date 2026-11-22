"""使用本地子进程验证所有权、退出回收和运行互斥。"""

import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading

import pytest

from firefly_workspace.process import ManagedProcess, active_processes, isolated_run


@pytest.mark.parametrize("entry", ["firefly-sim", "mimir",
                                  "firefly_sim.main"])
def test_stop_reaps_child_and_output_thread(tmp_path, entry):
    if entry.startswith("firefly_sim."):
        package = tmp_path / "firefly_sim"
        package.mkdir()
        (package / "__init__.py").write_text("")
        script = package / (entry.rsplit(".", 1)[1] + ".py")
        command = [sys.executable, "-m", entry]
    else:
        script = tmp_path / entry
        command = [sys.executable, str(script)]
    script.write_text("""import signal, sys
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
print('ready', flush=True)
signal.pause()
""")
    ready = threading.Event()
    child = ManagedProcess(command, cwd=tmp_path, env=dict(os.environ),
                           on_line=lambda line: ready.set())
    try:
        assert ready.wait(timeout=5.)
        assert child.pid in active_processes(tmp_path)
        with pytest.raises(RuntimeError, match="already running"):
            with isolated_run(tmp_path):
                pytest.fail("must refuse an occupied repository")
    finally:
        assert child.stop() == 0
    assert child.process.stdout.closed
    assert not child.reader.is_alive()
    assert child.pid not in active_processes(tmp_path)
    assert child.stop() == 0


def test_exclusive_run_releases_lock_on_failure(tmp_path):
    with pytest.raises(ValueError):
        with isolated_run(tmp_path):
            with pytest.raises(BlockingIOError):
                with isolated_run(tmp_path):
                    pytest.fail("must refuse overlapping runs")
            raise ValueError("failed run")
    with isolated_run(tmp_path):
        pass


def test_managed_run_captures_both_streams_and_preserves_child_identity(tmp_path):
    with isolated_run(tmp_path):
        command = [sys.executable, "-c", (
            "import os, sys\n"
            "print('recording: ' + os.environ['FIREFLY_RUN_DIR'])\n"
            "print('diagnostic', file=sys.stderr)\n"
        )]
        child = ManagedProcess(command, cwd=tmp_path, env=dict(os.environ))
        run = child.wait_recording()
        assert child.process.wait(timeout=10) == 0
        assert child.stop() == 0
    assert run.parent == tmp_path.resolve() / "logs/runs"
    assert json.loads((run / "manifest.json").read_text())["state"] == "closed"
    [text] = (run / "text").glob(f"{Path(sys.executable).name}.*.log")
    records = text.read_text().splitlines()
    assert any(f" {child.pid} stdout recording: {run}" in row for row in records)
    assert any(f" {child.pid} stderr diagnostic" in row for row in records)


def test_shutdown_timeout_does_not_report_success():
    class UnresponsiveProcess:
        pid = 123
        signals = []

        def poll(self):
            return None

        def send_signal(self, value):
            self.signals.append(value)

        def wait(self, *, timeout):
            raise subprocess.TimeoutExpired("child", timeout)

    child = ManagedProcess.__new__(ManagedProcess)
    child.process = UnresponsiveProcess()
    with pytest.raises(subprocess.TimeoutExpired):
        child.stop()
    assert child.process.signals == [signal.SIGINT]


@pytest.mark.parametrize("exit_code", [0, 7])
def test_output_callback_failure_drains_child_and_surfaces_after_cleanup(tmp_path, exit_code):
    callback_error = ValueError("recording unavailable")
    calls = []

    def record(line):
        calls.append(line)
        raise callback_error

    command = [sys.executable, "-c", (
        "import sys\n"
        "print('first', flush=True)\n"
        "for _ in range(256): print('x' * 8192)\n"
        "print('complete', flush=True)\n"
        f"sys.exit({exit_code})\n"
    )]
    child = ManagedProcess(command, cwd=tmp_path, env=dict(os.environ), on_line=record)
    try:
        child.process.wait(timeout=5.)
    finally:
        with pytest.raises(RuntimeError, match="output reader failed") as raised:
            child.stop()

    assert raised.value.__cause__ is callback_error
    assert child.process.returncode == exit_code
    assert child.process.stdout.closed
    assert not child.reader.is_alive()
    assert calls == ["first"]
    assert child.tail[-1] == "complete"
