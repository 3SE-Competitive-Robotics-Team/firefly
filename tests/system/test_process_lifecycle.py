"""进程所有权、退出回收和验收互斥；无需 IPC 或场景资产。"""

import os
import signal
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest

from process_lifecycle import ManagedProcess, active_processes, isolated_run


def test_stop_reaps_child_and_output_thread(tmp_path):
    script = tmp_path / "firefly-sim"
    script.write_text("""import signal, sys
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
print('ready', flush=True)
signal.pause()
""")
    ready = threading.Event()
    child = ManagedProcess([sys.executable, str(script)], cwd=tmp_path, env=dict(os.environ),
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


def test_mission_cleans_remaining_children_after_one_stop_times_out(tmp_path):
    from mission import Mission, Options

    stopped = []

    class Child:
        def __init__(self, name, pid):
            self.name, self.pid = name, pid
            self.process = SimpleNamespace(poll=lambda: None if name == "extra" else 0)

        def stop(self):
            stopped.append(self.name)
            if self.name == "extra":
                raise subprocess.TimeoutExpired(self.name, 15.)
            return 0

    mission = Mission(tmp_path, "nominal", Options())
    mission.processes = {name: Child(name, pid) for pid, name in
                         enumerate(("viz", "render", "extra", "sim"))}
    mission.shutdown()
    assert stopped == ["sim", "extra", "render", "viz"]
    assert mission.result["stages"]["shutdown"]["status"] == "failed"
    assert mission.result["remaining_pids"] == [2]
