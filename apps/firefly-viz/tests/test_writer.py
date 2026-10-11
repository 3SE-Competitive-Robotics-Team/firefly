"""写入线程的空闲等待、排序与退出排空契约。"""

import queue
from types import SimpleNamespace

import pytest

from firefly_viz import main as viz


def test_idle_writer_waits_for_each_flush_period(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(viz, "time", SimpleNamespace(monotonic=lambda: clock.now))

    class IdleQueue:
        calls = 0

        def get(self, *, timeout):
            if timeout != pytest.approx(viz.FLUSH_PERIOD):
                pytest.fail(f"idle writer must block for a full period, got {timeout}")
            self.calls += 1
            if self.calls == 4:
                return None
            clock.now += timeout
            raise queue.Empty

    viz._writer_loop(IdleQueue())


def test_shutdown_flushes_pending_messages_in_stable_timestamp_order(monkeypatch):
    monkeypatch.setattr(viz, "time", SimpleNamespace(monotonic=lambda: 0.0))
    written = []
    monkeypatch.setattr(viz, "_handle", lambda msg, trace: written.append((msg, trace)))
    monkeypatch.setattr(viz, "_log_text", lambda msg: written.append(msg))
    inbox = queue.Queue()
    for item in [("viz", 2., ("pose", "trace")), ("log", 1., "first"),
                 ("log", 1., "second"), None]:
        inbox.put(item)
    viz._writer_loop(inbox)
    assert written == ["first", "second", ("pose", "trace")]


def test_writer_shutdown_timeout_is_reported():
    writer = SimpleNamespace(join=lambda **kwargs: None, is_alive=lambda: True)
    inbox = queue.Queue()
    with pytest.raises(RuntimeError, match="did not finish draining"):
        viz._stop_writer(inbox, writer)
    assert inbox.get_nowait() is None
