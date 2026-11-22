"""工作区进程的身份检查、输出消费与 SIGINT 收尾。"""

from collections import deque
from collections.abc import Callable, Sequence
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from queue import Full, Queue
import fcntl
import json
import os
import selectors
import signal
import subprocess
import threading
import time

from firefly_workspace import workspace_root


class _Archive:
    """One Rust archive writer serves all processes launched inside isolated_run."""

    def __init__(self, root):
        self.root = root
        self.path = None
        self.queue = Queue(maxsize=256)
        self.dropped = 0
        self.error = None

    def open(self, env):
        if self.path is None:
            self.worker = subprocess.Popen(
                ["cargo", "xtask", "archive-session", str(self.root.resolve())],
                cwd=workspace_root(Path(__file__)), env=env,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, encoding="utf-8",
            )
            with self.worker.stdout as stdout:
                path = stdout.readline().rstrip()
            if not path:
                self.worker.stdin.close()
                raise RuntimeError(f"archive startup failed ({self.worker.wait()})")
            self.path = Path(path)
            self.writer = threading.Thread(target=self._write, name="run-archive")
            self.writer.start()
        return self.path

    def _write(self):
        with self.worker.stdin as stream:
            while (record := self.queue.get()) is not None:
                if self.error is None:
                    try:
                        stream.write(json.dumps(record) + "\n")
                        stream.flush()
                    except OSError as error:
                        self.error = error

    def line(self, pid, source, stream, line):
        try:
            self.queue.put_nowait(dict(pid=pid, source=source, stream=stream, line=line))
        except Full:
            self.dropped += 1

    def close(self):
        if self.path is not None:
            if self.dropped:
                self.queue.put(dict(pid=os.getpid(), source="python", stream="gap",
                                    line=f"Python output queue dropped {self.dropped} records"))
            self.queue.put(None)
            self.writer.join()
            if self.worker.wait() or self.error or self.dropped:
                raise RuntimeError(f"archive output incomplete: {self.path}") from self.error


_archive: ContextVar[_Archive | None] = ContextVar("run_archive", default=None)


class ManagedProcess:
    """拥有一个子进程及输出线程；stop 回收两者后报告退出结果与输出错误。"""

    def __init__(self, command: Sequence[str], *, cwd: Path, env: dict[str, str],
                 on_line: Callable[[str], None] | None = None, tail_size: int = 30):
        self._archive = _archive.get()
        self._source = Path(command[0]).name
        if self._archive is not None:
            env = {**env, "FIREFLY_RUN_DIR": str(self._archive.open(env))}
        self.process = subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE)
        if self._archive is not None:
            self._archive.line(self.pid, self._source, "started", "")
        self.tail: deque[str] = deque(maxlen=tail_size)
        self.recording: Path | None = None
        self._recording_ready = threading.Event()
        self._reader_error: BaseException | None = None
        self.reader = threading.Thread(target=self._drain, args=(on_line,),
                                       name=f"output-{self.process.pid}", daemon=True)
        try:
            self.reader.start()
        except BaseException:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=15.)
            self.process.stdout.close()
            self.process.stderr.close()
            raise

    def _drain(self, on_line):
        def receive(stream, data):
            nonlocal on_line
            line = data.decode("utf-8", errors="replace").removesuffix("\r")
            if self._archive is not None:
                self._archive.line(self.pid, self._source, stream, line)
            self.tail.append(line)
            if stream == "stdout" and line.startswith("recording: "):
                self.recording = Path(line.removeprefix("recording: "))
                self._recording_ready.set()
            if on_line is not None:
                try:
                    on_line(line)
                except BaseException as error:
                    self._reader_error = error
                    on_line = None

        try:
            with selectors.DefaultSelector() as selector, self.process.stdout, self.process.stderr:
                for name in ("stdout", "stderr"):
                    selector.register(getattr(self.process, name), selectors.EVENT_READ, (name, bytearray()))
                while selector.get_map():
                    for key, _ in selector.select():
                        name, pending = key.data
                        raw = os.read(key.fd, 8192)
                        pending.extend(raw)
                        while (end := pending.find(b"\n")) >= 0:
                            receive(name, pending[:end])
                            del pending[:end + 1]
                        if not raw or len(pending) >= 65536:
                            if pending:
                                receive(name, pending)
                                pending.clear()
                        if not raw:
                            selector.unregister(key.fileobj)
        except BaseException as error:
            if self._reader_error is None:
                self._reader_error = error

    @property
    def pid(self) -> int:
        return self.process.pid

    def stop(self, timeout: float = 15.) -> int:
        deadline = time.monotonic() + timeout
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        code = self.process.wait(timeout=max(0., deadline - time.monotonic()))
        self.reader.join(timeout=max(0., deadline - time.monotonic()))
        if self.reader.is_alive():
            raise RuntimeError(f"output pipe still open after process exit, PID={self.pid}")
        if self._reader_error is not None:
            raise RuntimeError(
                f"output reader failed after child exit {code}, PID={self.pid}"
            ) from self._reader_error
        return code

    def wait_recording(self, timeout: float = 10.) -> Path:
        """读取 Mimir 返回的路径；启动失败和超时保留子进程输出。"""
        deadline = time.monotonic() + timeout
        while not self._recording_ready.wait(.01):
            if not self.reader.is_alive() or time.monotonic() >= deadline:
                raise RuntimeError("recorder did not return a recording path: " + " | ".join(self.tail))
        assert self.recording is not None
        return self.recording


def active_processes(root: Path) -> list[int]:
    """读取 xtask 对指定工作区的进程检查结果。"""
    workspace = workspace_root(Path(__file__))
    occupied = json.loads(subprocess.check_output(
        ["cargo", "xtask", "processes", str(root.resolve())], cwd=workspace,
    ))
    return [pid for pid in occupied if pid != os.getpid()]


def require_idle(root: Path) -> None:
    if occupied := active_processes(root):
        raise RuntimeError(f"repository flight processes already running: {occupied}")


@contextmanager
def isolated_run(root: Path):
    """独占运行锁，并在确认仓库进程空闲后启动一轮验收。"""
    directory = root / "logs"
    directory.mkdir(exist_ok=True)
    with (directory / "run.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require_idle(root)
        session = _Archive(root)
        token = _archive.set(session)
        try:
            yield
        finally:
            _archive.reset(token)
            session.close()
