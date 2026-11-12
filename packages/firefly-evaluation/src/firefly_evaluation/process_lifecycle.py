"""闭环测试进程的身份检查、输出消费与 SIGINT 收尾。"""

from collections import deque
from collections.abc import Callable, Sequence
from contextlib import contextmanager
from pathlib import Path
import fcntl
import os
import signal
import subprocess
import threading
import time
import psutil


class ManagedProcess:
    """拥有一个子进程及输出线程；stop 回收两者后报告退出结果与输出错误。"""

    def __init__(self, command: Sequence[str], *, cwd: Path, env: dict[str, str],
                 on_line: Callable[[str], None] | None = None, tail_size: int = 30):
        self.process = subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True,
                                        encoding="utf-8", errors="replace")
        self.tail: deque[str] = deque(maxlen=tail_size)
        self._reader_error: BaseException | None = None
        self.reader = threading.Thread(target=self._drain, args=(on_line,),
                                       name=f"output-{self.process.pid}", daemon=True)
        try:
            self.reader.start()
        except BaseException:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=15.)
            self.process.stdout.close()
            raise

    def _drain(self, on_line):
        try:
            with self.process.stdout as stream:
                for line in stream:
                    line = line.rstrip()
                    self.tail.append(line)
                    if on_line is not None:
                        try:
                            on_line(line)
                        except BaseException as error:
                            self._reader_error = error
                            on_line = None
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


def active_processes(root: Path) -> list[int]:
    """按可执行路径、工作目录与 argv 识别本仓库进程。"""
    root = root.resolve()
    found = []
    binaries = {"firefly-rootslam", "render", "mimir", "fc", "hil", "localization", "planner", "aliked", "lightglue"}
    scripts = {"firefly-sim", "mimir", "rerun", "collect_vision_map.py"}
    modules = {"firefly_sim", "firefly_sim.main",
               "firefly_evaluation.mission_sim", "firefly_evaluation.disturbance_sim"}
    for process in psutil.process_iter():
        if process.pid == os.getpid():
            continue
        try:
            with process.oneshot():
                args = process.cmdline()
                executable = process.exe()
                directory = process.cwd()
            if not executable or not directory:
                continue
            exe = Path(executable).resolve()
            cwd = Path(directory).resolve()
            native = exe.is_relative_to(root / "target") and exe.name in binaries
            python_entry = any(Path(arg).name in scripts for arg in args[:2])
            if args[1:2] == ["-m"]:
                python_entry = len(args) > 2 and args[2] in modules
            elif args[1:2] == ["-c"]:
                code = next(iter(args[2:3]), "")
                python_entry |= any(f"import_module({module!r})" in code for module in modules)
            python = cwd.is_relative_to(root) and python_entry
            if native or python:
                found.append(process.pid)
        except (psutil.Error, OSError, UnicodeError):
            continue
    return sorted(found)


def require_idle(root: Path) -> None:
    if occupied := active_processes(root):
        raise RuntimeError(f"repository flight processes already running: {occupied}")


@contextmanager
def isolated_run(root: Path):
    """独占验收锁，并在确认仓库进程空闲后启动一轮验收。"""
    directory = root / "logs"
    directory.mkdir(exist_ok=True)
    with (directory / "acceptance.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require_idle(root)
        yield
