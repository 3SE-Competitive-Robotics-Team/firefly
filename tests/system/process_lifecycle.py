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


class ManagedProcess:
    """拥有一个直接启动的子进程及其输出线程；stop 必须完成两者的回收。"""

    def __init__(self, command: Sequence[str], *, cwd: Path, env: dict[str, str],
                 on_line: Callable[[str], None] | None = None, tail_size: int = 30):
        self.process = subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True,
                                        encoding="utf-8", errors="replace")
        self.tail: deque[str] = deque(maxlen=tail_size)
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
        with self.process.stdout as stream:
            for line in stream:
                line = line.rstrip()
                self.tail.append(line)
                if on_line is not None:
                    on_line(line)

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
        return code


def active_processes(root: Path) -> list[int]:
    """识别本仓库进程；依赖可执行路径与 argv，禁止模糊匹配命令行全文。"""
    found = []
    binaries = {"vio", "render", "fc", "localization", "planner", "aliked", "lightglue"}
    scripts = {"firefly-sim", "firefly-viz", "rerun", "mission_sim.py", "collect_vision_map.py"}
    modules = {"firefly_sim", "firefly_sim.main", "firefly_viz", "firefly_viz.main"}
    for path in Path("/proc").glob("[0-9]*"):
        if int(path.name) == os.getpid():
            continue
        try:
            args = path.joinpath("cmdline").read_bytes().decode().strip("\0").split("\0")
            exe = path.joinpath("exe").resolve(strict=True)
            cwd = path.joinpath("cwd").resolve(strict=True)
            native = exe.is_relative_to(root / "target") and exe.name.split("-")[0] in binaries
            python_entry = any(Path(arg).name in scripts for arg in args[:2])
            if args[1:2] == ["-m"]:
                python_entry = len(args) > 2 and args[2] in modules
            elif args[1:2] == ["-c"]:
                code = next(iter(args[2:3]), "")
                python_entry |= any(f"import_module({module!r})" in code for module in modules)
            python = cwd.is_relative_to(root) and python_entry
            if native or python:
                found.append(int(path.name))
        except (OSError, UnicodeError):
            continue
    return sorted(found)


def require_idle(root: Path) -> None:
    if occupied := active_processes(root):
        raise RuntimeError(f"repository flight processes already running: {occupied}")


@contextmanager
def isolated_run(root: Path):
    """同仓库运行验收互斥；进程残留必须在创建下一轮子进程之前报错。"""
    directory = root / "logs"
    directory.mkdir(exist_ok=True)
    with (directory / "acceptance.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require_idle(root)
        yield
