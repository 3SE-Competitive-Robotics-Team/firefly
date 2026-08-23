"""一键拉起 sim + vio + planner 全栈（前缀日志 + Ctrl+C 统一收尾）。"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]  # repo 根
MAPS = ROOT / "apps" / "planner" / "maps"


def _prefix(stream: int, tag: str) -> None:
    """把子进程 stdout/stderr 逐行加上前缀转发到父进程同一流。"""
    while True:
        line = os.read(stream, 4096)
        if not line:
            return
        text = line.decode(errors="replace").rstrip("\n")
        for part in text.split("\n"):
            print(f"[{tag}] {part}", flush=True)


def _kill_tree(p: subprocess.Popen) -> None:
    """整棵进程树收尾（先 SIGTERM，宽限后 SIGKILL）。

    `cargo run` 会再 fork 出真正的二进制（cargo 是父进程）——只 terminate
    子进程本身杀不干净，二进制会作为孤儿继续跑（rerun 仍在写数据）。
    每个子进程自成一个进程组（`preexec_fn=os.setsid`），向进程组发信号
    才能连同 cargo + 二进制一起杀掉。
    """
    if p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait(timeout=5)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        epilog=(
            "示例：uv run firefly-launch  → 默认地图 gate.ffmap；"
            "启动后 uv run firefly-goal X Y Z 发布目标，无人机即飞往该点。"
        ),
    )
    ap.add_argument("--map", default="gate.ffmap", help="地图文件名（apps/planner/maps/ 下）")
    ap.add_argument("--release", action="store_true", help="Rust 侧用 release 构建")
    args = ap.parse_args()

    map_path = MAPS / args.map
    if not map_path.exists():
        print(f"地图不存在：{map_path}", file=sys.stderr)
        return 2

    release = ["--release"] if args.release else []
    cargo = ["cargo", "run", *release, "--manifest-path", str(ROOT / "Cargo.toml")]

    def spawn(cmd: list[str]) -> subprocess.Popen:
        return subprocess.Popen(
            cmd,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            # 每个子进程独立进程组（进程组长）：Ctrl+C 不直接波及，收尾时\n
            # 由 `_kill_tree` 对整个进程组发信号，才能连同 cargo 派生件杀净。\n
            start_new_session=True,
        )

    procs: list[tuple[str, subprocess.Popen]] = [
        ("sim", spawn(["uv", "run", "firefly-sim", "--no-trace"])),
        ("vio", spawn([*cargo, "-p", "vio"])),
        ("planner", spawn([*cargo, "-p", "planner", "--", "--map", str(map_path)])),
    ]

    print(
        f"已启动 3 进程：sim / vio / planner（地图 {args.map}，"
        f"{'release' if args.release else 'debug'}）\n"
        "planner 悬停等待目标——用 `uv run firefly-goal X Y Z` 发布飞行点。\n"
        "Ctrl+C 结束全部进程。",
        flush=True,
    )

    for tag, p in procs:
        threading.Thread(target=_prefix, args=(p.stdout.fileno(), tag), daemon=True).start()
        threading.Thread(target=_prefix, args=(p.stderr.fileno(), tag + ":err"), daemon=True).start()

    try:
        while True:
            exited = [tag for tag, p in procs if p.poll() is not None]
            if exited:
                print(f"进程退出：{', '.join(exited)}，收尾全部进程。", flush=True)
                break
            signal.pause()
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在收尾 …", flush=True)

    for _, p in procs:
        _kill_tree(p)
    for _, p in procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _kill_tree(p)

    code = max((p.returncode or 0) for _, p in procs)
    print("全部进程已结束。")
    return 0 if code == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
