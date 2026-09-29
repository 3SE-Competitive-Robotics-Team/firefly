"""发布构建的无真值启动检查；运行记录仅写 logs/ 下的 rrd。

从仓库根运行：uv run --no-dev python bench/check_sensor_startup.py
需要预先 cargo build --release -p vio -p fc，且没有其他闭环进程运行。
"""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import iceoryx2 as iox2
import numpy as np

from firefly_mujoco import ControlMessage, OdomMessage, TraceContext

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    for name in ("vio", "fc"):
        if not (ROOT / "target" / "release" / name).is_file():
            raise RuntimeError(f"missing release binary: {name}")
    output = ROOT / "logs" / f"vio_sensor_startup_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}.rrd"
    output.parent.mkdir(exist_ok=True)
    env = os.environ.copy()
    env.update(MUJOCO_GL="egl", RUST_LOG="info")
    processes: list[tuple[str, subprocess.Popen, deque[str]]] = []

    def start(name: str, command: list[str]) -> None:
        process = subprocess.Popen(command, cwd=ROOT, env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        tail: deque[str] = deque(maxlen=30)
        def drain() -> None:
            for line in process.stdout:
                tail.append(line.rstrip())
        threading.Thread(target=drain, daemon=True).start()
        processes.append((name, process, tail))

    def check_alive() -> None:
        for name, process, tail in processes:
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited ({process.returncode}):\n" + "\n".join(tail))

    try:
        start("viz", [sys.executable, "-m", "firefly_viz.main", "--save", str(output)])
        time.sleep(2)
        check_alive()
        start("vio", [str(ROOT / "target/release/vio")])
        start("fc", [str(ROOT / "target/release/fc")])
        iox2.set_log_level(iox2.LogLevel.Error)
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        def subscribe(topic, payload, capacity=None):
            builder = node.service_builder(iox2.ServiceName.new(topic)).publish_subscribe(payload).user_header(TraceContext)
            if capacity is not None:
                builder = builder.subscriber_max_buffer_size(capacity)
            return builder.open_or_create().subscriber_builder().create()
        odom = subscribe("Firefly/Odometry", OdomMessage)
        control = subscribe("Firefly/Control", ControlMessage, 32)
        # 在夹具中屏蔽两种真值发布；算法进程使用与正常启动相同的二进制和配置。
        simulation = """
import importlib
import sys
sim = importlib.import_module('firefly_sim.main')
sim.load_scene_name = lambda: 'boxes'
sim._publish_gt = lambda *args: None
sim._publish_plant_state = lambda *args: None
sys.argv = ['firefly-sim', '--no-trace']
sim.main()
"""
        start("sim", [sys.executable, "-c", simulation])
        first_ready = None
        first_position = None
        samples = 0
        controls = 0
        previous_timestamp = None
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            check_alive()
            while (sample := odom.receive()) is not None:
                msg = sample.payload().contents
                samples += 1
                if msg.is_initialized and first_ready is None:
                    first_ready = msg.timestamp
                    first_position = np.array([msg.position_x, msg.position_y, msg.position_z])
                if previous_timestamp is not None:
                    assert msg.timestamp > previous_timestamp, "里程计状态时间必须严格递增"
                previous_timestamp = msg.timestamp
            while (sample := control.receive()) is not None:
                msg = sample.payload().contents
                controls += 1
                assert all(value == 0.0 for value in msg.thrust), "未解锁时不得输出推力"
            if first_ready is not None and samples >= 20 and controls >= 100:
                break
            time.sleep(0.01)
        if first_ready is None:
            diagnostics = "\n".join(f"{name}:\n" + "\n".join(tail) for name, _, tail in processes)
            raise AssertionError("无真值启动未就绪\n" + diagnostics)
        assert first_ready >= 1.0, "不得跳过静止观测窗口"
        assert np.linalg.norm(first_position) < 0.1, f"初始位置应为局部原点: {first_position}"
        assert controls >= 100, "无 PlantState 时也必须持续发布零推力"
        print(f"PASS: ready={first_ready:.3f}s, local_position={first_position}, odom={samples}, controls={controls}")
    finally:
        # 只停止本次创建的进程；SIGINT 触发端口 Drop 与录制 flush。
        failures = []
        for name, process, _ in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    failures.append(f"{name} 未能优雅退出，PID={process.pid}")
        print(f"recording: {output}")
        if failures:
            raise RuntimeError("; ".join(failures))


if __name__ == "__main__":
    main()
