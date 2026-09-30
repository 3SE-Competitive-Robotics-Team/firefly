"""发布构建的传感器启动检查；运行记录仅写 logs/ 下的 rrd。

从仓库根运行：FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q
需要 RMUC 资产、图形会话与 release 构建的 vio / fc / render，且没有其他闭环进程。
GroundTruth 仅供 render 合成图像；屏蔽 PlantState，算法只使用 IMU 与图像。
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
import pytest

from firefly_mujoco import ControlMessage, OdomMessage, TraceContext

ROOT = Path(__file__).resolve().parents[2]

pytestmark = pytest.mark.skipif(
    os.environ.get("FIREFLY_RUN_SENSOR_STARTUP") != "1",
    reason="requires explicit FIREFLY_RUN_SENSOR_STARTUP=1, RMUC assets and graphics session",
)


def test_sensor_startup() -> None:
    from firefly_mujoco import build_scene, load_scene_name
    load_scene_name()
    build_scene()
    mesh = ROOT / "models/rmuc2026/field.glb"
    if not mesh.is_file():
        raise FileNotFoundError(f"RMUC 视觉资产缺失：{mesh}")
    for name in ("vio", "fc", "render"):
        if not (ROOT / "target" / "release" / name).is_file():
            raise RuntimeError(f"missing release binary: {name}")
    output = ROOT / "logs" / f"vio_sensor_startup_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}.rrd"
    output.parent.mkdir(exist_ok=True)
    env = os.environ.copy()
    env.update(RUST_LOG="info")
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
        start("render", [str(ROOT / "target/release/render")])
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
        ground_truth = subscribe("Firefly/GroundTruth", OdomMessage)
        control = subscribe("Firefly/Control", ControlMessage, 32)
        # GroundTruth 供 render 摆放传感器；PlantState 不参与启动。
        simulation = """
import importlib
import sys
sim = importlib.import_module('firefly_sim.main')
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
        deadline = time.monotonic() + 90
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
            raise AssertionError("传感器启动未就绪\n" + diagnostics)
        assert first_ready >= 1.0, "不得跳过静止观测窗口"
        assert np.linalg.norm(first_position) < 0.1, f"初始位置应为局部原点: {first_position}"
        assert controls >= 100, "无 PlantState 时也必须持续发布零推力"
        if os.environ.get("FIREFLY_RUN_FLIGHT") == "1":
            _verify_flight(ground_truth, check_alive)
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


def _verify_flight(ground_truth, check_alive):
    """真值只用于起飞/悬停/降落验收；指令不携带真值反馈。"""
    def command(*args):
        subprocess.run([str(ROOT / "target/release/ffctl"), "fc", *args], cwd=ROOT, check=True)

    def latest():
        value = None
        while (sample := ground_truth.receive()) is not None:
            msg = sample.payload().contents
            value = (msg.timestamp, np.array([msg.position_x, msg.position_y, msg.position_z]),
                     np.array([msg.velocity_x, msg.velocity_y, msg.velocity_z]))
        return value

    initial = latest()
    assert initial is not None, "flight evaluation requires ground truth samples"
    _, origin, _ = initial
    command("arm")
    time.sleep(0.2)
    command("takeoff", "1.0")
    settled_at = None
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        check_alive()
        state = latest()
        if state is not None:
            stamp, position, velocity = state
            assert np.isfinite(position).all()
            assert np.linalg.norm(position[:2] - origin[:2]) < 0.5, "lateral drift during hover"
            assert -0.1 < position[2] - origin[2] < 1.5, "unsafe altitude excursion"
            stable = abs(position[2] - origin[2] - 1.) < 0.15 and np.linalg.norm(velocity) < 0.2
            settled_at = (settled_at if settled_at is not None else stamp) if stable else None
            if settled_at is not None and stamp - settled_at >= 2.:
                break
        time.sleep(0.01)
    else:
        raise AssertionError("takeoff did not reach two seconds of stable hover")
    command("land")
    settled_at = None
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        check_alive()
        state = latest()
        if state is not None:
            stamp, position, velocity = state
            assert np.isfinite(position).all() and np.isfinite(velocity).all()
            assert np.linalg.norm(position[:2] - origin[:2]) < 0.5, "lateral drift during landing"
            assert -0.1 < position[2] - origin[2] < 1.5, "unsafe landing altitude"
            stable = abs(position[2] - origin[2]) < 0.08 and np.linalg.norm(velocity) < 0.1
            settled_at = (settled_at if settled_at is not None else stamp) if stable else None
            if settled_at is not None and stamp - settled_at >= 1.:
                command("disarm")
                return
        time.sleep(0.01)
    raise AssertionError("landing did not settle at the pad")
