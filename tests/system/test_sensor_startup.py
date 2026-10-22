"""发布构建的传感器启动检查；运行记录仅写 logs/ 下的 rrd。

从仓库根运行：FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q
需要 RMUC 资产、图形会话与 release 构建的 firefly-okvis-x / fc / render，且没有其他闭环进程。
GroundTruth 仅供 render 合成图像；屏蔽 PlantState，算法只使用 IMU 与图像。
"""
from __future__ import annotations

from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import sys
import time
import tomllib

import iceoryx2 as iox2
import numpy as np
import pytest

from firefly_mujoco import ControlMessage, DepthImageMessage, OdomMessage, TraceContext

from firefly_evaluation.process_lifecycle import ManagedProcess, isolated_run

ROOT = Path(__file__).resolve().parents[2]

pytestmark = pytest.mark.skipif(
    os.environ.get("FIREFLY_RUN_SENSOR_STARTUP") != "1",
    reason="requires explicit FIREFLY_RUN_SENSOR_STARTUP=1, RMUC assets and graphics session",
)


def test_sensor_startup() -> None:
    with isolated_run(ROOT):
        _sensor_startup()


def _sensor_startup() -> None:
    from firefly_mujoco import build_scene, load_scene_name, resolve_scene

    scene = resolve_scene(load_scene_name())
    build_scene()
    mesh = scene.visual
    if not mesh.is_file():
        raise FileNotFoundError(f"RMUC 视觉资产缺失：{mesh}")
    for name in ("firefly-okvis-x", "fc", "render"):
        if not (ROOT / "target" / "release" / name).is_file():
            raise RuntimeError(f"missing release binary: {name}")
    output = ROOT / "logs" / f"vio_sensor_startup_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}.rrd"
    output.parent.mkdir(exist_ok=True)
    env = os.environ.copy()
    env.update(RUST_LOG="info")
    processes: dict[str, ManagedProcess] = {}

    def start(name: str, command: list[str]) -> None:
        processes[name] = ManagedProcess(command, cwd=ROOT, env=env)

    def check_alive() -> None:
        for name, child in processes.items():
            if child.process.poll() is not None:
                raise RuntimeError(f"{name} exited ({child.process.returncode}):\n" + "\n".join(child.tail))

    try:
        start("viz", [sys.executable, "-m", "firefly_viz.main", "--save", str(output)])
        time.sleep(2)
        check_alive()
        start("render", [str(ROOT / "target/release/render")])
        start("firefly-okvis-x", [str(ROOT / "target/release/firefly-okvis-x")])
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
        depth = subscribe("Firefly/Depth", DepthImageMessage)
        with (ROOT / "configs/render.toml").open("rb") as stream:
            depth_options = tomllib.load(stream).get("depth_noise", {})
        depth_min = depth_options.get("min_depth_m", 0.2)
        depth_max = depth_options.get("max_depth_m", 8.0)
        # GroundTruth 供 render 摆放传感器；PlantState 不参与启动。
        simulation = """
from firefly_sim.main import main
main(["--no-trace"], publish_plant_state=False)
"""
        start("sim", [sys.executable, "-c", simulation])
        first_ready = None
        first_position = None
        samples = 0
        controls = 0
        valid_depth_frames = 0
        previous_depth_timestamp = None
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
            while (sample := depth.receive()) is not None:
                msg = sample.payload().contents
                values = np.ctypeslib.as_array(msg.data)
                assert (msg.width, msg.height) == (320, 240)
                assert np.isfinite(values).all()
                assert np.all((values == 0.) | ((values >= depth_min) & (values <= depth_max)))
                if previous_depth_timestamp is not None:
                    assert msg.timestamp > previous_depth_timestamp
                previous_depth_timestamp = msg.timestamp
                valid_depth_frames += int(np.any(values > 0.))
            if first_ready is not None and samples >= 20 and controls >= 100 and valid_depth_frames >= 3:
                break
            time.sleep(0.01)
        if first_ready is None:
            diagnostics = "\n".join(f"{name}:\n" + "\n".join(child.tail) for name, child in processes.items())
            raise AssertionError("传感器启动未就绪\n" + diagnostics)
        assert first_ready >= 1.0, "不得跳过静止观测窗口"
        assert np.linalg.norm(first_position) < 0.1, f"初始位置应为局部原点: {first_position}"
        assert controls >= 100, "无 PlantState 时也必须持续发布零推力"
        assert valid_depth_frames >= 3, "必须从发布端收到至少三帧有效深度测量"
        if os.environ.get("FIREFLY_RUN_FLIGHT") == "1":
            _verify_flight(ground_truth, check_alive)
        print(f"PASS: ready={first_ready:.3f}s, local_position={first_position}, odom={samples}, controls={controls}")
    finally:
        # 只停止本次创建的进程；SIGINT 触发端口 Drop 与录制 flush。
        failures = []
        for name, child in reversed(processes.items()):
            try:
                code = child.stop()
                if code != 0:
                    failures.append(f"{name} 退出码={code}，PID={child.pid}")
            except (subprocess.TimeoutExpired, RuntimeError, OSError) as error:
                failures.append(f"{name} 未能完整退出，PID={child.pid}: {error}")
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
