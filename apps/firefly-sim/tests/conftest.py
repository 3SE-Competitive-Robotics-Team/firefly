"""VIO E2E 测试共享 fixtures：启动 sim + vio 进程，收集 odom/GT，计算指标。"""
import asyncio
import subprocess
import time
from pathlib import Path
from typing import Generator

import numpy as np
import pytest

import iceoryx2 as iox2
from firefly_mujoco.messages import OdomMessage


# ===== 轨迹定义（3 场景 × 10 条曲线）=====

TRAJECTORIES = {
    # --- 场景 A: 基础运动 ---
    "line": {"duration": 60.0, "speed": 0.8},
    "snake": {"duration": 80.0, "amp_y": 1.5, "freq": 0.5, "speed": 0.8},
    "helix": {"duration": 100.0, "radius": 3.0, "climb": 0.1, "speed": 0.8},
    "figure8": {"duration": 90.0, "scale": 2.0, "speed": 0.8},
    # --- 场景 B: 机动/边界 ---
    "hover": {"duration": 30.0, "noise": 0.05},
    "yaw_spin": {"duration": 40.0, "rate": 0.3},
    "aggressive": {"duration": 50.0, "max_accel": 3.0, "max_jerk": 5.0},
    # --- 场景 C: 任务相似 ---
    "waypoints": {"duration": 70.0, "waypoints": [[1,4,1], [10,4,1], [10,6,2], [20,6,2]]},
    "avoid_column": {"duration": 80.0, "obs_x": 12.0, "obs_y": 4.0, "radius": 2.0},
    "combo": {"duration": 120.0, "segments": ["line", "snake", "helix"]},
}

# 预期指标阈值（根据 vio_verify.md 经验值设定）
THRESHOLDS = {
    "line": {"ate_rmse": 0.3, "rpe_trans": 0.15},
    "snake": {"ate_rmse": 0.5, "rpe_trans": 0.2},
    "helix": {"ate_rmse": 0.6, "rpe_trans": 0.25},
    "figure8": {"ate_rmse": 0.8, "rpe_trans": 0.3},  # 闭环累积漂移较大
    "hover": {"ate_rmse": 0.2, "rpe_trans": 0.1},
    "yaw_spin": {"ate_rmse": 0.4, "rpe_trans": 0.2},
    "aggressive": {"ate_rmse": 1.0, "rpe_trans": 0.4},
    "waypoints": {"ate_rmse": 0.6, "rpe_trans": 0.25},
    "avoid_column": {"ate_rmse": 0.7, "rpe_trans": 0.3},
    "combo": {"ate_rmse": 1.0, "rpe_trans": 0.35},
}


def _scripted_ref(t: float, traj_type: str, params: dict) -> tuple[np.ndarray, np.ndarray]:
    """生成脚本化参考轨迹：返回 (pos, vel)。与 firefly-sim/main.py 保持一致。"""
    p = params
    if traj_type == "line":
        pos = np.array([1.0 + p["speed"] * t, 4.0, 1.0])
        vel = np.array([p["speed"], 0.0, 0.0])
    elif traj_type == "snake":
        pos = np.array([
            1.0 + p["speed"] * t,
            4.0 + p["amp_y"] * np.sin(p["freq"] * t),
            1.0 + 0.5 * np.sin(0.4 * t),
        ])
        vel = np.array([
            p["speed"],
            p["amp_y"] * p["freq"] * np.cos(p["freq"] * t),
            0.5 * 0.4 * np.cos(0.4 * t),
        ])
    elif traj_type == "helix":
        pos = np.array([
            1.0 + p["speed"] * t,
            4.0 + p["radius"] * np.sin(0.3 * t),
            1.0 + p["climb"] * t,
        ])
        vel = np.array([
            p["speed"],
            p["radius"] * 0.3 * np.cos(0.3 * t),
            p["climb"],
        ])
    elif traj_type == "figure8":
        s = p["scale"]
        pos = np.array([
            1.0 + s * np.sin(0.2 * t),
            4.0 + s * np.sin(0.4 * t),
            1.0 + 0.3 * np.sin(0.2 * t),
        ])
        vel = np.array([
            s * 0.2 * np.cos(0.2 * t),
            s * 0.4 * np.cos(0.4 * t),
            0.3 * 0.2 * np.cos(0.2 * t),
        ])
    elif traj_type == "hover":
        pos = np.array([1.0, 4.0, 1.0]) + np.random.normal(0, p["noise"], 3)
        vel = np.zeros(3)
    elif traj_type == "yaw_spin":
        # 纯偏航：位置固定，偏航角随时间变化（通过速度向量体现）
        pos = np.array([5.0, 4.0, 1.0])
        vel = np.array([0.0, 0.0, 0.0])
    elif traj_type == "aggressive":
        # 分段高加速度：正弦加速度曲线
        a = p["max_accel"] * np.sin(0.5 * t)
        pos = np.array([1.0 + 0.5 * t**2, 4.0, 1.0])  # 简化
        vel = np.array([a * t, 0.0, 0.0])
    elif traj_type == "waypoints":
        # 简化：线性插值航点
        wps = np.array(p["waypoints"])
        seg_dur = p["duration"] / (len(wps) - 1)
        idx = min(int(t / seg_dur), len(wps) - 2)
        alpha = (t - idx * seg_dur) / seg_dur
        pos = wps[idx] + alpha * (wps[idx + 1] - wps[idx])
        vel = (wps[idx + 1] - wps[idx]) / seg_dur
    elif traj_type == "avoid_column":
        # 绕圆柱半圆
        cx, cy = p["obs_x"], p["obs_y"]
        r = p["radius"]
        angle = np.pi * t / p["duration"]
        pos = np.array([cx + r * np.cos(angle), cy + r * np.sin(angle), 1.5])
        vel = np.array([-r * np.pi / p["duration"] * np.sin(angle),
                        r * np.pi / p["duration"] * np.cos(angle), 0.0])
    elif traj_type == "combo":
        # 串联三段
        segs = p["segments"]
        seg_dur = p["duration"] / len(segs)
        idx = min(int(t / seg_dur), len(segs) - 1)
        return _scripted_ref(t - idx * seg_dur, segs[idx], {})
    else:
        raise ValueError(f"Unknown trajectory: {traj_type}")
    return pos, vel


# ===== 指标计算 =====

def align_trajectories(gt: np.ndarray, est: np.ndarray, gt_times: np.ndarray, est_times: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """时间对齐：将 est 插值到 gt 时间戳。"""
    from scipy.interpolate import interp1d
    if len(est_times) < 2:
        return gt, est
    f = interp1d(est_times, est, axis=0, bounds_error=False, fill_value="extrapolate")
    est_aligned = f(gt_times)
    return gt, est_aligned


def compute_ate(gt: np.ndarray, est: np.ndarray) -> dict:
    """Absolute Trajectory Error (RMSE, mean, max)。"""
    err = np.linalg.norm(gt - est, axis=1)
    return {"rmse": float(np.sqrt(np.mean(err**2))), "mean": float(np.mean(err)), "max": float(np.max(err))}


def compute_rpe(gt: np.ndarray, est: np.ndarray, delta: int = 10) -> dict:
    """Relative Pose Error (平移/旋转 RMSE)。"""
    if len(gt) < delta + 1:
        return {"trans_rmse": 0.0, "rot_rmse": 0.0}
    gt_rel = gt[delta:] - gt[:-delta]
    est_rel = est[delta:] - est[:-delta]
    trans_err = np.linalg.norm(gt_rel - est_rel, axis=1)
    return {"trans_rmse": float(np.sqrt(np.mean(trans_err**2))), "rot_rmse": 0.0}  # 旋转暂略


# ===== Fixtures =====

@pytest.fixture(scope="session")
def sim_vio_processes() -> Generator[dict, None, None]:
    """Session 级：启动 sim + vio，提供 iceoryx2 订阅器。"""
    # 由各测例单独启动特定轨迹的 sim；这里只提供通用订阅器创建函数
    yield {}


@pytest.fixture
def trajectory_data(request) -> dict:
    """当前测例的轨迹参数与阈值。"""
    name = request.param
    return {
        "name": name,
        "params": TRAJECTORIES[name],
        "thresholds": THRESHOLDS[name],
        "duration": TRAJECTORIES[name]["duration"],
    }


# ===== 进程管理 =====

class SimVioRunner:
    """管理 sim + vio 子进程，订阅 odom/GT，收集数据。"""

    def __init__(self, traj_name: str, params: dict, duration: float):
        self.traj_name = traj_name
        self.params = params
        self.duration = duration
        self.sim_proc: subprocess.Popen | None = None
        self.vio_proc: subprocess.Popen | None = None
        self.node = None
        self.gt_sub = None
        self.odom_sub = None
        self.gt_data: list[tuple[float, np.ndarray]] = []
        self.odom_data: list[tuple[float, np.ndarray]] = []

    def start_sim(self):
        """启动 firefly-sim，传入轨迹参数（通过环境变量或 stdin）。"""
        env = {"PYTHONPATH": "apps/firefly-sim/src"}
        # 这里简化：直接用 CLI --script line 等，实际可扩展为参数化
        # 但为了 pytest 化，我们在 Python 里直接跑 sim 循环（见下方 run_inline）
        raise NotImplementedError("Use run_inline for pytest-native execution")

    def start_vio(self):
        """启动 vio 进程。"""
        self.vio_proc = subprocess.Popen(
            ["cargo", "run", "-p", "vio"],
            cwd="/Users/flamingo/Projects/robomaster/firefly",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(2)  # 等待 vio 连上 iceoryx2

    def setup_subscribers(self):
        """创建 iceoryx2 订阅器。"""
        self.node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self.gt_sub = self.node.service_builder(iox2.ServiceName.new("Firefly/GroundTruth")) \
            .publish_subscribe(OdomMessage).user_header(None).open_or_create() \
            .subscriber_builder().create()
        self.odom_sub = self.node.service_builder(iox2.ServiceName.new("Firefly/Odometry")) \
            .publish_subscribe(OdomMessage).user_header(None).open_or_create() \
            .subscriber_builder().create()

    def collect(self, timeout: float) -> tuple[list, list]:
        """收集指定时长的数据。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            # GT
            while (sample := self.gt_sub.receive()) is not None:
                m = sample.payload().contents
                self.gt_data.append((m.timestamp, np.array([m.position_x, m.position_y, m.position_z])))
            # Odom
            while (sample := self.odom_sub.receive()) is not None:
                m = sample.payload().contents
                if m.is_initialized:
                    self.odom_data.append((m.timestamp, np.array([m.position_x, m.position_y, m.position_z])))
            time.sleep(0.05)
        return self.gt_data, self.odom_data

    def stop(self):
        if self.vio_proc:
            self.vio_proc.terminate()
            self.vio_proc.wait(timeout=5)
        if self.sim_proc:
            self.sim_proc.terminate()
            self.sim_proc.wait(timeout=5)


# ===== Inline sim 运行器（pytest-native，无子进程 sim）=====

async def run_sim_inline(traj_name: str, params: dict, duration: float,
                         gt_queue: asyncio.Queue, odom_queue: asyncio.Queue) -> None:
    """在当前进程异步运行 sim 循环，发布 GT 到 queue；vio 另起进程订阅 iceoryx2。"""
    from firefly_mujoco import DroneEnv
    from firefly_mujoco.messages import (
        IMAGE_HEIGHT, IMAGE_WIDTH, ImuMessage, GrayImageMessage, DepthImageMessage, OdomMessage, ReferenceMessage
    )
    import iceoryx2 as iox2

    env = DroneEnv()
    env.reset(np.array([1.0, 4.0, 1.0]), np.array([0.0, 0.0, 0.0, 1.0]))

    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    imu_pub = node.service_builder(iox2.ServiceName.new("Firefly/Imu")).publish_subscribe(ImuMessage).open_or_create().publisher_builder().create()
    left_pub = node.service_builder(iox2.ServiceName.new("Firefly/CameraLeft")).publish_subscribe(GrayImageMessage).open_or_create().publisher_builder().create()
    right_pub = node.service_builder(iox2.ServiceName.new("Firefly/CameraRight")).publish_subscribe(GrayImageMessage).open_or_create().publisher_builder().create()
    depth_pub = node.service_builder(iox2.ServiceName.new("Firefly/Depth")).publish_subscribe(DepthImageMessage).open_or_create().publisher_builder().create()
    gt_pub = node.service_builder(iox2.ServiceName.new("Firefly/GroundTruth")).publish_subscribe(OdomMessage).open_or_create().publisher_builder().create()

    next_imu = 0.0
    next_cam = 0.0
    IMU_PERIOD = 0.01
    CAM_PERIOD = 0.1

    while env.time < duration:
        # 控制
        ref_pos, ref_vel = _scripted_ref(env.time, traj_name, params)
        env.apply_pd(ref_pos, ref_vel)
        env.step()

        # IMU 100Hz
        if env.time >= next_imu:
            gyro, accel = env.imu()
            msg = ImuMessage()
            msg.timestamp = env.time
            msg.angular_velocity_x, msg.angular_velocity_y, msg.angular_velocity_z = gyro
            msg.linear_acceleration_x, msg.linear_acceleration_y, msg.linear_acceleration_z = accel
            imu_pub.loan_uninit().write_payload(msg).send()
            next_imu += IMU_PERIOD

        # Camera 10Hz
        if env.time >= next_cam:
            # 灰度
            left = GrayImageMessage()
            left.timestamp = env.time
            left.sensor_id = 0
            left.width = IMAGE_WIDTH
            left.height = IMAGE_HEIGHT
            left.data[:] = env.render_left().reshape(-1)
            left_pub.loan_uninit().write_payload(left).send()

            right = GrayImageMessage()
            right.timestamp = env.time
            right.sensor_id = 1
            right.width = IMAGE_WIDTH
            right.height = IMAGE_HEIGHT
            right.data[:] = env.render_right().reshape(-1)
            right_pub.loan_uninit().write_payload(right).send()

            # 深度
            depth = DepthImageMessage()
            depth.timestamp = env.time
            depth.sensor_id = 0
            depth.width = IMAGE_WIDTH
            depth.height = IMAGE_HEIGHT
            depth.data[:] = env.render_depth().reshape(-1)
            depth_pub.loan_uninit().write_payload(depth).send()

            # GT 发布 + 入队
            pos, quat_xyzw, vel = env.gt_pose()
            gt_msg = OdomMessage()
            gt_msg.timestamp = env.time
            gt_msg.position_x, gt_msg.position_y, gt_msg.position_z = pos
            gt_msg.velocity_x, gt_msg.velocity_y, gt_msg.velocity_z = vel
            gt_msg.quat_x, gt_msg.quat_y, gt_msg.quat_z, gt_msg.quat_w = quat_xyzw
            gt_msg.is_initialized = True
            gt_pub.loan_uninit().write_payload(gt_msg).send()
            await gt_queue.put((env.time, pos.copy()))

            next_cam += CAM_PERIOD

        await asyncio.sleep(0.001)  # 让出控制权


async def run_vio_collector(duration: float, odom_queue: asyncio.Queue) -> None:
    """在当前进程异步运行 vio 采集（订阅 iceoryx2 odom）。"""
    import iceoryx2 as iox2
    from firefly_mujoco.messages import OdomMessage

    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    odom_sub = node.service_builder(iox2.ServiceName.new("Firefly/Odometry")) \
        .publish_subscribe(OdomMessage).open_or_create().subscriber_builder().create()

    t0 = time.time()
    while time.time() - t0 < duration:
        while (sample := odom_sub.receive()) is not None:
            m = sample.payload().contents
            if m.is_initialized:
                pos = np.array([m.position_x, m.position_y, m.position_z])
                await odom_queue.put((m.timestamp, pos))
        await asyncio.sleep(0.01)


# ===== 实际测试 =====

@pytest.mark.parametrize("trajectory_data", list(TRAJECTORIES.keys()), indirect=True)
@pytest.mark.asyncio
async def test_vio_e2e(trajectory_data):
    """VIO 端到端测试：跑单条轨迹，验证 ATE/RPE 阈值。"""
    name = trajectory_data["name"]
    params = trajectory_data["params"]
    thresholds = trajectory_data["thresholds"]
    duration = trajectory_data["duration"]

    print(f"\n=== VIO E2E: {name} (duration={duration}s) ===")

    gt_queue: asyncio.Queue = asyncio.Queue()
    odom_queue: asyncio.Queue = asyncio.Queue()

    # 启动 sim + vio collector 并发
    sim_task = asyncio.create_task(run_sim_inline(name, params, duration, gt_queue, odom_queue))
    vio_task = asyncio.create_task(run_vio_collector(duration + 5, odom_queue))

    await asyncio.gather(sim_task, vio_task, return_exceptions=True)

    # 收集数据
    gt_list = []
    while not gt_queue.empty():
        gt_list.append(await gt_queue.get())
    odom_list = []
    while not odom_queue.empty():
        odom_list.append(await odom_queue.get())

    assert len(gt_list) > 10, f"{name}: GT 数据太少 ({len(gt_list)})"
    assert len(odom_list) > 10, f"{name}: Odom 数据太少 ({len(odom_list)})"

    gt_times = np.array([t for t, _ in gt_list])
    gt_pos = np.array([p for _, p in gt_list])
    odom_times = np.array([t for t, _ in odom_list])
    odom_pos = np.array([p for _, p in odom_list])

    # 时间对齐
    gt_pos_aligned, odom_pos_aligned = align_trajectories(gt_pos, odom_pos, gt_times, odom_times)

    # 计算指标
    ate = compute_ate(gt_pos_aligned, odom_pos_aligned)
    rpe = compute_rpe(gt_pos_aligned, odom_pos_aligned)

    print(f"  ATE RMSE: {ate['rmse']:.3f} (阈值 {thresholds['ate_rmse']})")
    print(f"  RPE trans RMSE: {rpe['trans_rmse']:.3f} (阈值 {thresholds['rpe_trans']})")

    # 断言
    assert ate["rmse"] < thresholds["ate_rmse"], f"{name}: ATE RMSE {ate['rmse']:.3f} > {thresholds['ate_rmse']}"
    assert rpe["trans_rmse"] < thresholds["rpe_trans"], f"{name}: RPE trans {rpe['trans_rmse']:.3f} > {thresholds['rpe_trans']}"