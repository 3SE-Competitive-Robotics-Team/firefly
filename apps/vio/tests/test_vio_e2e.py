"""VIO 端到端测试：pytest 编排 sim + vio (三进程)，验证 ATE/RPE 指标。"""
import asyncio
import os
import subprocess
import time
from pathlib import Path
from typing import Generator

import numpy as np
import pytest

# 尝试导入 iceoryx2（测试环境需已安装）
try:
    import iceoryx2 as iox2
    from firefly_mujoco.messages import OdomMessage, TraceContext
    HAS_ICEORYX2 = True
except ImportError:
    HAS_ICEORYX2 = False
    iox2 = None
    OdomMessage = None
    TraceContext = None


# ===== 10 条标准曲线定义（3 场景）=====

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
    "waypoints": {"duration": 70.0, "waypoints": [[1, 4, 1], [10, 4, 1], [10, 6, 2], [20, 6, 2]]},
    "avoid_column": {"duration": 80.0, "obs_x": 12.0, "obs_y": 4.0, "radius": 2.0},
    "combo": {"duration": 120.0, "segments": ["line", "snake", "helix"]},
}

# 预期指标阈值（按 vio_verify.md 经验值设定）
THRESHOLDS = {
    "line": {"ate_rmse": 0.3, "rpe_trans": 0.15},
    "snake": {"ate_rmse": 0.5, "rpe_trans": 0.2},
    "helix": {"ate_rmse": 0.6, "rpe_trans": 0.25},
    "figure8": {"ate_rmse": 0.8, "rpe_trans": 0.3},
    "hover": {"ate_rmse": 0.2, "rpe_trans": 0.1},
    "yaw_spin": {"ate_rmse": 0.4, "rpe_trans": 0.2},
    "aggressive": {"ate_rmse": 1.0, "rpe_trans": 0.4},
    "waypoints": {"ate_rmse": 0.6, "rpe_trans": 0.25},
    "avoid_column": {"ate_rmse": 0.7, "rpe_trans": 0.3},
    "combo": {"ate_rmse": 1.0, "rpe_trans": 0.35},
}


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
    """Relative Pose Error (平移 RMSE)。"""
    if len(gt) < delta + 1:
        return {"trans_rmse": 0.0}
    gt_rel = gt[delta:] - gt[:-delta]
    est_rel = est[delta:] - est[:-delta]
    trans_err = np.linalg.norm(gt_rel - est_rel, axis=1)
    return {"trans_rmse": float(np.sqrt(np.mean(trans_err**2)))}


# ===== Fixtures =====

@pytest.fixture(scope="session", autouse=True)
def build_vio_release():
    """Session 级：构建 vio release 二进制。"""
    repo_root = Path(__file__).parent.parent.parent.parent
    cargo_path = "/Users/flamingo/.cargo/bin/cargo"
    result = subprocess.run(
        [cargo_path, "build", "-p", "vio", "--release"],
        cwd=repo_root,
        capture_output=True,
        text=True,
        timeout=300,
    )
    if result.returncode != 0:
        pytest.fail(f"cargo build -p vio --release 失败:\n{result.stderr}")
    vio_bin = repo_root / "target" / "release" / "vio"
    assert vio_bin.exists(), f"vio 二进制不存在: {vio_bin}"
    return vio_bin


@pytest.fixture(scope="session")
def iceoryx2_node():
    """提供 iceoryx2 node（session 共享）。"""
    if not HAS_ICEORYX2:
        pytest.skip("iceoryx2 未安装")
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    yield node


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


# ===== 三进程编排器 =====

class VioE2ERunner:
    """管理 sim 子进程 + vio 子进程，收集 odom/GT 数据。"""

    def __init__(self, traj_name: str, params: dict, duration: float, vio_bin: Path, node):
        self.traj_name = traj_name
        self.params = params
        self.duration = duration
        self.vio_bin = vio_bin
        self.node = node
        self.sim_proc: subprocess.Popen | None = None
        self.vio_proc: subprocess.Popen | None = None
        self.gt_sub = None
        self.odom_sub = None
        self.gt_data: list[tuple[float, np.ndarray]] = []
        self.odom_data: list[tuple[float, np.ndarray]] = []

    def setup_subscribers(self):
        """创建 iceoryx2 订阅器。"""
        self.gt_sub = self.node.service_builder(iox2.ServiceName.new("Firefly/GroundTruth")) \
            .publish_subscribe(OdomMessage).user_header(TraceContext).open_or_create().subscriber_builder().create()
        self.odom_sub = self.node.service_builder(iox2.ServiceName.new("Firefly/Odometry")) \
            .publish_subscribe(OdomMessage).user_header(TraceContext).open_or_create().subscriber_builder().create()

    def start_sim(self):
        """启动 firefly-sim 子进程（--script 模式）。"""
        repo_root = self.vio_bin.parent.parent.parent
        # 使用 uv run 确保环境一致（安装了 firefly-mujoco 等依赖）
        uv_path = "/Users/flamingo/.local/bin/uv"
        env = os.environ.copy()
        env["PYTHONPATH"] = str(repo_root / "apps" / "firefly-sim" / "src")
        self.sim_proc = subprocess.Popen(
            [uv_path, "run", "firefly-sim", "--script"],
            cwd=repo_root,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(5.0)  # 等待 sim 启动并开始发布

    def start_vio(self):
        """启动 vio 子进程。"""
        env = os.environ.copy()
        env["RUST_LOG"] = "warn"
        self.vio_proc = subprocess.Popen(
            [str(self.vio_bin)],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(3.0)  # 等待 vio 连上 iceoryx2

    def wait_for_services(self, timeout: float = 15.0) -> bool:
        """等待 iceoryx2 服务就绪（IMU 话题出现）。"""
        from firefly_mujoco.messages import ImuMessage
        import iceoryx2 as iox2
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                sub = node.service_builder(iox2.ServiceName.new("Firefly/Imu")) \
                    .publish_subscribe(ImuMessage).user_header(TraceContext).open_or_create().subscriber_builder().create()
                # 尝试接收一次
                sub.receive()
                return True
            except Exception:
                pass
            time.sleep(0.5)
        return False

    def collect_async(self, timeout: float, gt_queue: asyncio.Queue, odom_queue: asyncio.Queue):
        """异步收集数据（在单独任务中运行）。"""
        async def _collect():
            t0 = time.time()
            while time.time() - t0 < timeout:
                # GT
                while (sample := self.gt_sub.receive()) is not None:
                    m = sample.payload().contents
                    await gt_queue.put((m.timestamp, np.array([m.position_x, m.position_y, m.position_z])))
                # Odom
                while (sample := self.odom_sub.receive()) is not None:
                    m = sample.payload().contents
                    pos = np.array([m.position_x, m.position_y, m.position_z])
                    if m.is_initialized and not np.any(np.isnan(pos)):
                        await odom_queue.put((m.timestamp, pos))
                    else:
                        # 诊断：记录被跳过的消息
                        import sys
                        print(f"  [odom skip] t={m.timestamp:.2f} init={m.is_initialized} pos={pos}", file=sys.stderr)
                await asyncio.sleep(0.02)
        return _collect()

    def stop(self):
        for proc, name in [(self.vio_proc, "vio"), (self.sim_proc, "sim")]:
            if proc:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


# ===== 实际测试 =====

@pytest.mark.skipif(not HAS_ICEORYX2, reason="iceoryx2 未安装")
@pytest.mark.parametrize("trajectory_data", list(TRAJECTORIES.keys()), indirect=True)
@pytest.mark.asyncio
async def test_vio_e2e(trajectory_data, build_vio_release, iceoryx2_node):
    """VIO 端到端测试：跑单条轨迹，验证 ATE/RPE 阈值。"""
    name = trajectory_data["name"]
    params = trajectory_data["params"]
    thresholds = trajectory_data["thresholds"]
    duration = trajectory_data["duration"]

    print(f"\n=== VIO E2E: {name} (duration={duration}s) ===")

    runner = VioE2ERunner(name, params, duration, build_vio_release, iceoryx2_node)

    # 1. 启动 sim
    print(f"  启动 sim (--script)...")
    runner.start_sim()

    # 2. 等待 sim 发布 IMU 话题（iceoryx2 服务就绪）
    print(f"  等待 iceoryx2 服务就绪...")
    if not runner.wait_for_services(timeout=15.0):
        pytest.fail(f"{name}: iceoryx2 服务未就绪 (Firefly/Imu 未出现)")

    # 3. 启动 vio（它会创建 Odometry 话题作为发布者）
    print(f"  启动 vio...")
    runner.start_vio()

    # 4. 多留一点时间让 vio 初始化完成（真值先验 + 首帧相机）
    print(f"  等待 vio 初始化...")
    time.sleep(3.0)

    # 5. 现在创建订阅器（vio 已创建 Odometry 话题，sim 已创建 GT 话题）
    print(f"  创建测试订阅器...")
    runner.setup_subscribers()

    # 6. 并发收集数据（sim_time 持续 duration 秒，wall-clock 多留 5s 余量）
    print(f"  收集数据 {duration}s...")
    gt_queue: asyncio.Queue = asyncio.Queue()
    odom_queue: asyncio.Queue = asyncio.Queue()
    collect_task = asyncio.create_task(runner.collect_async(duration + 5.0, gt_queue, odom_queue))
    await collect_task

    # 6. 停止进程
    runner.stop()

    # 打印 sim/vio stderr 用于调试
    if runner.sim_proc and runner.sim_proc.stderr:
        stderr = runner.sim_proc.stderr.read().decode(errors='ignore')
        if stderr:
            print(f"  SIM stderr: {stderr[:500]}")
    if runner.vio_proc and runner.vio_proc.stderr:
        stderr = runner.vio_proc.stderr.read().decode(errors='ignore')
        if stderr:
            print(f"  VIO stderr: {stderr[:500]}")

    # 7. 断言
    gt_list = []
    while not gt_queue.empty():
        gt_list.append(await gt_queue.get())
    odom_list = []
    while not odom_queue.empty():
        odom_list.append(await odom_queue.get())

    print(f"  GT 样本: {len(gt_list)}, Odom 样本: {len(odom_list)}")

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