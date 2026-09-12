#!/usr/bin/env python3
"""VIO bench 引擎：单轮 GT vs odom 评测（套件入口见 bench_suite.py）。

Bench keeps rerun enabled (viewer auto-spawn or connect). For many turns,
do NOT write all turns into a single rrd — use --save-dir to get per-run
isolated files: each run gets its own `{save_dir}/vio_{duration}s_{timestamp}.rrd`
via a dedicated viewer. Default (no --save-dir) is viewer-only, no file.

Usage:
  uv run python bench/bench_vio.py --duration 34
  uv run python bench/bench_vio.py --duration 34 --trajectory lissajous_wide
  uv run python bench/bench_vio.py --duration 34 --save-dir logs/bench
  uv run python bench/bench_vio.py --duration 10 --output logs/bench/bench_10s.json
  uv run python bench/bench_vio.py --duration 34 --turns 10

Replaces the former pytest e2e (apps/*/tests/test_vio_e2e.py) which was
not a true unit test — bench is the correct harness for VIO accuracy.

Logs & outputs go to repo-local paths (logs/bench/), never /tmp.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
# repo-local output (gitignored via /logs/)
DEFAULT_OUTPUT = REPO_ROOT / "logs" / "bench" / "vio_bench.json"
# ensure firefly_mujoco importable when run via `python` (not `uv run`)
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))
sys.path.insert(0, str(REPO_ROOT / "apps" / "firefly-sim" / "src"))

VIO_BIN_RELEASE = REPO_ROOT / "target" / "release" / "vio"
VIO_BIN_DEBUG = REPO_ROOT / "target" / "debug" / "vio"
GICP_BIN_RELEASE = REPO_ROOT / "target" / "release" / "gicp"
GICP_BIN_DEBUG = REPO_ROOT / "target" / "debug" / "gicp"
ALIKED_BIN_RELEASE = REPO_ROOT / "target" / "release" / "aliked"
ALIKED_BIN_DEBUG = REPO_ROOT / "target" / "debug" / "aliked"
LIGHTGLUE_BIN_RELEASE = REPO_ROOT / "target" / "release" / "lightglue"
LIGHTGLUE_BIN_DEBUG = REPO_ROOT / "target" / "debug" / "lightglue"
PLANNER_BIN_RELEASE = REPO_ROOT / "target" / "release" / "planner"
PLANNER_BIN_DEBUG = REPO_ROOT / "target" / "debug" / "planner"
#: 视觉库图（`lightglue --map` 必需；相对仓库根，与离线建库产物一致）
#: boxes 旧场景缺省；`wh_*` 轨迹由下表覆盖。
VISION_MAP = REPO_ROOT / "apps" / "planner" / "maps" / "straight_forward.ffvmap"
#: 轨迹→地图（gicp 静态图，lightglue 视觉库图）：场景与轨迹同源，
#: 检索/重定位才有意义；未列出的轨迹沿用旧行为（gicp 无图降级、视觉用上值）。
TRAJECTORY_MAPS: dict[str, tuple[Path | None, Path]] = {
    "wh_corridor": (
        REPO_ROOT / "apps" / "planner" / "maps" / "warehouse.ffmap",
        REPO_ROOT / "apps" / "planner" / "maps" / "wh_corridor.ffvmap",
    ),
}


def resolve_maps(
    trajectory: str | None,
    gicp_map: Path | str | None,
    vision_map: Path | str | None,
) -> tuple[Path | None, Path]:
    """地图决议：显式参数 > 轨迹查表 > 旧缺省（gicp 无图降级/visual 用 boxes 库图）。"""
    g, v = TRAJECTORY_MAPS.get(trajectory or "", (None, VISION_MAP))
    g = Path(gicp_map) if gicp_map else g
    v = Path(vision_map) if vision_map else v
    return g, v
UV_BIN = Path("/Users/flamingo/.local/bin/uv")
if not UV_BIN.exists():
    UV_BIN = Path("uv")  # fallback to PATH


def find_bin(name: str) -> Path:
    """release 优先、debug 回退的二进制定位（5 进程常态）。"""
    release = REPO_ROOT / "target" / "release" / name
    if release.exists():
        return release
    return REPO_ROOT / "target" / "debug" / name


def find_vio_bin() -> Path:
    return find_bin("vio")


def ensure_vio_built(vio_bin: Path) -> Path:
    if vio_bin.exists():
        return vio_bin
    print(f"[bench] building vio release ({vio_bin}) ...")
    cargo = Path("/Users/flamingo/.cargo/bin/cargo")
    if not cargo.exists():
        cargo = Path("cargo")
    r = subprocess.run(
        [str(cargo), "build", "-p", "vio", "--release"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    if r.returncode != 0:
        print(r.stderr, file=sys.stderr)
        sys.exit(1)
    return VIO_BIN_RELEASE


#: 清理匹配的进程模式（舰队＋sim＋可视化；`pkill -f` 全命令行匹配，
#: bench 自身命令行不含这些子串，误杀不了自己）。
CLEANUP_PATTERNS = [
    "firefly-sim",
    "firefly-viz",
    "target/release/vio", "target/debug/vio",
    "target/release/gicp", "target/debug/gicp",
    "target/release/aliked", "target/debug/aliked",
    "target/release/lightglue", "target/debug/lightglue",
    "target/release/planner", "target/debug/planner",
    "target/release/void", "target/debug/void",
]


def _live_pids() -> set[str]:
    """当前存活的目标进程 PID 集合（显式列出，不数数——空列表才是干净）。"""
    pids: set[str] = set()
    for pat in CLEANUP_PATTERNS:
        r = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True)
        pids.update(r.stdout.split())
    return pids


def cleanup_iceoryx() -> None:
    """全栈清理：SIGTERM → 轮询至零（幽灵发布端会污染话题/占槽位，
    `sleep 1` 碰运气不够，ORT 进程退出常需数秒）→ 超时 SIGKILL → 再轮询 →
    确认零残留后删 shm（删非空服务的 shm 等于制造幽灵，必须后删）。"""
    for pat in CLEANUP_PATTERNS:
        subprocess.run(["pkill", "-f", pat], capture_output=True)
    for _ in range(30):
        if not _live_pids():
            break
        time.sleep(1)
    else:
        for pid in _live_pids():
            subprocess.run(["kill", "-9", pid], capture_output=True)
        for _ in range(10):
            if not _live_pids():
                break
            time.sleep(1)
    survivors = _live_pids()
    if survivors:
        raise RuntimeError(f"iceoryx2 清理失败，残留进程：{sorted(survivors)}")
    # remove only iceoryx2 shm/services, not /tmp whole tree — and use repo-local cleanup
    for p in ["/tmp/iceoryx2", "/tmp/iceoryx2/services", "/tmp/iceoryx2/nodes"]:
        subprocess.run(["rm", "-rf", p], capture_output=True)
    # also clean stale shm files (macOS)
    subprocess.run(["bash", "-lc", "rm -rf /private/tmp/iox2*.shm_state 2>/dev/null; true"], capture_output=True)


def start_service(cmd: list[str], log_path: Path, name: str, wait_s: float = 3.0) -> "subprocess.Popen":
    """启动一个常态服务：死即报错（fail loudly），日志落 `logs/bench/`。

    日志级别：`RUST_LOG` 环境透传，缺省 `info`（融合接受/就绪门为 info，
    bench 后可据此统计；`warn` 下视觉观测计数无从查起）。
    """
    print(f"[bench] starting {name}: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        cwd=REPO_ROOT,
        env={**os.environ, "RUST_LOG": os.environ.get("RUST_LOG", "info")},
        stdout=open(log_path, "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(wait_s)
    if proc.poll() is not None:
        print(open(log_path).read()[-4000:], file=sys.stderr)
        raise RuntimeError(f"{name} died on start")
    return proc


def start_fleet(
    log_dir: Path,
    gicp_map: Path | None = None,
    vision_map: Path | None = None,
    with_planner: bool = True,
) -> dict[str, "subprocess.Popen"]:
    """常态舰队（gicp/aliked/lightglue/planner）逐个起、逐个验活（vio 除外）。

    ORT 模型加载慢（aliked/lightglue 约 10s），各自给足等待；返回进程表，
    调用方负责 terminate。vision 缺库图时 lightglue 起不来——直接报错，
    不静默降级（5 进程是常态，缺一即 bench 无效）。`gicp_map` 为空时不传
    `--map`（沿用旧行为：旧场景缺省图或空图降级）。
    """
    vision_map = vision_map or VISION_MAP
    # 注意：vio 不在这里起（调用方 `run_bench` 自有 vio，全局唯一状态源；
    # 双 vio 会向同一话题交错发布，采集混叠）。
    fleet: dict[str, "subprocess.Popen"] = {}
    gicp_cmd = [str(find_bin("gicp"))]
    if gicp_map is not None:
        gicp_cmd += ["--map", str(gicp_map)]
    fleet["gicp"] = start_service(gicp_cmd, log_dir / "gicp.log", "gicp")
    fleet["aliked"] = start_service(
        [str(find_bin("aliked"))], log_dir / "aliked.log", "aliked", wait_s=12.0
    )
    if not vision_map.is_file():
        raise RuntimeError(f"视觉库图缺失：{vision_map}（先跑离线建库，见 docs/how_to_run.md）")
    fleet["lightglue"] = start_service(
        [str(find_bin("lightglue")), "--map", str(vision_map)],
        log_dir / "lightglue.log",
        "lightglue",
        wait_s=12.0,
    )
    if with_planner:
        fleet["planner"] = start_service(
            [str(find_bin("planner"))], log_dir / "planner.log", "planner"
        )
    return fleet


def stop_fleet(fleet: dict[str, "subprocess.Popen"]) -> None:
    """优雅停止常态舰队（SIGTERM → 超时 KILL）。"""
    for name, p in fleet.items():
        if p is None:
            continue
        try:
            p.terminate()
            p.wait(timeout=5)
        except Exception:
            try:
                p.kill()
                p.wait(timeout=2)
            except Exception:
                print(f"[bench] WARN: {name} 未能停止", file=sys.stderr)


def interp_linear(x: np.ndarray, xp: np.ndarray, fp: np.ndarray) -> np.ndarray:
    """Linear interp fp(xp) -> x, xp sorted, fp (N,D). No scipy."""
    idx = np.searchsorted(xp, x)
    idx = np.clip(idx, 1, len(xp) - 1)
    x0 = xp[idx - 1]
    x1 = xp[idx]
    f0 = fp[idx - 1]
    f1 = fp[idx]
    denom = np.where(x1 != x0, x1 - x0, 1.0)
    w = ((x - x0) / denom)[:, None]
    return f0 + w * (f1 - f0)


def compute_metrics(
    gt_times: np.ndarray, gt_pos: np.ndarray, odom_times: np.ndarray, odom_pos: np.ndarray, duration: float
) -> dict:
    if len(gt_times) < 10 or len(odom_times) < 10:
        raise ValueError(f"not enough samples GT={len(gt_times)} odom={len(odom_times)}")
    # duration is relative to first GT sample (sim_time drifts ~8s wall offset)
    t0 = float(gt_times[0])
    mask = gt_times <= t0 + duration + 0.05
    gt_times = gt_times[mask]
    gt_pos = gt_pos[mask]
    if len(gt_times) < 10:
        raise ValueError(f"not enough samples after duration filter GT={len(gt_times)}")
    # keep only gt times covered by odom
    valid = (gt_times >= odom_times[0]) & (gt_times <= odom_times[-1])
    gt_times = gt_times[valid]
    gt_pos = gt_pos[valid]
    if len(gt_times) < 10:
        raise ValueError("no overlapping time after trim")
    odom_aligned = interp_linear(gt_times, odom_times, odom_pos)
    err = odom_aligned - gt_pos
    norm = np.linalg.norm(err, axis=1)
    ate_rmse = float(np.sqrt(np.mean(norm**2)))
    ate_mean = float(np.mean(norm))
    ate_max = float(np.max(norm))
    ate_final = float(norm[-1])
    # RPE delta 1s (10 frames @10Hz)
    rpe_rmse = 0.0
    rpe_mean = 0.0
    delta = 10
    if len(gt_pos) > delta:
        gt_rel = gt_pos[delta:] - gt_pos[:-delta]
        od_rel = odom_aligned[delta:] - odom_aligned[:-delta]
        rpe = np.linalg.norm(gt_rel - od_rel, axis=1)
        rpe_rmse = float(np.sqrt(np.mean(rpe**2)))
        rpe_mean = float(np.mean(rpe))
    # per-time snapshot (relative to t0)
    snapshots = {}
    for tt in [5, 10, 15, 20, 25, 30, 34]:
        if tt > duration:
            continue
        target = t0 + tt
        idx = int(np.argmin(np.abs(gt_times - target)))
        snapshots[str(tt)] = {
            "t": float(gt_times[idx]),
            "gt": gt_pos[idx].tolist(),
            "odom": odom_aligned[idx].tolist(),
            "err": err[idx].tolist(),
            "norm": float(norm[idx]),
        }
    # also at duration
    if str(int(duration)) not in snapshots:
        snapshots[str(int(duration))] = {
            "t": float(gt_times[-1]),
            "gt": gt_pos[-1].tolist(),
            "odom": odom_aligned[-1].tolist(),
            "err": err[-1].tolist(),
            "norm": float(norm[-1]),
        }
    return {
        "duration_s": float(duration),
        "num_frames": int(len(gt_times)),
        "ate_rmse": ate_rmse,
        "ate_mean": ate_mean,
        "ate_max": ate_max,
        "ate_final": ate_final,
        "rpe_rmse_1s": rpe_rmse,
        "rpe_mean_1s": rpe_mean,
        "snapshots": snapshots,
        "err_mean_xyz": err.mean(axis=0).tolist(),
        "err_std_xyz": err.std(axis=0).tolist(),
        "err_rmse_xyz": np.sqrt(np.mean(err**2, axis=0)).tolist(),
    }


def run_bench(
    duration: float,
    save_dir: Path | None,
    output: Path,
    trajectory: str | None = None,
    *,
    with_fleet: bool = True,
    with_planner: bool = False,
    gicp_map: Path | str | None = None,
    vision_map: Path | str | None = None,
) -> dict:
    """单轮 bench：sim + vio 精度（GT vs odom）+ 可选常态舰队。

    - `with_fleet=True`（缺省）：同时起 gicp/aliked/lightglue（planner 除外，
      见下），并采集 `Firefly/CorrectedOdometry`，输出 `corrected` 一套对照
      指标（同一 GT 时间轴，`ATE_corr_*`）——视觉/GICP 融合是否改善精度，
      一轮即见分晓。
    - `with_planner=True`：再起 planner（闭环参考；`--script` 模式下 sim
      忽略外部参考，仅验证 planner 存活与话题接线，不参与精度）。
    - `WITH_VOID=1`：沿用旧语义，额外起 void（A/B 对比，不影响互锁话题选择）。
    """
    import iceoryx2 as iox2
    from firefly_mujoco.messages import ImuMessage, OdomMessage, TraceContext

    vio_bin = ensure_vio_built(find_vio_bin())
    gicp_map_resolved, vision_map_resolved = resolve_maps(trajectory, gicp_map, vision_map)
    print(f"[bench] vio_bin={vio_bin} trajectory={trajectory or 'lissajous_classic'} fleet={with_fleet}")
    print(f"[bench] maps: gicp={gicp_map_resolved} vision={vision_map_resolved}")

    cleanup_iceoryx()
    log_dir = REPO_ROOT / "logs" / "bench"
    log_dir.mkdir(parents=True, exist_ok=True)

    # 可视化统一写入：per-turn 独立 rrd，一律经 `firefly-viz`（`Firefly/Viz` +
    # `Firefly/Log` 的唯一消费者；裸 `rerun --save` 收不到数据，落空文件）。
    # firefly-viz 必须最先启动（先创建 `Firefly/Log` 服务定上限，见 runbook）。
    # - save_dir 有值：每轮落各自时间戳文件（永不单文件）。
    # - save_dir 为空（单轮）：不自起 viz（外部共享 viewer 模式，无文件）。
    viz_proc = None
    rrd_path = None
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        # per-turn isolated file: vio_34s_<turn>_<ts>.rrd — caller may include turn in output name,
        # but we ensure uniqueness per invocation
        # Use output stem to make rrd name traceable to turn
        turn_hint = output.stem  # e.g. turn_01
        rrd_path = save_dir / f"{turn_hint}_{int(duration)}s_{ts}.rrd"
        print(f"[bench] per-run rrd -> {rrd_path} (dedicated firefly-viz, isolated per turn)")
        # 本轮清理已在入口 `cleanup_iceoryx` 做完（轮询至零＋删 shm），此处不重复
        # 杀/删（删 shm 只允许在零残留后做一次）。
        viz_log = log_dir / "viz.log"
        viz_proc = subprocess.Popen(
            [str(UV_BIN), "run", "firefly-viz", "--save", str(rrd_path)],
            cwd=REPO_ROOT,
            stdout=open(viz_log, "w"),
            stderr=subprocess.STDOUT,
        )
        time.sleep(4)  # viz 建服务 + 订阅就绪（Rust 发布端只 open，先起方定上限）
        if viz_proc.poll() is not None:
            print(open(viz_log).read()[-2000:], file=sys.stderr)
            print("[bench] firefly-viz failed to start, continuing without rrd", file=sys.stderr)
            viz_proc = None
            rrd_path = None
    else:
        print("[bench] no firefly-viz (no file) — single-turn mode without rrd")

    # 启动顺序（采集窗口从任务 0 开始的关键）：viz → vio → fleet → sim。
    # sim 最后起：任务时钟在 vio ready 后才走，fleet（尤其 ORT 的 ~24s 加载）
    # 在任务开始前全部就绪——旧顺序（sim 最先）下采集从任务 ~35s 才开始，
    # 去程永远测不到。各进程无输入时空转等待（vio GT 等待 120s 内，sim 未起
    # 即无数据），顺序反转无死锁。
    # VIO_CONFIG env overrides --config (A/B experiment configs, e.g. logs/bench/voxel.toml)
    vio_cmd = [str(vio_bin)]
    vio_config = os.environ.get("VIO_CONFIG")
    if vio_config:
        vio_cmd += ["--config", vio_config]
    vio_proc = start_service(vio_cmd, log_dir / "vio.log", "vio")
    log_vio = REPO_ROOT / "logs" / "bench" / "vio.log"

    # 常态舰队（vio 之后、sim 之前起：只消费不阻塞，sim 互锁 latch 的是 vio ready）。
    fleet: dict[str, "subprocess.Popen"] = {}
    if with_fleet:
        fleet = start_fleet(log_dir, gicp_map_resolved, vision_map_resolved, with_planner)

    # sim 最后起（headless，--no-trace 关 OTel，1x realtime）。
    env = os.environ.copy()
    # ensure clean PYTHONPATH for uv run; --script [NAME] selects trajectory instance
    sim_cmd = [str(UV_BIN), "run", "firefly-sim", "--script"]
    if trajectory:
        sim_cmd.append(trajectory)
    sim_cmd.append("--no-trace")
    log_sim = REPO_ROOT / "logs" / "bench" / "sim.log"
    log_sim.parent.mkdir(parents=True, exist_ok=True)
    print(f"[bench] starting sim: {' '.join(sim_cmd)}")
    sim_proc = subprocess.Popen(
        sim_cmd,
        cwd=REPO_ROOT,
        env=env,
        stdout=open(log_sim, "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(5)
    if sim_proc.poll() is not None:
        print(open(log_sim).read()[-4000:], file=sys.stderr)
        raise RuntimeError("sim died on start")

    # void 里程计（WITH_VOID=1 时）：--script 任务时钟等它的 ready 电平，
    # 必须在 sim 15s 启动超时前就绪，故紧随 sim 启动。
    # 注意：sim 互锁缺省监听 Firefly/Odometry（vio），WITH_VOID 场景需
    # sim 侧 --odom-topic Firefly/VoidOdom 才 latch 到 void（见 main.py）。
    void_proc = None
    if os.environ.get("WITH_VOID"):
        void_bin = find_bin("void")
        log_void = REPO_ROOT / "logs" / "bench" / "void.log"
        void_proc = start_service([str(void_bin)], log_void, "void")

    # wait for IMU topic to appear
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    t0 = time.time()
    imu_ready = False
    while time.time() - t0 < 15:
        try:
            _sub = (
                node.service_builder(iox2.ServiceName.new("Firefly/Imu"))
                .publish_subscribe(ImuMessage)
                .user_header(TraceContext)
                .open_or_create()
                .subscriber_builder()
                .create()
            )
            imu_ready = True
            break
        except Exception:
            time.sleep(0.5)
    if not imu_ready:
        print("[bench] WARN: IMU topic not ready", file=sys.stderr)
    if sim_proc.poll() is not None:
        print(open(log_sim).read()[-4000:], file=sys.stderr)
        raise RuntimeError("sim died after wait")

    # subscribers AFTER vio creates Odometry topic（fleet 就绪后建，双采集）
    node2 = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    gt_sub = (
        node2.service_builder(iox2.ServiceName.new("Firefly/GroundTruth"))
        .publish_subscribe(OdomMessage)
        .user_header(TraceContext)
        .open_or_create()
        .subscriber_builder()
        .create()
    )
    odom_sub = (
        node2.service_builder(iox2.ServiceName.new("Firefly/Odometry"))
        .publish_subscribe(OdomMessage)
        .user_header(TraceContext)
        .open_or_create()
        .subscriber_builder()
        .create()
    )
    corrected_sub = None
    if with_fleet:
        corrected_sub = (
            node2.service_builder(iox2.ServiceName.new("Firefly/CorrectedOdometry"))
            .publish_subscribe(OdomMessage)
            .user_header(TraceContext)
            .open_or_create()
            .subscriber_builder()
            .create()
        )
    print(f"[bench] collecting {duration}s (sim_time 0..{duration}) ...")
    gt_list: list[tuple[float, np.ndarray]] = []
    odom_list: list[tuple[float, np.ndarray]] = []
    corrected_list: list[tuple[float, np.ndarray]] = []

    async def collect(timeout: float):
        t_wall0 = time.time()
        while time.time() - t_wall0 < timeout:
            while (s := gt_sub.receive()) is not None:
                m = s.payload().contents
                gt_list.append((float(m.timestamp), np.array([m.position_x, m.position_y, m.position_z], dtype=float)))
            while (s := odom_sub.receive()) is not None:
                m = s.payload().contents
                pos = np.array([m.position_x, m.position_y, m.position_z], dtype=float)
                if bool(m.is_initialized) and np.all(np.isfinite(pos)):
                    odom_list.append((float(m.timestamp), pos))
            if corrected_sub is not None:
                while (s := corrected_sub.receive()) is not None:
                    m = s.payload().contents
                    pos = np.array([m.position_x, m.position_y, m.position_z], dtype=float)
                    if bool(m.is_initialized) and np.all(np.isfinite(pos)):
                        corrected_list.append((float(m.timestamp), pos))
            await asyncio.sleep(0.02)

    # sim_time duration + grace for odom lag (2s)
    asyncio.run(collect(duration + 4))

    stop_fleet(fleet)
    for p in [vio_proc, sim_proc, void_proc]:
        if p is None:
            continue
        try:
            p.terminate()
            p.wait(timeout=5)
        except Exception:
            try:
                p.kill()
                p.wait(timeout=2)
            except Exception:
                pass
    if viz_proc is not None:
        try:
            viz_proc.terminate()
            viz_proc.wait(timeout=5)
        except Exception:
            try:
                viz_proc.kill()
                viz_proc.wait(timeout=2)
            except Exception:
                pass
        # force kill any remaining viz/viewer (avoid single-rrd mixing)
        for pat in ["firefly-viz", "rerun", "Rerun"]:
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        time.sleep(0.5)

    print(f"[bench] collected GT {len(gt_list)} odom {len(odom_list)} corrected {len(corrected_list)}")
    if len(gt_list) < 10 or len(odom_list) < 10:
        print(open(log_vio).read()[-4000:], file=sys.stderr)
        raise RuntimeError(f"not enough data GT={len(gt_list)} odom={len(odom_list)}")

    gt_times = np.array([t for t, _ in gt_list])
    gt_pos = np.vstack([p for _, p in gt_list])
    odom_times = np.array([t for t, _ in odom_list])
    odom_pos = np.vstack([p for _, p in odom_list])
    print(f"[bench] GT t [{gt_times[0]:.2f},{gt_times[-1]:.2f}] odom t [{odom_times[0]:.2f},{odom_times[-1]:.2f}]")

    metrics = compute_metrics(gt_times, gt_pos, odom_times, odom_pos, duration)
    # 融合对照：同一 GT 时间轴上的 corrected 精度（fleet 缺席/无样本时为 None）
    corrected_metrics: dict | None = None
    if len(corrected_list) >= 10:
        corr_times = np.array([t for t, _ in corrected_list])
        corr_pos = np.vstack([p for _, p in corrected_list])
        try:
            corrected_metrics = compute_metrics(gt_times, gt_pos, corr_times, corr_pos, duration)
        except ValueError as e:
            print(f"[bench] WARN: corrected metrics skipped ({e})", file=sys.stderr)

    # pretty print
    print("\n=== VIO bench ===")
    print(f" duration {metrics['duration_s']:.1f}s  frames {metrics['num_frames']}")
    print(f" ATE RMSE {metrics['ate_rmse']:.3f}  mean {metrics['ate_mean']:.3f}  max {metrics['ate_max']:.3f}  final {metrics['ate_final']:.3f}")
    print(f" RPE 1s RMSE {metrics['rpe_rmse_1s']:.3f}  mean {metrics['rpe_mean_1s']:.3f}")
    if corrected_metrics is not None:
        print(
            f" CORR RMSE {corrected_metrics['ate_rmse']:.3f}  mean {corrected_metrics['ate_mean']:.3f} "
            f" max {corrected_metrics['ate_max']:.3f}  final {corrected_metrics['ate_final']:.3f} "
            f"(ΔRMSE {metrics['ate_rmse'] - corrected_metrics['ate_rmse']:+.3f})"
        )
    print(f" err mean xyz {np.array(metrics['err_mean_xyz']).round(3)}  std {np.array(metrics['err_std_xyz']).round(3)}  rmse {np.array(metrics['err_rmse_xyz']).round(3)}")
    for k in sorted(metrics["snapshots"], key=lambda x: float(x)):
        s = metrics["snapshots"][k]
        print(f"  t={k:>3s}s |{s['norm']:7.2f}|m  err {np.array(s['err']).round(2)}  GT {np.array(s['gt']).round(2)} odom {np.array(s['odom']).round(2)}")

    # save repo-local (never /tmp)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": metrics["duration_s"],
        "trajectory": trajectory or "lissajous_classic",
        "vio_bin": str(vio_bin),
        "save_dir": str(save_dir) if save_dir else None,
        "metrics": metrics,
        "corrected_metrics": corrected_metrics,
        "fleet": with_fleet,
        "gicp_map": str(gicp_map_resolved) if gicp_map_resolved else None,
        "vision_map": str(vision_map_resolved),
        "logs": {
            "sim": str(log_sim),
            "vio": str(log_vio),
            "rrd": str(rrd_path) if rrd_path else None,
        },
    }
    with open(output, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[bench] saved -> {output} (repo-local, not tmpfs)")
    if save_dir is not None:
        print(f"[bench] per-run rrd in {save_dir} (one file per turn, never single rrd)")
    print(f"[bench] logs: {log_sim} {log_vio}")
    return payload


def main():
    ap = argparse.ArgumentParser(description="VIO bench: GT vs odom (rerun kept, per-run rrd isolation)")
    ap.add_argument("--duration", type=float, default=34.0, help="sim duration seconds (default 34)")
    ap.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="json output (repo-local, default logs/bench/vio_bench.json)")
    ap.add_argument("--save-dir", type=Path, default=None, help="if set, launch dedicated viewer saving per-run isolated rrd to this dir (one file per turn, never single rrd)")
    ap.add_argument("--turns", type=int, default=1, help="number of turns to run sequentially (default 1); each turn is isolated, per-turn rrd not single file")
    ap.add_argument("--trajectory", type=str, default=None, help="trajectory instance name (see firefly_sim.trajectories.TRAJECTORIES; default lissajous_classic)")
    ap.add_argument("--no-fleet", action="store_true", help="只起 sim+vio（旧语义；缺省起常态舰队 gicp/aliked/lightglue 并采 corrected 对照）")
    ap.add_argument("--with-planner", action="store_true", help="再起 planner（仅验证存活与接线；--script 模式下 sim 忽略外部参考）")
    ap.add_argument("--gicp-map", type=Path, default=None, help="gicp 静态地图（缺省按轨迹查表；boxes 旧行为=不传，降级）")
    ap.add_argument("--vision-map", type=Path, default=None, help="lightglue 视觉库图（缺省按轨迹查表；旧缺省 straight_forward 库图）")
    args = ap.parse_args()
    try:
        if args.turns <= 1:
            run_bench(
                float(args.duration),
                args.save_dir,
                Path(args.output),
                trajectory=args.trajectory,
                with_fleet=not args.no_fleet,
                with_planner=args.with_planner,
                gicp_map=args.gicp_map,
                vision_map=args.vision_map,
            )
        else:
            turns = int(args.turns)
            print(f"[bench] running {turns} turns x {args.duration}s (each isolated, per-run rrd)")
            # For many turns, default to per-turn isolated rrds in logs/bench
            # (never single shared rrd). If user explicitly passes --save-dir, use it;
            # otherwise auto-enable per-turn saving to logs/bench.
            effective_save_dir = args.save_dir
            if effective_save_dir is None:
                effective_save_dir = REPO_ROOT / "logs" / "bench"
                print(f"[bench] turns>1: auto per-turn rrd -> {effective_save_dir} (isolated, not single file)")
            results = []
            for i in range(1, turns + 1):
                print(f"\n{'='*20} TURN {i}/{turns} {'='*20}")
                # per-turn output: logs/bench/turn_XX.json if --output is default or dir
                out = Path(args.output)
                if turns > 1:
                    # if output is logs/bench/vio_bench.json -> turn into turn_XX.json
                    # if user gave explicit file, we still split per turn
                    stem = out.stem
                    suffix = out.suffix
                    parent = out.parent
                    out = parent / f"{stem}_turn_{i:02d}{suffix}" if stem != "vio_bench" else parent / f"turn_{i:02d}{suffix}"
                    # nicer: logs/bench/turn_01.json
                    if out.name.startswith("vio_bench"):
                        out = out.parent / f"turn_{i:02d}{suffix}"
                payload = run_bench(
                    float(args.duration),
                    effective_save_dir,
                    out,
                    trajectory=args.trajectory,
                    with_fleet=not args.no_fleet,
                    with_planner=args.with_planner,
                    gicp_map=args.gicp_map,
                    vision_map=args.vision_map,
                )
                results.append(payload)
                time.sleep(1)
            # summary table (repo-local, no tmpfs)
            print("\n" + "=" * 80)
            print(f"{turns}-turn {args.duration:.0f}s bench summary")
            print("=" * 80)
            print(f"{'turn':>4} {'ATE_RMSE':>9} {'ATE_mean':>9} {'ATE_max':>9} {'ATE_final':>10} {'RPE_1s':>7} {'err@34s':>8} {'frames':>6}")
            for idx, payload in enumerate(results, 1):
                m = payload["metrics"]
                snap = m["snapshots"].get("34") or m["snapshots"].get(str(int(args.duration))) or {}
                err34 = snap.get("norm", m["ate_final"])
                print(f"{idx:4d} {m['ate_rmse']:9.1f} {m['ate_mean']:9.1f} {m['ate_max']:9.1f} {m['ate_final']:10.1f} {m['rpe_rmse_1s']:7.1f} {err34:8.1f} {m['num_frames']:6d}")
                cm = payload.get("corrected_metrics")
                if cm is not None:
                    print(
                        f"  corr{cm['ate_rmse']:9.1f} {cm['ate_mean']:9.1f} {cm['ate_max']:9.1f} "
                        f"{cm['ate_final']:10.1f} (ΔRMSE {m['ate_rmse'] - cm['ate_rmse']:+.3f})"
                    )
            # aggregated stats
            ates = np.array([r["metrics"]["ate_rmse"] for r in results])
            finals = np.array([r["metrics"]["ate_final"] for r in results])
            print("-" * 80)
            print(f" avg  {ates.mean():9.1f} {np.array([r['metrics']['ate_mean'] for r in results]).mean():9.1f} {'':9} {finals.mean():10.1f} {np.array([r['metrics']['rpe_rmse_1s'] for r in results]).mean():7.1f}")
            print(f" std  {ates.std():9.1f} {'':9} {'':9} {finals.std():10.1f}")
            # save summary
            summary_path = Path(args.output).parent / "summary.json" if turns > 1 else Path(args.output)
            if turns > 1:
                summary_path = Path(args.output).parent / f"summary_{turns}x{int(args.duration)}s.json"
                with open(summary_path, "w") as f:
                    json.dump({"turns": turns, "duration_s": float(args.duration), "results": results}, f, indent=2)
                print(f"\n[bench] summary -> {summary_path} (repo-local)")
            # also show rrd isolation note
            if args.save_dir is not None:
                print(f"[bench] per-turn rrds in {args.save_dir} (one file per turn, never single rrd)")
            else:
                print(f"[bench] rerun viewer kept (shared, no file); for many turns use --save-dir for per-turn isolated rrds")
    except KeyboardInterrupt:
        print("\n[bench] interrupted, cleaning up...")
        cleanup_iceoryx()
        sys.exit(130)


if __name__ == "__main__":
    main()
