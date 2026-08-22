#!/usr/bin/env python3
"""VIO bench: GT vs odom for 34s (scripted Lissajous) — no single-rrd, no tmpfs.

Bench keeps rerun enabled (viewer auto-spawn or connect). For many turns,
do NOT write all turns into a single rrd — use --save-dir to get per-run
isolated files: each run gets its own `{save_dir}/vio_{duration}s_{timestamp}.rrd`
via a dedicated viewer. Default (no --save-dir) is viewer-only, no file.

Usage:
  uv run python bench/bench_vio.py --duration 34
  uv run python bench/bench_vio.py --duration 34 --save-dir logs/bench
  uv run python bench/bench_vio.py --duration 10 --output logs/bench/bench_10s.json
  uv run python bench/bench_vio.py --duration 34 --turns 10
  uv run python bench/bench_vio.py --duration 34 --turns 10 --save-dir logs/bench

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
UV_BIN = Path("/Users/flamingo/.local/bin/uv")
if not UV_BIN.exists():
    UV_BIN = Path("uv")  # fallback to PATH


def find_vio_bin() -> Path:
    if VIO_BIN_RELEASE.exists():
        return VIO_BIN_RELEASE
    if VIO_BIN_DEBUG.exists():
        return VIO_BIN_DEBUG
    return VIO_BIN_RELEASE


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


def cleanup_iceoryx() -> None:
    # remove only iceoryx2 shm/services, not /tmp whole tree — and use repo-local cleanup
    for p in ["/tmp/iceoryx2", "/tmp/iceoryx2/services", "/tmp/iceoryx2/nodes"]:
        subprocess.run(["rm", "-rf", p], capture_output=True)
    # also clean stale shm files (macOS)
    subprocess.run(["bash", "-lc", "rm -rf /private/tmp/iox2*.shm_state 2>/dev/null; true"], capture_output=True)
    # kill previous sim/vio/demo (graceful SIGTERM)
    for pat in ["firefly-sim", "target/release/vio", "target/debug/vio", "target/debug/firefly-demo"]:
        subprocess.run(["pkill", "-f", pat], capture_output=True)
    time.sleep(1)


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


def run_bench(duration: float, save_dir: Path | None, output: Path) -> dict:
    import iceoryx2 as iox2
    from firefly_mujoco.messages import ImuMessage, OdomMessage, TraceContext

    vio_bin = ensure_vio_built(find_vio_bin())
    print(f"[bench] vio_bin={vio_bin}")

    cleanup_iceoryx()

    # Rerun handling: per-run isolated rrd, never single shared file.
    # - If save_dir is given, each turn saves to its own timestamped file there.
    # - If save_dir is None and this is a single turn, reuse shared viewer (no file).
    # - If save_dir is None but turns>1 (caller handles), bench caller will have
    #   already set a per-turn save_dir; this function just honors what is passed.
    viewer_proc = None
    rrd_path = None
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        # per-turn isolated file: vio_34s_<turn>_<ts>.rrd — caller may include turn in output name,
        # but we ensure uniqueness per invocation
        # Use output stem to make rrd name traceable to turn
        turn_hint = output.stem  # e.g. turn_01
        rrd_path = save_dir / f"{turn_hint}_{int(duration)}s_{ts}.rrd"
        print(f"[bench] per-run rrd -> {rrd_path} (dedicated viewer, isolated per turn)")
        # Kill any stale viewer that would otherwise capture data into single stream
        # (bench turns must not share a viewer that writes to one file)
        for pat in ["rerun", "Rerun"]:
            subprocess.run(["pkill", "-f", pat], capture_output=True)
        # ensure port freed
        time.sleep(1.5)
        # also clean iceoryx2 viewer shm
        subprocess.run(["bash", "-lc", "rm -rf /tmp/iceoryx2 2>/dev/null; true"], capture_output=True)
        viewer_proc = subprocess.Popen(
            ["rerun", "--save", str(rrd_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(3)  # let viewer bind 9876
        if viewer_proc.poll() is not None:
            print(f"[bench] viewer failed to start, continuing with existing/shared viewer", file=sys.stderr)
            viewer_proc = None
            rrd_path = None
    else:
        print("[bench] rerun viewer: shared (no file) — single-turn mode")

    # start sim headless (--no-trace disables OTel, 1x realtime)
    env = os.environ.copy()
    # ensure clean PYTHONPATH for uv run
    sim_cmd = [str(UV_BIN), "run", "firefly-sim", "--script", "--no-trace"]
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

    # start vio — keep rerun enabled (connect_or_spawn will reuse viewer or spawn)
    env_vio = os.environ.copy()
    env_vio["RUST_LOG"] = os.environ.get("RUST_LOG", "warn")
    log_vio = REPO_ROOT / "logs" / "bench" / "vio.log"
    print(f"[bench] starting vio: {vio_bin}")
    vio_proc = subprocess.Popen(
        [str(vio_bin)],
        env=env_vio,
        stdout=open(log_vio, "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(3)
    if vio_proc.poll() is not None:
        print(open(log_vio).read()[-4000:], file=sys.stderr)
        sim_proc.terminate()
        raise RuntimeError("vio died on start")

    # subscribers AFTER vio creates Odometry topic
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
    print(f"[bench] collecting {duration}s (sim_time 0..{duration}) ...")
    gt_list: list[tuple[float, np.ndarray]] = []
    odom_list: list[tuple[float, np.ndarray]] = []

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
            await asyncio.sleep(0.02)

    # sim_time duration + grace for odom lag (2s)
    asyncio.run(collect(duration + 4))

    for p in [vio_proc, sim_proc]:
        try:
            p.terminate()
            p.wait(timeout=5)
        except Exception:
            try:
                p.kill()
                p.wait(timeout=2)
            except Exception:
                pass
    if viewer_proc is not None:
        try:
            viewer_proc.terminate()
            viewer_proc.wait(timeout=5)
        except Exception:
            try:
                viewer_proc.kill()
                viewer_proc.wait(timeout=2)
            except Exception:
                pass
        # force kill any remaining rerun (avoid single-rrd mixing)
        for pat in ["rerun", "Rerun"]:
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        time.sleep(0.5)

    print(f"[bench] collected GT {len(gt_list)} odom {len(odom_list)}")
    if len(gt_list) < 10 or len(odom_list) < 10:
        print(open(log_vio).read()[-4000:], file=sys.stderr)
        raise RuntimeError(f"not enough data GT={len(gt_list)} odom={len(odom_list)}")

    gt_times = np.array([t for t, _ in gt_list])
    gt_pos = np.vstack([p for _, p in gt_list])
    odom_times = np.array([t for t, _ in odom_list])
    odom_pos = np.vstack([p for _, p in odom_list])
    print(f"[bench] GT t [{gt_times[0]:.2f},{gt_times[-1]:.2f}] odom t [{odom_times[0]:.2f},{odom_times[-1]:.2f}]")

    metrics = compute_metrics(gt_times, gt_pos, odom_times, odom_pos, duration)

    # pretty print
    print("\n=== VIO bench ===")
    print(f" duration {metrics['duration_s']:.1f}s  frames {metrics['num_frames']}")
    print(f" ATE RMSE {metrics['ate_rmse']:.3f}  mean {metrics['ate_mean']:.3f}  max {metrics['ate_max']:.3f}  final {metrics['ate_final']:.3f} (km-scale right now)")
    print(f" RPE 1s RMSE {metrics['rpe_rmse_1s']:.3f}  mean {metrics['rpe_mean_1s']:.3f}")
    print(f" err mean xyz {np.array(metrics['err_mean_xyz']).round(3)}  std {np.array(metrics['err_std_xyz']).round(3)}  rmse {np.array(metrics['err_rmse_xyz']).round(3)}")
    for k in sorted(metrics["snapshots"], key=lambda x: float(x)):
        s = metrics["snapshots"][k]
        print(f"  t={k:>3s}s |{s['norm']:7.2f}|m  err {np.array(s['err']).round(2)}  GT {np.array(s['gt']).round(2)} odom {np.array(s['odom']).round(2)}")

    # save repo-local (never /tmp)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": metrics["duration_s"],
        "vio_bin": str(vio_bin),
        "save_dir": str(save_dir) if save_dir else None,
        "metrics": metrics,
        "logs": {"sim": str(log_sim), "vio": str(log_vio), "rrd": str(save_dir) if save_dir else None},
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
    args = ap.parse_args()
    try:
        if args.turns <= 1:
            run_bench(float(args.duration), args.save_dir, Path(args.output))
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
                payload = run_bench(float(args.duration), effective_save_dir, out)
                results.append(payload)
                time.sleep(1)
            # summary table (repo-local, no tmpfs)
            print("\n" + "=" * 80)
            print(f"{turns}-turn {args.duration:.0f}s bench summary (km-scale right now)")
            print("=" * 80)
            print(f"{'turn':>4} {'ATE_RMSE':>9} {'ATE_mean':>9} {'ATE_max':>9} {'ATE_final':>10} {'RPE_1s':>7} {'err@34s':>8} {'frames':>6}")
            for idx, payload in enumerate(results, 1):
                m = payload["metrics"]
                snap = m["snapshots"].get("34") or m["snapshots"].get(str(int(args.duration))) or {}
                err34 = snap.get("norm", m["ate_final"])
                print(f"{idx:4d} {m['ate_rmse']:9.1f} {m['ate_mean']:9.1f} {m['ate_max']:9.1f} {m['ate_final']:10.1f} {m['rpe_rmse_1s']:7.1f} {err34:8.1f} {m['num_frames']:6d}")
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
