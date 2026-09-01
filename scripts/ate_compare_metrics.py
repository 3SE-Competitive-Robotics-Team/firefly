#!/usr/bin/env python3
"""ATE 口径对比：同一数据上 Umeyama 相似变换 vs 首点对齐 vs 绝对误差。

时间对齐逻辑与 run_void_e2e.sh 内嵌 ATE 完全一致（GT 为基准线性插值、
共同时间窗、终点截到 record_end），只替换对齐算法，量化"虚低"多少。

用法：uv run python scripts/ate_compare_metrics.py <run_dir>...
"""
import sys

import numpy as np


def load(run_dir: str):
    gt = np.load(f"{run_dir}/gt.npy")
    gt_t = np.load(f"{run_dir}/gt_t.npy")
    odom = np.load(f"{run_dir}/odom.npy")
    odom_t = np.load(f"{run_dir}/odom_t.npy")
    t_end_rec = 0.0
    try:
        with open(f"{run_dir}/record_end.txt") as f:
            t_end_rec = float(f.read().strip())
    except FileNotFoundError:
        pass
    t0 = max(gt_t.min(), odom_t.min()) + 0.1
    t1 = min(gt_t.max(), odom_t.max(), t_end_rec if t_end_rec > 0 else float("inf")) - 0.1
    if t1 <= t0:
        return None
    mask = (gt_t >= t0) & (gt_t <= t1)
    gt_w = gt[mask]
    gt_tw = gt_t[mask]
    odom_w = np.stack(
        [np.interp(gt_tw, odom_t, odom[:, k]) for k in range(3)], axis=1
    )
    return gt_w, odom_w


def umeyama(a, b):
    """相似变换 s·R·a + t ≈ b，返回对齐后的 a'。"""
    A, B = a.T, b.T
    mu_a = A.mean(axis=1, keepdims=True)
    mu_b = B.mean(axis=1, keepdims=True)
    aa = A - mu_a
    bb = B - mu_b
    h = aa @ bb.T
    u, s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    scale = np.trace(np.diag(s) @ r) / np.sum(aa * aa)
    t_vec = mu_b - scale * r @ mu_a
    return scale * r @ A + t_vec, scale


def metrics(a, b):
    err = np.linalg.norm(a - b, axis=1)
    return np.sqrt(np.mean(err**2)), np.mean(err), np.max(err)


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    print(f"{'run':<28} {'Umeyama-RMS':>12} {'首点对齐-RMS':>13} {'绝对误差-RMS':>13} "
          f"{'绝对-mean':>10} {'虚低系数':>8}")
    for run_dir in sys.argv[1:]:
        loaded = load(run_dir)
        if loaded is None:
            print(f"{run_dir:<28} 无共同时间窗")
            continue
        gt_w, odom_w = loaded
        # 绝对误差（无对齐）
        abs_rms, abs_mean, _ = metrics(odom_w, gt_w)
        # 首点对齐（平移使首点重合）
        first_aligned = odom_w - odom_w[0] + gt_w[0]
        fa_rms, fa_mean, _ = metrics(first_aligned, gt_w)
        # Umeyama
        u_aligned, scale = umeyama(odom_w, gt_w)
        u_rms, _, _ = metrics(u_aligned.T, gt_w)
        ratio = u_rms / fa_rms if fa_rms > 1e-9 else float("nan")
        # 三维分项（首点对齐下）
        d = first_aligned - gt_w
        print(f"{run_dir:<28} {u_rms:>12.4f} {fa_rms:>13.4f} {abs_rms:>13.4f} "
              f"{abs_mean:>10.4f} {ratio:>7.2f}x")
        print(f"{'':<28}  Δx={np.mean(np.abs(d[:,0])):.3f}  Δy={np.mean(np.abs(d[:,1])):.3f}  "
              f"Δz={np.mean(np.abs(d[:,2])):.3f}  (首点对齐分项)")


if __name__ == "__main__":
    main()
