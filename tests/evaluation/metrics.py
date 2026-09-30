"""固定尺度的航向/平移对齐与 ATE/RPE 评测；结果不得回流估计器。"""

import numpy as np


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
    gt_times: np.ndarray, gt_pos: np.ndarray, odom_times: np.ndarray, odom_pos: np.ndarray, duration: float,
    *, alignment: str = "yaw_translation",
) -> dict:
    """时间插值后评测；航向/平移只作用于副本，尺度固定，不反馈到算法。"""
    for name, times, positions in [("GT", gt_times, gt_pos), ("odom", odom_times, odom_pos)]:
        if times.ndim != 1 or positions.shape != (len(times), 3):
            raise ValueError(f"invalid {name} array shape")
        if not np.all(np.isfinite(times)) or not np.all(np.isfinite(positions)):
            raise ValueError(f"non-finite {name} samples")
        if np.any(np.diff(times) <= 0):
            raise ValueError(f"{name} timestamps must strictly increase")
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive and finite")
    if len(gt_times) < 10 or len(odom_times) < 10:
        raise ValueError(f"not enough samples GT={len(gt_times)} odom={len(odom_times)}")
    # duration is relative to the first GT sample.
    t0 = float(gt_times[0])
    mask = gt_times <= t0 + duration
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
    raw_err = odom_aligned - gt_pos
    if alignment == "yaw_translation":
        est_center = odom_aligned.mean(axis=0)
        gt_center = gt_pos.mean(axis=0)
        est_delta = odom_aligned - est_center
        gt_delta = gt_pos - gt_center
        dot = np.sum(est_delta[:, 0] * gt_delta[:, 0] + est_delta[:, 1] * gt_delta[:, 1])
        cross = np.sum(est_delta[:, 0] * gt_delta[:, 1] - est_delta[:, 1] * gt_delta[:, 0])
        yaw = float(np.arctan2(cross, dot))
        c, sn = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
        translation = gt_center - rotation @ est_center
        odom_aligned = odom_aligned @ rotation.T + translation
    elif alignment == "none":
        rotation = np.eye(3)
        translation = np.zeros(3)
    else:
        raise ValueError(f"unknown evaluation alignment: {alignment}")
    err = odom_aligned - gt_pos
    norm = np.linalg.norm(err, axis=1)
    ate_rmse = float(np.sqrt(np.mean(norm**2)))
    ate_mean = float(np.mean(norm))
    ate_max = float(np.max(norm))
    ate_final = float(norm[-1])
    # 按测量时间配对 t 与 t+1s，不假设采样频率；不足 1 秒不报告零误差。
    eligible = gt_times + 1.0 <= gt_times[-1] + 1e-12
    rpe_count = int(np.count_nonzero(eligible))
    rpe_rmse = None
    rpe_mean = None
    if rpe_count:
        future = np.minimum(gt_times[eligible] + 1.0, gt_times[-1])
        gt_rel = interp_linear(future, gt_times, gt_pos) - gt_pos[eligible]
        # 从原始里程计时间轴插值，避免两次重采样引入额外误差。
        od_future = interp_linear(future, odom_times, odom_pos) @ rotation.T + translation
        od_rel = od_future - odom_aligned[eligible]
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
        "alignment": alignment,
        "alignment_rotation": rotation.tolist(),
        "alignment_translation": translation.tolist(),
        "raw_ate_rmse": float(np.sqrt(np.mean(np.sum(raw_err**2, axis=1)))),
        "duration_s": float(duration),
        "num_frames": int(len(gt_times)),
        "ate_rmse": ate_rmse,
        "ate_mean": ate_mean,
        "ate_max": ate_max,
        "ate_final": ate_final,
        "rpe_num_pairs_1s": rpe_count,
        "evaluated_duration_s": float(gt_times[-1] - gt_times[0]),
        "rpe_rmse_1s": rpe_rmse,
        "rpe_mean_1s": rpe_mean,
        "snapshots": snapshots,
        "err_mean_xyz": err.mean(axis=0).tolist(),
        "err_std_xyz": err.std(axis=0).tolist(),
        "err_rmse_xyz": np.sqrt(np.mean(err**2, axis=0)).tolist(),
    }
