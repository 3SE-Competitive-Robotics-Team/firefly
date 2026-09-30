"""碰撞盒与 FFMap 共用体素格；不膨胀、不再次量化几何。"""

from pathlib import Path

import numpy as np


def collision_grid(report: dict, padding: float = 1.0, ceiling: float = 6.0):
    """返回 (origin, resolution, occupancy)；盒边必须落在同一体素格上。"""
    resolution = float(report["res"])
    boxes = np.asarray(report["boxes"], dtype=float)
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("collision resolution must be finite and positive")
    if boxes.ndim != 2 or boxes.shape[1] != 6 or not len(boxes):
        raise ValueError("collision boxes must be nonempty N×6")
    if not np.isfinite(boxes).all() or np.any(boxes[:, 3:] <= 0):
        raise ValueError("invalid collision boxes")
    if not np.isfinite([padding, ceiling]).all() or padding < 0:
        raise ValueError("invalid map bounds")
    lower, upper = boxes[:, :3] - boxes[:, 3:], boxes[:, :3] + boxes[:, 3:]
    origin = lower.min(axis=0) - np.ceil(padding / resolution) * resolution
    high = upper.max(axis=0) + np.ceil(padding / resolution) * resolution
    high[2] = max(high[2], ceiling)
    dims = np.ceil((high - origin) / resolution - 1e-9).astype(int)
    if np.prod(dims.astype(float)) > 100_000_000:
        raise ValueError("map exceeds 100 million voxels")
    first, last = (lower - origin) / resolution, (upper - origin) / resolution
    if max(np.abs(first - np.rint(first)).max(), np.abs(last - np.rint(last)).max()) > 1e-3:
        raise ValueError("collision boxes do not share a voxel lattice")
    occupied = np.zeros(tuple(dims), dtype=bool)
    for lo, hi in zip(np.rint(first).astype(int), np.rint(last).astype(int), strict=True):
        region = tuple(slice(a, b) for a, b in zip(lo, hi, strict=True))
        if occupied[region].any():
            raise ValueError("collision boxes overlap")
        occupied[region] = True
    return origin, resolution, occupied


def export_map(report: dict, path: Path, padding: float = 1.0, ceiling: float = 6.0) -> dict:
    """输出 FFMap v1，坐标为米、Z 向上、格心占据；返回机器可核验的几何摘要。"""
    origin, resolution, occupied = collision_grid(report, padding, ceiling)
    indices = np.argwhere(occupied)
    centers = origin + (indices + 0.5) * resolution
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        stream.write("FORMAT firefly-map 1\n")
        stream.write(f"RESOLUTION {resolution:.9g}\n")
        stream.write("ORIGIN " + " ".join(f"{x:.9g}" for x in origin) + "\n")
        stream.write("DIMS " + " ".join(str(x) for x in occupied.shape) + "\nOCCUPANCY\n")
        np.savetxt(stream, centers, fmt="%.9g")
    return {
        "resolution_m": resolution, "origin_m": origin.tolist(), "dims": list(occupied.shape),
        "occupied_voxels": len(indices), "occupied_volume_m3": len(indices) * resolution**3,
    }
