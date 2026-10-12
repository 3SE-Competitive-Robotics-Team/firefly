"""离线库图的资产指纹、采集缓存与运行时兼容检查。"""
import json
import subprocess
import tomllib

import numpy as np

from firefly_workspace import digest, load_toml, workspace_root


def capture_assets(root=None):
    """场地/传感器兼容条件与采集选择；二进制只作为生成来源保存。"""
    root = workspace_root(root)
    contract = root / "models/rmuc2026/derived/render_contract.toml"
    contract.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([str(root / "target/release/render"), "--export-asset-contract", str(contract)], check=True, cwd=root)
    with contract.open("rb") as stream:
        sensor = tomllib.load(stream)
    selection = load_toml(root / "configs/vision_map.toml")
    return {"schema": 2, "files": {name: digest(root / name) for name in [
        "models/rmuc2026/field.glb", "models/rmuc2026/rmuc2026_collision.json",
        *(["configs/scene.toml"] if (root / "configs/scene.toml").exists() else []),
    ]}, "sensor": sensor, "selection": selection}


def verify_map_compatibility(assets, *, root=None, allow_rig_translation=False, capture=capture_assets):
    current = capture(root)
    if assets.get("schema") != 2 or assets.get("files") != current["files"]:
        raise ValueError("visual map geometry/calibration/appearance contract differs from runtime")
    if assets.get("sensor") == current["sensor"]:
        return {"status": "exact_match"}
    previous = assets.get("sensor", {})
    sensor = current["sensor"]
    offsets = {"left_offset", "depth_offset"}
    same_camera = ({k: v for k, v in previous.items() if k not in offsets}
                   == {k: v for k, v in sensor.items() if k not in offsets})
    if allow_rig_translation and same_camera and offsets <= previous.keys() and offsets <= sensor.keys():
        shifts = [np.asarray(sensor[k]) - np.asarray(previous[k]) for k in sorted(offsets)]
        if all(s.shape == (3,) and np.isfinite(s).all() for s in shifts) and np.allclose(shifts[0], shifts[1], atol=1e-12, rtol=0):
            # 冻结路标已经在地图系；共同平移不改变左右/深度相对标定。
            # 库帧机体位置只用于候选检索，保留采集时的位置及其偏移记录。
            return {"status": "explicit_rig_translation_exception", "rig_translation_body_m": shifts[0].tolist(),
                    "landmarks": "unchanged_map_coordinates", "candidate_body_position_offset_m": float(np.linalg.norm(shifts[0]))}
    raise ValueError("visual map geometry/calibration/appearance contract differs from runtime")


def reusable_capture(directory, assets):
    """清单完整且输入、逐帧产物均匹配才可复用；空采集不是成功。"""
    try:
        manifest = json.loads((directory / "capture_manifest.json").read_text())
        rows = manifest["frames"]
        if (manifest["status"] != "complete" or manifest["assets"] != assets
                or len(rows) != manifest["requested"]):
            return False
        expected = {f"frame_{row['index']:05d}.bin": row["sha256"] for row in rows if row["accepted"]}
        actual = {path.name: digest(path) for path in directory.glob("*.bin")}
        return bool(expected) and expected == actual
    except (OSError, ValueError, KeyError, TypeError):
        return False
