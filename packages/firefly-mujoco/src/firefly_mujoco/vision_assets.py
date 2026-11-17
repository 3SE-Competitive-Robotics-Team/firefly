"""离线库图的资产指纹与采集缓存完整性检查。"""
import json
from pathlib import Path
import subprocess
import tomllib

from .scene import load_scene_name
from firefly_workspace import digest, load_toml, parse_scene, workspace_root


def capture_assets(root=None, *, run=subprocess.run):
    """场地文件、渲染传感器契约与采集选择共同组成缓存指纹。"""
    root = workspace_root(root)
    scene = parse_scene(load_scene_name(root / "configs/scene.toml"))
    paths = [scene.dir / "field.glb", scene.dir / scene.collision]
    if (root / "configs/scene.toml").exists():
        paths.append(Path("configs/scene.toml"))
    files = {str(path): digest(root / path) for path in paths}
    contract = root / scene.dir / "derived/render_contract.toml"
    contract.parent.mkdir(parents=True, exist_ok=True)
    run([str(root / "target/release/render"), "--export-asset-contract", str(contract)], check=True, cwd=root)
    with contract.open("rb") as stream:
        sensor = tomllib.load(stream)
    selection = load_toml(root / "configs/vision_map.toml")
    return {"schema": 2, "files": files, "sensor": sensor, "selection": selection}


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
