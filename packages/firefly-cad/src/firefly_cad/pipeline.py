"""可复现的 RMUC 资产构建：逐阶段内容校验缓存，验收通过后写入总清单。"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
from importlib.metadata import version
import json
import logging
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tomllib

import numpy as np

from .collide import collide, write_collision
from .map_export import export_map
from .tessellate import tessellate

log = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parents[4]


@dataclass(frozen=True)
class Options:
    deflection_mm: float = 2.0
    resolution_m: float = 0.15
    padding_m: float = 1.0
    ceiling_m: float = 6.0

    def validate(self):
        values = asdict(self)
        if not all(np.isfinite(x) for x in values.values()):
            raise ValueError("CAD options must be finite")
        if min(self.deflection_mm, self.resolution_m, self.ceiling_m) <= 0 or self.padding_m < 0:
            raise ValueError("invalid CAD options")


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_json(path: Path, value: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def blender_environment(executable: str, cache: Path, numpy_version: str):
    """发行版 Blender 可共享系统 Python；缺 NumPy 时只安装到构建目录。"""
    probe = 'import sys,json,importlib.util; print("FIREFLY_PYTHON="+json.dumps({"executable":sys.executable,"version":list(sys.version_info[:2]),"numpy":importlib.util.find_spec("numpy") is not None}))'
    output = subprocess.check_output([executable, "--background", "--python-exit-code", "1", "--python-expr", probe], text=True)
    info = json.loads(next(line.removeprefix("FIREFLY_PYTHON=") for line in output.splitlines() if line.startswith("FIREFLY_PYTHON=")))
    environment = os.environ.copy()
    if not info["numpy"]:
        target = cache / ("blender-python-" + ".".join(map(str, info["version"])))
        marker = target / ".numpy-version"
        if not marker.is_file() or marker.read_text() != numpy_version:
            subprocess.run(["uv", "pip", "install", "--python", info["executable"], "--target", str(target),
                            "--upgrade", "--no-deps", f"numpy=={numpy_version}"], check=True)
            marker.write_text(numpy_version)
        environment["PYTHONPATH"] = str(target.resolve())
    return environment


def stage(name: str, cache: Path, inputs: dict, outputs: list[Path], action):
    """输入与全部产物 hash 都匹配才复用；失败不写成功戳。"""
    stamp = cache / f"{name}.json"
    current = {str(p.resolve()): digest(p) for p in outputs if p.is_file()}
    if stamp.is_file():
        previous = json.loads(stamp.read_text())
        if previous.get("inputs") == inputs and len(current) == len(outputs) and previous.get("outputs") == current:
            log.info("复用已校验阶段 %s", name)
            return
    log.info("执行阶段 %s", name)
    action()
    save_json(stamp, {"inputs": inputs, "outputs": {str(p.resolve()): digest(p) for p in outputs}})


def validate_assets(raw: Path, visual: Path, collision: Path, map_path: Path) -> dict:
    """独立读回 GLB/FFMap，检查坐标边界、格心和碰撞盒覆盖；失败不发布总清单。"""
    import trimesh

    with np.load(raw) as data:
        vertices = data["vertices"]
        if not np.isfinite(vertices).all():
            raise ValueError("non-finite CAD vertices")
        expected_bounds = np.array([vertices.min(axis=0), vertices.max(axis=0)])
    scene = trimesh.load(visual, force="scene", process=False)
    visual_bounds = np.asarray(scene.bounds)
    if not np.allclose(visual_bounds, expected_bounds, atol=1e-4, rtol=0):
        raise ValueError(f"GLB coordinate mismatch: {visual_bounds} != {expected_bounds}")
    del scene
    report = json.loads(collision.read_text())
    boxes = np.asarray(report["boxes"])
    resolution = report["res"]
    header = map_path.read_text().splitlines()
    if (header[0] != "FORMAT firefly-map 1" or header[4] != "OCCUPANCY"
            or header[1].split()[0] != "RESOLUTION"
            or header[2].split()[0] != "ORIGIN" or header[3].split()[0] != "DIMS"):
        raise ValueError("invalid FFMap header")
    if not np.isclose(float(header[1].split()[1]), resolution, atol=1e-9, rtol=1e-8):
        raise ValueError("FFMap and collision resolutions differ")
    origin = np.array([float(v) for v in header[2].split()[1:]])
    dims = np.array([int(v) for v in header[3].split()[1:]])
    positions = np.loadtxt(map_path, skiprows=5).reshape(-1, 3)
    index_float = (positions - origin) / resolution - 0.5
    indices = np.rint(index_float).astype(int)
    if not np.allclose(indices, index_float, atol=1e-5, rtol=0):
        raise ValueError("FFMap positions are not voxel centers")
    if np.any(indices < 0) or np.any(indices >= dims) or len(np.unique(indices, axis=0)) != len(indices):
        raise ValueError("FFMap has duplicate or out-of-bounds cells")
    # 每个格心必须属于且仅属于一个物理盒；体积相等排除漏掉完整体素。
    covered = np.zeros(len(positions), dtype=np.uint16)
    for box in boxes:
        covered += np.all(np.abs(positions - box[:3]) < box[3:] + 1e-7, axis=1)
    if not np.all(covered == 1):
        raise ValueError("FFMap and physics collision disagree")
    collision_volume = float(np.sum(8 * np.prod(boxes[:, 3:], axis=1)))
    if not np.isclose(len(positions) * resolution**3, collision_volume, atol=1e-5, rtol=1e-6):
        raise ValueError("FFMap and collision volumes differ")
    bounds = np.array([(boxes[:, :3] - boxes[:, 3:]).min(axis=0), (boxes[:, :3] + boxes[:, 3:]).max(axis=0)])
    if np.max(np.abs(bounds - expected_bounds)) > resolution + 1e-4:
        raise ValueError("collision and visual bounds differ by more than one voxel")
    return {"visual_bounds_m": visual_bounds.tolist(), "collision_bounds_m": bounds.tolist(),
            "collision_boxes": len(boxes), "occupied_voxels": len(positions),
            "collision_volume_m3": collision_volume, "geometry_consistency": "passed",
            "flight_validation": "not_run", "vision_localization_validation": "not_run"}


def build(source: Path, output: Path, map_path: Path, options: Options, blender: str) -> dict:
    options.validate()
    source = source.resolve(strict=True)
    executable = shutil.which(blender)
    if executable is None:
        raise FileNotFoundError(f"Blender not found: {blender}")
    blender_version = subprocess.check_output([executable, "--version"], text=True).splitlines()[0]
    output.mkdir(parents=True, exist_ok=True)
    cache = output / "derived" / "stages"
    raw_dir = output / "derived" / "raw"
    raw = raw_dir / "field_raw.npz"
    visual = output / "field.glb"
    collision = output / "rmuc2026_collision.json"
    source_hash = digest(source)
    # 模块源码与依赖版本参与缓存，避免沿用另一套转换规则的资产。
    code = {p.name: digest(p) for p in Path(__file__).parent.glob("*.py")}
    versions = {p: version(p) for p in ["cadquery-ocp-novtk", "numpy", "trimesh", "scipy"]}
    common = {"dependencies": versions}
    save_json(output / "asset_manifest.json", {"status": "building", "source_sha256": source_hash})
    stage("tessellate", cache, {**common, "code": code["tessellate.py"], "source_sha256": source_hash, "deflection_mm": options.deflection_mm},
          [raw, raw_dir / "_manifest.json"], lambda: tessellate(source, raw_dir, options.deflection_mm))
    raw_hash = digest(raw)
    stage("collision", cache, {**common, "code": code["collide.py"], "raw": raw_hash, "resolution": options.resolution_m}, [collision],
          lambda: write_collision(collide([raw], options.resolution_m), collision))
    export_script = ROOT / "scripts" / "blender_field.py"
    environment = blender_environment(executable, output / "derived", versions["numpy"])
    stage("visual", cache, {"raw": raw_hash, "script": digest(export_script), "blender": blender_version}, [visual],
          lambda: subprocess.run([executable, "--background", "--python-use-system-env", "--python-exit-code", "1", "--python", str(export_script),
                                  "--", str(raw.resolve()), str(visual.resolve())], check=True, env=environment))
    stage("map", cache, {**common, "code": code["map_export.py"], "collision": digest(collision), "padding": options.padding_m, "ceiling": options.ceiling_m}, [map_path],
          lambda: export_map(json.loads(collision.read_text()), map_path, options.padding_m, options.ceiling_m))
    summary = validate_assets(raw, visual, collision, map_path)
    if digest(source) != source_hash:
        raise ValueError("source STP changed during build")
    summary.update({"status": "passed", "source": {"path": str(source), "sha256": source_hash, "bytes": source.stat().st_size},
                    "options": asdict(options), "dependencies": versions, "blender": blender_version,
                    "world_frame": {"units": "m", "up": "Z", "normalization": json.loads((raw_dir / "_manifest.json").read_text())["shift_mm"]},
                    "artifacts": {str(p.resolve()): digest(p) for p in [raw, visual, collision, map_path]}})
    save_json(output / "asset_manifest.json", summary)
    log.info("资产一致性验收通过：%d 碰撞盒，%d 占据格", summary["collision_boxes"], summary["occupied_voxels"])
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stp", type=Path)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "models/rmuc2026")
    parser.add_argument("--map-out", type=Path, default=ROOT / "apps/planner/maps/rmuc2026.ffmap")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/cad.toml")
    parser.add_argument("--blender", default="blender")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    with args.config.open("rb") as stream:
        options = Options(**tomllib.load(stream))
    build(args.stp, args.out_dir, args.map_out, options, args.blender)
    return 0


if __name__ == "__main__":
    sys.exit(main())
