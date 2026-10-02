#!/usr/bin/env python3
"""Build a visual map only from verified captures and verified model exports."""
import argparse
import json
from pathlib import Path
import subprocess
import shutil

from collect_vision_map import ROOT, capture_assets, digest, reusable_capture


def build(frames, output):
    assets = capture_assets()
    if not reusable_capture(frames, assets):
        raise ValueError("captures do not match current scene, camera, renderer or per-frame hashes")
    model_manifest = ROOT / "models/vision_models.json"
    models = json.loads(model_manifest.read_text())
    for name, expected in models["artifacts"].items():
        if digest(ROOT / "models" / name) != expected:
            raise ValueError(f"model content changed: {name}")
    manifest_path = output.with_suffix(".manifest.json")
    if output.is_file() and manifest_path.is_file():
        existing = json.loads(manifest_path.read_text())
        if (existing["map_sha256"] == digest(output) and existing["assets"] == assets
                and existing["models"] == models):
            return
        archive = ROOT / "models/rmuc2026/derived/vision_maps" / digest(output)
        archive.mkdir(parents=True, exist_ok=True)
        shutil.copy2(output, archive / output.name)
        shutil.copy2(manifest_path, archive / manifest_path.name)
    binary = ROOT / "target/release/aliked"
    builder_sha256 = digest(binary)
    subprocess.run([str(binary), "--build-map", str(frames), "--out", str(output)], check=True, cwd=ROOT)
    manifest = {"map_sha256": digest(output), "models": models,
                "builder_sha256": builder_sha256, "capture_manifest": str(frames / "capture_manifest.json"),
                "capture_manifest_sha256": digest(frames / "capture_manifest.json"), "assets": assets}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "apps/planner/maps/rmuc2026.ffvmap")
    args = parser.parse_args()
    build(args.frames, args.out)
