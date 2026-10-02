#!/usr/bin/env python3
"""Build a visual map only from verified captures and verified model exports."""
import argparse
import json
from pathlib import Path
import subprocess

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
    binary = ROOT / "target/release/aliked"
    subprocess.run([str(binary), "--build-map", str(frames), "--out", str(output)], check=True, cwd=ROOT)
    manifest = {"map_sha256": digest(output), "models": models,
                "builder_sha256": digest(binary), "capture_manifest": str(frames / "capture_manifest.json"),
                "capture_manifest_sha256": digest(frames / "capture_manifest.json"), "assets": assets}
    output.with_suffix(".manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "apps/planner/maps/rmuc2026.ffvmap")
    args = parser.parse_args()
    build(args.frames, args.out)
