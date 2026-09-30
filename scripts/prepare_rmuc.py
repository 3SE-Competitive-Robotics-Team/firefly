#!/usr/bin/env python3
"""RMUC 全链路准备与分层验收；缺失阶段显式报告 blocked，不计作通过。

uv run --all-packages --extra test python scripts/prepare_rmuc.py <场地.stp>
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import hashlib
import logging
import os
from pathlib import Path
import signal
import subprocess
import sys
import tomllib
import uuid

from firefly_cad.pipeline import Options, build, save_json
from collect_vision_map import capture_assets, reusable_capture

ROOT = Path(__file__).resolve().parents[1]
log = logging.getLogger(__name__)


def capture_directory(parent, assets):
    """有效产物复用，失败产物保留，重试使用新的空目录。"""
    key = hashlib.sha256(json.dumps(assets, sort_keys=True).encode()).hexdigest()[:16]
    attempt = 0
    while True:
        directory = parent / (key if attempt == 0 else f"{key}-{attempt:03d}")
        if not directory.exists() or reusable_capture(directory, assets):
            return directory
        attempt += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stp", type=Path)
    parser.add_argument("--blender", default="blender")
    parser.add_argument("--assets-only", action="store_true", help="只构建与验收几何资产")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    output = ROOT / "models/rmuc2026"
    report = {"status": "running", "stages": {}}
    def run(name, command, env=None):
        log.info("验收阶段 %s", name)
        code = subprocess.run(command, cwd=ROOT, env=env).returncode
        report["stages"][name] = {"status": "passed" if code == 0 else "failed", "exit_code": code}
        return code == 0
    try:
        with (ROOT / "configs/cad.toml").open("rb") as stream:
            options = Options(**tomllib.load(stream))
        assets = build(args.stp, output, ROOT / "apps/planner/maps/rmuc2026.ffmap", options, args.blender)
        report["source_sha256"] = assets["source"]["sha256"]
        report["stages"]["geometry"] = {"status": "passed", "manifest": str(output / "asset_manifest.json")}
        env = os.environ.copy()
        env["FIREFLY_RUN_RMUC_ASSETS"] = "1"
        run("physics", [sys.executable, "-m", "pytest", "tests/system/test_rmuc_assets.py", "-q", "-p", "no:cacheprovider"], env)
        if args.assets_only:
            report["scope"] = "assets_only"
        elif run("release_build", ["cargo", "build", "--release", "-j", "2", "-p", "render", "-p", "vio", "-p", "fc", "-p", "ffctl", "-p", "gicp", "-p", "planner", "-p", "aliked", "-p", "lightglue"]):
            run("drone_visual", [sys.executable, str(ROOT / "scripts/fetch_drone_model.py")])
            run("planning", ["cargo", "test", "--release", "-p", "firefly-planner", "--test", "rmuc_asset", "--", "--ignored"])
            assets = capture_assets()
            frames = capture_directory(output / "derived/vision", assets)
            manifest = frames / "capture_manifest.json"
            if reusable_capture(frames, assets):
                report["stages"]["capture"] = {"status": "passed", "manifest": str(manifest)}
            else:
                record = ROOT / "logs" / f"rmuc_offline_capture_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}.rrd"
                record.parent.mkdir(exist_ok=True)
                viz = subprocess.Popen([sys.executable, "-m", "firefly_viz.main", "--save", str(record)], cwd=ROOT)
                try:
                    run("capture", [sys.executable, str(ROOT / "scripts/collect_vision_map.py"), "--frames-dir", str(frames)])
                    report["stages"]["capture"]["recording"] = str(record)
                finally:
                    if viz.poll() is None:
                        viz.send_signal(signal.SIGINT)
                        viz.wait(timeout=30)
            if manifest.is_file():
                capture = json.loads(manifest.read_text())
                report["stages"]["capture"].update({"manifest": str(manifest), "requested": capture["requested"], "accepted": sum(row["accepted"] for row in capture["frames"])})
            aliked, lightglue = [ROOT / "models" / x for x in ["aliked-n16-k512.onnx", "lightglue-aliked-k512.onnx"]]
            if aliked.is_file() and report["stages"]["capture"]["status"] == "passed":
                run("vision_map", [str(ROOT / "target/release/aliked"), "--build-map", str(frames), "--out", str(ROOT / "apps/planner/maps/rmuc2026.ffvmap")])
            else:
                report["stages"]["vision_map"] = {"status": "blocked", "reason": "ALIKED weights or synchronized captures unavailable"}
            # 建库与在线定位是不同验收项；权重存在也不能替代在线重定位测试。
            report["stages"]["visual_localization"] = {
                "status": "not_run", "weights_available": aliked.is_file() and lightglue.is_file(),
                "reason": "This entry does not implement online relocalization acceptance",
            }
            acceptance = ROOT / "logs/acceptance" / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8])
            run("mission_acceptance", [sys.executable, str(ROOT / "scripts/accept_rmuc.py"), "--output-dir", str(acceptance)])
            report["stages"]["mission_acceptance"]["report"] = str(acceptance / "report.json")
        report["status"] = "passed" if all(s["status"] == "passed" for s in report["stages"].values()) else "incomplete"
    except BaseException as error:
        report["status"] = "failed"
        report["error"] = str(error)
        raise
    finally:
        save_json(output / "preparation_report.json", report)
        log.info("验收报告：%s", output / "preparation_report.json")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
