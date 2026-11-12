"""USB HIL 起飞、20 秒悬停与降落验收；板端先运行 cargo xtask deploy-hil。"""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import logging
from pathlib import Path
import subprocess

from firefly_evaluation.hover_metrics import HoverLimits, score_hover
from firefly_evaluation.mission import Mission, Options


def digest(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    root = Path(__file__).resolve().parents[1]
    subprocess.run([str(root / "target/release/hil"), "--check"], cwd=root, check=True)
    directory = root / "logs" / f"test_hil_hover_{datetime.now(timezone.utc):%Y%m%dT%H%M%S%fZ}"
    directory.mkdir()
    options, limits = Options(), HoverLimits()
    options.validate()
    mission = Mission(directory, "hil_hover", options, workspace=root)

    def sustained_hover():
        end = mission.t + limits.seconds
        mission.wait(lambda: mission.t >= end, sim=limits.seconds + 1.)
        return {"seconds": limits.seconds}

    files = [root / "target/release" / name for name in ("render", "hil", "ffctl", "mimir")]
    files += [root / "target/container/aarch64/release" / name for name in ("camera", "motor", "fc", "firefly-rootslam", "hil-gadget")]
    files += [root / "configs" / name for name in ("hil.toml", "rootslam-hil.toml", "processes-hil.toml", "fc-hil.toml", "scene.toml")]
    files.append(Path(__file__).resolve())
    provenance = {
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "working_tree_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "diff", "HEAD"], cwd=root)).hexdigest(),
        "files_sha256": {str(p.relative_to(root)): digest(p) for p in files},
        "options": asdict(options), "hover_limits": asdict(limits),
    }
    result = mission.run_phases([
        ("initialization", mission.initialized), ("takeoff", mission.takeoff),
        ("hover", mission.hover), ("sustained_hover", sustained_hover),
        ("landing", mission.landing),
    ], ("render", "hil"))
    result["transport"] = "USB HIL; RootSLAM and FC on firefly-0001"
    result["provenance"] = provenance
    score_hover(result, options, limits)
    report = directory / "report.json"
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(f"{result['status']}: {report}")
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
