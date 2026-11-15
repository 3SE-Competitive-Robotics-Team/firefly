"""USB HIL 起飞、20 秒悬停与降落验收；板端先运行 cargo xtask deploy-hil。"""
import logging
from pathlib import Path
import subprocess

from firefly_evaluation.hover_metrics import HoverLimits, score_hover
from firefly_evaluation.mission import Mission, Options


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    root = Path(__file__).resolve().parents[1]
    subprocess.run([str(root / "target/release/hil"), "--check"], cwd=root, check=True)
    options, limits = Options(), HoverLimits()
    options.validate()
    mission = Mission("hil_hover", options, workspace=root)

    def sustained_hover():
        end = mission.t + limits.seconds
        mission.wait(lambda: mission.t >= end, sim=limits.seconds + 1.)
        return {"seconds": limits.seconds}

    result = mission.run_phases([
        ("initialization", mission.initialized), ("takeoff", mission.takeoff),
        ("hover", mission.hover), ("sustained_hover", sustained_hover),
        ("landing", mission.landing),
    ], ("render", "hil"))
    result["transport"] = "USB HIL; RootSLAM and FC on firefly-0001"
    score_hover(result, options, limits)
    print(f"{result['status']}: {result.get('recording')}")
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
