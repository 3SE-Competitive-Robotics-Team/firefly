"""运行真实 RMUC 任务与失联故障验收；原始证据统一由 Mimir 写入 log.rrd。"""
import argparse
import json
import logging
from pathlib import Path
import sys

from .mission import Mission, Options
from .process_lifecycle import require_idle
from .mission_metrics import score
from firefly_mujoco import load_scene_name
from firefly_workspace import digest, parse_scene, startup, workspace_root

log = logging.getLogger("acceptance")


def validate_assets(root):
    """校验当前场景的验收清单及资产内容。"""
    scene = parse_scene(load_scene_name(root / "configs/scene.toml"))
    manifest = json.loads((root / scene.dir / "asset_manifest.json").read_text())
    if manifest.get("status") != "passed":
        raise ValueError("geometry has not passed acceptance")
    if manifest.get("scene") != scene.name:
        raise ValueError("geometry manifest belongs to another scene")
    for path, expected in manifest["artifacts"].items():
        if Path(path).suffix == ".ffmap":
            continue
        if digest(root / path) != expected:
            raise ValueError(f"asset content changed: {path}")


def decode_options(values: dict) -> Options:
    """转换并校验验收配置，不启动任务或创建证据目录。"""
    options = Options(**values)
    options.validate()
    return options


def main(argv=None, *, resolve_workspace=workspace_root, check_idle=require_idle,
         check_assets=validate_assets, mission_factory=Mission, score_case=score):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, help="firefly 工作区目录；默认从当前目录向上查找")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--case", choices=["all", "nominal", "reference_loss", "estimator_loss"], default="all")
    parser.add_argument("--repeat", type=int, default=1, help="每种任务独立启动次数；全部使用固定配置")
    args = parser.parse_args(argv)
    try:
        root = resolve_workspace(args.workspace)
    except ValueError as error:
        parser.error(str(error))
    if args.config is None:
        args.config = root / "configs/acceptance.toml"
    if not 1 <= args.repeat <= 100:
        parser.error("--repeat must be in 1..100")
    passed = True
    try:
        options = startup(args.config, decode_options)
        check_idle(root)
        check_assets(root)
        cases = ["nominal", "reference_loss", "estimator_loss"] if args.case == "all" else [args.case]
        for attempt, name in [(attempt, name) for attempt in range(1, args.repeat + 1) for name in cases]:
            log.info("开始任务验收 %s，轮次 %s", name, attempt)
            mission = mission_factory(name, options, workspace=root)
            case = mission.run()
            try:
                score_case(case, options)
            except Exception as error:
                case["status"] = "failed"
                case["evidence_error"] = str(error)
            passed &= case["status"] == "passed"
            log.info("%s: %s recording=%s error=%s evidence_error=%s", name, case["status"],
                     case.get("recording"), case.get("error", ""), case.get("evidence_error", ""))
            if case["remaining_pids"]:
                raise RuntimeError(f"processes did not exit: {case['remaining_pids']}")
            if case.get("interrupted"):
                raise KeyboardInterrupt("acceptance interrupted")
            del mission
    except BaseException as error:
        log.error("验收中断：%s", error)
        return 1
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
