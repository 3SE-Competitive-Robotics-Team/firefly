#!/usr/bin/env python3
"""运行真实 RMUC 任务与失联故障验收；RRD 原始证据 + 内容指纹 + JSON/HTML 报告。"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from html import escape
from importlib.metadata import version
import json
import logging
import os
from pathlib import Path
import platform
import subprocess
import sys
import tomllib
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests/system"))
sys.path.insert(0, str(ROOT / "tests/evaluation"))
from mission import Mission, Options
from process_lifecycle import require_idle
from mission_metrics import score
from mission_sim import CONTACT_POLICY
from collect_vision_map import verify_map_compatibility

log = logging.getLogger("acceptance")


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_report(directory, report):
    categories = {
        "初始化": ["initialization"], "起飞": ["takeoff"], "悬停": ["hover", "hover_accuracy", "terminal_hold", "terminal_hold_accuracy"],
        "降落": ["landing", "landing_accuracy"],
        "路径跟踪": ["tracking", "tracking_error", "waypoint_accuracy"], "定位误差": ["vio_accuracy", "map_accuracy"], "库图观测": ["visual_localization"],
        "在线回环": ["online_loop"], "碰撞": ["collision"], "失联安全响应": ["reference_loss", "failure_hold", "failure_hold_accuracy", "estimator_fallback", "degraded_support", "failure_terminal"],
        "进程退出": ["shutdown"],
    }
    summary = {}
    for label, names in categories.items():
        evidence = [{"case": case["case"], "attempt": case.get("attempt", 1), "stage": name, "status": case["stages"][name]["status"]}
                    for case in report["cases"] for name in names if name in case["stages"]]
        statuses = [row["status"] for row in evidence]
        status = ("failed" if "failed" in statuses else "passed"
                  if statuses and all(s == "passed" for s in statuses) else "incomplete")
        summary[label] = {"status": status, "evidence": evidence}
    report["summary"] = summary
    payload = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    temporary = directory / "report.json.partial"
    temporary.write_text(payload + "\n")
    temporary.replace(directory / "report.json")
    rows = []
    for case in report["cases"]:
        rows.append(f"<h2>{escape(case['case'])} / attempt {case.get('attempt', 1)}: {escape(case['status'])}</h2><p><a href='{escape(str(Path(case['recording']).relative_to(directory)))}'>原始 RRD</a></p><pre>{escape(case.get('error', ''))} {escape(case.get('evidence_error', ''))}</pre><table><tr><th>验收项</th><th>状态</th><th>证据 / 原因</th></tr>")
        for name, stage in case["stages"].items():
            details = json.dumps({k: v for k, v in stage.items() if k != "status"}, ensure_ascii=False, indent=2)
            rows.append(f"<tr><td>{escape(name)}</td><td class='{escape(stage['status'])}'>{escape(stage['status'])}</td><td><pre>{escape(details)}</pre></td></tr>")
        rows.append("</table>")
        if "mission_summary" in case:
            rows.append("<h3>往返与回环证据</h3><pre>" + escape(json.dumps(case["mission_summary"], ensure_ascii=False, indent=2)) + "</pre>")
    document = ("<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>RMUC 自动任务验收</title>"
                "<style>body{font:16px system-ui;max-width:1200px;margin:40px auto;padding:0 24px;color:#202a34}table{border-collapse:collapse;width:100%}td,th{border:1px solid #d2d8de;padding:10px;text-align:left;vertical-align:top}pre{white-space:pre-wrap;overflow-wrap:anywhere;margin:0;font-size:13px}.passed{color:#176a37}.failed{color:#b02020}.blocked{color:#8a6000}h2{margin-top:36px}</style>"
                f"<h1>RMUC 自动任务验收：{escape(report['status'])}</h1><p>运行 ID：{escape(report['run_id'])}</p>"
                f"<p>运行范围：{escape(report.get('scope', ''))}</p>"
                "<p>真实 sim / render / VIO / FC；路径任务另启 localization / planner。定位模式见运行范围。真值仅供评分。局部 VIO 误差允许固定尺度航向和平移对齐，地图定位与路径跟踪不对齐。正常起降接触与碰撞分开统计。</p>"
                "<p>默认空地图模式要求已接受的在线回环；外部地图模式要求已接受的库图观测。纯 VIO 模式关闭回环。会话地图评分只用首次真值定义固定坐标变换，不拟合估计轨迹，也不反馈在线系统。回环与库图观测分别验收。操作系统与 GPU 调度不是确定性的，单次通过不代表可靠性概率。</p>"
                "<p><a href='report.json'>完整机器报告（配置、源码、二进制、资产及 RRD 的 SHA-256）</a></p>"
                + "<table><tr><th>汇总项</th><th>结果</th></tr>" + "".join(
                    f"<tr><td>{escape(label)}</td><td class='{value['status']}'>{value['status']}</td></tr>" for label, value in summary.items()) + "</table>"
                + "".join(rows) + f"<h2>运行前固定的阈值</h2><pre>{escape(json.dumps(report.get('options', {}), ensure_ascii=False, indent=2))}</pre>"
                + f"<h2>执行错误</h2><pre>{escape(report.get('error', '无'))}</pre></html>")
    (directory / "report.html").write_text(document)


def provenance(*, allow_map_rig_translation=False, vio_only=False, prior_maps=False):
    files = subprocess.check_output(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=ROOT).decode().split("\0")
    source = {name: digest(ROOT / name) for name in files if name and (ROOT / name).is_file()}
    names = ["vio", "render", "fc", "ffctl", "localization", "planner"]
    if not vio_only:
        names += ["slam"]
        if prior_maps:
            names += ["aliked", "lightglue"]
    binaries = {name: digest(ROOT / "target/release" / name) for name in names}
    models = {} if vio_only or not prior_maps else {name: digest(ROOT / "models" / name) for name in ("aliked-n16-k512.onnx", "lightglue-aliked-k512.onnx")}
    manifest_path = ROOT / "models/rmuc2026/asset_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("status") != "passed":
        raise ValueError("geometry has not passed acceptance")
    for path, expected in manifest["artifacts"].items():
        if not prior_maps and Path(path).suffix == ".ffmap":
            continue
        if digest(Path(path)) != expected:
            raise ValueError(f"asset content changed: {path}")
    visual = None
    compatibility = {"status": "not_used_empty_map"}
    if prior_maps:
        visual_manifest = ROOT / "apps/planner/maps/rmuc2026.manifest.json"
        visual = json.loads(visual_manifest.read_text())
        if digest(ROOT / "apps/planner/maps/rmuc2026.ffvmap") != visual["map_sha256"]:
            raise ValueError("visual map content changed")
        for name, expected in visual["models"]["artifacts"].items():
            if digest(ROOT / "models" / name) != expected:
                raise ValueError(f"visual model content changed: {name}")
        compatibility = verify_map_compatibility(visual["assets"], allow_rig_translation=allow_map_rig_translation)
    return {"visual_map": visual, "visual_map_compatibility": compatibility, "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            "git_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
            "source_sha256": source, "binary_sha256": binaries, "model_sha256": models,
            "configs": {str(p.relative_to(ROOT)): p.read_text() for p in (ROOT / "configs").glob("*.toml")},
            "assets": manifest, "asset_manifest_sha256": digest(manifest_path),
            "platform": platform.platform(), "python": sys.version,
            "display_environment": {key: os.environ.get(key) for key in ["DISPLAY", "WAYLAND_DISPLAY", "WAYLAND_SOCKET", "WGPU_BACKEND"]},
            "dependencies": {name: version(name) for name in ["numpy", "mujoco", "iceoryx2", "rerun-sdk"]}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/acceptance.toml")
    parser.add_argument("--case", choices=["all", "nominal", "reference_loss", "estimator_loss"], default="all")
    parser.add_argument("--vio-only", action="store_true", help="纯 VIO + 会话坐标系；不启动 slam 或 ALIKED/LightGlue")
    parser.add_argument("--prior-maps", action="store_true", help="显式加载 RMUC 离线视觉地图与规划地图")
    parser.add_argument("--localization-config", type=Path, help="外部地图模式所需的显式启动先验配置")
    parser.add_argument("--repeat", type=int, default=1, help="每种任务独立启动次数；全部使用固定配置")
    parser.add_argument("--output-dir", type=Path, help="logs/ 下不存在的报告目录；默认使用唯一运行 ID")
    parser.add_argument("--allow-map-rig-translation", action="store_true",
                        help="显式允许冻结地图与当前 rig 的共同安装平移差异；记录差异，仍校验其他标定与资产")
    args = parser.parse_args(argv)
    if args.vio_only and args.prior_maps:
        parser.error("--vio-only and --prior-maps are mutually exclusive")
    if args.allow_map_rig_translation and not args.prior_maps:
        parser.error("--allow-map-rig-translation requires --prior-maps")
    if args.prior_maps != (args.localization_config is not None):
        parser.error("--prior-maps requires --localization-config with explicit [origin]")
    if args.localization_config:
        with args.localization_config.open("rb") as stream:
            if "origin" not in tomllib.load(stream):
                parser.error("external maps require explicit [origin]")
    if not 1 <= args.repeat <= 100:
        parser.error("--repeat must be in 1..100")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    directory = args.output_dir.resolve() if args.output_dir else ROOT / "logs/acceptance" / run_id
    if not directory.is_relative_to(ROOT / "logs"):
        parser.error("acceptance evidence must be stored under repository logs/")
    directory.mkdir(parents=True)
    report = {"schema_version": 2, "mission_decisions": "fresh estimated state; independent ground-truth scoring", "run_id": run_id, "status": "running", "cases": [],
              "repeat": args.repeat, "scope": "RMUC mission; empty session map + VIO + online loops + depth mapping/planning; simulator geometry is evaluation infrastructure"}
    if args.vio_only:
        report["scope"] = "RMUC mission; empty session map + VIO + depth mapping/planning; loop closure disabled"
    elif args.prior_maps:
        report["scope"] = "RMUC mission; VIO + explicit external map alignment + offline visual map localization + online loops + prior/depth mapping"
    try:
        require_idle(ROOT)
        with args.config.open("rb") as stream:
            options = Options(**tomllib.load(stream))
        options.validate()
        report["options"] = asdict(options)
        report["contact_policy"] = CONTACT_POLICY
        report["provenance"] = (provenance(vio_only=True) if args.vio_only else
                                provenance(prior_maps=True, allow_map_rig_translation=args.allow_map_rig_translation) if args.prior_maps else provenance())
        cases = ["nominal", "reference_loss", "estimator_loss"] if args.case == "all" else [args.case]
        save_report(directory, report)
        for attempt, name in [(attempt, name) for attempt in range(1, args.repeat + 1) for name in cases]:
            case_directory = directory if args.repeat == 1 else directory / f"attempt_{attempt:02d}"
            case_directory.mkdir(exist_ok=True)
            log.info("开始任务验收 %s，证据目录 %s", name, directory)
            mission = (Mission(case_directory, name, options, vio_only=True) if args.vio_only
                       else Mission(case_directory, name, options, prior_maps=True, localization_config=args.localization_config) if args.prior_maps
                       else Mission(case_directory, name, options))
            case = mission.run()
            case["attempt"] = attempt
            try:
                case["recording_sha256"] = digest(Path(case["recording"]))
                score(case, options)
            except Exception as error:
                case["status"] = "failed"
                case["evidence_error"] = str(error)
            report["cases"].append(case)
            save_report(directory, report)
            log.info("%s: %s %s", name, case["status"], case.get("error", ""))
            if case["remaining_pids"]:
                raise RuntimeError(f"processes did not exit: {case['remaining_pids']}")
            if case.get("interrupted"):
                raise KeyboardInterrupt("acceptance interrupted")
            del mission
        report["status"] = "passed" if all(case["status"] == "passed" for case in report["cases"]) else "failed"
    except BaseException as error:
        report.update(status="failed", error=str(error))
        log.error("验收中断：%s", error)
    finally:
        save_report(directory, report)
        log.info("可追溯报告：%s", directory / "report.html")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
