#!/usr/bin/env python3
"""运行真实 RMUC 任务与失联故障验收；RRD 原始证据 + 内容指纹 + JSON/HTML 报告。"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import fcntl
import hashlib
from html import escape
from importlib.metadata import version
import json
import logging
from pathlib import Path
import platform
import subprocess
import sys
import tomllib
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/system"))
sys.path.insert(0, str(ROOT / "tests/evaluation"))
from mission import Mission, Options
from mission_metrics import score
from mission_sim import CONTACT_POLICY

log = logging.getLogger("acceptance")


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_report(directory, report):
    categories = {
        "初始化": ["initialization"], "起飞": ["takeoff"], "悬停": ["hover"],
        "路径跟踪": ["tracking", "tracking_error"], "定位误差": ["vio_accuracy", "map_accuracy"],
        "碰撞": ["collision"], "失联安全响应": ["reference_loss", "failure_hold", "estimator_inhibit", "failure_terminal"],
        "进程退出": ["shutdown"],
    }
    summary = {}
    for label, names in categories.items():
        evidence = [{"case": case["case"], "stage": name, "status": case["stages"][name]["status"]}
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
        rows.append(f"<h2>{escape(case['case'])}: {escape(case['status'])}</h2><p><a href='{escape(Path(case['recording']).name)}'>原始 RRD</a></p><pre>{escape(case.get('error', ''))} {escape(case.get('evidence_error', ''))}</pre><table><tr><th>验收项</th><th>状态</th><th>证据 / 原因</th></tr>")
        for name, stage in case["stages"].items():
            details = json.dumps({k: v for k, v in stage.items() if k != "status"}, ensure_ascii=False, indent=2)
            rows.append(f"<tr><td>{escape(name)}</td><td class='{escape(stage['status'])}'>{escape(stage['status'])}</td><td><pre>{escape(details)}</pre></td></tr>")
        rows.append("</table>")
    document = ("<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>RMUC 自动任务验收</title>"
                "<style>body{font:16px system-ui;max-width:1200px;margin:40px auto;padding:0 24px;color:#202a34}table{border-collapse:collapse;width:100%}td,th{border:1px solid #d2d8de;padding:10px;text-align:left;vertical-align:top}pre{white-space:pre-wrap;overflow-wrap:anywhere;margin:0;font-size:13px}.passed{color:#176a37}.failed{color:#b02020}.blocked{color:#8a6000}h2{margin-top:36px}</style>"
                f"<h1>RMUC 自动任务验收：{escape(report['status'])}</h1><p>运行 ID：{escape(report['run_id'])}</p>"
                "<p>真实 sim / render / VIO / FC；路径任务另启 GICP / planner。真值仅供评分。局部 VIO 误差允许固定尺度航向和平移对齐，地图定位与路径跟踪不对齐。正常起降接触与碰撞分开统计。</p>"
                "<p>本报告不包含 ALIKED / LightGlue 在线重定位与回环验收。操作系统与 GPU 调度不是确定性的，单次通过不代表可靠性概率。</p>"
                "<p><a href='report.json'>完整机器报告（配置、源码、二进制、资产及 RRD 的 SHA-256）</a></p>"
                + "<table><tr><th>汇总项</th><th>结果</th></tr>" + "".join(
                    f"<tr><td>{escape(label)}</td><td class='{value['status']}'>{value['status']}</td></tr>" for label, value in summary.items()) + "</table>"
                + "".join(rows) + f"<h2>运行前固定的阈值</h2><pre>{escape(json.dumps(report.get('options', {}), ensure_ascii=False, indent=2))}</pre>"
                + f"<h2>执行错误</h2><pre>{escape(report.get('error', '无'))}</pre></html>")
    (directory / "report.html").write_text(document)


def provenance():
    files = subprocess.check_output(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=ROOT).decode().split("\0")
    source = {name: digest(ROOT / name) for name in files if name and (ROOT / name).is_file()}
    binaries = {name: digest(ROOT / "target/release" / name) for name in ["vio", "render", "fc", "ffctl", "gicp", "planner"]}
    manifest_path = ROOT / "models/rmuc2026/asset_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("status") != "passed":
        raise ValueError("geometry has not passed acceptance")
    for path, expected in manifest["artifacts"].items():
        if digest(Path(path)) != expected:
            raise ValueError(f"asset content changed: {path}")
    return {"git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            "git_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
            "source_sha256": source, "binary_sha256": binaries,
            "configs": {str(p.relative_to(ROOT)): p.read_text() for p in (ROOT / "configs").glob("*.toml")},
            "assets": manifest, "asset_manifest_sha256": digest(manifest_path),
            "platform": platform.platform(), "python": sys.version,
            "dependencies": {name: version(name) for name in ["numpy", "mujoco", "iceoryx2", "rerun-sdk"]}}


def active_processes():
    """只识别本仓库的闭环进程；有其他任务占用时拒绝运行，不终止它们。"""
    found = []
    binaries = {str(ROOT / "target/release" / name) for name in ["vio", "render", "fc", "gicp", "planner", "aliked", "lightglue"]}
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().decode().split("\0")
            cwd = (path.parent / "cwd").resolve()
            if args[0] in binaries or (cwd == ROOT and any(arg in {
                "firefly_sim.main", "firefly_viz.main", str(ROOT / "tests/system/mission_sim.py")
            } for arg in args)):
                found.append(int(path.parent.name))
        except (OSError, UnicodeError):
            continue
    return found


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/acceptance.toml")
    parser.add_argument("--case", choices=["all", "nominal", "reference_loss", "estimator_loss"], default="all")
    parser.add_argument("--output-dir", type=Path, help="logs/ 下不存在的报告目录；默认使用唯一运行 ID")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    directory = args.output_dir.resolve() if args.output_dir else ROOT / "logs/acceptance" / run_id
    if not directory.is_relative_to(ROOT / "logs"):
        parser.error("acceptance evidence must be stored under repository logs/")
    directory.mkdir(parents=True)
    report = {"schema_version": 1, "run_id": run_id, "status": "running", "cases": [],
              "scope": "RMUC fixed-pad mission; VIO + GICP + planner; no learned relocalization/loop closure"}
    lock = (ROOT / "logs/acceptance.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        occupied = active_processes()
        if occupied:
            raise RuntimeError(f"repository flight processes already running: {occupied}")
        with args.config.open("rb") as stream:
            options = Options(**tomllib.load(stream))
        options.validate()
        report["options"] = asdict(options)
        report["contact_policy"] = CONTACT_POLICY
        report["provenance"] = provenance()
        cases = ["nominal", "reference_loss", "estimator_loss"] if args.case == "all" else [args.case]
        save_report(directory, report)
        for name in cases:
            log.info("开始任务验收 %s，证据目录 %s", name, directory)
            mission = Mission(directory, name, options)
            case = mission.run()
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
        lock.close()
        log.info("可追溯报告：%s", directory / "report.html")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
