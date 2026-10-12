"""安装后的包不依赖源码目录导入；工作区用于资产定位。"""
import subprocess
import sys


def test_installed_modules_and_cli_help_work_outside_workspace(tmp_path):
    modules = ["cli", "mission", "mission_io", "mission_sim", "disturbance_sim", "disturbance",
               "process_lifecycle", "metrics", "mission_metrics", "hover_metrics",
               "disturbance_metrics", "vision_assets"]
    imports = "; ".join(f"import firefly_evaluation.{name}" for name in modules)
    subprocess.run([sys.executable, "-I", "-c", imports], cwd=tmp_path, check=True,
                   capture_output=True, text=True)
    result = subprocess.run([sys.executable, "-I", "-m", "firefly_evaluation.cli", "--help"],
                            cwd=tmp_path, check=True, capture_output=True, text=True)
    assert "--workspace" in result.stdout and "--case" in result.stdout
