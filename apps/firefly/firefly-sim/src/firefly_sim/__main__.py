"""通过 `python -m firefly_sim` 启动物理进程。"""

from .main import cli

if __name__ == "__main__":
    raise SystemExit(cli())
