"""空地图在线 SLAM 的 22m 去程、返航与降落；使用正式任务验收器。

uv run pytest tests/system/test_roundtrip.py
需要图形会话、RMUC 资产和包含 slam 的全链路 release 程序。
"""
from firefly_evaluation.cli import main
from firefly_workspace import workspace_root


def test_roundtrip_22m(flight_directory):
    code = main([
        "--case", "nominal",
        "--config", str(workspace_root() / "configs/acceptance_22m.toml"),
        "--output-dir", str(flight_directory),
    ])
    assert code == 0, f"see {flight_directory / 'report.html'}"
