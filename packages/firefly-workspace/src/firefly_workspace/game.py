"""game.toml 的队伍契约；世界几何归场景模块。"""
from pathlib import Path


def load_team(path: Path | None = None) -> str:
    from . import load_toml, workspace_root
    values = load_toml(path or workspace_root(Path(__file__)) / "configs/game.toml")
    if unknown := values.keys() - {"team"}:
        raise ValueError(f"未知 game 配置项: {', '.join(sorted(unknown))}")
    team = values.get("team", "red")
    if team not in ("red", "blue"):
        raise ValueError("game.team 必须为 red 或 blue")
    return team
