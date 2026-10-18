"""场景标识 `rmuc<年份>@<主>.<次>.<修订>` 的解析与资产布局。

配置、资产目录、碰撞文件与规划地图名由同一处推导，避免各应用各拼一套路径。
"""

from dataclasses import dataclass
from pathlib import Path
import re

_PATTERN = re.compile(r"^([a-z]+\d{4})@(\d+\.\d+\.\d+)$")


@dataclass(frozen=True)
class SceneId:
    """已解析的场景标识；`version` 不带 `v` 前缀。"""

    season: str
    version: str

    @property
    def name(self) -> str:
        return f"{self.season}@{self.version}"

    @property
    def dir(self) -> Path:
        """`models/` 下的版本目录，相对仓库根。"""
        return Path("models") / self.season / f"v{self.version}"

    @property
    def collision(self) -> str:
        return f"{self.season}_collision.json"

    @property
    def map(self) -> str:
        """`apps/planner/maps/` 下的规划地图名。"""
        return f"{self.season}-{self.version}.ffmap"


def parse_scene(name: str) -> SceneId:
    """解析场景标识；格式不符视为配置错误。"""
    match = _PATTERN.fullmatch(name.strip())
    if match is None:
        raise ValueError(f"场景标识必须是 <season>@<x.y.z>，收到 {name!r}")
    return SceneId(match.group(1), match.group(2))
