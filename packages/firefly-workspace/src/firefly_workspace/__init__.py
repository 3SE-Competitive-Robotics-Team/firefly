"""工具共用的工作区定位、可选配置、场景标识与文件内容指纹；仅使用标准库。"""
from collections.abc import Callable
from typing import TypeVar

import hashlib
import logging
from pathlib import Path
import tomllib

from .scene import SceneId, parse_scene


def workspace_root(start: Path | None = None) -> Path:
    """从指定文件、目录或当前目录向上查找 firefly workspace，不依赖包的安装位置。"""
    directory = (start if start is not None else Path.cwd()).resolve()
    if directory.is_file():
        directory = directory.parent
    for candidate in (directory, *directory.parents):
        manifest = candidate / "pyproject.toml"
        if manifest.is_file() and (candidate / "Cargo.toml").is_file():
            with manifest.open("rb") as stream:
                config = tomllib.load(stream)
            if config.get("project", {}).get("name") == "firefly" and "workspace" in config.get("tool", {}).get("uv", {}):
                return candidate
    raise ValueError(f"firefly workspace not found from {directory}; run from the repository or pass --workspace")


def digest(path: str | Path) -> str:
    """流式计算文件 SHA-256，返回小写十六进制摘要。"""
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


T = TypeVar("T")


def init_logging() -> None:
    """CLI 基础日志；已配置的宿主日志保持其处理器与过滤级别。"""
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")


def startup(path: str | Path, decode: Callable[[dict], T], *,
            init_logging: Callable[[], None] = init_logging) -> T:
    """命令行解析后初始化日志、读取并转换/校验主配置，然后报告来源。

    decode 只构造和校验配置；返回成功后调用方才创建运行资源。
    """
    init_logging()
    values, exists = _read_toml(path)
    config = decode(values)
    _report_config(path, exists)
    return config


def load_toml(path: str | Path) -> dict:
    """已初始化日志的进程读取辅助配置；仅缺文件返回空表，其他错误传播。"""
    values, exists = _read_toml(path)
    _report_config(path, exists)
    return values


def _read_toml(path: str | Path) -> tuple[dict, bool]:
    try:
        with Path(path).open("rb") as stream:
            return tomllib.load(stream), True
    except FileNotFoundError:
        return {}, False


def _report_config(path: str | Path, exists: bool) -> None:
    message = "已加载配置 %s" if exists else "配置 %s 不存在，使用默认值"
    logging.getLogger(__name__).info(message, path)
