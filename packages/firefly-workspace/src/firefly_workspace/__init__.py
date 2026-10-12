"""工具共用的工作区定位与文件内容指纹；仅使用标准库。"""
import hashlib
from pathlib import Path
import tomllib


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
