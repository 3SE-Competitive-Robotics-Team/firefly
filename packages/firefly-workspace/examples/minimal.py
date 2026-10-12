"""最小 TOML 应用：命令行 → 公共启动入口 → 已校验配置 → 运行。"""

import argparse
from dataclasses import dataclass
import logging
import math

from firefly_workspace import startup


@dataclass(frozen=True)
class Options:
    rate_hz: float = 10.0


def decode(values: dict) -> Options:
    options = Options(**values)
    if type(options.rate_hz) not in (int, float) or not math.isfinite(options.rate_hz) or options.rate_hz <= 0:
        raise ValueError("rate_hz must be finite and positive")
    return options


def run(options: Options) -> None:
    # IPC、线程、模型等运行资源从这里开始创建。
    logging.getLogger(__name__).info("应用就绪：%s Hz", options.rate_hz)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/minimal.toml")
    args = parser.parse_args()
    options = startup(args.config, decode)
    run(options)


if __name__ == "__main__":
    main()
