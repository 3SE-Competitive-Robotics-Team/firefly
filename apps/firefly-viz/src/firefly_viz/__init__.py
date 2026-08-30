"""firefly-viz：rerun 统一写入进程（双语言闭环的可视化出口）。

Rust 计算线程（vio/planner）经 iceoryx2 `Firefly/Viz` 话题零拷贝发布
`VizMessage`，本进程订阅后统一写 rerun viewer（默认连共享 viewer
`127.0.0.1:9876`）或离线 rrd（`--save`）。计算线程零 IO。

运行：`uv run firefly-viz [--save out.rrd]`（先起 `rerun` viewer 或加
`--serve` 由本进程起内置 viewer）。
"""
