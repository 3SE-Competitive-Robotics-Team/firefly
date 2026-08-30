# firefly-viz

rerun 统一写入进程：订阅 iceoryx2 `Firefly/Viz` 话题，把可视化数据写入
共享 rerun viewer（默认 `127.0.0.1:9876`）或离线 rrd 文件。

Rust 计算线程（vio/planner）零 IO——经 `Firefly/Viz` 话题零拷贝发布
`VizMessage`，由本进程统一写 rerun（blueprint、时间轴、录制全部收归
Python 端）。

## 运行

```bash
# 先起共享 viewer（多进程共用）
rerun

# 连共享 viewer
uv run firefly-viz

# 离线录制
uv run firefly-viz --save logs/run.rrd

# 由本进程起内置 viewer
uv run firefly-viz --serve
```
