# Mimir 录制规则

所有生产进程、脚本、CLI、系统测试和数据集测评均通过 Mimir 归档运行日志。
每次启动只产出一个录制文件：

```text
logs/<yyyymmdd_hhmmss>/log.rrd
```

`logs/` 位于 Firefly 安装根目录，开发机默认是仓库根目录；根目录解析沿用
[运行期路径约定](development.md#机载部署)。时间目录由 Mimir 读取系统当前本地时区生成，
精度为秒，例如 `logs/20261115_093005/log.rrd`。Rust 使用
`jiff::Zoned::now().strftime("%Y%m%d_%H%M%S")`；时区遵循系统设置和 `TZ`。
Jiff 的本地时区行为见[官方文档](https://docs.rs/jiff/0.2.41/jiff/struct.Zoned.html#method.now)。

**录制目录和文件名由 Mimir 独占决定。** 命令行只接受 `--config` 和帮助参数，
配置及库入口均使用这套固定规则。调用方从 stdout 的 `recording: <绝对路径>`
读取本次产物路径，用例名称与阶段属于 RRD 中的运行事件。
同一秒的目录已存在时，Mimir 等待实际本地时间进入下一个可用秒后创建新目录，保留已有录制。

**运行归档只保存 `log.rrd`。** 原始时序观测、阶段事件、进程日志与录制完整性状态
进入 RRD。验收评分从本次 RRD 读取原始证据，在内存中计算，并通过终端和退出码返回结果。
调用方通过统一录制入口获取路径，归档产物固定为上述单个 RRD 文件。
数据集的轨迹、地图等算法导出物遵循各自格式和配置契约。

时间目录和面向人的墙钟显示使用本地时区。RRD 的 `wall_time` 保存 Unix 时间，
`sim_time` 保存原始测量时间，保留原有跨进程对时和测量语义。

## 使用

```bash
cargo build --release -p mimir
./target/release/mimir
```

先启动 Mimir，再启动数据生产者。Mimir 打印本次绝对路径；生产者全部退出后，
向 Mimir 发送 Ctrl-C（SIGINT）或 SIGTERM，等待排空、写完索引并同步文件。
将下面的示例时间目录替换为本次返回的目录：

```bash
uv run rerun rrd verify logs/20261115_093005/log.rrd
uv run rerun logs/20261115_093005/log.rrd configs/firefly.rbl
```

配置、topic、生产开关、预算、观测登记及完整性验证见
[可观测性](observability.md#recorder-architecture-and-operation)。
