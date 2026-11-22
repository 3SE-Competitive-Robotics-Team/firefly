# 运行归档与 Mimir 录制

板端每次开机在 `/opt/firefly/logs/runs/` 下使用一个目录。同一次开机内，服务重启和后续任务通过 Linux boot ID 找回该目录。桌面独立运行 Mimir、SIL 或 USB HIL 时，每次启动在工作区的 `logs/runs/` 下创建一个目录。

目录从 `log_0001` 顺次编号到 `log_9999`，随后从头寻找空号。所有编号都已占用时，复用最早可清理的目录。清单中的 `sequence` 记录创建顺序，即使板端开机时间显示为 1970 年，也能按先后轮转。

```text
logs/runs/log_0001/
  manifest.json
  text/camera.000001.log
  system/board.000001.log
  rrd/000001.rrd
  rrd/000002.rrd.partial
```

| 日志 | 用途与写入方 |
|---|---|
| `text/` | 受管子进程的 stdout、stderr。启动器分流、标记来源并在后台写入。 |
| `system/` | 板端从 journal 收集本次开机的内核和 Firefly 服务日志，服务重启后从上次保存的游标续读。 |
| `rrd/` | Mimir 订阅 IPC 并写入 Rerun 观测。进程事件沿用 `Firefly/Log`。当前段写入 `.partial`，完成 footer 和文件同步之后发布 `.rrd`。每段携带蓝图、静态元数据和回放基线。 |
| `manifest.json` | 运行身份、创建顺序、生命周期、分段序号、各来源缺口与已淘汰范围。`started_ms` 和 `ended_ms` 分别记录归档创建和结束时的系统时间，单位为 Unix 毫秒。设备在校时前启动时，时间值可能落在 1970 年。RRD 中的测量时间保持原始 `sim_time`。 |

服务重启后，如果该来源最后一个文本分段以完整行结尾且还有空间，就接着写入该段。

SIL 和 USB HIL 正常退出时，启动器按任务时段和进程 ID 导出主机系统日志。启动器被强制终止时，可在主机的系统日志中查询这些事件。USB HIL 还会拷回板端代理就绪至任务返回期间的文本和 journal 日志。如果断连、重启或板端轮转造成缺失，主机清单会标明缺口。

## 录制与读取

先启动 Mimir，再启动生产者。所有生产者退出后向 Mimir 发送 SIGINT 或 SIGTERM，等待其排空、封口并退出。Mimir 在 stdout 输出 `recording: <运行目录绝对路径>`。以下命令中将 `run` 替换为该目录：

```bash
cargo build --release -p mimir
./target/release/mimir

run=logs/runs/log_0001
uv run rerun rrd verify "$run"/rrd/*.rrd
cargo xtask view "$run"
target/release/firefly-verify --transport board "$run"
```

可单独打开任一已封口分段排查近期事件。完整运行评分会检查分段连续性和录制缺口。同一目录内 Mimir 停止后再次启动，两次采集之间的覆盖范围记为未知。已有时间戳目录继续使用原目录名和 `run.json`，参与容量轮转。历史单文件 `logs/<启动时间>/log.rrd` 仍可直接读取。数据集导出的轨迹、地图及其他本地产物沿用各自的接口。

## 容量与恢复

固定安装配置为 [configs/archive.toml](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/archive.toml)，所有容量单位为字节。

| 配置 | 默认值 |
|---|---:|
| `max_bytes`，受管归档总预算 | 8 GiB |
| `cleanup_bytes`，触发清理后的目标 | 7 GiB |
| `keep_free_bytes`，文件系统可用余量 | 2 GiB |
| `rrd_segment_bytes`，RRD 切段目标 | 256 MiB |
| `stream_segment_bytes`，文本/系统流切段目标 | 16 MiB |
| `stream_segments`，每个文本/系统来源最多保留的段数 | 4 |

写入前在同一文件系统锁下分配字节预算，包括活动文件已承诺的写入和 RRD 收尾空间。归档管理器按文件内容长度计账，并为每次运行预留元数据空间。文件系统余量另按卷上的可用块检查。分段目标允许一个完整写入单元跨过阈值。RRD 在一段累计写入 4096 个数据块后也会切段，以限制索引开销。原始 journal 的保留由系统的 journald 配置管理。

空间不足时，先按 `sequence` 从旧到新删除可清理的运行目录。正在写入或读取的目录受文件锁保护。归档管理器先给待删目录改名，清理中断后可在下次启动时继续。如果仍然缺少空间，再回收当前运行中的旧分段，并保留最新已封口的 RRD。

可从清单、stderr 或板端 journal 查看写入暂停和恢复的原因。断电留下的 `.rrd.partial` 会让完整评分报告录制中断。轮转范围限于受管归档，`logs/` 下单独存放的测评结果、旧录制和运行锁由各自的调用方管理。

采集开关、时钟、观测注册表、队列预算和完整性验证见[可观测性文档](observability.md)。
