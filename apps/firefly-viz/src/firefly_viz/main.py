"""firefly-viz 主循环（CLI 入口，见 `firefly_viz.__init__` 文档）。

运行方式：`uv run firefly-viz [--save out.rrd] [--serve]`。

默认连共享 rerun viewer（`127.0.0.1:9876`，需先 `rerun` 起 viewer）；
`--serve` 由本进程起内置 viewer；`--save path.rrd` 离线录制（与 --serve
互斥）。无任何选项时自动 `rerun.spawn()` 起 viewer 并连接。

线程模型：收/落盘分离（channel IPC + 落盘批处理）——
- 收线程（主线程）：排空 `Firefly/Viz` + `Firefly/Log` 订阅，把消息
  `put_nowait` 进有界队列（满即丢，日志语义，见 `QUEUE_MAX`），不做 rerun IO；
- 落盘线程：`get` 阻塞消费，按（时间戳，序号）排序后批量 `rr.log`，
  每批最多 `BATCH_MAX` 条或 `FLUSH_PERIOD` 超时刷一次。
"""

from __future__ import annotations

import argparse
import ctypes
import faulthandler
import os
import queue
import sys
import threading
import time
from pathlib import Path

import iceoryx2 as iox2
import numpy as np
import rerun as rr

from firefly_mujoco import LogMessage, TraceContext

from .messages import (
    VIZ_KIND_ARROWS,
    VIZ_KIND_BAR_CHART,
    VIZ_KIND_CLEAR,
    VIZ_KIND_LINE_STRIP,
    VIZ_KIND_POSE,
    VIZ_KIND_SCALARS,
    VIZ_KIND_VOXELS,
    VizMessage,
)

#: 话题名（与 Rust `firefly_pubsub::{viz::VIZ_TOPIC, log::LOG_TOPIC}` 一致）
TOPIC_VIZ = "Firefly/Viz"
TOPIC_LOG = "Firefly/Log"

#: 共享 ApplicationId / RecordingId（多进程共享同一 recording，对照已删
#: firefly-rerun 的 APP_ID/RECORDING_ID：各进程流合并为 viewer 单应用）
APP_ID = "firefly"
RECORDING_ID = "firefly-sim-loop"

#: 订阅缓冲区深度：多实体（vio 4 条 + planner 5+ 条）10Hz 突发，防溢出丢帧
VIZ_BUFFER_SIZE = 256
#: 日志订阅缓冲：`Firefly/Log` 低频（各进程数条/秒），2 足够
#:（服务默认 `subscriber_max_buffer_size` 即 2，不显式调大）。

#: 收/落盘队列上限：日志低频，1024 足够突发；满即丢（日志语义）。
QUEUE_MAX = 1024
#: 落盘批上限（条）：单批排序后写入，防长尾延迟。
BATCH_MAX = 256
#: 落盘刷盘周期（秒）：超时即刷，不足一批也写。
FLUSH_PERIOD = 0.2

#: 日志级别映射（Rust `firefly_pubsub::log::level` → rerun `TextLogLevel`）
_LEVELS = {1: "ERROR", 2: "WARN", 3: "INFO", 4: "DEBUG", 5: "TRACE"}


def log(msg: str) -> None:
    print(f"[firefly-viz] {msg}", flush=True)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="firefly-viz",
        description="订阅 Firefly/Viz 话题，统一写 rerun viewer / rrd",
    )
    p.add_argument("--save", metavar="out.rrd", help="离线录制到 rrd 文件")
    p.add_argument("--serve", action="store_true", help="本进程起内置 rerun viewer")
    p.add_argument("--connect", metavar="URL", default=None, help="rerun viewer gRPC 地址（缺省 127.0.0.1:9876）")
    return p.parse_args()


def _open_recording(args: argparse.Namespace) -> None:
    """初始化 rerun recording（app_id + recording_id 共享）并打开目标。

    --save 优先，其次 --serve / --connect，否则连共享 viewer。
    """
    rr.init(APP_ID, recording_id=RECORDING_ID)
    if args.save:
        rr.save(args.save)
        log(f"离线录制到 {args.save}")
    elif args.serve:
        rr.spawn(port=9876, connect=True)
        log("已启动内置 viewer 并连接（127.0.0.1:9876）")
    elif args.connect:
        rr.connect_grpc(args.connect)
        log(f"连接共享 viewer {args.connect}")
    else:
        rr.connect_grpc("127.0.0.1:9876")
        log("连接共享 viewer 127.0.0.1:9876（需先起 `rerun`；或加 --serve）")


def _send_default_blueprint() -> None:
    """默认布局：场景 3D 视图 + 全部空间实体；sim_time 配 [最早, 游标] 可见
    时间范围，viewer 端用 range 查询聚合全部历史增量段（增量写 + 全量显示，
    对照 rerun 官方示例 line_strips3d_time_window）。"""
    scene = rr.blueprint.Spatial3DView(
        origin="/",
        contents=["+ /**"],
        time_ranges=[
            rr.blueprint.VisibleTimeRange(
                "sim_time",
                start=rr.blueprint.TimeRangeBoundary.infinite(),
                end=rr.blueprint.TimeRangeBoundary.cursor_relative(),
            )
        ],
    )
    # 时间面板锁定 `sim_time`（全链路统一时钟；不限则 viewer 默认落到
    # `log_time`，轨迹实体不可见）+ 游标跟随最新（回放直接看到整条轨迹）。
    time_panel = rr.blueprint.TimePanel(
        timeline="sim_time",
        play_state=rr.blueprint.components.PlayState.Following,
    )
    rr.send_blueprint(rr.blueprint.Blueprint(scene, time_panel))
    log("已发送默认布局（场景 3D，sim_time 全历史可见范围，时间面板锁定 sim_time）")


#: 静态场景 mesh 实体（世界系，与仿真同原点同单位，见下）。
ENTITY_WAREHOUSE_MESH = "world/warehouse"


def _repo_root() -> Path:
    """仓库根（`firefly_mujoco` 包位置反推，与启动 CWD 无关）。

    `__init__.py` 的五级父目录：包 → `src` → 包发行目录 → `packages` → 根。
    """
    import firefly_mujoco

    return Path(firefly_mujoco.__file__).resolve().parent.parent.parent.parent.parent


def _log_static_mesh() -> None:
    """静态场景 mesh（`world/warehouse`，`Asset3D` 一次性，无时间轴）。

    场景选择与 `sim` 同源（`FIREFLY_SCENE`，缺省 `warehouse`）；只有对应场景
    的 mesh 文件存在才记（boxes 等程序化场景无 mesh 文件，自然跳过——
    绝不在错误的场景上叠别家的房子）。坐标系：`structure.obj` 即世界系
    （米，与仿真同原点，x∈[0,46]、y∈[-8,8]、z∈[0,5]），直接落盘不做变换。
    """
    if os.environ.get("FIREFLY_SCENE", "warehouse") != "warehouse":
        log("非 warehouse 场景，不记静态 mesh")
        return
    mesh_path = Path(
        os.environ.get(
            "FIREFLY_MESH", str(_repo_root() / "models" / "warehouse" / "structure.obj")
        )
    )
    if not mesh_path.is_file():
        log(f"静态 mesh 缺失（{mesh_path}），跳过")
        return
    rr.log(ENTITY_WAREHOUSE_MESH, rr.Asset3D(path=mesh_path), static=True)
    log(f"已记静态场景 mesh（{ENTITY_WAREHOUSE_MESH}，{mesh_path.stat().st_size // 1024}KB）")


def detach_copy(msg):
    """零拷贝 loan 的私有化深拷贝：进跨线程队列前必须调用。

    `sample.payload().contents` 只是 loan 内存的别名包装——发布端复用 loan
    发下一帧（覆盖）或进程退出（解除映射）后，队列里滞留的别名读到撕裂
    数据（rrd 里 `vio/deb` 类垃圾实体即此来源），乃至释放后使用直接
    `SIGSEGV`（writer 线程滞后秒级，窗口极大）。拷贝一次（`VizMessage`
    约 217KB，百 Hz 下约数 MB/s）买断生命周期；背压丢弃只影响新鲜度。
    """
    cls = type(msg)
    dst = cls()
    ctypes.memmove(ctypes.addressof(dst), ctypes.addressof(msg), ctypes.sizeof(cls))
    return dst


def _entity(msg: VizMessage) -> str:
    n = min(msg.entity_len, len(msg.entity))
    return bytes(msg.entity[:n]).decode("utf-8", errors="replace")


def _handle(msg: VizMessage, trace_id: str) -> None:
    kind = msg.kind
    entity = _entity(msg)
    rr.set_time("sim_time", duration=msg.timestamp)

    if kind == VIZ_KIND_POSE:
        # 四元数 xyzw 分量顺序与 Rust JPL 一致，直接透传（rerun Quaternion 为 xyzw）
        rr.log(
            entity,
            rr.Transform3D(
                translation=[msg.xyz[0], msg.xyz[1], msg.xyz[2]],
                quaternion=[msg.quat_xyzw[0], msg.quat_xyzw[1], msg.quat_xyzw[2], msg.quat_xyzw[3]],
            ),
        )
    elif kind == VIZ_KIND_LINE_STRIP:
        n = msg.point_count
        pts = [[msg.points[i][0], msg.points[i][1], msg.points[i][2]] for i in range(n)]
        rr.log(entity, rr.LineStrips3D(strips=[pts], colors=[[*msg.color, 255]]))
    elif kind == VIZ_KIND_VOXELS:
        n = msg.voxel_count
        indices = [[msg.voxels[i][0], msg.voxels[i][1], msg.voxels[i][2]] for i in range(n)]
        # 不传 colors：rerun 0.36 的 colors 路径在体素数超 ~1000 时渲染性能
        # 悬崖（实测 4590 体素 30s+ 卡死）；去掉 colors 用默认着色 <0.5s
        rr.log(
            entity,
            rr.VoxelGridMap(
                indices,
                [msg.voxel_size[0], msg.voxel_size[1], msg.voxel_size[2]],
                translation=[msg.voxel_origin[0], msg.voxel_origin[1], msg.voxel_origin[2]],
            ),
        )
    elif kind == VIZ_KIND_SCALARS:
        n = msg.scalar_count
        rr.log(entity, rr.Scalars([msg.scalars[i] for i in range(n)]))
    elif kind == VIZ_KIND_BAR_CHART:
        n = msg.bin_count
        values = [float(msg.bins[i]) for i in range(n)]
        # x 轴 bin 标注：首桶下界 bin_start、桶宽 bin_width（1 时等价于桶序号）
        rr.log(entity, rr.BarChart(values, abscissa=_bin_abscissa(msg)))
    elif kind == VIZ_KIND_ARROWS:
        n = msg.arrow_count
        origins = [[msg.arrow_origins[i][0], msg.arrow_origins[i][1], msg.arrow_origins[i][2]] for i in range(n)]
        vectors = [[msg.arrow_vectors[i][0], msg.arrow_vectors[i][1], msg.arrow_vectors[i][2]] for i in range(n)]
        rr.log(entity, rr.Arrows3D(vectors=vectors, origins=origins, colors=[[*msg.color, 255]]))
    elif kind == VIZ_KIND_CLEAR:
        rr.log(entity if entity else "/", rr.Clear(recursive=True))
    else:
        log(f"未知 kind {kind}（entity={entity}，trace={trace_id}），忽略")


def _bin_abscissa(msg: VizMessage) -> list[float]:
    """BarChart 的 x 轴坐标：bin_start + k*bin_width。"""
    n = msg.bin_count
    return [msg.bin_start + k * msg.bin_width for k in range(n)]


def _log_text(msg: LogMessage) -> None:
    """结构化日志 → rerun `TextLog`（实体 `logs/<tag>`，持久可检索）。

    时间轴规则（问答决议）：`sim_time >= 0` 落 `sim_time` 轴（与位姿同轴
    对齐）；`< 0`（发布端尚无 sim 时钟，如 ORT 模型加载期）落墙钟轴
    （`wall_secs + wall_nanos`，显式未对齐、可检索）。
    """
    n_tag = min(msg.tag_len, len(msg.tag))
    tag = bytes(msg.tag[:n_tag]).decode("utf-8", errors="replace") or "unknown"
    n_text = min(msg.text_len, len(msg.text))
    text = bytes(msg.text[:n_text]).decode("utf-8", errors="replace")
    level = _LEVELS.get(msg.log_level, "INFO")
    if msg.sim_time >= 0:
        rr.set_time("sim_time", duration=float(msg.sim_time))
    else:
        # 墙钟回落轴：`set_time(timeline, timestamp=...)` 取 unix 秒
        #（纳秒值会被 rerun 误判为毫秒，见 `to_nanos_since_epoch` 校验）。
        rr.set_time("wall_time", timestamp=float(msg.wall_secs) + float(msg.wall_nanos) * 1e-9)
    rr.log(f"logs/{tag}", rr.TextLog(text, level=level))


def _subscribe(node, topic: str, payload_cls, buffer_size: int = VIZ_BUFFER_SIZE):
    builder = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload_cls)
        .user_header(TraceContext)
    )
    if topic == TOPIC_VIZ:
        # 高频突发话题：与 Rust 发布端一致（先启动方创建服务，
        # 谁创建都要给出 256 上限，订阅端 buffer_size 才能匹配）
        builder = builder.subscriber_max_buffer_size(VIZ_BUFFER_SIZE)
    service = builder.open_or_create()
    return service.subscriber_builder().buffer_size(buffer_size).create()


#: 同话题发布端上限（与 Rust `firefly_pubsub::log::LOG_MAX_PUBLISHERS` 一致；
#: 本进程先创建服务定上限，Rust 发布端只 open，见 `_precreate_log_service`）。
LOG_MAX_PUBLISHERS = 10


def _precreate_log_service(node) -> None:
    """预创建 `Firefly/Log` 服务（`max_publishers=10` 定上限）。

    本进程常驻、先于全部 Rust 发布端启动（runbook 顺序），故上限恒成立。
    预创建后立即关闭临时订阅端（服务定义保留，发布端随后可 open）。
    """
    service = (
        node.service_builder(iox2.ServiceName.new(TOPIC_LOG))
        .publish_subscribe(LogMessage)
        .user_header(TraceContext)
        .max_publishers(LOG_MAX_PUBLISHERS)
        .open_or_create()
    )
    # 返回值必须存活：Python 绑定里 builder 链式调用返回新对象（`is` 为 False，
    # 见探针），`service` 局部变量持有服务句柄；订阅端建在服务上（不弃置）。
    return service.subscriber_builder().buffer_size(2).create()


def _writer_loop(inbox: queue.Queue) -> None:
    """落盘线程：阻塞消费 → 按（时间戳，序号）排序 → 批量 `rr.log`。

    第一元素的序号保证同时间戳多实体间的稳定顺序（`itertools.count`
    单调递增，跨实体可比）；`FLUSH_PERIOD` 超时即刷，不足一批也写。
    单条写入异常只记 stderr（落盘线程常驻，不因一条坏消息退出——
    否则其后全部日志/可视化静默丢失）。
    """
    import itertools
    import traceback

    seq = itertools.count()
    buf: list = []
    deadline = time.monotonic() + FLUSH_PERIOD
    while True:
        timeout = max(0.0, deadline - time.monotonic())
        try:
            item = inbox.get(timeout=timeout)
            if item is None:  # 退出哨兵：排空残余后返回
                break
            kind, ts, payload = item
            buf.append((ts, next(seq), kind, payload))
            if len(buf) >= BATCH_MAX:
                _flush_sorted(buf)
                buf.clear()
                deadline = time.monotonic() + FLUSH_PERIOD
        except queue.Empty:
            pass
        except Exception:  # noqa: BLE001 - 落盘线程永不退出
            print("[firefly-viz] writer 入队异常（跳过）：", flush=True)
            traceback.print_exc()
        if time.monotonic() >= deadline and buf:
            _flush_sorted(buf)
            buf.clear()
            deadline = time.monotonic() + FLUSH_PERIOD
    for ts, _, kind, payload in sorted(buf):
        try:
            if kind == "viz":
                msg, trace_id = payload
                _handle(msg, trace_id)
            else:
                _log_text(payload)
        except Exception:  # noqa: BLE001 - 退出排空尽力而为
            traceback.print_exc()


def _flush_sorted(buf: list) -> None:
    import traceback

    for _, _, kind, payload in sorted(buf):
        try:
            if kind == "viz":
                msg, trace_id = payload
                _handle(msg, trace_id)
            else:
                _log_text(payload)
        except Exception:  # noqa: BLE001 - 单条坏消息不杀落盘线程
            print("[firefly-viz] writer 落盘异常（跳过本条）：", flush=True)
            traceback.print_exc()


def main() -> None:
    # native 段错误时打 Python 栈（`rerun` 底层是 Rust，裸 `SIGSEGV` 否则无信息）。
    faulthandler.enable()
    args = _parse_args()
    if args.save and args.serve:
        sys.exit("[firefly-viz] --save 与 --serve 互斥，只能二选一")
    iox2.set_log_level(iox2.LogLevel.Error)
    _open_recording(args)
    _send_default_blueprint()
    _log_static_mesh()
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    viz_sub = _subscribe(node, TOPIC_VIZ, VizMessage)
    # `Firefly/Log` 服务必须由本进程先创建（`max_publishers=10`，7 进程同话题
    # 发布；iceoryx2 `open_or_create` 语义：先创建方定上限，后续 open 必须 ≤
    # 该值——Rust 发布端只 `open` 不创建，见 `LogPublisher::new`）。
    # 预创建返回的订阅端即本进程的 Log 订阅端（服务句柄由其持有，不另建）。
    log_sub = _precreate_log_service(node)
    log(f"iceoryx2 已订阅 {TOPIC_VIZ} + {TOPIC_LOG}（Rust 计算线程零 IO，统一写 rerun）")
    inbox: queue.Queue = queue.Queue(maxsize=QUEUE_MAX)
    writer = threading.Thread(target=_writer_loop, args=(inbox,), name="firefly-viz-writer", daemon=True)
    writer.start()
    dropped = 0
    try:
        while True:
            # 收线程：只排空订阅、进队，不做 rerun IO（落盘由 writer 线程批处理）
            while (sample := viz_sub.receive()) is not None:
                header = sample.user_header().contents
                trace_id = f"{header.trace_id_hi:016x}{header.trace_id_lo:016x}"
                # 深拷贝后进队（见 `detach_copy`：别名滞留即撕裂读/`SIGSEGV`）。
                msg = detach_copy(sample.payload().contents)
                if _enqueue(inbox, ("viz", float(msg.timestamp), (msg, trace_id))):
                    dropped += 1
            while (sample := log_sub.receive()) is not None:
                msg = detach_copy(sample.payload().contents)
                ts = float(msg.sim_time) if msg.sim_time >= 0 else float(msg.wall_secs) + float(msg.wall_nanos) * 1e-9
                if _enqueue(inbox, ("log", ts, msg)):
                    dropped += 1
            # 拉模型：无新样本时让出 CPU（事件通知缺席时也不 busy-loop）
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        inbox.put(None)
        writer.join(timeout=5.0)
        if dropped:
            log(f"退出（队列满丢弃 {dropped} 条）")
        else:
            log("退出")


def _enqueue(inbox: queue.Queue, item) -> bool:
    """进队；队列满返回 `True`（调用方计数），落盘永不阻塞接收线程。"""
    try:
        inbox.put_nowait(item)
    except queue.Full:
        return True
    return False


if __name__ == "__main__":
    main()
