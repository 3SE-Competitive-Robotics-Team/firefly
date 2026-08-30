"""firefly-viz 主循环（CLI 入口，见 `firefly_viz.__init__` 文档）。

运行方式：`uv run firefly-viz [--save out.rrd] [--serve]`。

默认连共享 rerun viewer（`127.0.0.1:9876`，需先 `rerun` 起 viewer）；
`--serve` 由本进程起内置 viewer；`--save path.rrd` 离线录制（与 --serve
互斥）。无任何选项时自动 `rerun.spawn()` 起 viewer 并连接。
"""

from __future__ import annotations

import argparse
import sys
import time

import iceoryx2 as iox2
import numpy as np
import rerun as rr

from firefly_mujoco import TraceContext

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

#: 话题名（与 Rust `firefly_pubsub::viz::VIZ_TOPIC` 一致）
TOPIC_VIZ = "Firefly/Viz"

#: 共享 ApplicationId / RecordingId（多进程共享同一 recording，对照已删
#: firefly-rerun 的 APP_ID/RECORDING_ID：各进程流合并为 viewer 单应用）
APP_ID = "firefly"
RECORDING_ID = "firefly-sim-loop"

#: 订阅缓冲区深度：多实体（vio 4 条 + planner 5+ 条）10Hz 突发，防溢出丢帧
VIZ_BUFFER_SIZE = 256


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
    rr.send_blueprint(rr.blueprint.Blueprint(scene))
    log("已发送默认布局（场景 3D，sim_time 全历史可见范围）")


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


def _subscribe(node) -> iox2.Subscriber:
    service = (
        node.service_builder(iox2.ServiceName.new(TOPIC_VIZ))
        .publish_subscribe(VizMessage)
        .user_header(TraceContext)
        # 服务级环形缓冲历史上限：与 Rust 发布端一致（先启动方创建服务，
        # 谁创建都要给出 256 上限，订阅端 buffer_size 才能匹配）
        .subscriber_max_buffer_size(VIZ_BUFFER_SIZE)
        .open_or_create()
    )
    return service.subscriber_builder().buffer_size(VIZ_BUFFER_SIZE).create()


def main() -> None:
    args = _parse_args()
    if args.save and args.serve:
        sys.exit("[firefly-viz] --save 与 --serve 互斥，只能二选一")
    iox2.set_log_level(iox2.LogLevel.Error)
    _open_recording(args)
    _send_default_blueprint()
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    sub = _subscribe(node)
    log(f"iceoryx2 已订阅 {TOPIC_VIZ}（Rust 计算线程零 IO，统一写 rerun）")
    try:
        while True:
            while (sample := sub.receive()) is not None:
                header = sample.user_header().contents
                trace_id = f"{header.trace_id_hi:016x}{header.trace_id_lo:016x}"
                _handle(sample.payload().contents, trace_id)
            # 拉模型：无新样本时让出 CPU（事件通知缺席时也不 busy-loop）
            time.sleep(0.001)
    except KeyboardInterrupt:
        log("退出")


if __name__ == "__main__":
    main()
