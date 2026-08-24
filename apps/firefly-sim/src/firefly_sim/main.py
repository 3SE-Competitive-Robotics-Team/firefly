"""firefly-sim 主循环（CLI 入口，见 `firefly_sim.__init__` 文档）。

运行方式：`uv run firefly-sim`（pyproject `[project.scripts]` 定义），
或 `uv run python -m firefly_sim`。

每个相机帧周期（10Hz）开一条新 OTel trace：周期内发布的 IMU/双目/深度/
真值共享同一 `trace_id`，Rust 侧续接后形成 传感器→vio→demo→参考 的闭环
单周期 trace；下一相机帧开新 trace（可区分每次输入）。
"""

from __future__ import annotations

import time
import sys

import iceoryx2 as iox2
import numpy as np

from firefly_mujoco import (
    DroneEnv,
    GrayImageMessage,
    DepthImageMessage,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    ImuMessage,
    OdomMessage,
    ReferenceMessage,
    TraceContext,
)

from . import trace as ftrace

#: 话题名（与 Rust `firefly-pubsub` 常量一致）
TOPIC_IMU = "Firefly/Imu"
TOPIC_CAM_LEFT = "Firefly/CameraLeft"
TOPIC_CAM_RIGHT = "Firefly/CameraRight"
#: 相机对事件（左右目成对发布完成后单次通知；与 Rust event::CAMERA_PAIR_TOPIC 一致）
TOPIC_CAM_PAIR = "Firefly/CameraPair"
TOPIC_DEPTH = "Firefly/Depth"
TOPIC_GT = "Firefly/GroundTruth"
TOPIC_REF = "Firefly/Reference"

#: 事件 id：「该话题有新样本」（与 Rust event::EVENT_ID_SENT_SAMPLE 一致）
EVENT_ID_SENT_SAMPLE = 0

#: 发布周期（秒）
IMU_PERIOD = 0.01  # 100Hz
CAM_PERIOD = 0.1  # 10Hz（双目 + 深度 + 真值；一个周期 = 一条 trace）
#: 物理步长（秒，200Hz）
PHYSICS_PERIOD = 0.005
#: 无人机起点（= demo 地图 start）
START_POS = np.array([1.0, 4.0, 1.0])


def _publisher(node, topic: str, payload_cls):
    service = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload_cls)
        .user_header(TraceContext)
        .open_or_create()
    )
    return service.publisher_builder().create()


def _subscriber(node, topic: str, payload_cls):
    service = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload_cls)
        .user_header(TraceContext)
        .open_or_create()
    )
    return service.subscriber_builder().create()


def _notifier(node, topic: str):
    """话题同名 event service 的通知端（订阅端 WaitSet 即到即醒）。"""
    service = node.service_builder(iox2.ServiceName.new(topic)).event().open_or_create()
    return service.notifier_builder().create()


_notify_failed = False


def _notify(notifier) -> None:
    """唤醒订阅端。失败不阻断物理循环（订阅端退化为兜底节拍轮询）。"""
    global _notify_failed
    try:
        notifier.notify_with_custom_event_id(iox2.EventId.new(EVENT_ID_SENT_SAMPLE))
    except Exception as exc:  # noqa: BLE001 - 通知绝不能拖垮仿真主循环
        if not _notify_failed:
            _notify_failed = True
            log(f"事件通知失败（后续静默）: {exc}")


def _publish_traced(pub, cycle, name: str, msg, ts: float) -> None:
    """带 trace 上下文发布：cycle 子 span → 填 User Header → 零拷贝发送。
    --no-trace 模式下 span=None，跳过 trace 操作，只做零拷贝发布。"""
    sample = pub.loan_uninit()
    span = ftrace.child_span(cycle, name)
    ftrace.fill_header(sample.user_header().contents, span, ts)
    if span is not None:
        span.end()
    sample.write_payload(msg).send()


def _scripted_ref(t: float) -> tuple[np.ndarray, np.ndarray]:
    """`--script` 模式：柔和脚本化参考（闭合周期轨迹），供 VIO 验证。

    planner/demo 不参与：直接给出平滑 (pos, vel)，让双目相机在无规划器的
    情况下获得受控运动。三轴取 T=20s 的整倍频正弦（Lissajous），位置/速度
    在周期边界连续——`--no-trace` 的 `t % 20` 循环不会产生参考跳变
    （此前线性前向 + 回卷导致 16m 跳变 → PD 发散 → MuJoCo QACC NaN）。
    """
    period = 20.0
    w1, w2, w3 = 2 * np.pi / period, 2 * 2 * np.pi / period, 3 * 2 * np.pi / period
    pos = np.array([
        1.0 + 3.0 * np.sin(w1 * t),
        4.0 + 1.0 * np.sin(w2 * t),
        1.0 + 0.5 * np.sin(w3 * t),
    ])
    vel = np.array([
        3.0 * w1 * np.cos(w1 * t),
        1.0 * w2 * np.cos(w2 * t),
        0.5 * w3 * np.cos(w3 * t),
    ])
    return pos, vel


def main() -> None:
    # --script：不使用 planner/demo，改由脚本参考驱动运动（VIO 验证用）
    # --no-trace：禁用 OTel tracing（消除 Python span 开销，sim 从 0.37x → 14x real-time）
    script_mode = "--script" in sys.argv
    trace_enabled = "--no-trace" not in sys.argv
    env = DroneEnv()
    env.reset(START_POS, np.array([0.0, 0.0, 0.0, 1.0]))  # xyzw 单位四元数
    ftrace.init(enabled=trace_enabled)
    log("MuJoCo 环境就绪：质量 {:.1f} kg，物理 {:.0f} Hz".format(env.mass, 1 / PHYSICS_PERIOD))
    if script_mode:
        log("--script：脚本化参考驱动运动（跳过 planner）")
    if not trace_enabled:
        log("--no-trace：OTel tracing 已禁用（高性能模式）")

    # 抑制 iceoryx2 内部告警噪音（如投递到历史残留幽灵监听端的
    # `FailedToDeliverSignal`——良性，数据面仍由订阅端兜底节拍驱动）。
    iox2.set_log_level(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    imu_pub = _publisher(node, TOPIC_IMU, ImuMessage)
    left_pub = _publisher(node, TOPIC_CAM_LEFT, GrayImageMessage)
    right_pub = _publisher(node, TOPIC_CAM_RIGHT, GrayImageMessage)
    depth_pub = _publisher(node, TOPIC_DEPTH, DepthImageMessage)
    gt_pub = _publisher(node, TOPIC_GT, OdomMessage)
    ref_sub = _subscriber(node, TOPIC_REF, ReferenceMessage)
    imu_notify = _notifier(node, TOPIC_IMU)
    cam_notify = _notifier(node, TOPIC_CAM_PAIR)
    log("iceoryx2 已就绪：发布 IMU/双目/深度/真值（带事件唤醒），订阅参考")

    # 参考状态（demo 未发布时悬停在起点）
    ref_pos = START_POS
    ref_vel = np.zeros(3)
    got_ref = False

    cycle = None
    next_imu = 0.0
    next_cam = 0.0
    frame = 0
    t_start = time.perf_counter()
    try:
        while True:
            # 拉取最新参考（非阻塞；其 trace 属已闭合周期，仅记录关联）
            while (sample := ref_sub.receive()) is not None:
                m = sample.payload().contents
                ref_pos = np.array([m.position_x, m.position_y, m.position_z])
                ref_vel = np.array([m.velocity_x, m.velocity_y, m.velocity_z])
                got_ref = True
                log("收到参考 t={:.3f} pos=({:.2},{:.2},{:.2}) trace={:032x}".format(
                    m.timestamp, m.position_x, m.position_y, m.position_z,
                    ftrace.header_trace_id(sample.user_header().contents),
                ))

            # 控制 + 物理步进
            if script_mode:
                # 脚本化参考：直接按仿真时刻给出平滑 pos/vel（跳过 planner）；
                # 轨迹严格 20s 周期，长跑直接用连续时间即可
                ref_pos, ref_vel = _scripted_ref(env.time)
            env.apply_pd(ref_pos, ref_vel)
            env.step()
            # 失稳守卫：MuJoCo 发散（QACC NaN/Inf）后状态永久污染且传感器
            # 全变 NaN，下游 VIO 会被毒化——检测到即重置到起点，长跑不挂。
            if not np.isfinite(env.data.qpos).all() or not np.isfinite(env.data.qvel).all():
                log("物理失稳（NaN/Inf）@ t={:.2f}，重置到起点".format(env.time))
                env.reset(START_POS, np.array([0.0, 0.0, 0.0, 1.0]))
            t = env.time
            frame += 1

            # 新相机帧 → 新周期 trace（周期内所有发布共享同一 trace_id）
            if t + 1e-12 >= next_cam:
                if cycle is not None:
                    ftrace.end_cycle(cycle)
                cycle = ftrace.start_cycle()

            # 100Hz IMU（当前周期的子 span）
            if t + 1e-12 >= next_imu:
                gyro, accel = env.imu()
                msg = ImuMessage()
                msg.timestamp = t
                msg.angular_velocity_x, msg.angular_velocity_y, msg.angular_velocity_z = gyro
                msg.linear_acceleration_x, msg.linear_acceleration_y, msg.linear_acceleration_z = accel
                _publish_traced(imu_pub, cycle, "publish-imu", msg, t)
                _notify(imu_notify)
                # 推进到 t 之后的网格点（防止 t 越过后下一步重复发布，
                # 保证 IMU 严格 0.01s 间隔、相机严格 0.1s 间隔）
                while next_imu <= t + 1e-12:
                    next_imu += IMU_PERIOD

            # 10Hz 双目 + 深度 + 真值
            if t + 1e-12 >= next_cam:
                _publish_camera(left_pub, right_pub, depth_pub, cycle, env, t)
                _notify(cam_notify)  # 左右目成对发布完成后单次唤醒
                _publish_gt(gt_pub, cycle, env, t)
                while next_cam <= t + 1e-12:
                    next_cam += CAM_PERIOD
                if (got_ref or script_mode) and frame % 200 == 0:
                    pos, _, vel = env.gt_pose()
                    log(
                        "t={:6.2f} 无人机 ({:6.2f},{:6.2f},{:6.2f}) 参考 ({:6.2f},{:6.2f},{:6.2f})".format(
                            t, pos[0], pos[1], pos[2], ref_pos[0], ref_pos[1], ref_pos[2]
                        )
                    )

            # 实时节奏（--no-trace 时仍限速 1x：步进 0.36ms << 5ms 预算，sleep 自动限速）
            wall = time.perf_counter() - t_start
            target = t
            if wall < target - PHYSICS_PERIOD:
                time.sleep(target - wall - PHYSICS_PERIOD)
    except KeyboardInterrupt:
        if cycle is not None:
            ftrace.end_cycle(cycle)
        log("退出")


def _publish_camera(left_pub, right_pub, depth_pub, cycle, env: DroneEnv, t: float) -> None:
    left = GrayImageMessage()
    left.timestamp = t
    left.sensor_id = 0
    left.width = IMAGE_WIDTH
    left.height = IMAGE_HEIGHT
    left.data[:] = env.render_left().reshape(-1)
    _publish_traced(left_pub, cycle, "publish-camera-left", left, t)

    right = GrayImageMessage()
    right.timestamp = t
    right.sensor_id = 1
    right.width = IMAGE_WIDTH
    right.height = IMAGE_HEIGHT
    right.data[:] = env.render_right().reshape(-1)
    _publish_traced(right_pub, cycle, "publish-camera-right", right, t)

    depth = DepthImageMessage()
    depth.timestamp = t
    depth.sensor_id = 0
    depth.width = IMAGE_WIDTH
    depth.height = IMAGE_HEIGHT
    depth.data[:] = env.render_depth().reshape(-1)
    _publish_traced(depth_pub, cycle, "publish-depth", depth, t)


def _publish_gt(gt_pub, cycle, env: DroneEnv, t: float) -> None:
    pos, quat_xyzw, vel = env.gt_pose()
    msg = OdomMessage()
    msg.timestamp = t
    msg.position_x, msg.position_y, msg.position_z = pos
    msg.velocity_x, msg.velocity_y, msg.velocity_z = vel
    msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w = quat_xyzw
    msg.is_initialized = True
    _publish_traced(gt_pub, cycle, "publish-gt", msg, t)


def log(msg: str) -> None:
    print(f"[firefly-sim] {msg}", flush=True)
