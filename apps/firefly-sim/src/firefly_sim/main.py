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
    LOG_LEVEL_ERROR,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_TOPIC,
    DroneEnv,
    GrayImageMessage,
    DepthImageMessage,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    ImuMessage,
    LogMessage,
    OdomMessage,
    ReferenceMessage,
    TraceContext,
)

from . import trace as ftrace
from .trajectories import Trajectory, get_trajectory

#: 话题名（与 Rust `firefly-pubsub` 常量一致）
TOPIC_IMU = "Firefly/Imu"
TOPIC_CAM_LEFT = "Firefly/CameraLeft"
TOPIC_CAM_RIGHT = "Firefly/CameraRight"
#: 相机对事件（左右目成对发布完成后单次通知；与 Rust event::CAMERA_PAIR_TOPIC 一致）
TOPIC_CAM_PAIR = "Firefly/CameraPair"
TOPIC_DEPTH = "Firefly/Depth"
TOPIC_GT = "Firefly/GroundTruth"
TOPIC_REF = "Firefly/Reference"
#: 状态源里程计（启动互锁：任务时钟等它首个 is_initialized=true 才走；
#: 状态源由 --odom-topic 选择：vio（`Firefly/Odometry`）或 void
#: （`Firefly/VoidOdom`，DIVO 里程计 A/B 对比用）。
TOPIC_ODOM = "Firefly/Odometry"
TOPIC_VOIDODOM = "Firefly/VoidOdom"
#: 任务启动超时（秒）：上电后无 ready 则报错退出（fail loudly），
#: 不静默起飞。估计器侧 GT 等待 30s 是双保险，这里先触发。
MISSION_TIMEOUT = 15.0

#: 事件 id：「该话题有新样本」（与 Rust event::EVENT_ID_SENT_SAMPLE 一致）
EVENT_ID_SENT_SAMPLE = 0


def load_config(path: str = "configs/sim.toml") -> dict:
    """加载 TOML 配置（缺键回落代码默认值；文件缺失即退出）。"""
    import tomllib

    try:
        with open(path, "rb") as f:
            return tomllib.load(f)
    except FileNotFoundError as exc:
        sys.exit(f"[firefly-sim] 配置文件缺失：{path}（{exc}）")
    except tomllib.TOMLDecodeError as exc:
        sys.exit(f"[firefly-sim] 配置解析失败：{path}（{exc}）")


#: IMU 服务订阅端上限（与 Rust `imu.rs: IMU_SERVICE_MAX` 同值；先创建方定上限，
#: 后续 open 不得超限——bench 流程 sim 先起，手动流程 vio 先起，两边必须一致）。
IMU_SERVICE_MAX = 128


def _publisher(node, topic: str, payload_cls, subscriber_max: int | None = None):
    builder = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload_cls)
        .user_header(TraceContext)
    )
    if subscriber_max is not None:
        builder = builder.subscriber_max_buffer_size(subscriber_max)
    service = builder.open_or_create()
    return service.publisher_builder().create()


def _subscriber(node, topic: str, payload_cls):
    service = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload_cls)
        .user_header(TraceContext)
        .open_or_create()
    )
    return service.subscriber_builder().create()


def advance_grid(next_t: float, t: float, period: float) -> tuple[float, int]:
    """推进网格时刻，返回（新时刻，跳过的格点数）。

    跳过发生在物理步进跟不上发布节拍时（进程被抢占数百 ms）：跳过的采样/帧
    永久丢失（物理不可倒带），由调用方计数告警。正常节拍下返回跳过 0。
    """
    skipped = 0
    while next_t <= t + 1e-12:
        next_t += period
        skipped += 1
    return next_t, skipped - 1


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


def main() -> None:
    # --script [NAME]：不使用 planner，改由具名轨迹生成器驱动运动（VIO 验证用）；
    # 省略 NAME 时用 lissajous_classic（历史基线曲线）
    # --no-trace：禁用 OTel tracing（消除 Python span 开销，sim 从 0.37x → 14x real-time）
    script_mode = "--script" in sys.argv
    trajectory_name = "lissajous_classic"
    if script_mode:
        idx = sys.argv.index("--script")
        if idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith("-"):
            trajectory_name = sys.argv[idx + 1]
    # 状态源选择（启动互锁监听的话题）：缺省 vio；--odom-topic 可切 void。
    odom_topic = TOPIC_ODOM
    if "--odom-topic" in sys.argv:
        idx = sys.argv.index("--odom-topic")
        if idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith("-"):
            odom_topic = sys.argv[idx + 1]
    if odom_topic not in (TOPIC_ODOM, TOPIC_VOIDODOM):
        sys.exit(f"[firefly-sim] --odom-topic 非法：{odom_topic}（仅支持 {TOPIC_ODOM} / {TOPIC_VOIDODOM}）")
    trajectory: Trajectory = get_trajectory(trajectory_name)
    trace_enabled = "--no-trace" not in sys.argv
    cfg = load_config()
    rates = cfg.get("rates", {})
    physics_period = 1.0 / rates.get("physics", 200.0)
    imu_period = 1.0 / rates.get("imu", 100.0)
    cam_period = 1.0 / rates.get("cam", 10.0)
    start_pos = np.array(cfg.get("start", [1.0, 4.0, 1.0]))
    env = DroneEnv()
    env.reset(start_pos, np.array([0.0, 0.0, 0.0, 1.0]))  # xyzw 单位四元数
    ftrace.init(enabled=trace_enabled)
    log("MuJoCo 环境就绪：质量 {:.1f} kg，物理 {:.0f} Hz".format(env.mass, 1 / physics_period))
    if script_mode:
        log(f"--script：轨迹 {trajectory.name} 驱动运动（跳过 planner）")
    if not trace_enabled:
        log("--no-trace：OTel tracing 已禁用（高性能模式）")

    # 抑制 iceoryx2 内部告警噪音（如投递到历史残留幽灵监听端的
    # `FailedToDeliverSignal`——良性，数据面仍由订阅端兜底节拍驱动）。
    iox2.set_log_level(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    imu_pub = _publisher(node, TOPIC_IMU, ImuMessage, IMU_SERVICE_MAX)
    left_pub = _publisher(node, TOPIC_CAM_LEFT, GrayImageMessage)
    right_pub = _publisher(node, TOPIC_CAM_RIGHT, GrayImageMessage)
    depth_pub = _publisher(node, TOPIC_DEPTH, DepthImageMessage)
    gt_pub = _publisher(node, TOPIC_GT, OdomMessage)
    ref_sub = _subscriber(node, TOPIC_REF, ReferenceMessage)
    # 启动互锁订阅（--script 模式）：状态源 ready 电平（is_initialized），
    # 任务时钟据此启动；电平（非边沿）语义——晚订阅 100ms 内必收到。
    odom_sub = _subscriber(node, odom_topic, OdomMessage)
    imu_notify = _notifier(node, TOPIC_IMU)
    cam_notify = _notifier(node, TOPIC_CAM_PAIR)
    _init_log_ipc(node)
    log("iceoryx2 已就绪：发布 IMU/双目/深度/真值（带事件唤醒），订阅参考")

    # 参考状态（demo 未发布时悬停在起点）
    ref_pos = start_pos
    ref_vel = np.zeros(3)
    got_ref = False
    # 任务时钟（--script 模式）：None = 等 VOID 就绪中，原地悬停；
    # 收到首个 is_initialized=true  latch 为当前仿真时刻，此后轨迹
    # 时间 = sim 时间 - 任务起点（t_go 等起飞等待相对任务起点，不含
    # 启动不定耗时——三个旧定时器退役的落点）。
    mission_t0 = None

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
                # 启动互锁：先排空状态源 odom，有 ready 就 latch 任务起点；
                # 超时无 ready 则报错退出（fail loudly）。
                if mission_t0 is None:
                    while (sample := odom_sub.receive()) is not None:
                        if sample.payload().contents.is_initialized:
                            mission_t0 = env.time
                            log(f"状态源就绪（{odom_topic}），任务时钟启动 t0={mission_t0:.2f}")
                            break
                    if mission_t0 is None and env.time > MISSION_TIMEOUT:
                        sys.exit(
                            f"[firefly-sim] 任务启动超时：{MISSION_TIMEOUT:.0f}s 未收到状态源 ready "
                            f"（{odom_topic} 是否存活？iceoryx2 是否残留幽灵服务？）"
                        )
                if mission_t0 is None:
                    # 未就绪：原地悬停（位置=起点，速度=0）
                    ref_pos, ref_vel = start_pos, np.zeros(3)
                else:
                    # 脚本化参考：按任务时刻给出平滑 pos/vel；实例满足周期连续
                    # 不变量（见 trajectories.py），长跑直接用连续时间即可
                    ref_pos, ref_vel = trajectory.ref(env.time - mission_t0)
            env.apply_pd(ref_pos, ref_vel)
            env.step()
            # 失稳守卫：MuJoCo 发散（QACC NaN/Inf）后状态永久污染且传感器
            # 全变 NaN，下游 VIO 会被毒化——检测到即重置到起点，长跑不挂。
            if not np.isfinite(env.data.qpos).all() or not np.isfinite(env.data.qvel).all():
                log("物理失稳（NaN/Inf）@ t={:.2f}，重置到起点".format(env.time), LOG_LEVEL_ERROR, env.time)
                env.reset(start_pos, np.array([0.0, 0.0, 0.0, 1.0]))
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
                next_imu, skipped = advance_grid(next_imu, t, imu_period)
                if skipped >= 1:
                    # 物理不可倒带：跳过的采样永久丢失（VIO 断流告警的对端证据）。
                    log(
                        "IMU 追赶跳过 {} 采样（{:.2f}s 数据丢失）".format(
                            skipped, skipped * imu_period
                        ),
                        LOG_LEVEL_WARN,
                        t,
                    )

            # 10Hz 双目 + 深度 + 真值
            if t + 1e-12 >= next_cam:
                _publish_camera(left_pub, right_pub, depth_pub, cycle, env, t)
                _notify(cam_notify)  # 左右目成对发布完成后单次唤醒
                _publish_gt(gt_pub, cycle, env, t)
                next_cam, skipped = advance_grid(next_cam, t, cam_period)
                if skipped >= 1:
                    log(
                        "相机追赶跳过 {} 帧（{:.2f}s 图像丢失）".format(
                            skipped, skipped * cam_period
                        ),
                        LOG_LEVEL_WARN,
                        t,
                    )
                if (got_ref or script_mode) and frame % 200 == 0:
                    pos, _, vel = env.gt_pose()
                    log(
                        "t={:6.2f} 无人机 ({:6.2f},{:6.2f},{:6.2f}) 参考 ({:6.2f},{:6.2f},{:6.2f})".format(
                            t, pos[0], pos[1], pos[2], ref_pos[0], ref_pos[1], ref_pos[2]
                        ),
                        LOG_LEVEL_INFO,
                        t,
                    )

            # 实时节奏（--no-trace 时仍限速 1x：步进 0.36ms << 5ms 预算，sleep 自动限速）
            wall = time.perf_counter() - t_start
            target = t
            if wall < target - physics_period:
                time.sleep(target - wall - physics_period)
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


def log(msg: str, level: int = LOG_LEVEL_INFO, sim_time: float = -1.0) -> None:
    """双 sink 日志：stderr 打印 + `Firefly/Log` 聚合（rrd 持久可检索）。

    存量调用点保持 `log(text)` 不变（INFO + 无 sim 时钟）；主循环内逐步
    传入 `sim_time`（与位姿同轴对齐，可检索）。
    """
    print(f"[firefly-sim] {msg}", flush=True)
    publish_log(level, msg, sim_time)


#: 日志聚合发布端（`Firefly/Log` → `firefly-viz`；stderr 双 sink 保留，
#: 见 `firefly_observability::IpcAppend` 的双 sink 语义）。
_log_pub = None


def _init_log_ipc(node) -> None:
    """挂载日志聚合（建端失败降级纯 stderr，不阻断物理循环）。

    sim_time 由调用方每帧传入（`publish_log` 参数），此处只建端。
    `max_publishers` 与 Rust `LOG_MAX_PUBLISHERS` 一致（10）：本进程常与
    viz 同早启动，谁先创建服务谁定上限——必须显式调大，默认 2 只够
    sim+viz，Rust 发布端随后 open 即 `DoesNotSupportRequestedAmountOfPublishers`。
    """
    global _log_pub
    try:
        service = (
            node.service_builder(iox2.ServiceName.new(LOG_TOPIC))
            .publish_subscribe(LogMessage)
            .user_header(TraceContext)
            .max_publishers(10)
            .open_or_create()
        )
        _log_pub = service.publisher_builder().create()
        log("日志聚合已挂载（tag=sim，Firefly/Log → firefly-viz）")
    except Exception as exc:  # noqa: BLE001 - 聚合缺席不阻断仿真
        log(f"日志聚合挂载失败（降级纯 stderr）：{exc}")


def publish_log(level: int, text: str, sim_time: float = -1.0) -> None:
    """结构化日志发布（stderr + IPC 双 sink；调用点保持 `log()` 不变）。

    `sim_time < 0` 表示尚无 sim 时钟（聚合端回落墙钟轴，可检索）。
    发布失败静默丢弃：可视化不阻断物理循环。
    """
    if _log_pub is None:
        return
    try:
        import time as _time

        wall = _time.time()
        sample = _log_pub.loan_uninit()
        msg = LogMessage()
        msg.log_level = level
        tag = b"sim"
        msg.tag[: len(tag)] = tag
        msg.tag_len = len(tag)
        raw = text.encode("utf-8")[:512]
        msg.text[: len(raw)] = raw
        msg.text_len = len(raw)
        msg.sim_time = sim_time
        msg.wall_secs = int(wall)
        msg.wall_nanos = int((wall % 1.0) * 1e9)
        sample.write_payload(msg).send()
    except Exception:  # noqa: BLE001 - 日志路径永不抛异常
        pass
