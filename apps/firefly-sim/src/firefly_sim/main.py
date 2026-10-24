"""RMUC 物理主循环：接收飞控电机推力，发布 IMU、仿真位姿与机体描述。

图像与深度由 render 发布。无新鲜飞控指令时暂停物理与传感器时间。
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from functools import partial
from typing import Protocol

import time

from firefly_workspace import startup

import iceoryx2 as iox2
import numpy as np

from firefly_mujoco import (
    AIRFRAME_TOPIC,
    CONTROL_TOPIC,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_TOPIC,
    PLANT_STATE_TOPIC,
    AirframeMessage,
    ControlMessage,
    DroneEnv,
    ImuMessage,
    LogMessage,
    OdomMessage,
    PlantStateMessage,
    TraceContext,
    build_scene,
    drone_pad,
    load_scene_name,
)

from . import trace as ftrace

#: 话题名（与 Rust `firefly-pubsub` 常量一致）
TOPIC_IMU = "Firefly/Imu"
TOPIC_GT = "Firefly/GroundTruth"
#: 事件 id：「该话题有新样本」（与 Rust event::EVENT_ID_SENT_SAMPLE 一致）
EVENT_ID_SENT_SAMPLE = 0


#: IMU 服务订阅端上限（与 Rust `imu.rs: IMU_SERVICE_MAX` 同值；先创建方定上限，
#: 后续 open 不得超限——sim / firefly-okvis-x 启动顺序任意，两边必须一致）。
IMU_SERVICE_MAX = 128

#: 被控对象状态服务订阅端上限（与 Rust `plant.rs: PLANT_STATE_SERVICE_MAX` 同值；
#: sim 先起时由本进程创建服务，评测订阅不得超限）。
PLANT_STATE_SERVICE_MAX = 8
#: 机体描述服务订阅端上限（与 Rust `plant.rs: AIRFRAME_SERVICE_MAX` 同值）。
AIRFRAME_SERVICE_MAX = 4
#: 飞控指令服务订阅端上限（与 Rust `control.rs: CONTROL_SERVICE_MAX` 同值）。
CONTROL_SERVICE_MAX = 32

#: 机体描述发布周期（秒）：1Hz 持续电平（iceoryx2 无 latch，靠重发保证晚订阅必收）。
AIRFRAME_PERIOD = 1.0
#: 飞控指令陈旧阈值（墙钟秒）：锁步下 sim 时间由指令推进，陈旧只能按墙钟判——
#: 超过即物理不推进（等飞控），被控对象不自行供力。
CONTROL_STALE = 0.05


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


def _subscriber(node, topic: str, payload_cls, subscriber_max: int | None = None):
    builder = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload_cls)
        .user_header(TraceContext)
    )
    if subscriber_max is not None:
        builder = builder.subscriber_max_buffer_size(subscriber_max)
    service = builder.open_or_create()
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


def _publish_plant_state(
    pub, env: DroneEnv, t: float,
    *, out: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None,
) -> None:
    """发布被控对象状态（真值，每个物理步；无 span——高频状态不进 trace 树）。"""
    pos, vel, quat_xyzw, angvel = env.state(out=out)
    msg = PlantStateMessage()
    msg.timestamp = t
    msg.position_x, msg.position_y, msg.position_z = pos
    msg.velocity_x, msg.velocity_y, msg.velocity_z = vel
    msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w = quat_xyzw
    msg.angular_velocity_x, msg.angular_velocity_y, msg.angular_velocity_z = angvel
    _publish_traced(pub, None, "publish-plant-state", msg, t)


def _publish_airframe(pub, env: DroneEnv, t: float) -> None:
    """发布机体/执行器描述（质量/惯量/旋翼几何/电机上限；飞控参数的唯一来源）。"""
    af = env.airframe()
    msg = AirframeMessage()
    msg.timestamp = t
    msg.mass = af["mass"]
    msg.inertia_x, msg.inertia_y, msg.inertia_z = af["inertia"]
    msg.linear_drag = af["linear_drag"]
    msg.angular_drag = af["angular_drag"]
    for i, pos in enumerate(af["rotor_positions"]):
        msg.rotor_positions[i][0], msg.rotor_positions[i][1], msg.rotor_positions[i][2] = pos
    for i, spin in enumerate(af["rotor_spins"]):
        msg.rotor_spins[i] = spin
    msg.max_thrust_per_motor = af["max_thrust_per_motor"]
    msg.torque_coefficient = af["torque_coefficient"]
    _publish_traced(pub, None, "publish-airframe", msg, t)


class EnvironmentFactory(Protocol):
    """创建仿真环境；运行器依赖 DroneEnv 的物理、传感器和状态接口。"""

    def __call__(self, *, timestep: float, imu_period: float,
                 scene_factory: Callable[[], str]) -> DroneEnv: ...


def decode_config(cfg: dict, *, scene_loader: Callable[[], str] = load_scene_name
                  ) -> tuple[float, float, float, np.ndarray, str]:
    """将 TOML 覆盖项转换为已校验的周期和初始位置。"""
    rates = cfg.get("rates", {})
    frequencies = [rates.get("physics", 1000.0), rates.get("imu", 1000.0), rates.get("pose", 10.0)]
    if not np.isfinite(frequencies).all() or any(rate <= 0 for rate in frequencies):
        raise ValueError("物理、IMU 与位姿频率必须有限且为正")
    for rate in frequencies[1:]:
        ratio = frequencies[0] / rate
        if ratio < 1 or abs(ratio - round(ratio)) > 1e-9:
            raise ValueError("物理频率必须为 IMU 与位姿频率的整数倍")
    physics_period, imu_period, pose_period = (1.0 / rate for rate in frequencies)
    # 停机坪：缺配置时按场景定义（`firefly_mujoco.scene.SCENES[].pad`）——上电停在地面
    scene_name = scene_loader()
    start_pos = np.array(cfg.get("start", drone_pad(scene_name)))
    return physics_period, imu_period, pose_period, start_pos, scene_name


def main(argv: Sequence[str] | None = None, *, env_factory: EnvironmentFactory = DroneEnv,
         publish_plant_state: bool = True,
         scene_loader: Callable[[], str] = load_scene_name) -> None:
    parser = argparse.ArgumentParser(description="RMUC MuJoCo 物理进程；图像由 render 提供")
    parser.add_argument("--no-trace", action="store_true")
    parser.add_argument("--config", default="configs/sim.toml")
    args = parser.parse_args(argv)
    trace_enabled = not args.no_trace
    physics_period, imu_period, pose_period, start_pos, scene_name = startup(
        args.config, partial(decode_config, scene_loader=scene_loader))
    env = env_factory(timestep=physics_period, imu_period=imu_period,
                      scene_factory=partial(build_scene, name=scene_name))
    env.reset(start_pos, np.array([0.0, 0.0, 0.0, 1.0]))  # xyzw 单位四元数
    ftrace.init(enabled=trace_enabled)
    airframe = env.airframe()
    log(
        "MuJoCo 环境就绪：质量 {:.3f} kg，惯量 [{:.2e} {:.2e} {:.2e}] kg·m²，"
        "单电机上限 {:.2f} N，物理 {:.0f} Hz".format(
            airframe["mass"],
            airframe["inertia"][0],
            airframe["inertia"][1],
            airframe["inertia"][2],
            airframe["max_thrust_per_motor"],
            1 / physics_period,
        )
    )
    log("RMUC 双目与深度由 render 进程发布")

    # 仅输出 error 及以上级别的 iceoryx2 日志；消息消费由订阅端轮询驱动。
    iox2.set_log_level(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    imu_pub = _publisher(node, TOPIC_IMU, ImuMessage, IMU_SERVICE_MAX)
    gt_pub = _publisher(node, TOPIC_GT, OdomMessage)
    # 被控对象状态供评测；机体描述以 1Hz 发布给飞控
    plant_pub = (_publisher(node, PLANT_STATE_TOPIC, PlantStateMessage, PLANT_STATE_SERVICE_MAX)
                 if publish_plant_state else None)
    airframe_pub = _publisher(node, AIRFRAME_TOPIC, AirframeMessage, AIRFRAME_SERVICE_MAX)
    # 飞控指令订阅（闭环模式）：位置/速度反馈由飞控消费，本进程只做被控对象。
    # 服务订阅端上限须与 Rust 侧同值：先创建方定上限，否则飞控发布端声明 32 会被拒。
    control_sub = _subscriber(node, CONTROL_TOPIC, ControlMessage, CONTROL_SERVICE_MAX)
    imu_notify = _notifier(node, TOPIC_IMU)
    _init_log_ipc(node)
    log("iceoryx2 已就绪：发布 IMU/仿真位姿/状态/机体，订阅飞控指令")

    motor_thrusts = np.empty(4)
    state_out = (np.empty(3), np.empty(3), np.empty(4), np.empty(3))
    imu_out = (np.empty(3), np.empty(3))
    # 飞控指令的墙钟到达时刻（锁步下 sim 时间由指令推进，指令陈旧只能按墙钟判）
    control_rx_wall = None
    # 物理是否处于暂停（无有效飞控指令；告警一次，恢复时告知）
    physics_paused = False

    cycle = None
    next_imu = imu_period
    next_pose = 0.0
    next_airframe = 0.0
    t_start = time.perf_counter()
    try:
        while True:
            while (sample := control_sub.receive()) is not None:
                m = sample.payload().contents
                # 样本释放前复制到自有缓冲，后续物理步只读取该副本。
                motor_thrusts[0], motor_thrusts[1], motor_thrusts[2], motor_thrusts[3] = m.thrust
                control_rx_wall = time.monotonic()

            if (
                control_rx_wall is not None
                and time.monotonic() - control_rx_wall <= CONTROL_STALE
            ):
                # 闭环模式：施加飞控下发的 4 电机推力（被控对象按机体几何合成 wrench）
                if physics_paused:
                    physics_paused = False
                    log("飞控指令恢复，物理继续推进")
                env.apply_motor_thrusts(motor_thrusts)
                env.step()
            else:
                # 无有效指令（飞控未启动/停发）：物理不推进——锁步语义，对照
                # ArduPilot SITL（仿真与飞控同进程）与 PX4 lockstep（物理由飞控
                # 传感器节拍驱动）：两家都没有被控对象侧兜底。状态/传感器仍按节拍
                # 发布冻结内容，飞控一起就能续上；本循环的暂停时长不计入实时节奏
                # （t_start 后移），恢复后不追赶。
                if not physics_paused:
                    physics_paused = True
                    log(
                        "无有效飞控指令（飞控未启动或已停发），物理不推进",
                        LOG_LEVEL_ERROR,
                        env.time,
                    )
                t_start += physics_period
                time.sleep(physics_period)
            # qpos 或 qvel 非有限时重置被控对象，避免发布无效状态。
            if not np.isfinite(env.data.qpos).all() or not np.isfinite(env.data.qvel).all():
                log("物理失稳（NaN/Inf）@ t={:.2f}，重置到起点".format(env.time), LOG_LEVEL_ERROR, env.time)
                env.reset(start_pos, np.array([0.0, 0.0, 0.0, 1.0]))
            t = env.time

            # 被控对象状态供评测（每个物理步），机体描述供飞控（1Hz）
            if plant_pub is not None:
                _publish_plant_state(plant_pub, env, t, out=state_out)
            if t + 1e-12 >= next_airframe:
                _publish_airframe(airframe_pub, env, t)
                next_airframe = t + AIRFRAME_PERIOD

            # 位姿发布节拍开启 trace 周期，周期内所有发布共享同一 trace_id。
            if t + 1e-12 >= next_pose:
                if cycle is not None:
                    ftrace.end_cycle(cycle)
                cycle = ftrace.start_cycle()

            # IMU（当前周期的子 span）
            if t + 1e-12 >= next_imu:
                gyro, accel = env.imu(out=imu_out)
                msg = ImuMessage()
                msg.timestamp = t
                msg.angular_velocity_x, msg.angular_velocity_y, msg.angular_velocity_z = gyro
                msg.linear_acceleration_x, msg.linear_acceleration_y, msg.linear_acceleration_z = accel
                _publish_traced(imu_pub, cycle, "publish-imu", msg, t)
                _notify(imu_notify)
                # 推进到 t 之后的网格点（防止 t 越过后下一步重复发布，
                # 保证传感器时间戳落在各自的采样网格上）
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

            # 仿真位姿供 render 摆放传感器及评测使用。
            if t + 1e-12 >= next_pose:
                _publish_gt(gt_pub, cycle, env, t)
                next_pose, skipped = advance_grid(next_pose, t, pose_period)
                if skipped >= 1:
                    log(
                        "位姿追赶跳过 {} 样本（{:.2f}s 位姿数据丢失）".format(
                            skipped, skipped * pose_period
                        ),
                        LOG_LEVEL_WARN,
                        t,
                    )
            # 以仿真时刻为绝对截止时间，处理耗时计入物理周期预算。
            wall = time.perf_counter() - t_start
            target = t
            if wall < target:
                time.sleep(target - wall)
    except KeyboardInterrupt:
        if cycle is not None:
            ftrace.end_cycle(cycle)
        log("退出")


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
            .subscriber_max_buffer_size(256)
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
