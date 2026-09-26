"""`firefly-cmd` CLI：向 `Firefly/Command` 发布一条地面站指令（外部工具 → 飞控）。

用法：`uv run firefly-cmd <verb> [altitude]`，verb ∈ {arm, disarm, takeoff, hold,
track, land}，依次为解锁 / 上锁 / 自动起飞 / 位置保持 / 跟踪外部参考 / 自动降落。
`altitude` 仅 takeoff 可带，单位 m、相对起飞点；省略则发 0.0，飞控按自身配置的
默认起飞高度执行（`<= 0` 或非有限值同义），其余指令忽略该参数。`track` 要求
planner 正在发布 `Firefly/Reference`，否则飞控拒收。

指令通道无回执：结果看飞控进程的终端日志与 rrd（`logs/*.rrd` 里的
`fc/debug/state`），拒收原因也记录在那里。

与 Rust 侧 `firefly_pubsub::command` 完全对偶：同一话题名、同一
`CommandMessage`（`#[repr(C)]`/`ctypes.Structure` 定长布局，
`type_name` 严格一致）。一次性 CLI 的连接竞态处理同 `firefly-goal`；重复投递的
副本共用同一 `sequence`（一次调用只取一次 `time.time_ns()`），飞控按序号去重，
整条指令只执行一次。
"""

from __future__ import annotations

import sys
import time

import iceoryx2 as iox2

from firefly_mujoco.messages import (
    COMMAND_KIND_ARM,
    COMMAND_KIND_DISARM,
    COMMAND_KIND_HOLD,
    COMMAND_KIND_LAND,
    COMMAND_KIND_TAKEOFF,
    COMMAND_KIND_TRACK,
    COMMAND_TOPIC,
    CommandMessage,
    TraceContext,
)

#: 动词 → （指令种类，显示名）——种类编码见 Rust `firefly_pubsub::command::KIND_*`
COMMANDS: dict[str, tuple[int, str]] = {
    "arm": (COMMAND_KIND_ARM, "ARM"),
    "disarm": (COMMAND_KIND_DISARM, "DISARM"),
    "takeoff": (COMMAND_KIND_TAKEOFF, "TAKEOFF"),
    "hold": (COMMAND_KIND_HOLD, "HOLD"),
    "track": (COMMAND_KIND_TRACK, "TRACK"),
    "land": (COMMAND_KIND_LAND, "LAND"),
}

_USAGE = (
    "用法：uv run firefly-cmd <verb> [altitude]\n"
    "verb ∈ {arm, disarm, takeoff, hold, track, land}；"
    "altitude 仅 takeoff 可带（米，相对起飞点，省略 = 飞控默认）\n"
    "如：uv run firefly-cmd takeoff 1.0"
)


def main() -> int:
    if not 2 <= len(sys.argv) <= 3:
        print(_USAGE, file=sys.stderr)
        return 2

    verb = sys.argv[1]
    entry = COMMANDS.get(verb)
    if entry is None:
        print(f"未知指令：{verb}\n{_USAGE}", file=sys.stderr)
        return 2
    kind, name = entry

    altitude = 0.0
    if len(sys.argv) == 3:
        if verb != "takeoff":
            print(f"{verb} 不接受 altitude 参数\n{_USAGE}", file=sys.stderr)
            return 2
        try:
            altitude = float(sys.argv[2])
        except ValueError:
            print(f"altitude 必须是数字：{sys.argv[2]}\n{_USAGE}", file=sys.stderr)
            return 2

    # 一次性 CLI：抑制 iceoryx2 内部清理告警（如退出时 "Unable to remove
    # node resources"——服务仍被飞控占用时的良性告警，不影响发布）。
    iox2.set_log_level(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    service = (
        node.service_builder(iox2.ServiceName.new(COMMAND_TOPIC))
        .publish_subscribe(CommandMessage)
        .user_header(TraceContext)  # 与 Rust 订阅端 user_header 一致（类型签名的一部分）
        .open_or_create()
    )
    pub = service.publisher_builder().create()

    msg = CommandMessage()
    msg.timestamp = time.time()
    msg.kind = kind
    msg.altitude = altitude
    msg.sequence = time.time_ns()

    # 一次性 CLI 的投递竞态（同 `firefly-goal`）：发布端端口与既有订阅端的连接是
    # 异步建立的，发完即退会让样本丢失。显式刷新连接后连发数次，订阅端按序号
    # 去重只执行一次；进程退出前停留让握手完成。
    pub.update_connections()
    for _ in range(8):
        pub.loan_uninit().write_payload(msg).send()
        time.sleep(0.1)
    time.sleep(0.2)

    if verb == "takeoff":
        detail = f"（高度 {altitude:.2f} m）" if altitude > 0.0 else "（高度 = 飞控默认）"
    else:
        detail = ""
    print(f"已发布指令 → {COMMAND_TOPIC}：{name}{detail}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
