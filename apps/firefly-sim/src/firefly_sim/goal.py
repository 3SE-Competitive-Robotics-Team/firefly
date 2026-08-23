"""`firefly-goal` CLI：向 `Firefly/Goal` 发布一个飞行目标点。

用法：`uv run firefly-goal X Y Z`——规划进程订阅后经
`PlannerManager::set_goal` 动态重目标（重算全局路径 + 重新规划），
无人机即飞往该点。坐标是地图系米制，需落在加载的地图范围内
（gate.ffmap 为 0~28 × 0~8 × 0~3.2 m）。

与 Rust 侧 `firefly_pubsub::goal` 完全对偶：同一话题名、同一
`GoalMessage`（`#[repr(C)]`/`ctypes.Structure` 定长布局，
`type_name` 严格一致）。
"""

from __future__ import annotations

import sys
import time

import iceoryx2 as iox2

from firefly_mujoco.messages import GoalMessage, TraceContext

GOAL_TOPIC = "Firefly/Goal"


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "用法：uv run firefly-goal X Y Z\n"
            "发布飞行目标点（地图系，米）。如：uv run firefly-goal 20.0 4.0 1.5",
            file=sys.stderr,
        )
        return 2

    try:
        x, y, z = (float(v) for v in sys.argv[1:4])
    except ValueError:
        print("坐标必须是数字", file=sys.stderr)
        return 2

    # 一次性 CLI：抑制 iceoryx2 内部清理告警（如退出时 "Unable to remove
    # node resources"——服务仍被 planner 占用时的良性告警，不影响发布）。
    iox2.set_log_level(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    service = (
        node.service_builder(iox2.ServiceName.new(GOAL_TOPIC))
        .publish_subscribe(GoalMessage)
        .user_header(TraceContext)  # 与 Rust 订阅端 user_header 一致（类型签名的一部分）
        .open_or_create()
    )
    pub = service.publisher_builder().create()

    msg = GoalMessage()
    msg.timestamp = time.time()
    msg.position_x, msg.position_y, msg.position_z = x, y, z

    # 一次性 CLI 的投递竞态：发布端端口与既有订阅端的连接是异步建立的，
    # 发完即退会让样本丢失（实测单发 0/5 送达）。显式刷新连接后连发数次，
    # 订阅端 10Hz 轮询总能收到最新一条；进程退出前停留让握手完成。
    pub.update_connections()
    for _ in range(8):
        pub.loan_uninit().write_payload(msg).send()
        time.sleep(0.1)
    time.sleep(0.2)

    print(f"已发布目标 → Firefly/Goal：({x:.2f}, {y:.2f}, {z:.2f}) m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
