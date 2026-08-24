# firefly

EGO 规划器 Rust 实现：MINCO 后端 + Rebound 主循环 + 集群避碰。

## crates

| crate | 职责 |
|---|---|
| `firefly-trajectory` | MINCO 轨迹（M 矩阵/求解/梯度传播） |
| `firefly-map` | 占据栅格 + 平面障碍 |
| `firefly-search` | A* + 字符串拉直 |
| `firefly-obstacle` | 动态障碍：自行车模型运动 + 轨迹预测 |
| `firefly-optimize` | L-BFGS |
| `firefly-cost` | Js/Jt/Jd/Jo/Jw/Jf/Ju 七项惩罚 |
| `firefly-planner` | Rebound 主循环 + 分布式集群接口 |
| `firefly-vio-types` | VIO 基础类型：JPL 四元数 / SO(3) / SE(3)（对照 OpenVINS `ov_core/types`） |
| `firefly-vio-core` | VIO 核心数学：传感器数据、IMU 标定、传播/更新（对照 `ov_core`） |
| `firefly-vio-init` | 静止/动态初始化器（对照 `ov_init`） |
| `firefly-vio` | MSCKF 编排：滑动窗口状态 + 视觉更新（对照 `ov_msckf`） |
| `firefly-pubsub` | iceoryx2 零拷贝发布订阅 + trace 上下文中间件 |
| `firefly-rerun` | Rerun 连接层：多进程共享 viewer + 图像/深度/位姿记录 |
| `firefly-viewer` | 规划过程可视化：地图/路径/轨迹/障碍写入 rerun |
| `firefly-error` | kind/status 错误设计 |
| `firefly-observability` | logforth + fastrace |

## apps

| app | 职责 |
|---|---|
| `planner` | 规划器进程入口 |
| `vio` | VIO 进程入口 |
| `firefly-sim` | Python 仿真环境（独立于 workspace） |

## 快速开始

```bash
cargo test
RUST_LOG=info cargo run -p firefly-planner --example demo
cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored
```

## 参考

1. Zhou X, Wang Z, Xu C, Gao F. EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors. *IEEE Robotics and Automation Letters*, 6(2): 4452–4459, 2021. [arXiv:2008.08835](https://arxiv.org/abs/2008.08835)
2. Zhou X, Zhu J, Zhou H, Xu C, Gao F. Swarm of Micro Flying Robots in the Wild. *Science Robotics*, 7(66): eabm5954, 2022.（即 EGO-Planner-v2）
3. Wang Z, Zhou X, Xu C, Gao F. Geometrically Constrained Trajectory Optimization for Multicopters. *IEEE Transactions on Robotics*, 38(5): 3259–3278, 2022. [arXiv:2103.00190](https://arxiv.org/abs/2103.00190)

- [OpenVINS](https://github.com/uci-cbgl/OpenVINS)（Geneva et al., ICRA 2020）— `firefly-vio*` 系列的对照移植蓝本
- [purecv](https://github.com/webarkit/purecv) — 纯 Rust 计算机视觉库，`firefly-vio-core` 特征跟踪依赖
