# firefly

EGO 规划器 Rust 实现：MINCO 后端 + Rebound 主循环 + 集群避碰。

## crates

| crate | 职责 |
|---|---|
| `firefly-trajectory` | MINCO 轨迹（M 矩阵/求解/梯度传播） |
| `firefly-map` | 占据栅格 + 平面障碍 |
| `firefly-search` | A* + 字符串拉直 |
| `firefly-optimize` | L-BFGS |
| `firefly-cost` | Js/Jt/Jd/Jo/Jw/Jf/Ju 七项惩罚 |
| `firefly-planner` | Rebound 主循环 + 分布式集群接口 |
| `firefly-error` | kind/status 错误设计 |
| `firefly-observability` | logforth + fastrace |

## 快速开始

```bash
cargo test
RUST_LOG=info cargo run -p firefly-planner --example demo
cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored
```

## 论文依据

- EGO-Planner (RAL 2021, arXiv:2008.08835)
- Swarm of micro flying robots in the wild (Sci. Robot. 2022)
- MINCO (arXiv:2103.00190)
