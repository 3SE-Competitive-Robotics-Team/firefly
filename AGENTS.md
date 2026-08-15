# AGENTS.md

## 仓库主人的要求

- 本仓库主人有非常强的强迫症（OCD）。
- 每次修改代码之后，必须对代码整体做一次同样的审视：检查新改动与既有代码在风格、结构、命名、配置上是否保持一致，是否有冗余、杂乱、不一致之处，并主动修正。
- 不写多余的注释，不留死代码、临时文件、调试残留。
- 保持工作区间干净、整洁、统一。

## 新增 crate

```bash
cargo new crates/<name> --lib --edition 2024
```

- 本仓库已 git init，`cargo new` 不会嵌套创建 `.git`。
- 新 crate 自动被 `members = ["crates/*"]` 纳入 workspace，无需修改根 `Cargo.toml`。
- 生成的默认 `lib.rs` 模板（add 示例 + 测试）按实际用途改写或删除。

## 工程规范

- 能力按 DDD 拆分为独立 crate（firefly-*），依赖方向自上而下，领域层不依赖应用层。
- 错误：firefly-error 统一类型，按 kind（调用者能做什么）/status（重试语义）分类，
  模块边界必须添加上下文（.with_context），禁止裸传播。
- 可观测性：应用层依赖 firefly-observability（logforth + fastrace），
  关键函数标 `#[fastrace::trace]`，日志用 log crate 宏，禁止 println!。
- 依赖版本统一在根 `[workspace.dependencies]` 管理。

## 验证与演示

- 全量测试：`cargo test`（65 项，含梯度数值验证、Rebound 逃逸、集群避碰）
- 随机地图 benchmark：`cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored`
- 端到端演示：`RUST_LOG=info cargo run -p firefly-planner --example demo`
