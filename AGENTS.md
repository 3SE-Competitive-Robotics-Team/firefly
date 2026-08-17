# AGENTS.md

## 规范

- 强迫症：每次改动后审视全仓库一致性，不留冗余、死代码、临时文件。
- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），依赖自上而下，领域层不依赖应用层。
- 错误用 firefly-error：kind/status 分类，模块边界 .with_context。
- 可观测性：关键函数 `#[fastrace::trace]`，日志用 log 宏，禁 println!。
- 依赖统一在根 `[workspace.dependencies]`。

## VIO（firefly-vio*）约定

- **不接真实驱动**：`apps/vio` 保持合成数据最小闭环（GT 初始化 + 合成 IMU +
  高频传播 + odom 发布），不引入 realsense/串口驱动。
- **不做配置文件系统**：相机内参/外参、IMU 噪声、时间偏移等标定一律硬编码
  在代码里（`VioManagerOptions::default()` / `InitOptions::default()` 即为
  事实配置源），不引入 YAML/JSON/serde 解析。
- 新增标定类数值参数时，直接改对应 `*Options` 的默认值并在 doc 注释标注
  单位与来源。

## 性能

- 结论只认 trace 实测（ConsoleReporter span 树），不靠读代码推断。
- 主循环建 root span（`Span::root` + `set_local_parent`），结束 `flush()`。
- 热路径禁 `#[logcall]`（无条件格式化大对象，开销爆炸），只留 trace。
- 输出先写清晰，不写解析脚本。

## 验证

- `cargo test`
- `cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored`
- `RUST_LOG=info cargo run -p firefly-planner --example demo`
