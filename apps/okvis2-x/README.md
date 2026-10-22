# firefly-okvis-x

OKVIS2-X 的 Rust 实现，包含前端、图优化、在线回环及针孔深度占据建图。
Cargo 包和二进制名均为 `firefly-okvis-x`，使用工作区 Rust 2024 配置。
默认入口为同步离线数据集处理，不创建 IPC 端点。

```bash
cargo build --locked -p firefly-okvis-x
cargo run --locked --release -p firefly-okvis-x -- --help
cargo run --locked --release -p firefly-okvis-x -- --config configs/okvis2-x.toml --dataset <dataset-directory> --output <output-directory>
```

TOML 配置必须提供实际相机与 IMU 标定。默认资源目录为本包的 `resources/`，
`--vocabulary <directory>` 可覆盖；`small_voc.yml.gz` 是 BRISK 词袋检索的视觉词典，
不是神经网络。深度建图使用外部传入的深度图；默认离线工厂不装配网络推理。
需要 CNN、原生 UI、双目校正、LiDAR 地图积分或未提供硬件 SDK 的配置显式失败。

历史源码审计、兼容差异和已知边界统一保留在 `apps/firefly-okvis-x/README.md`。
许可证和版权声明汇总于仓库主 README 末尾；本包 LICENSE 继续保留。
