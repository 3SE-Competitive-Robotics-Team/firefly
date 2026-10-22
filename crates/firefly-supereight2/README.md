# firefly-supereight2

针孔深度的多分辨率占据地图，包含分配、融合、插值/梯度、预处理、网格和地图边界；相机投影依赖 firefly-srl-projection。

使用工作区 Rust 2024 配置与标准 src 模块树。

```bash
cargo build --locked -p firefly-supereight2
cargo test --locked -p firefly-supereight2
```

算法审计与兼容边界保留在 `apps/okvis2-x/AUDIT.md`；
许可证及版权声明汇总于仓库主 README 末尾，各包许可文件与源码署名保留。

地图文本夹具用于 Rust 数值回归。显式生成新基线：

```bash
cargo run --locked --release -p firefly-supereight2 --example generate_baseline -- --output-dir <output-directory>
```

可用 `--inputs-dir <directory>` 指定输入夹具目录；只读取场景输入，忽略其中的期望输出。
生成器使用当前 Rust 算法，不证明 C++ 等价性或算法正确性。审阅数值变化后再替换基线；
正常测试不重新生成夹具。
