# firefly-brisk

BRISK 检测、描述子与匹配；AGAST 角点检测位于同一 crate 的 agast 模块。默认 pattern 内嵌 Rust，不依赖外部图片或 brisk.ptn。

使用工作区 Rust 2024 配置与标准 src 模块树。

```bash
cargo build --locked -p firefly-brisk
cargo test --locked -p firefly-brisk
```

算法审计与兼容边界保留在 `apps/okvis2-x/AUDIT.md`；
许可证文本与第三方声明见[许可证索引](../../docs/licenses/README.md)。
