# firefly-brisk

BRISK 检测、描述子与匹配；AGAST 角点检测位于同一 crate 的 agast 模块。默认 pattern 内嵌 Rust，不依赖外部图片或 brisk.ptn。

使用工作区 Rust 2024 配置与标准 src 模块树。

```bash
cargo build --locked -p firefly-brisk
cargo test --locked -p firefly-brisk
```

算法审计与兼容边界保留在 `apps/firefly-okvis-x/README.md`；
许可证及版权声明汇总于仓库主 README 末尾，各包许可文件与源码署名保留。
