# firefly-srl-projection

f32 无畸变针孔投影、反投影、雅可比及掩码。独立于 Supereight2，供深度相机使用。

使用工作区 Rust 2024 配置与标准 src 模块树。

```bash
cargo build --locked -p firefly-srl-projection
cargo test --locked -p firefly-srl-projection
```

算法审计与兼容边界保留在 `apps/firefly-okvis-x/README.md`；
许可证及版权声明汇总于仓库主 README 末尾，各包许可文件与源码署名保留。
