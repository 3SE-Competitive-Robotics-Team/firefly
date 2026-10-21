# firefly-opengv

中心与非中心多相机几何、绝对/相对位姿、三角化及采样共识。RustNumericalBackend 提供本地数值求解。MINPACK LM 文件保留 MPL-2.0 声明。

使用工作区 Rust 2024 配置与标准 src 模块树。

```bash
cargo build --locked -p firefly-opengv
cargo test --locked -p firefly-opengv
```

算法审计与兼容边界保留在 `apps/firefly-okvis-x/README.md`；
许可证及版权声明汇总于仓库主 README 末尾，各包许可文件与源码署名保留。
