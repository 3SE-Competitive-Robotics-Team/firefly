# firefly-dbow2

DBoW2 词典、词袋评分与在线倒排检索；描述子运算通过 DescriptorOps 注入。支持 OpenCV YAML 词典及 gzip 压缩。

使用工作区 Rust 2024 配置与标准 src 模块树。

```bash
cargo build --locked -p firefly-dbow2
cargo test --locked -p firefly-dbow2
```

算法审计与兼容边界保留在 `apps/okvis2-x/AUDIT.md`；
许可证及版权声明汇总于仓库主 README 末尾，各包许可文件与源码署名保留。
