# firefly-opengv

`firefly-opengv` 为中心和非中心相机模型提供绝对位姿估计、相对位姿估计、三角化和采样共识算法。`RustNumericalBackend` 实现数值求解。

```bash
cargo build --locked -p firefly-opengv
cargo test --locked -p firefly-opengv
```

MINPACK LM 的 MPL-2.0 声明和其他许可证见[许可证索引](../../docs/licenses/README.md)。
