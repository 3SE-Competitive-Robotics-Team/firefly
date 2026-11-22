# firefly-basalt

This crate implements the Basalt estimator, IMU, optical flow, mapping, and
serialization modules in Rust. It uses the camera models and calibration data
from `firefly-calibration`.

The source baseline is Basalt commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`. Numerical tests check individual algorithms against reference results. To measure
trajectory accuracy, run the [RootSLAM dataset evaluation](../../apps/firefly/rootslam/README.md).

## References

- [Original Basalt source](https://gitlab.com/VladyslavUsenko/basalt)
- [Workspace architecture](../../docs/architecture.md)
- [License index](../../docs/licenses/README.md)
