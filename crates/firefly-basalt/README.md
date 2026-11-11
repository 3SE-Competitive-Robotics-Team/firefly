# firefly-basalt

Rust translation of the original Basalt estimator, IMU, optical-flow, mapping,
serialization, and calibration-control modules. Shared camera and calibration
primitives live in `firefly-calibration`.

The source baseline is Basalt commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`. Numerical fixtures verify individual
contracts; [dataset evaluation](../../apps/rootslam/README.md) measures application behavior.

## References

- [Original Basalt source](https://gitlab.com/VladyslavUsenko/basalt)
- [Workspace architecture](../../docs/architecture.md#basalt-与-rootslam-独立应用)
- [License index](../../docs/licenses/README.md)
