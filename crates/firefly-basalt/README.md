# firefly-basalt

Reference: <https://gitlab.com/VladyslavUsenko/basalt>, fixed at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
Primitive sources from the upstream vcpkg port at
`aa441ba3e51050c47ba1902537792a2e4db7e43d` are organized by responsibility:
shared camera, calibration, image-buffer, scalar, and Euclidean-spline modules
live in `firefly-calibration`; Basalt-specific IMU, image-pyramid, pose-spline,
serialization, and Lie-group modules live here with the estimator. There is no
separate headers crate or re-export facade.

Each translated module identifies its source file and retains its file-level license.
The [license summary](LICENSE) identifies separately licensed algorithms, including
files containing both Basalt and Eigen-derived code. Reference C++ programs are used
for independent comparisons, not as runtime substitutes for the Rust implementation.

This crate includes local VO/VIO estimators and mapper implementations; their
presence does not establish end-to-end equivalence with the upstream applications
or RootSLAM's reported performance.
Development and validation conventions: [development guide](../../docs/development.md).
