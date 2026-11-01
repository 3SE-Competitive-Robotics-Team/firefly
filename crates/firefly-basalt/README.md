# firefly-basalt

Reference: <https://gitlab.com/VladyslavUsenko/basalt>, fixed at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
Primitive dependency: `firefly-basalt-headers`, pinned by the upstream vcpkg port
at `aa441ba3e51050c47ba1902537792a2e4db7e43d`.

Each translated module identifies its source file and retains its file-level license.
The [license summary](LICENSE) identifies separately licensed algorithms, including
files containing both Basalt and Eigen-derived code. Reference C++ programs are used
for independent comparisons, not as runtime substitutes for the Rust implementation.

The presence of this crate does not establish a complete VIO implementation.
Development and validation conventions: [development guide](../../docs/development.md).

The optional `realsense` feature enables the real T265 SDK callback adapter.
`RsT265Processor` handles IMU interpolation, paired images, poses and calibration
conversion without an SDK dependency. `RsT265Device` owns the real bridge and
processing worker, exposes calibration and output queues, and joins on stop.
Stream outputs use bounded channels. Consumers drain through EOS during normal
stop; detaching output queues cancels pending sends before recording sessions
close. The raw SDK bridge is bounded and cancellation-aware. Device destruction
detaches outputs before stopping so a full output cannot block destruction.

Hardware reset and waiting for device re-enumeration are explicit application
operations, unlike the upstream constructor's automatic reset/wait sequence.
Manual exposure is configured on the first device sensor before startup.
These lifecycle adaptations are not a claim of hardware equivalence. The SDK
2.51.1 bridge has been compiled, linked and enumerated with zero connected T265s;
only pure processing and injected payloads have automated numerical tests.
