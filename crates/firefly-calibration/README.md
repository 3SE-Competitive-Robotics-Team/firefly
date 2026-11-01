# firefly-calibration

Reusable AprilGrid geometry, observation data, injected image-source corner
detection, and Kneip camera-pose initialization shared by estimation and calibration
tools. Image loading and tag detection are supplied through traits, so this crate
does not depend on the complete Basalt estimator or a dataset implementation.

`firefly-basalt` supplies the real AprilTag detector and dataset adapter, and retains
camera/IMU calibration controllers, intrinsics initialization, vignette estimation,
optimization and archive I/O. Existing Basalt type paths re-export these shared types.
Camera models and calibration parameter serialization remain in
`firefly-basalt-headers`.

The AprilGrid, pose initialization, and time-camera key translations originate
in Basalt. Each source file retains its upstream attribution. The extracted
files are licensed under BSD-3-Clause; see `LICENSE`.
