# firefly-calibration

Reusable AprilGrid geometry, observation data, injected image-source corner
detection, and Kneip camera-pose initialization shared by estimation and calibration
tools. Image loading and tag detection are supplied through traits, so this crate
does not depend on the complete Basalt estimator or a dataset implementation.

This crate also owns reusable camera models, image buffers, calibration parameters,
and the Euclidean spline primitives used by calibration. Basalt-specific IMU
preintegration, image pyramids, pose splines, and Lie-group helpers live in
`firefly-basalt`, alongside the estimator that consumes them. The split follows
module responsibilities directly; there is no compatibility facade crate.

The AprilGrid, pose initialization, time-camera keys, camera models, image buffers,
and calibration parameters originate in Basalt. Each source file retains its
upstream attribution. See `LICENSE` and `LICENSE-BASALT-PRIMITIVES` for the
applicable notices.
