# Basalt RealSense callback bridge

The optional `sdk` feature uses the system SDK discovered through `pkg-config`.
Runtime accepts SDK 2.51.x, the pinned Basalt T265 dependency. The exact
`realsense-sys = 2.44.0` dependency supplies the unchanged C ABI declarations
used here; it does not install a 2.44 runtime. No 2.56 runtime is substituted.

`Device` owns context, configuration, pipeline, profile, and callback state.
The SDK callback copies motion, stereo and pose data into owned `Payload`s.
Errors are structured `firefly-error` values, and callback panics are contained.
Its bounded raw channel has eight slots, a bridge implementation choice.
Callback sends use cancellable timeouts; stop sets an atomic closed flag before
unregistering the token and dropping the sender. No queue lock is held while
waiting for capacity. Accepted payloads drain before receiver disconnection.
Callbacks look up never-reused integer tokens in an Arc registry; SDK user data
is never dereferenced as a Rust pointer. Stop removes the token and closes its
sender before stopping/destroying the pipeline.
Late callbacks release their frames; already-dispatched callbacks retain an Arc
and cancel pending sends. Disconnection is the end marker, so no marker must be
forced into a full queue. This also covers exceptional SDK stop paths that
do not guarantee callback quiescence.
Consumers drain the receiver and then send image and IMU end-of-stream notifications.

Unsafe SDK calls are isolated in this crate. The existing workspace unsafe-code
policy remains unchanged. `cargo check --features sdk` requires installed SDK
development headers/library metadata. For static SDK builds, the installation's
pkg-config metadata must also describe its C++/USB/firmware dependencies.
The combined Basalt reader/device build requires the shared SDK: the SDK's
static `realsense-file` archive and Rust's `lz4-sys` both define XXH32 symbols.
Build SDK 2.51.1 with `BUILD_SHARED_LIBS=ON`, `BUILD_WITH_TM2=ON`, and the desired
installation prefix; expose its installed pkg-config directory with
`PKG_CONFIG_PATH` and its shared-library directory with `LD_LIBRARY_PATH`.
No duplicate-symbol suppression or SDK version substitution is used.

`tests/reference/sdk_abi.cpp` checks the consumed layouts, offsets and enum
values against actual SDK 2.51.1 headers; the Rust unit test checks the same
values against the pinned 2.44 declarations. Unit tests also inject payloads,
SDK errors and panics, and race an in-flight callback against token removal.

The `enumerate` example has been linked and executed against the original
Basalt SDK 2.51.1, returning zero devices and exiting normally. No T265 was
connected: live acquisition, calibration retrieval and hardware stop behavior
are not hardware-validated. Frame-processing arithmetic is tested separately
in `firefly-basalt/tests/rs_t265.rs` against the C++ arithmetic fixture.
