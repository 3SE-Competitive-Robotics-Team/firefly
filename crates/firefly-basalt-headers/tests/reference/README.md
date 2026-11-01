# C++ reference fixtures

`calibration.cpp` evaluates the official calibration defaults, discrete-time
noise conversion, accelerometer/gyroscope bias-and-scale extraction and forward
calibration for both f32 and f64. It also checks inverse calibration in f64 and
a nontrivial f64-to-f32 SE(3)/bias cast. The original inverse methods use
`Eigen::Matrix3d::Identity()` internally; this fixture deliberately does not
instantiate those methods for f32, so it makes no f32 inverse-equivalence claim.
The random test helper accepts an explicit uniform sample stream in Rust; it does
not reproduce the C library's global random generator.

`imu_types.cpp` and `preintegration.cpp` use official headers at the revision
below and Sophus `d0b7315a0d90fc6143defa54596a3a95d9fa10ec`. They record both
floating-point instantiations, state increments, rotation Jacobians, accumulated
covariance, its square-root information factor, prediction, residuals and all
residual Jacobians. Their JSON outputs preserve C++ decimal literals directly.
Set `SOPHUS` to the Sophus source root and `FMT` to the fmt installation prefix:

```sh
c++ -std=c++17 -O2 -I"$HEADERS/include" -I"$EIGEN" -I"$JSON" -I"$SOPHUS" -I"$FMT/include" imu_types.cpp -L"$FMT/lib" -Wl,-rpath,"$FMT/lib" -lfmt -o "$HEADERS/build/imu_types_reference"
"$HEADERS/build/imu_types_reference"
c++ -std=c++17 -O2 -I"$HEADERS/include" -I"$EIGEN" -I"$JSON" -I"$SOPHUS" -I"$FMT/include" preintegration.cpp -L"$FMT/lib" -Wl,-rpath,"$FMT/lib" -lfmt -o "$HEADERS/build/preintegration_reference"
"$HEADERS/build/preintegration_reference"
```

`image.json` is output of `image.cpp` compiled against official basalt-headers
`aa441ba3e51050c47ba1902537792a2e4db7e43d`, pinned by official Basalt
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`. GCC 15.2.0, C++17, `-O2`, Eigen
3.4.0 (Debian 3.4.0-5), and nlohmann JSON were used.

With `HEADERS`, `EIGEN`, `JSON` set to source/include roots and `BASALT` to the
upstream checkout, compile and execute from this directory:

```sh
c++ -std=c++17 -O2 -I"$HEADERS/include" -I"$EIGEN" -I"$JSON" image.cpp -o "$BASALT/build/image_reference"
"$BASALT/build/image_reference"
```

The source specifies all pixels, padding, subpixel locations and border cases.
Upstream `interpGradCubicSplines<float>` cannot instantiate because its helper
takes `double*` outputs. That field is explicitly marked
`unsupported_upstream_float_instantiation`; it is not a numeric oracle result.
All other recorded fields were executed against the unmodified upstream header.

`image_operations.cpp` uses the same compile command with the corresponding
source/output filename. `image_operations.json` covers mixed double pixels and
float coordinates, NaN placement, and signed-zero reduction order. Pixel inputs
also include their exact 64-bit representation so decimal JSON parser rounding
cannot alter cancellation-sensitive comparisons. Preserve integer literals when
capturing the oracle output; they exceed the exact integer range of JavaScript.

Camera probes include the same official headers and use Sophus
`d0b7315a0d90fc6143defa54596a3a95d9fa10ec` plus fmt. With `SOPHUS` set to its
source root and `FMT` to its installation prefix, the standard command is:

```sh
c++ -std=c++17 -O2 -I"$HEADERS/include" -I"$EIGEN" -I"$JSON" -I"$SOPHUS" -I"$FMT/include" spherical_cameras.cpp -L"$FMT/lib" -Wl,-rpath,"$FMT/lib" -lfmt -o "$HEADERS/build/spherical_cameras_reference"
"$HEADERS/build/spherical_cameras_reference" ds f32
```

Select `ds`, `ucm`, `kb4`, `fov`, or `bal` and `f32`/`f64`. The model's native
test intrinsics and synthetic parameter boundaries generate projection,
unprojection, every implemented Jacobian, initialization and increments. KB4
also records zero, one and three Newton steps and the returned derivative.
BAL's unprojection Jacobians are unsupported upstream and are never requested.
Nonfinite values are explicitly encoded as strings, preserving their meaning.

`pinhole_camera.cpp` uses the same command with its corresponding source/output
name and takes no arguments. `generic_camera.cpp` takes `f32` or `f64`; its
fixtures cover all six actual variant alternatives, unknown-name fallback,
single-point dispatch, world-to-camera transforms, batch projection/unprojection
and the original polar/azimuthal angle convention. FOV and BAL are intentionally
not alternatives in the original GenericCamera variant.

`so3_normalization.cpp` uses the same Sophus/fmt compile flags. Its no-argument
output records 1, 100, 1000 and 10000 products and pose increments for both
scalar types, plus inverse and same-/cross-scalar casts. This checks Sophus's
normalizing group operations independently from Rust's quaternion implementation.

`image_pyr.json` is the output of `image_pyr.cpp` against the same revision and
dependencies. It covers the entire mipmap, including unused zero-filled regions,
for 8-bit and 16-bit pixels, odd dimensions, zero additional levels and saturated
input. The Rust comparison uses exact integer equality.

```sh
c++ -std=c++17 -O2 -I"$HEADERS/include" -I"$EIGEN" -I"$JSON" image_pyr.cpp -o "$HEADERS/build/image_pyr_reference"
"$HEADERS/build/image_pyr_reference"
```

`stereographic_param.json` uses `stereographic_param.cpp` and the same compiler
and include paths. It records projection, unprojection and both Jacobians in
row-major order for float and double, including directions behind the camera.

`extended_radtan.cpp` executes the official extended-unified and radial-tangential
models. Compile with the same include roots and `-DFMT_HEADER_ONLY`, then pass
`eucm f32`, `eucm f64`, `radtan8 f32`, or `radtan8 f64`. The corresponding
`extended_*.json` and `radtan8_*.json` fixtures include forward/inverse projections,
all supported Jacobians, domain rejection, nonfinite arithmetic, initialization,
parameter updates, distortion derivatives, and the radtan radius estimator.
Unprojection derivatives are intentionally not requested for radtan: upstream
asserts that this operation is unimplemented. Its rejection is tested in Rust.

`rd_spline.cpp` executes the original `RdSpline`/`RdBezier` and coefficient
generators, orders 4 and 5, float and double. `rd_spline.json` was reproduced
byte-for-byte using the official vcpkg Eigen 5.0.1 headers (89,092 bytes).
The ordinary value/derivative comparisons retain 2e-14 (double) and 2e-6
(float). Converted Bezier evaluations additionally propagate the measured
control-point input difference; a separate same-input comparison retains the
ordinary threshold. Integral Jacobians use componentwise forward bounds for
three length-N matrix products, quadratic-coefficient arithmetic and interval
scaling, with upward-rounded positive path magnitudes. An independent central
difference test retains a fixed 1e-8 threshold.

The original cancelled-Jacobian discrepancy remains informative: order 5,
segment 2, second derivative, knot 2/y has C++ output
1.33226762955018785e-12, whereas an 80-digit evaluation using the exact exported
binary64 control points and rational coefficients gives
1.33226762955018785e-13. Its absolute path sum is 885300. The initial Rust
split product gave 8.881784197001252e-13. Consequently, comparing this component
to an absolute 2e-14 threshold incorrectly rejects the C++ reference's own
rounding error; the test now derives its bound from operation counts and input
propagation rather than fitting SIMD packet order. No existing repository test
threshold was changed.

```sh
c++ -std=c++17 -O2 -I"$HEADERS/include" -I"$EIGEN" -I"$JSON" stereographic_param.cpp -o "$HEADERS/build/stereographic_param_reference"
"$HEADERS/build/stereographic_param_reference"
```

`euroc_ds_calib.json` is the unchanged `data/euroc_ds_calib.json` from original
Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` (BSD-3-Clause).
The calibration serialization test loads this original archive and checks every
serialized calibration field against it, including raw quaternion coefficients,
camera parameters, inertial calibration, and vignette spline knots.

`so3_spline.cpp` and `se3_spline.cpp` use the official headers pin above,
Eigen 5.0.1 from Basalt's release-preset vcpkg installation, and Sophus
`d0b7315a0d90fc6143defa54596a3a95d9fa10ec`. Compile with C++17,
`-O1 -DFMT_HEADER_ONLY` and those include roots plus fmt and nlohmann JSON;
each binary emits its corresponding JSON without arguments. Orders 4 and 5
cover knot boundaries and interior samples, pose, velocity, acceleration, jerk,
and analytic knot/calibration Jacobians. The Ceres helper additionally covers
coupled SE(3) and SO(3) Lie-group evaluations.

SO(3) value/pose-Jacobian fixtures instantiate both float and double. Its
velocity/acceleration Jacobian overloads hardcode `Sophus::SO3d`, and several
SE(3) overloads also hardcode double, so their C++ fixtures instantiate double
only. The Rust scalar-generic counterparts are not claimed to have an upstream
float oracle for those non-instantiable overloads. The Ceres helper accepts typed
knot arrays and a Lie-group trait instead of raw parameter pointers.

`mocap_calibration.cpp` uses the same official headers, Eigen 5.0.1, and
Sophus pins with Cereal from the release-preset vcpkg installation. Compile
with C++17, `-O0 -DFMT_HEADER_ONLY` and these include roots. Its JSON archive
checks both rigid transforms and signed 64-bit time offsets without converting
the offsets through floating point.

`rotation_matrix.cpp` directly executes the pinned Sophus/Eigen conversion for
float and double rotations, including tiny angles and larger rotations. Compile
with C++17, `-O0 -DFMT_HEADER_ONLY` and the Basalt headers, Eigen, Sophus, fmt and
JSON roots above.
The test reuses the emitted raw quaternion coefficients and checks every matrix
element exactly. This isolates conversion arithmetic from quaternion construction;
Eigen's `1 - (2*y*y + 2*z*z)` diagonal differs in floating point from nalgebra's
algebraically equivalent `w*w + x*x - y*y - z*z` expression.
The same probe exercises both decoupled SE(3) right Jacobians with float and
double tangents; these use the existing scalar-specific fixed tolerances.
# Eigen and Sophus archives

`eigen_io.cpp` includes the unmodified official `serialization/eigen_io.h` and
records native binary bytes, JSON values and actual JSON reload outcomes.
Compile with GCC `-std=c++17 -O2`, the official headers include root, release
vcpkg include/eigen3 roots and vcpkg `-lfmt`; its sole argument is output JSON.
Tests compare bytes and JSON exactly for fixed f32/f64/i32 matrices, both
partially dynamic shapes, fully dynamic/empty matrices, SE3, scaled-quaternion
Sim3, and raw nonunit/zero quaternion coefficients. Every binary truncation
must fail. Native binary byte fixtures target little-endian hosts.

The original fully dynamic matrix JSON format loses both dimensions: the two
Cereal size tags produce only a flat array, and reload reads the array length
for both dimensions. The 2x3 original probe therefore fails its own JSON
reload; Rust deliberately preserves that failure rather than inventing a
different interoperable schema. Partially dynamic dimensions retain the
source integer-division behavior for nonmultiples. Archive load also writes
Sophus coefficients directly; no quaternion normalization or similarity-scale
validation is added. Invalid native bool bytes and overflowing dimensions
return I/O errors instead of reproducing C++ invalid object representation or
allocation overflow behavior.
