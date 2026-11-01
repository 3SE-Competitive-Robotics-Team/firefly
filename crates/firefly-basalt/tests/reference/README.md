# C++ reference fixtures

`cam_calib_frozen.cpp` replays the official controller from a real dataset and
its existing Cereal corner/initial-pose caches. Arguments are bag, AprilGrid,
source-cache directory, and output directory. It exports initial calibration
and vignette projections before optimizing. The `cam_calib_frozen` Rust example
accepts the same inputs plus an optional initial calibration archive, which
also freezes the random-RANSAC-dependent intrinsic initialization.

`apriltag_image.cpp` exports an actual bag image, raw ETH tag corners, and the
official OpenCV-refined corners (arguments: bag, timestamp, camera, output JSON).
The `apriltag_image` Rust example separately refines those fixed C++ corners
and its own raw detections. `apriltag_image_subpixel.json` preserves the sampled
24×24 patch at full-image coordinates from official TUM-VI calib-cam3, camera 1,
timestamp 1520526024123132022, tag 12 corner 0. Its expected refinement comes
from pinned OpenCV 4.12; the test requires identical output bits. Outside-patch
pixels are unused by this corner's fixed 3-pixel window.
`apriltag_image_quality.json` similarly preserves tag 19 corner 3 at timestamp
1520526040373640022. It retains both official native and no-contraction initial
points and their OpenCV outputs, exercising opposite sides of the unchanged
radius-squared quality boundary with bit-exact refinement.

These real-data probes link the official release `cam_calib.cpp` object and/or
`libbasalt.so`, reusing the compile/link commands from `ninja -t commands
basalt_calibrate`. The diagnostic no-FMA variant additionally compiles all
`thirdparty/apriltag/ethz_apriltag2/src/*.cc` with `-ffp-contract=off` into the
probe executable. It isolates compiler contraction from algorithm differences;
it does not replace the default native-build reference or relax its tests.

`imu_block.cpp` uses the unmodified official inertial linearization class with
Eigen 5.0.1 from the pinned Basalt vcpkg installation. It records both scalar
types, fixed-linearization residuals, whitened rows, normal equations, column
scaling and predicted model decrease. Compile with the same includes and link
flags as `imu_types.cpp`, substituting the source and output basename.

The normal-equation comparison propagates the measured Jacobian/residual input
differences and the IEEE dot-product `gamma_(2n)` forward error bound. The fixed
Jacobian/residual tolerances are checked separately. A relative error on a
cancelled Gram-matrix entry is not a valid rounding criterion: for the f32
fixture's H[5,6], the sum of absolute products is approximately 5.18e9, while
the C++ entry is 7928 and the exact dot product of its exported binary32
Jacobian values is 7746.195281982421875. Test bounds come from operation counts
and machine epsilon, not from these observed differences.

`landmark_database.cpp` links the unmodified upstream
`src/vi_estimator/landmark_database.cpp`. Its snapshots sort only unordered
container output for comparison, while retaining duplicate host-query results.
`imu_types.cpp` uses the upstream linearization-state classes directly. These
cover observation updates/removal, geometry backup/restore and frozen-state
increment accumulation for both scalar types.

```sh
c++ -std=c++17 -O2 -I"$BASALT/include" -I"$HEADERS/include" -I"$EIGEN" -I"$SOPHUS" -I"$JSON" -I"$CEREAL/include" -I"$FMT/include" landmark_database.cpp "$BASALT/src/vi_estimator/landmark_database.cpp" -L"$FMT/lib" -Wl,-rpath,"$FMT/lib" -lfmt -ltbb -o "$HEADERS/build/landmark_database_reference"
"$HEADERS/build/landmark_database_reference"
c++ -std=c++17 -O2 -I"$BASALT/include" -I"$HEADERS/include" -I"$EIGEN" -I"$SOPHUS" -I"$JSON" -I"$CEREAL/include" -I"$FMT/include" imu_types.cpp -L"$FMT/lib" -Wl,-rpath,"$FMT/lib" -lfmt -ltbb -o "$BASALT/build/imu_types_reference"
"$BASALT/build/imu_types_reference"
```

These fixtures execute unmodified official Basalt headers at commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, with its pinned basalt-headers
`aa441ba3e51050c47ba1902537792a2e4db7e43d`.

The recorded compiler is GCC 15.2.0, C++17, `-O2`, with Eigen 3.4.0
(Debian package 3.4.0-5), Sophus commit
`d0b7315a0d90fc6143defa54596a3a95d9fa10ec`, nlohmann JSON, fmt and TBB.
These dependency versions describe the oracle environment; they are not a claim
that the full upstream vcpkg dependency graph was built.

Given paths `BASALT`, `HEADERS`, `EIGEN`, `SOPHUS`, `JSON`, `FMT` to the respective
source/include roots (FMT is its installation prefix), compile from this directory:

```sh
c++ -std=c++17 -O2 -I"$BASALT/include" -I"$HEADERS/include" -I"$EIGEN" -I"$SOPHUS" -I"$JSON" -I"$FMT/include" optical_flow.cpp -L"$FMT/lib" -Wl,-rpath,"$FMT/lib" -lfmt -o "$BASALT/build/optical_flow_reference"
c++ -std=c++17 -O2 -I"$BASALT/include" -I"$HEADERS/include" -I"$EIGEN" -I"$SOPHUS" -I"$JSON" -I"$FMT/include" utils.cpp -L"$FMT/lib" -Wl,-rpath,"$FMT/lib" -lfmt -ltbb -o "$BASALT/build/utils_reference"
"$BASALT/build/optical_flow_reference" f32 normal
"$BASALT/build/utils_reference"
```

The optical-flow arguments are scalar `f32`/`f64` and scenario
`normal`/`edge`/`outside`/`black`/`constant`/`plane`. Each JSON file contains
all four sampling patterns. Input construction is fully specified by the C++
source. Matrices use explicit row-major values; nonfinite scalars are strings
`NaN`, `+Inf`, or `-Inf`. The plane and constant cases expose singular or nearly
singular Hessian behavior; they must not be treated as ordinary well-conditioned
inverse comparisons.

`utils.json` checks union-find merges/compression/reinitialization, descriptor
hashes, normalized bags of words, database timestamp filtering and conflicting
feature-track rejection. Unordered outputs are sorted for stable fixtures.
The single image-pair track input avoids hash iteration changing representative IDs.

`vio_config.cpp` includes the official configuration implementation without changes
and produces `vio_config.json` from its constructor and cereal serializer. It uses
cereal v1.3.2 (`ebef1e929807629befafbb2918ea1a08c7194554`) and magic_enum v0.9.7
(`e046b69a3736d314fad813e159b1c192eaef92cd`). Compile with `-I"$BASALT"`
in addition to the Basalt and basalt-headers include directories and the two
dependency include directories. Passing an argument reads an archive from stdin
before writing it; missing fields report a cereal exception. The Rust test compares
all 48 fields and their JSON representation exactly, including cereal's promotion
of the three float fields to double. Formatting whitespace is not part of the test.

`keypoints.cpp` and `keypoints_ransac.cpp` include the complete official
`src/utils/keypoints.cpp` without changing its functions. They additionally use
system OpenCV 4.10.0 and OpenGV revision
`91f4b19c73450833a40e463ad3648aae80b3a7f3`, the exact revision in the official
Basalt vcpkg port. Both are compiled with C++17, `-O2 -march=native`, the include
paths above plus the Basalt root, OpenGV include directory and OpenCV include
directory, and linked against `libopengv.a`, `opencv_features2d`, `opencv_imgproc`,
`opencv_core` and fmt. OpenGV uses its upstream CMake build, C++17 and
`-include cassert`; its original source is not modified.

The keypoints argument is `normal`, `checker`, `black`, `occupied` or `pitched`.
The fixture compares corner coordinates and order, intensity-centroid angles,
all 256 descriptor bits and mutual-match pairs (sorted to remove unordered-map
iteration order). The pitched case deliberately preserves the source's linear
`w*h` pixel copy in mapping corner detection. The RANSAC fixture uses 48 exact
correspondences, the original Stewenius model, 100 maximum iterations, probability
0.99, threshold 1e-8 and nonlinear refinement. Its random sampling is enabled as
upstream; tests compare the resulting pose to 1e-6 and the inlier set exactly.
This fixture does not establish equivalence for all outlier or degenerate scenes.

`frame_to_frame.cpp` exercises the complete official `processFrame` path for all
three tracker modes. Its arguments are camera count and mode
(`frame_to_frame`, `patch`, `multiscale_frame_to_frame`); the last mode requires
at least two cameras upstream. It uses the same native OpenGV/OpenCV build as
the keypoint fixtures, plus cereal, magic_enum and TBB. Three consecutive
synthetic-image inputs and an output skip of two cover temporal tracking,
feature replenishment, stereo tracking and epipolar filtering. Tests compare
all feature IDs and pyramid-level metadata exactly, and every affine-transform
coefficient within 1e-4. The Rust tests execute the actual processing thread,
including input capacity 10, missing-image early return and both end-of-stream
sentinels. A separate lifecycle test covers all twelve mode/pattern combinations.

An isolated rebuild of `optical_flow.cpp` against the official vcpkg-installed
Eigen 5.0.1 produced the same numerical values and validity flags in all twelve
optical-flow JSON files. This check retained the earlier Sophus dependency and
does not claim a completed build of the entire locked upstream dependency graph.

`common_types.cpp` uses the same cereal version and the official `common_types.h`,
`eigen_io.h`, and `hash.h` to produce `common_types.json`. Build it with the
Basalt, basalt-headers, cereal, Eigen and Sophus include roots and
`-DFMT_HEADER_ONLY`. It checks signed timestamps, native image/pair hashes,
keypoints, all-zero/high-bit/full descriptors, and a nonidentity pose with matches.
The `binary` field is the exact hexadecimal native cereal archive; it is specific
to this little-endian, 64-bit Linux oracle. The JSON representation is also tested.

`marg_helper.cpp` links the unchanged official `src/vi_estimator/marg_helper.cpp`.
Its `f32` and `f64` outputs cover all three marginalization methods, full-rank,
rank-deficient, prior-augmented, zero and near-threshold systems; COD rank and
pseudoinverse; LDLT factors and multiple-right-hand-side solves; block scaling,
diagonal insertion and moved-from accumulator semantics. These fixtures use
Eigen **5.0.1** from the official release preset's vcpkg installation, GCC 15.2.0,
and baseline x86-64 SSE (no `-march=native`). The scalar Rust triangular matrix
solve preserves the corresponding 8-float / 4-double small-panel ordering.
This ordering is configuration-specific; equivalence on other SIMD configurations
must be evaluated numerically rather than claimed bitwise.

```sh
c++ -std=c++17 -O1 -DBASALT_INSTANTIATIONS_FLOAT \
  -I/home/zhongjin/Projects/basalt/include \
  -I/home/zhongjin/Projects/basalt-headers/include \
  -I/home/zhongjin/Projects/basalt/build/release/vcpkg_installed/x64-linux/include/eigen3 \
  -I/home/zhongjin/Projects/basalt-headers/build/deps/prefix/usr/include \
  crates/firefly-basalt/tests/reference/marg_helper.cpp \
  /home/zhongjin/Projects/basalt/src/vi_estimator/marg_helper.cpp \
  -o /home/zhongjin/Projects/basalt-headers/build/marg_helper_reference
/home/zhongjin/Projects/basalt-headers/build/marg_helper_reference f32
/home/zhongjin/Projects/basalt-headers/build/marg_helper_reference f64
```

`ba_utils.cpp` and `landmark_block_abs_dynamic.cpp` also use Eigen 5.0.1.
Compile them separately with C++17, `-O1 -DFMT_HEADER_ONLY`, the include roots
above plus `basalt/build/release/vcpkg_installed/x64-linux/include`, the pinned
Sophus include root and fmt include root. Each executable accepts `f32` or `f64`.
The landmark probe changes only the access modifier around the unmodified
header to inspect its private numeric storage; calls to protected overrides go
through the original public virtual interface. It records both Householder and
Givens paths, every damping transition and the final back-substitution.
Invalid reprojection outputs in the BA utility probe are initialized to known
sentinels so early-return behavior is testable without reading uninitialized
C++ values. The production upstream landmark path itself reads uninitialized
Jacobians if invalid projections are explicitly enabled; safe Rust reports
`NumericalFailure` via NaN Jacobians for that undefined case.

`amd.cpp`, `sparse_accumulator.cpp` and `linearization.cpp` use the same official
Basalt revision and Eigen 5.0.1. AMD records exact permutations for 28 graph
patterns. The sparse accumulator records direct and conjugate-gradient solves,
with and without damping, for four matrix sizes; it executes the upstream
double-precision implementation. The linearization probe includes the original
template implementation files and compares all three modes on a stereo window,
including FEJ states and a square-root prior. Its outputs include cost, reduced
normal equations, predicted decrease and landmark back-substitution.
The window fixtures include separate inertial states and preintegrated IMU
measurements as well as the visual-only and FEJ/prior cases. Both scalar types
pass the fixed Schur thresholds, including FEJ right-hand sides. Intermediate
current transforms, Jacobians, residuals, landmark normal blocks and inverses
are checked separately, and the Schur reducer also receives exact C++ block
inputs. QR Jacobian/residual comparisons retain fixed thresholds,
while normal-product checks use input perturbations plus the dot-product
gamma_(2n) forward bound, as in the independent landmark and IMU tests.
Compile the linearization probe with `-std=c++17 -O0 -DFMT_HEADER_ONLY`, the
include roots above, the cereal and magic_enum include roots, and `-ltbb`
(system TBB 2022.3.0). The other two probes use `-O1`; the AMD probe requires only
Eigen and nlohmann JSON. Run each executable without arguments to emit its JSON
fixture; the linearization executable accepts `f32` for its float fixture and
defaults to double. These fixtures do not constitute a full-sequence VIO comparison.

`utility_helpers.cpp` executes the original `sim_utils.h`, `test_utils.h`, and
`vis_utils.h` using the same source pins. Compile with C++17, `-O0`,
`-DFMT_HEADER_ONLY -DHAVE_EIGEN`, and the include roots above including the
release-preset Pangolin headers. It supplies GL call interceptors to capture
the actual camera vertex array and render state without opening a window.
The fixture covers Cereal JSON observations, all color-ramp intervals, four
camera sizes, and the Jacobian diagnostic's finite/NaN reporting predicate.
The Rust visualization helper returns drawing primitives to the caller;
Pangolin interactive rendering is not implemented by this helper.

`kitti_eval.cpp` includes the unmodified upstream executable source with only
its `main` symbol renamed. Compile with C++17, `-O0 -ffunction-sections
-fdata-sections -DFMT_HEADER_ONLY`, the include roots above, `-Wl,--gc-sections`,
and the official vcpkg `libCLI11.a`. It compares actual original KITTI segment
evaluation, including lengths with no complete segment. Non-finite values are
represented as null in the fixture; the Rust result writer preserves Cereal's
`NaN`/`Infinity` extension instead of silently converting them to null.

`time_alignment.cpp` similarly includes the original executable source and
calls its nearest-sample error function; its refinement block executes the
original Eigen JacobiSVD expression. Compile with the same flags plus
`-march=native -DNDEBUG -DEIGEN_DONT_PARALLELIZE -DGL_GLEXT_PROTOTYPES
-DHAVE_EIGEN` and the official `basalt_time_alignment` link libraries from
`build/release/build.ninja`. No window is opened. The Rust implementation keeps
the full-rank CPQR preconditioner and two-sided Jacobi updates for the fixed
21-by-3 polynomial fit; it does not replace this with normal equations.

`vio_sim.cpp` includes the original simulation executable and calls its complete
`gen_data()` for 1000 frames, 200 Hz IMU, and 32 spherical landmarks. It uses the
same compile/link flags as the time-alignment probe and writes the original
generated archive under the upstream `build/results/vio-sim-reference` directory.
The committed fixture samples knots, frames, IMU values, noise, bias walks,
landmarks, and stereo image observations across the sequence. The Linux reference
uses glibc's default rand stream, Eigen 5's two-draw 52-bit random mantissa, and
MT19937 seed 1 with the libstdc++ polar normal sampler. No GUI window is opened.

`spline_linearize.cpp` executes every original spline calibration measurement
family (pose, accelerometer, gyroscope, Aprilgrid, motion capture), with optional
intrinsics/gravity/time/IMU-scale blocks enabled and disabled. It stores sparse
entries of the unmodified dense accumulator, the complete gradient, robust cost
and the independently evaluated cost. `spline_optimize.cpp` executes three
original sparse LM calls and records every knot, gravity and both bias vectors.
Both probes use the same official revisions and Eigen 5.0.1, C++17,
`-O0 -DFMT_HEADER_ONLY`, the Basalt and headers include roots, vcpkg's include
root and Eigen subdirectory, pinned Sophus, fmt, and `-ltbb`. Neither probe links
the Basalt shared library. Run without arguments to emit the corresponding JSON.

`nfr.cpp` calls the original `utils/nfr.h` residuals and analytic Jacobians,
and Eigen 5.0.1 `FullPivHouseholderQR` on zero, full-rank, rank-deficient and
ill-conditioned matrices. It is compiled with GCC, `-std=c++17 -O2`, the official
Basalt/header include roots and the official release vcpkg include/eigen3 roots.
The executable takes the output JSON path as its sole argument. `tests/nfr.rs`
compares the residuals/Jacobians with fixed absolute thresholds and uses separate
central differences; QR checks compare the original rank and a normalized
backward residual. The mapper covariance test uses analytic identity information
and checks both its Gauss–Newton and LM factor optimization paths.

`dataset_io.cpp` runs the original Euroc/UZH/KITTI readers on the same disk
fixtures created by `tests/dataset_io.rs`. Set `BASALT_DATASET_REFERENCE` to the
compiled executable to run that comparison. The executed oracle uses official
Basalt/header sources, Eigen 5.0.1 and system OpenCV 4.10.0; timestamps, IMU,
ground truth poses, exposure and decoded pixels are compared. `dataset_archive.cpp`
records the original native Cereal image/IMU bytes. The ROSbag reader is not
covered by these three directory-format reader cases.

`marg_data.cpp` includes the official `src/io/marg_data_io.cpp` serialization
overloads and writes native Cereal `marg_data.cereal` and `optical_flow.cereal`
(two output path arguments). It uses official release vcpkg Eigen 5.0.1,
Cereal, Sophus and TBB, compiled with GCC `-std=c++17 -O2` and linked with
`-ltbb -lfmt`. `tests/marg_data_io.rs` requires byte-exact read/write agreement,
including frozen/current states, nonzero deltas, row-major dynamic matrices,
repeated shared pointers, shared camera images and null images. Native binary
fixtures target little-endian 64-bit platforms. The directory worker test also
covers image deduplication, image reattachment and decode-failure EOS.

`optical_flow_dataset.cpp` links the official `libbasalt` and exports the first
five real dataset frames: image hashes/exposure, feature IDs and affine
coefficients. Its arguments are bag path, VIO config, calibration, output JSON
and frame count. Match the library's Eigen ABI flags (`-O3 -DNDEBUG
-DEIGEN_DONT_PARALLELIZE -march=native` for the release preset), and link
`basalt`, `tbb`, `fmt` with the release vcpkg include/library roots.
`BASALT_FLOW_DIRECT` instantiates the same official tracker in the probe;
`-ffp-contract=off` enables a separate compiler-rounding control experiment.
The ignored `tests/optical_flow_dataset.rs` requires `BASALT_SOURCE`,
`BASALT_DATASET`, and `BASALT_FLOW_REFERENCE`; it writes numeric comparison
artifacts to `logs/basalt_optical_flow_{cpp,rust}.json`. Its absolute `1e-5`
coefficient assertion is independent of the synthetic tracker fixtures.

`triangulation.cpp` calls the original templated `BundleAdjustmentBase::triangulate`
with Eigen 5 `JacobiSVD` for ten cases in each precision: stereo geometry,
rotated rig, noisy rays, zero/tiny baseline, reversed rays, rays at infinity,
zero rays and large coordinates. Its sole argument is the output JSON path.
Compile with `-std=c++17 -O2`, official source/header and release vcpkg include
roots and `-lfmt`. `tests/triangulation.rs` compares all four inverse-distance
coordinates at the existing scalar thresholds (`1e-5`/`1e-10`, scaled by
`1+abs(reference)`) and checks nonfinite classification explicitly.

`poses_optimization.cpp` invokes the original `LinearizePosesOpt` with pinhole
and double-sphere cameras, two rig poses, robust residuals, and both settings of
intrinsic optimization. Compile with GCC `-std=c++17 -O2`, the official Basalt
include root and release vcpkg include/eigen3 roots; its sole argument is the
output JSON path. `tests/poses_optimization.rs` compares every normal-matrix
entry, right-hand-side entry and accumulated cost at `1e-10 * (1 + abs(reference))`.
The separate nonlinear test checks reprojection convergence and the fixed
reference camera. It does not establish complete calibration equivalence.

`calibration.cpp` compiles original `src/calibration/aprilgrid.cpp` and
`src/calibration/vignette.cpp` with GCC `-std=c++17 -O2`, official Eigen 5.0.1,
system OpenCV 4.10.0 and TBB. Arguments are the output JSON path and a writable
grid configuration path (the probe removes that configuration on completion).
The fixture checks grid geometry and vignette initialization, irradiance update,
spline update and ten alternating updates on two cameras. Tests use `1e-15`
geometry, `1e-12` knots, `1e-10` cost and exact float log output.

`apriltag.cpp` compiles original `thirdparty/apriltag/src/apriltag.cpp` and all
`thirdparty/apriltag/ethz_apriltag2/src/*.cc` with GCC `-std=c++17 -O2`, the
official Basalt and AprilTag include roots, release vcpkg include roots and
`pkg-config --cflags --libs opencv4` (system OpenCV 4.10.0). Its sole argument is
the output JSON path. Four pixel fixtures cover four-tag grids, Gaussian blur,
90-degree rotation and a blank image. Raw detections and complete wrapper
outputs are independently compared: IDs and Hamming distances exactly, corner
coordinates within `2e-3` pixels and radii within `1e-4`. These are synthetic
fixtures, not a real calibration sequence validation. Rust's subpixel kernel
translates OpenCV 4.10.0 `cornersubpix.cpp` and `samplers.cpp` at verified commit
`71d3237a093b60a27601c20e9ee6c3e52154e8b1`; existing PureCV's different gradient
formulation is not used for this step. The ETH detector and dictionary retain
their BSD-4-Clause license and required advertising acknowledgement.
The same probe also includes five fractional/border subpixel starts on a
textured image (`2e-5` coordinate tolerance), and tag `getRelativeTransform`
outputs on the 12 detected tags plus four tilted/noisy projections (`1e-6`
matrix-entry tolerance). The pose specialization retains the original planar
initialization, Rodrigues coordinates and 20-iteration multiplicative-diagonal
Levenberg–Marquardt loop; it does not use PureCV's six-point DLT-only PnP.

`dataset_rosbag.cpp` links the unmodified official release `libbasalt.so` and
calls its dataset factory. Arguments are input bag and output JSON. The
`dataset_rosbag.json` fixture is TUMVI `dataset-room1_512_16.bag`, SHA-256
`ce73ddd08e607e1cef53d8bd2db4f4675639fe82cdd32e7df41fb8ad1cc584d8`.
It records all 2821 timestamps, byte hashes of all 28122 samples in each IMU
stream and all 16540 GT poses, explicit values every 137 samples, and full
pixel hashes/ranges/exposure/border values from 31 stereo frames. Hash input
is explicitly little-endian u64 words, with doubles hashed by IEEE bits.
The ignored real-data test is executed with `BASALT_DATASET_BAG=<bag>` and
`cargo test -p firefly-basalt --test dataset_rosbag -- --ignored`; ignoring it
by default avoids requiring the large external dataset in ordinary unit tests.

`dataset_rosbag_fixture.cpp` uses the official release ROS writer to generate
the three small checked-in `.bag` fixtures (none, BZ2, LZ4); its argument is
this reference directory. Compile with GCC `-std=c++17 -O2`, the original
`thirdparty/ros/include` and release vcpkg include roots, link `libbasalt.so`
and vcpkg `libconsole_bridge.a`. Run `dataset_rosbag.cpp` on each resulting
bag to regenerate its JSON. The ordinary test checks exact complete metadata
and all three stereo frames: mono8 scaling, mono16 values 0/1/255/256/32767/
32768/65534/65535, exposure parsing, lexical camera ordering, nonzero clock
offset and the upstream omission of the final GT sample. The original reader
ignores `is_bigendian`; the test preserves that behavior rather than claiming
portable big-endian image conversion.

`calibration_archive.cpp` invokes the original calibration-helper Cereal
overloads. Compile with GCC `-std=c++17 -O2`, official source/header and
release vcpkg include roots and vcpkg `-lfmt`; its argument is the JSON output
path. The Rust test compares exact native archive bytes for nonempty and empty
corner/initial-pose records, reads the C++ archive back, and rejects every
truncation. These native archive fixtures target little-endian 64-bit hosts.

`cam_calib.cpp` links the actual original
`CMakeFiles/basalt_calibrate.dir/src/calibration/cam_calib.cpp.o` and its
original executable dependencies. It exposes private controller fields only
inside the probe; no controller method is replaced. Compile the probe with
the official release ABI flags (`-march=native`, `-DEIGEN_DONT_PARALLELIZE`,
`-DHAVE_EIGEN`, `-DHAVE_EPOXY` and both Basalt scalar instantiation defines),
then reuse the final link command from `ninja -t commands basalt_calibrate`,
substituting the probe object for `calibrate.cpp.o`, a separate output binary,
and removing the executable dependency-file flag. Mismatching Eigen alignment
flags across that object boundary is invalid. Arguments are output JSON, a
temporary grid JSON path and an existing cache directory. The probe removes
its grid file. It executes the headless camera overlap graph, optimization
initialization, projection and one optimization step. The Rust test compares
extrinsics/initial poses at `1e-12`, projected corners at `1e-10`, and statistics
at `1e-9*(1+abs(reference))`; another test loads the original Cereal corner/pose
maps and checks dataset subsampling. GUI windows/overlays are not exercised.
# Camera–IMU rotation initialization

`cam_imu_calib.cpp` evaluates the angular-velocity least-squares expression and
Eigen `AngleAxis` projection in original `cam_imu_calib.cpp`, using the original
release dependencies (Eigen 5.0.1 and Sophus). It exercises four rotations and
nonorthogonal least-squares matrices. `cam_imu_calib.json` is its output;
`tests/cam_imu_calib.rs` compares the Rust result at absolute tolerance `1e-10`
and checks exclusion of gyro sample zero and unobservable inputs. This probe
tests the initialization expression, not the complete calibration controller.

Build with `c++ -std=c++17 -O2 -DFMT_HEADER_ONLY`, include directories
`$BASALT/build/release/vcpkg_installed/x64-linux/include/eigen3` and
`$BASALT/build/release/vcpkg_installed/x64-linux/include`, and output executable
`$BASALT/build/cam_imu_calib_reference`.

`cam_imu_controller.cpp` calls the unmodified original `CamImuCalib` controller
object from `CMakeFiles/basalt_calibrate_imu.dir/src/calibration/cam_imu_calib.cpp.o`.
Use the camera-controller compile/link procedure above with target
`basalt_calibrate_imu`, replacing `calibrate_imu.cpp.o` with this probe. Keep
`-march=native`, `-DEIGEN_DONT_PARALLELIZE` and both Basalt instantiation defines.
Arguments are output JSON and the existing `cam_calib_grid.json` fixture.
`cam_imu_controller.json` contains initialization with two cameras, 201 IMU
samples, six image timestamps, eight mocap poses, projected corners and complete
Pangolin diagnostics after mocap initialization. The Rust test checks geometric
outputs at `1e-10*(1+abs(reference))` and float diagnostics at
`1e-5*(1+abs(reference))`. This test covers initialization and reporting, not
the complete nonlinear camera–IMU calibration convergence or GUI rendering.

## Mapper simulation entry points

`mapper_sim.cpp` includes the unmodified original translation unit with its
`main` renamed. Its three matrix cases cover LDLT-based nonlinear factor
extraction and the 21/36-variable pose/IMU marginalization callbacks. The
12-variable pose-only case tests factor extraction directly: the original
standalone marginalization callback crashes for an empty elimination set in
the locked release build. The headless production path uses `NfrMapper`, which
guards that case. `tests/mapper_sim.rs` compares all reduced matrices, vectors
and information matrices at `1e-10*(1+abs(reference))`.
The same probe executes `randomInc` followed by `randomYawInc`; its pose is
compared against the persistent Eigen/glibc stream at absolute `1e-10`.

`mapper_sim_naive.cpp` creates native original Cereal inputs using
`MargDataSaver`, writes the simulation JSON with duplicate keys, and calls the
original `gen_data`. Inputs reside in `mapper_sim_naive/`; expected decoded
values are in `mapper_sim_naive.json`. The Rust test checks all archived IMU
timestamps, selected keyframes, point/pose data and the two distinct
`noisy_accel` fields. Separate integration tests execute both Rust headless
pipelines and exercise input-error shutdown. They do not claim numerical
equivalence of an entire optimized trajectory to the original executable.

For each probe, use its original target's `ninja -t commands` compile/link
flags, including `-march=native` and both instantiation defines. Replace the
target's main object with the probe object and remove the linker dependency
file flag. The mapper probe takes its JSON output path; the naive probe takes
its dedicated fixture directory and output JSON path. The original saver
recreates that directory, so pass only the dedicated fixture path.

`mapper.cpp` and `opt_flow.cpp` include the original executable sources at
`0f3b2b52` with renamed `main`, and invoke their original trajectory writer and
result collector. Their JSON fixtures were produced by GCC C++17, `-O0
-march=native -DNDEBUG -DEIGEN_DONT_PARALLELIZE -DHAVE_EIGEN
-DGL_GLEXT_PROTOTYPES -ffunction-sections -fdata-sections`, using the official
release Eigen 5 headers and the `basalt_time_alignment` link libraries from
`build/release/build.ninja`, with `-Wl,--gc-sections`. The mapper probe runs in
`basalt/build/results/mapper-reference` and produces native `gt.cereal` plus
both original trajectory files there; JSON embeds those outputs for portable
Rust tests. Export tests compare the exact header/column order and numeric
values at the existing double threshold; C++ and Rust exponent spelling may
differ. Flow collection covers repeated timestamps, multi-camera IDs and image
reference release. The fixed fixtures do not validate a full mapper sequence.
When given CLI arguments, `mapper.cpp` instead runs the unchanged original
`main` and then its TUM trajectory-export callback. Compile and link this mode
with the exact `basalt_mapper` commands from `ninja -t commands`, replacing only
the main source/object and output paths. It accepts the original `--show-gui
false --cam-calib ... --marg-data ... --config-path ... --result-path ...`
options and writes `keyframeTrajectory.txt` in its working directory. This mode
has read the Rust application's real dataset cache and completed all mapping
stages. Default RANSAC uses random seeds, so repeated runs can change tracks;
the end-to-end run alone does not establish strict optimizer equivalence.
The same probe accepts `--freeze CALIBRATION CACHE CONFIG OUTPUT.json` to save
original detected pixels and tracks plus geometry before setup and after each
optimization/filter stage. `--replay CALIBRATION CACHE CONFIG INPUT.json
OUTPUT.json` skips random detection/matching and injects the frozen tracks.
The ignored `mapper_frozen` Rust test reads `BASALT_SOURCE`,
`BASALT_MAPPER_CACHE`, and `BASALT_MAPPER_FROZEN`, then checks setup and final
geometry with the existing double-precision threshold and discrete membership
at every stage. Repeated C++ runs on identical frozen inputs exceeded that
threshold in intermediate optimization states, so these states are diagnostic
outputs rather than deterministic numeric oracles. It writes diagnostics to
`logs/basalt_mapper_frozen_rust.json`; intermediate and final comparisons are
reported separately without changing the existing unit-test tolerances.

`vio_sim_trajectory.cpp` calls the complete original simulation executable's
renamed `main` and then exports retained poses at all input timestamps to
`trajectory.txt` in its working directory. It checks that each latest
visualization pose has exactly the independently collected state-output
translation. This obtains full-precision trajectory evidence without modifying
upstream source; the original executable only writes six-significant-digit ATE.
Compile with the mapper probe's includes/libraries, using the original release
`-O3 -march=native -DNDEBUG -DEIGEN_DONT_PARALLELIZE` flags.
