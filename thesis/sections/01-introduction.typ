// --- shared macros (must be defined per-file: include has its own scope) ---
#let Exp(x) = math.op("Exp") + x
#let Log(x) = math.op("Log") + x
#let boxplus(x, d) = x + math.cal("⊞") + d
#let boxminus(a, b) = a + math.cal("⊟") + b
#let xG = math.bold("x")
#let Rwi = math.bold("R") + math.attach("", b: "wi")
#let pwi = math.bold("p") + math.attach("", b: "wi")
#let vwi = math.bold("v") + math.attach("", b: "wi")
#let bg = math.bold("b") + math.attach("", b: "g")
#let ba = math.bold("b") + math.attach("", b: "a")
#let gvec = math.bold("g")
#let qq = math.bold("q")
#let xhat = "x" + "\u{0302}"
#let Phat = "P" + "\u{0302}"
#let pv = math.bold("p")
#let uv = math.bold("u")
#let tv = math.bold("t")
#let dhat = math.bold("d") + "\u{0302}"
#let phat = math.bold("p") + "\u{0302}"
// --- end shared macros ---

#heading[Introduction]

Autonomous micro aerial vehicles (MAVs) operating in cluttered, GPS-denied
indoor environments require an accurate, low-latency ego-motion estimate
together with a consistent local geometric map for planning and control
#cite(<sun2022egoplanner>). Tightly-coupled sensor fusion — combining an
inertial measurement unit (IMU) with range and/or vision sensors — has become
the de-facto standard for such estimation: the IMU provides high-rate motion
priors between slow, information-rich measurements, while the latter correct
the accumulated drift and anchor the state to the physical scene
#cite(<mourikis2007msckf>), #cite(<li2013msckf>), #cite(<qin2018vinsmono>).

LiDAR-inertial-visual (LIV) odometry systems such as R3LIVE
#cite(<lin2022r3live>), LVI-SAM #cite(<shan2021lvisam>) and FAST-LIVO2
#cite(<lin2025fastlivo2>) achieve exceptional accuracy and robustness by
tightly fusing LiDAR point-to-plane residuals with sparse direct visual
alignment inside a single iterated Kalman filter. FAST-LIVO2 in particular
demonstrated that a *direct* visual model — minimizing photometric residuals
between warped reference patches and the current image — can be tightly
coupled with LiDAR-inertial odometry at real-time rates, without explicit
feature matching or descriptors.

However, the LiDAR-centric assumptions of these systems do not transfer
trivially to platforms whose primary range sensor is a *depth camera*.
Commodity stereo and structured-light depth sensors differ from LiDAR in
three fundamental ways:

1. *Measurement geometry.* A LiDAR scan samples the scene with beam
   divergence in angular space; a depth camera produces a dense range image
   whose range noise grows quadratically with depth in the disparity domain
   (#math.op("σ")#math.op("∝")#math.op("z")#math.attach("", t: "2")).
2. *Sensor rate and throughput.* LiDARs deliver 100–500 k points per second;
   a 320×240 depth camera at 10 Hz delivers up to 76.8 k points per frame,
   concentrated in a narrow field of view, and with 5–15% invalid holes.
3. *System integration.* Production LiDAR pipelines rely on ROS
   #cite(<quigley2009ros>) message passing; embedded and simulation platforms
   increasingly prefer zero-copy, lock-free shared-memory transport with
   strict memory-safety guarantees.

This paper presents *DIVO* (Direct Depth-Inertial-Visual Odometry), the
fifth-stage deliverable of the firefly-void module of the firefly project
#cite(<fireflydesign>): a complete, from-scratch Rust implementation of the
FAST-LIVO2 algorithm family adapted to depth-camera input, structured
exactly along the FAST-LIVO2 paper #cite(<lin2025fastlivo2>) so that the two
implementations can be compared section by section.

The contributions of this paper are:

- *A depth-camera measurement-model adaptation.* The LiDAR point
  uncertainty model of FAST-LIVO2 — range noise plus encoder-direction
  noise plus beam divergence — is replaced by a disparity-domain model
  #math.op("σ")#math.op("∝")#math.op("z")#math.attach("", t: "2") with a hole-neighborhood
  term and pixel-quantization angular uncertainty. The point-to-plane
  residual and its Jacobian (with respect to the right-perturbation
  rotation error) are preserved and re-derived for the depth case.
- *A full ESIKF with sequential state update* (Section #ref(<sec:esikf>))
  over a 19-dimensional manifold state
  #math.op("[")#math.bold("R")#math.attach("", b: "wi")#math.comma#math.bold("p")#math.attach("", b: "wi")#math.comma#math.bold("v")#math.attach("", b: "wi")#math.comma#math.bold("b")#math.attach("", b: "g")#math.comma#math.bold("b")#math.attach("", b: "a")#math.comma#math.bold("g")#math.comma#math.op("τ")#math.op("]"),
  with forward/backward IMU propagation and covariance propagation.
- *An adaptive hierarchical voxel map* (Section #ref(<sec:mapping>)) that
  fits local planar patches by SVD, maintains visual map points with
  three-level image patches, scores reference patches by NCC and viewpoint
  cosine, refines plane normals offline by photometric minimization, and
  prunes the map with a sliding window.
- *A pipeline-level update gate* (Section #ref(<sec:gate>)) that rejects
  single-frame depth/visual updates whose state change exceeds a motion
  bound (0.1 m / 3° / 0.5 m s⁻¹), eliminating the explosion observed when a
  freshly-initialized plane is used before its parameters have converged.
- *An industrial-grade Rust implementation* (Section #ref(<sec:system>)):
  `unsafe`-free code, ROS-free zero-copy IPC through iceoryx2
  #cite(<iceoryx2>), fastrace-based distributed tracing, and 493 unit and
  integration tests with finite-difference Jacobian cross-checks at errors
  below 1e-6.

The remainder of this paper is organized as follows. Section #ref(<sec:related>)
reviews related work. Section #ref(<sec:system>) gives the system overview.
Section #ref(<sec:esikf>) derives the ESIKF with sequential state update.
Section #ref(<sec:mapping>) details the local mapping. Sections
#ref(<sec:depth>) and #ref(<sec:visual>) present the depth and visual
measurement models. Section #ref(<sec:experiments>) reports experimental
results, and Section #ref(<sec:conclusion>) concludes.
