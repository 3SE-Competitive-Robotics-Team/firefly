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

#heading[Related Works]

<sec:related>

*Visual-inertial odometry (VIO).* Filter-based VIO dates back to the
multi-state constraint Kalman filter (MSCKF) #cite(<mourikis2007msckf>),
#cite(<li2013msckf>), which marginalizes past camera poses and maintains a sliding
window of feature-bearing states; OpenVINS #cite(<geneva2020openvins>)
provides a mature open-source research platform for this paradigm.
Optimization-based systems such as VINS-Mono #cite(<qin2018vinsmono>) and
VINS-Fusion #cite(<qin2019vinsfusion>) formulate tightly-coupled
bundle-adjustment problems that deliver high accuracy at the cost of higher
computation. All of these systems rely on explicit feature detection,
matching, and triangulation, and none exploits dense geometric structure.

*Direct methods.* Direct photometric alignment avoids feature extraction by
minimizing intensity residuals directly. SVO #cite(<forster2017direct>)
combines sparse feature alignment with a depth filter, and DSO
#cite(<engel2018dsolam>) demonstrates that dense direct alignment with
photometric calibration can rival feature-based methods on monocular SLAM.
FAST-LIVO2 #cite(<lin2025fastlivo2>) extends this idea to a tightly-coupled
LiDAR-inertial filter: reference image patches stored on visual map points
are warped by the current pose hypothesis and compared photometrically,
jointly estimating the inverse exposure time. DIVO inherits this direct
alignment formulation and applies it to depth-camera platforms.

*LiDAR-inertial odometry (LIO).* LOAM #cite(<zhang2014loam>) established the
point-to-edge and point-to-plane residual paradigm. FAST-LIO2
#cite(<xu2023fastlio2>) reformulated LIO as an iterated Kalman filter with an
incremental iKD-tree map, achieving state-of-the-art accuracy and efficiency.
VoxelMap #cite(<zhou2021voxelmap>) proposed an adaptive hierarchical
probabilistic voxel map in which each voxel fits a local plane with an
SVD-based covariance; FAST-LIO2 with VoxelMap reduces point-to-plane
residual noise and improves accuracy. DIVO's map is a direct port of this
voxel-map design to depth input: 0.5 m root voxels, octree subdivision to
depth 3, and plane fitting with an explicit 6-DoF uncertainty
#math.op("Σ")#math.attach("", b: "nq").

*LiDAR-inertial-visual fusion.* R3LIVE #cite(<lin2022r3live>) performs
LIV fusion with an ESIKF for LiDAR-inertial odometry and a separate
factor-graph visual update that also colors the map. LVI-SAM
#cite(<shan2021lvisam>) couples LIO with visual-inertial odometry through a
factor graph and loop closure. FAST-LIVO2 #cite(<lin2025fastlivo2>) achieves
tight coupling by feeding both LiDAR point-plane residuals and sparse direct
photometric residuals into the *same* ESIKF sequential update, which is
exactly the architecture DIVO adopts. The primary difference is the range
sensor: FAST-LIVO2 consumes LiDAR scans, while DIVO consumes depth maps with
a disparity-domain noise model, which is the central adaptation described in
Section #ref(<sec:depth>).

*Depth-aided estimation.* Onboard depth sensing has been used for drone
navigation in simulation-to-reality pipelines #cite(<loquercio2021drod>),
and surfel-based reconstruction from monocular depth has been shown to
produce high-quality maps in real time #cite(<quenzel2020sdf>). These works
treat depth as a source of dense geometric cues for mapping or learning, but
do not tightly couple depth residuals into an inertial filter with
principled depth-dependent noise — the gap DIVO fills.

*Safety and motion planning context.* DIVO is developed inside the firefly
project as the state-estimation front end of an end-to-end autonomy stack
whose planning module follows EGO-Planner #cite(<sun2022egoplanner>) and
FASTER #cite(<tordesillas2020factor>). The accuracy targets (sub-0.2 m ATE on
a hovering platform) are set by the closed-loop planning and control
requirements of this stack.
