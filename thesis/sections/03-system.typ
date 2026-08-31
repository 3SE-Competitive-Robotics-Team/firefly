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

#heading[System Overview]

<sec:system>

DIVO is the odometry-and-mapping front end of the firefly autonomy stack. It
runs as a standalone process (`void`) that subscribes to three sensor topics
published by a MuJoCo #cite(<todorov2012mujoco>) physics simulation over the
iceoryx2 #cite(<iceoryx2>) zero-copy shared-memory transport, and publishes
pose and visualization topics that downstream planning and rendering
consumers subscribe to. Table #ref(<tab:sensors>) summarizes the sensor
configuration.

#figure(
  table(
    columns: (auto, auto, auto, auto),
    align: (left, left, center, center),
    stroke: 0.5pt,
    table.header(
      table.cell[#text(weight: "bold")[Sensor]],
      table.cell[#text(weight: "bold")[Topic]],
      table.cell[#text(weight: "bold")[Rate]],
      table.cell[#text(weight: "bold")[Content]],
    ),
    [IMU], [$"Firefly/Imu"$], [100 Hz], [angular velocity, specific force],
    [Left gray camera], [$"Firefly/CameraLeft"$], [10 Hz], [320×240 8-bit gray],
    [Depth camera], [$"Firefly/Depth"$], [10 Hz], [320×240 float depth (m)],
  ),
  caption: [Sensor configuration of the MuJoCo-based evaluation platform.],
)<tab:sensors>

The depth camera and the left gray camera are rigidly attached to the same
body, so their extrinsic transform is near-identity and remains
configurable. Internally, DIVO operates in a *virtual pinhole camera frame*:
depth pixels and left-gray pixels coincide exactly after a fixed rotation
that maps the OpenGL-style depth convention (forward #math.op("−z")) to the
pinhole convention (forward #math.op("+z")). IMU angular velocities and
specific forces are rotated into this virtual frame before propagation, so
both measurement models share a single camera frame with unit extrinsic.

#figure(
  {
    let w = 200pt
    let h = 116pt
    let pad = 6pt
    let box_fill = rgb("#ffffff")
    let border = 0.6pt + rgb("#333333")
    let mono = text.with(weight: "regular", font: ("New Computer Modern", "DejaVu Sans Mono", "PingFang SC"), size: 7.6pt)
    let arr = (len) => {
      // horizontal arrow: line + triangle head
      box(width: len, height: 6pt)[
        #place(dx: 0pt, dy: 2pt)[
          #line(length: len - 6pt, stroke: 0.6pt)
        ]
        #place(dx: len - 6pt, dy: 0pt)[
          #polygon((0pt, 0pt), (6pt, 3pt), (0pt, 6pt), fill: rgb("#333333"), stroke: none)
        ]
      ]
    }

    // Sensor boxes (left column)
    let imu_box = box(
      width: 150pt, height: 22pt,
      fill: box_fill, stroke: border, radius: 3pt, inset: (x: 4pt, y: 2pt),
    )[#align(center)[#mono[Firefly/Imu — 100 Hz]]]
    let cam_box = box(
      width: 150pt, height: 22pt,
      fill: box_fill, stroke: border, radius: 3pt, inset: (x: 4pt, y: 2pt),
    )[#align(center)[#mono[Firefly/CameraLeft — 10 Hz]]]
    let dep_box = box(
      width: 150pt, height: 22pt,
      fill: box_fill, stroke: border, radius: 3pt, inset: (x: 4pt, y: 2pt),
    )[#align(center)[#mono[Firefly/Depth — 10 Hz]]]

    // Core pipeline box
    let core_box = box(
      width: 180pt, height: 62pt,
      fill: rgb("#f2f6ff"), stroke: border, radius: 3pt, inset: (x: 6pt, y: 3pt),
    )[
      #align(center)[
        #text(weight: "bold", size: 8.4pt)[DIVO core (firefly-void)]
      ]
      #v(2pt)
      #mono[ESIKF ⊞-update: ① depth ② visual]
      #v(1.5pt)
      #mono[adaptive voxel map]
      #v(1.5pt)
      #mono[update gate: 0.1 m / 3°]
    ]

    // Output boxes
    let odom_box = box(
      width: 150pt, height: 22pt,
      fill: box_fill, stroke: border, radius: 3pt, inset: (x: 4pt, y: 2pt),
    )[#align(center)[#mono[Firefly/VoidOdom — 10 Hz]]]
    let viz_box = box(
      width: 150pt, height: 22pt,
      fill: box_fill, stroke: border, radius: 3pt, inset: (x: 4pt, y: 2pt),
    )[#align(center)[#mono[Firefly/Viz — void namespace]]]

    place(
      dx: 0pt, dy: 0pt,
      box(width: w, height: h)[
        // IMU → core
        #place(dx: 0pt, dy: 0pt)[#imu_box]
        #place(dx: 0pt, dy: 28pt)[#cam_box]
        #place(dx: 0pt, dy: 56pt)[#dep_box]
        // core
        #place(dx: 210pt, dy: 16pt)[#core_box]
        // outputs
        #place(dx: 440pt, dy: 0pt)[#odom_box]
        #place(dx: 440pt, dy: 56pt)[#viz_box]
        // arrows: sensors → core
        #place(dx: 152pt, dy: 9pt)[#arr(58pt)]
        #place(dx: 152pt, dy: 37pt)[#arr(58pt)]
        #place(dx: 152pt, dy: 65pt)[#arr(58pt)]
        // arrows: core → outputs
        #place(dx: 392pt, dy: 16pt)[#arr(48pt)]
        #place(dx: 392pt, dy: 56pt)[#arr(48pt)]
        // annotations
        #place(dx: 30pt, dy: 86pt)[#text(size: 7.6pt)[Depth, gray and IMU arrive over zero-copy iceoryx2 IPC]]
        #place(dx: 230pt, dy: 92pt)[#text(size: 7.6pt)[Sequential ESIKF updates, then map update]]
        #place(dx: 448pt, dy: 86pt)[#text(size: 7.6pt)[Pose + visualization to downstream modules]]
      ],
    )
  },
  caption: [System data flow of DIVO. Three sensor streams enter the core
  pipeline through iceoryx2 subscriptions; the core runs IMU propagation,
  sequential depth/visual ESIKF updates, and the local map update, then
  publishes odometry at 10 Hz and visualization entities under the `void`
  namespace.],
)<fig:system>

The pipeline (Algorithm #ref(<alg:overview>)) processes each synchronized
depth+gray frame as follows: (i) all buffered IMU samples up to the frame
timestamp are consumed by forward propagation; (ii) the depth map is
back-projected into a structured point cloud and voxel-downsampled; (iii)
the depth point-to-plane measurement runs a sequential ESIKF update; (iv)
the sparse direct visual measurement runs a coarse-to-fine pyramid update
with re-warping; (v) both updates are screened by the *pipeline update gate*
(Section #ref(<sec:gate>)), which rejects state jumps exceeding 0.1 m or 3°;
and (vi) the accepted state is used to register the depth cloud into the
voxel map and to add/refine visual map points.

#figure(
  block[
    #text(weight: "bold")[Algorithm 1: DIVO main loop (`process_frame`)]
    #v(4pt)
    #set text(size: 8.8pt)
    #set par(leading: 0.35em)
    #par[
      #h(12pt)#text(weight: "bold")[Input:]#h(4pt) depth frame #math.bold("D")#math.attach("", b: "k"),
      gray frame #math.bold("I")#math.attach("", b: "k"), IMU buffer #math.bold("U")#math.attach("", b: "k")#h(4pt)
      #text(weight: "bold")[Output:]#h(4pt) state #xG#math.attach("", b: "k")
    ]
    #par[
      1. Forward propagate #xG over #math.bold("U")#math.attach("", b: "k") up to #math.bold("t")#math.attach("", b: "k")  (Section #ref(<sec:prop>))
    ]
    #par[
      2. Back-project #math.bold("D")#math.attach("", b: "k") → structured cloud; voxel-downsample  (Section #ref(<sec:depth>))
    ]
    #par[
      3. Depth point-to-plane ESIKF update → #xG#math.attach("", t: "d")  (Algorithm #ref(<alg:esikf>))
    ]
    #par[
      4. Pyramid direct-visual ESIKF update → #xG#math.attach("", t: "v")  (Section #ref(<sec:visual>))
    ]
    #par[
      5. Gate: if #math.norm(math.bold("p") + math.attach("", b: "k") + math.op("−") + math.bold("p") + math.attach("", b: "k−1")) > 0.1 m or
      #math.norm(math.op("Log") + math.op("(") + math.bold("R") + math.attach("", b: "k") + math.op("ᵀ") + math.bold("R") + math.attach("", b: "k−1") + math.op(")")) > 3°
      → reject update, keep propagation prior  (Section #ref(<sec:gate>))
    ]
    #par[
      6. Register cloud into voxel map; update visual map points  (Section #ref(<sec:mapping>))
    ]
  ],
  caption: [Overview of the DIVO main loop, mirroring the structure of
  FAST-LIVO2's `stateEstimationAndMapping`.],
)<alg:overview>
