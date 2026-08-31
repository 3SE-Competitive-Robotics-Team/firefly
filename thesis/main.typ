#set document(
  title: "DIVO: Direct Depth-Inertial-Visual Odometry — An Adaptive Voxel-based Tightly-coupled State Estimation and Mapping Framework",
  author: "Firefly Project",
  date: datetime(year: 2026, month: 1, day: 1),
)
#set page(
  paper: "us-letter",
  margin: (x: 2.2cm, y: 2.2cm),
  numbering: "1",
)
#set text(
  font: ("New Computer Modern", "New Computer Modern Math", "Noto Serif CJK SC", "PingFang SC"),
  size: 10pt,
  lang: "en",
)
#set par(justify: true, leading: 0.62em)
#set heading(numbering: "I.1")

#show heading.where(level: 1): it => block(breakable: false)[
  #set text(weight: "bold", size: 11pt)
  #v(0.35em)
  #it.body
  #v(0.25em)
  #line(length: 100%, stroke: 0.5pt)
  #v(0.35em)
]

// ---------------------------------------------------------------------------
// Notation & math helpers (consistent with FAST-LIVO2's convention)
// ---------------------------------------------------------------------------

// Rotation error: right-perturbation Exp, SO(3) exponential/logarithm.
#let Exp(x) = math.op("Exp") + x
#let Log(x) = math.op("Log") + x
#let skew(x) = math.op("⌊" + x + "×⌋")
#let vee(x) = x.sup("∨")
#let norm(x) = math.norm(x)
#let trace(x) = math.op("tr")(x)
#let diag(x) = math.op("diag")(x)

// Manifold state: x ⊞ δx, x ⊟ y (right-multiplication convention).
#let boxplus(x, d) = x + math.cal("⊞") + d
#let boxminus(a, b) = a + math.cal("⊟") + b

// The 19-D state vector.
#let xG = math.bold("x")

// Useful shorthands.
#let Rwi = math.bold("R") + math.attach("", b: "wi")
#let pwi = math.bold("p") + math.attach("", b: "wi")
#let vwi = math.bold("v") + math.attach("", b: "wi")
#let bg = math.bold("b") + math.attach("", b: "g")
#let ba = math.bold("b") + math.attach("", b: "a")
#let gvec = math.bold("g")

// Environment math.
#set math.equation(numbering: "(1)")

// ---------------------------------------------------------------------------
// Title block
// ---------------------------------------------------------------------------

#align(center)[
  #text(size: 16pt, weight: "bold")[DIVO: Direct Depth-Inertial-Visual Odometry]
  #v(2pt)
  #text(size: 13pt)[An Adaptive Voxel-based Tightly-coupled State Estimation and Mapping Framework]
]

#v(8pt)
#align(center)[
  #text(size: 10pt)[Firefly Project — The firefly-void Module]
  #v(2pt)
  #text(size: 9.5pt, fill: rgb("#444444"))[Manuscript draft aligned with the FAST-LIVO2 paper structure]
]

#v(10pt)

#block(width: 100%)[
  #rect(
    width: 100%,
    stroke: (top: 1pt, bottom: 1pt, left: none, right: none),
    inset: (x: 6pt, y: 8pt),
  )[
    #text(weight: "bold", size: 10pt)[Abstract]  #v(4pt)
    #par[
      This paper presents *DIVO*, a direct *depth*-inertial-visual odometry
      framework that tightly couples a depth camera, a stereo-pair gray
      camera, and an inertial measurement unit (IMU) through an
      error-state iterated Kalman filter (ESIKF) with sequential state
      updates. DIVO follows the architecture of FAST-LIVO2
      #cite(<lin2025fastlivo2>) and adapts its LiDAR-based measurement
      models to commodity depth sensors: the beam-divergence uncertainty of
      LiDAR points is replaced by a disparity-domain noise model
      #math.op("σ")#math.op("∝")#math.op("z")#math.attach("", t: "2") combined with
      pixel-quantization angular uncertainty, and the point-plane residuals
      are formed from back-projected structured depth maps rather than
      LiDAR scans. A hierarchical adaptive voxel map maintains local planar
      patches with online normal refinement, while visual map points carry
      multi-scale image patches that are tracked by sparse direct
      photometric alignment with joint inverse-exposure estimation. The
      entire system is implemented in Rust with a memory-safe,
      `unsafe`-free codebase and a ROS-free, zero-copy shared-memory IPC
      layer. On a 75 s hovering experiment in a high-fidelity physics
      simulation, DIVO achieves an ATE-RMS of 0.1782 m with all updates
      gated at the pipeline level to reject degeneracy-induced state jumps.
      With 493 unit and integration tests and finite-difference Jacobian
      validation at errors below 1e-6, DIVO provides a reproducible,
      test-driven reference implementation of the FAST-LIVO2 algorithm
      family for depth-camera platforms.
    ]
  ]
]

#v(8pt)
#par[#text(weight: "bold")[Index Terms] — depth-inertial-visual odometry, error-state
iterated Kalman filter, adaptive voxel map, direct photometric alignment,
tightly-coupled state estimation, Rust, simulation]

#v(12pt)

// ===========================================================================
// I. Introduction
// ===========================================================================
#include "sections/01-introduction.typ"

// ===========================================================================
// II. Related Works
// ===========================================================================
#include "sections/02-related.typ"

// ===========================================================================
// III. System Overview
// ===========================================================================
#include "sections/03-system.typ"

// ===========================================================================
// IV. ESIKF with Sequential State Update
// ===========================================================================
#include "sections/04-esikf.typ"

// ===========================================================================
// V. Local Mapping
// ===========================================================================
#include "sections/05-mapping.typ"

// ===========================================================================
// VI. Depth Measurement Model
// ===========================================================================
#include "sections/06-depth.typ"

// ===========================================================================
// VII. Visual Measurement Model
// ===========================================================================
#include "sections/07-visual.typ"

// ===========================================================================
// VIII. Experiments
// ===========================================================================
#include "sections/08-experiments.typ"

// ===========================================================================
// IX. Conclusion
// ===========================================================================
#include "sections/09-conclusion.typ"

// ===========================================================================
// References
// ===========================================================================
#set heading(numbering: none)
#heading(level: 1)[References]
#bibliography("refs.bib", title: none, style: "ieee")
