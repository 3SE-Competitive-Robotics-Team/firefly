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

#heading[Error-State Iterated Kalman Filter with Sequential State Update]

<sec:esikf>

#heading(level: 2)[Manifold State and Box-Plus / Box-Minus]

The DIVO state lives on the manifold

$ #xG = [#Rwi,\ #pwi,\ #vwi,\ #bg,\ #ba,\ #gvec,\ τ]^T ∈ M ≜ "SO"(3) × ℝ^16, $

where #Rwi is the rotation from the world frame to the IMU (virtual pinhole
camera) frame, #pwi and #vwi are the position and velocity of the IMU in the
world frame, #bg and #ba are the gyroscope and accelerometer biases, #gvec
is the gravity vector in the world frame, and τ is the inverse exposure
time. The rotation uses the Hamilton convention with *right-multiplication*
perturbations, consistent with the FAST-LIVO2 implementation
#cite(<lin2025fastlivo2>):

$ #xG ⊞ δ#xG = [#Rwi · Exp(δ θ), #pwi + δ p, #vwi + δ v, #bg + δ#bg, #ba + δ#ba, #gvec + δ#gvec, τ + δ τ]^T $ <eq:boxplus>

$ #xG#math.attach("", b: "a") ⊟ #xG#math.attach("", b: "b") = [Log(#Rwi#math.attach("", b: "b")^T · #Rwi#math.attach("", b: "a")), #pwi#math.attach("", b: "a") − #pwi#math.attach("", b: "b"), …]^T $ <eq:boxminus>

where Exp(·) and Log(·) denote the SO(3) exponential and logarithmic maps
#cite(<sola2017micro>). The 19×19 state covariance P is maintained
alongside the point on the manifold; the initial covariance is diagonal
with 0.01 on rotation/position blocks and 1e-5 on velocity, bias, gravity,
and exposure blocks, matching the reference implementation.

#heading(level: 2)[IMU Propagation]

<sec:prop>

Between consecutive IMU samples the state is propagated by the standard
zero-order-hold discretization (Eq. (1)–(2) of FAST-LIVO2):

$ #Rwi &← #Rwi · Exp((ω_m − #bg) Δ t) $ <eq:rotprop>

$ #pwi &← #pwi + #vwi Δ t + 1/2 (#Rwi (a_m − #ba) + #gvec) Δ t^2 $ <eq:posprop>

$ #vwi &← #vwi + (#Rwi (a_m − #ba) + #gvec) Δ t $ <eq:velprop>

with biases and gravity held constant and τ following a random walk. The
discrete-time error-state transition matrix #math.bold("F")#math.attach("", b: "x") and
process noise #math.bold("Q") follow the FAST-LIVO2 discretization: the
rotation block is Exp(−ωΔ t), the velocity block contains the
#math.op("−")#Rwi#math.op("⌊")a#math.op("×⌋")Δ t gravity/bias couplings, and
the accelerometer noise is rotated into the world frame as
#Rwi#math.op("·")#math.op("diag")#math.op("(")σ#math.attach("", b: "a")#math.op("²)")#Rwi#math.op("ᵀ")Δ t#math.op("²").

For depth-point motion compensation we additionally implement a *backward
propagation*: given the end-of-scan state, the pose at an intermediate
timestamp t is recovered by constant-acceleration reverse integration

$ #Rwi (t) = #Rwi (t_"end") · Exp(−ω #math.Delta t),  quad  #pwi (t) = #pwi (t_"end") − #vwi #math.Delta t + 1/2 #math.bold("a") #math.Delta t^2, $

with $ #math.Delta t = t_"end" − t $, which is the inverse of
Eq. #ref(<eq:rotprop>)–#ref(<eq:velprop>) under the same constant-acceleration
assumption.

#heading(level: 2)[Iterated Update with Sequential Measurement Processing]

<sec:sequpdate>

Let #xhat be the propagated prior with covariance #Phat, and let a
measurement be modeled as

$ #math.bold("z")^κ = h(#xhat^κ, 0) − #math.bold("y") ≈ #math.bold("H")^κ δ#xG^κ + #math.bold("v"),  quad  #math.bold("v") ~ 𝒩(0, #math.bold("R")), $ <eq:residual>

where h(·,·) is the measurement function, #math.bold("y") is the observation,
#math.bold("H")^κ is the Jacobian of h with respect to the error state, and
#math.bold("v") is the measurement noise. The iterated update solves for the
maximum-a-posteriori error state by iterating the gain

$ #math.bold("K")^κ = (#math.bold("H")^("κT") #math.bold("R")^{-1} #math.bold("H")^κ + #Phat^{-1})^{-1} #math.bold("H")^("κT") #math.bold("R")^{-1}, $ <eq:gain>

$ #xG^{κ+1} = #xG^κ ⊞ (−#math.bold("K")^κ #math.bold("z")^κ − (#math.bold("I") − #math.bold("K")^κ #math.bold("H")^κ)(#xG^κ ⊟ #xhat)), $ <eq:iter>

until the step $ #xG^{κ+1} ⊟ #xG^κ $ falls below a threshold ε
(1.5e-4 for depth, 1e-4 for visual) or the iteration budget (5) is
exhausted; divergence is detected by comparing the residual norm against
the prior residual norm with a safety factor of 1e6, in which case the
update is rejected. After convergence the covariance is updated by the
simplified Joseph-free form

$ #math.bold("P") = (#math.bold("I") − #math.bold("K")#math.bold("H")) #Phat, $ <eq:covupdate>

consistent with the reference implementation.

*Sequential processing.* Depth and visual measurements are processed one
batch after the other inside the same frame (Algorithm #ref(<alg:esikf>)):
the depth point-to-plane batch is applied first using the propagated prior;
the resulting posterior becomes the prior of the visual batch. Under
linearization this sequential application is equivalent to the joint update
(Algorithm 1 of FAST-LIVO2), and our unit test
`sequential_two_measurements_equals_joint` verifies this equivalence
numerically on the 19-dimensional manifold to 1e-6.

#figure(
  block[
    #text(weight: "bold")[Algorithm 2: Sequential ESIKF update (single batch)]
    #v(4pt)
    #set text(size: 8.8pt)
    #set par(leading: 0.35em)
    #par[
      #h(12pt)#text(weight: "bold")[Input:]#h(4pt) prior (#xhat, #Phat), measurement model h, noise #math.bold("R")
    ]
    #par[
      #h(12pt)#text(weight: "bold")[Output:]#h(4pt) posterior (#xG, #math.bold("P"))
    ]
    #par[1. #xG ← #xhat]
    #par[2. For κ = 0 … max-iter:]
    #par[  a. Compute #math.bold("z")^κ, #math.bold("H")^κ at #xG^κ (Eq. #ref(<eq:residual>))]
    #par[  b. #math.bold("K")^κ ← (#math.bold("H")^("κT")#math.bold("R")^{-1}#math.bold("H")^κ + #Phat^{-1})^{-1}#math.bold("H")^("κT")#math.bold("R")^{-1}  (Eq. #ref(<eq:gain>))]
    #par[  c. #xG^{κ+1} ← #xG^κ ⊞ (−#math.bold("K")^κ#math.bold("z")^κ − (#math.bold("I")−#math.bold("K")^κ#math.bold("H")^κ)(#xG^κ⊟#xhat))  (Eq. #ref(<eq:iter>))]
    #par[  d. If $ #xG^{κ+1}⊟#xG^κ < ε $: converged, break]
    #par[3. #math.bold("P") ← (#math.bold("I") − #math.bold("K")#math.bold("H"))#Phat  (Eq. #ref(<eq:covupdate>))]
  ],
  caption: [Iterated ESIKF update for one measurement batch (depth or visual),
  matching Algorithm 1 of FAST-LIVO2 #cite(<lin2025fastlivo2>).],
)<alg:esikf>
