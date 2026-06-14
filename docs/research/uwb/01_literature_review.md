# UWB → EKF Fusion — Literature Review (Tightly- vs Loosely-Coupled)

> **What this file covers:** the *theory and published evidence* for fusing UWB (ultra-wideband)
> tag-to-anchor ranges into an IMU-driven Extended / error-state Kalman filter of the PX4 EKF2
> kind. It is a **literature review only** — measurement models, observability, sequential-vs-batch
> fusion, and the tight-vs-loose architecture trade-off, with cited sources. It does **not**
> prescribe code; for the implementation plan see [`03_uwb_ekf2_implementation.md`](03_v1_implementation_plan.md),
> and for EKF2 ESKF internals see [`02f_ekf2_unified.md`](../control_stack/02f_ekf2_unified.md).
>
> **Scope assumed** (matches the simulation world `worlds/uwb.sdf`): clear line-of-sight, a small
> set (≈4) of **fixed anchors at known surveyed positions**, GPS-denied. NLOS mitigation, anchor
> self-calibration, and clock-sync are explicitly **out of scope** here.

---

## Table of Contents

- [0. One-paragraph verdict](#0-one-paragraph-verdict)
- [1. Notation](#1-notation)
- [2. The two architectures](#2-the-two-architectures)
  - [2.1 Tightly-coupled (raw range as measurement)](#21-tightly-coupled-raw-range-as-measurement)
  - [2.2 Loosely-coupled (trilateration → position fix)](#22-loosely-coupled-trilateration--position-fix)
- [3. Comparison table](#3-comparison-table)
- [4. Observability — does range-only aiding make the state observable?](#4-observability--does-range-only-aiding-make-the-state-observable)
- [5. Sequential vs batch fusion of simultaneous ranges](#5-sequential-vs-batch-fusion-of-simultaneous-ranges)
- [6. Mapping onto the PX4 EKF2 ESKF position-aiding pattern](#6-mapping-onto-the-px4-ekf2-eskf-position-aiding-pattern)
- [7. What the evidence does NOT support](#7-what-the-evidence-does-not-support)
- [8. Open questions](#8-open-questions)
- [9. Annotated bibliography](#9-annotated-bibliography)

---

## 0. One-paragraph verdict

For an IMU-driven error-state EKF like PX4 EKF2, under ideal LOS with ~4 surveyed anchors, the
literature favors **tightly-coupled** fusion: feed each raw range $z = \lVert p - a\rVert + \eta$
directly into the filter as a scalar nonlinear measurement, with the position-block Jacobian equal
to the **unit line-of-sight direction** $H_{\text{pos}} = (p-a)/\lVert p-a\rVert$. This preserves all
range information, avoids the error magnification of an intermediate position fix, stays usable with
**fewer than the 4 anchors** trilateration needs, and is naturally applied as cheap **sequential
scalar updates** with per-range innovation gating — exactly the update pattern EKF2 already uses for
GPS/EV aiding. **Loosely-coupled** fusion (raw ranges → least-squares multilateration → a 3-D fix fed
in like GPS) is simpler and modular but needs ≥4 well-distributed anchors, discards range geometry,
and is qualitatively less accurate. The strongest single reference is Mueller, Hamer & D'Andrea
(ICRA 2015) [[1]](#ref1).

---

## 1. Notation

Consistent with [`../quadcopter_control_math.md`](../quadcopter_control_math.md) §0 and
[`02f_ekf2_unified.md`](../control_stack/02f_ekf2_unified.md):

| Symbol | Meaning |
|---|---|
| $p \in \mathbb{R}^3$ | UAV (tag) position, NED world frame $\{W\}$ |
| $a_i \in \mathbb{R}^3$ | known position of anchor $i$, NED |
| $z_i$ | measured range tag→anchor $i$ (m) |
| $h_i(x)=\lVert p - a_i\rVert$ | range measurement model |
| $e_i = (p-a_i)/\lVert p-a_i\rVert$ | unit LOS direction (tag relative to anchor) |
| $H_i$ | measurement Jacobian $\partial h_i/\partial x$ |
| $R$ | scalar measurement-noise variance |
| ESKF | error-state KF (PX4 EKF2 family); $\boxplus$ error-state injection |

---

## 2. The two architectures

### 2.1 Tightly-coupled (raw range as measurement)

Each anchor range is fed in **directly**, with no intermediate position solve:

$$z_i = \lVert p - a_i \rVert + \eta_i, \qquad \eta_i \sim \mathcal{N}(0, R).$$

Linearizing about the current estimate, the **position block** of the Jacobian is the unit LOS
vector:

$$H_{i,\text{pos}} = \frac{\partial \lVert p - a_i\rVert}{\partial p} = \frac{(p - a_i)^{\top}}{\lVert p - a_i\rVert} = e_i^{\top}.$$

This is the exact $H=(p-a)/\lVert p-a\rVert$ in the brief. Two corroborating forms in the literature:

- **15-state ESKF** (MDPI *Sensors* 2025 [[2]](#ref2)): $r_i = H_i\,\delta x + n_i$ with
  $H_i = [\,e_i^{\top},\; 0,\; e_i^{\top} R_{bw}[t_{ub}]_\times,\; 0\,]$, where the extra term
  $e_i^{\top} R_{bw}[t_{ub}]_\times$ is the **antenna lever-arm** coupling — it ties range to
  *attitude*, which is what makes orientation observable in the tightly-coupled INS case
  (§4). The paper states verbatim that it "directly use[s] raw ranging data as the filter's
  measurement input rather than first computing positions… avoid[ing] information loss and error
  magnification caused by intermediate processing." *(claim verified 3-0)*
- **Squared-range form** (FEJ-VIRO, arXiv:2207.08214 [[3]](#ref3)): from $d^2=\lVert p-a\rVert^2$ the
  gradient is $2(p-a)^{\top}$ — **collinear** with $e_i$ (same direction, scale $2\lVert p-a\rVert$).
  Either parameterization gives the same measurement direction.

The same raw-range model appears in tightly-coupled UWB/IMU work (Sensors 2023, PMC10346922
[[10]](#ref10)) and in Mueller et al. [[1]](#ref1), which model $z_{uwb,i}=\lVert p_{uwb,i}-x\rVert+\eta$
and linearize it straight into the EKF. *(verified 3-0)*

### 2.2 Loosely-coupled (trilateration → position fix)

Raw ranges are first collapsed into a 3-D position by **least-squares trilateration /
multilateration**, then that fix is fused like a GPS or external-vision position:

$$\{z_i\}_{i=1}^{n} \;\xrightarrow{\text{LS multilateration}}\; \hat{p} \;\xrightarrow{\text{EKF}}\; \text{position aiding (GPS-like)}.$$

Representative work: an adaptive-KF loosely-coupled INS/UWB indoor system (*Measurement* 2025
[[4]](#ref4)); Cao et al. (arXiv:2005.10648 [[5]](#ref5)) self-describe "a loosely coupled tracking
algorithm fusing IMU, UWB, and the proposed speed estimation"; and the common drone integration
pattern of computing $(x,y,z)$ by LS multilateration and emitting **NMEA into the flight controller
as a GPS substitute** [[6]](#ref6). *(verified 3-0)* This is modular — the filter sees a position and
needs no UWB-specific math — but it **requires ≥4 well-distributed anchors** for a 3-D fix and throws
away the underlying range geometry before filtering.

---

## 3. Comparison table

| Dimension | Tightly-coupled (raw range) | Loosely-coupled (position fix) |
|---|---|---|
| **Measurement** | scalar $z_i=\lVert p-a_i\rVert$ per anchor | 3-D position $\hat p$ from LS multilateration |
| **Jacobian** | nonlinear; $H_{\text{pos}}=e_i^{\top}$ (+ lever-arm term) | identity position block (like GPS) |
| **Min anchors for a useful update** | **1–3** usable (with IMU); single anchor possible | **≥4** for a 3-D fix |
| **Behavior with <4 anchors** | degrades gracefully, still informative | trilateration unsolvable / ill-posed |
| **Information retained** | full range geometry | geometry discarded before filter |
| **Accuracy (qualitative)** | ≥ loose; advantage grows as anchors drop below 4 [[2]](#ref2) | baseline; less accurate [[2]](#ref2) |
| **Compute** | cheap scalar updates, scalar $S$ inversion [[1]](#ref1) | one position update + an external LS solve each epoch |
| **Outlier handling** | per-range Mahalanobis gate [[1]](#ref1)[[7]](#ref7) | gating on the fused position only |
| **Integration effort** | UWB-specific $h(x)$, $H$ in the filter | reuses existing GPS/EV position-aiding path |
| **Attitude/yaw coupling** | yes, via lever-arm → aids observability [[2]](#ref2) | none (position only) |
| **Preferred when** | few anchors, max accuracy, GPS-denied | simplicity, modularity, vendor already outputs a fix |

> **Honesty note:** the *direction* (tight ≥ loose) is verified 3-0, but a specific numeric delta
> (a "0.097 m vs 0.140 m, ~30.7%" square-trajectory RMSE) was **refuted 0-3** — see §7. Treat the
> accuracy advantage as qualitative, not quantified.

---

## 4. Observability — does range-only aiding make the state observable?

The central question for a GPS-denied EKF2: can ranges + IMU actually pin down position (and
orientation)?

1. **With ≥4 well-distributed *known* anchors, position is geometrically observable even at
   standstill** — the range set is trilateration-equivalent. With *known* surveyed anchors the
   4-DOF gauge freedom (global translation + yaw-about-gravity) that plagues *unknown*-anchor
   systems is **fixed**, so global position is observable and drift-free [[3]](#ref3). *(verified 3-0,
   with scoping)*

2. **Range-only + IMU can render the full rigid-body state observable, including yaw about the
   thrust axis** — but orientation observability comes from the vehicle's *corrective motion*, it is
   not static. Mueller et al. [[1]](#ref1): "the remaining quadrocopter states, including the yaw
   orientation, are rendered observable by fusing ultra-wideband range measurements," and "the
   quadrocopter's corrective motions when holding a constant position set point are sufficient for
   observing… orientation about its thrust axis." This relies on an extended aerodynamic/airspeed
   model under a **no-wind assumption** (see caveat in §7). *(verified 3-0)*

3. **Observability of position/velocity from range-only aiding is conditional on a
   persistency-of-excitation (PE) motion condition.** A cascaded single-range observer
   (arXiv:2512.06198 [[8]](#ref8)) reconstructs full state from IMU + one body-frame vector + one
   range to a fixed anchor, but **only under sufficiently rich, non-rectilinear motion**
   (its Lemma 4 requires $\int \phi\phi^{\top} \succeq \mu I$ over a window). Corroborated by Batista
   et al. (single-range observability, *Automatica* [[9]](#ref9)). The PE requirement is most acute
   for single/under-constrained geometry; ≥4 anchors relax it (point 1). *(verified 3-0)*

**Takeaway for the sim world (4 known anchors, LOS):** position is observable from geometry alone;
yaw observability via tight coupling is *conditional on motion* — a hovering, perfectly still drone
may have weakly observable heading, so in practice EKF2 still benefits from its existing heading aid
(mag/GPS-yaw/EV-yaw). See open question §8.

---

## 5. Sequential vs batch fusion of simultaneous ranges

When several anchor ranges arrive in one epoch, two options exist: stack them into one vector update
(**batch**), or apply them one at a time (**sequential** scalar updates).

The literature and EKF2 practice both favor **sequential scalar updates**. Mueller et al.
[[1]](#ref1): the vehicle "requests a range to an anchor, starting at the first anchor and proceeding
sequentially… Because the measurements are scalar, the resulting innovation covariance will also be
scalar, and may thus be inverted at low computational cost… A measurement with a distance larger
than some given threshold may thus be rejected as an outlier." *(verified 3-0)*

Advantages, all relevant to EKF2:

- **No matrix inversion** — each $S_i = H_i P H_i^{\top} + R$ is a scalar.
- **Per-range gating** — each anchor gets its own Mahalanobis/innovation test; one bad anchor is
  rejected without poisoning the others.
- **Re-linearization between updates** — re-evaluating $H_i$ at the freshly-updated state after each
  scalar fuse is statistically sound when per-anchor noises are uncorrelated (independent two-way
  ranging), and is exactly how EKF2 fuses its scalar aiding sources.

(Whether sequential vs batch produces *materially* different consistency in the ESKF is listed as an
open question, §8.)

---

## 6. Mapping onto the PX4 EKF2 ESKF position-aiding pattern

Both architectures slot into EKF2's existing position-aiding structure
([`02f_ekf2_unified.md`](../control_stack/02f_ekf2_unified.md), PX4 ECL-EKF tuning guide [[11]](#ref11)):

- **Loosely-coupled** is the *minimal* path: a trilaterated fix is just another position
  observation, fused like GPS/EV — no new measurement model in the filter core. This is why vendor
  "UWB-as-GPS" integrations exist.
- **Tightly-coupled** requires adding a UWB range aiding source: a nonlinear $h(x)=\lVert p-a\rVert$
  with $H_{\text{pos}}=e_i^{\top}$, fused as **scalar** updates — the same shape as EKF2's existing
  scalar aiding fusions, so it composes with the ESKF $\boxplus$ injection and Joseph-form covariance
  update without architectural changes. The IMU-driven EKF + exteroceptive aiding pattern is exactly
  the analog used by Benini et al. (IMU prediction + UWB 802.15.4a + vision in one EKF, ~10 cm indoor
  [[12]](#ref12)) and You & Li (UWB/IMU, UKF chosen to avoid dropping higher-order terms, shown to
  "suppress the error accumulation of the IMU" [[13]](#ref13)). The architecture is **filter-agnostic**
  (EKF/ESKF/UKF) — its conclusions transfer to EKF2. *(verified 3-0)*

A practical note from tightly-coupled UWB/INS work (Li et al., *Adv. Space Res.* 2016 [[7]](#ref7)):
raw-observation fusion pairs naturally with robust outlier handling (windowed Mahalanobis-distance
gross-error rejection) — mirroring EKF2's innovation gates.

---

## 7. What the evidence does NOT support

Adversarial verification **killed** three plausible-sounding claims (each refuted 0-3). Recorded here
so they are not repeated as fact:

1. **No reliable numeric tight-vs-loose accuracy delta.** The specific "square-trajectory RMSE
   0.0972 m (tight) vs 0.1402 m (loose), ≈30.7% reduction" did not survive verification. Only the
   qualitative direction (tight ≥ loose) is supported.
2. **"Loose coupling fails specifically because it assumes weak translational–rotational coupling
   that breaks in aggressive maneuvers"** — refuted as a stated mechanism; not a sound general
   explanation.
3. **"Range-only UWB needs velocity to be observable; static single-anchor is unobservable, fixed by
   UWB-speed + IMU"** — refuted; observability conditions are governed by anchor geometry and the PE
   motion condition (§4), not a velocity prerequisite.

**Additional caveats:**

- Mueller et al.'s full-state observability uses an **extended aerodynamic model under no wind**; a
  bare 9-state range+IMU model is not automatically fully observable without that model *or*
  sufficient anchor geometry/motion.
- The 4-unobservable-directions / First-Estimates-Jacobian consistency results (FEJ-VIRO
  [[3]](#ref3), CVIRO [[14]](#ref14)) apply to **unknown** anchors. With the **known surveyed
  anchors** assumed here, that gauge freedom is fixed — those consistency concerns are largely moot
  unless anchor positions are estimated online.
- The single-range full-state result [[8]](#ref8) is an **arXiv preprint** (peer-reviewed
  corroboration: Batista et al. [[9]](#ref9)).
- The loosely-coupled drone exemplar [[6]](#ref6) is a **non-peer-reviewed GitHub repo**, cited only
  to illustrate the position-fix-first pattern, not as an accuracy authority.

---

## 8. Open questions

1. **Quantitative gap.** What is the *actual* RMSE difference between tight and loose in the ideal
   4-known-anchor LOS quadrotor case? No reliable number survived verification.
2. **Native EKF2 range aiding.** Is there a published implementation fusing raw UWB ranges as a
   *native* EKF2 aiding source (like GPS/EV), or do existing PX4 integrations feed a trilaterated
   position as a GPS/EV substitute?
3. **Yaw conditioning at hover.** With exactly 4 anchors and typical hover/corrective motion, is the
   lever-arm/attitude coupling strong enough to make yaw *well-conditioned*, or is a magnetometer /
   heading aid still required?
4. **Sequential vs batch in the ESKF.** Does sequential scalar fusion (re-linearized per update)
   differ materially in consistency/accuracy from a single batched vector update, and which does
   EKF2's update scheduling effectively realize?

---

## 9. Annotated bibliography

Quality: all entries below are peer-reviewed unless marked. Verified via 3-vote adversarial check.

<a id="ref1"></a>**[1] Mueller, Hamer & D'Andrea (2015)** — *Fusing Ultra-wideband Range Measurements
with Accelerometers and Rate Gyroscopes for Quadrocopter State Estimation.* IEEE ICRA 2015
(ETH/Berkeley HiPeRLab). **Primary reference.** Raw-range EKF, sequential scalar updates, yaw
observability via corrective motion (no-wind aerodynamic model).
https://hiperlab.berkeley.edu/wp-content/uploads/2018/05/2015_FusingUltra-widebandRangeMeasurementsWithAccelerometersAndRateGyroscopesForQuadrocopterStateEstimation.pdf

<a id="ref2"></a>**[2] Zhao, Deng, Hu, Su, Lou & Liu (2025)** — *An Indoor UAV Localization Framework
with ESKF Tightly-Coupled Fusion and Multi-Epoch UWB Outlier Rejection.* Sensors (Basel) 25(24):7673.
DOI 10.3390/s25247673. Tightly-coupled UWB/IMU ESKF; explicit raw-range-as-input statement; Jacobian
$H_i=[e_i^{\top},0,e_i^{\top}R_{bw}[t_{ub}]_\times,0]$ with antenna lever-arm.
https://pmc.ncbi.nlm.nih.gov/articles/PMC12737027/

<a id="ref3"></a>**[3] Jia, Jiao, Zhang, Xiong & Wang (2022)** — *FEJ-VIRO: A Consistent First-Estimate
Jacobian Visual-Inertial-Ranging Odometry.* IEEE/RSJ IROS 2022, arXiv:2207.08214. Squared-range
Jacobian $2(p-a)^{\top}$; proves 4 unobservable directions for **unknown** anchors and the
spurious-yaw-observability inconsistency that FEJ fixes. https://arxiv.org/abs/2207.08214

<a id="ref4"></a>**[4] *Measurement* (2025), S0263224124022978** — *An adaptive Kalman filter loosely
coupled indoor fusion positioning system based on INS and UWB.* Canonical loosely-coupled INS/UWB.
https://www.sciencedirect.com/science/article/abs/pii/S0263224124022978

<a id="ref5"></a>**[5] Cao et al. (arXiv:2005.10648)** — loosely-coupled IMU + UWB + speed estimation
(ground robot; transfers partially). https://arxiv.org/pdf/2005.10648

<a id="ref6"></a>**[6] UWB-Drone-Positioning (GitHub, *not peer-reviewed*)** — LS multilateration →
$(x,y,z)$ → NMEA into the flight controller. Illustrates the position-fix-first / GPS-emulation
pattern. https://github.com/Mertcagliyan/UWB-Drone-Positioning

<a id="ref7"></a>**[7] Li et al. (2016)** — *Advances in Space Research*, S0273117716303982.
Tightly-coupled GPS/UWB/MEMS-IMU with an improved robust KF (windowed Mahalanobis gross-error
rejection). https://www.sciencedirect.com/science/article/abs/pii/S0273117716303982

<a id="ref8"></a>**[8] Cascaded single-range observer (arXiv:2512.06198, *preprint*)** — full-state
reconstruction from IMU + one vector + one range; PE motion condition (Lemma 4).
https://arxiv.org/html/2512.06198v1

<a id="ref9"></a>**[9] Batista et al.** — single-range observability, *Automatica*,
S0167691111001174. Peer-reviewed corroboration of [8].
https://www.sciencedirect.com/science/article/abs/pii/S0167691111001174

<a id="ref10"></a>**[10] *Sensors* (2023), PMC10346922** — tightly-coupled UWB/IMU; identical
raw-range model. https://pmc.ncbi.nlm.nih.gov/articles/PMC10346922/

<a id="ref11"></a>**[11] PX4 ECL-EKF tuning guide** — EKF2 aiding-source / position-fusion reference.
https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf

<a id="ref12"></a>**[12] Benini et al. (2013)** — *J. Intelligent & Robotic Systems*,
10.1007/s10846-012-9742-1. IMU (prediction) + UWB 802.15.4a + vision in a single EKF for indoor
mini-UAV (~10 cm); direct PX4-EKF2 analog. https://link.springer.com/article/10.1007/s10846-012-9742-1

<a id="ref13"></a>**[13] You & Li (2020)** — *IEEE Access*. UWB/IMU fusion via **UKF** (avoids dropping
higher-order linearization terms); suppresses IMU error accumulation.
https://ieeexplore.ieee.org/document/9055018/

<a id="ref14"></a>**[14] CVIRO (arXiv:2508.10867)** — consistent visual-inertial-ranging; corroborates
the unknown-anchor consistency analysis of [3]. https://arxiv.org/pdf/2508.10867

**[15] Sang et al. (2019)** — IEEE WPNC, doc 8970249. Benchmarks five true-range UWB methods
(geometric trilateration, closed-form LS multilateration, iterative Taylor-series, EKF, UKF) — places
snapshot loosely-coupled methods alongside recursive filters (UWB-only, no IMU).
https://ieeexplore.ieee.org/document/8970249/

---

*Generated from a verified multi-source deep-research pass (19 sources → 88 claims → 25
adversarially verified → 22 confirmed, 3 refuted). Refuted claims are documented in §7 rather than
silently dropped. Scope per request: ideal LOS, ~4 known anchors, fusion methodology only.*
