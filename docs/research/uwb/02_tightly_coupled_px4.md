# UWB Tightly-Coupled Fusion — From Papers to a PX4 EKF2 Integration

> **What this file covers:** the chosen approach — **tightly-coupled** range fusion — argued
> *from the literature* and then carried *to a concrete PX4 EKF2 integration path*. Every design
> decision is backed by a cited paper (see [`00_uwb_ekf2_literature_review.md`](01_literature_review.md)
> for the full review) and grounded in the *actual* EKF2 source tree of this repo.
>
> **Read order:** [`00_uwb_ekf2_literature_review.md`](01_literature_review.md) (WHY / theory)
> → this file (approach + PX4 mapping) → [`02f_ekf2_unified.md`](../control_stack/02f_ekf2_unified.md) (ESKF internals).
>
> **Scope:** ideal LOS, ~4 **fixed anchors at known surveyed positions** (matches `worlds/uwb.sdf`),
> GPS-denied. NLOS / anchor self-survey / clock-sync are out of scope.

This file is **self-contained**: the papers the approach rests on are named in [§0](#0-the-papers-this-approach-is-built-on)
and fully cited in [§10 References](#10-references). `[n]` markers resolve to §10 in this file (and match
the same numbers in [`00_uwb_ekf2_literature_review.md`](01_literature_review.md#9-annotated-bibliography)).

---

## Table of Contents

- [0. The papers this approach is built on](#0-the-papers-this-approach-is-built-on)
- [1. Why tightly-coupled — the evidence](#1-why-tightly-coupled--the-evidence)
- [2. The measurement model (paper-grounded)](#2-the-measurement-model-paper-grounded)
- [3. Observability in *our* setting](#3-observability-in-our-setting)
- [4. Sequential scalar fusion](#4-sequential-scalar-fusion)
- [5. Paper → PX4: how this lands on EKF2's real aiding pattern](#5-paper--px4-how-this-lands-on-ekf2s-real-aiding-pattern)
- [6. The end-to-end data path (sim and hardware)](#6-the-end-to-end-data-path-sim-and-hardware)
- [7. Practical gotchas the papers don't mention](#7-practical-gotchas-the-papers-dont-mention)
- [8. Staged integration plan](#8-staged-integration-plan)
- [9. Verification against theory](#9-verification-against-theory)
- [10. References](#10-references)

---

## 0. The papers this approach is built on

The tightly-coupled choice is not invented here — it follows these peer-reviewed sources. Full
citations in [§10](#10-references).

| # | Paper | Role in this approach |
|---|---|---|
| **[1]** | **Mueller, Hamer & D'Andrea**, *Fusing UWB range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation*, **IEEE ICRA 2015** | **Backbone.** Raw range $z=\lVert p-a\rVert$ fed straight into an accel/gyro EKF on a quadrotor; sequential scalar updates; yaw observability from corrective motion. |
| **[2]** | **Zhao et al.**, *An Indoor UAV Localization Framework with ESKF Tightly-Coupled Fusion…*, **Sensors 2025**, 25(24):7673 | Modern **ESKF** form; exact raw-range Jacobian with antenna lever-arm; explicit "raw ranges avoid information loss / error magnification". |
| **[3]** | **Jia et al.**, *FEJ-VIRO: A Consistent First-Estimate Jacobian Visual-Inertial-Ranging Odometry*, **IEEE/RSJ IROS 2022** | Squared-range Jacobian $2(p-a)$; observability & the unknown-anchor gauge-freedom analysis (why known anchors matter). |
| **[12]** | **Benini, Mancini & Longhi**, *An IMU/UWB/Vision-based EKF for Mini-UAV Localization…*, **J. Intell. Robot. Syst. 2013**, 70:461–476 | Closest architectural analog to EKF2: IMU in prediction, UWB as exteroceptive aiding, ~10 cm indoor. |
| **[7]** | **Li et al.**, *GPS/UWB/MEMS-IMU tightly coupled navigation with improved robust Kalman filter*, **Adv. Space Res. 2016**, 58(11):2424–2434 | Raw-observation tight coupling + windowed-Mahalanobis outlier rejection (mirrors EKF2 innovation gates). |

> The single most load-bearing reference is **Mueller, Hamer & D'Andrea (ICRA 2015) [1]** — it is the
> canonical demonstration that an IMU-driven EKF can fly a quadrotor on raw UWB ranges alone, which is
> exactly the EKF2 case. Everything else refines or modernizes it.

---

## 1. Why tightly-coupled — the evidence

The decision to fuse **raw ranges** (not a trilaterated position) is not a preference; it is what the
peer-reviewed literature converges on for an IMU-driven filter with few anchors.

| Reason | Evidence (paper named inline) |
|---|---|
| **No information loss / error magnification** from an intermediate position solve | **Zhao et al., Sensors 2025** [[2]](#ref2), verbatim: "we directly use raw ranging data… rather than first computing positions… avoids information loss and error magnification caused by intermediate processing." |
| **Usable with <4 anchors** (trilateration needs ≥4 for a 3-D fix) | **Mueller, Hamer & D'Andrea, ICRA 2015** [[1]](#ref1) fuse ranges one anchor at a time; loose coupling is unsolvable below 4 anchors. |
| **Cheap scalar updates + per-range outlier gating** | **Mueller et al., ICRA 2015** [[1]](#ref1): scalar innovation covariance "may thus be inverted at low computational cost… rejected as an outlier." |
| **Couples range to attitude** (lever-arm), aiding yaw observability | **Zhao et al., Sensors 2025** [[2]](#ref2): the ESKF Jacobian carries the antenna lever-arm term $e_i^\top R_{bw}[t_{ub}]_\times$. |
| **Bounds IMU dead-reckoning drift** indoors | **Benini, Mancini & Longhi, JIRS 2013** [[12]](#ref12): IMU-prediction EKF + UWB aiding, ~10 cm indoor. |

**Honest caveat (from the review §7):** the *qualitative* "tight ≥ loose" direction is verified, but a
specific numeric RMSE advantage was **refuted** — so we adopt tight coupling for its structural
benefits (few-anchor robustness, full geometry, native scalar fusion), not a promised accuracy number.

---

## 2. The measurement model (paper-grounded)

For anchor $i$ at known NED position $a_i$ and UAV position $p$:

$$z_i = \lVert p - a_i\rVert + \eta_i, \qquad \eta_i\sim\mathcal N(0,R).$$

Linearized, the **position block** of the Jacobian is the unit line-of-sight direction:

$$H_{i,\text{pos}} = \frac{(p-a_i)^\top}{\lVert p-a_i\rVert} = e_i^\top.$$

This is the exact form in the brief and in Mueller et al. [[1]](#ref1).
The ESKF treatment [[2]](#ref2) adds the antenna lever-arm:

$$H_i = \big[\, \underbrace{e_i^\top}_{\text{pos}},\ \underbrace{0}_{\text{vel}},\ \underbrace{e_i^\top R_{bw}[t_{ub}]_\times}_{\text{attitude (lever-arm)}},\ 0,\ \dots \big].$$

The squared-range parameterization [[3]](#ref3) gives the collinear
gradient $2(p-a_i)^\top$ — same direction. **For a first PX4 integration the lever-arm term can be
dropped** (assume antenna at body origin), leaving only the 3-entry position block — this is the
minimal correct model.

---

## 3. Observability in *our* setting

What the theory says, specialized to **4 known anchors, LOS** (review §4):

- **Position is geometrically observable even at standstill** — 4 well-distributed known anchors are
  trilateration-equivalent; the global-translation + yaw gauge freedom that affects *unknown*-anchor
  systems [[3]](#ref3) is **fixed** by the survey. → drift-free global
  position is expected.
- **Yaw observability via tight coupling is conditional on motion** (a persistency-of-excitation
  condition [[8]](#ref8)[[9]](#ref9));
  a perfectly still hover may leave heading weakly observable.
  → **Keep EKF2's existing heading aid** (mag / GPS-yaw / EV-yaw) at first; treat UWB-driven yaw as a
  bonus, not a dependency.

This directly shapes the integration: **enable UWB as a position aid; do not initially rely on it for
heading.**

---

## 4. Sequential scalar fusion

When all 4 ranges arrive in one epoch, fuse them **one at a time as scalar updates**, not as one
stacked vector — Mueller et al. [[1]](#ref1). Benefits (all native to
EKF2): scalar $S_i=H_iPH_i^\top+R$ (no matrix inverse), independent per-anchor innovation gate, and
re-linearization of $H_i$ at the updated state between anchors (valid because two-way-ranging noises
are per-pair independent). **This is exactly how EKF2 already fuses scalar aiding sources** — so it is
a natural fit, not an adaptation (see §5).

---

## 5. Paper → PX4: how this lands on EKF2's real aiding pattern

The key realization: the paper model maps **one-to-one** onto a pattern that *already exists* in this
repo's EKF2. PX4 has two scalar-fusion entry points:

| Function (verified) | Jacobian shape | Fits UWB? |
|---|---|---|
| `fuseDirectStateMeasurement(innov, innov_var, R, state_index)` — [`ekf_helper.cpp:1029`](../../../src/modules/ekf2/EKF/ekf_helper.cpp#L1029) | basis vector (single state) | ❌ — range $H$ has 3 nonzero entries |
| `measurementUpdate(K, H, R, innov)` — [`ekf_helper.cpp:1089`](../../../src/modules/ekf2/EKF/ekf_helper.cpp#L1089) | arbitrary `VectorState` | ✅ — required for UWB |

Because $H_{i,\text{pos}}=e_i^\top$ has **three** simultaneous nonzero entries (one per NED axis), UWB
**must** use `measurementUpdate` with an explicitly built `H`, not `fuseDirectStateMeasurement`.

**There is already a real template for exactly this shape:** GNSS yaw fusion, a *nonlinear scalar*
aiding source, builds an explicit `VectorState H` and calls `measurementUpdate` after gating with
`updateAidSourceStatus`:

```text
src/modules/ekf2/EKF/aid_sources/gnss/gnss_yaw_control.cpp
  :144   VectorState H;                 // explicit Jacobian
  :207   measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);
```
([gnss_yaw_control.cpp](../../../src/modules/ekf2/EKF/aid_sources/gnss/gnss_yaw_control.cpp#L144))

UWB range fusion is structurally the same — only $h(x)$ and $H$ differ. Concretely:

- **Position error-state index is 6** — `State::pos {6, 3}` in
  [`state.h:48`](../../../src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h#L48). So set
  `H(State::pos.idx + 0..2) = e_i` and leave the rest zero.
- **Per-anchor gating** via `updateAidSourceStatus(...)` — [`ekf_helper.cpp:1155`](../../../src/modules/ekf2/EKF/ekf_helper.cpp#L1155) — fills an
  `estimator_aid_source1d_s` ([`msg/EstimatorAidSource1d.msg`](../../../msg/EstimatorAidSource1d.msg))
  with innovation / test-ratio / rejection, exactly like every other source.
- **Folder + flag convention** is one directory per source under
  [`aid_sources/`](../../../src/modules/ekf2/EKF/aid_sources/) (`gnss/`, `optical_flow/`,
  `range_finder/`, …) and a `menuconfig EKF2_*` in [`Kconfig`](../../../src/modules/ekf2/Kconfig) →
  so a new aid is `aid_sources/uwb/` + `EKF2_UWB`.

**Conclusion:** tightly-coupled UWB is not a foreign body in EKF2 — it is a new scalar aiding source
that reuses `measurementUpdate` + `updateAidSourceStatus` + the `aid_sources/<name>/` + `EKF2_*`
pattern, with `gnss_yaw_control.cpp` as the closest existing analog.

---

## 6. The end-to-end data path (sim and hardware)

The "paper → reality" chain, reusing PX4's standard sensor→bridge→uORB→EKF2 architecture:

```
                     ┌─────────────── SIMULATION ───────────────┐
 gtec_uwb_plugin  ──►  /gtec/toa/ranging  ──►  gz_bridge          │
 (px4::msgs::Ranging,   (gz-transport)        rangingCallback()   │
  per-anchor range)                              │                │
                                                 ▼                │
                     ┌──────────────── HARDWARE ─────────────────┐│
 DWM3000 / SR150  ──► UART driver  ──────────────┤               ││
 (real ranges)        (publishes uORB)           │               ││
                                                 ▼               ││
                                   sensor_uwb (or sensor_uwb_range) uORB
                                                 │
                                                 ▼
                              EKF2::UpdateUwbSample()  →  _ekf.setUwbData()
                                                 │   (ring buffer, delay align)
                                                 ▼
                              Ekf::controlUwbFusion(imu_delayed)
                                                 │   pop per anchor, sequential
                                                 ▼
                              Ekf::fuseUwbRange()  →  measurementUpdate(K,H,R,innov)
                                                 │
                                                 ▼
                              vehicle_local_position  (+ estimator_aid_source per anchor)
```

Two clean properties:
1. **Sim and hardware converge** at the same uORB topic — EKF2 is agnostic to whether ranges came
   from `gtec_uwb_plugin` (via `gz_bridge`) or a real DWM3000 UART driver.
2. The only **new** filter code is `aid_sources/uwb/` + the wrapper subscription; everything upstream
   (the gz plugin, the bridge callback, the driver) just produces ranges.

> The bridge step (`/gtec/toa/ranging` → uORB) does **not exist yet** in `gz_bridge` — it is the first
> concrete coding task (see §8). The gz plugin already publishes correctly.

---

## 7. Practical gotchas the papers don't mention

Distilled from the EKF2 source (kept from earlier implementation notes — verify against current tree):

1. **The `_state.pos.zero()` trap.** After each fuse, EKF2 zeroes the position error state; the true
   NED position lives in the global-position object, recovered via the local-origin projection.
   **Never read the position error state to form the predicted range** — project the global position
   to local NED instead. Verify with `grep -n "_gpos\|_state.pos" src/modules/ekf2/EKF/ekf_helper.cpp`.
2. **Time-alignment uses `timestamp_sample`, not publish time.** EKF2 runs on the delayed IMU horizon;
   the UWB sample must carry the measurement time and be pushed into a ring buffer with the configured
   delay (`EKF2_UWB_DELAY`), then popped at the aligned horizon — same pattern as GPS.
3. **Singular Jacobian on top of an anchor.** If $\lVert p-a_i\rVert \to 0$, $e_i$ is undefined —
   guard with a minimum-range check before forming $H$.
4. **Innovation sign.** With EKF2's convention, use `innovation = measurement − predicted` ($z-h(x)$);
   confirm against the sign in `fuse()`/`measurementUpdate`.
5. **Anchor positions are parameters in EKF NED frame.** Store anchor $a_i$ as NED relative to the EKF
   origin (e.g. `EKF2_UWB_A{i}_{N,E,D}`); they must match the survey used in the world/field.

---

## 8. Staged integration plan

Ordered so each stage is independently testable (matches the data path in §6):

| Stage | Task | Done-when |
|---|---|---|
| **A. Bridge** | Add `rangingCallback` in `gz_bridge` → publish a UWB uORB topic (reuse `sensor_uwb` or add `sensor_uwb_range` with `timestamp_sample` + `range_variance`). | `listener sensor_uwb*` shows per-anchor ranges in SITL. |
| **B. Topic** | Decide topic: reuse `sensor_uwb` (exists, hardware-shaped) vs new `sensor_uwb_range` (EKF-shaped: adds `timestamp_sample`, `range_variance`, `anchor_id`). | msg registered; both sim + a stub driver can publish it. |
| **C. EKF aid** | `aid_sources/uwb/` with `fuseUwbRange()` (build `H`, `measurementUpdate`, `updateAidSourceStatus`), `controlUwbFusion()` sequential pop, `EKF2_UWB` Kconfig + params (anchors, noise, gate, delay). | `ekf2 status` shows UWB aiding; aid-source innovations sane. |
| **D. Validate** | Hover + square trajectory in `gz_f450-uwb_uwb`; compare `vehicle_local_position` to groundtruth with GPS off. | position converges from UWB alone; blocking one anchor still fuses 3. |

Stage A is the smallest, highest-value next step and unblocks everything downstream.

---

## 9. Verification against theory

Cross-checks that the implementation matches what the papers predict (review §4, §5):

```
□ With 4 anchors, position converges at standstill (geometric observability [1][3])
□ Sequential per-anchor fusion: each estimator_aid_source has its own test_ratio (sequential scalar [1])
□ Blocking 1 anchor → 3 remaining still fuse, position degrades gracefully (few-anchor robustness [2])
□ Yaw stays dependent on mag/heading aid at hover (conditional yaw observability [8][9])
□ Covariance P decreases on fuse, grows on predict-only; re-converges after UWB dropout
□ Innovations zero-mean in LOS sim (correct H direction e_i [1][2])
```

---

## 10. References

All citations below were verified against the publisher / arXiv records (June 2026). Numbering is
shared with [`00_uwb_ekf2_literature_review.md` §9](01_literature_review.md#9-annotated-bibliography).

<a id="ref1"></a>**[1]** M. W. Mueller, M. Hamer, R. D'Andrea. *Fusing ultra-wideband range measurements
with accelerometers and rate gyroscopes for quadrocopter state estimation.* IEEE International
Conference on Robotics and Automation (ICRA), Seattle, WA, 2015, pp. 1730–1736.
PDF: https://www.flyingmachinearena.ethz.ch/wp-content/publications/2015/mueICRA15.pdf —
**the load-bearing reference for this approach.**

<a id="ref2"></a>**[2]** J. Zhao, Z. Deng, E. Hu, W. Su, B. Lou, Y. Liu. *An Indoor UAV Localization
Framework with ESKF Tightly-Coupled Fusion and Multi-Epoch UWB Outlier Rejection.* Sensors (Basel),
25(24):7673, 2025. DOI: 10.3390/s25247673. https://pmc.ncbi.nlm.nih.gov/articles/PMC12737027/

<a id="ref3"></a>**[3]** S. Jia, Y. Jiao, Z. Zhang, R. Xiong, Y. Wang. *FEJ-VIRO: A Consistent
First-Estimate Jacobian Visual-Inertial-Ranging Odometry.* IEEE/RSJ International Conference on
Intelligent Robots and Systems (IROS), 2022. arXiv:2207.08214. https://arxiv.org/abs/2207.08214

<a id="ref7"></a>**[7]** Z. Li, G. Chang, J. Gao, J. Wang, A. Hernandez. *GPS/UWB/MEMS-IMU tightly
coupled navigation with improved robust Kalman filter.* Advances in Space Research, 58(11):2424–2434,
2016. https://www.sciencedirect.com/science/article/abs/pii/S0273117716303982

<a id="ref8"></a>**[8]** *Cascaded observer for full-state reconstruction from IMU + one vector + a
single range to a fixed anchor* (persistency-of-excitation condition). arXiv:2512.06198 (preprint).
https://arxiv.org/abs/2512.06198

<a id="ref9"></a>**[9]** P. Batista, C. Silvestre, P. Oliveira. *Single range/bearing observability*
results, Automatica (Systems & Control Letters family). https://www.sciencedirect.com/science/article/abs/pii/S0167691111001174

<a id="ref12"></a>**[12]** A. Benini, A. Mancini, S. Longhi. *An IMU/UWB/Vision-based Extended Kalman
Filter for Mini-UAV Localization in Indoor Environment using 802.15.4a Wireless Sensor Network.*
Journal of Intelligent & Robotic Systems, 70:461–476, 2013. DOI: 10.1007/s10846-012-9742-1.
https://link.springer.com/article/10.1007/s10846-012-9742-1

> Refs [4], [6], [13] (loosely-coupled exemplars, You & Li UKF) are cited only in the literature
> review; this file deliberately leans on the tightly-coupled sources above. Bibliographic metadata
> for [8]/[9] is from the deep-research pass and not yet re-fetched from publisher records — verify
> before formal citation.

---

*This document supersedes the earlier implementation-only guide. ESKF internals in
[`02f_ekf2_unified.md`](../control_stack/02f_ekf2_unified.md); the full tight-vs-loose review (with refuted claims) in
[`00_uwb_ekf2_literature_review.md`](01_literature_review.md). All PX4 file:line references
were verified against the current source tree on the `EKF-UWB-fusion` branch — re-verify after
upstream merges.*
