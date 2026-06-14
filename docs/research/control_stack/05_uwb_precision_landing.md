# UWB Precision Landing in GPS-Degraded Zones — Scenario, Methodology & Test Plan

> **Application:** use UWB (anchors surveyed around the landing pad) to land a multirotor
> **accurately** when GPS has drifted during the mission. En-route the drone flies on GPS; on
> return it enters UWB range (≤ ~14 m of the pad) and UWB corrects the accumulated GPS error so it
> touches down on the pad. Built on the UWB→EKF2 fusion in
> [`01_uwb_tightly_coupled_px4.md`](01_uwb_tightly_coupled_px4.md) /
> [`04_uwb_ekf2_v2_native_plan.md`](04_uwb_ekf2_v2_native_plan.md).
>
> **Scope here:** Mode **UWB+GPS** only. Landing via standard **AUTO.LAND**. Everything scripted
> (MAVSDK) + analyzed with plots/metrics.

---

## 1. The demonstration (narrative)

- **Scenario A — GPS-only (baseline):** takeoff from pad → fly a multi-lap mission far from the pad
  for several minutes (GPS error accumulates) → return → LAND. The EKF believes it is over the pad,
  but it is offset by the GPS error → **lands off-pad** by ~the GPS drift.
- **Scenario B — GPS+UWB:** same mission, same GPS error; near the pad UWB injects pad-relative
  position → corrects the drift → **lands on the pad**.

Value proposition: **UWB precision landing in GPS-weakened areas.**

---

## 2. What the literature establishes (verified, cited)

Multi-source deep-research pass (peer-reviewed prioritized). Key findings:

- **Architecture = tightly-coupled raw-range fusion** (what we built). Closest landing study:
  **Ochoa-de-Eribe-Landaberea et al., *Sensors* 2022, 22(6):2347** — tightly-coupled EKF (8 anchors
  around a 2×2 m pad, dual tags + IMU), horizontal landing **RMSE 0.208 m vs 0.410 m** UWB-only
  (~50%), 99.95 % of errors < 1 m, mocap ground truth. [[1]](#ref1)
- **GPS degradation modeling:** a **slowly-varying first-order Gauss-Markov bias** (e.g. σ≈1.6 m,
  τ≈2 min) kept **separate** from white pseudorange noise — WVU ION GNSS+ 2014 [[3]](#ref3). This is
  exactly the f450 sim's `dynamic_bias` model. Real GNSS/INS-only multipath error reached ~5.26 m.
- **UWB "wins" near the pad via adaptive measurement noise:** UWB R fixed small (~0.1 m) while
  degraded-GNSS R is inflated 80–150× (DOP/UERE) → Kalman gain trusts UWB. eVTOL result: 0.56 m H
  RMSE, **89.4 % improvement** over 5.26 m GNSS/INS-only — Osaka & Tsujii, *Sensors* 2025,
  25(24):7419 [[2]](#ref2).
- **Metrics:** **CEP** (radius of the circle about the aim point containing 50 % of touchdowns,
  ≈ 1.1774·σ) + **3-D RMSE** vs the surveyed pad center, against mocap/sim ground truth. [[1]](#ref1)

**Honest caveats (from the research):**
- Dedicated UWB precision-**landing** experiments are scarce — only [[1]](#ref1) is a true
  touchdown-error study; the rest inform architecture/degradation, not the landing scenario. The
  loiter-then-handoff profile is **not cleanly demonstrated in any single paper** → our scenario (ii)
  is a defensible engineering synthesis, not a copy.
- **Innovation chi-squared gating is weak against slow GPS bias drift** (good at noise spikes). So
  the EKF's innovation gate alone will **not** cleanly correct accumulated drift → we need adaptive R
  / GPS down-weighting (scenario (ii)). This shapes the whole design.

---

## 3. The fusion-conflict problem (why scenario (ii) matters)

On return with, say, 1.5 m of accumulated GPS bias and pad-accurate UWB:
- GPS says "over the pad"; UWB says "1.5 m off."
- The EKF fuses both. As the bias built up **slowly**, neither innovation is a sharp spike, so the
  chi-squared gate does **not** reject GPS → the estimate sits **between** GPS and UWB → still lands
  partly off-pad.

Two designs, matching the user's request:
- **Scenario B-(i) — passive (tuning only):** keep both GPS+UWB; tune `EKF2_UWB_NOISE` small + gate.
  Expectation (per literature): UWB pulls the estimate **partway** toward the pad but slow GPS drift
  is only partially corrected. *Measures how far tuning alone gets us.*
- **Scenario B-(ii) — active GPS down-weight (recommended):** when UWB is active & trusted (in range,
  fusing), **inflate GNSS R or stop GNSS horizontal fusion** so UWB dominates near the pad
  (the adaptive-R approach of [[2]](#ref2)). Expectation: estimate snaps to pad → cm–dm landing.
  *This is the new EKF feature (a later phase).*

---

## 4. Fairness analysis — identical GPS error across A and B (critical)

**A vs B is only valid if the GPS error is the same in both runs** (UWB on/off the only difference).

**Finding (sim investigation):** the simulated GPS error is **random and not reproducible run-to-run**:
- Two random sources: navsat SDF Gauss-Markov bias (gz RNG) **and** `gz_bridge::addGpsNoise()` (a
  Markov process driven by C `rand()`, `GZBridge.cpp:548-565`).
- `srand(1234)` is called (`simulator_sih`, `sensor_baro_sim`) — fixed seed — **but** `rand()` is a
  global generator consumed by multiple work-queue **threads**, so the call interleaving (and thus
  the GPS noise sequence) is **not deterministic** between runs.

→ A single A-vs-B run with the built-in noise is **confounded**. Two mechanisms fix this:

| Mechanism | How | Use |
|---|---|---|
| **Deterministic bias** (recommended for the headline) | Add a param-controlled, time-deterministic GPS bias in `gz_bridge` (e.g. linear ramp to N m over the mission); set the random noise ~0. A & B then see `GPS = truth + bias(t)` **identically**. | clean, repeatable graphs; isolates UWB |
| **N-run CEP** (statistical rigor, paper-style) | Run each scenario N times; compare **distributions** (CEP/RMSE over N touchdowns). Randomness handled by statistics, no determinism needed. | matches [[1]](#ref1) metric |

**Plan: use both** — deterministic bias for the clear single-run comparison + plots, and N-run CEP
for the statistical claim. (Note: even with a deterministic GPS bias, the *flight path* differs
slightly between A and B because UWB changes the estimate→control; that is the intended effect. The
controlled, identical input is the **GPS bias(t)**.)

---

## 5. Test scenarios & mission profile

**Common mission (scripted, identical):**
1. Arm + takeoff from pad (origin = anchor-frame origin).
2. Fly a **multi-lap** pattern outside UWB range (e.g. a 40–60 m box, 3–4 laps, ~3–5 min) so the GPS
   bias accumulates while UWB is out of range (>14 m).
3. Return toward the pad → enter UWB range (≤14 m).
4. **AUTO.LAND** on the pad.
5. Record touchdown position (ground truth) vs pad center (0,0).

| Scenario | Config | Expected landing error |
|---|---|---|
| **A — GPS-only** | `EKF2_UWB_CTRL=0` | ≈ GPS bias at landing (e.g. ~1–2 m) |
| **B-(i) — GPS+UWB passive** | `EKF2_UWB_CTRL=1`, tuned noise/gate | partial correction (slow-drift caveat) |
| **B-(ii) — GPS+UWB + GPS down-weight** | `EKF2_UWB_CTRL=1` + adaptive-R/GPS-stop near pad | cm–dm (UWB dominates) |

---

## 6. Metrics & plots (read the system from graphs)

Computed from the `.ulg` (estimate) + `*_groundtruth` (truth):

1. **Landing error** = ‖(x,y)_groundtruth − pad(0,0)‖ at touchdown. Per scenario; CEP & RMSE over N runs.
2. **Horizontal position error vs time** = ‖(x,y)_est − (x,y)_gt‖ over the whole flight — shows GPS
   drift accumulating (A), and the UWB pull-in near the pad (B).
3. **XY top-down trajectory**: groundtruth path, EKF-estimated path, pad, anchors, touchdown points
   (A vs B overlaid).
4. **GPS bias(t)**: injected bias vs the EKF horizontal error — confirms the controlled input.
5. **`estimator_aid_src_uwb` innovation/test_ratio** + `cs_uwb`/`cs_gnss_pos` flags vs time —
   shows when UWB engages and (B-ii) when GPS is down-weighted.

Plots: 1 figure per metric, A/B overlaid; a summary table (CEP_A, CEP_B, RMSE_A, RMSE_B, % improvement).

---

## 7. Tooling to build (all scripted)

| Tool | Purpose |
|---|---|
| **`scripts/uwb_landing/mission.py`** (MAVSDK-Python) | connect, set params (scenario A/B), upload multi-lap mission + LAND, arm, run, detect touchdown, save run metadata (scenario, params, log name). Loop N times for CEP. |
| **Deterministic GPS bias** | small `gz_bridge` addition: param `SIM_GPS_BIAS_*` (constant or time-ramp) added in `navSatCallback`/`addGpsNoise`; random noise scaled by a param so it can be ~0. Identical in A & B. |
| **`scripts/uwb_landing/analyze.py`** | parse `.ulg` (pyulog): extract `vehicle_local_position`, `*_groundtruth`, `estimator_aid_src_uwb`, `estimator_status_flags`; compute landing error, CEP, RMSE; render the §6 plots (matplotlib). |
| **`scripts/uwb_landing/run_all.sh`** | orchestrate: for scenario in {A, B-i, B-ii} × N runs → launch SITL headless, run mission.py, collect logs → analyze.py → report. |

MAVSDK chosen (user preference) for full scripting; failure/param injection via MAVSDK param &
(optionally) the PX4 `failure` plugin.

---

## 8. Implementation plan (proposed order, task-by-task)

1. **Deterministic GPS bias in gz_bridge** (param-controlled) + verify reproducibility (same bias two runs).
2. **MAVSDK mission script** (multi-lap + LAND) + touchdown detection + run metadata.
3. **ulog analysis + plot script** (metrics + figures).
4. **Run Scenario A** (GPS-only) → baseline landing error + plots.
5. **Run Scenario B-(i)** (UWB passive) → compare; quantify partial correction.
6. **GPS down-weight feature** (B-ii): EKF adaptive-R / stop GNSS-pos when UWB trusted (new param, e.g. `EKF2_UWB_GPS`), then run + compare.
7. **N-run CEP** sweep for A / B-i / B-ii → final comparison table + report.

Each step: build/script → run in SITL → user reviews plots → commit (one task per commit).

---

## 9. References

<a id="ref1"></a>**[1]** Ochoa-de-Eribe-Landaberea, A. et al. *UWB and IMU-Based UAV Precision Landing
System.* Sensors 2022, 22(6):2347. https://www.mdpi.com/1424-8220/22/6/2347 — tightly-coupled EKF,
landing RMSE 0.208 m, CEP/RMSE vs pad, mocap ground truth.

<a id="ref2"></a>**[2]** Osaka & Tsujii. *GNSS/UWB/INS fusion for eVTOL navigation* (adaptive R via
DOP/UERE). Sensors 2025, 25(24):7419. https://pmc.ncbi.nlm.nih.gov/articles/PMC12736905/ — UWB R
fixed 0.1 m, GNSS R inflated; 0.56 m H RMSE, 89.4 % improvement over 5.26 m GNSS/INS-only.

<a id="ref3"></a>**[3]** West Virginia University, *Tightly-Coupled GPS/UWB for Formation Flight*,
ION GNSS+ 2014. https://navigationlab.wvu.edu/ — multipath as Gauss-Markov σ=1.6 m, τ=2 min, distinct
from white pseudorange noise.

**[4]** Zhao et al. *Indoor UAV Localization, ESKF tightly- vs loosely-coupled.* Sensors 2025,
25(24):7673. https://pmc.ncbi.nlm.nih.gov/articles/PMC12737027/ — tightly-coupled raw-range ~30.7 %
better RMSE than loosely-coupled.

**[5]** Song & Hsu. *Tightly-coupled INS/UWB.* Aerospace Science & Technology.
https://www.sciencedirect.com/science/article/abs/pii/S127096382031052X — uses raw ranges, not
computed positions.

---

*Builds on the UWB→EKF2 fusion (v1/v2). Sim facts verified against the tree (navsat SDF noise,
`gz_bridge::addGpsNoise`, `srand(1234)`, groundtruth topics, AUTO.LAND). Methodology from a verified
multi-source research pass (peer-reviewed prioritized; landing-specific evidence is thin — see §2 caveats).*
