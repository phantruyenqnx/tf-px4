# UWB Precision Landing in GPS-Degraded Zones — Methodology & Implementation

> **Application:** use UWB (anchors surveyed around the landing pad) to land a multirotor
> **accurately** when GPS has drifted during the mission. En route the drone flies on GPS; on return
> it enters UWB range (≤ ~14 m of the pad) and UWB corrects the accumulated GPS error so it touches
> down on the pad. Built on the UWB→EKF2 fusion in [`02_tightly_coupled_px4.md`](02_tightly_coupled_px4.md)
> / [`04_v2_native_plan.md`](04_v2_native_plan.md).
>
> **Scope:** Mode **UWB+GPS** only. Landing via standard **AUTO.LAND**. Everything scripted (MAVSDK)
> + analyzed with plots/metrics. This file is both the **methodology/scenario** and the
> **task-by-task implementation plan**.
>
> **Workflow rule:** each task → build/script → run in SITL → **reviewer approves plots/metrics** →
> commit (one task per commit; no committing un-reviewed work).

---

## 1. The demonstration (narrative)

- **Scenario A — GPS-only (baseline):** takeoff from pad → fly a multi-lap mission far from the pad
  for several minutes (GPS error accumulates) → return → LAND. The EKF believes it is over the pad,
  but it is offset by the GPS error → **lands off-pad** by ~the GPS drift (~1-2 m for M9N).
- **Scenario B — GPS+UWB:** same mission, same GPS error; near the pad UWB injects pad-relative
  position → corrects the drift → **lands on the pad**.

Value proposition: **UWB precision landing in GPS-weakened areas.**

---

## 2. What the literature establishes (verified, cited)

- **Architecture = tightly-coupled raw-range fusion** (what we built, v1/v2). Closest landing study:
  **Ochoa-de-Eribe-Landaberea et al., *Sensors* 2022, 22(6):2347** — tightly-coupled EKF (8 anchors
  around a 2×2 m pad, dual tags + IMU), horizontal landing **RMSE 0.208 m vs 0.410 m** UWB-only
  (~50%), 99.95 % of errors < 1 m, mocap ground truth. [[1]](#ref1)
- **GPS degradation modeling:** a **slowly-varying first-order Gauss-Markov bias** (e.g. σ≈1.6 m,
  τ≈2 min) kept **separate** from white pseudorange noise — WVU ION GNSS+ 2014 [[3]](#ref3). Real
  GNSS/INS-only multipath error reached ~5.26 m.
- **UWB "wins" near the pad via adaptive measurement noise:** UWB R fixed small (~0.1 m) while
  degraded-GNSS R is inflated 80–150× (DOP/UERE) → Kalman gain trusts UWB. eVTOL result: 0.56 m H
  RMSE, **89.4 % improvement** over 5.26 m GNSS/INS-only — Osaka & Tsujii, *Sensors* 2025,
  25(24):7419 [[2]](#ref2).
- **Metrics:** **CEP** (radius of the circle about the aim point containing 50 % of touchdowns,
  ≈ 1.1774·σ) + **3-D RMSE** vs the surveyed pad center, against mocap/sim ground truth. [[1]](#ref1)

**Honest caveats:** dedicated UWB precision-**landing** experiments are scarce ([[1]](#ref1) is the
only true touchdown-error study; others inform architecture/degradation). The loiter-then-handoff
profile is **not cleanly demonstrated in any single paper** → our active GPS down-weight (scenario
B-ii) is a defensible synthesis. **Innovation chi-squared gating is weak against slow GPS bias
drift** (good at noise spikes) → the gate alone won't fully correct accumulated drift; we need
adaptive R / GPS down-weighting.

---

## 3. The fusion-conflict design (why scenario B-ii matters)

On return with ~1-2 m of accumulated GPS error and pad-accurate UWB: GPS says "over the pad", UWB
says "off by 1-2 m". Because the bias built up **slowly**, neither innovation is a sharp spike, so
the chi-squared gate does **not** reject GPS → the estimate sits **between** GPS and UWB → still
lands partly off-pad.

- **B-(i) passive (tuning only):** keep both GPS+UWB, tune `EKF2_UWB_NOISE`/gate. Expect UWB pulls
  the estimate **partway** to the pad; slow drift only partially corrected. *Measures how far tuning
  gets us.*
- **B-(ii) active GPS down-weight:** when UWB is active & trusted, **inflate GNSS position R** so UWB
  dominates near the pad (adaptive-R, [[2]](#ref2)). **R is the GPS-trust knob**:
  `K = P Hᵀ (HPHᵀ + R)⁻¹` — larger R ⇒ smaller gain ⇒ EKF trusts GPS less. Expect estimate snaps to
  pad → cm–dm landing. Default keeps EKF2's normal behavior; `EKF2_UWB_GPS=1` enables the inflation.

---

## 4. Realistic GPS model + fairness (locked decisions)

1. **Realistic GPS = u-blox NEO-M9N, standard precision** (autonomous, no RTK): horizontal **~2 m
   CEP**, velocity ~0.05 m/s ([datasheet](https://learn.sparkfun.com/tutorials/sparkfun-gps-neo-m9n-hookup-guide/all)).
   Default PX4 SITL GPS shows almost no error → we inject realistic M9N error so GPS-only landings
   miss by ~1-2 m.
2. **GPS error lives in ONE place = `gz_bridge`.** The custom navsat noise in `f450_base/model.sdf`
   is **commented out** → f450 navsat = x500 default (bare sensor). All error is produced in
   `gz_bridge::addGpsNoise()`.
3. **Error model = realistic noise (not a fake offset):** small white + a first-order **Gauss-Markov
   bias** whose steady-state σ dominates CEP (real GPS error is slowly correlated). Matches the WVU
   σ≈1.6 m, τ≈2 min model.
4. **Fairness = reproducible noise.** Rewrite the noise to use a **dedicated fixed-seed RNG**
   (`std::mt19937`, seed = `SIM_GPS_SEED`) instead of the global multithreaded `rand()`. Same seed +
   same scripted mission ⇒ **identical M9N noise in A and B** (GPS generated independently of UWB).
   Statistical claims (CEP) use **N runs with different seeds**.

**Why this is fair:** the GPS error trace is identical across A and B (same seed), so the only
difference is UWB on/off. (The flight path still differs slightly because UWB changes the
estimate→control — that is the intended effect.)

**Sim facts (verified):** two random GPS noise sources existed — the f450 navsat SDF Gauss-Markov
bias **and** `gz_bridge::addGpsNoise()` (global `rand()`, `GZBridge.cpp:548`). `srand(1234)` is set
but `rand()` is consumed by multiple work-queue threads → non-reproducible. Hence: comment the SDF
noise, consolidate into a single seeded RNG in the bridge.

---

## 5. Test scenarios & mission

**Common mission (scripted, identical):** arm + takeoff from pad (origin = anchor-frame origin) →
multi-lap **~50 m box** outside UWB range (>14 m) so GPS error accumulates uncorrected, for ~3-5 min
→ return toward pad → **AUTO.LAND** inside UWB range (≤14 m). Record touchdown (ground truth) vs pad
center (0,0).

| Scenario | Config | Expected landing error |
|---|---|---|
| **A — GPS-only** | `EKF2_UWB_CTRL=0` | ≈ GPS error at landing (~1-2 m, M9N) |
| **B-(i) — GPS+UWB passive** | `EKF2_UWB_CTRL=1`, `EKF2_UWB_GPS=0` | partial correction (slow-drift caveat) |
| **B-(ii) — GPS+UWB + GPS R-inflation** | `EKF2_UWB_CTRL=1`, `EKF2_UWB_GPS=1` | cm–dm (UWB dominates) |

---

## 6. Metrics & plots (read the system from graphs)

From the `.ulg` (estimate) + `*_groundtruth` (truth):
1. **Landing error** = ‖(x,y)_gt − pad(0,0)‖ at touchdown; CEP & RMSE over N runs.
2. **Horizontal position error vs time** = ‖(x,y)_est − (x,y)_gt‖ — drift accumulation (A), UWB pull-in (B).
3. **XY top-down trajectory:** truth path, EKF path, pad, anchors, touchdown points (A vs B overlaid).
4. **`estimator_aid_src_uwb`** innovation/test_ratio + `cs_uwb`/`cs_gnss_pos` flags vs time.
5. **Touchdown scatter + CEP circles** (A vs B) over N runs.

Summary table: CEP_A/B1/B2, RMSE_A/B1/B2, % improvement.

---

## 7. Implementation plan (task-by-task)

**Tooling:** `Tools/uwb_landing/` (Python MAVSDK + pyulog), consistent with `Tools/ecl_ekf/`.

### Task 1 — f450 navsat → x500 default + M9N seeded Gauss-Markov noise
**Files:** `Tools/simulation/gz/models/f450_base/model.sdf`; `src/modules/simulation/gz_bridge/GZBridge.{hpp,cpp}`; sim GPS params file.
- [x] **S1:** comment the `<navsat>` noise block of `navsat_sensor` in `f450_base/model.sdf` → bare sensor like x500; leave `navsat_ground_truth`. (Original M9N block preserved in git/this doc.)
- [x] **S2:** dedicated seeded RNG — file-static `std::mt19937 s_gps_rng` + `normal_distribution`, seeded once from `SIM_GPS_SEED`; replaces `generate_wgn()`'s global `rand()`.
- [x] **S3:** M9N error in `addGpsNoise()` = **white + first-order Gauss-Markov dynamic bias**, replicating
  gz-sensors `GaussianNoiseModel` [[4]](#ref4) faithfully (so it is seeded/scalable, which gz's RNG is not):
  `phi_d=expf(-dt/tau); sbd=sqrtf(-sigma_b*sigma_b*tau*0.5f*expm1f(-2*dt/tau)); bias=phi_d*bias+sbd*n01(); out=truth+SC*(bias+sigma_white*n01())+const_bias;`
  Params based on the f450 SDF block being replaced, with a longer M9N-realistic correlation time (NED): `tau=300 s` (minutes-scale, so the on-ground drift rate stays well under `EKF2_REQ_HDRIFT`=0.1 m/s and the EKF reliably accepts GPS — a `tau=150 s` drift sits on that gate and is intermittently rejected, ~94% accept); `sigma_b_h=0.1225` (steady σ=`sigma_b*sqrt(tau/2)`≈1.5 m → H CEP≈1.8 m), `sigma_b_v=0.245` (steady ≈3 m); white H 0.022 / V 0.05 m; vel white 0.05/0.10 m/s. Bias **warm-started at steady-state** so the EKF takes its first reference already biased (no 0→1.5 m ramp). **Why drift (not a frozen offset):** a constant bias is absorbed into the EKF local origin (local pos reads ~0) → land-at-home would miss by 0 m; the *slow drift* (bias at take-off ≠ at landing) is the physical source of the GPS-only landing error.
- [x] **S4:** params `SIM_GPS_SEED` (int=1), `SIM_GPS_NSC` (float=1; 0⇒off), `SIM_GPS_BIAS_N/E` (float=0, optional).
- [x] **S5:** build `make px4_sitl gz_f450-uwb_uwb` (EXIT 0).
- [ ] **S6: USER verify** — (a) realism: EKF `vehicle_local_position` slowly wanders ~1-2 m vs groundtruth, `xy_valid:True`, `dead_reckoning:False`, no persistent "GPS drift too high"; (b) reproducible: same `SIM_GPS_SEED` → same trace; `SIM_GPS_NSC 0` → ~0 error.
- [ ] **S7: commit** (after approval).

### Task 2 — MAVSDK mission runner (`Tools/uwb_landing/mission.py`)
- [ ] MAVSDK-Python: connect, set scenario params + 12 anchors + `SIM_GPS_SEED`/`SC`, upload ~50 m box (`--laps`), arm→start→wait→`action.land()`→wait on-ground, append run CSV. `--runs N` (seed=base+i), `--headless` opt-in.
- [ ] **USER verify** single run; **commit** after approval.

### Task 3 — Log analysis + plots (`Tools/uwb_landing/analyze.py`)
- [ ] pyulog extract est/truth/flags/uwb-aid/sensor_gps; metrics landing_error, RMSE, CEP=1.1774σ; the §6 plots (matplotlib; may reuse `Tools/ecl_ekf/plotting`).
- [ ] **USER verify** on a Task-2 log (A landing_error ~1-2 m); **commit** after approval.

### Task 4 — Scenario A baseline (GPS-only)
- [ ] Run `--scenario A` fixed seed; analyze. **USER review**: landing_error_A ~1-2 m, drift in `pos_err(t)`.

### Task 5 — Scenario B-(i) passive (GPS+UWB)
- [ ] Same seed, `EKF2_UWB_CTRL=1`, `EKF2_UWB_GPS=0`; analyze. **USER review**: partial pull-in.

### Task 6 — `EKF2_UWB_GPS` R-inflation (B-ii)
**Files:** `gps_control.cpp`, `common.h`, `EKF2.{hpp,cpp}`, `params_uwb.yaml`.
- [ ] param `EKF2_UWB_GPS` (0=normal default, 1=inflate GNSS pos R when `cs_uwb`, 2=stop GNSS hpos — optional). Apply ×~100 R scale in GNSS pos fusion when `_control_status.flags.uwb`.
- [ ] **USER verify** estimate snaps to pad on return; **commit** after approval.

### Task 7 — N-run CEP sweep + final report
- [ ] `run_all.sh` (headless opt-in): {A,B1,B2}×N (seed=base+i) → mission → analyze aggregate; final CEP/RMSE table + CEP-circle plot. **USER review**; record results here.

---

## 8. References

<a id="ref1"></a>**[1]** Ochoa-de-Eribe-Landaberea, A. et al. *UWB and IMU-Based UAV Precision
Landing.* Sensors 2022, 22(6):2347. https://www.mdpi.com/1424-8220/22/6/2347 — tightly-coupled EKF,
landing RMSE 0.208 m, CEP/RMSE vs pad, mocap ground truth.

<a id="ref2"></a>**[2]** Osaka & Tsujii. *GNSS/UWB/INS fusion for eVTOL navigation* (adaptive R via
DOP/UERE). Sensors 2025, 25(24):7419. https://pmc.ncbi.nlm.nih.gov/articles/PMC12736905/ — 0.56 m H
RMSE, 89.4 % improvement over 5.26 m GNSS/INS-only.

<a id="ref3"></a>**[3]** West Virginia University, *Tightly-Coupled GPS/UWB for Formation Flight*,
ION GNSS+ 2014. https://navigationlab.wvu.edu/ — multipath as Gauss-Markov σ=1.6 m, τ=2 min.

<a id="ref4"></a>**[4]** Gazebo `gz-sensors`, `GaussianNoiseModel` (navsat white + dynamic bias). The
sensor error = white Gaussian + a first-order Gauss-Markov (Ornstein-Uhlenbeck) dynamic bias:
`sigma_b_d=sqrt(-sigma_b²·tau/2·expm1(-2·dt/tau)); phi_d=exp(-dt/tau); bias=phi_d·bias+N(0,sigma_b_d); out=in+bias+white`.
Steady-state bias σ = `sigma_b·sqrt(tau/2)`. https://github.com/gazebosim/gz-sensors —
`src/GaussianNoiseModel.cc`. Matches the standard GNSS error model (white + 1st-order Gauss-Markov,
correlation `R(Δt)=σ²e^(-|Δt|/τ)`) in the GNSS stochastic-modelling literature.

**[5]** Zhao et al. *Indoor UAV Localization, ESKF tightly- vs loosely-coupled.* Sensors 2025,
25(24):7673. https://pmc.ncbi.nlm.nih.gov/articles/PMC12737027/ — tightly-coupled ~30.7 % better RMSE.

**[M9N]** u-blox NEO-M9N datasheet (standard precision, ~2 m CEP):
https://learn.sparkfun.com/tutorials/sparkfun-gps-neo-m9n-hookup-guide/all

---

*Builds on the UWB→EKF2 fusion (v1/v2, 3 modes implemented). Sim facts verified against the tree
(navsat SDF noise, `gz_bridge::addGpsNoise`, `srand`, groundtruth topics, AUTO.LAND). Methodology
from a verified multi-source research pass (peer-reviewed prioritized; landing-specific evidence is
thin — see §2 caveats).*
