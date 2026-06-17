# UWB Precision Landing in GPS-Degraded Zones — Methodology & Implementation

> **Application:** use UWB (anchors surveyed around the landing pad) to land a multirotor
> **accurately** when GPS has drifted during the mission. En route the drone flies on GPS; on return
> it enters UWB range (≤ ~14 m of the pad) and UWB corrects the accumulated GPS error so it touches
> down on the pad. Built on the UWB→EKF2 fusion in [`02_tightly_coupled_px4.md`](02_tightly_coupled_px4.md)
> / [`04_v2_native_plan.md`](04_v2_native_plan.md).
>
> **Scope:** two scenarios only — **A = GPS-only** (baseline) and **B = GPS+UWB** (UWB-dominant near
> the pad). Landing via an AUTO mission that descends to a low approach waypoint over the pad (UWB
> range) then **AUTO.LAND**s. Everything scripted (pymavlink) + analyzed with plots/metrics. This
> file is both the **methodology/scenario** and the **task-by-task implementation plan**.
>
> **Workflow rule:** each task → build/script → run in SITL → **reviewer approves plots/metrics** →
> commit (one task per commit; no committing un-reviewed work).

---

## 1. The demonstration (narrative)

- **Scenario A — GPS-only (baseline):** takeoff from pad → fly a multi-lap mission far from the pad
  for several minutes (GPS error accumulates) → return → LAND. The EKF believes it is over the pad,
  but it is offset by the GPS error → **lands off-pad** by ~the GPS drift (~1-2 m for M9N).
- **Scenario B — GPS+UWB (UWB-dominant):** same mission, same GPS error; on return the drone enters
  UWB range, UWB re-locks the position to the anchor frame and is trusted over the drifted GPS, so the
  estimate snaps to the pad → **lands on the pad**.

Value proposition: **UWB precision landing in GPS-weakened areas.** SITL result (laps=1): A lands
**1.07 m** off the pad (EKF blind — thinks it is on the pad, ~0.9 m bias); B lands **0.16 m** off
(EKF accurate, ~0.06 m bias). Same mission/approach for both — the only difference is the position
estimate.

> **Update (2026-06-17, post-B1 fix):** the multi-anchor drop bug (B1) found in the later audit was
> fixed (per-anchor buffers); with all 4 anchors fusing, a 3-seed A/B re-run gives **B CEP50 = 0.03 m
> vs A 1.54 m (true error vs ground-truth pad)** — the 0.16 m figure above predates that fix. See
> [`08_fix_plan.md`](08_fix_plan.md).

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
profile is **not cleanly demonstrated in any single paper** → our active GPS down-weight (scenario B)
is a defensible synthesis. **Innovation chi-squared gating is weak against slow GPS bias
drift** (good at noise spikes) → the gate alone won't fully correct accumulated drift; we need
adaptive R / GPS down-weighting.

---

## 3. Scenario B design — making UWB win near the pad (`EKF2_UWB_GPS`)

Three coupled problems had to be solved for B to actually land on the pad; each was found by reading
the SITL ulogs (see commit history). All are gated on UWB being active and well-constrained
(`control_status.flags.uwb` **and ≥3 anchors with a range < 0.5 s old**), and enabled by
`EKF2_UWB_GPS=1`:

1. **Fusion conflict → adaptive R.** On return GPS (drifted ~1-2 m) and UWB (pad-accurate) disagree,
   but the bias grew **slowly** so the chi-squared gate doesn't reject GPS → the estimate sits
   *between* them. Fix: **inflate GNSS R ×100** (horizontal **and** vertical) so the Kalman gain
   `K = P Hᵀ(HPHᵀ+R)⁻¹` trusts GPS far less and UWB dominates (adaptive-R, [[2]](#ref2)).
2. **Re-acquisition after drift.** Flying the box takes the drone out of UWB range; the EKF drifts on
   GPS, so when ranges return the UWB innovations are large (~1.9 m) and **73 % get gated out**
   (chicken-and-egg). Fix: when ≥3 anchors are ranging but UWB keeps being rejected, **snap the
   horizontal position to the UWB trilateration solution** (a position reset, like GPS
   reset-on-large-innovation) so UWB re-locks; adaptive-R then holds it.
3. **Z is mandatory and must be observable.** A range `r = √(rh² + Δz²)` couples horizontal and
   vertical — you cannot get XY from a range without the vertical separation Δz, and **coplanar
   anchors give no vertical observability** (vertical error amplified, height aiding required
   [[1]](#ref1)-class result). Fix: **anchors are non-coplanar** (staggered heights 0.5/3.0 m) so the
   full **3-D range fusion observes N/E/D** and UWB owns altitude too — no GPS/baro height crutch.
   Sensitivity `∂rh/∂Δz = −Δz/rh` is huge when high above the pad → UWB horizontal is only reliable
   when **low** (near the anchor plane), which is exactly the final-approach phase.

Out of the UWB zone (`flags.uwb` cleared on timeout) none of this applies → GPS keeps full weight.

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

**Common mission (scripted, identical for A and B):** arm + takeoff from the pad (= take-off point =
EKF local origin) → multi-lap **~50 m box** outside UWB range (>14 m) so GPS error accumulates
uncorrected → return → descend to a **low approach waypoint (~6 m) over the pad** (inside UWB range;
tight `NAV_ACC_RAD` so the drone centres on the pad) → **AUTO.LAND**. Land target = the EKF origin
(maps to local (0,0) = pad regardless of GPS bias). Record touchdown (ground truth) vs pad (0,0).

| Scenario | Config | Result (laps=1) |
|---|---|---|
| **A — GPS-only** | `EKF2_UWB_CTRL=0` | **1.07 m** off (EKF blind, ~0.9 m bias) |
| **B — GPS+UWB** | `EKF2_UWB_CTRL=1`, `EKF2_UWB_GPS=1` | **0.16 m** off (EKF accurate, ~0.06 m bias) |

---

## 6. Metrics & plots (read the system from graphs)

From the `.ulg` (estimate) + `*_groundtruth` (truth):
1. **Landing error** = ‖(x,y)_gt − pad(0,0)‖ at touchdown; CEP & RMSE over N runs.
2. **Horizontal position error vs time** = ‖(x,y)_est − (x,y)_gt‖ — drift accumulation (A), UWB pull-in (B).
3. **XY top-down trajectory:** truth path, EKF path, pad, anchors, touchdown points (A vs B overlaid).
4. **`estimator_aid_src_uwb`** innovation/test_ratio + `cs_uwb`/`cs_gnss_pos` flags vs time.
5. **Touchdown scatter + CEP circles** (A vs B) over N runs.

Summary table: CEP_A/CEP_B, RMSE_A/RMSE_B, % improvement (over N seeds, Task 7).

---

## 7. Implementation plan (task-by-task)

**Tooling:** `Tools/uwb_landing/` (pymavlink + pyulog + matplotlib).

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

### Task 2 — mission runner (`Tools/uwb_landing/mission.py`)
**Tooling = pymavlink** (`mavutil`), not MAVSDK: pymavlink is PX4's own pure-Python tool stack
(`Tools/mavlink_shell.py`, `mavlink_ulog_streaming.py`), already installed, no ROS / no server
binary. (PX4's MAVSDK tests are C++ in `test/mavsdk_tests/`; MAVROS needs ROS.)
- [x] One run = one already-booted SITL: connect to the **autopilot** heartbeat on
  `udpin:0.0.0.0:14540` (filter sys≠0/comp=AUTOPILOT1); set scenario params (`EKF2_GPS_CTRL=7`;
  A:`EKF2_UWB_CTRL=0` / B:`UWB_CTRL=1,UWB_GPS=1`); upload AUTO mission (TAKEOFF → ~50 m box `--laps`
  kept >14 m from pad → **low approach waypoint over the pad** at `--approach-alt` → `NAV_LAND` at the
  pad). Land/pad coordinate = **EKF global origin** (`GPS_GLOBAL_ORIGIN`) which maps to local (0,0) =
  take-off point regardless of GPS bias (NOT RTL/home, which targets the GPS-biased home and lands a
  full bias off). arm → `MISSION_START` → wait `landed_state=ON_GROUND`; record EKF landing x,y;
  disarm; append CSV. Robustness learned in SITL: **INT params are bit-cast into the MAVLink float
  field** by PX4 (encode/decode with struct or they corrupt, e.g. 7→1088421888); **read-before-set**
  (a redundant param_set resets EKF GPS aiding ~6 s); `wait_position_stable()` before arming; tight
  `NAV_ACC_RAD` so the drone centres before the final descent; log flight-mode transitions.
  > **Seed note:** `SIM_GPS_SEED` is consumed at sim boot, so per-seed runs are driven by a launcher
  > that reboots SITL per seed (Task 7 `run_all.sh`). True landing error (vs pad ground-truth) is
  > computed by `analyze.py` from the ulog (ground-truth isn't on the MAVLink link).
- [x] **USER verified** in SITL: A lands 1.07 m off (EKF blind), B lands 0.16 m off (EKF accurate).

### Task 3 — Log analysis + plots (`Tools/uwb_landing/analyze.py`)
- [x] pyulog: per ulog detect touchdown (last arm→disarm), TRUE landing error = ground-truth landing
  vs pad (ground-truth origin); scenario auto-labelled from `EKF2_UWB_CTRL`/`EKF2_UWB_GPS` active at
  flight time. Per-scenario **mean / CEP50 (median radius) / RMSE / worst**; 3 plots (touchdown
  scatter + CEP circles, EKF−truth error vs time, trajectory truth vs EKF) plus detail charts (3-D
  trajectory, X/Y/Z, roll/pitch/yaw, Vx/Vy/Vz). Shows an interactive window by default (`--out` to
  save, `--no-show` headless). Verified on real ulogs: A 1.07 m, B 0.16 m (match hand calc).
- [x] **USER verified.**

### Task 6 — Scenario B feature: `EKF2_UWB_GPS` (UWB-dominant near the pad)
**Files:** `aid_sources/uwb/uwb_range_control.cpp`, `aid_sources/gnss/gps_control.cpp`,
`aid_sources/gnss/gnss_height_control.cpp`, `ekf.h`, `common.h`, `EKF2.{hpp,cpp}`, `params_uwb.yaml`,
`ROMFS/.../4023_gz_f450-uwb`, gz submodule `worlds/uwb.sdf`.
- [x] **Non-coplanar anchors** (worlds/uwb.sdf staggered 0.5/3.0 m + matching `EKF2_UWB_A*` forced in
  the airframe) → UWB observes N/E/D → **full 3-D range fusion** (`fuseUwbRange`).
- [x] param **`EKF2_UWB_GPS`** (0=normal, 1=UWB-dominant). When set AND `flags.uwb` AND
  `countRecentUwbAnchors() >= 3`: inflate GNSS **horizontal** (`gps_control`) and **vertical**
  (`gnss_height_control`) R ×100 so UWB wins.
- [x] **Re-acquisition reset**: after ≥5 consecutive gated UWB ranges with ≥3 anchors, snap the
  position to the UWB trilateration solution (`tryInitUwb`) so UWB re-locks after the box drift.
- [x] **USER verified**: B estimate snaps to the pad on return, lands 0.16 m off (EKF bias 0.06 m).

### Task 7 — N-seed CEP sweep + final report
- [x] `run_all.sh [N] [base_seed] [laps]`: for each seed × scenario {A,B}, two headless boots — a
  **config boot** (set `SIM_GPS_SEED` + `EKF2_UWB_CTRL`/`EKF2_UWB_GPS`, `param save`) then a
  **measure boot** (run `mission.py`) — so A & B see the **same** GPS bias per seed (fair). Collects
  the ulogs and runs `analyze.py --compare`.
- [x] `analyze.py --compare`: the standard landing-accuracy charts — **(1)** touchdown scatter with
  **CEP50 (solid) / R95 (dashed)** circles, **(2)** horizontal-error **CDF**, **(3)** metric bars
  (mean / CEP50 / R95 / RMSE) — all A (red) vs B (green), n shown; plus a printed table with the
  **CEP50 improvement %**. Interactive window by default (user saves if wanted).
- [ ] **USER review** the sweep result; record final CEP/RMSE table here.

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
