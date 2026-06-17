# UWB → EKF2 — Deep Audit, Fix & Verification (GPS+UWB)

> **Scope.** The Gazebo plugin / model / world (UWB ranging generation) is treated as correct. This
> doc audits the **PX4 side only** — uORB ingestion → EKF delay buffer → tightly-coupled fusion →
> GPS↔UWB switching — for the requirement:
>
> **Outdoor GPS+UWB: fly on GPS out of range; inside UWB range near the pad UWB dominates and corrects
> GPS drift for precision landing; GPS-only outside the zone.** (Indoor UWB-only = separate, later.)
>
> Findings (B1–B6) came from **this session's audit** — two independent review passes, then
> re-verified line-by-line against the source and against SITL logs. Grounded in
> [`07_gps_uwb_theory_grounding.md`](07_gps_uwb_theory_grounding.md).

## Status at a glance

| ID | What | Severity | Status |
|----|------|----------|--------|
| **B1** | Multi-anchor samples dropped in the shared EKF delay buffer | 🔴 real bug | ✅ **FIXED + SITL-verified** |
| **B2** | Buffer depth ignored the multi-anchor multiplier | 🟠 | ✅ **Resolved by B1** (per-anchor buffers are single-source again) |
| **B3** | Hard position reset on GPS→UWB re-entry | 🟠 | ✅ **Moot after B1** — re-entry verified smooth, reset never fires |
| **B4** | GPS-height ×100 de-weight vs weak vertical observability | 🟡 conditional | ⏳ Future hardening — *only* bites with near-coplanar anchors (ours aren't) |
| **B5** | Shared aiding-timeout can mask a GPS dropout | 🟡 conditional | ⏳ Future hardening — *only* bites if UWB locks a bad fix |
| **B6** | Scale anchor count (max 6, default 4) | 🔵 capacity | ⏳ Future, requested; independent of B1 |
| N1, N2 | "GPS velocity not inflated", "double aiding" | 🟢 not bugs | Leave as-is (see below) |
| M1–M3 | Timestamp delay / hard-coded ×100 / silent skips | ⚪ polish | Optional |

**One-line summary:** B1 was the real bug; fixing it also dissolved B2 and B3. B4/B5 are precautionary
hardening that **did not trigger** in testing. GPS+UWB precision landing is **done and verified**
(3-seed A/B: 3 cm vs 1.5 m). B6 (scale) and B4/B5 (field hardening) are the only open GPS+UWB items.

---

## 1. Findings

### 🔴 B1 — Multi-anchor samples silently dropped in the EKF delay buffer — ✅ FIXED
**`RingBuffer.h:120-142` + `estimator_interface.cpp:441-470` + `uwb_range_control.cpp:184`**

The ECL `RingBuffer` is a **single-source FIFO**. `pop_first_older_than()` pops the matching sample
then does `_tail = (index + 1) % _size` (or empties the buffer if it was the head), **discarding every
sample older than the one it popped** (RingBuffer.h:130-139):

```cpp
// Now we can set the tail to the item which comes after the one we removed
// since we don't want to have any older data in the buffer
if (index == _head) { _tail = _head; _first_write = true; }
else                { _tail = (index + 1) % _size; }
```

Correct for one sensor (drop stale duplicates). But `setUwbData()` pushed **every anchor into the same
buffer**, and `SensorUwb.msg` ships **all anchors in one cycle with the same timestamp**
(`ORB_QUEUE_LENGTH = 8`). So the fusion loop popped the newest anchor and the tail jumped past **the
other anchors pushed earlier in that cycle — they were dropped, never fused.**

**Effect:** ~1 anchor fused per cycle (the last-pushed); a single range is weakly observable → estimate
wanders, innovations stay large, re-acquisition gate trips — *exactly the "problematic ever since
EKF2" symptom*, and the fusion-layer cause of the historical "~1 anchor" observation
([[gz-uwb-plugin-one-anchor]]), **separate from** the already-applied uORB queue-depth fix.

**Fix:** one delay buffer **per anchor** (`_uwb_buffer[4]`); `setUwbData()` routes each anchor to its
own monotonic buffer; `controlUwbRangeFusion()` iterates anchors and fuses each one's freshest due
range (≤ n_anch fusions/cycle). Scalar `fuseUwbRange()` unchanged (correct, A1-grounded).

**Empirical proof (log 11_39_15.ulg):** 100 % of measurement bursts carry all 4 anchors at the *same*
timestamp → the old single buffer dropped 3/4 every burst. After the fix all 4 fuse independently:
anchor0 979/982, anchor1 970/982, anchor2 980/982, anchor3 962/982 (98–100 %); |innov| 5–8 cm.

### 🟠 B2 — Buffer depth ignored the multi-anchor multiplier — ✅ RESOLVED BY B1
**`estimator_interface.cpp:582-586`** — `_obs_buffer_length = round(delay_max_ms*1.5 / update_period)`
was sized for **one** sample/cycle. With per-anchor buffers each buffer is single-source again, so the
original formula is correct per buffer. No separate fix needed.

### 🟠 B3 — Hard position reset on GPS→UWB re-entry — ✅ MOOT AFTER B1
**`uwb_range_control.cpp:214-224`** — when ranges return and innovations were gated out (the old
~73 % reject storm), the code snaps position via `tryInitUwb()` trilateration — a meters-level jump in
the worst phase (descent). **But that storm was a *symptom of B1*** (one weak range → loose estimate →
gating). With all anchors fusing, re-entry re-locks through ordinary gated fusion.

**Empirical proof (log 11_46_56.ulg, fly-out-35 m-and-return):** re-entry reject rate ≈ 2 % (not
73 %); the 2 `reset_count_pos_ne` increments were at **t+0.1 s / t+0.5 s (boot init)**, and **re-entry
(t+96.2 s) produced NO reset** — the hard-reset never fired. The theory upgrade once proposed for this
(R-taper + gate-widen, A3/A4; [`07`](07_gps_uwb_theory_grounding.md) §4) is therefore **not needed** for
GPS+UWB. The hard-reset code stays only as an inert last-resort cold-start fallback.

### 🟡 B4 — GPS-height ×100 de-weight vs weak vertical observability — ⏳ CONDITIONAL FUTURE
**`gnss_height_control.cpp:74-76`** inflates GPS height R ×100 when UWB is active, but `tryInitUwb()`
cold-starts **2-D only** (`uwb_range_control.cpp:85-87`). With **near-coplanar** anchors UWB's vertical
constraint is weak, so height could drift onto baro alone. **Does not bite our setup** — the airframe
anchors are non-coplanar (D = -0.5 / -3.0 / -0.5 / -3.0), so vertical is observable and landings are
cm-level. **Fix when needed:** gate height inflation on mean |LOS_z|, else inflate ≤ ×10 until vertical
converges. Matters only for field setups with flat anchor layouts.

### 🟡 B5 — Shared aiding-timeout can mask a GPS dropout — ⏳ CONDITIONAL FUTURE
**`uwb_range_control.cpp:79,147` + `ekf_helper.cpp:813`** — UWB refreshes the same
`_time_last_hor_pos_fuse` used for GPS-timeout / dead-reckoning. If UWB ever locks a *bad* few-anchor
fix, the aiding-timeout never fires and inflated GPS can't pull it back. **Did not trigger** (good
geometry, 4 anchors). **Fix when needed:** require a short run of accepted UWB innovations before
crediting it as horizontal aiding; expose a `uwb_healthy` flag.

### 🔵 B6 — Scale anchor count: support up to 6, default 4 — ⏳ FUTURE (requested)
The whole UWB stack is hardwired to **4** anchors (pre-existing, not from the B1 fix): `_aid_src_uwb[4]`,
`_uwb_latest[4]` (`ekf.h`); `uwb_anchor_{n,e,d}[4]`, `uwb_n_anchors` (`common.h`); `getUwbAnchorPos()`
`switch` 0–3; params `EKF2_UWB_A0_* … A3_*`; `EKF2_UWB_N_ANCH` max 4; `_estimator_aid_src_uwb_pub[4]`.
**Task:** raise the cap to **6** — arrays `[6]`, params `A4_*`/`A5_*`, `N_ANCH` max 6, `getUwbAnchorPos`
→ array index, one shared `kMaxUwbAnchors` constant — **default `EKF2_UWB_N_ANCH` stays 4**. Pure
capacity refactor, independent of B1.

### 🟢 N1 / N2 — Not bugs (agent overstatements, corrected)
- **N1** "GPS velocity not R-inflated": verified, **benign** — GPS velocity is Doppler-derived and
  bias-free, so it doesn't carry the position drift UWB corrects; keeping it helps. *(Documented so
  it isn't "fixed" by mistake.)*
- **N2** "GPS+UWB fuse in the same cycle": **correct** — sequential R-weighted updates are the standard
  reduction of the papers' stacked measurement vector ([`07`](07_gps_uwb_theory_grounding.md) §2).
  Priority is the R ratio, not mutual exclusion.

### ⚪ M1–M3 — Polish (optional)
- **M1** `SensorUwb.msg` has only `timestamp`; fusion uses publish time + fixed `EKF2_UWB_DELAY=50 ms`
  (`EKF2.cpp:2526`, `estimator_interface.cpp:458`). Runs show |innov| 5–8 cm → 50 ms is about right;
  validate against logged innovation-vs-delay if tuning.
- **M2** Hard-coded ×100 inflation (`gps_control.cpp:277`, `gnss_height_control.cpp:75`) → make a param.
- **M3** Silent `continue` on `tryInitUwb()` failure / anchor-id mismatch → add a throttled log.

---

## 2. What was implemented & verified

**Code (this session):**
- **B1 fix** — `estimator_interface.h/.cpp` (`_uwb_buffer[4]` + per-anchor routing + destructor),
  `uwb_range_control.cpp` (per-anchor fusion loop).
- **Observability** — `estimator_aid_src_uwb` now published as **4 ORB instances** (one per anchor) in
  `EKF2.hpp/.cpp`, so per-anchor fusion is logged & plottable. (The first-attempt `ECL_INFO` counters
  were removed — all `ECL_*` collapse to the suppressed `PX4_DEBUG`, and the old single muxed publisher
  hid the per-anchor split.)

**SITL verification (2026-06-17, gz `f450-uwb`, A=GPS-only vs B=GPS+UWB):**
- **Per-anchor fusion:** all 4 anchors 98–100 % across the full mission (incl. the 35 m far leg).
- **3-seed A/B landing (true error vs ground-truth pad):** A = 1.16 / 1.69 / 1.54 m (CEP50 1.54),
  **B = 0.04 / 0.03 / 0.02 m (CEP50 0.03) → 98 % better.** B std ~1 cm; A tracks the per-seed GPS bias
  (1.2–1.67 m) → GPS drift is the error source, UWB nullifies it regardless of magnitude/direction.
- **Clean hand-off:** UWB disables when leaving range, re-enables on return (1/1, no flapping); re-entry
  smooth (≈2 % reject, no position reset — see B3).

➡ **GPS+UWB precision landing meets the requirement and is verified.**

---

## 3. Traceability — requirement ↔ finding ↔ paper

| Requirement | Mechanism / finding | Paper grounding |
|---|---|---|
| UWB usable near pad (all anchors fuse) | **B1** per-anchor buffers ✅ | A1: each range an independent constraint; tight > loose |
| UWB dominates near pad | existing GPS R-inflation (kept); **M2** to make tunable | A2: lower R ⇒ higher Kalman weight |
| Smooth GPS→UWB hand-off | falls out of **B1** (no reset needed) ✅ | A3/A4 (R-taper) — proposed but verified unnecessary |
| Reliable vertical for touchdown | OK with non-coplanar anchors; **B4** if flat layout | A1: vertical gains smallest |
| Don't mask a GPS dropout | **B5** (field hardening) | — (safety) |
| GPS-only outside zone | already correct (inflation gated on `flags.uwb`) ✅ | A1: keep GPS authoritative outside |
| More anchors | **B6** scale to 6 | — |

---

## 4. Remaining GPS+UWB work (none blocking)

1. **B6** — scale to 6 anchors (default 4). Capacity refactor.
2. **B4 / B5** — field-hardening for coplanar-anchor / bad-lock edge cases; implement when deploying on
   real hardware with imperfect geometry. Not triggered in sim.
3. **M1–M3** — polish (param-ize ×100, validate delay, add throttled logs).

Separate tracks: **indoor UWB-only** (later), and the cleanup decision on whether the 4-instance
aid_src logging + the `estimator_aid_src_gnss_pos/vel` debug topics stay permanent or move behind a
debug flag before merge.
